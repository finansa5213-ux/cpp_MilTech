// ============================================================
// ResultUploader.cpp — реалізація HTTP-клієнта здачі результатів.
// ============================================================
#include "ResultUploader.h"

#include "httplib.h"
#include "json.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace dz15 {

using json = nlohmann::json;

namespace {

// Ширина колонок підсумкової таблиці.
constexpr int kColTest = 8;
constexpr int kColStatus = 26;
constexpr int kColAttempts = 8;

// Код 0 у httplib означає «відповіді не отримано» (таймаут/обрив).
constexpr int kNoResponse = 0;

// ------------------------------------------------------------
// Ширина рядка в символах, а не в байтах.
//
// std::setw рахує байти, тому кирилиця (2 байти на символ у UTF-8)
// ламає вирівнювання таблиці: «ТЕСТ» — це 4 символи, але 8 байтів.
// Рахуємо стартові байти UTF-8: усі, крім continuation-байтів 10xxxxxx.
// ------------------------------------------------------------
std::size_t displayWidth(const std::string& s) {
    std::size_t width = 0;
    for (const unsigned char c : s) {
        if ((c & 0xC0) != 0x80) {
            ++width;
        }
    }
    return width;
}

// Друкує текст і доповнює пробілами до потрібної ширини в символах.
void padTo(const std::string& s, std::size_t width) {
    std::cout << s;
    const std::size_t used = displayWidth(s);
    if (used < width) {
        std::cout << std::string(width - used, ' ');
    } else {
        std::cout << ' ';  // мінімальний розділювач, якщо текст задовгий
    }
}

}  // namespace

const char* toString(UploadStatus status) {
    switch (status) {
        case UploadStatus::Success:      return "OK (201 збережено)";
        case UploadStatus::Overwritten:  return "OK (200 перезаписано)";
        case UploadStatus::Rejected:     return "ВІДХИЛЕНО (400/401)";
        case UploadStatus::Exhausted:    return "НЕВДАЧА (спроби вичерпано)";
        case UploadStatus::Missing:      return "ПРОПУЩЕНО (немає файла)";
        case UploadStatus::BadLocalFile: return "ПОМИЛКА (битий JSON)";
        case UploadStatus::NotConfirmed: return "POST ok, GET не підтвердив";
    }
    return "?";
}

bool isSuccessful(UploadStatus status) {
    return status == UploadStatus::Success || status == UploadStatus::Overwritten;
}

ResultUploader::ResultUploader(UploaderConfig config) : cfg_(std::move(config)) {}

std::string ResultUploader::resultPath(const std::string& testId) const {
    return cfg_.resultsDir + "/" + testId + "/simulation.json";
}

// ------------------------------------------------------------
// ТЗ п.3: 2xx — успіх; 400/401 — не повторювати; 503 і таймаут —
// повторювати. Решта кодів трактуємо консервативно як фатальні:
// повторення однакового запиту їх не виправить.
// ------------------------------------------------------------
RetryDecision ResultUploader::classify(int httpStatus) {
    if (httpStatus >= 200 && httpStatus < 300) {
        return RetryDecision::Done;
    }
    if (httpStatus == 400 || httpStatus == 401) {
        return RetryDecision::Stop;
    }
    if (httpStatus == 503 || httpStatus == kNoResponse) {
        return RetryDecision::Again;
    }
    return RetryDecision::Stop;
}

bool ResultUploader::verifyOnServer(const std::string& testId, std::string& detail) {
    httplib::Client client(cfg_.host, cfg_.port);
    client.set_connection_timeout(cfg_.connectTimeoutSec, 0);
    client.set_read_timeout(cfg_.readTimeoutSec, 0);
    client.set_write_timeout(10, 0);   // ← сюди
    
    const std::string path = "/api/dz12/results/" + testId + "/" + cfg_.studentId;

    httplib::Headers headers = {{"x-api-key", cfg_.apiKey}};
    auto res = client.Get(path, headers);

    if (!res) {
        detail = "GET: немає відповіді (таймаут)";
        return false;
    }
    if (res->status == 404) {
        detail = "GET: 404 — на сервері не знайдено";
        return false;
    }
    if (res->status != 200) {
        detail = "GET: несподіваний код " + std::to_string(res->status);
        return false;
    }

    // Тіло має бути {"found":true,...}. Розбираємо акуратно: сервер
    // може віддати валідний 200 з found:false.
    try {
        const json body = json::parse(res->body);
        const bool found = body.value("found", false);
        if (!found) {
            detail = "GET: 200, але found=false";
        }
        return found;
    } catch (const json::exception& e) {
        detail = std::string("GET: тіло не парситься — ") + e.what();
        return false;
    }
}

TestReport ResultUploader::uploadOne(const std::string& testId) {
    TestReport report;
    report.testId = testId;
    

    // --- 1. Читання локального результату ---------------------
    const std::string path = resultPath(testId);
    std::ifstream in(path);
    if (!in.is_open()) {
        // ТЗ: «якщо якогось теста немає — надсилаєте ті, що є».
        report.status = UploadStatus::Missing;
        report.detail = "файл не знайдено: " + path;
        if (cfg_.verbose) {
            std::cout << "[" << testId << "] пропуск — немає " << path << "\n";
        }
        return report;
    }

    json simulation;
    try {
        in >> simulation;
    } catch (const json::exception& e) {
        report.status = UploadStatus::BadLocalFile;
        report.detail = e.what();
        if (cfg_.verbose) {
            std::cout << "[" << testId << "] локальний файл не є валідним JSON\n";
        }
        return report;
    }

    // --- 2. Формування тіла запиту ----------------------------
    json payload;
    payload["studentId"] = cfg_.studentId;
    payload["testId"] = testId;
    payload["simulation"] = simulation;
    const std::string body = payload.dump();

    // --- 3. POST з обмеженою кількістю спроб ------------------
    httplib::Client client(cfg_.host, cfg_.port);
    client.set_connection_timeout(cfg_.connectTimeoutSec, 0);  // ТЗ п.2
    client.set_read_timeout(cfg_.readTimeoutSec, 0);

    const httplib::Headers headers = {{"x-api-key", cfg_.apiKey}};

    for (int attempt = 1; attempt <= cfg_.maxAttempts; ++attempt) {
        report.attempts = attempt;

        auto res = client.Post("/api/dz12/results", headers, body, "application/json");
        const int code = res ? res->status : kNoResponse;
        report.lastHttpCode = code;

        if (cfg_.verbose) {
            std::cout << "[" << testId << "] спроба " << attempt << "/" << cfg_.maxAttempts
                      << " → " << (code == kNoResponse ? std::string("таймаут")
                                                       : std::to_string(code))
                      << "\n";
        }

        const RetryDecision decision = classify(code);

        if (decision == RetryDecision::Done) {
            report.status = (code == 201) ? UploadStatus::Success : UploadStatus::Overwritten;

            // ТЗ п.5: після успішного POST перевіряємо GET-ом.
            std::string detail;
            report.verifiedByGet = verifyOnServer(testId, detail);
            if (!report.verifiedByGet) {
                report.status = UploadStatus::NotConfirmed;
                report.detail = detail;
            }
            if (cfg_.verbose) {
                std::cout << "        GET-перевірка: "
                          << (report.verifiedByGet ? "підтверджено" : detail) << "\n";
            }
            return report;
        }

        if (decision == RetryDecision::Stop) {
            // 400/401 — повторювати заборонено (ТЗ п.3).
            report.status = UploadStatus::Rejected;
            report.detail = res ? res->body : "";
            if (cfg_.verbose && res && !res->body.empty()) {
                std::cout << "        тіло відповіді: " << res->body << "\n";
            }
            return report;
        }

        // RetryDecision::Again — пауза перед наступною спробою.
        // Після останньої спроби спати не має сенсу.
        if (attempt < cfg_.maxAttempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.retryPauseMs));
        }
    }

    // Усі спроби вичерпано — фіксуємо і йдемо далі (ТЗ п.4).
    report.status = UploadStatus::Exhausted;
    report.detail = "вичерпано " + std::to_string(cfg_.maxAttempts) + " спроб";
    return report;
}

std::vector<TestReport> ResultUploader::uploadAll(const std::vector<std::string>& testIds) {
    std::vector<TestReport> reports;
    reports.reserve(testIds.size());
    for (const auto& id : testIds) {
        reports.push_back(uploadOne(id));
    }
    return reports;
}

// ------------------------------------------------------------
// ТЗ п.7: підсумковий звіт «тест → статус → кількість спроб».
// ------------------------------------------------------------
void printReport(const std::vector<TestReport>& reports) {
    const std::size_t lineWidth = kColTest + kColStatus + kColAttempts + 4;

    std::cout << "\n==================== ЗВІТ ====================\n";
    padTo("ТЕСТ", kColTest);
    padTo("СТАТУС", kColStatus);
    padTo("СПРОБ", kColAttempts);
    std::cout << "GET\n";
    std::cout << std::string(lineWidth, '-') << "\n";

    int ok = 0;
    int failed = 0;
    int skipped = 0;

    for (const auto& r : reports) {
        padTo(r.testId, kColTest);
        padTo(toString(r.status), kColStatus);
        padTo(std::to_string(r.attempts), kColAttempts);
        std::cout << (r.verifiedByGet ? "OK" : "-") << "\n";

        if (!r.detail.empty()) {
            std::cout << "         └─ " << r.detail << "\n";
        }

        if (isSuccessful(r.status)) {
            ++ok;
        } else if (r.status == UploadStatus::Missing) {
            ++skipped;
        } else {
            ++failed;
        }
    }

    std::cout << std::string(lineWidth, '-') << "\n";
    std::cout << "Успішно: " << ok << "   Невдало: " << failed << "   Пропущено: " << skipped
              << "   Разом: " << reports.size() << "\n";
    std::cout << "==============================================\n";
}

}  // namespace dz15
