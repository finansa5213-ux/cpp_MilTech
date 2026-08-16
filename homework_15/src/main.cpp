// ============================================================
// main.cpp — ДЗ15 (Заняття 30): постинг результатів на сервер курсу.
//
// Запуск:
//   ./homework_15 --student <ID> [--results <dir>] [--host <host>]
//                 [--port <n>] [--key <api-key>] [--tests T01,T02]
//
// Мережевий код — власний (cpp-httplib), зовнішні програми
// (curl тощо) не викликаються, як вимагає ТЗ.
// ============================================================
#include "ResultUploader.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

// Ключ доступу з умови ДЗ. Можна перевизначити прапорцем --key.
constexpr const char* kDefaultApiKey = "dz12-vX7mK4qT9r2w";
constexpr const char* kDefaultHost = "cppmiltech.com.ua";
constexpr const char* kDefaultStudentId = "2009";
constexpr int kDefaultPort = 80;

// Повний перелік тестів за ТЗ п.1.
std::vector<std::string> defaultTestIds() {
    return {"T01", "T02", "T03", "T04", "T05", "T06", "T07", "T08", "T09", "T10"};
}

// Розбір списку тестів через кому: "T01,T03,T07".
std::vector<std::string> parseTestIds(const std::string& csv) {
    std::vector<std::string> ids;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) {
            ids.push_back(item);
        }
    }
    return ids;
}

void printUsage(const char* exe) {
    std::cout << "Використання:\n  " << exe << " --student <ID> [опції]\n\n"
              << "Опції:\n"
              << "  --student <ID>    ваш studentId (обов'язково)\n"
              << "  --results <dir>   каталог з результатами (типово: results)\n"
              << "  --host <host>     хост сервера (типово: " << kDefaultHost << ")\n"
              << "  --port <n>        порт (типово: " << kDefaultPort << ")\n"
              << "  --key <key>       x-api-key (типово: ключ курсу)\n"
              << "  --tests <list>    список тестів через кому (типово: T01..T10)\n"
              << "  --quiet           без покрокового логу\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    dz15::UploaderConfig cfg;
    cfg.apiKey = kDefaultApiKey;
    cfg.host = kDefaultHost;
    cfg.port = kDefaultPort;

    std::vector<std::string> testIds = defaultTestIds();

    // --- Розбір аргументів ------------------------------------
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasNext = (i + 1 < argc);

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--quiet") {
            cfg.verbose = false;
            continue;
        }
        if (!hasNext) {
            std::cerr << "Помилка: для " << arg << " не вказано значення\n";
            return 1;
        }

        const std::string value = argv[++i];
        if (arg == "--student") {
            cfg.studentId = value;
        } else if (arg == "--results") {
            cfg.resultsDir = value;
        } else if (arg == "--host") {
            cfg.host = value;
        } else if (arg == "--port") {
            cfg.port = std::stoi(value);
        } else if (arg == "--key") {
            cfg.apiKey = value;
        } else if (arg == "--tests") {
            testIds = parseTestIds(value);
        } else {
            std::cerr << "Невідомий аргумент: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (cfg.studentId.empty()) {
        std::cerr << "Помилка: не вказано --student <ID>\n\n";
        printUsage(argv[0]);
        return 1;
    }

    // --- Робота ------------------------------------------------
    std::cout << "Сервер:    http://" << cfg.host << ":" << cfg.port << "\n"
              << "Студент:   " << cfg.studentId << "\n"
              << "Каталог:   " << cfg.resultsDir << "\n"
              << "Тестів:    " << testIds.size() << "\n"
              << "Таймаути:  connect=" << cfg.connectTimeoutSec << "с read=" << cfg.readTimeoutSec
              << "с   Спроб: до " << cfg.maxAttempts << ", пауза " << cfg.retryPauseMs << " мс\n\n";

    dz15::ResultUploader uploader(cfg);
    const auto reports = uploader.uploadAll(testIds);
    dz15::printReport(reports);

    // Ненульовий код повернення, якщо хоч один тест провалився
    // з мережевої/протокольної причини (відсутній файл — не помилка).
    for (const auto& r : reports) {
        if (!dz15::isSuccessful(r.status) && r.status != dz15::UploadStatus::Missing) {
            return 2;
        }
    }
    return 0;
}
