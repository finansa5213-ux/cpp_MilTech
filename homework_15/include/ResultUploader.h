// ============================================================
// ResultUploader.h — HTTP-клієнт для здачі результатів на сервер курсу.
//
// Протокол (ТЗ ДЗ15):
//   POST /api/dz12/results          — надіслати результат
//   GET  /api/dz12/results/{testId}/{studentId} — перевірити наявність
// Автентифікація — заголовок x-api-key.
// ============================================================
#ifndef RESULT_UPLOADER_H
#define RESULT_UPLOADER_H

#include "UploadTypes.h"

#include <string>
#include <vector>

namespace dz15 {

// ------------------------------------------------------------
// Налаштування клієнта. Винесені в структуру, щоб параметри
// (таймаути, ліміт спроб, пауза) не були «магічними числами»
// всередині коду і легко перевизначались у тестах.
// ------------------------------------------------------------
struct UploaderConfig {
    std::string host = "cppmiltech.com.ua";  // без схеми
    int port = 80;
    std::string apiKey;
    std::string studentId;

    std::string resultsDir = "results";      // results/T01/simulation.json

    // ТЗ п.2: таймаути з'єднання і читання — не більше 2 секунд кожен.
    int connectTimeoutSec = 2;
    int readTimeoutSec = 2;

    // ТЗ п.4: не більше 5 спроб на тест, пауза не менше 1 секунди.
    int maxAttempts = 5;
    int retryPauseMs = 1000;

    bool verbose = true;
};

// ------------------------------------------------------------
// ResultUploader — послідовно обробляє список тестів.
//
// Ідемпотентність (ТЗ п.6) забезпечується самим протоколом:
// сервер перезаписує результат, тому повторний запуск програми
// безпечний і не потребує локального стану між запусками.
// ------------------------------------------------------------
class ResultUploader {
public:
    explicit ResultUploader(UploaderConfig config);

    // Обробляє один тест: читання файла → POST з ретраями → контрольний GET.
    TestReport uploadOne(const std::string& testId);

    // Обробляє список тестів підряд. Невдача одного теста не перериває
    // роботу — програма переходить до наступного (ТЗ п.4).
    std::vector<TestReport> uploadAll(const std::vector<std::string>& testIds);

private:
    // Шлях до локального результату: <resultsDir>/<testId>/simulation.json
    std::string resultPath(const std::string& testId) const;

    // Класифікація коду відповіді → рішення про повтор (ТЗ п.3).
    static RetryDecision classify(int httpStatus);

    // Контрольний GET після успішного POST (ТЗ п.5).
    bool verifyOnServer(const std::string& testId, std::string& detail);

    UploaderConfig cfg_;
};

// Друк підсумкової таблиці «тест → статус → спроби» (ТЗ п.7).
void printReport(const std::vector<TestReport>& reports);

}  // namespace dz15

#endif  // RESULT_UPLOADER_H
