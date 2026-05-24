// ============================================================
// main.cpp — демонстрація рефакторингу ДЗ3 в архітектуру з
// інтерфейсів, реалізацій, фабрики та MissionProcessor.
//
// Що тут показано:
//   1. Створення всіх трьох компонентів виключно через фабрику.
//   2. Завантаження даних: loader робить це сам у init();
//      провайдер завантажуємо явно (loadFromFile), бо ITargetProvider
//      за ТЗ не має методу load().
//   3. MissionProcessor::init() для підготовки даних.
//   4. Покрокова обробка циклом while (mission.hasNext()) { mission.step(); }
//   5. delete для всіх створених через фабрику об'єктів.
// ============================================================

#include "Factory.h"
#include "MissionProcessor.h"
#include "JsonTargetProvider.h"   // потрібен для виклику loadFromFile()
#include "FileConfigLoader.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <clocale>
#include "json.hpp"

using json = nlohmann::json;

int main() {
    setlocale(LC_ALL, "");
    std::cout << std::fixed << std::setprecision(3);

    // ---------- 1. Створюємо лоадер через фабрику ----------
    IConfigLoader* loader = createLoader(LoaderType::FILE,
                                          "config.json", "ammo.json");
    if (loader == nullptr) return 1;

    // Лоадеру довіримось завантажити дані ще до створення провайдера —
    // нам потрібен arrayTimeStep із конфігу, щоб коректно ініціалізувати
    // JsonTargetProvider.
    if (!loader->load()) {
        std::cerr << "main: помилка завантаження конфігу" << std::endl;
        delete loader;
        return 1;
    }
    const MissionConfig& cfg  = loader->getConfig();
    const AmmoParams&    ammo = loader->getAmmoParams();

    std::cout << "[main] Конфіг: ammo=" << cfg.ammoName
              << ", altitude=" << cfg.altitude
              << ", attackSpeed=" << cfg.attackSpeed
              << ", arrayTimeStep=" << cfg.arrayTimeStep << std::endl;
    std::cout << "[main] Боєприпас: " << ammo.name
              << " (m=" << ammo.mass
              << ", d=" << ammo.drag
              << ", l=" << ammo.lift << ")" << std::endl;

    // ---------- 2. Створюємо провайдер через фабрику ----------
    // Передаємо в param шлях ТА arrayTimeStep через роздільник '|'.
    char providerParam[128];
    std::snprintf(providerParam, sizeof(providerParam),
                  "targets.json|%f", cfg.arrayTimeStep);

    ITargetProvider* provider = createProvider(ProviderType::JSON, providerParam);
    if (provider == nullptr) { delete loader; return 1; }

    // ITargetProvider за ТЗ не має методу load(); викликаємо
    // конкретно для JsonTargetProvider через приведення.
    JsonTargetProvider* jsonProv = dynamic_cast<JsonTargetProvider*>(provider);
    if (jsonProv == nullptr || !jsonProv->loadFromFile()) {
        std::cerr << "main: не вдалося завантажити targets.json" << std::endl;
        delete provider;
        delete loader;
        return 1;
    }
    std::cout << "[main] Цілей завантажено: " << provider->getTargetCount() << std::endl;

    // ---------- 3. Створюємо solver через фабрику ----------
    IBallisticSolver* solver = createSolver(SolverType::ANALYTICAL);
    if (solver == nullptr) {
        delete provider; delete loader; return 1;
    }

    // ---------- 4. Збираємо MissionProcessor через інтерфейси ----------
    MissionProcessor mission(loader, provider, solver);

    if (!mission.init()) {
        std::cerr << "main: помилка init MissionProcessor" << std::endl;
        delete solver; delete provider; delete loader;
        return 1;
    }

    // ---------- 5. Покрокова обробка ----------
    std::cout << "\n=== Обробка цілей ===" << std::endl;

    // Збираємо результати для запису у simulation.json — простий формат
    // (масив точок скиду по цілях). Це демонстрація, не повна симуляція ДЗ3.
    json result;
    result["dropPoints"] = json::array();

    int processed = 0;
    while (mission.hasNext()) {
        DropPoint dp = mission.step();
        processed++;

        if (dp.valid) {
            std::cout << "  Ціль #" << dp.targetIndex
                      << " | fire=(" << dp.firePoint.x << ", " << dp.firePoint.y << ")"
                      << " | predTgt=(" << dp.predictedTarget.x << ", " << dp.predictedTarget.y << ")"
                      << " | tFall=" << dp.totalTime << " с"
                      << std::endl;
        } else {
            std::cout << "  Ціль #" << dp.targetIndex
                      << " | балістика не розв'язується" << std::endl;
        }

        json item;
        item["targetIndex"]     = dp.targetIndex;
        item["valid"]           = dp.valid;
        item["firePoint"]       = { {"x", dp.firePoint.x},       {"y", dp.firePoint.y} };
        item["predictedTarget"] = { {"x", dp.predictedTarget.x}, {"y", dp.predictedTarget.y} };
        item["totalTime"]       = dp.totalTime;
        result["dropPoints"].push_back(item);
    }
    result["processed"] = processed;

    std::cout << "\n[main] Оброблено цілей: " << processed << std::endl;

    // ---------- 6. Демонстрація reset() + changeSolver() ----------
    // (Тут поки немає альтернативного solver-а, тому покажемо лише
    // структурно: reset + повторний прохід з тим же solver-ом).
    std::cout << "\n=== Демонстрація reset() ===" << std::endl;
    mission.reset();
    int again = 0;
    while (mission.hasNext()) { mission.step(); again++; }
    std::cout << "[main] Повторний прохід: " << again << " цілей" << std::endl;

    // Якщо колись додамо TableSolver:
    //   IBallisticSolver* newSolver = createSolver(SolverType::TABLE);
    //   mission.changeSolver(newSolver);
    //   mission.reset();
    //   while (mission.hasNext()) mission.step();
    //   delete newSolver;

    // ---------- 7. Запис результату у simulation.json ----------
    std::ofstream out("simulation.json");
    if (out.is_open()) {
        out << result.dump(2);
        out.close();
        std::cout << "[main] Результат у simulation.json" << std::endl;
    } else {
        std::cerr << "[main] Не вдалось відкрити simulation.json" << std::endl;
    }

    // ---------- 8. Звільнення всіх створених через фабрику об'єктів ----------
    delete solver;   solver   = nullptr;
    delete provider; provider = nullptr;
    delete loader;   loader   = nullptr;

    return 0;
}
