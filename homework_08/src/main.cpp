// ============================================================
// main.cpp — ДЗ8: структура проекту + STL + DroneStateMachine.
// ============================================================

#include "config/ComponentFactory.h"
#include "MissionProcessor.h"
#include "providers/JsonTargetProvider.h"
#include "json.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

using json = nlohmann::json;

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    // --- 1. Лоадер ---
    IConfigLoader* loader = createLoader(LoaderType::FILE,
                                          "config.json", "ammo.json");
    if (!loader) return 1;

    if (!loader->load()) {
        std::cerr << "main: помилка завантаження конфігу\n";
        delete loader;
        return 1;
    }

    const MissionConfig& cfg  = loader->getConfig();
    const AmmoParams&    ammo = loader->getAmmoParams();

    std::cout << "[main] Конфіг:"
              << "  ammo="         << cfg.ammoName
              << "  altitude="     << cfg.altitude
              << "  attackSpeed="  << cfg.attackSpeed
              << "  simTimeStep="  << cfg.simTimeStep
              << "  hitRadius="    << cfg.hitRadius    << "\n";
    std::cout << "[main] Боєприпас: " << ammo.name
              << "  m=" << ammo.mass
              << "  d=" << ammo.drag
              << "  l=" << ammo.lift << "\n\n";

    // --- 2. Провайдер ---
    const std::string providerParam =
        "targets.json|" + std::to_string(cfg.arrayTimeStep);

    ITargetProvider* provider = createProvider(ProviderType::JSON, providerParam);
    if (!provider) { delete loader; return 1; }

    auto* jsonProv = dynamic_cast<JsonTargetProvider*>(provider);
    if (!jsonProv || !jsonProv->loadFromFile()) {
        std::cerr << "main: не вдалося завантажити targets.json\n";
        delete provider; delete loader;
        return 1;
    }
    std::cout << "[main] Цілей: " << provider->getTargetCount() << "\n\n";

    // --- 3. Solver ---
    IBallisticSolver* solver = createSolver(SolverType::ANALYTICAL);
    if (!solver) { delete provider; delete loader; return 1; }

    // --- 4. MissionProcessor ---
    MissionProcessor mission(loader, provider, solver);
    if (!mission.init()) {
        std::cerr << "main: помилка init\n";
        delete solver; delete provider; delete loader;
        return 1;
    }

    // --- 5. Покрокова обробка ---
    std::cout << "=== Обробка цілей ===\n\n";

    json result;
    result["dropPoints"] = json::array();
    int processed = 0;

    while (mission.hasNext()) {
        DropPoint dp = mission.step();
        ++processed;

        if (dp.valid) {
            std::cout << "  ✓ Ціль #"  << dp.targetIndex
                      << "  fire=("    << dp.firePoint.x
                      << ", "          << dp.firePoint.y << ")"
                      << "  pred=("    << dp.predictedTarget.x
                      << ", "          << dp.predictedTarget.y << ")"
                      << "  підліт="   << dp.totalTime  << " с"
                      << "  падіння="  << dp.fallTime   << " с\n\n";
        } else {
            std::cout << "  ✗ Ціль #" << dp.targetIndex
                      << "  балістика не розв'язується\n\n";
        }

        result["dropPoints"].push_back({
            {"targetIndex",     dp.targetIndex},
            {"valid",           dp.valid},
            {"firePoint",       {{"x", dp.firePoint.x},
                                 {"y", dp.firePoint.y}}},
            {"predictedTarget", {{"x", dp.predictedTarget.x},
                                 {"y", dp.predictedTarget.y}}},
            {"flightTime",      dp.totalTime},
            {"fallTime",        dp.fallTime}
        });
    }
    result["processed"] = processed;

    std::cout << "[main] Оброблено: " << processed << " цілей\n\n";

    // --- 6. Демонстрація reset() ---
    std::cout << "=== Демонстрація reset() ===\n";
    mission.reset();
    int again = 0;
    while (mission.hasNext()) { mission.step(); ++again; }
    std::cout << "[main] Повторний прохід: " << again << " цілей\n\n";

    // --- 7. Збереження результату ---
    if (std::ofstream out("simulation.json"); out.is_open()) {
        out << result.dump(2);
        std::cout << "[main] Результат → simulation.json\n";
    }

    // --- 8. Звільнення ---
    delete solver;
    delete provider;
    delete loader;
    return 0;
}