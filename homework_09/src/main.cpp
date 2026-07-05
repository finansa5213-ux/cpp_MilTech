// ============================================================
// main.cpp — ДЗ3.7: розумні вказівники + State + TableSolver.
// ============================================================

#include "config/ComponentFactory.h"
#include "MissionProcessor.h"
#include "providers/JsonTargetProvider.h"
#include "json.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <memory>
#include <utility>

using json = nlohmann::json;

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    // --- 1. Лоадер (unique_ptr — без ручного delete) ---
    auto loader = createLoader(LoaderType::FILE, "config.json", "ammo.json");
    if (!loader) return 1;

    if (!loader->load()) {
        std::cerr << "main: помилка завантаження конфігу\n";
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

    auto provider = createProvider(ProviderType::JSON, providerParam);
    if (!provider) return 1;

    auto* jsonProv = dynamic_cast<JsonTargetProvider*>(provider.get());
    if (!jsonProv || !jsonProv->loadFromFile()) {
        std::cerr << "main: не вдалося завантажити targets.json\n";
        return 1;
    }
    std::cout << "[main] Цілей: " << provider->getTargetCount() << "\n\n";

    // --- 3. Solver ---
    auto solver = createSolver(SolverType::ANALYTICAL);
    if (!solver) return 1;

    // --- 4. MissionProcessor — передаємо власність через std::move() ---
    MissionProcessor mission(std::move(loader),
                             std::move(provider),
                             std::move(solver));
    if (!mission.init()) {
        std::cerr << "main: помилка init\n";
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
            std::cout << "  [ok] Ціль #"  << dp.targetIndex
                      << "  fire=("    << dp.firePoint.x
                      << ", "          << dp.firePoint.y << ")"
                      << "  pred=("    << dp.predictedTarget.x
                      << ", "          << dp.predictedTarget.y << ")"
                      << "  підліт="   << dp.totalTime  << " с"
                      << "  падіння="  << dp.fallTime   << " с\n\n";
        } else {
            std::cout << "  [skip] Ціль #" << dp.targetIndex
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

    std::cout << "[main] Оброблено: " << processed << " цілей"
              << "  |  пропущено через помилки: "
              << mission.skippedTargets() << "\n\n";

    // --- 6. Демонстрація reset() ---
    std::cout << "=== Демонстрація reset() ===\n";
    mission.reset();
    int again = 0;
    while (mission.hasNext()) { mission.step(); ++again; }
    std::cout << "[main] Повторний прохід: " << again << " цілей\n\n";

    // --- 7. Демонстрація підміни солвера на табличний (TableSolver) ---
    std::cout << "=== Демонстрація changeSolver() -> TABLE ===\n";
    mission.changeSolver(createSolver(SolverType::TABLE, "ballistic_table.txt"));
    mission.reset();
    int tableRun = 0;
    while (mission.hasNext()) { mission.step(); ++tableRun; }
    std::cout << "[main] Прохід з табличним солвером: "
              << tableRun << " цілей\n\n";

    // --- 8. Збереження результату ---
    if (std::ofstream out("simulation.json"); out.is_open()) {
        out << result.dump(2);
        std::cout << "[main] Результат -> simulation.json\n";
    }

    // --- 9. Звільнення ---
    // Нічого не звільняємо вручну: усі компоненти у власності
    // MissionProcessor (unique_ptr) і знищуються автоматично.
    return 0;
}
