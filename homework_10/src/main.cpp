// ============================================================
// main.cpp — ДЗ10: цілі, фізика та місія у трьох окремих потоках.
//
// Потік 1: ThreadSafeTargetProvider — рухає цілі.
// Потік 2: DronePhysics            — інтегрує рух дрона.
// Потік 3: MissionProcessor        — рішення, балістика, лог.
//
// main чекає лише на потік місії; провайдер і фізика зупиняються
// стоп-прапорцями після завершення місії. Жодного detach().
// ============================================================
#include "config/ComponentFactory.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "drone/DronePhysics.h"
#include "MissionProcessor.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <utility>

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    // --- 1. Лоадер + конфіг ---
    auto loader = createLoader(LoaderType::FILE, "config.json", "ammo.json");
    if (!loader || !loader->load()) {
        std::cerr << "main: помилка завантаження конфігу\n";
        return 1;
    }
    const MissionConfig cfg = loader->getConfig();

    std::cout << "[main] simTimeStep="     << cfg.simTimeStep
              << "  physicsTimeStep="      << cfg.physicsTimeStep
              << "  arrayTimeStep="        << cfg.arrayTimeStep
              << "  timeScale="            << cfg.timeScale << "\n";

    // --- 2. Провайдер (власний потік) ---
    auto provider = std::make_unique<ThreadSafeTargetProvider>(
        "targets.json", cfg.arrayTimeStep, cfg.timeScale);
    if (!provider->loadFromFile()) {
        std::cerr << "main: не вдалося завантажити targets.json\n";
        return 1;
    }
    std::cout << "[main] Цілей: " << provider->getTargetCount() << "\n";

    // --- 3. Фізика (власний потік) ---
    auto physics = std::make_unique<DronePhysics>(cfg);

    // --- 4. Solver + MissionProcessor (власний потік) ---
    auto solver = createSolver(SolverType::ANALYTICAL);
    if (!solver) return 1;

    MissionProcessor mission(std::move(loader),
                             provider.get(),
                             std::move(solver),
                             physics.get());
    if (!mission.init()) {
        std::cerr << "main: помилка init\n";
        return 1;
    }

    // --- 5. Запуск потоків ---
    std::thread providerThread(&ThreadSafeTargetProvider::run, provider.get());
    std::thread physicsThread (&DronePhysics::run,             physics.get());
    std::thread missionThread (&MissionProcessor::run,         &mission);

    // Чекаємо готовності всіх потоків.
    while (!provider->isThreadReady()
           || !physics->isThreadReady()
           || !mission.isThreadReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::cout << "[main] Усі потоки готові — старт.\n\n";

    // Старт руху — одночасно, без розсинхронізації.
    provider->start();
    physics->start();
    mission.start();

    // main чекає лише на місію.
    missionThread.join();

    // Місія завершилась — зупиняємо решту.
    physics->stop();
    provider->stop();
    physicsThread.join();
    providerThread.join();

    std::cout << "\n[main] Готово. Пропущено цілей: "
              << mission.skippedTargets() << "\n";
    return 0;
}
