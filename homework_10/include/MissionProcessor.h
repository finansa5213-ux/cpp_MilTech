#pragma once
// ============================================================
// MissionProcessor — логіка місії у власному потоці (ДЗ10).
//
// Більше НЕ зберігає і не інтегрує стан дрона: щокроку запитує
// телеметрію у DronePhysics. Стейт-машина (класи станів з ДЗ9)
// лишається тут і виконує роль ПЛАНУВАЛЬНИКА — ухвалює рішення
// про режим, який надсилається фізиці командою.
//
// Власність: loader_ та solver_ — через unique_ptr.
// provider_ та physics_ — невласницькі вказівники (потоки і
// об'єкти живуть у main).
// ============================================================
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "drone/DroneStateMachine.h"
#include "drone/DronePhysics.h"
#include <memory>
#include <atomic>
#include <string>

class MissionProcessor {
public:
    MissionProcessor(std::unique_ptr<IConfigLoader>    loader,
                     ITargetProvider*                  provider,
                     std::unique_ptr<IBallisticSolver> solver,
                     DronePhysics*                     physics);
    ~MissionProcessor() = default;

    bool init();                       // завантажити конфіг, підготувати машину

    // --- життєвий цикл потоку ---
    void run();                        // тіло потоку
    bool isThreadReady() const { return ready_.load(); }
    void start()               { started_.store(true); }
    void stop()                { stop_.store(true); }

    void changeSolver(std::unique_ptr<IBallisticSolver> s);
    void setOutputFile(const std::string& f) { outFile_ = f; }

    const MissionConfig& config()        const { return cfg_;            }
    int                  skippedTargets() const { return skippedTargets_; }

private:
    int  findNearestNextTarget(const Coord& dronePos) const;
    bool recoverFromError(const Coord& dronePos);   // перейти до найближчої цілі
    DroneCommand makeCommand(DroneMode mode, const Coord& firePoint) const;

    std::unique_ptr<IConfigLoader>    loader_;
    ITargetProvider*                  provider_ = nullptr;   // невласницький
    std::unique_ptr<IBallisticSolver> solver_;
    DronePhysics*                     physics_  = nullptr;   // невласницький

    int               currentIdx_     = 0;
    int               skippedTargets_ = 0;
    bool              initialized_    = false;

    MissionConfig     cfg_;
    AmmoParams        ammo_;
    DroneStateMachine drone_;          // стейт-машина-планувальник
    std::string       outFile_ = "simulation.json";

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};
};
