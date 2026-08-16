#pragma once
// ============================================================
// DronePhysics — фізика дрона у власному потоці (ДЗ10).
// Потік запускається ззовні: std::thread(&DronePhysics::run, &phys).
// isThreadReady() → start() → (робота) → stop() (атомарний прапорець).
// ============================================================
#include "Types.h"
#include "ThreadSafeQueue.h"
#include <atomic>
#include <mutex>

class DronePhysics {
public:
    DronePhysics(const MissionConfig& cfg);

    // --- життєвий цикл потоку ---
    void run();                 // тіло потоку (передається у std::thread)
    bool isThreadReady() const { return ready_.load(); }
    void start()               { started_.store(true); }
    void stop();               // атомарний стоп + розблокування черги

    // --- інтерфейс для MissionProcessor ---
    void           sendCommand(const DroneCommand& cmd) { cmdQueue_.push(cmd); }
    DroneTelemetry getTelemetry() const;                // копія під м'ютексом

    // Скинути стан дрона у стартову позицію (до запуску потоку).
    void resetState(const Coord& startPos, float initialDir);

private:
    void integrate(float dt);   // один крок фізики

    MissionConfig cfg_;

    // --- стан дрона (захищений stateMtx_) ---
    mutable std::mutex stateMtx_;
    Coord       pos_;
    float       dir_   = 0.f;
    float       speed_ = 0.f;            // скалярна швидкість уздовж курсу
    DroneMode   mode_  = DroneMode::IDLE;
    float       simTime_ = 0.f;          // timeSecSinceStart (модельний час)

    // --- остання прийнята команда ---
    DroneCommand current_;

    // --- синхронізація потоку ---
    ThreadSafeQueue<DroneCommand> cmdQueue_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};
};
