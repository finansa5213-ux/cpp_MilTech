// ============================================================
// MissionProcessor.cpp
//
// step() виконує повний симуляційний цикл:
//   1. Балістика (solver) → firePoint
//   2. drone_.retarget(firePoint)
//   3. while tick() != ATTACK | ERROR | STOP
//   4. При ERROR → findNearestNextTarget() → перехід до неї
//   5. Фіксуємо реальну позицію дрона у момент скиду
// ============================================================

#include "MissionProcessor.h"
#include <iostream>
#include <limits>

MissionProcessor::MissionProcessor(IConfigLoader*    loader,
                                   ITargetProvider*  provider,
                                   IBallisticSolver* solver)
    : loader_(loader), provider_(provider), solver_(solver)
{}

// ------------------------------------------------------------
// findNearestNextTarget
// Перебирає всі необроблені цілі (від currentIdx_+1 до кінця),
// повертає індекс тієї, що найближча до поточної позиції дрона.
// Повертає -1 якщо більше цілей немає.
// ------------------------------------------------------------
int MissionProcessor::findNearestNextTarget() const
{
    const int total = provider_->getTargetCount();
    const int start = currentIdx_ + 1;   // поточну вже пропускаємо
    if (start >= total) return -1;

    const Coord dronePos = drone_.data().pos;
    int   bestIdx  = -1;
    float bestDist = std::numeric_limits<float>::max();

    for (int i = start; i < total; ++i) {
        Target t    = provider_->getTarget(i);
        float  dist = distance(dronePos, t.pos);
        if (dist < bestDist) {
            bestDist = dist;
            bestIdx  = i;
        }
    }
    return bestIdx;
}

// ------------------------------------------------------------
// init
// ------------------------------------------------------------
bool MissionProcessor::init(const char* /*configSource*/)
{
    if (!loader_ || !provider_ || !solver_) {
        std::cerr << "MissionProcessor::init — нульовий компонент\n";
        return false;
    }
    if (!loader_->load()) {
        std::cerr << "MissionProcessor::init — лоадер не завантажився\n";
        return false;
    }

    cfg_  = loader_->getConfig();
    ammo_ = loader_->getAmmoParams();

    if (provider_->getTargetCount() <= 0) {
        std::cerr << "MissionProcessor::init — провайдер не має цілей\n";
        return false;
    }

    drone_.init(cfg_.startPos,
                cfg_.initialDir,
                cfg_.attackSpeed,
                cfg_.accelPath,
                cfg_.angularSpeed,
                cfg_.turnThreshold,
                cfg_.hitRadius,
                cfg_.maxSteps,
                cfg_.startPos);

    currentIdx_      = 0;
    skippedTargets_  = 0;
    initialized_     = true;
    return true;
}

// ------------------------------------------------------------
// hasNext
// ------------------------------------------------------------
bool MissionProcessor::hasNext() const
{
    return initialized_ && (currentIdx_ < provider_->getTargetCount());
}

// ------------------------------------------------------------
// recoverFromError — спільна логіка відновлення після ERROR.
// Знаходить найближчу наступну ціль і переводить машину до неї.
// Повертає true якщо є куди летіти, false якщо цілей більше немає.
// ------------------------------------------------------------
static bool recoverFromError(DroneStateMachine& drone,
                              ITargetProvider*   provider,
                              int&               currentIdx,
                              int&               skippedTargets)
{
    ++skippedTargets;

    // findNearestNextTarget — метод MissionProcessor, тому дублюємо
    // логіку тут як лямбду через публічний інтерфейс.
    const int total   = provider->getTargetCount();
    const int start   = currentIdx + 1;
    if (start >= total) {
        std::cerr << "  [SM] Більше цілей немає після пропуску #"
                  << currentIdx << "\n";
        drone.stop();
        return false;
    }

    const Coord dronePos = drone.data().pos;
    int   bestIdx  = -1;
    float bestDist = std::numeric_limits<float>::max();
    for (int i = start; i < total; ++i) {
        Target t    = provider->getTarget(i);
        float  dist = distance(dronePos, t.pos);
        if (dist < bestDist) { bestDist = dist; bestIdx = i; }
    }

    if (bestIdx < 0) { drone.stop(); return false; }

    const int skipped = bestIdx - currentIdx - 1;
    std::cerr << "  [SM] Пропускаємо #" << currentIdx
              << " → переходимо до найближчої цілі #" << bestIdx
              << " (dist=" << bestDist << " м";
    if (skipped > 0)
        std::cerr << ", пропущено проміжних: " << skipped;
    std::cerr << ")\n";

    skippedTargets += skipped;
    currentIdx      = bestIdx;

    // Переорієнтовуємо машину на нову ціль без скидання позиції
    Target nextTgt = provider->getTarget(bestIdx);
    drone.retarget(nextTgt.pos);   // тимчасовий firePoint — уточниться в step()
    return true;
}

// ------------------------------------------------------------
// step
// ------------------------------------------------------------
DropPoint MissionProcessor::step()
{
    DropPoint dp;
    dp.targetIndex = currentIdx_;

    if (!hasNext()) return dp;

    Target tgt = provider_->getTarget(currentIdx_);

    // --- 1. Балістика ---
    dp = solver_->solve(drone_.data().pos,
                        tgt,
                        cfg_.altitude,
                        ammo_,
                        cfg_.attackSpeed);
    dp.targetIndex = currentIdx_;

    if (!dp.valid) {
        drone_.error(DroneErrorCode::BALLISTICS_FAIL,
                     "solver returned valid=false for target #"
                     + std::to_string(currentIdx_));
        std::cerr << "  [SM] Ціль #" << currentIdx_
                  << " [BALLISTICS_FAIL] — шукаємо найближчу наступну\n";

        recoverFromError(drone_, provider_, currentIdx_,
                         skippedTargets_);
        return dp;
    }

    // --- 2. Передаємо firePoint у стейт машину ---
    drone_.retarget(dp.firePoint);

    std::cout << "  [SM] Ціль #"  << currentIdx_
              << " firePoint=("   << dp.firePoint.x
              << ", "             << dp.firePoint.y << ")"
              << "  dronePos=("   << drone_.data().pos.x
              << ", "             << drone_.data().pos.y << ")"
              << "  стан="        << droneStateName(drone_.state()) << "\n";

    // --- 3. Симуляційний цикл ---
    DroneState prevState = drone_.state();
    int ticks = 0;

    while (drone_.state() != DroneState::ATTACK &&
           drone_.state() != DroneState::ERROR  &&
           drone_.state() != DroneState::STOP)
    {
        drone_.tick(cfg_.simTimeStep);
        ++ticks;

        if (drone_.state() != prevState) {
            std::cout << "    [SM] " << droneStateName(prevState)
                      << " → "      << droneStateName(drone_.state())
                      << "  t="     << drone_.data().simTime << " с"
                      << "  pos=("  << drone_.data().pos.x
                      << ", "       << drone_.data().pos.y << ")"
                      << "  spd="   << drone_.data().speed  << " м/с\n";
            prevState = drone_.state();
        }
    }

    // --- 4. Обробка результату ---
    if (drone_.state() == DroneState::ERROR) {
        std::cerr << "  [SM] Ціль #" << currentIdx_
                  << " [" << droneErrorName(drone_.data().errorCode) << "] "
                  << drone_.data().errorMsg
                  << " — шукаємо найближчу наступну\n";
        dp.valid = false;

        recoverFromError(drone_, provider_, currentIdx_,
                         skippedTargets_);
        return dp;
    }

    dp.firePoint = drone_.data().pos;
    dp.totalTime = drone_.data().simTime;

    std::cout << "  [SM] → ATTACK за " << ticks << " тіків"
              << " (" << dp.totalTime << " с)\n\n";

    ++currentIdx_;
    return dp;
}

// ------------------------------------------------------------
// reset
// ------------------------------------------------------------
void MissionProcessor::reset()
{
    currentIdx_     = 0;
    skippedTargets_ = 0;
    drone_.init(cfg_.startPos,
                cfg_.initialDir,
                cfg_.attackSpeed,
                cfg_.accelPath,
                cfg_.angularSpeed,
                cfg_.turnThreshold,
                cfg_.hitRadius,
                cfg_.maxSteps,
                cfg_.startPos);
}

void MissionProcessor::changeSolver(IBallisticSolver* s)
{
    if (s) solver_ = s;
}
