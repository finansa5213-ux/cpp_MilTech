// ============================================================
// MissionProcessor.cpp (ДЗ10) — місія у власному потоці.
//
// run() — тіло потоку:
//   для кожної цілі:
//     1. читає телеметрію фізики;
//     2. рахує балістику → firePoint;
//     3. крок планування (simTimeStep): синхронізує стейт-машину
//        з телеметрії, вирішує режим, шле команду фізиці, пише
//        крок у лог (з timeSecSinceStart);
//     4. ATTACK → наступна ціль; ERROR → найближча наступна.
// ============================================================
#include "MissionProcessor.h"
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <limits>
#include <thread>
#include <chrono>
#include <utility>

using json = nlohmann::json;

MissionProcessor::MissionProcessor(std::unique_ptr<IConfigLoader>    loader,
                                   ITargetProvider*                  provider,
                                   std::unique_ptr<IBallisticSolver> solver,
                                   DronePhysics*                     physics)
    : loader_(std::move(loader))
    , provider_(provider)
    , solver_(std::move(solver))
    , physics_(physics)
{}

bool MissionProcessor::init()
{
    if (!loader_ || !provider_ || !solver_ || !physics_) {
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

    drone_.init(cfg_.startPos, cfg_.initialDir, cfg_.attackSpeed,
                cfg_.accelPath, cfg_.angularSpeed, cfg_.turnThreshold,
                cfg_.hitRadius, cfg_.maxSteps, cfg_.startPos);

    currentIdx_     = 0;
    skippedTargets_ = 0;
    initialized_    = true;
    return true;
}

int MissionProcessor::findNearestNextTarget(const Coord& dronePos) const
{
    const int total = provider_->getTargetCount();
    const int start = currentIdx_ + 1;
    if (start >= total) return -1;

    int   bestIdx  = -1;
    float bestDist = std::numeric_limits<float>::max();
    for (int i = start; i < total; ++i) {
        Target t    = provider_->getTarget(i);
        float  dist = distance(dronePos, t.pos);
        if (dist < bestDist) { bestDist = dist; bestIdx = i; }
    }
    return bestIdx;
}

bool MissionProcessor::recoverFromError(const Coord& dronePos)
{
    ++skippedTargets_;
    int bestIdx = findNearestNextTarget(dronePos);
    if (bestIdx < 0) {
        std::cerr << "  [SM] Більше цілей немає після пропуску #"
                  << currentIdx_ << "\n";
        return false;
    }
    const int skipped = bestIdx - currentIdx_ - 1;
    if (skipped > 0) skippedTargets_ += skipped;
    std::cerr << "  [SM] Пропускаємо #" << currentIdx_
              << " → найближча ціль #" << bestIdx << "\n";
    currentIdx_ = bestIdx;
    Target nextTgt = provider_->getTarget(bestIdx);
    drone_.retarget(nextTgt.pos);
    return true;
}

DroneCommand MissionProcessor::makeCommand(DroneMode mode,
                                           const Coord& firePoint) const
{
    DroneCommand cmd;
    cmd.mode          = mode;
    cmd.firePoint     = firePoint;
    cmd.angleSpeed    = cfg_.angularSpeed;
    cmd.attackSpeed   = cfg_.attackSpeed;
    cmd.accelPath     = cfg_.accelPath;
    cmd.turnThreshold = cfg_.turnThreshold;
    cmd.hitRadius     = cfg_.hitRadius;
    return cmd;
}

void MissionProcessor::run()
{
    ready_.store(true);

    while (!started_.load() && !stop_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (stop_.load() || !initialized_) return;

    const float scale = (cfg_.timeScale > EPS) ? cfg_.timeScale : 1.0f;
    const float dt    = cfg_.simTimeStep;

    json result;
    result["steps"] = json::array();
    auto pushStep = [&](const DroneTelemetry& tel, const DropPoint& dp) {
        result["steps"].push_back({
            {"position",        {{"x", tel.pos.x}, {"y", tel.pos.y}}},
            {"direction",       tel.dir},
            {"state",           droneModeName(tel.mode)},
            {"targetIndex",     currentIdx_},
            {"dropPoint",       {{"x", dp.firePoint.x}, {"y", dp.firePoint.y}}},
            {"aimPoint",        {{"x", dp.firePoint.x}, {"y", dp.firePoint.y}}},
            {"predictedTarget", {{"x", dp.predictedTarget.x},
                                 {"y", dp.predictedTarget.y}}},
            {"timeSecSinceStart", tel.timeSecSinceStart}
        });
    };

    int processed = 0;

    while (currentIdx_ < provider_->getTargetCount() && !stop_.load()) {
        Target         tgt = provider_->getTarget(currentIdx_);
        DroneTelemetry tel = physics_->getTelemetry();

        // --- 1. Балістика ---
        DropPoint dp = solver_->solve(tel.pos, tgt, cfg_.altitude,
                                      ammo_, cfg_.attackSpeed);
        dp.targetIndex = currentIdx_;

        if (!dp.valid) {
            std::cerr << "  [SM] Ціль #" << currentIdx_
                      << " [BALLISTICS_FAIL]\n";
            if (!recoverFromError(tel.pos)) break;
            continue;
        }

        // --- 2. Націлюємо стейт-машину ---
        drone_.retarget(dp.firePoint);
        std::cout << "  [SM] Ціль #" << currentIdx_
                  << " firePoint=(" << dp.firePoint.x << ", "
                  << dp.firePoint.y << ")\n";

        // --- 3. Цикл планування для цього сегмента ---
        bool segmentClosed = false;
        while (!segmentClosed && !stop_.load()) {
            tel = physics_->getTelemetry();

            // Синхронізуємо планувальник із реальним станом фізики.
            drone_.syncKinematics(tel.pos, tel.dir, length(tel.speed));
            drone_.tick(dt);                 // рішення (інтегрування — у фізики)

            DroneMode mode = drone_.mode();
            physics_->sendCommand(makeCommand(mode, dp.firePoint));

            // лог: позиція/курс — з телеметрії; час — timeSecSinceStart
            tel.mode = mode;                 // показуємо ухвалений режим
            pushStep(tel, dp);

            if (drone_.isAttack()) {
                dp.firePoint = tel.pos;
                dp.totalTime = tel.timeSecSinceStart;
                std::cout << "  [SM] → ATTACK ціль #" << currentIdx_
                          << "  t=" << tel.timeSecSinceStart << " с\n";
                ++currentIdx_;
                ++processed;
                segmentClosed = true;
            } else if (drone_.isError()) {
                std::cerr << "  [SM] Ціль #" << currentIdx_ << " ["
                          << droneErrorName(drone_.data().errorCode) << "]\n";
                if (!recoverFromError(tel.pos)) stop_.store(true);
                segmentClosed = true;
            } else if (drone_.isStop()) {
                segmentClosed = true;
                stop_.store(true);
            }

            std::this_thread::sleep_for(
                std::chrono::duration<float>(dt / scale));
        }
    }

    // Фінальна команда: утримувати позицію.
    physics_->sendCommand(makeCommand(DroneMode::STOP, drone_.firePointParam()));

    // --- Збереження ---
    result["processed"]      = processed;
    result["skippedTargets"] = skippedTargets_;
    if (std::ofstream out(outFile_); out.is_open()) {
        out << result.dump(2);
        std::cout << "[mission] Результат -> " << outFile_
                  << "  (кроків: " << result["steps"].size() << ")\n";
    } else {
        std::cerr << "[mission] не вдалося записати " << outFile_ << "\n";
    }
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> s)
{
    if (s) solver_ = std::move(s);
}
