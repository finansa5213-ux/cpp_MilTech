// ============================================================
// DronePhysics.cpp — реалізація потоку фізики (ДЗ10).
// ============================================================
#include "drone/DronePhysics.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

DronePhysics::DronePhysics(const MissionConfig& cfg)
    : cfg_(cfg)
{
    pos_ = cfg.startPos;
    dir_ = cfg.initialDir;
}

void DronePhysics::resetState(const Coord& startPos, float initialDir)
{
    std::lock_guard<std::mutex> lk(stateMtx_);
    pos_     = startPos;
    dir_     = initialDir;
    speed_   = 0.f;
    mode_    = DroneMode::IDLE;
    simTime_ = 0.f;
}

DroneTelemetry DronePhysics::getTelemetry() const
{
    std::lock_guard<std::mutex> lk(stateMtx_);
    DroneTelemetry t;
    t.pos               = pos_;
    t.speed             = Coord{std::cos(dir_) * speed_, std::sin(dir_) * speed_};
    t.dir               = dir_;
    t.mode              = mode_;
    t.timeSecSinceStart = simTime_;
    return t;
}

// Просунутися вздовж курсу на step метрів, але не "перестрибнути"
// firePoint (інакше дрон осцилює навколо цілі й ніколи не входить
// у hitRadius). Якщо крок більший за відстань — стаємо на ціль.
static inline void advance(Coord& pos, float dir, float step, const Coord& fire)
{
    float dist = distance(pos, fire);
    if (step >= dist) { pos = fire; return; }
    pos.x += std::cos(dir) * step;
    pos.y += std::sin(dir) * step;
}

// ------------------------------------------------------------
// integrate — один крок фізики на dt секунд.
// Кінематику взято з ДЗ9; тут вона лише ВИКОНУЄ режим, заданий
// місією. Рішення про зміну режиму ухвалює стейт-машина місії.
// ------------------------------------------------------------
void DronePhysics::integrate(float dt)
{
    std::lock_guard<std::mutex> lk(stateMtx_);

    const DroneCommand& c = current_;
    mode_ = c.mode;

    switch (c.mode) {
        case DroneMode::TURNING: {
            Coord delta = c.firePoint - pos_;
            if (length(delta) >= EPS) {
                float angle   = normalizeAngle(std::atan2(delta.y, delta.x) - dir_);
                float maxTurn = c.angleSpeed * dt;
                if (std::fabs(angle) <= maxTurn) dir_ += angle;
                else                             dir_ += std::copysign(maxTurn, angle);
                dir_ = normalizeAngle(dir_);
            }
            speed_ = c.attackSpeed * 0.1f;            // повзе повільно
            advance(pos_, dir_, speed_ * dt, c.firePoint);
            break;
        }
        case DroneMode::ACCELERATING: {
            float dist = distance(pos_, c.firePoint);
            if (c.accelPath > EPS) {
                float ratio = std::max(0.f, std::min(1.f, 1.f - dist / c.accelPath));
                speed_ = std::max(c.attackSpeed * 0.05f, c.attackSpeed * ratio);
            } else {
                speed_ = c.attackSpeed;
            }
            advance(pos_, dir_, speed_ * dt, c.firePoint);
            break;
        }
        case DroneMode::CRUISE: {
            speed_ = c.attackSpeed;
            advance(pos_, dir_, speed_ * dt, c.firePoint);
            break;
        }
        case DroneMode::ATTACK:
        case DroneMode::STOP:
        case DroneMode::ERROR:
        case DroneMode::IDLE:
        default:
            speed_ = 0.f;                             // утримуємо позицію
            break;
    }

    simTime_ += dt;   // модельний час останнього оновлення
}

// ------------------------------------------------------------
// run — тіло потоку фізики.
// ------------------------------------------------------------
void DronePhysics::run()
{
    ready_.store(true);

    while (!started_.load() && !stop_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    const float dt    = cfg_.physicsTimeStep;
    const float scale = (cfg_.timeScale > EPS) ? cfg_.timeScale : 1.0f;

    while (!stop_.load()) {
        if (auto cmd = cmdQueue_.drainLatest())
            current_ = *cmd;

        integrate(dt);

        std::this_thread::sleep_for(
            std::chrono::duration<float>(dt / scale));
    }
}

void DronePhysics::stop()
{
    stop_.store(true);
    cmdQueue_.shutdown();
}
