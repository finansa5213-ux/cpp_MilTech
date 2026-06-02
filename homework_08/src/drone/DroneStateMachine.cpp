// ============================================================
// DroneStateMachine.cpp — реалізація переходів стейт машини.
// ============================================================

#include "drone/DroneStateMachine.h"
#include <cmath>
#include <algorithm>

// ------------------------------------------------------------
// init
// ------------------------------------------------------------
void DroneStateMachine::init(const Coord& startPos,
                              float        initialDir,
                              float        attackSpeed,
                              float        accelPath,
                              float        angularSpeed,
                              float        turnThreshold,
                              float        hitRadius,
                              int          maxSteps,
                              const Coord& firePoint)
{
    attackSpeed_   = attackSpeed;
    accelPath_     = accelPath;
    angularSpeed_  = angularSpeed;
    turnThreshold_ = turnThreshold;
    hitRadius_     = hitRadius;
    maxSteps_      = maxSteps;
    firePoint_     = firePoint;

    data_.pos       = startPos;
    data_.dir       = initialDir;
    data_.speed     = 0.f;
    data_.simTime   = 0.f;
    data_.stepCount = 0;
    data_.errorCode = DroneErrorCode::NONE;
    data_.errorMsg.clear();

    // Перший перехід залежить від початкового кута до firePoint
    float angle  = angleToFirePoint();
    data_.state  = (std::fabs(angle) > turnThreshold_)
                   ? DroneState::TURNING
                   : DroneState::ACCELERATING;
}

// ------------------------------------------------------------
// retarget
// ------------------------------------------------------------
void DroneStateMachine::retarget(const Coord& newFirePoint)
{
    firePoint_ = newFirePoint;

    // Термінальні та неініціалізований стан — не торкаємось
    if (data_.state == DroneState::STOP  ||
        data_.state == DroneState::ERROR ||
        data_.state == DroneState::NOT_INITIALIZED) return;

    // Скидаємо лічильник, час і код помилки для нової цілі,
    // але зберігаємо pos / dir / speed — дрон продовжує рух
    data_.stepCount = 0;
    data_.simTime   = 0.f;
    data_.errorCode = DroneErrorCode::NONE;
    data_.errorMsg.clear();

    float angle = angleToFirePoint();
    if (std::fabs(angle) > turnThreshold_)
        data_.state = DroneState::TURNING;
    else if (data_.speed < attackSpeed_ * 0.95f)
        data_.state = DroneState::ACCELERATING;
    else
        data_.state = DroneState::CRUISE;
}

// ------------------------------------------------------------
// tick — диспетчер одного кроку симуляції
// ------------------------------------------------------------
DroneState DroneStateMachine::tick(float dt)
{
    // Термінальні та NOT_INITIALIZED — нічого не робимо
    if (data_.state == DroneState::ATTACK  ||
        data_.state == DroneState::STOP    ||
        data_.state == DroneState::ERROR   ||
        data_.state == DroneState::NOT_INITIALIZED)
        return data_.state;

    ++data_.stepCount;
    data_.simTime += dt;

    if (data_.stepCount > maxSteps_) {
        error(DroneErrorCode::TIMEOUT,
              "stepCount=" + std::to_string(data_.stepCount)
              + " > maxSteps=" + std::to_string(maxSteps_));
        return data_.state;
    }

    switch (data_.state) {
        case DroneState::TURNING:      handleTurning(dt);      break;
        case DroneState::ACCELERATING: handleAccelerating(dt); break;
        case DroneState::CRUISE:       handleCruise(dt);       break;
        default: break;
    }

    return data_.state;
}

// ------------------------------------------------------------
// angleToFirePoint — повертає кут [-π, π] між поточним курсом
// і напрямком на firePoint
// ------------------------------------------------------------
float DroneStateMachine::angleToFirePoint() const
{
    Coord delta = firePoint_ - data_.pos;
    if (length(delta) < EPS) return 0.f;
    float target = std::atan2(delta.y, delta.x);
    return normalizeAngle(target - data_.dir);
}

// ------------------------------------------------------------
// handleTurning
// Дрон повертається до курсу на firePoint з кутовою швидкістю
// angularSpeed_ (рад/с). Під час повороту рухається повільно
// (10% attackSpeed). Перехід → ACCELERATING коли кут вписується
// у turnThreshold_.
// ------------------------------------------------------------
void DroneStateMachine::handleTurning(float dt)
{
    // Якщо ціль збіглась з позицією дрона — неможливо визначити курс
    if (length(firePoint_ - data_.pos) < EPS) {
        error(DroneErrorCode::INVALID_TARGET,
              "firePoint coincides with dronePos during TURNING");
        return;
    }

    float angle   = angleToFirePoint();
    float maxTurn = angularSpeed_ * dt;

    // Повертаємось на maxTurn, або точно виходимо на курс
    if (std::fabs(angle) <= maxTurn)
        data_.dir += angle;
    else
        data_.dir += std::copysign(maxTurn, angle);

    data_.dir = normalizeAngle(data_.dir);

    // Повзем повільно, щоб не стояти на місці
    data_.speed  = attackSpeed_ * 0.1f;
    data_.pos.x += std::cos(data_.dir) * data_.speed * dt;
    data_.pos.y += std::sin(data_.dir) * data_.speed * dt;

    // Перехід: курс вирівнявся → ACCELERATING
    if (std::fabs(angleToFirePoint()) <= turnThreshold_)
        data_.state = DroneState::ACCELERATING;
}

// ------------------------------------------------------------
// handleAccelerating
// Лінійний розгін: speed = attackSpeed * (1 - dist/accelPath).
// Перехід → CRUISE коли speed >= 95% attackSpeed.
// Перехід → ATTACK коли dist <= hitRadius.
// Перехід → TURNING якщо кут знову перевищив поріг.
// ------------------------------------------------------------
void DroneStateMachine::handleAccelerating(float dt)
{
    // Якщо кут збився — спочатку повернутись
    if (std::fabs(angleToFirePoint()) > turnThreshold_) {
        data_.state = DroneState::TURNING;
        return;
    }

    float dist = distance(data_.pos, firePoint_);

    // Перевіряємо досягнення точки скиду
    if (dist <= hitRadius_) {
        data_.state = DroneState::ATTACK;
        return;
    }

    // Лінійний розгін відносно accelPath
    if (accelPath_ > EPS) {
        float ratio  = std::max(0.f, std::min(1.f, 1.f - dist / accelPath_));
        data_.speed  = std::max(attackSpeed_ * 0.05f, attackSpeed_ * ratio);
    } else {
        data_.speed  = attackSpeed_;
    }

    data_.pos.x += std::cos(data_.dir) * data_.speed * dt;
    data_.pos.y += std::sin(data_.dir) * data_.speed * dt;

    // Перехід → CRUISE
    if (data_.speed >= attackSpeed_ * 0.95f) {
        data_.speed = attackSpeed_;
        data_.state = DroneState::CRUISE;
    }
}

// ------------------------------------------------------------
// handleCruise
// Постійна крейсерська швидкість. Перехід → ATTACK коли
// дистанція до firePoint <= hitRadius.
// Перехід → TURNING якщо кут перевищив поріг.
// ------------------------------------------------------------
void DroneStateMachine::handleCruise(float dt)
{
    // Якщо кут збився — повернутись
    if (std::fabs(angleToFirePoint()) > turnThreshold_) {
        data_.state = DroneState::TURNING;
        return;
    }

    float dist = distance(data_.pos, firePoint_);

    if (dist <= hitRadius_) {
        data_.state = DroneState::ATTACK;
        return;
    }

    data_.speed  = attackSpeed_;
    data_.pos.x += std::cos(data_.dir) * data_.speed * dt;
    data_.pos.y += std::sin(data_.dir) * data_.speed * dt;
}
