// ============================================================
// DroneStateMachine.cpp — реалізація переходів (патерн State).
// Кожен колишній case з switch тепер метод execute() окремого
// класу стану. Перехід — return std::make_unique<NextState>().
// ============================================================

#include "drone/DroneStateMachine.h"
#include <cmath>
#include <algorithm>

// ============================================================
//  StateTurning
//  Дрон повертається до курсу на firePoint з кутовою швидкістю
//  angularSpeed (рад/с). Під час повороту повзе повільно
//  (10% attackSpeed). Перехід → ACCELERATING коли кут вписується
//  у turnThreshold.
// ============================================================
std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx)
{
    // Ціль збіглась з позицією дрона — курс невизначений
    if (length(ctx.firePoint - ctx.pos) < EPS) {
        ctx.errorCode = DroneErrorCode::INVALID_TARGET;
        ctx.errorMsg  = "firePoint coincides with dronePos during TURNING";
        return std::make_unique<StateError>();
    }

    float angle   = ctx.angleToFirePoint();
    float maxTurn = ctx.angularSpeed * ctx.dt;

    if (std::fabs(angle) <= maxTurn)
        ctx.dir += angle;
    else
        ctx.dir += std::copysign(maxTurn, angle);

    ctx.dir = normalizeAngle(ctx.dir);

    // Повзем повільно, щоб не стояти на місці
    ctx.speed  = ctx.attackSpeed * 0.1f;
    ctx.pos.x += std::cos(ctx.dir) * ctx.speed * ctx.dt;
    ctx.pos.y += std::sin(ctx.dir) * ctx.speed * ctx.dt;

    // Курс вирівнявся → ACCELERATING
    if (std::fabs(ctx.angleToFirePoint()) <= ctx.turnThreshold)
        return std::make_unique<StateAccelerating>();

    return nullptr;   // лишаємось у TURNING
}

// ============================================================
//  StateAccelerating
//  Лінійний розгін: speed = attackSpeed * (1 - dist/accelPath).
//  → CRUISE коли speed >= 95% attackSpeed
//  → ATTACK коли dist <= hitRadius
//  → TURNING якщо кут знову перевищив поріг
// ============================================================
std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx)
{
    if (std::fabs(ctx.angleToFirePoint()) > ctx.turnThreshold)
        return std::make_unique<StateTurning>();

    float dist = distance(ctx.pos, ctx.firePoint);
    if (dist <= ctx.hitRadius)
        return std::make_unique<StateAttack>();

    if (ctx.accelPath > EPS) {
        float ratio = std::max(0.f, std::min(1.f, 1.f - dist / ctx.accelPath));
        ctx.speed   = std::max(ctx.attackSpeed * 0.05f, ctx.attackSpeed * ratio);
    } else {
        ctx.speed = ctx.attackSpeed;
    }

    ctx.pos.x += std::cos(ctx.dir) * ctx.speed * ctx.dt;
    ctx.pos.y += std::sin(ctx.dir) * ctx.speed * ctx.dt;

    if (ctx.speed >= ctx.attackSpeed * 0.95f) {
        ctx.speed = ctx.attackSpeed;
        return std::make_unique<StateCruise>();
    }

    return nullptr;   // лишаємось у ACCELERATING
}

// ============================================================
//  StateCruise
//  Постійна крейсерська швидкість, прямий курс.
//  → ATTACK коли dist <= hitRadius
//  → TURNING якщо кут перевищив поріг
// ============================================================
std::unique_ptr<IDroneState> StateCruise::execute(DroneContext& ctx)
{
    if (std::fabs(ctx.angleToFirePoint()) > ctx.turnThreshold)
        return std::make_unique<StateTurning>();

    float dist = distance(ctx.pos, ctx.firePoint);
    if (dist <= ctx.hitRadius)
        return std::make_unique<StateAttack>();

    ctx.speed  = ctx.attackSpeed;
    ctx.pos.x += std::cos(ctx.dir) * ctx.speed * ctx.dt;
    ctx.pos.y += std::sin(ctx.dir) * ctx.speed * ctx.dt;

    return nullptr;   // лишаємось у CRUISE
}

// ============================================================
//  DroneStateMachine
// ============================================================
std::unique_ptr<IDroneState> DroneStateMachine::chooseInitialState() const
{
    if (std::fabs(ctx_.angleToFirePoint()) > ctx_.turnThreshold)
        return std::make_unique<StateTurning>();
    if (ctx_.speed < ctx_.attackSpeed * 0.95f)
        return std::make_unique<StateAccelerating>();
    return std::make_unique<StateCruise>();
}

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
    ctx_ = DroneContext{};   // повний скид
    ctx_.attackSpeed   = attackSpeed;
    ctx_.accelPath     = accelPath;
    ctx_.angularSpeed  = angularSpeed;
    ctx_.turnThreshold = turnThreshold;
    ctx_.hitRadius     = hitRadius;
    ctx_.maxSteps      = maxSteps;
    ctx_.firePoint     = firePoint;

    ctx_.pos   = startPos;
    ctx_.dir   = initialDir;
    ctx_.speed = 0.f;

    // Перший стан: TURNING або ACCELERATING (швидкість = 0)
    state_ = (std::fabs(ctx_.angleToFirePoint()) > turnThreshold)
                 ? std::unique_ptr<IDroneState>(std::make_unique<StateTurning>())
                 : std::unique_ptr<IDroneState>(std::make_unique<StateAccelerating>());
}

void DroneStateMachine::retarget(const Coord& newFirePoint)
{
    ctx_.firePoint = newFirePoint;

    // Не ініціалізовано або термінальна помилка/стоп — не торкаємось
    if (!state_ || state_->isStop() || state_->isError()) return;

    // Скидаємо лічильник, час і код помилки для нової цілі,
    // але зберігаємо pos / dir / speed — дрон продовжує рух.
    ctx_.stepCount = 0;
    ctx_.simTime   = 0.f;
    ctx_.errorCode = DroneErrorCode::NONE;
    ctx_.errorMsg.clear();

    state_ = chooseInitialState();
}

void DroneStateMachine::tick(float dt)
{
    // Термінальні / не ініціалізовані — нічого не робимо
    if (!state_ || state_->isTerminal()) return;

    ++ctx_.stepCount;
    ctx_.simTime += dt;
    ctx_.dt       = dt;

    if (ctx_.stepCount > ctx_.maxSteps) {
        error(DroneErrorCode::TIMEOUT,
              "stepCount=" + std::to_string(ctx_.stepCount)
              + " > maxSteps=" + std::to_string(ctx_.maxSteps));
        return;
    }

    // Патерн State: делегуємо логіку поточному стану.
    auto next = state_->execute(ctx_);
    if (next) state_ = std::move(next);
}
