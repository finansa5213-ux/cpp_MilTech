#pragma once
// ============================================================
// DroneStateMachine — стейт машина руху дрона.
// Замість enum DroneState + switch/case
// кожен стан тепер окремий клас, що реалізує інтерфейс
// IDroneState. Перехід між станами — повернення
// std::unique_ptr<IDroneState> з методу execute().
//
// Стани руху:
//   StateTurning       повертається до курсу на firePoint
//   StateAccelerating  розганяється (0 → attackSpeed)
//   StateCruise        крейсерська швидкість, прямий курс
// Термінальні стани:
//   StateAttack        дрон у firePoint, скид виконано
//   StateStop          місія завершена ззовні
//   StateError         помилка — причину див. у DroneContext::errorCode
// ============================================================

#include "Types.h"
#include <string>
#include <memory>

// ------------------------------------------------------------
// DroneErrorCode — причина переходу у стан помилки.
// ------------------------------------------------------------
enum class DroneErrorCode {
    NONE,              // помилки немає
    TIMEOUT,           // stepCount > maxSteps — дрон не долетів
    BALLISTICS_FAIL,   // solver повернув valid=false
    INVALID_TARGET,    // firePoint збігається з dronePos (dist < EPS)
    TURN_STALL         // кут не сходиться після maxSteps у TURNING
};

inline std::string droneErrorName(DroneErrorCode e) {
    switch (e) {
        case DroneErrorCode::NONE:            return "NONE";
        case DroneErrorCode::TIMEOUT:         return "TIMEOUT";
        case DroneErrorCode::BALLISTICS_FAIL: return "BALLISTICS_FAIL";
        case DroneErrorCode::INVALID_TARGET:  return "INVALID_TARGET";
        case DroneErrorCode::TURN_STALL:      return "TURN_STALL";
    }
    return "UNKNOWN";
}

// ------------------------------------------------------------
// DroneContext — спільні дані, що передаються між станами за
// посиланням. Містить мутабельний стан дрона + конфіг руху.
// ------------------------------------------------------------
struct DroneContext {
    // --- мутабельний стан ---
    Coord          pos;
    float          dir       = 0.f;
    float          speed     = 0.f;
    float          simTime   = 0.f;
    int            stepCount = 0;
    float          dt        = 0.f;   // крок поточного tick()

    DroneErrorCode errorCode = DroneErrorCode::NONE;
    std::string    errorMsg;

    // --- ціль ---
    Coord          firePoint;

    // --- параметри руху (конфіг) ---
    float attackSpeed   = 0.f;
    float accelPath     = 0.f;
    float angularSpeed  = 0.f;
    float turnThreshold = 0.f;
    float hitRadius     = 5.f;
    int   maxSteps      = 10000;

    // Кут [-π, π] від поточного курсу до firePoint.
    float angleToFirePoint() const {
        Coord delta = firePoint - pos;
        if (length(delta) < EPS) return 0.f;
        return normalizeAngle(std::atan2(delta.y, delta.x) - dir);
    }
};

// ------------------------------------------------------------
// IDroneState — базовий клас стану (патерн State).
// ------------------------------------------------------------
class IDroneState {
public:
    virtual ~IDroneState() = default;

    // Виконати логіку стану. Повертає наступний стан або nullptr,
    // якщо стан не змінився (головний цикл лишає поточний).
    virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;

    virtual const char* name() const = 0;

    // Класифікація термінальних станів (для головного циклу).
    virtual bool isAttack() const { return false; }
    virtual bool isStop()   const { return false; }
    virtual bool isError()  const { return false; }
    bool isTerminal() const { return isAttack() || isStop() || isError(); }
};

// ------------------------------------------------------------
// Активні стани
// ------------------------------------------------------------
class StateTurning : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "TURNING"; }
};

class StateAccelerating : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "ACCELERATING"; }
};

class StateCruise : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "CRUISE"; }
};

// ------------------------------------------------------------
// Термінальні стани — execute() нічого не робить
// ------------------------------------------------------------
class StateAttack : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext&) override { return nullptr; }
    const char* name() const override { return "ATTACK"; }
    bool isAttack() const override { return true; }
};

class StateStop : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext&) override { return nullptr; }
    const char* name() const override { return "STOP"; }
    bool isStop() const override { return true; }
};

class StateError : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext&) override { return nullptr; }
    const char* name() const override { return "ERROR"; }
    bool isError() const override { return true; }
};

// ------------------------------------------------------------
// DroneStateMachine — контекст-власник поточного стану.
// ------------------------------------------------------------
class DroneStateMachine {
public:
    // Ініціалізувати машину. firePoint — перша ціль руху.
    void init(const Coord& startPos,
              float        initialDir,
              float        attackSpeed,
              float        accelPath,
              float        angularSpeed,
              float        turnThreshold,
              float        hitRadius,
              int          maxSteps,
              const Coord& firePoint);

    // Передати нову ціль без скидання позиції / швидкості / курсу.
    void retarget(const Coord& newFirePoint);

    // Один крок симуляції на dt секунд.
    void tick(float dt);

    // --- читання стану ---
    const char*          stateName() const { return state_ ? state_->name() : "NONE"; }
    const DroneContext&  data()      const { return ctx_; }

    bool isAttack()   const { return state_ && state_->isAttack(); }
    bool isStop()     const { return state_ && state_->isStop();   }
    bool isError()    const { return state_ && state_->isError();  }
    bool isTerminal() const { return !state_ || state_->isTerminal(); }

    // --- ДЗ10: інтеграція з потоком фізики ---
    // Поточний режим у вигляді DroneMode (для команд і логу).
    DroneMode mode() const {
        if (!state_)            return DroneMode::IDLE;
        if (state_->isAttack()) return DroneMode::ATTACK;
        if (state_->isStop())   return DroneMode::STOP;
        if (state_->isError())  return DroneMode::ERROR;
        std::string n = state_->name();
        if (n == "TURNING")      return DroneMode::TURNING;
        if (n == "ACCELERATING") return DroneMode::ACCELERATING;
        if (n == "CRUISE")       return DroneMode::CRUISE;
        return DroneMode::IDLE;
    }

    // Засіяти кінематику з телеметрії фізики. Стейт-машина в ДЗ10
    // лише УХВАЛЮЄ рішення (режим), а інтегрує фізика; тому перед
    // кожним кроком рішення синхронізуємо позицію/курс/швидкість.
    void syncKinematics(const Coord& pos, float dir, float speed) {
        ctx_.pos   = pos;
        ctx_.dir   = dir;
        ctx_.speed = speed;
    }

    // Параметри сегмента руху (для команди фізиці).
    float angularSpeedParam() const { return ctx_.angularSpeed;  }
    float attackSpeedParam()  const { return ctx_.attackSpeed;   }
    float accelPathParam()    const { return ctx_.accelPath;     }
    float turnThresholdParam()const { return ctx_.turnThreshold; }
    float hitRadiusParam()    const { return ctx_.hitRadius;     }
    Coord firePointParam()    const { return ctx_.firePoint;     }

    // --- зовнішні переходи у термінальні стани ---
    void stop() { state_ = std::make_unique<StateStop>(); }

    void error(DroneErrorCode code, const std::string& msg = "") {
        ctx_.errorCode = code;
        ctx_.errorMsg  = msg;
        state_ = std::make_unique<StateError>();
    }

private:
    DroneContext                 ctx_;
    std::unique_ptr<IDroneState> state_;   // поточний стан (nullptr = не ініціалізовано)

    // Вибрати стартовий стан за поточним кутом/швидкістю.
    std::unique_ptr<IDroneState> chooseInitialState() const;
};
