#pragma once
// ============================================================
// DroneStateMachine — явна стейт машина руху дрона.
//
// Стани:
//   NOT_INITIALIZED  до init()
//   TURNING          повертається до курсу на firePoint
//   ACCELERATING     розганяється (0 → attackSpeed)
//   CRUISE           крейсерська швидкість, прямий курс
//   ATTACK           дрон у firePoint, скид виконано
//   STOP             місія завершена ззовні
//   ERROR            помилка — причину дивись у DroneStateData::errorCode
// ============================================================

#include "Types.h"
#include <string>

// ------------------------------------------------------------
// Перелік станів
// ------------------------------------------------------------
enum class DroneState {
    NOT_INITIALIZED,
    TURNING,
    ACCELERATING,
    CRUISE,
    ATTACK,
    STOP,
    ERROR
};

// ------------------------------------------------------------
// DroneErrorCode — причина переходу у стан ERROR.
// Дозволяє MissionProcessor реагувати по-різному залежно від
// типу збою, а не просто фіксувати факт помилки.
// ------------------------------------------------------------
enum class DroneErrorCode {
    NONE,              // помилки немає (нормальний стан)
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

inline std::string droneStateName(DroneState s) {
    switch (s) {
        case DroneState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case DroneState::TURNING:         return "TURNING";
        case DroneState::ACCELERATING:    return "ACCELERATING";
        case DroneState::CRUISE:          return "CRUISE";
        case DroneState::ATTACK:          return "ATTACK";
        case DroneState::STOP:            return "STOP";
        case DroneState::ERROR:           return "ERROR";
    }
    return "UNKNOWN";
}

// ------------------------------------------------------------
// DroneStateData — весь мутабельний стан дрона.
// Відокремлений від машини, щоб зовнішній код (MissionProcessor,
// логер, тести) міг читати дані без доступу до приватних методів.
// ------------------------------------------------------------
struct DroneStateData {
    Coord          pos;
    float          dir        = 0.f;
    float          speed      = 0.f;
    DroneState     state      = DroneState::NOT_INITIALIZED;
    float          simTime    = 0.f;
    int            stepCount  = 0;
    DroneErrorCode errorCode  = DroneErrorCode::NONE;  // причина ERROR
    std::string    errorMsg;                            // деталі для логу
};

// ------------------------------------------------------------
// DroneStateMachine
// ------------------------------------------------------------
class DroneStateMachine {
public:
    // Ініціалізувати машину. firePoint — перша ціль руху.
    // Після init() стан одразу TURNING або ACCELERATING.
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
    // Викликається MissionProcessor при переході до наступної цілі.
    void retarget(const Coord& newFirePoint);

    // Один крок симуляції на dt секунд.
    // Повертає поточний стан після переходу.
    DroneState tick(float dt);

    // Читання стану
    DroneState             state() const { return data_.state; }
    const DroneStateData&  data()  const { return data_;       }

    // Зовнішні переходи у термінальні стани
    void stop() { data_.state = DroneState::STOP; }

    // Перейти в ERROR з явним кодом причини
    void error(DroneErrorCode code, const std::string& msg = "") {
        data_.errorCode = code;
        data_.errorMsg  = msg;
        data_.state     = DroneState::ERROR;
    }

private:
    DroneStateData data_;

    // Конфігураційні параметри (копія — машина автономна)
    float attackSpeed_   = 0.f;
    float accelPath_     = 0.f;
    float angularSpeed_  = 0.f;
    float turnThreshold_ = 0.f;
    float hitRadius_     = 5.f;
    int   maxSteps_      = 10000;

    Coord firePoint_;

    // Кут від поточного курсу до firePoint ([-π, π])
    float angleToFirePoint() const;

    // Обробники кожного активного стану
    void handleTurning(float dt);
    void handleAccelerating(float dt);
    void handleCruise(float dt);
};