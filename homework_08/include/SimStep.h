// ============================================================
// SimStep.h — один крок записаної траєкторії симуляції.
//
// Формат продиктований чекером курсу: він програє траєкторію в 3D
// і перевіряє її на відповідність фізичним обмеженням. Валідація
// вимагає в кожному кроці: position, direction, state, targetIndex,
// dropPoint, aimPoint, predictedTarget.
// ============================================================
#ifndef SIM_STEP_H
#define SIM_STEP_H

#include "Types.h"
#include "drone/DroneStateMachine.h"

// ------------------------------------------------------------
// Код стану у нумерації чекера.
//
// Чекер очікує число з власного переліку:
//   0 STOPPED, 1 ACCELERATING, 2 DECELERATING, 3 TURNING, 4 MOVING
// Він не збігається з нашим enum class DroneState, тому потрібне
// явне відображення. DECELERATING (2) не використовується — у нашій
// моделі гальмування окремим станом не виділене.
// ------------------------------------------------------------
inline int checkerStateCode(DroneState s)
{
    switch (s) {
        case DroneState::ACCELERATING:    return 1;
        case DroneState::TURNING:         return 3;
        case DroneState::CRUISE:          return 4;
        case DroneState::ATTACK:          return 4;
        case DroneState::STOP:            return 0;
        case DroneState::ERROR:           return 0;
        case DroneState::NOT_INITIALIZED: return 0;
    }
    return 0;
}

// ------------------------------------------------------------
// SimStep — знімок стану дрона на одному тіку симуляції.
//
// dropPoint і aimPoint у нашій моделі збігаються: проміжних
// маршрутних точок немає, дрон летить прямо на точку скиду.
// Чекер вимагає обидва поля, тому пишемо однакові значення.
// ------------------------------------------------------------
struct SimStep {
    Coord position;          // де дрон зараз
    float direction = 0.f;   // курс, радіани
    int   state     = 0;     // код стану в нумерації чекера
    int   targetIndex = 0;   // яку ціль зараз атакує
    Coord dropPoint;         // розрахована точка скиду
    Coord aimPoint;          // куди дрон кермує (= dropPoint)
    Coord predictedTarget;   // де буде ціль у момент падіння
};

#endif  // SIM_STEP_H
