#pragma once
// ============================================================
// MissionProcessor — основний клас алгоритму місії.
//
// Патерн "Стратегія": loader / provider / solver — зовнішні,
// підмінюються через changeSolver().
//
// Нове в ДЗ8: містить DroneStateMachine — явну стейт машину
// руху дрона. Метод step() більше не просто викликає solve():
// він запускає повний симуляційний цикл, у якому дрон реально
// летить від поточної позиції до firePoint через стани
// TURNING → ACCELERATING → CRUISE → ATTACK.
//
// Контракт:
//   init()    — завантажити конфіг, ініціалізувати стейт машину
//   hasNext() — чи є необроблені цілі
//   step()    — симулювати підліт дрона до наступної цілі,
//               повернути DropPoint з реальними координатами.
//               При помилці (ERROR) автоматично знаходить
//               найближчу необроблену ціль і переходить до неї.
//   reset()   — повернути ітератор і дрона на старт
//   changeSolver() — підміна балістичного солвера на льоту
// ============================================================

#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "drone/DroneStateMachine.h"
#include "SimStep.h"

#include <vector>

class MissionProcessor {
private:
    // Зовнішні компоненти (не володіємо)
    IConfigLoader*    loader_;
    ITargetProvider*  provider_;
    IBallisticSolver* solver_;

    int               currentIdx_   = 0;
    bool              initialized_  = false;
    int               skippedTargets_ = 0;  // лічильник пропущених через ERROR

    MissionConfig     cfg_;
    AmmoParams        ammo_;

    DroneStateMachine drone_;

    // Записана траєкторія: по одному запису на кожен тік симуляції.
    // Потрібна чекеру курсу, який програє політ і перевіряє його
    // на відповідність обмеженням швидкості, прискорення й повороту.
    std::vector<SimStep> trace_;

    // Знайти індекс найближчої необробленої цілі від поточної позиції дрона.
    // Повертає індекс або -1 якщо більше цілей немає.
    int findNearestNextTarget() const;

    // Записати поточний стан дрона в буфер траєкторії.
    void recordStep(const DropPoint& dp);

public:
    MissionProcessor(IConfigLoader*    loader,
                     ITargetProvider*  provider,
                     IBallisticSolver* solver);
    ~MissionProcessor() = default;

    bool      init(const char* configSource = nullptr);
    bool      hasNext() const;
    DropPoint step();
    void      reset();
    void      changeSolver(IBallisticSolver* s);

    const MissionConfig&      config()          const { return cfg_;            }
    const AmmoParams&         ammo()            const { return ammo_;           }
    int                       currentIndex()    const { return currentIdx_;     }
    int                       skippedTargets()  const { return skippedTargets_; }
    const DroneStateMachine&  drone()           const { return drone_;          }
    const std::vector<SimStep>& trace()         const { return trace_;          }
};