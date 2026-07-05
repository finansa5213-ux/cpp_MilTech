#pragma once
// ============================================================
// MissionProcessor — основний клас алгоритму місії.
//
// Патерн "Стратегія": loader / provider / solver — зовнішні,
// підмінюються через changeSolver().
// ============================================================

#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "drone/DroneStateMachine.h"
#include <memory>

class MissionProcessor {
private:
    // MissionProcessor тепер ВОЛОДІЄ компонентами через unique_ptr.
    std::unique_ptr<IConfigLoader>    loader_;
    std::unique_ptr<ITargetProvider>  provider_;
    std::unique_ptr<IBallisticSolver> solver_;

    int               currentIdx_   = 0;
    bool              initialized_  = false;
    int               skippedTargets_ = 0;  // лічильник пропущених через ERROR

    MissionConfig     cfg_;
    AmmoParams        ammo_;

    DroneStateMachine drone_;

    // Знайти індекс найближчої необробленої цілі від поточної позиції дрона.
    // Повертає індекс або -1 якщо більше цілей немає.
    int findNearestNextTarget() const;

public:
    // Власність над компонентами передається через std::move().
    MissionProcessor(std::unique_ptr<IConfigLoader>    loader,
                     std::unique_ptr<ITargetProvider>  provider,
                     std::unique_ptr<IBallisticSolver> solver);
    ~MissionProcessor() = default;

    bool      init(const char* configSource = nullptr);
    bool      hasNext() const;
    DropPoint step();
    void      reset();

    // Підміна солвера на льоту — приймаємо власність через unique_ptr.
    void      changeSolver(std::unique_ptr<IBallisticSolver> s);

    const MissionConfig&      config()          const { return cfg_;            }
    const AmmoParams&         ammo()            const { return ammo_;           }
    int                       currentIndex()    const { return currentIdx_;     }
    int                       skippedTargets()  const { return skippedTargets_; }
    const DroneStateMachine&  drone()           const { return drone_;          }
};