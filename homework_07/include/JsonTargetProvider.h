#pragma once
// ============================================================
// JsonTargetProvider — реалізація ITargetProvider, що читає
// цілі з targets.json у форматі ДЗ3:
//   {
//     "targetCount": 5,
//     "timeSteps": 60,
//     "targets": [
//       { "positions": [ {"x":..,"y":..}, ... ] },
//       ...
//     ]
//   }
//
// Внутрішнє представлення: динамічний 2D-масив Coord** із
// рознесеними рядками (як було в ДЗ3). У зовнішній світ
// видається через getTarget(i) як Target з позицією в t=0
// та швидкістю, обчисленою кінцевими різницями (як у ДЗ3).
// ============================================================

#include "ITargetProvider.h"
#include <cstdio>

class JsonTargetProvider : public ITargetProvider {
private:
    // Динамічний 2D-масив: tracks_[targetIdx][timeStep].
    // Зберігаємо ВСЮ траєкторію (а не лише t=0), бо це потрібно
    // для коректного обчислення швидкості кінцевою різницею.
    Coord**     tracks_;
    int         targetCount_;
    int         timeSteps_;
    float       arrayTimeStep_;
    char        filename_[256];
    bool        loaded_;

    // Лінійна інтерполяція позиції цілі в момент часу t
    // (масив зациклений, як у ДЗ3).
    Coord interpolate(int targetIdx, float t) const;

    void freeTracks();

public:
    // arrayTimeStep потрібен для обчислення швидкості;
    // приходить з MissionConfig.
    JsonTargetProvider(const char* filename, float arrayTimeStep);
    ~JsonTargetProvider() override;

    // Власне завантаження — викликається з фабрики або зовні.
    bool loadFromFile();

    // --- ITargetProvider ---
    int    getTargetCount()       override;
    Target getTarget(int index)   override;
};
