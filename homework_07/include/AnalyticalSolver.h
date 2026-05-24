#pragma once
// ============================================================
// AnalyticalSolver — реалізація IBallisticSolver.
//
// Використовує аналітичні формули з ДЗ1:
//   1) computeTimeOfFlight  — час падіння через метод Кардано
//      (кубічне рівняння на t).
//   2) computeHorizontalDistance — горизонтальна дистанція
//      h(t) як ряд до п'ятого степеня по t.
//
// На основі цих двох обчислюється firePoint = target_pred - n*h,
// де n — одиничний вектор від дрона до прогнозованого положення цілі.
//
// "Аналітичне" — у тому сенсі, що тут жодного числового інтегрування
// чи таблиць. Чисто формули. Швидко і детерміновано.
// ============================================================

#include "IBallisticSolver.h"

class AnalyticalSolver : public IBallisticSolver {
private:
    // Кардано: повертає true і записує t у вихід при успіху.
    bool  computeTimeOfFlight(const AmmoParams& ammo,
                              float V0, float Z0, float& t) const;
    // Горизонтальна дистанція як ряд по t.
    float computeHorizontalDistance(const AmmoParams& ammo,
                                    float V0, float t) const;
public:
    AnalyticalSolver() {}
    ~AnalyticalSolver() override {}

    DropPoint solve(const Coord& dronePos,
                    const Target& target,
                    float altitude,
                    const AmmoParams& ammo,
                    float attackSpeed) override;
};
