#pragma once
// ============================================================
// AnalyticalSolver — реалізація IBallisticSolver.
// Метод Кардано для часу польоту + ряд Тейлора до t^5
// для горизонтальної відстані. (Успадковано з ДЗ10;
// додатково заповнює DropPoint::carry.)
// ============================================================
#include "interfaces/IBallisticSolver.h"

class AnalyticalSolver : public IBallisticSolver {
private:
    bool  computeTimeOfFlight(const AmmoParams& ammo,
                              float V0, float Z0, float& t) const;
    float computeHorizontalDistance(const AmmoParams& ammo,
                                    float V0, float t) const;
public:
    AnalyticalSolver()  = default;
    ~AnalyticalSolver() override = default;

    DropPoint solve(const Coord&      dronePos,
                    const Target&     target,
                    float             altitude,
                    const AmmoParams& ammo,
                    float             attackSpeed) override;
};
