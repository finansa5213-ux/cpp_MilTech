#pragma once
// ============================================================
// AnalyticalSolver — реалізація IBallisticSolver.
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
