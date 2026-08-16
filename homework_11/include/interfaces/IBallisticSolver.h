#pragma once
// ============================================================
// IBallisticSolver — абстрактний інтерфейс балістичного солвера.
// Патерн "Стратегія" (успадковано з ДЗ8-ДЗ10).
// ============================================================
#include "Types.h"

class IBallisticSolver {
public:
    virtual DropPoint solve(const Coord&      dronePos,
                            const Target&     target,
                            float             altitude,
                            const AmmoParams& ammo,
                            float             attackSpeed) = 0;
    virtual ~IBallisticSolver() {}
};
