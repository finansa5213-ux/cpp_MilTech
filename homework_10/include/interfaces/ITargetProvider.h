#pragma once
// ============================================================
// ITargetProvider — абстрактний інтерфейс провайдера цілей.
// ============================================================
#include "Types.h"

class ITargetProvider {
public:
    virtual int    getTargetCount() = 0;
    virtual Target getTarget(int index) = 0;
    virtual ~ITargetProvider() {}
};