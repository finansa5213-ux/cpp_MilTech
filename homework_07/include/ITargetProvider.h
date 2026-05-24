#pragma once
// ============================================================
// ITargetProvider — абстрактний інтерфейс провайдера цілей.
// ============================================================

#include "Types.h"

class ITargetProvider {
public:
    // Скільки цілей доступно. Стабільно протягом життя об'єкта.
    virtual int    getTargetCount()       = 0;

    // Дані конкретної цілі. Індекс має бути в [0, getTargetCount()).
    // Реалізація сама вирішує, чи це поточна позиція, прогноз тощо.
    virtual Target getTarget(int index)   = 0;

    virtual ~ITargetProvider() {}
};
