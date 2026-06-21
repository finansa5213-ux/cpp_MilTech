#pragma once
// ============================================================
// IConfigLoader — абстрактний інтерфейс завантажувача конфігурації.
// ============================================================

#include "Types.h"

class IConfigLoader {
public:
    // Виконати завантаження. Повертає true при успіху.
    // Поки не викликаний load() з true, getConfig()/getAmmoParams()
    // повертають дефолтні / порожні значення.
    virtual bool                load()                       = 0;

    // Отримати завантажений конфіг місії.
    // Повертає посилання на внутрішнє сховище — копіювати не треба.
    virtual const MissionConfig& getConfig()         const   = 0;

    // Отримати параметри обраного боєприпасу (того, що в cfg.ammoName).
    // Завантажувач сам шукає його в таблиці і повертає готову структуру.
    virtual const AmmoParams&    getAmmoParams()     const   = 0;

    virtual ~IConfigLoader() {}
};
