#pragma once
// ============================================================
// IConfigLoader — абстрактний інтерфейс завантажувача конфігурації.
// ============================================================
#include "Types.h"

class IConfigLoader {
public:
    virtual bool                 load()               = 0;
    virtual const MissionConfig& getConfig()    const = 0;
    virtual const AmmoParams&    getAmmoParams() const = 0;
    virtual ~IConfigLoader() {}
};