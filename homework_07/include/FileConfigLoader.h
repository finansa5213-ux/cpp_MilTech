#pragma once
// ============================================================
// FileConfigLoader — реалізація IConfigLoader, що читає:
//   - config.json — параметри місії та назву обраного боєприпасу
//   - ammo.json   — таблицю всіх боєприпасів.
// ============================================================

#include "IConfigLoader.h"
#include <cstdio>

class FileConfigLoader : public IConfigLoader {
private:
    char          configPath_[256];
    char          ammoPath_[256];
    MissionConfig cfg_;
    AmmoParams    ammo_;
    bool          loaded_;

    bool loadConfigFile();
    bool loadAmmoFile();
public:
    FileConfigLoader(const char* configPath, const char* ammoPath);
    ~FileConfigLoader() override {}

    // --- IConfigLoader ---
    bool                 load()                      override;
    const MissionConfig& getConfig()       const     override { return cfg_; }
    const AmmoParams&    getAmmoParams()   const     override { return ammo_; }
};
