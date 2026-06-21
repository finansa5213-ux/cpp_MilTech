#pragma once
// ============================================================
// FileConfigLoader — реалізація IConfigLoader.
// Читає config.json + ammo.json.
// ============================================================
#include "interfaces/IConfigLoader.h"
#include <string>

class FileConfigLoader : public IConfigLoader {
private:
    std::string   configPath_;
    std::string   ammoPath_;
    MissionConfig cfg_;
    AmmoParams    ammo_;
    bool          loaded_ = false;

    bool loadConfigFile();
    bool loadAmmoFile();

public:
    FileConfigLoader(const std::string& configPath,
                     const std::string& ammoPath);
    ~FileConfigLoader() override = default;

    bool                 load()              override;
    const MissionConfig& getConfig()   const override { return cfg_;  }
    const AmmoParams&    getAmmoParams()const override { return ammo_; }
};
