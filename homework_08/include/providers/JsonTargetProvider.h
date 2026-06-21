#pragma once
// ============================================================
// JsonTargetProvider — реалізація ITargetProvider.
// Читає targets.json. ДЗ8: Coord** → vector<vector<Coord>>.
// ============================================================
#include "interfaces/ITargetProvider.h"
#include <vector>
#include <string>

class JsonTargetProvider : public ITargetProvider {
private:
    std::vector<std::vector<Coord>> tracks_;   // [targetIdx][timeStep]
    float       arrayTimeStep_;
    std::string filename_;
    bool        loaded_ = false;

    Coord interpolate(int targetIdx, float t) const;

public:
    JsonTargetProvider(const std::string& filename, float arrayTimeStep);
    ~JsonTargetProvider() override = default;

    bool loadFromFile();

    int    getTargetCount() override;
    Target getTarget(int index) override;
};