#pragma once

#include <filesystem>

#include "underground_world/scenario.hpp"

namespace underground_world {

Scenario load_scenario(const std::filesystem::path& path);

}  // namespace underground_world
