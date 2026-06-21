// ============================================================
// FileConfigLoader.cpp
// ДЗ8: char[] → std::string; unordered_map для пошуку ammo 
// за умови більше 1000 найменувань боєприпасів.
// ============================================================

#include "config/FileConfigLoader.h"
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <unordered_map>

using json = nlohmann::json;

FileConfigLoader::FileConfigLoader(const std::string& configPath,
                                   const std::string& ammoPath)
    : configPath_(configPath), ammoPath_(ammoPath)
{}

bool FileConfigLoader::loadConfigFile()
{
    std::ifstream in(configPath_);
    if (!in.is_open()) {
        std::cerr << "FileConfigLoader: не вдалось відкрити " << configPath_ << "\n";
        return false;
    }
    json j;
    try { in >> j; }
    catch (const json::parse_error& e) {
        std::cerr << "Помилка парсингу " << configPath_ << ": " << e.what() << "\n";
        return false;
    }

    try {
        const json& drone   = j.at("drone");
        cfg_.startPos.x     = drone.at("position").at("x").get<float>();
        cfg_.startPos.y     = drone.at("position").at("y").get<float>();
        cfg_.altitude       = drone.at("altitude").get<float>();
        cfg_.initialDir     = drone.at("initialDirection").get<float>();
        cfg_.attackSpeed    = drone.at("attackSpeed").get<float>();
        cfg_.accelPath      = drone.at("accelerationPath").get<float>();
        cfg_.angularSpeed   = drone.at("angularSpeed").get<float>();
        cfg_.turnThreshold  = drone.at("turnThreshold").get<float>();

        const json& sim     = j.at("simulation");
        cfg_.simTimeStep    = sim.at("timeStep").get<float>();
        cfg_.hitRadius      = sim.at("hitRadius").get<float>();

        cfg_.arrayTimeStep  = j.at("targetArrayTimeStep").get<float>();
        cfg_.ammoName       = j.at("ammo").get<std::string>();
        cfg_.maxSteps       = j.contains("maxSteps")
                              ? j.at("maxSteps").get<int>() : 10000;
    } catch (const json::exception& e) {
        std::cerr << "Помилка структури " << configPath_ << ": " << e.what() << "\n";
        return false;
    }

    if (cfg_.maxSteps <= 0) cfg_.maxSteps = 10000;
    return true;
}

bool FileConfigLoader::loadAmmoFile()
{
    std::ifstream in(ammoPath_);
    if (!in.is_open()) {
        std::cerr << "FileConfigLoader: не вдалось відкрити " << ammoPath_ << "\n";
        return false;
    }
    json j;
    try { in >> j; }
    catch (const json::parse_error& e) {
        std::cerr << "Помилка парсингу " << ammoPath_ << ": " << e.what() << "\n";
        return false;
    }

    if (!j.is_array() || j.empty()) {
        std::cerr << "Помилка: " << ammoPath_ << " має бути непорожнім масивом\n";
        return false;
    }

    // ДЗ8: unordered_map — пошук O(1) замість лінійного циклу з strcmp
    std::unordered_map<std::string, AmmoParams> table;
    try {
        for (const auto& item : j) {   // ДЗ8: range-based for
            AmmoParams ap;
            ap.name = item.at("name").get<std::string>();
            ap.mass = item.at("mass").get<float>();
            ap.drag = item.at("drag").get<float>();
            ap.lift = item.at("lift").get<float>();
            table[ap.name] = ap;
        }
    } catch (const json::exception& e) {
        std::cerr << "Помилка структури " << ammoPath_ << ": " << e.what() << "\n";
        return false;
    }

    auto it = table.find(cfg_.ammoName);
    if (it == table.end()) {
        std::cerr << "Помилка: боєприпас '" << cfg_.ammoName
                  << "' не знайдено в " << ammoPath_ << "\n";
        return false;
    }
    ammo_ = it->second;
    return true;
}

bool FileConfigLoader::load()
{
    if (!loadConfigFile()) return false;
    if (!loadAmmoFile())   return false;
    loaded_ = true;
    return true;
}
