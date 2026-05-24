// ============================================================
// FileConfigLoader.cpp — імплементація.
// Логіка читання повністю взята з ДЗ3 (функції loadConfig + loadAmmo),
// але інкапсульована й розв'язує ammoName → AmmoParams автоматично.
// ============================================================

#include "FileConfigLoader.h"
#include "json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

FileConfigLoader::FileConfigLoader(const char* configPath, const char* ammoPath)
    : loaded_(false)
{
    strncpy(configPath_, configPath, sizeof(configPath_) - 1);
    configPath_[sizeof(configPath_) - 1] = '\0';
    strncpy(ammoPath_, ammoPath, sizeof(ammoPath_) - 1);
    ammoPath_[sizeof(ammoPath_) - 1] = '\0';
}

bool FileConfigLoader::loadConfigFile() {
    std::ifstream in(configPath_);
    if (!in.is_open()) {
        std::cerr << "FileConfigLoader: не вдалось відкрити " << configPath_ << std::endl;
        return false;
    }
    json j;
    try { in >> j; }
    catch (json::parse_error& e) {
        std::cerr << "Помилка парсингу " << configPath_ << ": " << e.what() << std::endl;
        return false;
    }
    in.close();

    try {
        const json& drone = j.at("drone");
        cfg_.startPos.x      = drone.at("position").at("x").get<float>();
        cfg_.startPos.y      = drone.at("position").at("y").get<float>();
        cfg_.altitude        = drone.at("altitude").get<float>();
        cfg_.initialDir      = drone.at("initialDirection").get<float>();
        cfg_.attackSpeed     = drone.at("attackSpeed").get<float>();
        cfg_.accelPath       = drone.at("accelerationPath").get<float>();
        cfg_.angularSpeed    = drone.at("angularSpeed").get<float>();
        cfg_.turnThreshold   = drone.at("turnThreshold").get<float>();

        const json& sim = j.at("simulation");
        cfg_.simTimeStep     = sim.at("timeStep").get<float>();
        cfg_.hitRadius       = sim.at("hitRadius").get<float>();

        cfg_.arrayTimeStep   = j.at("targetArrayTimeStep").get<float>();

        const char* ammoSrc  = j.at("ammo").get<std::string>().c_str();
        strncpy(cfg_.ammoName, ammoSrc, sizeof(cfg_.ammoName) - 1);
        cfg_.ammoName[sizeof(cfg_.ammoName) - 1] = '\0';

        cfg_.maxSteps = j.contains("maxSteps")
                        ? j.at("maxSteps").get<int>() : 10000;
    } catch (json::exception& e) {
        std::cerr << "Помилка структури " << configPath_ << ": " << e.what() << std::endl;
        return false;
    }
    if (cfg_.maxSteps <= 0) cfg_.maxSteps = 10000;
    return true;
}

bool FileConfigLoader::loadAmmoFile() {
    std::ifstream in(ammoPath_);
    if (!in.is_open()) {
        std::cerr << "FileConfigLoader: не вдалось відкрити " << ammoPath_ << std::endl;
        return false;
    }
    json j;
    try { in >> j; }
    catch (json::parse_error& e) {
        std::cerr << "Помилка парсингу " << ammoPath_ << ": " << e.what() << std::endl;
        return false;
    }
    in.close();

    if (!j.is_array() || j.empty()) {
        std::cerr << "Помилка: " << ammoPath_ << " має бути непорожнім масивом" << std::endl;
        return false;
    }

    // Шукаємо запис з ім'ям, що збігається з cfg_.ammoName.
    // Не зберігаємо всю таблицю — клієнту потрібен лише ОБРАНИЙ боєприпас.
    bool found = false;
    try {
        for (size_t i = 0; i < j.size(); i++) {
            const json& item = j[i];
            const std::string nm = item.at("name").get<std::string>();
            if (strcmp(nm.c_str(), cfg_.ammoName) == 0) {
                strncpy(ammo_.name, nm.c_str(), sizeof(ammo_.name) - 1);
                ammo_.name[sizeof(ammo_.name) - 1] = '\0';
                ammo_.mass = item.at("mass").get<float>();
                ammo_.drag = item.at("drag").get<float>();
                ammo_.lift = item.at("lift").get<float>();
                found = true;
                break;
            }
        }
    } catch (json::exception& e) {
        std::cerr << "Помилка структури " << ammoPath_ << ": " << e.what() << std::endl;
        return false;
    }

    if (!found) {
        std::cerr << "Помилка: боєприпас '" << cfg_.ammoName
                  << "' не знайдено в " << ammoPath_ << std::endl;
        return false;
    }
    return true;
}

bool FileConfigLoader::load() {
    if (!loadConfigFile()) return false;
    if (!loadAmmoFile())   return false;
    loaded_ = true;
    return true;
}
