// ============================================================
// JsonTargetProvider.cpp
// ============================================================

#include "providers/JsonTargetProvider.h"
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <cmath>

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const std::string& filename,
                                       float arrayTimeStep)
    : arrayTimeStep_(arrayTimeStep), filename_(filename)
{}

bool JsonTargetProvider::loadFromFile()
{
    std::ifstream f(filename_);
    if (!f.is_open()) {
        std::cerr << "JsonTargetProvider: не вдалось відкрити " << filename_ << "\n";
        return false;
    }

    json j;
    try { f >> j; }
    catch (const json::parse_error& e) {
        std::cerr << "Помилка парсингу " << filename_ << ": " << e.what() << "\n";
        return false;
    }
    f.close();

    int nT = 0, nS = 0;
    try {
        nT = j.at("targetCount").get<int>();
        nS = j.at("timeSteps").get<int>();
    } catch (const json::exception& e) {
        std::cerr << "Помилка структури " << filename_ << ": " << e.what() << "\n";
        return false;
    }

    if (nT <= 0 || nS <= 0) {
        std::cerr << "Помилка: targetCount/timeSteps мають бути > 0\n";
        return false;
    }
    if (!j.contains("targets") || !j["targets"].is_array()
        || static_cast<int>(j["targets"].size()) != nT) {
        std::cerr << "Помилка: targets масив некоректний\n";
        return false;
    }

    std::vector<std::vector<Coord>> tmp(nT, std::vector<Coord>(nS));

    try {
        for (int i = 0; i < nT; ++i) {
            const json& positions = j["targets"][i].at("positions");
            if (!positions.is_array() || static_cast<int>(positions.size()) != nS) {
                std::cerr << "Помилка: ціль #" << i
                          << " positions.size() != " << nS << "\n";
                return false;
            }
            int k = 0;
            for (const auto& pt : positions) { 
                tmp[i][k].x = pt.at("x").get<float>();
                tmp[i][k].y = pt.at("y").get<float>();
                ++k;
            }
        }
    } catch (const json::exception& e) {
        std::cerr << "Помилка структури " << filename_ << ": " << e.what() << "\n";
        return false;
    }

    tracks_ = std::move(tmp);
    loaded_ = true;
    return true;
}

Coord JsonTargetProvider::interpolate(int targetIdx, float t) const
{
    const int   nS     = static_cast<int>(tracks_[targetIdx].size());
    const float period = arrayTimeStep_ * nS;

    float tt = std::fmod(t, period);
    if (tt < 0.f) tt += period;

    int   idx  = static_cast<int>(std::floor(tt / arrayTimeStep_)) % nS;
    int   next = (idx + 1) % nS;
    float frac = (tt - idx * arrayTimeStep_) / arrayTimeStep_;

    const Coord& a = tracks_[targetIdx][idx];
    const Coord& b = tracks_[targetIdx][next];
    return a + (b - a) * frac;
}

int JsonTargetProvider::getTargetCount()
{
    return static_cast<int>(tracks_.size());
}

Target JsonTargetProvider::getTarget(int index)
{
    Target out;
    if (!loaded_ || index < 0 || index >= static_cast<int>(tracks_.size()))
        return out;

    const float dt = 0.5f;
    Coord p1 = interpolate(index, 0.f);
    Coord p2 = interpolate(index, dt);

    out.pos      = p1;
    out.velocity = (p2 - p1) / dt;
    return out;
}
