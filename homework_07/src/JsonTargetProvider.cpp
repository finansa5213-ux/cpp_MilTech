// ============================================================
// JsonTargetProvider.cpp — імплементація.
// ============================================================

#include "JsonTargetProvider.h"
#include "json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const char* filename, float arrayTimeStep)
    : tracks_(nullptr), targetCount_(0), timeSteps_(0),
      arrayTimeStep_(arrayTimeStep), loaded_(false)
{
    strncpy(filename_, filename, sizeof(filename_) - 1);
    filename_[sizeof(filename_) - 1] = '\0';
}

JsonTargetProvider::~JsonTargetProvider() {
    freeTracks();
}

void JsonTargetProvider::freeTracks() {
    if (tracks_ == nullptr) return;
    for (int i = 0; i < targetCount_; i++) {
        delete[] tracks_[i];
        tracks_[i] = nullptr;
    }
    delete[] tracks_;
    tracks_ = nullptr;
}

bool JsonTargetProvider::loadFromFile() {
    std::ifstream f(filename_);
    if (!f.is_open()) {
        std::cerr << "JsonTargetProvider: не вдалось відкрити " << filename_ << std::endl;
        return false;
    }

    json j;
    try { f >> j; }
    catch (json::parse_error& e) {
        std::cerr << "Помилка парсингу " << filename_ << ": " << e.what() << std::endl;
        return false;
    }
    f.close();

    int nT = 0, nS = 0;
    try {
        nT = j.at("targetCount").get<int>();
        nS = j.at("timeSteps").get<int>();
    } catch (json::exception& e) {
        std::cerr << "Помилка структури " << filename_ << ": " << e.what() << std::endl;
        return false;
    }

    if (nT <= 0 || nS <= 0) {
        std::cerr << "Помилка: targetCount/timeSteps мають бути додатними" << std::endl;
        return false;
    }

    if (!j.contains("targets") || !j["targets"].is_array()
        || (int)j["targets"].size() != nT) {
        std::cerr << "Помилка: targets не масив або довжина не = " << nT << std::endl;
        return false;
    }

    // Виділення 2D-масиву
    Coord** t = new Coord*[nT];
    for (int i = 0; i < nT; i++) t[i] = new Coord[nS];

    try {
        for (int i = 0; i < nT; i++) {
            const json& positions = j["targets"][i].at("positions");
            if (!positions.is_array() || (int)positions.size() != nS) {
                std::cerr << "Помилка: ціль #" << i
                          << " має positions довжиною != " << nS << std::endl;
                for (int k = 0; k < nT; k++) delete[] t[k];
                delete[] t;
                return false;
            }
            for (int k = 0; k < nS; k++) {
                t[i][k].x = positions[k].at("x").get<float>();
                t[i][k].y = positions[k].at("y").get<float>();
            }
        }
    } catch (json::exception& e) {
        std::cerr << "Помилка структури " << filename_ << ": " << e.what() << std::endl;
        for (int k = 0; k < nT; k++) delete[] t[k];
        delete[] t;
        return false;
    }

    // Старі дані звільняємо лише після успіху — щоб у разі помилки
    // об'єкт залишався у консистентному стані.
    freeTracks();
    tracks_      = t;
    targetCount_ = nT;
    timeSteps_   = nS;
    loaded_      = true;
    return true;
}

Coord JsonTargetProvider::interpolate(int targetIdx, float t) const {
    const float period = arrayTimeStep_ * timeSteps_;
    float tt = fmod(t, period);
    if (tt < 0) tt += period;

    int idx   = (int)floor(tt / arrayTimeStep_) % timeSteps_;
    int next  = (idx + 1) % timeSteps_;
    float frac = (tt - idx * arrayTimeStep_) / arrayTimeStep_;

    const Coord& a = tracks_[targetIdx][idx];
    const Coord& b = tracks_[targetIdx][next];
    return a + (b - a) * frac;
}

int JsonTargetProvider::getTargetCount() {
    return targetCount_;
}

Target JsonTargetProvider::getTarget(int index) {
    Target out;
    if (!loaded_ || index < 0 || index >= targetCount_) return out;

    // Позиція в t=0 і швидкість через скінченну різницю
    // (як у computeTargetVelocity з ДЗ3).
    const float dt = 0.5f;
    Coord p1 = interpolate(index, 0.f);
    Coord p2 = interpolate(index, dt);

    out.pos      = p1;
    out.velocity = (p2 - p1) / dt;
    return out;
}
