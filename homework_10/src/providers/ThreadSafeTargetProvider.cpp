// ============================================================
// ThreadSafeTargetProvider.cpp (ДЗ10)
// ============================================================
#include "providers/ThreadSafeTargetProvider.h"
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string& filename,
                                                   float arrayTimeStep,
                                                   float timeScale)
    : arrayTimeStep_(arrayTimeStep > 0.f ? arrayTimeStep : 1.0f)
    , timeScale_(timeScale > 0.f ? timeScale : 1.0f)
    , filename_(filename)
{}

bool ThreadSafeTargetProvider::loadFromFile()
{
    std::ifstream f(filename_);
    if (!f.is_open()) {
        std::cerr << "ThreadSafeTargetProvider: не вдалось відкрити " << filename_ << "\n";
        return false;
    }

    json j;
    try { f >> j; }
    catch (const json::parse_error& e) {
        std::cerr << "Помилка парсингу " << filename_ << ": " << e.what() << "\n";
        return false;
    }

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

    // Початковий знімок (вузол 0), щоб getTarget() працював ще до start().
    nodeIndex_ = 0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        current_.assign(tracks_.size(), Target{});
        for (size_t i = 0; i < tracks_.size(); ++i) {
            const auto& tr = tracks_[i];
            int next = static_cast<int>((1) % tr.size());
            current_[i].pos      = tr[0];
            current_[i].velocity = (tr[next] - tr[0]) / arrayTimeStep_;
        }
    }
    return true;
}

// Перейти до наступного вузла траєкторії й оновити знімок.
void ThreadSafeTargetProvider::advanceNodes()
{
    if (tracks_.empty()) return;

    const int nS = static_cast<int>(tracks_[0].size());
    nodeIndex_ = (nodeIndex_ + 1) % nS;
    const int next = (nodeIndex_ + 1) % nS;

    std::lock_guard<std::mutex> lk(mtx_);
    for (size_t i = 0; i < tracks_.size(); ++i) {
        const auto& tr = tracks_[i];
        current_[i].pos      = tr[nodeIndex_];
        current_[i].velocity = (tr[next] - tr[nodeIndex_]) / arrayTimeStep_;
    }
}

void ThreadSafeTargetProvider::run()
{
    ready_.store(true);

    while (!started_.load() && !stop_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    while (!stop_.load()) {
        std::this_thread::sleep_for(
            std::chrono::duration<float>(arrayTimeStep_ / timeScale_));
        if (stop_.load()) break;
        advanceNodes();
    }
}

int ThreadSafeTargetProvider::getTargetCount()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<int>(current_.size());
}

Target ThreadSafeTargetProvider::getTarget(int index)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (index < 0 || index >= static_cast<int>(current_.size()))
        return Target{};
    return current_[index];   // копія
}
