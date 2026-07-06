#pragma once
// ============================================================
// ThreadSafeTargetProvider — провайдер цілей у власному потоці.
// ============================================================
#include "interfaces/ITargetProvider.h"
#include <vector>
#include <string>
#include <atomic>
#include <mutex>

class ThreadSafeTargetProvider : public ITargetProvider {
public:
    ThreadSafeTargetProvider(const std::string& filename, float arrayTimeStep,
                             float timeScale = 1.0f);
    ~ThreadSafeTargetProvider() override = default;

    bool loadFromFile();        // викликати ДО запуску потоку

    // --- життєвий цикл потоку ---
    void run();                 // тіло потоку
    bool isThreadReady() const { return ready_.load(); }
    void start()               { started_.store(true); }
    void stop()                { stop_.store(true); }

    // --- ITargetProvider (копії під м'ютексом) ---
    int    getTargetCount() override;
    Target getTarget(int index) override;

private:
    void advanceNodes();        // один крок: перейти до наступного вузла

    std::vector<std::vector<Coord>> tracks_;  // [targetIdx][node] — приватні
    std::vector<Target>             current_; // поточний знімок (під м'ютексом)
    mutable std::mutex              mtx_;

    float       arrayTimeStep_;
    float       timeScale_;
    std::string filename_;
    bool        loaded_   = false;
    int         nodeIndex_ = 0;               // поточний вузол (спільний для всіх)

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};
};
