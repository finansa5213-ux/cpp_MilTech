// Оцінка якості одного каналу.
//
// Головне правило, здобуте експериментом: живість каналу визначається ЛИШЕ
// трафіком, що приходить із землі. Успішний виклик передавання нічого не
// доводить - під час досліду 18.09 близько 8,5 КБ пішли в заблокований канал,
// і операційна система щоразу відрапортувала про успіх.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "uav/Types.hpp"

namespace uav {

class LinkMonitor {
public:
    LinkMonitor(const char* name, double tLostSec, double rttStaleSec);

    // --- події ---------------------------------------------------------
    void onGroundFrame(TimePoint now, std::size_t bytes);   ///< почули землю
    void onRadioStatus(const RadioStatus& rs, TimePoint now);
    void onRttSample(double ms, TimePoint now);             ///< зонд борт <-> сервер
    void onFullPathSample(double ms, TimePoint now);        ///< ICMP борт <-> оператор
    void onSent(std::size_t bytes)   { tx_ += bytes; }
    void onBypass(std::size_t bytes) { bypass_ += bytes; }

    // --- оцінки --------------------------------------------------------
    bool everHeard() const { return heard_; }
    bool alive(TimePoint now) const;

    /// Медіана останніх вимірів. Прострочені - недійсні: канал, що лежить,
    /// не повинен звітувати про 63 мс лише тому, що така була остання відповідь.
    std::optional<double> rttMedianMs(TimePoint now) const;

    /// Повний шлях борт <-> оператор. Якщо ICMP відповідає - це вимір.
    /// Якщо ні, береться зонд у тунелі плюс калібрувальне зміщення: гірше,
    /// але краще за ніщо. Два прогони 18.09 дали зміщення 72,0 і 65,2 мс,
    /// тож оцінка має похибку близько ±5 мс і позначається як оцінка.
    std::optional<double> fullPathMs(TimePoint now, double offsetMs) const;
    bool fullPathMeasured(TimePoint now) const;

    const RadioStatus& radio() const { return radio_; }
    bool radioFresh(TimePoint now) const;

    // --- статистика вікна звіту ----------------------------------------
    struct Window {
        std::uint64_t rx = 0, tx = 0, frames = 0, bypass = 0;
    };
    Window      window() const { return {rx_, tx_, frames_, bypass_}; }
    void        resetWindow()  { rx_ = tx_ = frames_ = bypass_ = 0; }
    const char* name() const   { return name_; }

private:
    static constexpr std::size_t kRttSamples = 10;

    const char* name_;
    double      tLostSec_;
    double      rttStaleSec_;

    bool      heard_     = false;
    TimePoint lastGround_{};
    TimePoint lastRtt_{};
    TimePoint lastRadio_{};

    static constexpr std::size_t kFullSamples = 5;

    std::array<double, kRttSamples> rtt_{};
    std::size_t rttCount_ = 0;
    std::size_t rttHead_  = 0;

    std::array<double, kFullSamples> full_{};
    std::size_t rttFullCount_ = 0;
    std::size_t rttFullHead_  = 0;
    TimePoint   lastFull_{};

    RadioStatus radio_{};

    std::uint64_t rx_ = 0, tx_ = 0, frames_ = 0, bypass_ = 0;
};

} // namespace uav
