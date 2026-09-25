// Правило вибору каналу.
//
// Пороги взяті не зі стелі, а з вимірювань 17-18.09:
//   медіана повного обігу каналу A .......... 160,9 мс  (точка рівноваги)
//   повний шлях каналу B за ніч ............. 126,8 ... 197,8 мс
// Канал B перетинає точку рівноваги сам собою протягом хвилин, тому керувати
// рівно на ній не можна - система задзвенить. Пороги рознесені в гістерезисну
// смугу: піти з B при 200 мс, повернутися при 170 мс. Ширина смуги 30 мс -
// удвічі більша за дрижання вимірювання (mdev 6...17 мс).
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "uav/Types.hpp"

namespace uav {

class Arbiter {
public:
    struct Params {
        double tBackSec        = 10.0;   ///< скільки B має бути здоровим перед поверненням
        double rttSwitchFullMs = 200.0;  ///< гірше за це - іти з B
        double rttBackFullMs   = 170.0;  ///< краще за це - можна повертатися
    };

    struct Inputs {
        bool                  aAlive     = false;
        bool                  bAlive     = false;
        std::optional<double> fullPathMs;   ///< борт <-> оператор, якщо відоме
        bool                  armed      = false; ///< чи був хоч один контакт із землею
    };

    /// Рішення за один такт. Тексти у фіксованих буферах: у гарячому шляху
    /// маршрутизатора немає жодного виділення пам'яті.
    struct Decision {
        bool                   changed = false;
        Channel                from    = Channel::B;
        Channel                to      = Channel::B;
        std::array<char, 48>   air{};    ///< ASCII для STATUSTEXT у наземну станцію
        std::array<char, 96>   log{};    ///< українською в журнал
    };

    explicit Arbiter(Params p) : p_(p) {}

    Decision update(TimePoint now, const Inputs& in);

    void requestManual(Channel c, TimePoint now);
    void requestAuto();

    Channel  active()   const { return active_; }
    Mode     mode()     const { return mode_; }

    /// Усього перемикань від запуску - для контролю за весь політ.
    unsigned switches() const { return switches_; }

    /// Перемикань за поточне вікно звіту. Саме ця величина йде в таблицю
    /// «перемикань за годину»: із загального лічильника спокійну годину від
    /// неспокійної видно лише порівнянням сусідніх рядків журналу.
    unsigned switchesInWindow() const { return switchesWindow_; }
    void     resetWindow() { switchesWindow_ = 0; }

    /// Чи вважається канал B придатним просто зараз (жива земля + затримка в нормі).
    bool bUsable(const Inputs& in) const;

private:
    Decision makeSwitch(Channel to, const char* air, const char* log, TimePoint now);

    Params    p_;
    Channel   active_        = Channel::B;
    Mode      mode_          = Mode::Auto;
    unsigned  switches_      = 0;
    unsigned  switchesWindow_ = 0;
    TimePoint bGoodSince_{};
    bool      bGoodSinceValid_ = false;
};

} // namespace uav
