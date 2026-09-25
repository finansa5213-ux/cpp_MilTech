// Багаторівневий захист від втрати зв'язку.
//
//   рівень 1  перемикання на резервний канал                (робить Arbiter)
//   рівень 2  обидва канали мовчать - маршрутизатор перестає приховувати
//             втрату: PX4 бачить зникнення HEARTBEAT наземної станції і
//             відпрацьовує власний NAV_DLL_ACT (Return mode)
//   рівень 3  PX4 не відреагував за holdSec - маршрутизатор сам надсилає
//             MAV_CMD_NAV_RETURN_TO_LAUNCH у польотний контролер
//
// Рівень 3 за умовчанням лише пишеться в журнал: на стенді команда повернення
// нікому не потрібна, а помилкове спрацювання коштувало б апарата.
#pragma once

#include <cstdint>

#include "uav/Types.hpp"

namespace uav {

class Failsafe {
public:
    enum class Level : std::uint8_t {
        Normal     = 0,
        Reserve    = 1,
        BothLost   = 2,
        SelfReturn = 3
    };

    struct Params {
        double holdSec    = 5.0;    ///< скільки чекати на реакцію PX4 перед рівнем 3
        bool   commandRtl = false;  ///< true - справді надсилати команду повернення
        /// Не командувати повернення, поки оцінювач не має дійсної
        /// абсолютної горизонтальної координати. Вимірювання 20.09.2026:
        /// під час підміни GNSS координати не було взагалі, і команда
        /// повернення була б нездійсненною.
        bool   requireNav = true;
    };

    struct Action {
        Level       level      = Level::Normal;
        bool        entered    = false;  ///< рівень щойно змінився
        bool        sendRtl    = false;  ///< цього такту треба надіслати команду
        bool        navBlocked = false;  ///< рівень 3 настав, але навігація непридатна
        const char* log        = nullptr;
    };

    explicit Failsafe(Params p) : p_(p) {}

    /// armed - чи був хоч один контакт із землею від запуску. До нього захист
    /// не озброєний: на увімкненні живлення обидва канали природно мовчать,
    /// і без цієї умови маршрутизатор оголошував би втрату зв'язку в першу ж
    /// мить, а через holdSec секунд командував би повернення апарату, який
    /// просто ще не встиг ні з ким з'єднатися.
    ///
    /// navUsable - чи має оцінювач дійсну абсолютну горизонтальну координату.
    /// Невідомий стан навігації передається як false: якщо ми не знаємо,
    /// чи є куди повертатися, команду повернення давати не можна. При цьому
    /// рівень 2 працює далі, тож захист лише вироджується до власного
    /// NAV_DLL_ACT польотного контролера, а не зникає.
    Action update(TimePoint now, bool armed, bool anyAlive, Channel active,
                  bool navUsable);

    Level level() const { return level_; }

private:
    Params    p_;
    Level     level_ = Level::Normal;
    TimePoint deadSince_{};
    bool      deadSinceValid_ = false;
};

} // namespace uav
