#include "uav/Failsafe.hpp"

namespace uav {

Failsafe::Action Failsafe::update(TimePoint now, bool armed, bool anyAlive, Channel active,
                                  bool navUsable) {
    Action a;

    if (!armed) {
        level_          = Level::Normal;
        deadSinceValid_ = false;
        a.level         = level_;
        return a;
    }

    if (anyAlive) {
        const Level want = (active == Channel::A) ? Level::Reserve : Level::Normal;
        a.entered        = (want != level_);
        level_           = want;
        deadSinceValid_  = false;
        a.level          = level_;
        if (a.entered)
            a.log = (want == Level::Reserve) ? "рівень 1: працюємо на резервному каналі"
                                             : "рівень 0: основний канал відновлено";
        return a;
    }

    if (!deadSinceValid_) {
        deadSince_      = now;
        deadSinceValid_ = true;
        level_          = Level::BothLost;
        a.entered       = true;
        a.log           = "РІВЕНЬ 2: обидва канали втрачено - PX4 бачить зникнення HEARTBEAT";
        a.level         = level_;
        return a;
    }

    if (level_ == Level::BothLost && secBetween(deadSince_, now) > p_.holdSec) {
        level_    = Level::SelfReturn;
        a.entered = true;

        if (p_.requireNav && !navUsable) {
            // Час рівня 3 настав, але повертатися нікуди: оцінювач не має
            // дійсної абсолютної горизонтальної координати. Команду не
            // надсилаємо й кажемо про це прямо - інакше в журналі був би
            // рядок «надіслано RETURN_TO_LAUNCH» там, де повернення
            // насправді неможливе.
            a.navBlocked = true;
            a.log = "РІВЕНЬ 3: команду повернення НЕ надіслано - оцінювач не має "
                    "дійсної горизонтальної координати";
        } else {
            a.sendRtl = p_.commandRtl;
            a.log     = p_.commandRtl
                            ? "РІВЕНЬ 3: надіслано RETURN_TO_LAUNCH"
                            : "РІВЕНЬ 3: спрацював би RETURN_TO_LAUNCH (failsafe_action=log)";
        }
    }

    a.level = level_;
    return a;
}

} // namespace uav
