#include "uav/Arbiter.hpp"

#include <cstdio>
#include <cstring>

namespace uav {

const char* toString(Channel c) {
    switch (c) {
        case Channel::A:        return "A";
        case Channel::B:        return "B";
        case Channel::Failsafe: return "FAILSAFE";
    }
    return "?";
}

const char* toString(Mode m) {
    return m == Mode::Auto ? "AUTO" : "MANUAL";
}

namespace {
void copyText(std::array<char, 48>& dst, const char* src) {
    std::snprintf(dst.data(), dst.size(), "%s", src);
}
void copyText(std::array<char, 96>& dst, const char* src) {
    std::snprintf(dst.data(), dst.size(), "%s", src);
}
} // namespace

bool Arbiter::bUsable(const Inputs& in) const {
    if (!in.bAlive) return false;
    // Затримка невідома - не привід вважати канал поганим: без вимірів
    // рішення ухвалюється лише за живістю.
    return !in.fullPathMs.has_value() || *in.fullPathMs < p_.rttSwitchFullMs;
}

Arbiter::Decision Arbiter::makeSwitch(Channel to, const char* air, const char* log,
                                      TimePoint now) {
    Decision d;
    d.changed = true;
    d.from    = active_;
    d.to      = to;
    copyText(d.air, air);
    copyText(d.log, log);
    active_ = to;
    ++switches_;
    ++switchesWindow_;
    bGoodSinceValid_ = false;
    (void)now;
    return d;
}

void Arbiter::requestManual(Channel c, TimePoint now) {
    mode_ = Mode::Manual;
    if (c != active_) makeSwitch(c, "operator command", "команда оператора", now);
    else              active_ = c;
}

void Arbiter::requestAuto() {
    mode_            = Mode::Auto;
    bGoodSinceValid_ = false;
}

Arbiter::Decision Arbiter::update(TimePoint now, const Inputs& in) {
    const bool usable = bUsable(in);

    // Відлік стабільності каналу B: скидається щоразу, коли B не бездоганний.
    const bool bGood = usable && (!in.fullPathMs.has_value()
                                  || *in.fullPathMs < p_.rttBackFullMs);
    if (!bGood) {
        bGoodSinceValid_ = false;
    } else if (!bGoodSinceValid_) {
        bGoodSince_      = now;
        bGoodSinceValid_ = true;
    }

    if (mode_ != Mode::Auto || !in.armed) return {};

    char air[48];
    char log[96];

    // Рівень 1: піти з основного каналу на резервний.
    if (active_ == Channel::B && !usable && in.aAlive) {
        if (!in.bAlive) {
            std::snprintf(air, sizeof(air), "B lost");
            std::snprintf(log, sizeof(log), "B недоступний");
        } else {
            std::snprintf(air, sizeof(air), "RTT %.0f ms", *in.fullPathMs);
            std::snprintf(log, sizeof(log), "повний шлях %.0f мс > %.0f",
                          *in.fullPathMs, p_.rttSwitchFullMs);
        }
        return makeSwitch(Channel::A, air, log, now);
    }

    // Повернення на основний - лише після витримки, інакше система задзвенить.
    if (active_ == Channel::A && bGood && bGoodSinceValid_
        && secBetween(bGoodSince_, now) > p_.tBackSec) {
        std::snprintf(air, sizeof(air), "B stable");
        std::snprintf(log, sizeof(log), "B стабільний %.0f с", p_.tBackSec);
        return makeSwitch(Channel::B, air, log, now);
    }

    // Вихід із режиму відмови, щойно бодай один канал ожив.
    if (active_ == Channel::Failsafe && (in.aAlive || usable)) {
        std::snprintf(air, sizeof(air), "link restored");
        std::snprintf(log, sizeof(log), "зв'язок відновлено");
        return makeSwitch(usable ? Channel::B : Channel::A, air, log, now);
    }

    // Обидва мовчать: перестаємо приховувати втрату - хай PX4 побачить
    // зникнення HEARTBEAT наземної станції і відпрацює NAV_DLL_ACT.
    if (active_ != Channel::Failsafe && !in.aAlive && !in.bAlive) {
        std::snprintf(air, sizeof(air), "both lost");
        std::snprintf(log, sizeof(log), "обидва канали втрачено");
        return makeSwitch(Channel::Failsafe, air, log, now);
    }

    return {};
}

} // namespace uav
