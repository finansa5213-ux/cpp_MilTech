#include "uav/Router.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cstdarg>
#include <cerrno>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <ctime>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <utility>

namespace uav {
namespace {

std::atomic<bool> gStop{false};

/// Журнал іде в stdout: systemd підхоплює його сам, а власного файла,
/// ротації та прав доступу програмі тримати не треба.
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logf(const char* fmt, ...) {
    char      line[512];
    va_list   ap;
    va_start(ap, fmt);
    std::vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    std::fputs(line, stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
}

std::string hhmmss() {
    const std::time_t t = std::time(nullptr);
    std::tm           tm{};
    ::localtime_r(&t, &tm);
    char b[16];
    std::snprintf(b, sizeof(b), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return b;
}

/// Спорожнити таймер і повернути кількість спрацювань. Значення понад одиницю
/// означає, що петля не встигла обробити попередній такт - це і є пропуск.
std::uint64_t drainTimer(int fd) {
    std::uint64_t total = 0, ticks = 0;
    while (::read(fd, &ticks, sizeof(ticks)) > 0) total += ticks;
    return total;
}

} // namespace

void Router::requestStop() { gStop = true; }

Router::Router(Config cfg)
    : cfg_(std::move(cfg)),
      fc_("контролер", cfg_.fcDevice, cfg_.fcBaud),
      radio_("радіо", cfg_.radioDevice, cfg_.radioBaud),
      udp_("тунель", cfg_.udpPort, cfg_.peerHost, cfg_.peerPort, cfg_.peerFixed),
      echo_("зонд", 0, cfg_.peerHost, cfg_.echoPort, true),
      icmp_(cfg_.operatorHost),
      monA_("A", cfg_.tLostSec, cfg_.rttStaleSec),
      monB_("B", cfg_.tLostSec, cfg_.rttStaleSec),
      arbiter_(Arbiter::Params{cfg_.tBackSec, cfg_.rttSwitchFullMs, cfg_.rttBackFullMs}),
      failsafe_(Failsafe::Params{cfg_.failsafeSec, cfg_.commandRtl(),
                                 cfg_.requireNavForRtl}) {
    out_.reserve(cfg_.maxBacklog + cfg_.maxDatagram);
}

Router::~Router() {
    for (int fd : {ctrlFd_, tickFd_, secFd_, fullFd_, reportFd_, epollFd_})
        if (fd >= 0) ::close(fd);
}

bool Router::addToEpoll(int fd) {
    epoll_event ev{};
    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    return ::epoll_ctl(epollFd_, EPOLL_CTL_ADD, fd, &ev) == 0;
}

int Router::makeTimer(double seconds) {
    const int fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) return -1;
    itimerspec its{};
    its.it_interval.tv_sec  = static_cast<time_t>(seconds);
    its.it_interval.tv_nsec = static_cast<long>((seconds - its.it_interval.tv_sec) * 1e9);
    its.it_value            = its.it_interval;
    if (::timerfd_settime(fd, 0, &its, nullptr) != 0) { ::close(fd); return -1; }
    return fd;
}

bool Router::init(std::string& err) {
    if (!fc_.open(err))    return false;
    if (!radio_.open(err)) {
        // Без радіо маршрутизатор ще має сенс: лишається канал B. Але це
        // втрата резерву, тож повідомляємо голосно й працюємо далі.
        logf("УВАГА: канал A недоступний (%s) - працюємо без резерву", err.c_str());
        err.clear();
    }
    if (!udp_.open(err))  return false;
    if (!echo_.open(err)) return false;

    std::string icmpErr;
    icmpReady_ = icmp_.open(icmpErr);
    if (!icmpReady_)
        logf("УВАГА: вимір повного шляху недоступний (%s) - буде оцінка зі зміщенням %.0f мс",
             icmpErr.c_str(), cfg_.rttOffsetMs);

    ctrlFd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (ctrlFd_ < 0) { err = "сокет керування: " + std::string(std::strerror(errno)); return false; }
    sockaddr_in ca{};
    ca.sin_family      = AF_INET;
    ca.sin_addr.s_addr = htonl(INADDR_ANY);
    ca.sin_port        = htons(cfg_.controlPort);
    if (::bind(ctrlFd_, reinterpret_cast<sockaddr*>(&ca), sizeof(ca)) != 0) {
        err = "bind " + std::to_string(cfg_.controlPort) + ": " + std::strerror(errno);
        return false;
    }

    epollFd_ = ::epoll_create1(EPOLL_CLOEXEC);
    if (epollFd_ < 0) { err = "epoll_create1"; return false; }

    tickFd_   = makeTimer(cfg_.flushSec);
    secFd_    = makeTimer(cfg_.keepaliveSec);
    fullFd_   = makeTimer(cfg_.fullProbeSec);
    reportFd_ = makeTimer(cfg_.reportSec);
    if (tickFd_ < 0 || secFd_ < 0 || fullFd_ < 0 || reportFd_ < 0) {
        err = "не вдалося створити таймери";
        return false;
    }

    bool ok = addToEpoll(fc_.fd()) && addToEpoll(udp_.fd()) && addToEpoll(echo_.fd())
              && addToEpoll(ctrlFd_) && addToEpoll(tickFd_) && addToEpoll(secFd_)
              && addToEpoll(fullFd_) && addToEpoll(reportFd_);
    if (radio_.isOpen()) ok = ok && addToEpoll(radio_.fd());
    if (icmpReady_)      ok = ok && addToEpoll(icmp_.fd());
    if (!ok) { err = "epoll_ctl"; return false; }
    return true;
}

// ---------------------------------------------------------------- контролер
void Router::onFcReadable(TimePoint now) {
    std::uint8_t buf[4096];
    for (;;) {
        const ssize_t r = fc_.read(buf, sizeof(buf));
        if (r <= 0) break;
        fcFramer_.feed(buf, static_cast<std::size_t>(r));
        mavlink::Frame f;
        while (fcFramer_.next(f)) {
            if (f.msgid == mavlink::kMsgHeartbeat && f.rawSize <= hb_.size()) {
                std::memcpy(hb_.data(), f.raw, f.rawSize);
                hbLen_ = f.rawSize;
            } else if (f.msgid == mavlink::kMsgEstimatorStatus) {
                EstimatorStatus es;
                if (mavlink::parseEstimatorStatus(f, es)) {
                    est_      = es;
                    estAt_    = now;
                    estValid_ = true;
                    ++estFrames_;
                }
            }
            out_.insert(out_.end(), f.raw, f.raw + f.rawSize);
        }
        if (r < static_cast<ssize_t>(sizeof(buf))) break;
    }

    // Усе, що не пішло, поки канал лежав, - застаріле. Віддати через три
    // секунди старий крен гірше, ніж не віддати нічого.
    if (out_.size() > cfg_.maxBacklog) {
        staleBytes_ += out_.size();
        out_.clear();
    }
}

// ---------------------------------------------------------------- канал A
void Router::onRadioReadable(TimePoint now) {
    std::uint8_t buf[2048];
    for (;;) {
        const ssize_t r = radio_.read(buf, sizeof(buf));
        if (r <= 0) break;
        monA_.onSent(0);
        radioFramer_.feed(buf, static_cast<std::size_t>(r));
        mavlink::Frame f;
        while (radioFramer_.next(f)) {
            RadioStatus rs;
            if (mavlink::parseRadioStatus(f, rs)) {
                // Звіт власного модема, а не кадр із землі: живості не додає.
                monA_.onRadioStatus(rs, now);
                // PX4 використовує txbuf для керування потоком - але тільки
                // тоді, коли телеметрія справді йде радіо.
                if (arbiter_.active() == Channel::A) fc_.write(f.raw, f.rawSize);
                continue;
            }
            monA_.onGroundFrame(now, f.rawSize);
            fc_.write(f.raw, f.rawSize);
            ++radioToFc_;
        }
        if (r < static_cast<ssize_t>(sizeof(buf))) break;
    }
}

// ---------------------------------------------------------------- канал B
void Router::onUdpReadable(TimePoint now) {
    std::uint8_t buf[2048];
    for (;;) {
        const ssize_t r = udp_.read(buf, sizeof(buf));
        if (r <= 0) break;
        udpFramer_.feed(buf, static_cast<std::size_t>(r));
        mavlink::Frame f;
        while (udpFramer_.next(f)) {
            monB_.onGroundFrame(now, f.rawSize);
            fc_.write(f.raw, f.rawSize);
        }
    }
    if (udp_.hasNewRejection())
        logf("ВІДХИЛЕНО: спроба обійти концентратор, джерело %s", udp_.lastRejected());
    monB_.onBypass(0);
}

void Router::onEchoReadable(TimePoint now) {
    std::uint8_t buf[64];
    for (;;) {
        const ssize_t r = echo_.read(buf, sizeof(buf));
        if (r <= 0) break;
        if (r != sizeof(std::uint64_t)) continue;
        std::uint64_t sentNs = 0;
        std::memcpy(&sentNs, buf, sizeof(sentNs));
        const auto nowNs = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        if (sentNs == 0 || sentNs > nowNs) continue;
        const double ms = static_cast<double>(nowNs - sentNs) / 1e6;
        if (ms > 0.0 && ms < 5000.0) monB_.onRttSample(ms, now);
    }
}

void Router::onIcmpReadable(TimePoint now) {
    while (const auto ms = icmp_.readReply(now)) monB_.onFullPathSample(*ms, now);
}

// ---------------------------------------------------------------- керування
void Router::onControlReadable(TimePoint now) {
    char        buf[64];
    sockaddr_in from{};
    socklen_t   len = sizeof(from);
    for (;;) {
        const ssize_t r = ::recvfrom(ctrlFd_, buf, sizeof(buf) - 1, 0,
                                     reinterpret_cast<sockaddr*>(&from), &len);
        if (r <= 0) break;
        buf[r] = '\0';

        char ip[INET_ADDRSTRLEN] = {};
        ::inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
        bool allowed = false;
        for (const auto& a : cfg_.controlAllow)
            if (a == ip) { allowed = true; break; }
        // Спостерігач каналами не керує - те саме розмежування прав,
        // що й у концентраторі, лише застосоване на борту.
        if (!allowed) continue;

        std::string cmd;
        for (ssize_t i = 0; i < r; ++i)
            if (buf[i] > ' ') cmd += static_cast<char>(std::toupper(buf[i]));

        if (cmd == "A" || cmd == "B") {
            arbiter_.requestManual(cmd == "A" ? Channel::A : Channel::B, now);
            announce(cmd == "A" ? "LINK -> A: operator" : "LINK -> B: operator",
                     "команда оператора", 4);
        } else if (cmd == "AUTO") {
            arbiter_.requestAuto();
            announce("LINK mode AUTO", "режим AUTO", 6);
        }

        const auto full = monB_.fullPathMs(now, cfg_.rttOffsetMs);
        const auto tun  = monB_.rttMedianMs(now);
        char ans[192];
        std::snprintf(ans, sizeof(ans),
                      "режим=%s активний=%s рівень=%d A=%s B=%s тунель=%s шлях=%s",
                      toString(arbiter_.mode()), toString(arbiter_.active()),
                      static_cast<int>(failsafe_.level()),
                      monA_.alive(now) ? "ok" : "lost", monB_.alive(now) ? "ok" : "lost",
                      tun ? std::to_string(static_cast<int>(*tun)).c_str() : "-",
                      full ? std::to_string(static_cast<int>(*full)).c_str() : "-");
        ::sendto(ctrlFd_, ans, std::strlen(ans), 0,
                 reinterpret_cast<sockaddr*>(&from), len);
    }
}

// ---------------------------------------------------------------- такти
void Router::sendToActive(TimePoint now) {
    if (out_.empty()) return;
    switch (arbiter_.active()) {
        case Channel::B: {
            std::size_t off = 0;
            while (off < out_.size()) {
                const std::size_t n = std::min(cfg_.maxDatagram, out_.size() - off);
                const ssize_t w = udp_.write(out_.data() + off, n);
                if (w <= 0) { staleBytes_ += out_.size() - off; break; }
                monB_.onSent(static_cast<std::size_t>(w));
                off += n;
            }
            break;
        }
        case Channel::A:
            if (radio_.isOpen()) {
                const ssize_t w = radio_.write(out_.data(), out_.size());
                if (w > 0) monA_.onSent(static_cast<std::size_t>(w));
                if (w < static_cast<ssize_t>(out_.size()))
                    staleBytes_ += out_.size() - static_cast<std::size_t>(w > 0 ? w : 0);
            }
            break;
        case Channel::Failsafe:
            staleBytes_ += out_.size();   // нікому віддавати
            break;
    }
    out_.clear();
    (void)now;
}

void Router::announce(const char* ascii, const char* uk, std::uint8_t severity) {
    std::uint8_t frame[64];
    const std::size_t n = mavlink::buildStatusText(frame, sizeof(frame), ascii, severity, seq_);
    // Обома каналами: наземна станція має дізнатися про подію навіть тим
    // каналом, який щойно перестав бути активним.
    if (n != 0) {
        udp_.write(frame, n);
        if (radio_.isOpen()) radio_.write(frame, n);
    }
    logf("%s", uk);
}

void Router::applyDecision(const Arbiter::Decision& d) {
    if (!d.changed) return;
    char air[64];
    std::snprintf(air, sizeof(air), "LINK -> %s: %s", toString(d.to), d.air.data());
    char uk[160];
    std::snprintf(uk, sizeof(uk), "КАНАЛ -> %s (%s)", toString(d.to), d.log.data());
    announce(air, uk, 4);
}

bool Router::navUsable(TimePoint now) const {
    // Стан, якого ми не знаємо, - непридатний. Це та сама засада, що й із
    // застарілим виміром затримки: канал, який лежить, не має звітувати про
    // 63 мс лише тому, що така була остання відповідь.
    if (!estValid_) return false;
    if (secBetween(estAt_, now) > cfg_.navStaleSec) return false;
    return est_.navUsable();
}

void Router::onFlushTick(TimePoint now) {
    const std::uint64_t ticks = drainTimer(tickFd_);
    if (ticks > 1) overruns_ += ticks - 1;

    if (lastTickValid_) {
        const double dev = std::fabs(secBetween(lastTick_, now) - cfg_.flushSec) * 1000.0;
        jitterSum_ += dev;
        if (dev > jitterMax_) jitterMax_ = dev;
        ++jitterCount_;
    }
    lastTick_      = now;
    lastTickValid_ = true;

    Arbiter::Inputs in;
    in.aAlive     = monA_.alive(now);
    in.bAlive     = monB_.alive(now);
    in.armed      = monA_.everHeard() || monB_.everHeard();
    in.fullPathMs = monB_.fullPathMs(now, cfg_.rttOffsetMs);

    applyDecision(arbiter_.update(now, in));

    const auto act = failsafe_.update(now, in.armed, in.aAlive || in.bAlive,
                                      arbiter_.active(), navUsable(now));
    if (act.entered && act.log != nullptr) logf("%s", act.log);
    if (act.sendRtl) {
        std::uint8_t frame[64];
        const std::size_t n = mavlink::buildReturnToLaunch(frame, sizeof(frame), seq_);
        if (n != 0) fc_.write(frame, n);
    }
    if (act.navBlocked && !navWarned_) {
        // Наземна станція має дізнатися, чому повернення не сталося, - інакше
        // оператор бачитиме лише мовчання й вирішить, що впав маршрутизатор.
        navWarned_ = true;
        announce("NAV unusable: no RTL", "навігація непридатна - повернення не командуємо", 2);
    }
    if (!act.navBlocked) navWarned_ = false;

    sendToActive(now);
}

void Router::onSecondTick(TimePoint now) {
    drainTimer(secFd_);

    // Резервний канал не можна лишати німим: без цих 21 Б/с наземна станція
    // згортає невживану лінію, борт перестає бачити резерв живим і не має
    // куди повертатися. Для каналу A це ще й єдиний спосіб мати свіжі
    // rssi/шум ДО перемикання, а не після нього.
    if (hbLen_ != 0) {
        const Channel act = arbiter_.active();
        if (act != Channel::A && radio_.isOpen()) {
            const ssize_t w = radio_.write(hb_.data(), hbLen_);
            if (w > 0) monA_.onSent(static_cast<std::size_t>(w));
        }
        if (act != Channel::B) {
            const ssize_t w = udp_.write(hb_.data(), hbLen_);
            if (w > 0) monB_.onSent(static_cast<std::size_t>(w));
        }
    }

    const auto ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    std::uint8_t stamp[sizeof(ns)];
    std::memcpy(stamp, &ns, sizeof(ns));
    echo_.write(stamp, sizeof(stamp));
}

void Router::onFullProbeTick(TimePoint now) {
    drainTimer(fullFd_);
    if (icmpReady_) icmp_.ping(now);
}

void Router::onReportTick(TimePoint now) {
    drainTimer(reportFd_);

    const auto a    = monA_.window();
    const auto b    = monB_.window();
    const auto rs   = monA_.radio();
    const auto tun  = monB_.rttMedianMs(now);
    const auto full = monB_.fullPathMs(now, cfg_.rttOffsetMs);

    char stale[96] = "";
    if (staleBytes_ != 0)
        std::snprintf(stale, sizeof(stale), "  відкинуто застарілих %llu Б",
                      static_cast<unsigned long long>(staleBytes_));

    logf("[%s] активний=%s  режим=%s  перемикань %u за вікно (%u від запуску)  рівень=%d%s",
         hhmmss().c_str(), toString(arbiter_.active()), toString(arbiter_.mode()),
         arbiter_.switchesInWindow(), arbiter_.switches(),
         static_cast<int>(failsafe_.level()), stale);

    logf("   A: %s  вгору %llu кадр  вниз %llu Б  rssi %u/%u  шум %u  txbuf %u%%  "
         "помилок %u  запас %.0f дБ",
         monA_.alive(now) ? "живий" : "МОВЧИТЬ",
         static_cast<unsigned long long>(a.frames), static_cast<unsigned long long>(a.tx),
         rs.rssi, rs.remrssi, rs.noise, rs.txbuf, rs.rxerrors, rs.marginDb());

    char bypass[96] = "";
    if (udp_.bypassBytes() != 0)
        std::snprintf(bypass, sizeof(bypass), "  повз концентратор відхилено %llu Б",
                      static_cast<unsigned long long>(udp_.bypassBytes()));

    char tunTxt[32]  = "немає";
    char fullTxt[48] = "немає";
    if (tun)  std::snprintf(tunTxt, sizeof(tunTxt), "%.1f мс", *tun);
    if (full) std::snprintf(fullTxt, sizeof(fullTxt), "%.1f мс (%s)", *full,
                            monB_.fullPathMeasured(now) ? "вимір" : "оцінка");

    logf("   B: %s  вгору %llu Б  вниз %llu Б  тунель %s  повний шлях %s%s",
         monB_.alive(now) ? "живий" : "МОВЧИТЬ",
         static_cast<unsigned long long>(b.rx), static_cast<unsigned long long>(b.tx),
         tunTxt, fullTxt, bypass);

    // Стан навігації: від нього залежить, чи має сенс третій рівень захисту.
    if (!estValid_) {
        logf("   навігація: контролер не передає ESTIMATOR_STATUS - "
             "рівень 3 не спрацює");
    } else {
        logf("   навігація: %s  прапорці 0x%04X%s%s  кадрів %llu  "
             "тривалість від останнього %.1f с",
             navUsable(now) ? "придатна" : "НЕПРИДАТНА",
             static_cast<unsigned>(est_.flags),
             est_.constPos()  ? "  стала позиція" : "",
             est_.gpsGlitch() ? "  збій GPS" : "",
             static_cast<unsigned long long>(estFrames_),
             secBetween(estAt_, now));
    }

    logf("   такт %.0f мс: відхилення середнє %.3f мс, найбільше %.3f мс, "
         "пропусків %llu з %llu",
         cfg_.flushSec * 1000.0,
         jitterCount_ ? jitterSum_ / static_cast<double>(jitterCount_) : 0.0,
         jitterMax_,
         static_cast<unsigned long long>(overruns_),
         static_cast<unsigned long long>(jitterCount_));

    monA_.resetWindow();
    monB_.resetWindow();
    arbiter_.resetWindow();
    udp_.resetBypass();
    staleBytes_  = 0;
    estFrames_   = 0;
    jitterSum_   = 0.0;
    jitterMax_   = 0.0;
    jitterCount_ = 0;
    overruns_    = 0;
}

// ---------------------------------------------------------------- петля
int Router::run() {
    logf("маршрутизатор: A=%s  B=%u -> %s:%u  режим=AUTO",
         cfg_.radioDevice.c_str(), cfg_.udpPort, cfg_.peerHost.c_str(), cfg_.peerPort);
    logf("   повний шлях до %s: іти з B при %.0f мс, повертатися при %.0f; "
         "точка рівноваги (канал A) 160.9 мс",
         cfg_.operatorHost.c_str(), cfg_.rttSwitchFullMs, cfg_.rttBackFullMs);

    epoll_event events[16];
    while (!gStop) {
        const int n = ::epoll_wait(epollFd_, events, 16, 1000);
        if (n < 0) {
            if (errno == EINTR) continue;
            logf("epoll_wait: %s", std::strerror(errno));
            return 1;
        }
        const TimePoint now = Clock::now();
        for (int i = 0; i < n; ++i) {
            const int fd = events[i].data.fd;
            if      (fd == fc_.fd())    onFcReadable(now);
            else if (fd == udp_.fd())   onUdpReadable(now);
            else if (fd == echo_.fd())  onEchoReadable(now);
            else if (fd == ctrlFd_)     onControlReadable(now);
            else if (fd == tickFd_)     onFlushTick(now);
            else if (fd == secFd_)      onSecondTick(now);
            else if (fd == fullFd_)     onFullProbeTick(now);
            else if (fd == reportFd_)   onReportTick(now);
            else if (radio_.isOpen() && fd == radio_.fd()) onRadioReadable(now);
            else if (icmpReady_ && fd == icmp_.fd())       onIcmpReadable(now);
        }
    }
    logf("зупинка за сигналом");
    return 0;
}

} // namespace uav
