// Маршрутизатор: єдиний власник UART польотного контролера й обох каналів.
//
// Однопотокова подієва петля на epoll. Це не спрощення, а вимога: у програмі
// немає жодного м'ютекса, бо немає спільних даних між потоками, а отже немає
// ні перегонів, ні пріоритетної інверсії, ні розкиду затримки від блокувань.
// Прототип на Python тримав вісім потоків і був змушений серіалізувати запис
// в UART замком - тут цієї проблеми просто не існує.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "uav/Arbiter.hpp"
#include "uav/Config.hpp"
#include "uav/Failsafe.hpp"
#include "uav/IcmpProbe.hpp"
#include "uav/LinkMonitor.hpp"
#include "uav/Mavlink.hpp"
#include "uav/SerialTransport.hpp"
#include "uav/UdpTransport.hpp"

namespace uav {

class Router {
public:
    explicit Router(Config cfg);
    ~Router();

    /// Відкрити пристрої й сокети. false - програма не може працювати.
    bool init(std::string& err);

    /// Подієва петля. Повертається після SIGINT/SIGTERM.
    int run();

    static void requestStop();   ///< з обробника сигналу

private:
    // --- джерела подій --------------------------------------------------
    void onFcReadable(TimePoint now);
    void onRadioReadable(TimePoint now);
    void onUdpReadable(TimePoint now);
    void onEchoReadable(TimePoint now);
    void onIcmpReadable(TimePoint now);
    void onControlReadable(TimePoint now);

    // --- такти -----------------------------------------------------------
    void onFlushTick(TimePoint now);     ///< віддати накопичену телеметрію + арбітр
    void onSecondTick(TimePoint now);    ///< keepalive резерву + зонд у тунелі
    void onFullProbeTick(TimePoint now); ///< вимір повного шляху
    void onReportTick(TimePoint now);

    // --- дії ---------------------------------------------------------------
    void sendToActive(TimePoint now);
    void announce(const char* ascii, const char* uk, std::uint8_t severity);
    void applyDecision(const Arbiter::Decision& d);

    /// Чи має оцінювач дійсну абсолютну горизонтальну координату - тобто чи
    /// є взагалі куди повертатися. Невідомий і застарілий стан трактуються
    /// як непридатні: якщо ми не знаємо, команду повернення давати не можна.
    bool navUsable(TimePoint now) const;

    bool addToEpoll(int fd);
    int  makeTimer(double seconds);

    Config cfg_;

    SerialTransport fc_;
    SerialTransport radio_;
    UdpTransport    udp_;
    UdpTransport    echo_;
    IcmpProbe       icmp_;
    int             ctrlFd_  = -1;
    int             epollFd_ = -1;

    int tickFd_   = -1;
    int secFd_    = -1;
    int fullFd_   = -1;
    int reportFd_ = -1;

    mavlink::Framer fcFramer_;
    mavlink::Framer radioFramer_;
    mavlink::Framer udpFramer_;

    LinkMonitor monA_;
    LinkMonitor monB_;
    Arbiter     arbiter_;
    Failsafe    failsafe_;

    std::vector<std::uint8_t>  out_;          ///< телеметрія, що чекає на відправлення
    std::array<std::uint8_t, 64> hb_{};       ///< останній HEARTBEAT борта
    std::size_t                hbLen_ = 0;
    std::uint8_t               seq_   = 0;

    std::uint64_t staleBytes_ = 0;            ///< відкинуто як застаріле за вікно
    std::uint64_t radioToFc_  = 0;
    bool          icmpReady_  = false;

    // Стан оцінювача польотного контролера: єдине, що відрізняє «є куди
    // повертатися» від «немає». Читається з ESTIMATOR_STATUS у потоці,
    // який маршрутизатор і так розбирає.
    EstimatorStatus est_{};
    TimePoint       estAt_{};
    bool            estValid_  = false;
    std::uint64_t   estFrames_ = 0;           ///< скільки таких кадрів за вікно
    bool            navWarned_ = false;       ///< щоб не повторювати попередження

    // Регулярність такту обробки. Міра детермінованості: наскільки фактичний
    // момент такту відхиляється від сітки, заданої flushSec.
    TimePoint     lastTick_{};
    bool          lastTickValid_ = false;
    double        jitterSum_     = 0.0;
    double        jitterMax_     = 0.0;
    std::uint64_t jitterCount_   = 0;
    std::uint64_t overruns_      = 0;         ///< тактів, які петля пропустила
};

} // namespace uav
