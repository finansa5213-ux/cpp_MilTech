// Базові типи маршрутизатора каналів керування БПЛА.
#pragma once

#include <chrono>
#include <cstdint>

namespace uav {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

/// Скільки мілісекунд минуло від a до b.
inline double msBetween(TimePoint a, TimePoint b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

inline double secBetween(TimePoint a, TimePoint b) {
    return std::chrono::duration<double>(b - a).count();
}

/// Канал, яким зараз іде телеметрія.
enum class Channel : std::uint8_t {
    A,        ///< радіо 433 МГц (SiK), незалежне від інфраструктури
    B,        ///< LTE + тунель WireGuard через концентратор
    Failsafe  ///< обидва канали мовчать
};

const char* toString(Channel c);

/// Режим арбітра.
enum class Mode : std::uint8_t { Auto, Manual };

const char* toString(Mode m);

/// Поля MAVLink RADIO_STATUS (msgid 109), які вставляє модем SiK.
/// Перетворення RSSI у дБм узяте з документації SiK: dBm = RSSI/1.9 - 127.
struct RadioStatus {
    std::uint16_t rxerrors = 0;
    std::uint16_t fixed    = 0;
    std::uint8_t  rssi     = 0;
    std::uint8_t  remrssi  = 0;
    std::uint8_t  txbuf    = 0;
    std::uint8_t  noise    = 0;
    std::uint8_t  remnoise = 0;

    static constexpr double kRssiScale  = 1.9;
    static constexpr double kRssiOffset = 127.0;

    double rssiDbm()  const { return rssi    / kRssiScale - kRssiOffset; }
    double noiseDbm() const { return noise   / kRssiScale - kRssiOffset; }
    /// Запас за сигналом: різниця рівня віддаленої станції й власного шуму.
    double marginDb() const { return (double(remrssi) - double(noise)) / kRssiScale; }
};

/// Поля MAVLink ESTIMATOR_STATUS (msgid 230). Потрібне лише поле прапорців:
/// це той самий набір бітів, що його PX4 публікує як solution_status_flags.
struct EstimatorStatus {
    std::uint16_t flags = 0;

    // ESTIMATOR_STATUS_FLAGS, common.xml
    static constexpr std::uint16_t kVelocityHoriz = 2;
    static constexpr std::uint16_t kPosHorizRel   = 8;
    static constexpr std::uint16_t kPosHorizAbs   = 16;
    static constexpr std::uint16_t kConstPosMode  = 128;
    static constexpr std::uint16_t kGpsGlitch     = 1024;

    /// Чи є куди повертатися. Команда повернення має сенс лише тоді, коли
    /// оцінювач має дійсну абсолютну горизонтальну координату.
    ///
    /// 20.09.2026 на стенді під час підміни GNSS цей біт був знятий, а
    /// ref_lat/ref_lon в оцінювачі дорівнювали nan: локальна система
    /// координат не мала прив'язки до Землі. Команда повернення в такому
    /// стані не ризикована - вона нездійсненна, бо немає звідки відлічувати
    /// шлях додому.
    bool navUsable()  const { return (flags & kPosHorizAbs) != 0; }
    bool gpsGlitch()  const { return (flags & kGpsGlitch)   != 0; }
    bool constPos()   const { return (flags & kConstPosMode) != 0; }
};

} // namespace uav
