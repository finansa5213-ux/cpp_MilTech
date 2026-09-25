// Розбір і збирання кадрів MAVLink без зовнішніх залежностей.
//
// Навмисно не використано pymavlink/c_library_v2: маршрутизатору потрібні лише
// межі кадрів, номер повідомлення та кілька полів RADIO_STATUS. Власний
// розбирач - це 150 рядків, нуль залежностей у польотному образі та повний
// контроль над поведінкою при смітті в потоці.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "uav/Types.hpp"

namespace uav::mavlink {

constexpr std::uint8_t kStxV1 = 0xFE;
constexpr std::uint8_t kStxV2 = 0xFD;

constexpr std::uint32_t kMsgHeartbeat       = 0;
constexpr std::uint32_t kMsgRadioStatus     = 109;
constexpr std::uint32_t kMsgCommandLong     = 76;
constexpr std::uint32_t kMsgStatusText      = 253;
constexpr std::uint32_t kMsgEstimatorStatus = 230;

// CRC_EXTRA з common.xml - друга половина контрольної суми MAVLink.
constexpr std::uint8_t kCrcExtraCommandLong = 152;
constexpr std::uint8_t kCrcExtraStatusText  = 83;

constexpr std::uint16_t kCmdNavReturnToLaunch = 20;

/// Один цілий кадр. Вказівники дійсні до наступного виклику feed() або next().
struct Frame {
    const std::uint8_t* raw         = nullptr;
    std::size_t         rawSize     = 0;
    const std::uint8_t* payload     = nullptr;
    std::size_t         payloadSize = 0;
    std::uint32_t       msgid       = 0;
    std::uint8_t        sysid       = 0;
};

/// Потоковий розбирач: приймає байти шматками довільного розміру,
/// віддає цілі кадри v1 і v2, ресинхронізується на смітті.
class Framer {
public:
    static constexpr std::size_t kCapacity = 8192;

    /// Додати байти. false - буфер переповнено (потік не схожий на MAVLink).
    bool feed(const std::uint8_t* data, std::size_t n);

    /// Витягти наступний цілий кадр. false - повних кадрів більше немає.
    bool next(Frame& out);

    void        reset()        { begin_ = end_ = 0; }
    std::size_t pending() const { return end_ - begin_; }
    /// Скільки байтів відкинуто під час ресинхронізації - міра засміченості каналу.
    std::uint64_t resyncBytes() const { return resync_; }

private:
    void compact();

    std::array<std::uint8_t, kCapacity> buf_{};
    std::size_t   begin_  = 0;
    std::size_t   end_    = 0;
    std::uint64_t resync_ = 0;
};

/// Контрольна сума MAVLink: X.25 (CRC-16/MCRF4XX) по заголовку й корисних
/// даних, потім ще один байт CRC_EXTRA, що залежить від типу повідомлення.
std::uint16_t crcX25(const std::uint8_t* data, std::size_t n, std::uint8_t crcExtra);

/// Зібрати кадр MAVLink v1. Повертає довжину або 0, якщо не вмістився.
/// Лишено для перевірок і як опис формату; власні повідомлення маршрутизатор
/// складає версією 2 - див. buildV2.
std::size_t buildV1(std::uint8_t* out, std::size_t cap, std::uint32_t msgid,
                    const std::uint8_t* payload, std::uint8_t payloadLen,
                    std::uint8_t crcExtra, std::uint8_t sysid, std::uint8_t compid,
                    std::uint8_t& seq);

/// Зібрати кадр MAVLink v2. Повертає довжину або 0, якщо не вмістився.
/// Нульові байти в кінці корисних даних відкидаються, як вимагає версія 2;
/// щонайменше один байт лишається. Підпис не застосовується, тому ознака
/// несумісності нульова.
std::size_t buildV2(std::uint8_t* out, std::size_t cap, std::uint32_t msgid,
                    const std::uint8_t* payload, std::uint8_t payloadLen,
                    std::uint8_t crcExtra, std::uint8_t sysid, std::uint8_t compid,
                    std::uint8_t& seq);

/// STATUSTEXT для банера в наземній станції. Тільки ASCII: поле тексту має
/// рівно 50 байтів, і багатобайтовий символ на межі зробив би кадр сміттям.
std::size_t buildStatusText(std::uint8_t* out, std::size_t cap, const char* text,
                            std::uint8_t severity, std::uint8_t& seq);

/// COMMAND_LONG з MAV_CMD_NAV_RETURN_TO_LAUNCH - третій рівень захисту.
std::size_t buildReturnToLaunch(std::uint8_t* out, std::size_t cap, std::uint8_t& seq);

/// Розібрати корисні дані RADIO_STATUS. Порядок полів - як на дроті
/// (спочатку uint16, потім uint8 у порядку оголошення).
bool parseRadioStatus(const Frame& f, RadioStatus& out);

/// Розібрати корисні дані ESTIMATOR_STATUS. На дроті поля впорядковані за
/// спаданням розміру: uint64 часу, вісім float, потім uint16 прапорців -
/// тобто прапорці лежать зі зсувом 40, а повний кадр має 42 байти.
bool parseEstimatorStatus(const Frame& f, EstimatorStatus& out);

} // namespace uav::mavlink
