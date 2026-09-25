#include "uav/Mavlink.hpp"

#include <cstring>

namespace uav::mavlink {
namespace {

/// Один крок накопичення контрольної суми X.25.
inline std::uint16_t crcAccumulate(std::uint8_t b, std::uint16_t crc) {
    std::uint8_t tmp = static_cast<std::uint8_t>(b ^ (crc & 0xFF));
    tmp = static_cast<std::uint8_t>(tmp ^ (tmp << 4));
    return static_cast<std::uint16_t>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

} // namespace

std::uint16_t crcX25(const std::uint8_t* data, std::size_t n, std::uint8_t crcExtra) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < n; ++i) crc = crcAccumulate(data[i], crc);
    return crcAccumulate(crcExtra, crc);
}

void Framer::compact() {
    if (begin_ == 0) return;
    const std::size_t n = end_ - begin_;
    if (n != 0) std::memmove(buf_.data(), buf_.data() + begin_, n);
    begin_ = 0;
    end_   = n;
}

bool Framer::feed(const std::uint8_t* data, std::size_t n) {
    if (n == 0) return true;
    if (kCapacity - (end_ - begin_) < n) compact();
    if (kCapacity - end_ < n) {
        // Потік не схожий на MAVLink: цілого кадру немає, а місця вже нема.
        // Скидаємо все, щоб не застрягнути назавжди на смітті.
        resync_ += end_ - begin_;
        reset();
        if (n > kCapacity) { resync_ += n; return false; }
    }
    std::memcpy(buf_.data() + end_, data, n);
    end_ += n;
    return true;
}

bool Framer::next(Frame& out) {
    while (begin_ < end_) {
        const std::uint8_t* p         = buf_.data() + begin_;
        const std::size_t   available = end_ - begin_;
        const std::uint8_t  stx       = p[0];

        if (stx == kStxV2) {
            if (available < 10) return false;               // заголовок ще не повний
            const std::uint8_t len      = p[1];
            const std::uint8_t incompat = p[2];
            const std::size_t  total    = 12u + len + ((incompat & 0x01) ? 13u : 0u);
            if (available < total) return false;            // кадр ще не дочитано
            out.raw         = p;
            out.rawSize     = total;
            out.payload     = p + 10;
            out.payloadSize = len;
            out.msgid       = std::uint32_t(p[7]) | (std::uint32_t(p[8]) << 8)
                              | (std::uint32_t(p[9]) << 16);
            out.sysid       = p[5];
            begin_ += total;
            return true;
        }

        if (stx == kStxV1) {
            if (available < 6) return false;
            const std::uint8_t len   = p[1];
            const std::size_t  total = 8u + len;
            if (available < total) return false;
            out.raw         = p;
            out.rawSize     = total;
            out.payload     = p + 6;
            out.payloadSize = len;
            out.msgid       = p[5];
            out.sysid       = p[3];
            begin_ += total;
            return true;
        }

        ++begin_;    // не початок кадру - шукаємо далі
        ++resync_;
    }
    reset();
    return false;
}

std::size_t buildV1(std::uint8_t* out, std::size_t cap, std::uint32_t msgid,
                    const std::uint8_t* payload, std::uint8_t payloadLen,
                    std::uint8_t crcExtra, std::uint8_t sysid, std::uint8_t compid,
                    std::uint8_t& seq) {
    const std::size_t total = 8u + payloadLen;
    if (cap < total || msgid > 0xFF) return 0;

    out[0] = kStxV1;
    out[1] = payloadLen;
    out[2] = seq++;
    out[3] = sysid;
    out[4] = compid;
    out[5] = static_cast<std::uint8_t>(msgid);
    if (payloadLen != 0) std::memcpy(out + 6, payload, payloadLen);

    const std::uint16_t crc = crcX25(out + 1, 5u + payloadLen, crcExtra);
    out[6 + payloadLen] = static_cast<std::uint8_t>(crc & 0xFF);
    out[7 + payloadLen] = static_cast<std::uint8_t>(crc >> 8);
    return total;
}

std::size_t buildV2(std::uint8_t* out, std::size_t cap, std::uint32_t msgid,
                    const std::uint8_t* payload, std::uint8_t payloadLen,
                    std::uint8_t crcExtra, std::uint8_t sysid, std::uint8_t compid,
                    std::uint8_t& seq) {
    if (msgid > 0xFFFFFFu) return 0;

    // Версія 2 вимагає відкидати нульові байти в кінці корисних даних.
    // Щонайменше один байт має лишитися, інакше приймач не відрізнить
    // порожній кадр від пошкодженого.
    std::uint8_t len = payloadLen;
    while (len > 1 && payload[len - 1] == 0) --len;

    const std::size_t total = 12u + len;
    if (cap < total) return 0;

    out[0] = kStxV2;
    out[1] = len;
    out[2] = 0;                       // incompat_flags: підпису немає
    out[3] = 0;                       // compat_flags
    out[4] = seq++;
    out[5] = sysid;
    out[6] = compid;
    out[7] = static_cast<std::uint8_t>(msgid & 0xFF);
    out[8] = static_cast<std::uint8_t>((msgid >> 8) & 0xFF);
    out[9] = static_cast<std::uint8_t>((msgid >> 16) & 0xFF);
    if (len != 0) std::memcpy(out + 10, payload, len);

    // Сума рахується по вкороченому вмісту, а CRC_EXTRA лишається тим самим:
    // він залежить від опису повідомлення, а не від довжини кадру.
    const std::uint16_t crc = crcX25(out + 1, 9u + len, crcExtra);
    out[10 + len] = static_cast<std::uint8_t>(crc & 0xFF);
    out[11 + len] = static_cast<std::uint8_t>(crc >> 8);
    return total;
}

std::size_t buildStatusText(std::uint8_t* out, std::size_t cap, const char* text,
                            std::uint8_t severity, std::uint8_t& seq) {
    std::uint8_t payload[51] = {};
    payload[0] = severity;
    for (std::size_t i = 0; i < 50 && text[i] != '\0'; ++i) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        payload[1 + i] = (c < 0x80) ? c : '?';   // нічого, крім ASCII
    }
    // MAV_COMP_ID_UDP_BRIDGE = 190: повідомлення від маршрутизатора, не від автопілота.
    return buildV2(out, cap, kMsgStatusText, payload, sizeof(payload),
                   kCrcExtraStatusText, /*sysid=*/1, /*compid=*/190, seq);
}

std::size_t buildReturnToLaunch(std::uint8_t* out, std::size_t cap, std::uint8_t& seq) {
    std::uint8_t payload[33] = {};                 // 7 x float + uint16 + 3 x uint8
    const std::uint16_t cmd = kCmdNavReturnToLaunch;
    std::memcpy(payload + 28, &cmd, sizeof(cmd));  // param1..7 лишаються нулями
    payload[30] = 1;                               // target_system
    payload[31] = 1;                               // target_component
    payload[32] = 0;                               // confirmation
    // Від імені наземної станції (255), інакше PX4 може не прийняти команду.
    return buildV2(out, cap, kMsgCommandLong, payload, sizeof(payload),
                   kCrcExtraCommandLong, /*sysid=*/255, /*compid=*/190, seq);
}

bool parseEstimatorStatus(const Frame& f, EstimatorStatus& out) {
    if (f.msgid != kMsgEstimatorStatus) return false;

    constexpr std::size_t kFlagsOffset = 40;   // uint64 + 8 x float
    // MAVLink v2 відкидає нульові байти в кінці кадру. Прапорці - останнє
    // поле, тож кадр із нульовими прапорцями приходить на два байти
    // коротшим, а то й зовсім без них. Саме цей випадок найнебезпечніший:
    // нульові прапорці означають, що жодної ознаки не піднято, тобто
    // навігації немає. Короткий кадр треба читати як нулі, а не відкидати.
    if (f.payloadSize < 8) return false;       // це вже не ESTIMATOR_STATUS

    std::uint16_t flags = 0;
    if (f.payloadSize >= kFlagsOffset + 2) {
        std::memcpy(&flags, f.payload + kFlagsOffset, sizeof(flags));
    } else if (f.payloadSize == kFlagsOffset + 1) {
        flags = f.payload[kFlagsOffset];       // старший байт відкинуто як нульовий
    }
    out.flags = flags;
    return true;
}

bool parseRadioStatus(const Frame& f, RadioStatus& out) {
    if (f.msgid != kMsgRadioStatus || f.payloadSize < 9) return false;
    std::memcpy(&out.rxerrors, f.payload + 0, 2);
    std::memcpy(&out.fixed,    f.payload + 2, 2);
    out.rssi     = f.payload[4];
    out.remrssi  = f.payload[5];
    out.txbuf    = f.payload[6];
    out.noise    = f.payload[7];
    out.remnoise = f.payload[8];
    return true;
}

} // namespace uav::mavlink
