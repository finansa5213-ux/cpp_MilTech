// drone_link.h — бінарний UART-протокол ДЗ11 (заняття 4.4)
// Спільний для симулятора фізики, чекера і програми студента.
//
// Кадр на дроті:
//   [0]  MAGIC0 = 0xA5
//   [1]  MAGIC1 = 0x5A
//   [2]  TYPE  (1 байт)      — тип пакета (PacketType)
//   [3]  LEN   (1 байт)      — довжина payload у байтах
//   [4..4+LEN-1]  payload    — корисні дані (little-endian)
//   [..] CRC16 (2 байти, LE) — CRC-16/CCITT-FALSE по TYPE+LEN+payload
//
// Усі multi-byte поля — little-endian (як на ARM/x86 за замовчуванням).
// Кадр самосинхронізується: приймач шукає MAGIC0,MAGIC1, потім читає LEN і CRC.

#ifndef DRONE_LINK_H
#define DRONE_LINK_H

#include <cstdint>
#include <cstring>

namespace dlink {

constexpr uint8_t MAGIC0 = 0xA5;
constexpr uint8_t MAGIC1 = 0x5A;

enum PacketType : uint8_t {
    PKT_TELEMETRY = 0x01, // стан дрона в момент часу
    PKT_TARGET    = 0x02, // позиція однієї цілі (шлеться періодично по кожній цілі)
    PKT_AMMO      = 0x03, // параметри боєприпасу + hitRadius (один раз на старті)
    PKT_RESULT    = 0x04, // вердикт чекера (HIT/MISS) — лише на реальній малині по UART назад
    PKT_CONTROL   = 0x05, // КЕРУВАННЯ: студент -> чекер (прискорення і швидкість повороту)
    PKT_CONFIG    = 0x06, // параметри місії з config
};

#pragma pack(push, 1)

// PKT_TELEMETRY — те, що читає і парсить студент
struct Telemetry {
    uint32_t t_ms;   // час від старту, мілісекунди (таймстемп)
    float    x, y;   // позиція дрона в площині, метри
    float    z;      // висота (altitude), метри
    float    vx, vy; // швидкість у площині, м/с
    float    speed;  // модуль горизонтальної швидкості, м/с
    float    dir;    // курс (напрямок польоту), радіани
    uint8_t  state;  // стан стейт-машини (0..4, як у DZ3)
};

// PKT_TARGET — позиція цілі «зараз» (ціль може рухатися)
struct TargetPos {
    uint8_t id;   // індекс цілі
    float   x, y; // поточна позиція цілі, метри
};

// PKT_AMMO — конфіг пострілу (надсилається раз на старті)
struct AmmoCfg {
    char    name[16]; // напр. "VOG-17"
    float   mass;     // m
    float   drag;     // d
    float   lift;     // l
    float   hitRadius;// радіус успішного влучання, метри
    uint8_t nTargets; // скільки цілей у місії
};

// PKT_RESULT — вердикт (зворотний канал на залізі)
struct Result {
    uint8_t  hit;      // 1 = влучив, 0 = промах
    uint8_t  targetId; // у яку ціль (або 0xFF)
    float    miss_m;   // відстань промаху, метри
    uint32_t drop_t_ms;// коли спрацював скид
};

// PKT_CONTROL — команда керування дроном (студент шле чекеру кожен такт)
// Нормовані значення; чекер множить на фізичні ліміти дрона (maxAccel, maxTurnRate).
struct Control {
    float accel;    // прискорення вздовж курсу, [-1..1] (1 = повний газ, -1 = гальмо)
    float turnRate; // швидкість повороту, [-1..1] (1 = макс. вліво, -1 = вправо)
};

// PKT_CONFIG — параметри місії з config (чекер шле студенту раз на старті, як AMMO).
struct DroneCfg {
    float attackSpeed;      // макс. швидкість дрона, м/с
    float accelerationPath; // шлях розгону до attackSpeed, м
    float angularSpeed;     // макс. кутова швидкість повороту, рад/с
    float turnThreshold;    // поріг кута повороту, рад
    float timeStep;         // крок симуляції, с
    float timeScale;        // прискорення симуляції (1 = реальний час)
};

#pragma pack(pop)

// ---- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ----
inline uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// Запакувати кадр у буфер out (повертає кількість записаних байтів).
// out має бути >= 6 + payloadLen.
inline size_t encode(uint8_t type, const void* payload, uint8_t payloadLen,
                     uint8_t* out) {
    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = type;
    out[3] = payloadLen;
    if (payloadLen && payload) std::memcpy(out + 4, payload, payloadLen);
    uint16_t c = crc16(out + 2, (size_t)payloadLen + 2); // по TYPE+LEN+payload
    out[4 + payloadLen]     = (uint8_t)(c & 0xFF);
    out[4 + payloadLen + 1] = (uint8_t)(c >> 8);
    return (size_t)payloadLen + 6;
}

// Інкрементальний парсер: годуєш байтами, на готовому валідному кадрі feed() повертає true.
struct Parser {
    enum { S_M0, S_M1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 } st = S_M0;
    uint8_t  type = 0, len = 0, idx = 0;
    uint8_t  buf[260];
    uint16_t crc_rx = 0;

    // Повертає true, коли зібрано повний валідний кадр.
    // type/payload/payloadLen — вихідні (валідні лише при true).
    bool feed(uint8_t byte, uint8_t& outType, uint8_t* outPayload, uint8_t& outLen) {
        switch (st) {
        case S_M0:   if (byte == MAGIC0) st = S_M1; break;
        case S_M1:   st = (byte == MAGIC1) ? S_TYPE : S_M0; break;
        case S_TYPE: type = byte; st = S_LEN; break;
        case S_LEN:  len = byte; idx = 0; st = len ? S_PAYLOAD : S_CRC0; break;
        case S_PAYLOAD:
            buf[idx++] = byte;
            if (idx >= len) st = S_CRC0;
            break;
        case S_CRC0: crc_rx = byte; st = S_CRC1; break;
        case S_CRC1: {
            crc_rx |= (uint16_t)byte << 8;
            st = S_M0;
            // перерахувати CRC по TYPE+LEN+payload
            uint8_t tmp[262];
            tmp[0] = type; tmp[1] = len;
            std::memcpy(tmp + 2, buf, len);
            if (crc16(tmp, (size_t)len + 2) == crc_rx) {
                outType = type; outLen = len;
                std::memcpy(outPayload, buf, len);
                return true;
            }
            break; // биті дані — кадр відкинуто, синхронізуємось далі
        }
        }
        return false;
    }
};

} // namespace dlink
#endif // DRONE_LINK_H
