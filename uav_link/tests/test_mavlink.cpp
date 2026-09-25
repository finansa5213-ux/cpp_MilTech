// Перевірка розбирача й збирача кадрів.
// Еталонні байти згенеровано незалежно й звірено з pymavlink - бібліотекою,
// яку використовує сам PX4 для генерації своїх повідомлень.
#include "uav/Mavlink.hpp"
#include "tests/Check.hpp"

#include <cstring>
#include <vector>

using namespace uav;
using namespace uav::mavlink;

namespace {

// STATUSTEXT("LINK -> A: B lost", severity 4), seq=7, sysid=1, compid=190.
// Версія 2: нульовий хвіст поля тексту відкинуто, тому 30 Б замість 59.
const std::uint8_t kStatusTextRef[] = {
    0xFD, 0x12, 0x00, 0x00, 0x07, 0x01, 0xBE, 0xFD, 0x00, 0x00, 0x04, 0x4C,
    0x49, 0x4E, 0x4B, 0x20, 0x2D, 0x3E, 0x20, 0x41, 0x3A, 0x20, 0x42, 0x20,
    0x6C, 0x6F, 0x73, 0x74, 0xAB, 0x73};

// COMMAND_LONG з MAV_CMD_NAV_RETURN_TO_LAUNCH, seq=9, sysid=255, compid=190.
// Версія 2: нульове поле confirmation у хвості відкинуто.
const std::uint8_t kRtlRef[] = {
    0xFD, 0x20, 0x00, 0x00, 0x09, 0xFF, 0xBE, 0x4C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x14, 0x00, 0x01, 0x01, 0xCB, 0xA0};

/// Зібрати кадр v1 вручну - джерело тестових даних для розбирача.
std::vector<std::uint8_t> frameV1(std::uint8_t msgid, std::uint8_t len) {
    std::vector<std::uint8_t> v(8u + len, 0);
    v[0] = 0xFE; v[1] = len; v[2] = 0; v[3] = 1; v[4] = 1; v[5] = msgid;
    return v;
}

std::vector<std::uint8_t> frameV2(std::uint32_t msgid, std::uint8_t len) {
    std::vector<std::uint8_t> v(12u + len, 0);
    v[0] = 0xFD; v[1] = len; v[2] = 0; v[3] = 0; v[4] = 0; v[5] = 1; v[6] = 1;
    v[7] = static_cast<std::uint8_t>(msgid & 0xFF);
    v[8] = static_cast<std::uint8_t>((msgid >> 8) & 0xFF);
    v[9] = static_cast<std::uint8_t>((msgid >> 16) & 0xFF);
    return v;
}

} // namespace

int main() {
    // --- 1. Збирач кадрів проти еталона -------------------------------
    {
        std::uint8_t buf[128];
        std::uint8_t seq = 7;
        const std::size_t n = buildStatusText(buf, sizeof(buf), "LINK -> A: B lost", 4, seq);
        CHECK_EQ(n, sizeof(kStatusTextRef), "STATUSTEXT: довжина кадру версії 2");
        CHECK(std::memcmp(buf, kStatusTextRef, n) == 0, "STATUSTEXT: байт у байт з еталоном");
        CHECK_EQ(seq, 8u, "лічильник послідовності просунувся");
    }
    {
        std::uint8_t buf[128];
        std::uint8_t seq = 9;
        const std::size_t n = buildReturnToLaunch(buf, sizeof(buf), seq);
        CHECK_EQ(n, sizeof(kRtlRef), "COMMAND_LONG: довжина кадру версії 2");
        CHECK(std::memcmp(buf, kRtlRef, n) == 0, "COMMAND_LONG: байт у байт з еталоном");
    }

    // --- 2. Не-ASCII не псує кадр -------------------------------------
    {
        std::uint8_t buf[128];
        std::uint8_t seq = 0;
        const std::size_t n = buildStatusText(buf, sizeof(buf), "канал А", 4, seq);
        // 7 літер кирилиці дають 13 байтів UTF-8; кожен байт понад 0x7F
        // замінюється на '?', тож текст лишається 13-байтовим: 12 + 1 + 13 = 26.
        CHECK_EQ(n, 26u, "кожен байт кирилиці замінено на один ASCII");
        bool ascii = true;
        for (std::size_t i = 11; i < n - 2; ++i) if (buf[i] >= 0x80) ascii = false;
        CHECK(ascii, "у полі тексту лише ASCII, кирилицю замінено");
    }

    // --- 2а. Властивості кадру версії 2 -------------------------------
    {
        std::uint8_t buf[128];
        std::uint8_t seq = 0;
        const std::size_t n = buildStatusText(buf, sizeof(buf), "ok", 6, seq);
        CHECK_EQ(buf[0], kStxV2, "власні кадри складаються версією 2");
        CHECK_EQ(buf[2], 0u, "ознака несумісності нульова: підпису немає");
        CHECK_EQ(n, 12u + buf[1], "довжина кадру = 12 плюс вміст");
        CHECK(buf[1] < 51u, "нульовий хвіст корисних даних відкинуто");
        const std::uint32_t id = std::uint32_t(buf[7]) | (std::uint32_t(buf[8]) << 8)
                                 | (std::uint32_t(buf[9]) << 16);
        CHECK_EQ(id, kMsgStatusText, "ідентифікатор займає три байти");
    }

    // --- 2б. Зібраний кадр проходить власний розбирач ------------------
    {
        std::uint8_t buf[128];
        std::uint8_t seq = 3;
        const std::size_t n = buildStatusText(buf, sizeof(buf), "round trip", 6, seq);
        Framer fr;
        CHECK(fr.feed(buf, n), "зібраний кадр прийнято розбирачем");
        Frame f;
        CHECK(fr.next(f), "розбирач знайшов кадр");
        CHECK_EQ(f.msgid, kMsgStatusText, "розібрано той самий тип");
        CHECK_EQ(f.rawSize, n, "довжина збігається зі зібраною");
        const std::uint16_t crc = crcX25(buf + 1, 9u + f.payloadSize, kCrcExtraStatusText);
        CHECK_EQ(buf[n - 2], std::uint8_t(crc & 0xFF), "молодший байт контрольної суми");
        CHECK_EQ(buf[n - 1], std::uint8_t(crc >> 8), "старший байт контрольної суми");
    }

    // --- 2в. Суцільно нульові дані не дають кадру без вмісту -----------
    {
        std::uint8_t buf[64];
        std::uint8_t payload[8] = {};
        std::uint8_t seq = 0;
        const std::size_t n = buildV2(buf, sizeof(buf), kMsgStatusText, payload,
                                      sizeof(payload), kCrcExtraStatusText, 1, 190, seq);
        CHECK_EQ(n, 13u, "щонайменше один байт вмісту лишається");
        CHECK_EQ(buf[1], 1u, "оголошена довжина дорівнює одиниці");
    }

    // --- 3. Розбирач на рваному потоці --------------------------------
    {
        std::vector<std::uint8_t> stream;
        const std::uint32_t expect[] = {0, 30, 253, 109};
        for (int i = 0; i < 40; ++i) {
            const auto f = (i % 4 == 1) ? frameV2(30, 28)
                         : (i % 4 == 2) ? frameV2(253, 51)
                         : (i % 4 == 3) ? frameV2(109, 9)
                                        : frameV2(0, 9);
            stream.insert(stream.end(), f.begin(), f.end());
        }
        Framer framer;
        Frame  f;
        bool   fed = true;
        std::vector<std::uint32_t> got;
        for (std::size_t i = 0; i < stream.size(); i += 7) {
            const std::size_t chunk = std::min<std::size_t>(7, stream.size() - i);
            if (!framer.feed(stream.data() + i, chunk)) fed = false;
            while (framer.next(f)) got.push_back(f.msgid);
        }
        CHECK(fed, "буфер розбирача жодного разу не переповнився");
        CHECK_EQ(got.size(), 40u, "усі 40 кадрів знайдено в потоці по 7 байтів");
        bool order = true;
        for (std::size_t i = 0; i < got.size(); ++i)
            if (got[i] != expect[i % 4]) order = false;
        CHECK(order, "порядок повідомлень збережено");
    }

    // --- 4. Ресинхронізація на смітті ---------------------------------
    {
        std::vector<std::uint8_t> mixed = {0x00, 0x11, 0x22};
        const auto a = frameV1(0, 9);
        mixed.insert(mixed.end(), a.begin(), a.end());
        mixed.push_back(0x7F);
        const auto b = frameV2(30, 28);
        mixed.insert(mixed.end(), b.begin(), b.end());

        Framer framer;
        Frame  f;
        std::vector<std::uint32_t> got;
        framer.feed(mixed.data(), mixed.size());
        while (framer.next(f)) got.push_back(f.msgid);
        CHECK_EQ(got.size(), 2u, "сміття між кадрами не заважає");
        CHECK(got[0] == 0 && got[1] == 30, "знайдено і v1, і v2");
        CHECK_EQ(framer.resyncBytes(), 4u, "відкинуто рівно 4 сміттєві байти");
    }

    // --- 5. Кадр, розірваний навпіл -----------------------------------
    {
        const auto  raw = frameV2(0, 9);
        Framer      framer;
        Frame       f;
        framer.feed(raw.data(), 5);
        CHECK(!framer.next(f), "неповний кадр не віддається");
        framer.feed(raw.data() + 5, raw.size() - 5);
        CHECK(framer.next(f), "дочитаний кадр з'являється");
        CHECK_EQ(f.msgid, 0u, "msgid правильний");
        CHECK(!framer.next(f), "більше кадрів немає");
    }

    // --- 6. RADIO_STATUS -----------------------------------------------
    {
        auto raw = frameV2(109, 9);
        const std::uint16_t rxerrors = 3, fixed = 0;
        std::memcpy(raw.data() + 10, &rxerrors, 2);
        std::memcpy(raw.data() + 12, &fixed, 2);
        raw[14] = 220;  // rssi
        raw[15] = 220;  // remrssi
        raw[16] = 100;  // txbuf
        raw[17] = 49;   // noise
        raw[18] = 45;   // remnoise

        Framer framer; Frame f;
        framer.feed(raw.data(), raw.size());
        CHECK(framer.next(f), "кадр RADIO_STATUS знайдено");
        RadioStatus rs;
        CHECK(parseRadioStatus(f, rs), "поля розібрано");
        CHECK_EQ(rs.rssi, 220u, "rssi");
        CHECK_EQ(rs.txbuf, 100u, "txbuf");
        CHECK_EQ(rs.rxerrors, 3u, "rxerrors");
        CHECK_NEAR(rs.marginDb(), 90.0, 0.6, "запас за сигналом ~90 дБ");
        CHECK_NEAR(rs.rssiDbm(), -11.2, 0.1, "рівень -11,2 дБм");
    }

    // --- ESTIMATOR_STATUS: чи є взагалі куди повертатися ------------------
    // Розкладка звірена з pymavlink: <QffffffffH, 42 байти, прапорці лежать
    // зі зсувом 40. Значення 1189 - справжнє, зняте зі стенда 20.09.2026 під
    // час підміни GNSS: кути орієнтації, вертикальна швидкість і висота
    // дійсні, горизонтальні координата й швидкість - ні, увімкнено режим
    // сталої позиції та піднято ознаку збою GPS.
    {
        auto put = [](std::uint16_t flags, std::uint8_t len) {
            auto raw = frameV2(230, len);
            if (len >= 42)       std::memcpy(raw.data() + 50, &flags, 2);
            else if (len == 41)  raw[50] = static_cast<std::uint8_t>(flags & 0xFF);
            return raw;
        };
        auto parse = [](const std::vector<std::uint8_t>& raw, EstimatorStatus& es) {
            Framer fr; Frame f;
            fr.feed(raw.data(), raw.size());
            return fr.next(f) && parseEstimatorStatus(f, es);
        };

        EstimatorStatus es;
        CHECK(parse(put(1189, 42), es), "ESTIMATOR_STATUS розібрано");
        CHECK_EQ(es.flags, 1189u, "прапорці прочитано зі зсуву 40");
        CHECK(!es.navUsable(), "стан 1189: абсолютної координати немає - повертатися нікуди");
        CHECK(es.constPos(),   "стан 1189: режим сталої позиції");
        CHECK(es.gpsGlitch(),  "стан 1189: піднято ознаку збою GPS");

        CHECK(parse(put(63, 42), es) && es.navUsable(),
              "стан 63: абсолютна координата дійсна - повернення можливе");
        CHECK(!es.gpsGlitch(), "стан 63: збою GPS немає");

        // MAVLink v2 відкидає нульові байти в кінці кадру, а прапорці - останнє
        // поле. Тому найнебезпечніший стан (жодної ознаки не піднято) приходить
        // коротшим кадром, і відкинути його означало б не помітити втрату
        // навігації саме тоді, коли вона сталася.
        CHECK(parse(put(0, 40), es), "кадр без прапорців розібрано, а не відкинуто");
        CHECK_EQ(es.flags, 0u, "відкинуті байти прочитано як нулі");
        CHECK(!es.navUsable(), "порожні прапорці = навігація непридатна");

        CHECK(parse(put(0x0021, 41), es) && es.flags == 0x21u,
              "кадр з одним відкинутим байтом прочитано правильно");

        EstimatorStatus other;
        CHECK(!parse(frameV2(0, 9), other),   "HEARTBEAT не приймається за ESTIMATOR_STATUS");
        CHECK(!parse(frameV2(230, 4), other), "надто короткий кадр відкинуто");
    }

    return uav::test::summary("Mavlink");
}
