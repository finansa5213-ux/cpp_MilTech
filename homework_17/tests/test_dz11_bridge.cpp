#include <gtest/gtest.h>

#include <common/mavlink.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <vector>

#include "dz11_bridge.hpp"
#include "geo.hpp"

namespace {

class FakeTransport final : public mavtel::ITransport {
public:
    bool send(const std::uint8_t* data, std::size_t len) override {
        sent.emplace_back(data, data + len);
        return true;
    }

    std::size_t receive(std::uint8_t* buffer, std::size_t capacity) override {
        if (inbox.empty()) return 0;
        const std::vector<std::uint8_t> frame = inbox.front();
        inbox.pop_front();
        const std::size_t n = std::min(capacity, frame.size());
        std::memcpy(buffer, frame.data(), n);
        return n;
    }

    void pushCommandAck(std::uint16_t command, std::uint8_t result) {
        mavlink_message_t message{};
        mavlink_msg_command_ack_pack(255, 190, &message, command, result, 0, 0, 1, 1);
        std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const std::uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
        inbox.emplace_back(buffer, buffer + len);
    }

    std::vector<std::vector<std::uint8_t>> sent;
    std::deque<std::vector<std::uint8_t>> inbox;
};

bool decodeFrame(const std::vector<std::uint8_t>& frame, mavlink_message_t& out) {
    mavlink_status_t status{};
    for (const std::uint8_t byte : frame) {
        if (mavlink_parse_char(MAVLINK_COMM_3, byte, &out, &status) == 1) return true;
    }
    return false;
}

int countMessages(const FakeTransport& transport, std::uint32_t msgid) {
    int count = 0;
    for (const auto& frame : transport.sent) {
        mavlink_message_t message{};
        if (decodeFrame(frame, message) && message.msgid == msgid) ++count;
    }
    return count;
}

/// Кути порівнюємо за кутовою відстанню: 359.9999 і 0.0 — це та сама точка
/// на компасі, а float-округлення pi/2 легко дає саме такий випадок.
::testing::AssertionResult headingNear(double actual, double expected, double tol) {
    const double diff = std::fabs(mavtel::wrapDeg180(actual - expected));
    if (diff <= tol) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure()
           << "kurs " << actual << " vs ochikuvano " << expected
           << " (kutova riznytsya " << diff << " > " << tol << ")";
}

/// Кадр ДЗ11: dir — математичний кут від осі X проти годинникової.
dlink::Telemetry makeTelemetry(std::uint32_t t_ms, float x, float y, float z,
                               float vx, float vy, float dir) {
    dlink::Telemetry t{};
    t.t_ms = t_ms;
    t.x = x;
    t.y = y;
    t.z = z;
    t.vx = vx;
    t.vy = vy;
    t.speed = std::hypot(vx, vy);
    t.dir = dir;
    t.state = 2;
    return t;
}

// ---------------------------------------------------------------------------
// Конверсія кадру
// ---------------------------------------------------------------------------

TEST(Dz11BridgeTest, MapsAxesAndUnits) {
    const dlink::Telemetry t = makeTelemetry(4321, 100.0F, 250.0F, 120.0F,
                                             10.0F, 0.0F, 0.0F);
    const mavtel::TelemetrySample s = mavtel::toSample(t);

    EXPECT_EQ(s.time_boot_ms, 4321U);
    EXPECT_DOUBLE_EQ(s.x_east_m, 100.0);
    EXPECT_DOUBLE_EQ(s.y_north_m, 250.0);
    EXPECT_DOUBLE_EQ(s.alt_m, 120.0);
    EXPECT_DOUBLE_EQ(s.vx_east_mps, 10.0);
    EXPECT_DOUBLE_EQ(s.vy_north_mps, 0.0);
}

TEST(Dz11BridgeTest, HeadingComesFromVelocityNotFromDir) {
    // Рух строго на схід -> компасний курс 90 градусів,
    // хоча математичний dir = 0.
    const dlink::Telemetry east = makeTelemetry(0, 0, 0, 100, 20.0F, 0.0F, 0.0F);
    EXPECT_TRUE(headingNear(mavtel::toSample(east).heading_deg, 90.0, 1e-4));

    // Рух на північ -> курс 0, математичний dir = pi/2.
    const dlink::Telemetry north =
        makeTelemetry(0, 0, 0, 100, 0.0F, 20.0F, static_cast<float>(mavtel::kPi / 2));
    EXPECT_TRUE(headingNear(mavtel::toSample(north).heading_deg, 0.0, 1e-4));

    // Рух на захід -> курс 270.
    const dlink::Telemetry west =
        makeTelemetry(0, 0, 0, 100, -20.0F, 0.0F, static_cast<float>(mavtel::kPi));
    EXPECT_TRUE(headingNear(mavtel::toSample(west).heading_deg, 270.0, 1e-4));
}

TEST(Dz11BridgeTest, FallsBackToDirWhenStandingStill) {
    // Швидкість нульова: курс береться з dir = pi/2 -> компасний 0.
    const dlink::Telemetry t =
        makeTelemetry(0, 0, 0, 100, 0.0F, 0.0F, static_cast<float>(mavtel::kPi / 2));
    EXPECT_TRUE(headingNear(mavtel::toSample(t).heading_deg, 0.0, 1e-4));
}

// ---------------------------------------------------------------------------
// Темпи і потік
// ---------------------------------------------------------------------------

TEST(Dz11BridgeTest, EmitsTelemetryPairPerFrame) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    bridge.onTelemetry(makeTelemetry(100, 10, 20, 120, 15, 0, 0), 100);

    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_GLOBAL_POSITION_INT), 1);
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_ATTITUDE), 1);
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_HEARTBEAT), 1);
}

TEST(Dz11BridgeTest, HeartbeatIsOncePerSecondRegardlessOfFrameRate) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    // 20 Гц телеметрії протягом 3 секунд -> 60 кадрів, але 4 HEARTBEAT
    // (нульовий + на 1000, 2000, 3000 мс).
    for (std::uint32_t ms = 0; ms <= 3000; ms += 50) {
        bridge.onTelemetry(makeTelemetry(ms, 0, 0, 120, 20, 0, 0), ms);
    }

    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_GLOBAL_POSITION_INT), 61);
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_HEARTBEAT), 4);
}

TEST(Dz11BridgeTest, TickKeepsHeartbeatAliveWithoutTelemetry) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    // Телеметрія ДЗ11 ще не пішла — цикл місії крутиться на тайм-аутах.
    for (std::uint32_t ms = 0; ms <= 2000; ms += 500) bridge.tick(ms);

    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_HEARTBEAT), 3);
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_GLOBAL_POSITION_INT), 0);
}

// ---------------------------------------------------------------------------
// Скид
// ---------------------------------------------------------------------------

TEST(Dz11BridgeTest, DropCarriesDronePositionInDegrees) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    // Рівно один градус широти на північ від опорної точки.
    const auto north = static_cast<float>(mavtel::kMetersPerDegLat);
    bridge.onDrop(makeTelemetry(5000, 0.0F, north, 90.0F, 0.0F, 20.0F, 0.0F), 5000);

    ASSERT_EQ(transport.sent.size(), 1U);
    mavlink_message_t message{};
    ASSERT_TRUE(decodeFrame(transport.sent[0], message));
    ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_COMMAND_LONG);

    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    EXPECT_EQ(command.command, MAV_CMD_USER_1);
    EXPECT_NEAR(command.param5, mavtel::kLat0Deg + 1.0, 1e-4);
    EXPECT_NEAR(command.param6, mavtel::kLon0Deg, 1e-4);
    EXPECT_FLOAT_EQ(command.param7, 90.0F);
}

TEST(Dz11BridgeTest, CoastFinishesRetriesAfterDz11GoesQuiet) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    bridge.onTelemetry(makeTelemetry(1000, 0, 0, 120, 20, 0, 0), 1000);
    bridge.onDrop(makeTelemetry(1000, 0, 0, 120, 20, 0, 0), 1000);

    // Телеметрія ДЗ11 обірвалась одразу після скиду — далі лише вибіг.
    for (std::uint32_t ms = 1020; ms <= 5000; ms += 20) bridge.coast(ms);

    EXPECT_EQ(bridge.dropState(), mavtel::DropState::Failed);
    EXPECT_EQ(bridge.dropAttempts(), mavtel::MavlinkLink::kMaxDropAttempts);
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG),
              mavtel::MavlinkLink::kMaxDropAttempts);
}

TEST(Dz11BridgeTest, CoastStopsRetriesOnAck) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    bridge.onTelemetry(makeTelemetry(1000, 0, 0, 120, 20, 0, 0), 1000);
    bridge.onDrop(makeTelemetry(1000, 0, 0, 120, 20, 0, 0), 1000);

    for (std::uint32_t ms = 1020; ms <= 1600; ms += 20) bridge.coast(ms);
    ASSERT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG), 2);

    transport.pushCommandAck(MAV_CMD_USER_1, MAV_RESULT_ACCEPTED);
    for (std::uint32_t ms = 1620; ms <= 5000; ms += 20) bridge.coast(ms);

    EXPECT_EQ(bridge.dropState(), mavtel::DropState::Acked);
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG), 2);
}

TEST(Dz11BridgeTest, CoastKeepsPositionConsistentWithVelocity) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    mavtel::Dz11Bridge bridge(link);

    // Летимо строго на схід 20 м/с.
    bridge.onTelemetry(makeTelemetry(1000, 0, 0, 120, 20, 0, 0), 1000);
    const double x0 = bridge.lastSample().x_east_m;

    for (std::uint32_t ms = 1020; ms <= 2000; ms += 20) bridge.coast(ms);

    const mavtel::TelemetrySample& s = bridge.lastSample();
    const double dt_s = (s.time_boot_ms - 1000) / 1000.0;
    EXPECT_NEAR(s.x_east_m - x0, 20.0 * dt_s, 1e-6); // dx = v * dt
    EXPECT_NEAR(s.y_north_m, 0.0, 1e-6);
    EXPECT_TRUE(headingNear(s.heading_deg, 90.0, 1e-6)); // курс не поплив
    EXPECT_GE(s.time_boot_ms, 1000U);                // час монотонний
}

} // namespace
