#include <gtest/gtest.h>

#include <common/mavlink.h>

#include <cstring>
#include <deque>
#include <vector>

#include "geo.hpp"
#include "mavlink_link.hpp"

namespace {

/// Транспорт-заглушка: запам'ятовує надіслані кадри й дозволяє «вкинути» вхідні.
class FakeTransport final : public mavtel::ITransport {
public:
    bool send(const std::uint8_t* data, std::size_t len) override {
        sent.emplace_back(data, data + len);
        return true;
    }

    std::size_t receive(std::uint8_t* buffer, std::size_t capacity) override {
        if (inbox.empty()) {
            return 0;
        }
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

/// Розбирає кадр із буфера у mavlink_message_t. Використовує окремий канал,
/// щоб не чіпати стан tx/rx самого MavlinkLink.
bool decodeFrame(const std::vector<std::uint8_t>& frame, mavlink_message_t& out) {
    mavlink_status_t status{};
    for (const std::uint8_t byte : frame) {
        if (mavlink_parse_char(MAVLINK_COMM_2, byte, &out, &status) == 1) {
            return true;
        }
    }
    return false;
}

int countMessages(const FakeTransport& transport, std::uint32_t msgid) {
    int count = 0;
    for (const auto& frame : transport.sent) {
        mavlink_message_t message{};
        if (decodeFrame(frame, message) && message.msgid == msgid) {
            ++count;
        }
    }
    return count;
}

TEST(MavlinkLinkTest, FramesAreMavlink2WithStableIds) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);
    link.sendHeartbeat();

    ASSERT_EQ(transport.sent.size(), 1U);
    EXPECT_EQ(transport.sent[0][0], 0xFD); // магічний байт MAVLink 2

    mavlink_message_t message{};
    ASSERT_TRUE(decodeFrame(transport.sent[0], message));
    EXPECT_EQ(message.msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(message.sysid, 1);
    EXPECT_EQ(message.compid, MAV_COMP_ID_AUTOPILOT1);

    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    EXPECT_EQ(heartbeat.type, MAV_TYPE_QUADROTOR);
    EXPECT_EQ(heartbeat.system_status, MAV_STATE_ACTIVE);
}

TEST(MavlinkLinkTest, TelemetryUsesCorrectUnits) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    mavtel::TelemetrySample sample{};
    sample.time_boot_ms = 1234;
    sample.x_east_m = 0.0;
    sample.y_north_m = mavtel::kMetersPerDegLat; // рівно +1 градус широти
    sample.alt_m = 120.0;
    sample.vx_east_mps = 0.0;
    sample.vy_north_mps = 25.0;
    sample.heading_deg = 90.0;

    link.sendTelemetry(sample);
    ASSERT_EQ(transport.sent.size(), 2U); // GLOBAL_POSITION_INT + ATTITUDE

    mavlink_message_t message{};
    ASSERT_TRUE(decodeFrame(transport.sent[0], message));
    ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);

    mavlink_global_position_int_t position{};
    mavlink_msg_global_position_int_decode(&message, &position);
    EXPECT_EQ(position.time_boot_ms, 1234U);
    EXPECT_NEAR(position.lat / 1e7, mavtel::kLat0Deg + 1.0, 1e-6); // градуси x1e7
    EXPECT_EQ(position.alt, 120000);                               // мм
    EXPECT_EQ(position.relative_alt, 120000);
    EXPECT_EQ(position.vx, 2500);  // північ, см/с
    EXPECT_EQ(position.vy, 0);     // схід
    EXPECT_EQ(position.vz, 0);     // вниз
    EXPECT_EQ(position.hdg, 9000); // соті градуса

    ASSERT_TRUE(decodeFrame(transport.sent[1], message));
    ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_ATTITUDE);
    mavlink_attitude_t attitude{};
    mavlink_msg_attitude_decode(&message, &attitude);
    EXPECT_NEAR(attitude.yaw, mavtel::kPi / 2.0, 1e-5);
    EXPECT_FLOAT_EQ(attitude.roll, 0.0F);
    EXPECT_FLOAT_EQ(attitude.pitch, 0.0F);
}

TEST(MavlinkLinkTest, DropCommandCarriesCoordinates) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    link.requestDrop({50.4521, 30.5277, 120.0}, 0);
    ASSERT_EQ(transport.sent.size(), 1U);

    mavlink_message_t message{};
    ASSERT_TRUE(decodeFrame(transport.sent[0], message));
    ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_COMMAND_LONG);

    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    EXPECT_EQ(command.command, MAV_CMD_USER_1);
    EXPECT_NEAR(command.param5, 50.4521, 1e-4);
    EXPECT_NEAR(command.param6, 30.5277, 1e-4);
    EXPECT_FLOAT_EQ(command.param7, 120.0F);
    EXPECT_EQ(link.dropState(), mavtel::DropState::WaitingAck);
}

TEST(MavlinkLinkTest, RepeatsCommandUntilMaxAttemptsWhenNoAck) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    link.requestDrop({50.45, 30.52, 100.0}, 0);
    for (std::uint32_t t = 0; t <= 5000; t += 100) {
        link.poll(t);
    }

    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG),
              mavtel::MavlinkLink::kMaxDropAttempts);
    EXPECT_EQ(link.dropAttempts(), mavtel::MavlinkLink::kMaxDropAttempts);
    EXPECT_EQ(link.dropState(), mavtel::DropState::Failed);
}

TEST(MavlinkLinkTest, StopsAfterAckAndIgnoresLateTime) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    link.requestDrop({50.45, 30.52, 100.0}, 0);
    link.poll(600); // таймаут -> друга спроба
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG), 2);

    transport.pushCommandAck(MAV_CMD_USER_1, MAV_RESULT_ACCEPTED);
    link.poll(700);
    EXPECT_EQ(link.dropState(), mavtel::DropState::Acked);

    for (std::uint32_t t = 800; t <= 6000; t += 100) {
        link.poll(t);
    }
    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG), 2); // після ACK — тиша
}

TEST(MavlinkLinkTest, IgnoresAckForOtherCommand) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    link.requestDrop({50.45, 30.52, 100.0}, 0);
    transport.pushCommandAck(MAV_CMD_NAV_TAKEOFF, MAV_RESULT_ACCEPTED);
    link.poll(100);

    EXPECT_EQ(link.dropState(), mavtel::DropState::WaitingAck);
}

TEST(MavlinkLinkTest, IgnoresNonAcceptedResult) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    link.requestDrop({50.45, 30.52, 100.0}, 0);
    transport.pushCommandAck(MAV_CMD_USER_1, MAV_RESULT_TEMPORARILY_REJECTED);
    link.poll(100);

    EXPECT_EQ(link.dropState(), mavtel::DropState::WaitingAck);
}

TEST(MavlinkLinkTest, DropIsRequestedOnlyOnce) {
    FakeTransport transport;
    mavtel::MavlinkLink link(transport);

    link.requestDrop({50.45, 30.52, 100.0}, 0);
    link.requestDrop({51.00, 31.00, 200.0}, 10); // повторний виклик має бути проігнорований

    EXPECT_EQ(countMessages(transport, MAVLINK_MSG_ID_COMMAND_LONG), 1);
    EXPECT_EQ(link.dropAttempts(), 1);
}

} // namespace
