#include "mavlink_link.hpp"

#include <common/mavlink.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "geo.hpp"

namespace mavtel {
namespace {

/// Канал для передавання (його статус використовує mavlink_msg_*_pack).
constexpr mavlink_channel_t kTxChannel = MAVLINK_COMM_0;
/// Окремий канал для приймання, щоб rx-парсер не заважав tx-лічильнику seq.
constexpr mavlink_channel_t kRxChannel = MAVLINK_COMM_1;

/// Скільки датаграм максимум обробляємо за один poll(), щоб не зациклитись.
constexpr int kMaxDatagramsPerPoll = 64;

std::int32_t degTo1e7(double deg)
{
  return static_cast<std::int32_t>(std::llround(deg * 1e7));
}

std::int32_t metersToMm(double meters)
{
  return static_cast<std::int32_t>(std::llround(meters * 1000.0));
}

std::int16_t mpsToCms(double mps)
{
  const long long cms = std::llround(mps * 100.0);
  const long long clamped = std::clamp<long long>(cms, INT16_MIN, INT16_MAX);
  return static_cast<std::int16_t>(clamped);
}

std::uint16_t headingToCentiDeg(double heading_deg)
{
  const long long cdeg = std::llround(wrapDeg360(heading_deg) * 100.0) % 36000;
  return static_cast<std::uint16_t>(cdeg);
}

bool sendMessage(ITransport& transport, const mavlink_message_t& message)
{
  std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const std::uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
  return transport.send(buffer, len);
}

}  // namespace

MavlinkLink::MavlinkLink(ITransport& transport, std::uint8_t sysid, std::uint8_t compid)
  : transport_(transport)
  , sysid_(sysid)
  , compid_(compid)
{
  // Явно вимагаємо MAVLink 2 на вихід (кадри з магічним байтом 0xFD).
  mavlink_status_t* tx_status = mavlink_get_channel_status(kTxChannel);
  tx_status->flags = static_cast<std::uint8_t>(tx_status->flags & ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
}

void MavlinkLink::sendHeartbeat()
{
  mavlink_message_t message{};
  mavlink_msg_heartbeat_pack(
    sysid_, compid_, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
  sendMessage(transport_, message);
}

void MavlinkLink::sendTelemetry(const TelemetrySample& sample)
{
  const GeoPoint point = localToGeo(sample.x_east_m, sample.y_north_m);

  mavlink_message_t message{};

  // GLOBAL_POSITION_INT: vx = північ, vy = схід, vz = ВНИЗ (система NED).
  mavlink_msg_global_position_int_pack(sysid_,
                                       compid_,
                                       &message,
                                       sample.time_boot_ms,
                                       degTo1e7(point.lat_deg),
                                       degTo1e7(point.lon_deg),
                                       metersToMm(sample.alt_m),  // alt
                                       metersToMm(sample.alt_m),  // relative_alt
                                       mpsToCms(sample.vy_north_mps),
                                       mpsToCms(sample.vx_east_mps),
                                       mpsToCms(-sample.vz_up_mps),
                                       headingToCentiDeg(sample.heading_deg));
  sendMessage(transport_, message);

  // ATTITUDE: yaw у радіанах, той самий курс, що й hdg.
  mavlink_msg_attitude_pack(sysid_,
                            compid_,
                            &message,
                            sample.time_boot_ms,
                            static_cast<float>(sample.roll_rad),
                            static_cast<float>(sample.pitch_rad),
                            static_cast<float>(headingDegToYawRad(sample.heading_deg)),
                            0.0F,
                            0.0F,
                            0.0F);
  sendMessage(transport_, message);
}

void MavlinkLink::requestDrop(const DropRequest& request, std::uint32_t now_ms)
{
  if (drop_state_ != DropState::Idle) {
    return;  // скид ініціюється рівно один раз
  }
  pending_ = request;
  drop_attempts_ = 0;
  drop_state_ = DropState::WaitingAck;
  sendDropCommand(now_ms);
}

void MavlinkLink::sendDropCommand(std::uint32_t now_ms)
{
  mavlink_message_t message{};
  mavlink_msg_command_long_pack(sysid_,
                                compid_,
                                &message,
                                /*target_system=*/sysid_,
                                /*target_component=*/compid_,
                                MAV_CMD_USER_1,
                                /*confirmation=*/static_cast<std::uint8_t>(drop_attempts_),
                                0.0F,
                                0.0F,
                                0.0F,
                                0.0F,
                                static_cast<float>(pending_.lat_deg),
                                static_cast<float>(pending_.lon_deg),
                                static_cast<float>(pending_.alt_m));
  sendMessage(transport_, message);

  ++drop_attempts_;
  last_send_ms_ = now_ms;
}

void MavlinkLink::onCommandAck(std::uint16_t command, std::uint8_t result)
{
  if (drop_state_ != DropState::WaitingAck) {
    return;
  }
  if (command != MAV_CMD_USER_1) {
    return;  // ACK на чужу команду — ігноруємо
  }
  if (result == MAV_RESULT_ACCEPTED) {
    drop_state_ = DropState::Acked;
  }
}

void MavlinkLink::poll(std::uint32_t now_ms)
{
  std::uint8_t buffer[2048];
  mavlink_message_t message{};
  mavlink_status_t status{};

  for (int datagram = 0; datagram < kMaxDatagramsPerPoll; ++datagram) {
    const std::size_t received = transport_.receive(buffer, sizeof(buffer));
    if (received == 0) {
      break;
    }
    for (std::size_t i = 0; i < received; ++i) {
      if (mavlink_parse_char(kRxChannel, buffer[i], &message, &status) != 1) {
        continue;
      }
      if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&message, &ack);
        onCommandAck(ack.command, ack.result);
      }
    }
  }

  if (drop_state_ != DropState::WaitingAck) {
    return;
  }
  if (now_ms - last_send_ms_ < kAckTimeoutMs) {
    return;
  }
  if (drop_attempts_ < kMaxDropAttempts) {
    sendDropCommand(now_ms);
  }
  else {
    drop_state_ = DropState::Failed;
  }
}

}  // namespace mavtel
