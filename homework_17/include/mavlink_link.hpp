#pragma once

#include <cstdint>

#include "transport.hpp"

namespace mavtel {

/// Один «зріз» стану дрона з кроку фізики.
/// Осі: x — на схід, y — на північ, z (alt) — вгору. Курс — компасний (0 = північ).
struct TelemetrySample {
  std::uint32_t time_boot_ms{};  ///< час симуляції від старту, мс (монотонний)
  double x_east_m{};
  double y_north_m{};
  double alt_m{};  ///< висота над точкою старту, м
  double vx_east_mps{};
  double vy_north_mps{};
  double vz_up_mps{};
  double heading_deg{};  ///< курс 0..360, 0 = північ, за годинниковою
  double roll_rad{};
  double pitch_rad{};
};

/// Точка скиду для COMMAND_LONG / MAV_CMD_USER_1.
struct DropRequest {
  double lat_deg{};
  double lon_deg{};
  double alt_m{};
};

enum class DropState {
  Idle,        ///< скиду ще не було
  WaitingAck,  ///< команда надіслана, чекаємо COMMAND_ACK
  Acked,       ///< ACK отримано — більше не шлемо
  Failed       ///< вичерпано всі спроби, ACK не отримано
};

/// Формує та розбирає кадри MAVLink 2 поверх довільного транспорту.
/// Клас не знає нічого про фізику дрона — тільки про протокол.
class MavlinkLink {
public:
  static constexpr std::uint8_t kDefaultSysId = 1;
  static constexpr std::uint8_t kDefaultCompId = 1;  ///< MAV_COMP_ID_AUTOPILOT1
  static constexpr int kMaxDropAttempts = 5;
  static constexpr std::uint32_t kAckTimeoutMs = 500;

  explicit MavlinkLink(ITransport& transport, std::uint8_t sysid = kDefaultSysId, std::uint8_t compid = kDefaultCompId);

  /// HEARTBEAT: MAV_TYPE_QUADROTOR + MAV_STATE_ACTIVE. Викликати 1 раз/с.
  void sendHeartbeat();

  /// GLOBAL_POSITION_INT + ATTITUDE. Викликати не рідше 2 разів/с.
  void sendTelemetry(const TelemetrySample& sample);

  /// Почати процедуру скиду. Викликається один раз, у момент спрацювання балістики.
  void requestDrop(const DropRequest& request, std::uint32_t now_ms);

  /// Прокачати вхідні кадри та за потреби повторити COMMAND_LONG.
  /// Викликати щокроку головного циклу — саме тут живе логіка таймаутів.
  void poll(std::uint32_t now_ms);

  DropState dropState() const noexcept { return drop_state_; }
  int dropAttempts() const noexcept { return drop_attempts_; }
  bool dropFinished() const noexcept { return drop_state_ == DropState::Acked || drop_state_ == DropState::Failed; }

private:
  void sendDropCommand(std::uint32_t now_ms);
  void onCommandAck(std::uint16_t command, std::uint8_t result);

  ITransport& transport_;
  std::uint8_t sysid_;
  std::uint8_t compid_;

  DropRequest pending_{};
  DropState drop_state_{DropState::Idle};
  int drop_attempts_{0};
  std::uint32_t last_send_ms_{0};
};

}  // namespace mavtel
