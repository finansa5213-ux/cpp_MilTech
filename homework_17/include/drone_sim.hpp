#pragma once

#include <cstdint>

#include "mavlink_link.hpp"

namespace mavtel {

/// УВАГА: це мінімальна довідкова модель польоту — «заглушка» замість вашого ДЗ11.
/// Якщо у вас уже є DroneStateMachine з ДЗ08/ДЗ11 — видаліть цей файл і віддавайте
/// у MavlinkLink::sendTelemetry() TelemetrySample, зібраний з вашого стану.
/// MavlinkLink навмисно нічого не знає про фізику, тому заміна безболісна.

struct DroneConfig {
  double target_x_m{800.0};  ///< ціль: схід, м
  double target_y_m{600.0};  ///< ціль: північ, м
  double cruise_speed_mps{25.0};
  double accel_mps2{5.0};
  double cruise_alt_m{120.0};
  double yaw_rate_dps{60.0};
  double gravity_mps2{9.81};
};

class DroneSim {
public:
  explicit DroneSim(DroneConfig config);

  /// Один крок фізики. Повертає true рівно на тому кроці, де балістика дає скид.
  bool step(double dt_s);

  TelemetrySample sample(std::uint32_t time_boot_ms) const;

  bool dropped() const noexcept { return dropped_; }
  double x() const noexcept { return x_; }
  double y() const noexcept { return y_; }
  double altitude() const noexcept { return alt_m_; }
  double distanceToTarget() const;

private:
  DroneConfig config_;
  double x_{0.0};
  double y_{0.0};
  double alt_m_{0.0};
  double speed_mps_{0.0};
  double heading_deg_{0.0};
  double vx_east_mps_{0.0};
  double vy_north_mps_{0.0};
  bool dropped_{false};
};

}  // namespace mavtel
