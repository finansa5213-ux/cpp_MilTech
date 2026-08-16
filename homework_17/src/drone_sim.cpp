#include "drone_sim.hpp"

#include <algorithm>
#include <cmath>

#include "geo.hpp"

namespace mavtel {

DroneSim::DroneSim(DroneConfig config)
  : config_(config)
{
  alt_m_ = config_.cruise_alt_m;
  heading_deg_ = headingFromVelocityDeg(config_.target_x_m, config_.target_y_m);
}

double DroneSim::distanceToTarget() const
{
  return std::hypot(config_.target_x_m - x_, config_.target_y_m - y_);
}

bool DroneSim::step(double dt_s)
{
  // 1. Довертаємось на ціль з обмеженою кутовою швидкістю.
  const double desired_heading = headingFromVelocityDeg(config_.target_x_m - x_, config_.target_y_m - y_, heading_deg_);
  const double max_turn = config_.yaw_rate_dps * dt_s;
  const double turn = std::clamp(wrapDeg180(desired_heading - heading_deg_), -max_turn, max_turn);
  heading_deg_ = wrapDeg360(heading_deg_ + turn);

  // 2. Розганяємось до крейсерської швидкості.
  speed_mps_ = std::min(config_.cruise_speed_mps, speed_mps_ + config_.accel_mps2 * dt_s);

  // 3. Інтегруємо позицію.
  const double heading_rad = heading_deg_ * kPi / 180.0;
  vx_east_mps_ = speed_mps_ * std::sin(heading_rad);
  vy_north_mps_ = speed_mps_ * std::cos(heading_rad);
  x_ += vx_east_mps_ * dt_s;
  y_ += vy_north_mps_ * dt_s;

  // 4. Балістика: вантаж летить t = sqrt(2h/g) і за цей час пролітає v*t уперед.
  if (!dropped_) {
    const double fall_time_s = std::sqrt(2.0 * alt_m_ / config_.gravity_mps2);
    const double release_distance_m = speed_mps_ * fall_time_s;
    if (distanceToTarget() <= release_distance_m) {
      dropped_ = true;
      return true;
    }
  }
  return false;
}

TelemetrySample DroneSim::sample(std::uint32_t time_boot_ms) const
{
  TelemetrySample s{};
  s.time_boot_ms = time_boot_ms;
  s.x_east_m = x_;
  s.y_north_m = y_;
  s.alt_m = alt_m_;
  s.vx_east_mps = vx_east_mps_;
  s.vy_north_mps = vy_north_mps_;
  s.vz_up_mps = 0.0;
  s.heading_deg = heading_deg_;
  s.roll_rad = 0.0;
  s.pitch_rad = 0.0;
  return s;
}

}  // namespace mavtel
