#pragma once

#include <stdexcept>
#include <string>

struct BallisticsInput {
  double drone_x_{};
  double drone_y_{};
  double drone_z_{};
  double target_x_{};
  double target_y_{};
  double attack_speed_{};
  double acceleration_path_{};
  std::string ammo_name_;
};

struct AmmoSpec {
  double weight_{};
  double drag_coef_{};
  double lift_coef_{};
};

struct DropSolution {
  double fire_x_{};
  double fire_y_{};
};

class UnknownAmmoError : public std::invalid_argument {
public:
  using std::invalid_argument::invalid_argument;
};

auto get_ammo_spec(const std::string& name) -> AmmoSpec;

// NOLINTNEXTLINE(readability-identifier-length, bugprone-easily-swappable-parameters)
auto compute_fall_time(double height, double speed, const AmmoSpec& ammo) -> double;

// 't' — час падіння у фізичній моделі (стандартне математичне позначення).
// NOLINTNEXTLINE(readability-identifier-length, bugprone-easily-swappable-parameters)
auto compute_horizontal_range(double t, double speed, const AmmoSpec& ammo) -> double;

auto compute_drop_solution(const BallisticsInput& input) -> DropSolution;