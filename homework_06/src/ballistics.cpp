#include "ballistics.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {
constexpr double kGravity = 9.81;
constexpr double kEps = 1e-10;
constexpr double kEpsDistance = 1e-6;

// Параметри боєприпасів: маса (кг), коефіцієнт опору, коефіцієнт підйому.
struct AmmoEntry {
  const char* name;
  double weight;
  double drag_coef;
  double lift_coef;
};

constexpr std::array<AmmoEntry, 5> kAmmoTable = {{
  {"VOG-17", 0.35, 0.07, 0.0},
  {"M67", 0.60, 0.10, 0.0},
  {"RKG-3", 1.20, 0.10, 0.0},
  {"GLIDING-VOG", 0.45, 0.10, 1.0},
  {"GLIDING-RKG", 1.40, 0.10, 1.0},
}};
}  // namespace

auto get_ammo_spec(const std::string& name) -> AmmoSpec
{
  for (const auto& entry : kAmmoTable) {
    if (name == entry.name) {
      return {entry.weight, entry.drag_coef, entry.lift_coef};
    }
  }
  throw UnknownAmmoError("Unknown ammunition type: " + name);
}

// NOLINTBEGIN(readability-identifier-length, readability-identifier-naming, readability-magic-numbers,
// cppcoreguidelines-avoid-magic-numbers, bugprone-easily-swappable-parameters) Короткі імена і числові коефіцієнти відповідають
// математичним позначенням у формулі Кардано: m = маса, d = drag, l = lift, g = прискорення, v0 = швидкість, z0 = висота; a, b, c =
// коефіцієнти кубічного рівняння a·t³ + b·t² + c = 0; коефіцієнти 2, 3, 6, 27, 36 — частина формули.
auto compute_fall_time(double height, double speed, const AmmoSpec& ammo) -> double
{
  const double m = ammo.weight_;
  const double d = ammo.drag_coef_;
  const double l = ammo.lift_coef_;
  const double g = kGravity;
  const double v0 = speed;
  const double z0 = height;

  const double a = d * g * m - 2.0 * d * d * l * v0;
  const double b = -3.0 * g * m * m + 3.0 * d * l * m * v0;
  const double c = 6.0 * m * m * z0;

  if (std::fabs(a) < kEps) {
    if (std::fabs(b) > kEps && -c / b > 0.0) {
      return std::sqrt(-c / b);
    }
    return std::sqrt(2.0 * z0 / g);
  }

  const double cardano_p = -b * b / (3.0 * a * a);
  const double cardano_q = 2.0 * b * b * b / (27.0 * a * a * a) + c / a;

  if (cardano_p >= 0.0) {
    throw std::runtime_error("Cardano: P >= 0, model does not apply");
  }

  const double sqrt_term = std::sqrt(-3.0 / cardano_p);
  double arg = (3.0 * cardano_q / (2.0 * cardano_p)) * sqrt_term;

  if (arg > 1.0) {
    arg = 1.0;
  }
  if (arg < -1.0) {
    arg = -1.0;
  }

  const double phi = std::acos(arg);
  const double r = std::sqrt(-cardano_p / 3.0);

  const double t = 2.0 * r * std::cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

  if (t <= 0.0) {
    throw std::runtime_error("Cardano: t <= 0, invalid solution");
  }
  return t;
}

// Короткі імена і числові коефіцієнти для математичної формули степеневого ряду.
auto compute_horizontal_range(double t, double speed, const AmmoSpec& ammo) -> double
{
  const double m = ammo.weight_;
  const double d = ammo.drag_coef_;
  const double l = ammo.lift_coef_;
  const double g = kGravity;
  const double v0 = speed;

  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  const double l2 = l * l;
  const double l3 = l2 * l;
  const double l4 = l2 * l2;
  const double one_plus_l2 = 1.0 + l2;
  const double m2 = m * m;
  const double m3 = m2 * m;
  const double m4 = m2 * m2;
  const double d2 = d * d;
  const double d3 = d2 * d;
  const double d4 = d2 * d2;

  const double term1 = v0 * t;
  const double term2 = -(d * v0 / (2.0 * m)) * t2;
  const double term3 = ((6.0 * d * g * l * m - 6.0 * d2 * (l2 - 1.0) * v0) / (36.0 * m2)) * t3;
  const double term4 = ((-6.0 * d2 * g * l * (1.0 + l2) * m + 3.0 * d3 * l2 * (1.0 + l2) * v0 + 6.0 * d3 * l4 * (1.0 + l2) * v0) /
                        (36.0 * one_plus_l2 * one_plus_l2 * m3)) *
                       t4;
  const double term5 = ((3.0 * d3 * g * l3 * m - 3.0 * d4 * l2 * (1.0 + l2) * v0) / (36.0 * one_plus_l2 * m4)) * t5;

  return term1 + term2 + term3 + term4 + term5;
}
// NOLINTEND(readability-identifier-length, readability-identifier-naming, readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers,
// bugprone-easily-swappable-parameters)

auto compute_drop_solution(const BallisticsInput& input) -> DropSolution
{
  const AmmoSpec ammo = get_ammo_spec(input.ammo_name_);

  double h = 0.0;
  // NOLINTBEGIN(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  // Коефіцієнт 2.0 — частина математичної моделі для перевірки умови виродження.
  if (std::fabs(ammo.drag_coef_ * kGravity * ammo.weight_ -
                2.0 * ammo.drag_coef_ * ammo.drag_coef_ * ammo.lift_coef_ * input.attack_speed_) < kEps) {
    h = 0.0;
  }
  else {
    const double t = compute_fall_time(input.drone_z_, input.attack_speed_, ammo);
    h = compute_horizontal_range(t, input.attack_speed_, ammo);
    if (h <= 0.0) {
      throw std::runtime_error("Horizontal range h <= 0");
    }
  }

  const double dx = input.target_x_ - input.drone_x_;
  const double dy = input.target_y_ - input.drone_y_;
  const double distance = std::sqrt(dx * dx + dy * dy);
  if (distance <= 0.0) {
    throw std::runtime_error("Distance to target D <= 0");
  }

  double approach_x = input.drone_x_;
  double approach_y = input.drone_y_;
  double approach_dist = distance;
  const double min_dist = h + input.acceleration_path_;
  if (min_dist > distance && distance > kEpsDistance) {
    approach_x = input.target_x_ - dx * (min_dist / distance);
    approach_y = input.target_y_ - dy * (min_dist / distance);
    approach_dist = min_dist;
  }

  const double ratio = (approach_dist > h + kEpsDistance) ? (approach_dist - h) / approach_dist : 0.0;
  return {
    approach_x + (input.target_x_ - approach_x) * ratio,
    approach_y + (input.target_y_ - approach_y) * ratio,
  };
}
// NOLINTEND(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
