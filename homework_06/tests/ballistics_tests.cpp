#include "ballistics.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <stdexcept>

namespace {

// Базові тестові значення (відповідають прикладу з PDF, стор. 5).
constexpr double kDefaultDroneCoord = 100.0;
constexpr double kDefaultTargetCoord = 200.0;
constexpr double kDefaultSpeed = 10.0;
constexpr double kDefaultAccelerationPath = 10.0;

// Очікувані координати точки скиду для еталонного тесту.
constexpr double kExpectedFireCoord = 173.759;
constexpr double kFireCoordTolerance = 0.01;

// Значення для тесту маневру (дрон занадто близько до цілі).
constexpr double kCloseDroneCoord = 150.0;
constexpr double kCloseTargetCoord = 155.0;
constexpr double kManeuverAccelerationPath = 50.0;

// Очікувані параметри боєприпасів.
constexpr double kVOG17Weight = 0.35;
constexpr double kVOG17DragCoef = 0.07;
constexpr double kVOG17LiftCoef = 0.0;
constexpr double kGlidingRKGWeight = 1.4;

auto make_default_input() -> BallisticsInput
{
  return BallisticsInput{
    .drone_x_ = kDefaultDroneCoord,
    .drone_y_ = kDefaultDroneCoord,
    .drone_z_ = kDefaultDroneCoord,
    .target_x_ = kDefaultTargetCoord,
    .target_y_ = kDefaultTargetCoord,
    .attack_speed_ = kDefaultSpeed,
    .acceleration_path_ = kDefaultAccelerationPath,
    .ammo_name_ = "VOG-17",
  };
}

}  // namespace

TEST(Ballistics, ComputesKnownDropPointVOG17)
{
  const BallisticsInput input = make_default_input();
  const DropSolution s = compute_drop_solution(input);

  EXPECT_NEAR(s.fire_x_, kExpectedFireCoord, kFireCoordTolerance);
  EXPECT_NEAR(s.fire_y_, kExpectedFireCoord, kFireCoordTolerance);
}

TEST(Ballistics, ThrowsOnUnknownAmmo)
{
  BallisticsInput input = make_default_input();
  input.ammo_name_ = "BANANA";

  EXPECT_THROW(compute_drop_solution(input), UnknownAmmoError);
}

TEST(Ballistics, GlidingAmmoProducesValidSolution)
{
  BallisticsInput input = make_default_input();
  input.ammo_name_ = "GLIDING-VOG";

  DropSolution s{};
  ASSERT_NO_THROW({ s = compute_drop_solution(input); });

  EXPECT_TRUE(std::isfinite(s.fire_x_));
  EXPECT_TRUE(std::isfinite(s.fire_y_));
}

TEST(Ballistics, M67ProducesValidSolution)
{
  BallisticsInput input = make_default_input();
  input.ammo_name_ = "M67";

  DropSolution s{};
  ASSERT_NO_THROW({ s = compute_drop_solution(input); });

  EXPECT_TRUE(std::isfinite(s.fire_x_));
  EXPECT_TRUE(std::isfinite(s.fire_y_));
}

TEST(Ballistics, ThrowsWhenDistanceIsZero)
{
  BallisticsInput input = make_default_input();
  input.target_x_ = input.drone_x_;
  input.target_y_ = input.drone_y_;

  EXPECT_THROW(compute_drop_solution(input), std::runtime_error);
}

TEST(Ballistics, RequiresApproachManeuverWhenTooClose)
{
  BallisticsInput input = make_default_input();
  input.drone_x_ = kCloseDroneCoord;
  input.drone_y_ = kCloseDroneCoord;
  input.target_x_ = kCloseTargetCoord;
  input.target_y_ = kCloseTargetCoord;
  input.acceleration_path_ = kManeuverAccelerationPath;

  DropSolution s{};
  ASSERT_NO_THROW({ s = compute_drop_solution(input); });

  EXPECT_TRUE(std::isfinite(s.fire_x_));
  EXPECT_TRUE(std::isfinite(s.fire_y_));
}

TEST(AmmoSpec, ReturnsCorrectVOG17)
{
  const AmmoSpec spec = get_ammo_spec("VOG-17");
  EXPECT_DOUBLE_EQ(spec.weight_, kVOG17Weight);
  EXPECT_DOUBLE_EQ(spec.drag_coef_, kVOG17DragCoef);
  EXPECT_DOUBLE_EQ(spec.lift_coef_, kVOG17LiftCoef);
}

TEST(AmmoSpec, GlidingHasLiftCoefficient)
{
  const AmmoSpec spec = get_ammo_spec("GLIDING-RKG");
  EXPECT_DOUBLE_EQ(spec.weight_, kGlidingRKGWeight);
  EXPECT_GT(spec.lift_coef_, 0.0);
}