#include <gtest/gtest.h>

#include "antidrone_turret/turret_logic.hpp"

namespace {

constexpr antidrone_turret::TurretConfig kConfig{0.8F, 30.0F};

antidrone_turret::TargetObservation make_target(
  const bool visible,
  const float x,
  const float y,
  const float distance_m,
  const float confidence)
{
  return antidrone_turret::TargetObservation{visible, x, y, distance_m, confidence};
}

}  // namespace

// Оцінка цілі: confidence нижче порога -> ACTION_IDLE, TRIGGER_SKIP.
TEST(EvaluateTarget, LowConfidenceIsIdleAndSkip)
{
  const auto target = make_target(true, 320.0F, 240.0F, 10.0F, 0.55F);

  EXPECT_EQ(
    antidrone_turret::TargetState::kLowConfidence,
    antidrone_turret::evaluate_target(target, kConfig));

  const auto decision = antidrone_turret::decide(target, true, kConfig);
  EXPECT_EQ(antidrone_turret::TurretAction::kIdle, decision.action);
  EXPECT_EQ(antidrone_turret::TriggerDecision::kSkip, decision.trigger);
}

// Оцінка цілі: не видима -> TARGET_NONE.
TEST(EvaluateTarget, HiddenTargetIsNone)
{
  const auto target = make_target(false, 0.0F, 0.0F, 5.0F, 0.99F);

  EXPECT_EQ(
    antidrone_turret::TargetState::kNone,
    antidrone_turret::evaluate_target(target, kConfig));

  const auto decision = antidrone_turret::decide(target, true, kConfig);
  EXPECT_EQ(antidrone_turret::TurretAction::kIdle, decision.action);
  EXPECT_EQ(antidrone_turret::TriggerDecision::kSkip, decision.trigger);
}

// Команда yaw-серво: x > 320 -> RIGHT, error_x > 0.
TEST(ServoCommand, TargetRightOfCenterTurnsRight)
{
  const auto command = antidrone_turret::make_servo_command(420.0F);

  EXPECT_EQ(antidrone_turret::kDirectionRight, command.direction);
  EXPECT_FLOAT_EQ(420.0F, command.target_x);
  EXPECT_GT(command.error_x, 0.0F);
  EXPECT_FLOAT_EQ(100.0F, command.error_x);
}

// Команда yaw-серво: x < 320 -> LEFT; x == 320 -> CENTER.
TEST(ServoCommand, LeftAndCenterDirections)
{
  EXPECT_EQ(
    antidrone_turret::kDirectionLeft,
    antidrone_turret::make_servo_command(200.0F).direction);
  EXPECT_EQ(
    antidrone_turret::kDirectionCenter,
    antidrone_turret::make_servo_command(320.0F).direction);
}

// Команда гімбала: y < 240 -> UP, error_y > 0.
TEST(GimbalCommand, TargetAboveCenterMovesUp)
{
  const auto command = antidrone_turret::make_gimbal_command(180.0F);

  EXPECT_EQ(antidrone_turret::kDirectionUp, command.direction);
  EXPECT_FLOAT_EQ(180.0F, command.target_y);
  EXPECT_GT(command.error_y, 0.0F);
  EXPECT_FLOAT_EQ(60.0F, command.error_y);
}

// Команда гімбала: y > 240 -> DOWN; y == 240 -> CENTER.
TEST(GimbalCommand, DownAndCenterDirections)
{
  EXPECT_EQ(
    antidrone_turret::kDirectionDown,
    antidrone_turret::make_gimbal_command(300.0F).direction);
  EXPECT_EQ(
    antidrone_turret::kDirectionCenter,
    antidrone_turret::make_gimbal_command(240.0F).direction);
}

// Рішення щодо пострілу: близька ціль + актуатор READY -> TRIGGER_REQUESTED.
TEST(TriggerDecision, CloseTargetWithReadyActuatorRequestsTrigger)
{
  EXPECT_EQ(
    antidrone_turret::TriggerDecision::kRequested,
    antidrone_turret::decide_trigger(25.0F, true, kConfig));
}

// Рішення щодо пострілу: близька ціль + актуатор RELOADING -> TRIGGER_RELOADING.
TEST(TriggerDecision, CloseTargetWhileReloadingIsMarkedReloading)
{
  EXPECT_EQ(
    antidrone_turret::TriggerDecision::kReloading,
    antidrone_turret::decide_trigger(25.0F, false, kConfig));
}

// Рішення щодо пострілу: distance_m > max_distance_m -> TRIGGER_SKIP.
TEST(TriggerDecision, FarTargetIsSkipped)
{
  EXPECT_EQ(
    antidrone_turret::TriggerDecision::kSkip,
    antidrone_turret::decide_trigger(45.0F, true, kConfig));
}

// Статус контролера: далека коректна ціль ->
// TARGET_LOCKED, ACTION_TRACK, TRIGGER_SKIP.
TEST(TurretDecision, FarLockedTargetTracksWithoutTrigger)
{
  const auto target = make_target(true, 400.0F, 200.0F, 45.0F, 0.90F);
  const auto decision = antidrone_turret::decide(target, true, kConfig);

  EXPECT_EQ(antidrone_turret::TargetState::kLocked, decision.target_state);
  EXPECT_EQ(antidrone_turret::TurretAction::kTrack, decision.action);
  EXPECT_EQ(antidrone_turret::TriggerDecision::kSkip, decision.trigger);
}

// Статус контролера: близька коректна ціль з готовим актуатором ->
// TARGET_LOCKED, ACTION_TRACK, TRIGGER_REQUESTED.
TEST(TurretDecision, CloseLockedTargetRequestsTrigger)
{
  const auto target = make_target(true, 340.0F, 230.0F, 25.0F, 0.90F);
  const auto decision = antidrone_turret::decide(target, true, kConfig);

  EXPECT_EQ(antidrone_turret::TargetState::kLocked, decision.target_state);
  EXPECT_EQ(antidrone_turret::TurretAction::kTrack, decision.action);
  EXPECT_EQ(antidrone_turret::TriggerDecision::kRequested, decision.trigger);
}
