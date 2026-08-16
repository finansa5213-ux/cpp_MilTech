#pragma once

// Чиста C++ логіка контролера турелі.
// Без rclcpp, без publisher/subscriber/Node: тільки вхідні структури,
// правила ухвалення рішення і вихідні структури. ROS-нода перекладає
// ці типи у повідомлення та запит сервісу.

#include <cstdint>

namespace antidrone_turret {

// Спрощена модель кадру з умови ДЗ.
inline constexpr float kFrameCenterX = 320.0F;
inline constexpr float kFrameCenterY = 240.0F;

// Значення напрямків збігаються з константами у ServoCommand.msg
// і GimbalCommand.msg.
inline constexpr std::int8_t kDirectionLeft = -1;
inline constexpr std::int8_t kDirectionDown = -1;
inline constexpr std::int8_t kDirectionCenter = 0;
inline constexpr std::int8_t kDirectionRight = 1;
inline constexpr std::int8_t kDirectionUp = 1;

struct TargetObservation {
  bool visible{false};
  float x{0.0F};
  float y{0.0F};
  float distance_m{0.0F};
  float confidence{0.0F};
};

struct TurretConfig {
  float confidence_threshold{0.8F};
  float max_distance_m{30.0F};
};

enum class TargetState : std::uint8_t {
  kNone = 0,
  kLowConfidence = 1,
  kLocked = 2,
};

enum class TurretAction : std::uint8_t {
  kIdle = 0,
  kTrack = 1,
};

enum class TriggerDecision : std::uint8_t {
  kSkip = 0,
  kRequested = 1,
  kReloading = 2,
};

struct ServoCommandData {
  std::int8_t direction{kDirectionCenter};
  float target_x{0.0F};
  float error_x{0.0F};
};

struct GimbalCommandData {
  std::int8_t direction{kDirectionCenter};
  float target_y{0.0F};
  float error_y{0.0F};
};

struct TurretDecision {
  TargetState target_state{TargetState::kNone};
  TurretAction action{TurretAction::kIdle};
  TriggerDecision trigger{TriggerDecision::kSkip};
};

// Оцінка цілі: visible + confidence_threshold ->
// TARGET_NONE, TARGET_LOW_CONFIDENCE або TARGET_LOCKED.
inline TargetState evaluate_target(
  const TargetObservation& target,
  const TurretConfig& config)
{
  if (!target.visible) {
    return TargetState::kNone;
  }

  if (target.confidence < config.confidence_threshold) {
    return TargetState::kLowConfidence;
  }

  return TargetState::kLocked;
}

// Команда yaw-серво: Target.x -> direction, target_x, error_x.
// error_x = x - 320; x росте праворуч.
inline ServoCommandData make_servo_command(const float x)
{
  auto command = ServoCommandData{};
  command.target_x = x;
  command.error_x = x - kFrameCenterX;

  if (command.error_x > 0.0F) {
    command.direction = kDirectionRight;
  } else if (command.error_x < 0.0F) {
    command.direction = kDirectionLeft;
  } else {
    command.direction = kDirectionCenter;
  }

  return command;
}

// Команда гімбала: Target.y -> direction, target_y, error_y.
// error_y = 240 - y: у кадрі y росте вниз, тому ціль вище центру
// має менше y, а для гімбала це рух UP.
inline GimbalCommandData make_gimbal_command(const float y)
{
  auto command = GimbalCommandData{};
  command.target_y = y;
  command.error_y = kFrameCenterY - y;

  if (command.error_y > 0.0F) {
    command.direction = kDirectionUp;
  } else if (command.error_y < 0.0F) {
    command.direction = kDirectionDown;
  } else {
    command.direction = kDirectionCenter;
  }

  return command;
}

// Рішення щодо пострілу: distance_m, max_distance_m, останній стан
// актуатора -> TRIGGER_SKIP, TRIGGER_REQUESTED або TRIGGER_RELOADING.
inline TriggerDecision decide_trigger(
  const float distance_m,
  const bool actuator_ready,
  const TurretConfig& config)
{
  if (distance_m > config.max_distance_m) {
    return TriggerDecision::kSkip;
  }

  return actuator_ready ? TriggerDecision::kRequested
                        : TriggerDecision::kReloading;
}

// Повне рішення контролера для одного повідомлення /perception/target.
inline TurretDecision decide(
  const TargetObservation& target,
  const bool actuator_ready,
  const TurretConfig& config)
{
  auto decision = TurretDecision{};
  decision.target_state = evaluate_target(target, config);

  if (decision.target_state != TargetState::kLocked) {
    decision.action = TurretAction::kIdle;
    decision.trigger = TriggerDecision::kSkip;
    return decision;
  }

  decision.action = TurretAction::kTrack;
  decision.trigger = decide_trigger(target.distance_m, actuator_ready, config);
  return decision;
}

}  // namespace antidrone_turret
