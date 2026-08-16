#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/turret_logic.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

namespace {

constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kGimbalCommandTopic = "/gimbal/cmd";
constexpr auto kServoCommandTopic = "/servo/cmd";
constexpr auto kTurretStatusTopic = "/turret/status";
constexpr auto kTriggerService = "/actuator/trigger";

antidrone_turret::TargetObservation to_observation(
  const antidrone_turret::msg::Target& message)
{
  auto observation = antidrone_turret::TargetObservation{};
  observation.visible = message.visible;
  observation.x = message.x;
  observation.y = message.y;
  observation.distance_m = message.distance_m;
  observation.confidence = message.confidence;
  return observation;
}

}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  using Target = antidrone_turret::msg::Target;
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    config_.confidence_threshold =
      static_cast<float>(declare_parameter<double>("confidence_threshold", 0.8));
    config_.max_distance_m =
      static_cast<float>(declare_parameter<double>("max_distance_m", 30.0));

    gimbal_publisher_ = create_publisher<GimbalCommand>(kGimbalCommandTopic, 10);
    servo_publisher_ = create_publisher<ServoCommand>(kServoCommandTopic, 10);
    status_publisher_ = create_publisher<TurretStatus>(kTurretStatusTopic, 10);

    trigger_client_ = create_client<TriggerActuator>(kTriggerService);

    actuator_status_subscription_ = create_subscription<ActuatorStatus>(
      kActuatorStatusTopic,
      10,
      [this](const ActuatorStatus& status) { on_actuator_status(status); });

    target_subscription_ = create_subscription<Target>(
      kTargetTopic,
      10,
      [this](const Target& target) { on_target(target); });

    RCLCPP_INFO(
      get_logger(),
      "confidence_threshold=%.2f max_distance_m=%.1f",
      config_.confidence_threshold,
      config_.max_distance_m);
  }

private:
  void on_actuator_status(const ActuatorStatus& status)
  {
    actuator_ready_ = status.state == ActuatorStatus::READY;
  }

  void on_target(const Target& target)
  {
    const auto observation = to_observation(target);
    const auto decision =
      antidrone_turret::decide(observation, actuator_ready_, config_);

    if (decision.action == antidrone_turret::TurretAction::kTrack) {
      publish_aim_commands(observation);
    }

    if (decision.trigger == antidrone_turret::TriggerDecision::kRequested) {
      request_trigger(observation);
    }

    publish_status(observation, decision);
  }

  void publish_aim_commands(const antidrone_turret::TargetObservation& observation)
  {
    const auto servo = antidrone_turret::make_servo_command(observation.x);
    auto servo_message = ServoCommand{};
    servo_message.direction = servo.direction;
    servo_message.target_x = servo.target_x;
    servo_message.error_x = servo.error_x;
    servo_publisher_->publish(servo_message);

    const auto gimbal = antidrone_turret::make_gimbal_command(observation.y);
    auto gimbal_message = GimbalCommand{};
    gimbal_message.direction = gimbal.direction;
    gimbal_message.target_y = gimbal.target_y;
    gimbal_message.error_y = gimbal.error_y;
    gimbal_publisher_->publish(gimbal_message);
  }

  void request_trigger(const antidrone_turret::TargetObservation& observation)
  {
    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "trigger service %s is not ready", kTriggerService);
      return;
    }

    auto request = std::make_shared<TriggerActuator::Request>();
    request->confidence = observation.confidence;
    request->distance_m = observation.distance_m;

    // Локально вважаємо актуатор зайнятим одразу після запиту, щоб не
    // надіслати повторний постріл, поки /actuator/status ще не оновився.
    actuator_ready_ = false;

    trigger_client_->async_send_request(
      request,
      [this](const rclcpp::Client<TriggerActuator>::SharedFuture future) {
        const auto response = future.get();
        if (response->accepted) {
          RCLCPP_INFO(
            get_logger(),
            "trigger accepted trigger_count=%u",
            response->trigger_count);
        } else {
          RCLCPP_WARN(get_logger(), "trigger rejected while actuator reloading");
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "trigger requested confidence=%.2f distance_m=%.1f",
      observation.confidence,
      observation.distance_m);
  }

  void publish_status(
    const antidrone_turret::TargetObservation& observation,
    const antidrone_turret::TurretDecision& decision)
  {
    auto status = TurretStatus{};
    status.target_state = static_cast<std::uint8_t>(decision.target_state);
    status.action = static_cast<std::uint8_t>(decision.action);
    status.trigger_state = static_cast<std::uint8_t>(decision.trigger);
    status.confidence = observation.confidence;
    status.distance_m = observation.distance_m;
    status_publisher_->publish(status);
  }

  antidrone_turret::TurretConfig config_;
  bool actuator_ready_{false};

  rclcpp::Publisher<GimbalCommand>::SharedPtr gimbal_publisher_;
  rclcpp::Publisher<ServoCommand>::SharedPtr servo_publisher_;
  rclcpp::Publisher<TurretStatus>::SharedPtr status_publisher_;
  rclcpp::Client<TriggerActuator>::SharedPtr trigger_client_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_status_subscription_;
  rclcpp::Subscription<Target>::SharedPtr target_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}
