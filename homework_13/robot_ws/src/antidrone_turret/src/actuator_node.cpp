#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

namespace {

constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kTriggerService = "/actuator/trigger";

std::uint8_t to_message_state(const antidrone_turret::ActuatorState state)
{
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  return state == antidrone_turret::ActuatorState::kReady
           ? ActuatorStatus::READY
           : ActuatorStatus::RELOADING;
}

}  // namespace

class ActuatorNode final : public rclcpp::Node {
public:
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  ActuatorNode()
    : Node("actuator_node")
  {
    reload_ms_ = declare_parameter<int>("reload_ms", 1000);
    const auto status_publish_hz = declare_parameter<double>("status_publish_hz", 2.0);

    status_publisher_ = create_publisher<ActuatorStatus>(kActuatorStatusTopic, 10);
    trigger_service_ = create_service<TriggerActuator>(
      kTriggerService,
      [this](
        const std::shared_ptr<TriggerActuator::Request> request,
        std::shared_ptr<TriggerActuator::Response> response) {
        on_trigger(request, response);
      });

    const auto safe_hz = std::max(0.1, status_publish_hz);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / safe_hz));
    status_timer_ = create_wall_timer(period, [this]() { publish_status(); });

    publish_status();

    RCLCPP_INFO(
      get_logger(),
      "serving %s and publishing %s",
      kTriggerService,
      kActuatorStatusTopic);
  }

private:
  void on_trigger(
    const std::shared_ptr<TriggerActuator::Request>& request,
    const std::shared_ptr<TriggerActuator::Response>& response)
  {
    const auto result = actuator_.trigger();
    response->accepted = result.accepted;
    response->trigger_count = result.trigger_count;

    if (!result.accepted) {
      RCLCPP_WARN(get_logger(), "trigger rejected while actuator is reloading");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "trigger accepted confidence=%.2f distance_m=%.1f trigger_count=%u",
      request->confidence,
      request->distance_m,
      result.trigger_count);

    publish_status();
    schedule_reload();
  }

  void schedule_reload()
  {
    if (reload_timer_) {
      reload_timer_->cancel();
    }

    const auto delay = std::chrono::milliseconds{std::max(0, reload_ms_)};
    reload_timer_ = create_wall_timer(delay, [this]() {
      actuator_.mark_ready();
      publish_status();
      RCLCPP_INFO(get_logger(), "actuator ready after reload");
      reload_timer_->cancel();
    });
  }

  void publish_status()
  {
    auto status = ActuatorStatus{};
    status.state = to_message_state(actuator_.state());
    status.trigger_count = actuator_.trigger_count();
    status_publisher_->publish(status);
  }

  int reload_ms_{1000};
  antidrone_turret::ActuatorModel actuator_;
  rclcpp::Publisher<ActuatorStatus>::SharedPtr status_publisher_;
  rclcpp::Service<TriggerActuator>::SharedPtr trigger_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr reload_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorNode>());
  rclcpp::shutdown();
  return 0;
}
