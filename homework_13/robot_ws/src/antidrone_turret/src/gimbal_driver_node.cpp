#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/gimbal_command.hpp"

namespace {

constexpr auto kGimbalCommandTopic = "/gimbal/cmd";

const char* direction_name(const std::int8_t direction)
{
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  switch (direction) {
    case GimbalCommand::UP:
      return "UP";
    case GimbalCommand::DOWN:
      return "DOWN";
    case GimbalCommand::CENTER:
      return "CENTER";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class GimbalDriverNode final : public rclcpp::Node {
public:
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;

  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    subscription_ = create_subscription<GimbalCommand>(
      kGimbalCommandTopic,
      10,
      [this](const GimbalCommand& command) { on_command(command); });

    RCLCPP_INFO(get_logger(), "listening on %s", kGimbalCommandTopic);
  }

private:
  void on_command(const GimbalCommand& command)
  {
    RCLCPP_INFO(
      get_logger(),
      "отримав: direction=%s target_y=%.1f error_y=%.1f",
      direction_name(command.direction),
      command.target_y,
      command.error_y);
  }

  rclcpp::Subscription<GimbalCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}
