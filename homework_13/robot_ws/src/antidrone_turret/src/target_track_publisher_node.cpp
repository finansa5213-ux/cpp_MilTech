#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/target_sequence.hpp"
#include "antidrone_turret/target_track_loader.hpp"
#include "antidrone_turret/target_track_simulation.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/target.hpp"

namespace {

constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kAllTracksSelection = "all";

antidrone_turret::msg::Target to_message(const antidrone_turret::TargetSample& sample)
{
  auto message = antidrone_turret::msg::Target{};
  message.visible = sample.visible;
  message.x = sample.x;
  message.y = sample.y;
  message.distance_m = sample.distance_m;
  message.confidence = sample.confidence;
  return message;
}

std::vector<antidrone_turret::TargetSample> load_samples_from_parameter(
  const std::string& track_file,
  const rclcpp::Logger& logger)
{
  if (track_file.empty()) {
    RCLCPP_WARN(logger, "track_file is empty, using built-in fallback target track");
    return antidrone_turret::default_target_samples();
  }

  const auto track_path = std::filesystem::path{track_file};
  auto load_result =
    track_path.filename().string() == kAllTracksSelection
      ? antidrone_turret::load_target_track_csv_files(
          track_path.parent_path(),
          antidrone_turret::default_target_track_files())
      : antidrone_turret::load_target_track_csv(track_path);

  if (!load_result.error.empty()) {
    throw std::runtime_error{load_result.error};
  }

  RCLCPP_INFO(
    logger,
    "loaded %zu target samples from %s",
    load_result.samples.size(),
    track_file.c_str());
  return load_result.samples;
}

}  // namespace

class TargetTrackPublisherNode final : public rclcpp::Node {
public:
  TargetTrackPublisherNode()
    : Node("target_track_publisher_node")
  {
    const auto publish_hz = declare_parameter<double>("publish_hz", 1.0);
    const auto repeat_sequence = declare_parameter<bool>("repeat_sequence", true);
    const auto track_file = declare_parameter<std::string>("track_file", "");

    auto config = antidrone_turret::TargetTrackSimulationConfig{};
    config.repeat_sequence = repeat_sequence;

    simulation_ = antidrone_turret::TargetTrackSimulation{
      antidrone_turret::TargetSequence{load_samples_from_parameter(track_file, get_logger())},
      config,
    };

    publisher_ = create_publisher<antidrone_turret::msg::Target>(kTargetTopic, 10);
    actuator_status_subscription_ =
      create_subscription<antidrone_turret::msg::ActuatorStatus>(
        kActuatorStatusTopic,
        10,
        [this](const antidrone_turret::msg::ActuatorStatus& status) {
          on_actuator_status(status);
        });

    const auto safe_hz = std::max(0.1, publish_hz);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / safe_hz));

    timer_ = create_wall_timer(period, [this]() { publish_next_target(); });

    RCLCPP_INFO(
      get_logger(),
      "publishing %zu target samples on %s at %.2f Hz",
      simulation_.size(),
      kTargetTopic,
      safe_hz);
  }

private:
  void publish_next_target()
  {
    const auto target = simulation_.next_target();
    if (!target.has_value()) {
      return;
    }

    publisher_->publish(to_message(target->sample));

    RCLCPP_INFO(
      get_logger(),
      "target visible=%s x=%.1f y=%.1f distance_m=%.1f confidence=%.2f",
      target->sample.visible ? "true" : "false",
      target->sample.x,
      target->sample.y,
      target->sample.distance_m,
      target->sample.confidence);

    if (target->reached_protected_zone_while_reloading) {
      RCLCPP_DEBUG(
        get_logger(),
        "target reached protected zone while actuator is reloading");
    }
  }

  void on_actuator_status(const antidrone_turret::msg::ActuatorStatus& status)
  {
    const auto actuator_ready =
      status.state == antidrone_turret::msg::ActuatorStatus::READY;
    const auto segment_ended =
      simulation_.apply_actuator_status(actuator_ready, status.trigger_count);

    if (segment_ended) {
      RCLCPP_DEBUG(
        get_logger(),
        "target track segment ended after accepted trigger");
    }
  }

  antidrone_turret::TargetTrackSimulation simulation_;
  rclcpp::Publisher<antidrone_turret::msg::Target>::SharedPtr publisher_;
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr
    actuator_status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTrackPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
