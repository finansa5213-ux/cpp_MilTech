#pragma once

#include <cstdint>
#include <cstddef>
#include <optional>
#include <utility>

#include "antidrone_turret/target_sequence.hpp"

namespace antidrone_turret {

struct TargetTrackSimulationConfig {
  bool repeat_sequence{true};
  float hit_confidence{0.80F};
  float hit_distance_m{30.0F};
  float impact_distance_m{8.0F};
};

struct PublishedTarget {
  TargetSample sample;
  bool reached_protected_zone_while_reloading{false};
};

class TargetTrackSimulation {
public:
  TargetTrackSimulation() = default;

  TargetTrackSimulation(TargetSequence sequence, TargetTrackSimulationConfig config)
    : sequence_(std::move(sequence))
    , config_(config)
  {}

  [[nodiscard]] bool empty() const
  {
    return sequence_.empty();
  }

  [[nodiscard]] std::size_t size() const
  {
    return sequence_.size();
  }

  std::optional<PublishedTarget> next_target()
  {
    if (sequence_.empty() || sequence_.finished(config_.repeat_sequence)) {
      return std::nullopt;
    }

    const auto sample = sequence_.current();
    has_published_sample_ = true;

    if (!sample.visible) {
      hit_window_open_ = false;
    } else if (sample.confidence >= config_.hit_confidence &&
               sample.distance_m <= config_.hit_distance_m) {
      hit_window_open_ = true;
    }

    sequence_.advance();

    const auto reached_protected_zone =
      sample.visible &&
      sample.distance_m <= config_.impact_distance_m &&
      !actuator_ready_;

    if (reached_protected_zone) {
      sequence_.skip_to_next_track();
      has_published_sample_ = false;
      hit_window_open_ = false;
    }

    return PublishedTarget{sample, reached_protected_zone};
  }

  [[nodiscard]] bool apply_actuator_status(
    const bool actuator_ready,
    const std::uint32_t trigger_count)
  {
    actuator_ready_ = actuator_ready;

    if (trigger_count <= last_trigger_count_) {
      return false;
    }

    last_trigger_count_ = trigger_count;
    if (!has_published_sample_ || !hit_window_open_) {
      return false;
    }

    sequence_.skip_to_next_track();
    has_published_sample_ = false;
    hit_window_open_ = false;
    return true;
  }

private:
  TargetSequence sequence_;
  TargetTrackSimulationConfig config_;
  bool actuator_ready_{true};
  bool has_published_sample_{false};
  bool hit_window_open_{false};
  std::uint32_t last_trigger_count_{0};
};

}  // namespace antidrone_turret
