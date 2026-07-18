#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace antidrone_turret {

struct TargetSample {
  bool visible{false};
  float x{0.0F};
  float y{0.0F};
  float distance_m{0.0F};
  float confidence{0.0F};
};

class TargetSequence {
public:
  TargetSequence() = default;

  explicit TargetSequence(std::vector<TargetSample> samples)
    : samples_(std::move(samples))
  {}

  [[nodiscard]] bool empty() const
  {
    return samples_.empty();
  }

  [[nodiscard]] std::size_t size() const
  {
    return samples_.size();
  }

  [[nodiscard]] bool finished(bool repeat) const
  {
    return !repeat && next_index_ >= samples_.size();
  }

  [[nodiscard]] const TargetSample& current() const
  {
    return samples_.at(next_index_ % samples_.size());
  }

  void advance()
  {
    ++next_index_;
  }

  void skip_to_next_track()
  {
    if (samples_.empty()) {
      return;
    }

    auto candidate_index = next_index_;
    const auto stop_index = next_index_ + samples_.size();
    bool passed_gap = false;

    while (candidate_index < stop_index) {
      const auto& sample = samples_.at(candidate_index % samples_.size());

      if (!sample.visible) {
        passed_gap = true;
      } else if (passed_gap) {
        next_index_ = candidate_index;
        return;
      }

      ++candidate_index;
    }

    next_index_ = stop_index;
  }

private:
  std::vector<TargetSample> samples_;
  std::size_t next_index_{0};
};

inline std::vector<TargetSample> default_target_samples()
{
  return {
    TargetSample{true, 320.0F, 240.0F, 70.0F, 0.70F},
    TargetSample{true, 340.0F, 230.0F, 55.0F, 0.82F},
    TargetSample{true, 365.0F, 215.0F, 42.0F, 0.87F},
    TargetSample{true, 390.0F, 200.0F, 31.0F, 0.91F},
    TargetSample{true, 420.0F, 180.0F, 25.0F, 0.93F},
    TargetSample{true, 445.0F, 170.0F, 19.0F, 0.95F},
  };
}

}  // namespace antidrone_turret
