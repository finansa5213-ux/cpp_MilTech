#pragma once

#include <cstdint>

namespace antidrone_turret {

enum class ActuatorState : std::uint8_t {
  kReady,
  kReloading,
};

struct TriggerResult {
  bool accepted{false};
  std::uint32_t trigger_count{0};
};

class ActuatorModel {
public:
  [[nodiscard]] ActuatorState state() const
  {
    return ready_ ? ActuatorState::kReady : ActuatorState::kReloading;
  }

  [[nodiscard]] std::uint32_t trigger_count() const
  {
    return trigger_count_;
  }

  [[nodiscard]] bool ready() const
  {
    return ready_;
  }

  TriggerResult trigger()
  {
    if (!ready_) {
      return TriggerResult{false, trigger_count_};
    }

    ++trigger_count_;
    ready_ = false;
    return TriggerResult{true, trigger_count_};
  }

  void mark_ready()
  {
    ready_ = true;
  }

private:
  bool ready_{true};
  std::uint32_t trigger_count_{0};
};

}  // namespace antidrone_turret
