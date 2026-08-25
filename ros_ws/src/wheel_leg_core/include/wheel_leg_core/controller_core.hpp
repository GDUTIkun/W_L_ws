#pragma once

#include <cstdint>
#include <optional>

#include "wheel_leg_core/types.hpp"

namespace wheel_leg {

struct ControllerConfig {
  std::uint64_t max_state_age_ns{100'000'000};
  double quaternion_norm_tolerance{1.0e-6};
};

enum class StepStatus {
  kOk,
  kNotConfigured,
  kInvalidState,
  kFutureState,
  kStaleState,
  kNonMonotonicState,
};

struct StepResult {
  StepStatus status{StepStatus::kNotConfigured};
  double dt_s{0.0};
  TorqueCommand command{};

  [[nodiscard]] bool accepted() const { return status == StepStatus::kOk; }
};

class ControllerCore {
 public:
  [[nodiscard]] bool configure(const ControllerConfig &config);
  void reset();
  [[nodiscard]] StepResult step(
      const RobotState &state, std::uint64_t now_ns);

 private:
  ControllerConfig config_{};
  bool configured_{false};
  std::optional<std::uint64_t> last_sample_time_ns_;
};

}  // namespace wheel_leg
