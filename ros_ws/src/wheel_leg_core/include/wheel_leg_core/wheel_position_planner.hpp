#pragma once

#include <cmath>

namespace wheel_leg {

class WheelPositionPlanner {
 public:
  struct Config {
    double frequency_hz{2.0};
    double damping{1.0};
    double velocity_max_m_s{0.15};
    double acceleration_max_m_s2{0.5};
    double position_min_m{-0.3303432354};
    double position_max_m{0.1659029424};
  };

  struct Output {
    double common_position_m{0.0};
    double common_velocity_m_s{0.0};
    double common_acceleration_m_s2{0.0};
  };

  WheelPositionPlanner();
  explicit WheelPositionPlanner(Config config);

  [[nodiscard]] bool valid() const;
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] const Config &config() const { return config_; }
  [[nodiscard]] const Output &output() const { return output_; }

  // Reset is bumpless: the planner starts from an in-contract measured
  // common position and velocity. Out-of-contract measurements fail closed.
  [[nodiscard]] Output reset(
      double measured_common_position_m,
      double measured_common_velocity_m_s);
  [[nodiscard]] Output step(double target_common_position_m, double dt_s);

 private:
  Config config_;
  Output output_{};
  bool initialized_{false};
};

}  // namespace wheel_leg
