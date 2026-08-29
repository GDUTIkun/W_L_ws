#include "wheel_leg_core/wheel_position_planner.hpp"

#include <algorithm>
#include <limits>

namespace wheel_leg {
namespace {

double clamp(double value, double lower, double upper) {
  return std::min(std::max(value, lower), upper);
}

}  // namespace

WheelPositionPlanner::WheelPositionPlanner() = default;

WheelPositionPlanner::WheelPositionPlanner(Config config) : config_(config) {}

bool WheelPositionPlanner::valid() const {
  return std::isfinite(config_.frequency_hz) && config_.frequency_hz > 0.0 &&
      std::isfinite(config_.damping) && config_.damping > 0.0 &&
      std::isfinite(config_.velocity_max_m_s) &&
      config_.velocity_max_m_s > 0.0 &&
      std::isfinite(config_.acceleration_max_m_s2) &&
      config_.acceleration_max_m_s2 > 0.0 &&
      std::isfinite(config_.position_min_m) &&
      std::isfinite(config_.position_max_m) &&
      config_.position_min_m < config_.position_max_m;
}

WheelPositionPlanner::Output WheelPositionPlanner::reset(
    double measured_common_position_m,
    double measured_common_velocity_m_s) {
  if (!valid() || !std::isfinite(measured_common_position_m) ||
      !std::isfinite(measured_common_velocity_m_s) ||
      measured_common_position_m < config_.position_min_m ||
      measured_common_position_m > config_.position_max_m ||
      std::abs(measured_common_velocity_m_s) > config_.velocity_max_m_s) {
    initialized_ = false;
    output_ = Output{};
    return output_;
  }
  output_.common_position_m = measured_common_position_m;
  output_.common_velocity_m_s = measured_common_velocity_m_s;
  output_.common_acceleration_m_s2 = 0.0;
  initialized_ = true;
  return output_;
}

WheelPositionPlanner::Output WheelPositionPlanner::step(
    double target_common_position_m, double dt_s) {
  if (!initialized_ || !valid() || !std::isfinite(target_common_position_m) ||
      !std::isfinite(dt_s) || dt_s <= 0.0) {
    return output_;
  }
  constexpr double kPi = 3.14159265358979323846;
  const double omega = 2.0 * kPi * config_.frequency_hz;
  const double target = clamp(
      target_common_position_m, config_.position_min_m,
      config_.position_max_m);
  const double requested_acceleration =
      omega * omega * (target - output_.common_position_m) -
      2.0 * config_.damping * omega * output_.common_velocity_m_s;
  const double acceleration = clamp(
      requested_acceleration, -config_.acceleration_max_m_s2,
      config_.acceleration_max_m_s2);
  const double previous_velocity = output_.common_velocity_m_s;
  output_.common_velocity_m_s = clamp(
      previous_velocity + dt_s * acceleration,
      -config_.velocity_max_m_s, config_.velocity_max_m_s);
  const double candidate =
      output_.common_position_m + dt_s * output_.common_velocity_m_s;
  output_.common_position_m = clamp(
      candidate, config_.position_min_m, config_.position_max_m);
  if (output_.common_position_m != candidate) {
    output_.common_velocity_m_s = 0.0;
  }
  output_.common_acceleration_m_s2 =
      (output_.common_velocity_m_s - previous_velocity) / dt_s;
  return output_;
}

}  // namespace wheel_leg
