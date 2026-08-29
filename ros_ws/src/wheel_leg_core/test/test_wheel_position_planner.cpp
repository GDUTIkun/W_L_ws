#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <limits>

#include "wheel_leg_core/wheel_position_planner.hpp"

namespace {

bool near(double actual, double expected, double tolerance = 1.0e-12) {
  return std::abs(actual - expected) <= tolerance;
}

}  // namespace

int main() {
  wheel_leg::WheelPositionPlanner planner;
  assert(planner.valid());
  assert(!planner.initialized());

  const auto reset = planner.reset(-0.08, 0.02);
  assert(planner.initialized());
  assert(near(reset.common_position_m, -0.08));
  assert(near(reset.common_velocity_m_s, 0.02));
  assert(near(reset.common_acceleration_m_s2, 0.0));

  // This vector matches the frozen bounded second-order governor equation:
  // requested acceleration saturates at +0.5 m/s^2.
  const auto first = planner.step(0.10, 0.02);
  assert(near(first.common_acceleration_m_s2, 0.5));
  assert(near(first.common_velocity_m_s, 0.03));
  assert(near(first.common_position_m, -0.0794));

  // Repeated execution is deterministic and respects every bound.
  wheel_leg::WheelPositionPlanner replay;
  const auto replay_reset = replay.reset(-0.08, 0.02);
  assert(near(replay_reset.common_position_m, -0.08));
  auto replay_output = replay.step(0.10, 0.02);
  assert(near(replay_output.common_position_m, first.common_position_m));
  assert(near(replay_output.common_velocity_m_s, first.common_velocity_m_s));
  for (int tick = 0; tick < 1000; ++tick) {
    replay_output = replay.step(1.0, 0.02);
    assert(replay_output.common_position_m >= replay.config().position_min_m);
    assert(replay_output.common_position_m <= replay.config().position_max_m);
    assert(std::abs(replay_output.common_velocity_m_s) <=
           replay.config().velocity_max_m_s);
  }
  assert(near(replay_output.common_position_m,
              replay.config().position_max_m));
  assert(near(replay_output.common_velocity_m_s, 0.0));

  // Out-of-workspace reset and invalid step inputs fail closed.
  const auto rejected = replay.reset(-1.0, -1.0);
  assert(near(rejected.common_position_m, 0.0));
  assert(near(rejected.common_velocity_m_s, 0.0));
  assert(!replay.initialized());
  const auto valid_again = replay.reset(-0.08, 0.02);
  assert(replay.initialized());
  const auto held = replay.step(
      std::numeric_limits<double>::quiet_NaN(), 0.02);
  assert(near(held.common_position_m, valid_again.common_position_m));
  assert(near(held.common_velocity_m_s, valid_again.common_velocity_m_s));

  wheel_leg::WheelPositionPlanner::Config invalid_config;
  invalid_config.position_min_m = invalid_config.position_max_m;
  wheel_leg::WheelPositionPlanner invalid(invalid_config);
  assert(!invalid.valid());
  const auto invalid_reset = invalid.reset(0.0, 0.0);
  assert(near(invalid_reset.common_position_m, 0.0));
  assert(!invalid.initialized());
  return 0;
}
