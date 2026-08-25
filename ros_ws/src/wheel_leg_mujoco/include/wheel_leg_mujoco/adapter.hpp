#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "mujoco/mujoco.h"
#include "wheel_leg_core/types.hpp"

namespace wheel_leg_mujoco {

inline constexpr std::array<double, wheel_leg::kJointCount>
    kDefaultJointOffsetsRad{
        -1.3267204093873923, 2.2088002548867229, 0.0,
        -1.3267204093873923, 2.2088002548867229, 0.0};

struct AdapterConfig {
  std::array<double, wheel_leg::kJointCount> joint_offsets_rad{
      kDefaultJointOffsetsRad};
  std::uint64_t command_timeout_ns{100'000'000U};
  std::uint64_t max_source_lag_ns{50'000'000U};
  bool command_enabled{false};
  bool floating_base{false};
};

class Adapter final {
 public:
  Adapter(const mjModel *model, AdapterConfig config = {});

  void reset(mjData *data);
  [[nodiscard]] wheel_leg::RobotState extractState(mjData *data);
  [[nodiscard]] bool acceptCommand(
      const wheel_leg::TorqueCommand &command,
      std::uint64_t receipt_time_ns,
      std::uint64_t current_source_time_ns);
  void writeControls(mjData *data, std::uint64_t receipt_time_ns);

  [[nodiscard]] static std::uint64_t simulationTimeNs(double time_s);

 private:
  const mjModel *model_;
  AdapterConfig config_;
  int base_control_site_id_{-1};
  int floor_geom_id_{-1};
  int base_weld_id_{-1};
  std::array<int, wheel_leg::kJointCount> joint_ids_{};
  std::array<int, wheel_leg::kJointCount> actuator_ids_{};
  std::array<int, 2> wheel_geom_ids_{};
  std::optional<wheel_leg::TorqueCommand> command_;
  std::optional<std::uint64_t> command_receipt_time_ns_;
  std::optional<std::uint64_t> last_command_source_time_ns_;
  std::optional<wheel_leg::QuaternionWxyz> last_quaternion_;
};

}  // namespace wheel_leg_mujoco
