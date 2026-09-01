#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "gtest/gtest.h"
#include "mujoco/mujoco.h"
#include "wheel_leg_mujoco/adapter.hpp"

namespace {

struct ModelDeleter {
  void operator()(mjModel *model) const { mj_deleteModel(model); }
};

struct DataDeleter {
  void operator()(mjData *data) const { mj_deleteData(data); }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

ModelPtr loadModel() {
  char error[1024]{};
  ModelPtr model(
      mj_loadXML(WHEEL_LEG_SCENE_PATH, nullptr, error, sizeof(error)));
  EXPECT_NE(model, nullptr) << error;
  return model;
}

int objectId(const mjModel *model, mjtObj type, const char *name) {
  const int id = mj_name2id(model, type, name);
  EXPECT_GE(id, 0) << name;
  return id;
}

double segmentAngle(const mjtNum *from, const mjtNum *to) {
  return std::atan2(to[0] - from[0], -(to[2] - from[2]));
}

}  // namespace

TEST(Adapter, ModelInvariantsAndCanonicalOffsets) {
  auto model = loadModel();
  ASSERT_NE(model, nullptr);
  EXPECT_STREQ(mj_versionString(), "3.7.0");
  EXPECT_EQ(model->nu, 6);
  EXPECT_NEAR(model->opt.timestep, 0.002, 1.0e-12);

  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::Adapter adapter(model.get());
  adapter.reset(data.get());
  const int weld = objectId(model.get(), mjOBJ_EQUALITY, "base_weld");
  EXPECT_EQ(data->eq_active[weld], 1);

  const auto state = adapter.extractState(data.get());
  EXPECT_EQ(state.sample_time_ns, 0U);
  for (std::size_t index = 0; index < wheel_leg::kJointCount; ++index) {
    EXPECT_NEAR(
        state.joint_position_rad[index],
        wheel_leg_mujoco::kDefaultJointOffsetsRad[index], 1.0e-12);
    EXPECT_DOUBLE_EQ(state.joint_velocity_rad_s[index], 0.0);
  }
}

TEST(Adapter, SecondPosePreservesGeometricJointAngles) {
  auto model = loadModel();
  ASSERT_NE(model, nullptr);
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::Adapter adapter(model.get());
  adapter.reset(data.get());

  const std::array<double, 6> deltas{0.10, -0.07, 0.13, -0.08, 0.11, -0.17};
  constexpr std::array<const char *, 6> joint_names{
      "left_hip_joint", "left_knee_joint", "left_wheel_joint",
      "right_hip_joint", "right_knee_joint", "right_wheel_joint"};
  for (std::size_t index = 0; index < deltas.size(); ++index) {
    const int joint = objectId(model.get(), mjOBJ_JOINT, joint_names[index]);
    data->qpos[model->jnt_qposadr[joint]] += deltas[index];
  }
  mj_forward(model.get(), data.get());
  const auto state = adapter.extractState(data.get());
  for (std::size_t index = 0; index < deltas.size(); ++index) {
    EXPECT_NEAR(
        state.joint_position_rad[index],
        wheel_leg_mujoco::kDefaultJointOffsetsRad[index] - deltas[index],
        1.0e-12);
  }

  for (std::size_t side = 0; side < 2; ++side) {
    const char *hip_name = side == 0 ? "left_hip_joint" : "right_hip_joint";
    const char *knee_name = side == 0 ? "left_knee_joint" : "right_knee_joint";
    const char *wheel_name = side == 0 ? "left_wheel_body" : "right_wheel_body";
    const int hip = objectId(model.get(), mjOBJ_JOINT, hip_name);
    const int knee = objectId(model.get(), mjOBJ_JOINT, knee_name);
    const int wheel = objectId(model.get(), mjOBJ_BODY, wheel_name);
    const double hip_angle = segmentAngle(
        data->xanchor + 3 * hip, data->xanchor + 3 * knee);
    const double shank_angle = segmentAngle(
        data->xanchor + 3 * knee, data->xpos + 3 * wheel);
    const std::size_t canonical = side * 3;
    EXPECT_NEAR(hip_angle, state.joint_position_rad[canonical], 2.0e-6);
    EXPECT_NEAR(
        shank_angle - hip_angle,
        state.joint_position_rad[canonical + 1], 2.0e-6);
  }
}

TEST(Adapter, FloatingBaseTwistUsesComSiteAndWorldAxes) {
  auto model = loadModel();
  ASSERT_NE(model, nullptr);
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig config;
  config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), config);
  adapter.reset(data.get());
  const int weld = objectId(model.get(), mjOBJ_EQUALITY, "base_weld");
  EXPECT_EQ(data->eq_active[weld], 0);

  data->qvel[0] = 0.3;
  data->qvel[1] = -0.2;
  data->qvel[2] = 0.1;
  data->qvel[3] = 0.4;
  mj_forward(model.get(), data.get());
  const auto state = adapter.extractState(data.get());
  EXPECT_NEAR(state.base_angular_velocity_n_rad_s[0], 0.4, 1.0e-12);
  EXPECT_NEAR(state.base_angular_velocity_n_rad_s[1], 0.0, 1.0e-12);
  EXPECT_NEAR(state.base_angular_velocity_n_rad_s[2], 0.0, 1.0e-12);

  const auto initial_position = state.base_position_n_m;
  constexpr double epsilon = 1.0e-7;
  mj_integratePos(model.get(), data->qpos, data->qvel, epsilon);
  mj_forward(model.get(), data.get());
  const int site = objectId(model.get(), mjOBJ_SITE, "base_control_frame");
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double finite_difference =
        (data->site_xpos[3 * site + axis] - initial_position[axis]) / epsilon;
    EXPECT_NEAR(
        state.base_linear_velocity_n_m_s[axis], finite_difference, 2.0e-7);
  }
}

TEST(Adapter, TorqueMappingAndWatchdogsFailToZero) {
  auto model = loadModel();
  ASSERT_NE(model, nullptr);
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig config;
  config.command_enabled = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), config);
  adapter.reset(data.get());

  wheel_leg::TorqueCommand command;
  for (std::size_t active = 0; active < wheel_leg::kJointCount; ++active) {
    command.source_sample_time_ns = active;
    command.joint_torque_nm.fill(0.0);
    command.joint_torque_nm[active] = static_cast<double>(active + 1);
    data->time = static_cast<double>(active) * 1.0e-9;
    ASSERT_TRUE(adapter.acceptCommand(command, 1'000U + active, active));
    adapter.writeControls(data.get(), 1'000U + active);
    for (std::size_t index = 0; index < wheel_leg::kJointCount; ++index) {
      const double expected =
          index == active ? -command.joint_torque_nm[index] : 0.0;
      EXPECT_DOUBLE_EQ(data->ctrl[index], expected);
    }
  }

  adapter.writeControls(data.get(), 100'001'006U);
  for (int index = 0; index < model->nu; ++index) {
    EXPECT_DOUBLE_EQ(data->ctrl[index], 0.0);
  }

  command.source_sample_time_ns = 7;
  EXPECT_FALSE(adapter.acceptCommand(command, 2'000U, 6U));
  adapter.writeControls(data.get(), 2'000U);
  for (int index = 0; index < model->nu; ++index) {
    EXPECT_DOUBLE_EQ(data->ctrl[index], 0.0);
  }
  command.source_sample_time_ns = 50'000'006U;
  EXPECT_FALSE(adapter.acceptCommand(command, 2'000U, 100'000'007U));
  command.source_sample_time_ns = 5;
  EXPECT_FALSE(adapter.acceptCommand(command, 2'000U, 6U));

  command.source_sample_time_ns = 6;
  command.joint_torque_nm[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.acceptCommand(command, 2'000U, 1U));
  adapter.reset(data.get());
  for (int index = 0; index < model->nu; ++index) {
    EXPECT_DOUBLE_EQ(data->ctrl[index], 0.0);
  }
}

TEST(Adapter, ContactAggregationUsesOnlyNamedWheelFloorPairs) {
  auto model = loadModel();
  ASSERT_NE(model, nullptr);
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig config;
  config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), config);
  adapter.reset(data.get());

  auto state = adapter.extractState(data.get());
  EXPECT_EQ(state.contact_state[0], wheel_leg::ContactState::kNoContact);
  EXPECT_EQ(state.contact_state[1], wheel_leg::ContactState::kNoContact);

  data->qpos[2] = 0.25;
  const double roll = -0.25;
  data->qpos[3] = std::cos(roll / 2.0);
  data->qpos[4] = std::sin(roll / 2.0);
  data->qpos[5] = 0.0;
  data->qpos[6] = 0.0;
  state = adapter.extractState(data.get());
  EXPECT_EQ(state.contact_state[0], wheel_leg::ContactState::kContact);
  EXPECT_EQ(state.contact_state[1], wheel_leg::ContactState::kNoContact);

  data->qpos[4] = -data->qpos[4];
  state = adapter.extractState(data.get());
  EXPECT_EQ(state.contact_state[0], wheel_leg::ContactState::kNoContact);
  EXPECT_EQ(state.contact_state[1], wheel_leg::ContactState::kContact);
}

TEST(Adapter, ResetReplaysFloatingZeroTorqueDeterministically) {
  auto model = loadModel();
  ASSERT_NE(model, nullptr);
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig config;
  config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), config);

  auto run = [&]() {
    adapter.reset(data.get());
    for (int step = 0; step < 20; ++step) {
      adapter.writeControls(data.get(), static_cast<std::uint64_t>(step));
      mj_step(model.get(), data.get());
    }
    return adapter.extractState(data.get());
  };
  const auto first = run();
  const auto second = run();
  EXPECT_EQ(first.sample_time_ns, 40'000'000U);
  EXPECT_EQ(second.sample_time_ns, first.sample_time_ns);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    EXPECT_DOUBLE_EQ(second.base_position_n_m[axis], first.base_position_n_m[axis]);
    EXPECT_DOUBLE_EQ(
        second.base_linear_velocity_n_m_s[axis],
        first.base_linear_velocity_n_m_s[axis]);
    EXPECT_TRUE(std::isfinite(second.base_position_n_m[axis]));
    EXPECT_TRUE(std::isfinite(second.base_linear_velocity_n_m_s[axis]));
  }
}

TEST(Adapter, CurrentWeightedWbcH0MatchesFrozenStateAndResets) {
  char error[1024]{};
  ModelPtr model(mj_loadXML(
      WHEEL_LEG_CURRENT_WBC_SCENE_PATH, nullptr, error, sizeof(error)));
  ASSERT_NE(model, nullptr) << error;
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig config;
  config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), config);

  auto initialize = [&]() {
    adapter.reset(data.get());
    wheel_leg_mujoco::initializeCurrentWeightedWbcH0(model.get(), data.get());
    return adapter.extractState(data.get());
  };
  const auto first = initialize();
  const auto second = initialize();
  constexpr std::array<double, 3> base{
      -0.077378152000000006, 8.1e-7, 0.31543998403249462};
  constexpr wheel_leg::JointVector joints{
      -0.97199891583533837, 1.6393957458903228, 0.0,
      -0.98339093564557467, 1.6394010277077622, 0.0};
  for (std::size_t axis = 0; axis < base.size(); ++axis) {
    EXPECT_NEAR(first.base_position_n_m[axis], base[axis], 1.0e-12);
    EXPECT_DOUBLE_EQ(second.base_position_n_m[axis], first.base_position_n_m[axis]);
    EXPECT_DOUBLE_EQ(first.base_linear_velocity_n_m_s[axis], 0.0);
    EXPECT_DOUBLE_EQ(first.base_angular_velocity_n_rad_s[axis], 0.0);
  }
  EXPECT_DOUBLE_EQ(first.q_n_from_b[0], 1.0);
  for (std::size_t joint = 0; joint < joints.size(); ++joint) {
    EXPECT_NEAR(first.joint_position_rad[joint], joints[joint], 1.0e-12);
    EXPECT_DOUBLE_EQ(second.joint_position_rad[joint], first.joint_position_rad[joint]);
    EXPECT_DOUBLE_EQ(first.joint_velocity_rad_s[joint], 0.0);
  }
  EXPECT_EQ(first.contact_state[0], wheel_leg::ContactState::kContact);
  EXPECT_EQ(first.contact_state[1], wheel_leg::ContactState::kContact);
}
