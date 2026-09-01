#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "wheel_leg_msgs/msg/robot_state.hpp"
#include "wheel_leg_msgs/msg/torque_command.hpp"
#include "wheel_leg_ros/controller_node.hpp"
#include "wheel_leg_ros/conversions.hpp"

namespace {

wheel_leg_msgs::msg::RobotState currentWeightedWbcH0(std::uint64_t time_ns) {
  wheel_leg_msgs::msg::RobotState state;
  state.sample_time_ns = time_ns;
  state.base_position_n_m.x = -0.077378152000000006;
  state.base_position_n_m.y = 8.1e-7;
  state.base_position_n_m.z = 0.31543998403249462;
  state.q_n_from_b.w = 1.0;
  state.joint_position_rad = {
      -0.97199891583533837, 1.6393957458903228, 0.0,
      -0.98339093564557467, 1.6394010277077622, 0.0};
  state.contact_state = {2U, 2U};
  return state;
}

}  // namespace

TEST(ControllerNode, ValidStatePublishesOnlyZeroTorque) {
  auto controller = std::make_shared<wheel_leg_ros::ControllerNode>();
  auto probe = std::make_shared<rclcpp::Node>("wheel_leg_controller_probe");
  auto publisher = probe->create_publisher<wheel_leg_msgs::msg::RobotState>(
      "robot_state", rclcpp::SensorDataQoS());
  auto reset_client = probe->create_client<std_srvs::srv::Trigger>(
      "reset_controller");
  std::atomic<int> received{0};
  std::atomic<bool> all_zero{true};
  auto subscription =
      probe->create_subscription<wheel_leg_msgs::msg::TorqueCommand>(
          "torque_command", 10,
          [&](wheel_leg_msgs::msg::TorqueCommand::SharedPtr message) {
            ++received;
            for (const double torque : message->joint_torque_nm) {
              if (torque != 0.0) {
                all_zero = false;
              }
            }
          });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(probe);
  wheel_leg_msgs::msg::RobotState state;
  state.q_n_from_b.w = 1.0;
  state.sample_time_ns = 10'000'000U;

  auto spin_until = [&](auto predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
  };
  auto spin_for = [&](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };

  ASSERT_TRUE(spin_until(
      [&]() {
        return publisher->get_subscription_count() > 0 &&
               subscription->get_publisher_count() > 0;
      },
      std::chrono::seconds(3)));
  publisher->publish(state);
  ASSERT_TRUE(spin_until(
      [&]() { return received.load() == 1; }, std::chrono::seconds(1)));
  EXPECT_TRUE(all_zero.load());

  const int accepted_count = received.load();
  publisher->publish(state);
  spin_for(std::chrono::milliseconds(100));
  EXPECT_EQ(received.load(), accepted_count);

  ASSERT_TRUE(reset_client->wait_for_service(std::chrono::seconds(1)));
  auto reset_future = reset_client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
      executor.spin_until_future_complete(reset_future, std::chrono::seconds(1)),
      rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(reset_future.get()->success);
  publisher->publish(state);
  ASSERT_TRUE(spin_until(
      [&]() { return received.load() == accepted_count + 1; },
      std::chrono::seconds(1)));
  const int post_reset_count = received.load();

  state.sample_time_ns += 1;
  state.joint_position_rad[0] = std::numeric_limits<double>::quiet_NaN();
  publisher->publish(state);
  spin_for(std::chrono::milliseconds(100));
  EXPECT_EQ(received.load(), post_reset_count);
  (void)subscription;
}

TEST(ControllerNode, ExplicitJointProfileRequiresCompleteValidParameters) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("controller.mode", "joint_pd_gravity"),
      rclcpp::Parameter("controller.enable_pd", true),
      rclcpp::Parameter("controller.enable_gravity", true),
      rclcpp::Parameter(
          "controller.reference_position_rad",
          std::vector<double>{-1.3267204090965414, 2.2088002542738268, 0.0,
                              -1.3267204090965414, 2.2088002542738268, 0.0}),
      rclcpp::Parameter(
          "controller.reference_velocity_rad_s",
          std::vector<double>(wheel_leg::kJointCount, 0.0)),
      rclcpp::Parameter(
          "controller.kp_nm_per_rad",
          std::vector<double>(wheel_leg::kJointCount, 1.0)),
      rclcpp::Parameter(
          "controller.kd_nm_s_per_rad",
          std::vector<double>(wheel_leg::kJointCount, 0.1)),
      rclcpp::Parameter(
          "controller.torque_limit_nm",
          std::vector<double>(wheel_leg::kJointCount, 5.0)),
      rclcpp::Parameter("controller.gravity_profile", "current_nominal"),
  });
  {
    auto controller =
        std::make_shared<wheel_leg_ros::ControllerNode>(options);
    auto probe = std::make_shared<rclcpp::Node>("wheel_leg_profile_probe");
    auto publisher = probe->create_publisher<wheel_leg_msgs::msg::RobotState>(
        "robot_state", rclcpp::SensorDataQoS());
    std::atomic<bool> received{false};
    std::atomic<bool> finite_nonzero_same_time{false};
    auto subscription =
        probe->create_subscription<wheel_leg_msgs::msg::TorqueCommand>(
            "torque_command", 10,
            [&](wheel_leg_msgs::msg::TorqueCommand::SharedPtr message) {
              bool finite = true;
              bool nonzero = false;
              for (const double torque : message->joint_torque_nm) {
                finite &= std::isfinite(torque);
                nonzero |= torque != 0.0;
              }
              finite_nonzero_same_time =
                  finite && nonzero && message->source_sample_time_ns == 123U;
              received = true;
            });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller);
    executor.add_node(probe);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((publisher->get_subscription_count() == 0 ||
            subscription->get_publisher_count() == 0) &&
           std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0U);
    wheel_leg_msgs::msg::RobotState state;
    state.q_n_from_b.w = 1.0;
    state.sample_time_ns = 123U;
    publisher->publish(state);
    const auto message_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!received && std::chrono::steady_clock::now() < message_deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_TRUE(received);
    EXPECT_TRUE(finite_nonzero_same_time);
  }

  rclcpp::NodeOptions configured_options;
  configured_options.parameter_overrides({
      rclcpp::Parameter("controller.mode", "joint_pd_gravity"),
      rclcpp::Parameter(
          "controller.reference_position_rad",
          std::vector<double>(wheel_leg::kJointCount, 0.0)),
      rclcpp::Parameter(
          "controller.reference_velocity_rad_s",
          std::vector<double>(wheel_leg::kJointCount, 0.0)),
      rclcpp::Parameter(
          "controller.kp_nm_per_rad",
          std::vector<double>(wheel_leg::kJointCount, 1.0)),
      rclcpp::Parameter(
          "controller.kd_nm_s_per_rad",
          std::vector<double>(wheel_leg::kJointCount, 0.1)),
      rclcpp::Parameter(
          "controller.torque_limit_nm",
          std::vector<double>(wheel_leg::kJointCount, 5.0)),
      rclcpp::Parameter("controller.gravity_profile", "configured_harmonics"),
      rclcpp::Parameter(
          "controller.gravity_canonical_offset_rad",
          std::vector<double>(wheel_leg::kJointCount, 0.0)),
      rclcpp::Parameter(
          "controller.gravity_left_sin_torque_nm",
          std::vector<double>(3, 0.0)),
      rclcpp::Parameter(
          "controller.gravity_left_cos_torque_nm",
          std::vector<double>(3, 0.0)),
      rclcpp::Parameter(
          "controller.gravity_right_sin_torque_nm",
          std::vector<double>(3, 0.0)),
      rclcpp::Parameter(
          "controller.gravity_right_cos_torque_nm",
          std::vector<double>(3, 0.0)),
  });
  EXPECT_NO_THROW(
      std::make_shared<wheel_leg_ros::ControllerNode>(configured_options));

  rclcpp::NodeOptions invalid_options;
  invalid_options.parameter_overrides({
      rclcpp::Parameter("controller.mode", "joint_pd_gravity"),
      rclcpp::Parameter(
          "controller.torque_limit_nm", std::vector<double>{1.0, 2.0}),
  });
  EXPECT_THROW(
      std::make_shared<wheel_leg_ros::ControllerNode>(invalid_options),
      std::runtime_error);
}

TEST(ControllerNode, WeightedWbcMatchesDirectCoreAtFrozenH0) {
  constexpr std::uint64_t time_ns = 10'000'000U;
  const auto state = currentWeightedWbcH0(time_ns);
  wheel_leg::ControllerCore direct;
  ASSERT_TRUE(direct.configure(
      wheel_leg::currentNominalWeightedWbcControllerConfig()));
  const auto expected = direct.step(wheel_leg_ros::fromRos(state));
  ASSERT_TRUE(expected.accepted());
  ASSERT_TRUE(expected.weighted_wbc_active);
  ASSERT_EQ(expected.weighted_wbc_solver_status,
            wheel_leg::DenseQpSolver::Status::kConverged);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("controller.mode", "weighted_wbc")});
  auto controller =
      std::make_shared<wheel_leg_ros::ControllerNode>(options);
  auto probe = std::make_shared<rclcpp::Node>("wheel_leg_wbc_probe");
  auto publisher = probe->create_publisher<wheel_leg_msgs::msg::RobotState>(
      "robot_state", rclcpp::SensorDataQoS());
  std::atomic<bool> received{false};
  wheel_leg_msgs::msg::TorqueCommand actual;
  auto subscription =
      probe->create_subscription<wheel_leg_msgs::msg::TorqueCommand>(
          "torque_command", 10,
          [&](wheel_leg_msgs::msg::TorqueCommand::SharedPtr message) {
            actual = *message;
            received = true;
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(probe);
  const auto discovery_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while ((publisher->get_subscription_count() == 0 ||
          subscription->get_publisher_count() == 0) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  publisher->publish(state);
  const auto message_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!received && std::chrono::steady_clock::now() < message_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(actual.source_sample_time_ns, time_ns);
  bool nonzero = false;
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    EXPECT_DOUBLE_EQ(
        actual.joint_torque_nm[joint], expected.command.joint_torque_nm[joint]);
    EXPECT_TRUE(std::isfinite(actual.joint_torque_nm[joint]));
    nonzero |= actual.joint_torque_nm[joint] != 0.0;
  }
  EXPECT_TRUE(nonzero);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
