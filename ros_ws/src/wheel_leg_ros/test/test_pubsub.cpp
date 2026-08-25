#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "wheel_leg_msgs/msg/robot_state.hpp"
#include "wheel_leg_msgs/msg/torque_command.hpp"
#include "wheel_leg_ros/controller_node.hpp"

namespace {

std::uint64_t steadyNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

}  // namespace

TEST(ControllerNode, ValidStatePublishesOnlyZeroTorque) {
  auto controller = std::make_shared<wheel_leg_ros::ControllerNode>();
  auto probe = std::make_shared<rclcpp::Node>("wheel_leg_controller_probe");
  auto publisher = probe->create_publisher<wheel_leg_msgs::msg::RobotState>(
      "robot_state", rclcpp::SensorDataQoS());
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

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    state.sample_time_ns = steadyNowNs();
    publisher->publish(state);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(received.load(), 0);
  EXPECT_TRUE(all_zero.load());

  for (int index = 0; index < 20; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const int accepted_count = received.load();
  state.sample_time_ns = steadyNowNs() - 200'000'000U;
  publisher->publish(state);
  for (int index = 0; index < 10; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(received.load(), accepted_count);

  state.sample_time_ns = steadyNowNs();
  state.joint_position_rad[0] = std::numeric_limits<double>::quiet_NaN();
  publisher->publish(state);
  for (int index = 0; index < 10; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(received.load(), accepted_count);
  (void)subscription;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
