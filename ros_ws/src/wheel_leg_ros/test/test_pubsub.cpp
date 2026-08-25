#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "wheel_leg_msgs/msg/robot_state.hpp"
#include "wheel_leg_msgs/msg/torque_command.hpp"
#include "wheel_leg_ros/controller_node.hpp"

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
