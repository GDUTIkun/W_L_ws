#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wheel_leg_mujoco/adapter.hpp"
#include "wheel_leg_msgs/msg/robot_state.hpp"
#include "wheel_leg_msgs/msg/torque_command.hpp"
#include "wheel_leg_ros/conversions.hpp"

namespace wheel_leg_mujoco {
namespace {

std::uint64_t steadyNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

struct ModelDeleter {
  void operator()(mjModel *model) const { mj_deleteModel(model); }
};

struct DataDeleter {
  void operator()(mjData *data) const { mj_deleteData(data); }
};

}  // namespace

class MuJoCoNode final : public rclcpp::Node {
 public:
  MuJoCoNode() : Node("wheel_leg_mujoco") {
    const std::string model_path = declare_parameter<std::string>("model_path", "");
    if (model_path.empty()) {
      throw std::invalid_argument("model_path parameter is required");
    }
    AdapterConfig config;
    config.floating_base = declare_parameter<bool>("floating_base", false);
    config.command_enabled = declare_parameter<bool>("command_enabled", false);
    const auto command_timeout_ms =
        declare_parameter<std::int64_t>("command_timeout_ms", 100);
    const auto max_source_lag_ms =
        declare_parameter<std::int64_t>("max_source_lag_ms", 50);
    publish_decimation_ =
        declare_parameter<std::int64_t>("state_publish_decimation", 5);
    if (command_timeout_ms <= 0 || max_source_lag_ms <= 0 ||
        publish_decimation_ <= 0) {
      throw std::invalid_argument("MuJoCo timing parameters must be positive");
    }
    config.command_timeout_ns =
        static_cast<std::uint64_t>(command_timeout_ms) * 1'000'000U;
    config.max_source_lag_ns =
        static_cast<std::uint64_t>(max_source_lag_ms) * 1'000'000U;

    char error[1024]{};
    model_.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) {
      throw std::runtime_error(std::string("MuJoCo model load failed: ") + error);
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
      throw std::runtime_error("MuJoCo data allocation failed");
    }
    adapter_ = std::make_unique<Adapter>(model_.get(), config);
    adapter_->reset(data_.get());

    state_publisher_ = create_publisher<wheel_leg_msgs::msg::RobotState>(
        "robot_state", rclcpp::SensorDataQoS());
    command_subscription_ =
        create_subscription<wheel_leg_msgs::msg::TorqueCommand>(
            "torque_command", 10,
            [this](wheel_leg_msgs::msg::TorqueCommand::SharedPtr message) {
              const auto command = wheel_leg_ros::fromRos(*message);
              if (!adapter_->acceptCommand(
                      command, steadyNowNs(),
                      Adapter::simulationTimeNs(data_->time))) {
                RCLCPP_WARN(get_logger(), "Rejected invalid/stale torque command");
              }
            });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "reset_simulation",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          adapter_->reset(data_.get());
          step_count_ = 0;
          response->success = true;
          response->message =
              "Simulation reset; now call reset_controller to accept the new epoch";
        });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(model_->opt.timestep));
    timer_ = create_wall_timer(period, [this]() { step(); });
  }

 private:
  void step() {
    adapter_->writeControls(data_.get(), steadyNowNs());
    mj_step(model_.get(), data_.get());
    ++step_count_;
    if (step_count_ % publish_decimation_ == 0) {
      state_publisher_->publish(
          wheel_leg_ros::toRos(adapter_->extractState(data_.get())));
    }
  }

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  std::unique_ptr<Adapter> adapter_;
  std::int64_t publish_decimation_{5};
  std::int64_t step_count_{0};
  rclcpp::Publisher<wheel_leg_msgs::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::Subscription<wheel_leg_msgs::msg::TorqueCommand>::SharedPtr
      command_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wheel_leg_mujoco

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_leg_mujoco::MuJoCoNode>());
  rclcpp::shutdown();
  return 0;
}
