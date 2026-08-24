#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "wheel_leg_stm32_bridge/msg/bridge_status.hpp"
#include "wheel_leg_stm32_bridge/msg/identification_command.hpp"
#include "wheel_leg_stm32_bridge/msg/identification_telemetry.hpp"
#include "wheel_leg_stm32_bridge/msg/normal_command.hpp"
#include "wheel_leg_stm32_bridge/msg/normal_state.hpp"
#include "wheel_leg_stm32_bridge/protocol.hpp"
#include "wheel_leg_stm32_bridge/serial_port.hpp"

namespace wheel_leg_stm32_bridge {
namespace {

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

bool finiteNormalCommand(const msg::NormalCommand &message) {
  return std::all_of(
      message.efforts_nm.begin(), message.efforts_nm.end(),
      [](float value) { return std::isfinite(value); });
}

bool validIdentificationCommand(const msg::IdentificationCommand &message) {
  return (message.actuator_index == 2U || message.actuator_index == 5U) &&
         message.excitation <= 1U && std::isfinite(message.target_current_a) &&
         std::abs(message.target_current_a) <= 1.0F &&
         message.step_delay_ms <= 5000U &&
         message.step_duration_ms <= 2000U &&
         !(message.excitation == 1U && message.step_duration_ms == 0U);
}

std::array<double, 4> quaternionFromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy,
  };
}

}  // namespace

class Stm32BridgeNode final : public rclcpp::Node {
 public:
  Stm32BridgeNode() : Node("stm32_bridge") {
    serial_device_ = declare_parameter<std::string>("serial_device", "/dev/ttyAMA4");
    baud_rate_ = declare_parameter<int>("baud_rate", 921600);
    const double command_rate_hz =
        declare_parameter<double>("command_rate_hz", 200.0);
    command_source_timeout_ms_ =
        declare_parameter<int>("command_source_timeout_ms", 100);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"left_hip", "left_knee", "left_wheel", "right_hip",
         "right_knee", "right_wheel"});

    if (joint_names_.size() != protocol::kJointCount) {
      throw std::runtime_error("joint_names must contain exactly six names");
    }
    if (command_rate_hz <= 0.0 || command_rate_hz > 1000.0) {
      throw std::runtime_error("command_rate_hz must be in (0, 1000]");
    }
    if (command_source_timeout_ms_ <= 0) {
      throw std::runtime_error("command_source_timeout_ms must be positive");
    }

    normal_state_publisher_ =
        create_publisher<msg::NormalState>("~/normal_state", rclcpp::SensorDataQoS());
    identification_publisher_ = create_publisher<msg::IdentificationTelemetry>(
        "~/identification_telemetry", rclcpp::SensorDataQoS());
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_states", rclcpp::SensorDataQoS());
    imu_publisher_ =
        create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<msg::BridgeStatus>("~/status", 10);

    normal_command_subscription_ = create_subscription<msg::NormalCommand>(
        "~/normal_command", 10,
        std::bind(&Stm32BridgeNode::onNormalCommand, this, std::placeholders::_1));
    identification_subscription_ =
        create_subscription<msg::IdentificationCommand>(
            "~/identification_command", 10,
            std::bind(
                &Stm32BridgeNode::onIdentificationCommand, this,
                std::placeholders::_1));

    io_timer_ = create_wall_timer(1ms, std::bind(&Stm32BridgeNode::onIoTimer, this));
    const auto tx_period = std::chrono::nanoseconds(
        static_cast<std::int64_t>(1.0e9 / command_rate_hz));
    tx_timer_ = create_wall_timer(
        tx_period, std::bind(&Stm32BridgeNode::onTxTimer, this));
    status_timer_ = create_wall_timer(
        1s, std::bind(&Stm32BridgeNode::publishStatus, this));
    next_reconnect_ = SteadyClock::now();

    RCLCPP_INFO(
        get_logger(),
        "STM32 experimental bridge configured for %s at %d baud",
        serial_device_.c_str(), baud_rate_);
  }

 private:
  enum class CommandMode { kNone, kNormal, kIdentification };

  void onNormalCommand(const msg::NormalCommand::SharedPtr message) {
    if (!finiteNormalCommand(*message)) {
      RCLCPP_ERROR(get_logger(), "Rejected normal command containing NaN/Inf");
      return;
    }
    normal_command_ = *message;
    command_mode_ = CommandMode::kNormal;
    last_command_source_time_ = SteadyClock::now();
    safe_frame_sent_ = false;
  }

  void onIdentificationCommand(
      const msg::IdentificationCommand::SharedPtr message) {
    if (!validIdentificationCommand(*message)) {
      RCLCPP_ERROR(
          get_logger(),
          "Rejected invalid identification command (index/excitation/value/timing)");
      return;
    }
    identification_command_ = *message;
    command_mode_ = CommandMode::kIdentification;
    last_command_source_time_ = SteadyClock::now();
    safe_frame_sent_ = false;
  }

  bool ensureSerialOpen() {
    if (serial_.isOpen()) {
      return true;
    }
    const auto now = SteadyClock::now();
    if (now < next_reconnect_) {
      return false;
    }
    next_reconnect_ = now + 1s;
    std::string error;
    if (!serial_.open(serial_device_, baud_rate_, error)) {
      RCLCPP_WARN(
          get_logger(), "Cannot open %s: %s; retrying",
          serial_device_.c_str(), error.c_str());
      return false;
    }
    pending_tx_.clear();
    pending_tx_offset_ = 0U;
    RCLCPP_INFO(get_logger(), "Opened %s", serial_device_.c_str());
    return true;
  }

  void closeSerial(const std::string &reason) {
    if (serial_.isOpen()) {
      RCLCPP_ERROR(get_logger(), "Serial connection closed: %s", reason.c_str());
    }
    serial_.close();
    pending_tx_.clear();
    pending_tx_offset_ = 0U;
    next_reconnect_ = SteadyClock::now() + 1s;
  }

  void onIoTimer() {
    if (!ensureSerialOpen()) {
      return;
    }
    flushTransmit();
    if (!serial_.isOpen()) {
      return;
    }

    std::array<std::uint8_t, 4096> buffer{};
    std::string error;
    while (true) {
      const auto count = serial_.read(buffer.data(), buffer.size(), error);
      if (count < 0) {
        closeSerial(error);
        return;
      }
      if (count == 0) {
        break;
      }
      for (std::ptrdiff_t i = 0; i < count; ++i) {
        const auto frame = parser_.consume(buffer[static_cast<std::size_t>(i)]);
        if (frame.has_value()) {
          processFrame(*frame);
        }
      }
      if (static_cast<std::size_t>(count) < buffer.size()) {
        break;
      }
    }
  }

  void flushTransmit() {
    if (pending_tx_.empty()) {
      return;
    }
    std::string error;
    const std::size_t remaining = pending_tx_.size() - pending_tx_offset_;
    const auto count = serial_.write(
        pending_tx_.data() + pending_tx_offset_, remaining, error);
    if (count < 0) {
      ++tx_write_errors_;
      closeSerial(error);
      return;
    }
    if (count == 0) {
      return;
    }
    if (static_cast<std::size_t>(count) < remaining) {
      ++tx_partial_writes_;
    }
    pending_tx_offset_ += static_cast<std::size_t>(count);
    if (pending_tx_offset_ == pending_tx_.size()) {
      ++tx_frames_;
      tx_bytes_ += pending_tx_.size();
      pending_tx_.clear();
      pending_tx_offset_ = 0U;
    }
  }

  void onTxTimer() {
    if (!ensureSerialOpen() || command_mode_ == CommandMode::kNone) {
      return;
    }
    if (!pending_tx_.empty()) {
      ++tx_skipped_busy_;
      return;
    }

    const bool stale =
        SteadyClock::now() - last_command_source_time_ >
        std::chrono::milliseconds(command_source_timeout_ms_);
    if (stale) {
      if (!safe_frame_sent_) {
        queueSafeCommand();
        safe_frame_sent_ = true;
        RCLCPP_WARN(
            get_logger(),
            "ROS command source timed out; sent one estop frame and stopped heartbeats");
      }
      return;
    }

    if (command_mode_ == CommandMode::kNormal) {
      protocol::NormalCommand command;
      command.enable = normal_command_.enable;
      command.estop = normal_command_.estop;
      std::copy(
          normal_command_.efforts_nm.begin(), normal_command_.efforts_nm.end(),
          command.efforts_nm.begin());
      queueFrame(protocol::kNormalCommandType, protocol::encodeNormalCommand(command));
    } else {
      queueIdentificationCommand(false);
    }
  }

  void queueSafeCommand() {
    if (command_mode_ == CommandMode::kIdentification) {
      queueIdentificationCommand(true);
      return;
    }
    protocol::NormalCommand command;
    command.enable = false;
    command.estop = true;
    queueFrame(protocol::kNormalCommandType, protocol::encodeNormalCommand(command));
  }

  void queueIdentificationCommand(bool force_estop) {
    protocol::IdentificationCommand command;
    command.enable = force_estop ? false : identification_command_.enable;
    command.estop = force_estop ? true : identification_command_.estop;
    command.actuator_index = identification_command_.actuator_index;
    command.excitation = identification_command_.excitation;
    command.trial_id = identification_command_.trial_id;
    command.target_current_a = identification_command_.target_current_a;
    command.step_delay_ms = identification_command_.step_delay_ms;
    command.step_duration_ms = identification_command_.step_duration_ms;
    queueFrame(
        protocol::kIdentificationCommandType,
        protocol::encodeIdentificationCommand(command));
  }

  void queueFrame(
      std::uint8_t type, const std::vector<std::uint8_t> &payload) {
    pending_tx_ = protocol::encodeFrame(type, tx_sequence_, payload);
    pending_tx_offset_ = 0U;
    ++tx_sequence_;
  }

  void processFrame(const protocol::Frame &frame) {
    const auto now = SteadyClock::now();
    if (rx_sequence_initialized_) {
      const auto expected = static_cast<std::uint16_t>(last_rx_sequence_ + 1U);
      if (frame.sequence != expected) {
        ++rx_sequence_gaps_;
      }
    }
    rx_sequence_initialized_ = true;
    last_rx_sequence_ = frame.sequence;
    last_rx_type_ = frame.type;
    last_rx_time_ = now;

    if (frame.type == protocol::kNormalStateType) {
      protocol::NormalState state;
      if (!protocol::decodeNormalState(frame.payload, state)) {
        ++rx_payload_errors_;
        return;
      }
      publishNormalState(frame.sequence, state);
      return;
    }
    if (frame.type == protocol::kIdentificationStateType) {
      protocol::IdentificationTelemetry telemetry;
      if (!protocol::decodeIdentificationTelemetry(frame.payload, telemetry) ||
          telemetry.protocol_version != protocol::kIdentificationProtocolVersion) {
        ++rx_payload_errors_;
        return;
      }
      publishIdentificationTelemetry(frame.sequence, telemetry);
      return;
    }
    ++rx_unknown_type_;
  }

  void publishNormalState(
      std::uint16_t sequence, const protocol::NormalState &state) {
    const auto stamp = now();
    msg::NormalState normal_message;
    normal_message.receipt_stamp = stamp;
    normal_message.frame_seq = sequence;
    normal_message.stm_tick_ms = state.stm_tick_ms;
    normal_message.roll_rad = state.roll_rad;
    normal_message.pitch_rad = state.pitch_rad;
    normal_message.yaw_rad = state.yaw_rad;
    normal_message.angular_velocity_rad_s = state.angular_velocity_rad_s;
    normal_message.linear_acceleration_m_s2 = state.linear_acceleration_m_s2;
    normal_message.position_rad = state.position_rad;
    normal_message.velocity_rad_s = state.velocity_rad_s;
    normal_message.feedback_torque_nm = state.feedback_torque_nm;
    normal_message.online_mask = state.online_mask;
    normal_message.safety_state = state.safety_state;
    normal_message.last_command_timeout = state.last_command_timeout;
    normal_message.knee_limit_flag = state.knee_limit_flag;
    normal_message.comm_rx_error_count = state.comm_rx_error_count;
    normal_message.comm_crc_error_count = state.comm_crc_error_count;
    normal_message.can_error_count = state.can_error_count;
    normal_state_publisher_->publish(normal_message);

    sensor_msgs::msg::JointState joint_message;
    joint_message.header.stamp = stamp;
    joint_message.name = joint_names_;
    joint_message.position.assign(state.position_rad.begin(), state.position_rad.end());
    joint_message.velocity.assign(
        state.velocity_rad_s.begin(), state.velocity_rad_s.end());
    joint_message.effort.assign(
        state.feedback_torque_nm.begin(), state.feedback_torque_nm.end());
    joint_state_publisher_->publish(joint_message);

    sensor_msgs::msg::Imu imu_message;
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = frame_id_;
    const auto quaternion =
        quaternionFromRpy(state.roll_rad, state.pitch_rad, state.yaw_rad);
    imu_message.orientation.x = quaternion[0];
    imu_message.orientation.y = quaternion[1];
    imu_message.orientation.z = quaternion[2];
    imu_message.orientation.w = quaternion[3];
    imu_message.angular_velocity.x = state.angular_velocity_rad_s[0];
    imu_message.angular_velocity.y = state.angular_velocity_rad_s[1];
    imu_message.angular_velocity.z = state.angular_velocity_rad_s[2];
    imu_message.linear_acceleration.x = state.linear_acceleration_m_s2[0];
    imu_message.linear_acceleration.y = state.linear_acceleration_m_s2[1];
    imu_message.linear_acceleration.z = state.linear_acceleration_m_s2[2];
    imu_publisher_->publish(imu_message);
  }

  void publishIdentificationTelemetry(
      std::uint16_t sequence,
      const protocol::IdentificationTelemetry &telemetry) {
    msg::IdentificationTelemetry message;
    message.receipt_stamp = now();
    message.frame_seq = sequence;
    message.protocol_version = telemetry.protocol_version;
    message.actuator_index = telemetry.actuator_index;
    message.actuator_type = telemetry.actuator_type;
    message.excitation = telemetry.excitation;
    message.safety_state = telemetry.safety_state;
    message.flags = telemetry.flags;
    message.selected_online = telemetry.selected_online;
    message.step_state = telemetry.step_state;
    message.sample_seq = telemetry.sample_seq;
    message.stm_tick_ms = telemetry.stm_tick_ms;
    message.trial_id = telemetry.trial_id;
    message.current_requested_a = telemetry.current_requested_a;
    message.current_applied_a = telemetry.current_applied_a;
    message.driver_command_raw = telemetry.driver_command_raw;
    message.driver_feedback_raw = telemetry.driver_feedback_raw;
    message.feedback_current_a = telemetry.feedback_current_a;
    message.position_rad = telemetry.position_rad;
    message.velocity_rad_s = telemetry.velocity_rad_s;
    message.command_age_ms = telemetry.command_age_ms;
    message.comm_rx_error_count = telemetry.comm_rx_error_count;
    identification_publisher_->publish(message);
  }

  void publishStatus() {
    const auto &parser_stats = parser_.stats();
    msg::BridgeStatus message;
    message.stamp = now();
    message.serial_open = serial_.isOpen();
    message.rx_bytes = parser_stats.rx_bytes;
    message.rx_frames_ok = parser_stats.frames_ok;
    message.rx_crc_errors = parser_stats.crc_errors;
    message.rx_length_errors = parser_stats.length_errors + rx_payload_errors_;
    message.rx_sync_losses = parser_stats.sync_losses;
    message.rx_seq_gaps = rx_sequence_gaps_;
    message.rx_unknown_type = rx_unknown_type_;
    message.tx_frames = tx_frames_;
    message.tx_bytes = tx_bytes_;
    message.tx_write_errors = tx_write_errors_;
    message.tx_partial_writes = tx_partial_writes_;
    message.tx_skipped_busy = tx_skipped_busy_;
    message.last_rx_type = last_rx_type_;
    message.last_rx_seq = last_rx_sequence_;
    if (!rx_sequence_initialized_) {
      message.last_rx_age_ms = std::numeric_limits<std::uint32_t>::max();
    } else {
      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
          SteadyClock::now() - last_rx_time_).count();
      message.last_rx_age_ms = static_cast<std::uint32_t>(std::min<std::int64_t>(
          age, std::numeric_limits<std::uint32_t>::max()));
    }
    status_publisher_->publish(message);
  }

  std::string serial_device_;
  int baud_rate_{921600};
  int command_source_timeout_ms_{100};
  std::string frame_id_;
  std::vector<std::string> joint_names_;

  SerialPort serial_;
  protocol::StreamParser parser_;
  CommandMode command_mode_{CommandMode::kNone};
  msg::NormalCommand normal_command_{};
  msg::IdentificationCommand identification_command_{};
  SteadyClock::time_point last_command_source_time_{};
  SteadyClock::time_point next_reconnect_{};
  SteadyClock::time_point last_rx_time_{};
  bool safe_frame_sent_{false};
  bool rx_sequence_initialized_{false};

  std::vector<std::uint8_t> pending_tx_;
  std::size_t pending_tx_offset_{0U};
  std::uint16_t tx_sequence_{0U};
  std::uint16_t last_rx_sequence_{0U};
  std::uint8_t last_rx_type_{0U};
  std::uint64_t rx_sequence_gaps_{0U};
  std::uint64_t rx_payload_errors_{0U};
  std::uint64_t rx_unknown_type_{0U};
  std::uint64_t tx_frames_{0U};
  std::uint64_t tx_bytes_{0U};
  std::uint64_t tx_write_errors_{0U};
  std::uint64_t tx_partial_writes_{0U};
  std::uint64_t tx_skipped_busy_{0U};

  rclcpp::Publisher<msg::NormalState>::SharedPtr normal_state_publisher_;
  rclcpp::Publisher<msg::IdentificationTelemetry>::SharedPtr
      identification_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<msg::BridgeStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<msg::NormalCommand>::SharedPtr
      normal_command_subscription_;
  rclcpp::Subscription<msg::IdentificationCommand>::SharedPtr
      identification_subscription_;
  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace wheel_leg_stm32_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_leg_stm32_bridge::Stm32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
