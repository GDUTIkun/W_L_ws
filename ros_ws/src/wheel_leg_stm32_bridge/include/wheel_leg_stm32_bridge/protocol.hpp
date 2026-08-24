#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace wheel_leg_stm32_bridge::protocol {

constexpr std::uint8_t kFrameHead0 = 0xA5;
constexpr std::uint8_t kFrameHead1 = 0x5A;
constexpr std::uint8_t kNormalCommandType = 0x01;
constexpr std::uint8_t kIdentificationCommandType = 0x02;
constexpr std::uint8_t kNormalStateType = 0x81;
constexpr std::uint8_t kIdentificationStateType = 0x82;
constexpr std::uint8_t kIdentificationProtocolVersion = 2;
constexpr std::size_t kJointCount = 6;
constexpr std::size_t kNormalCommandPayloadSize = 26;
constexpr std::size_t kIdentificationCommandPayloadSize = 24;
constexpr std::size_t kNormalStatePayloadSize = 128;
constexpr std::size_t kIdentificationStatePayloadSize = 56;
constexpr std::size_t kMaxPayloadSize = 160;

struct Frame {
  std::uint8_t type{0};
  std::uint16_t sequence{0};
  std::vector<std::uint8_t> payload;
};

struct ParserStats {
  std::uint64_t rx_bytes{0};
  std::uint64_t frames_ok{0};
  std::uint64_t crc_errors{0};
  std::uint64_t length_errors{0};
  std::uint64_t sync_losses{0};
};

struct NormalCommand {
  bool enable{false};
  bool estop{false};
  std::array<float, kJointCount> efforts_nm{};
};

struct IdentificationCommand {
  bool enable{false};
  bool estop{false};
  std::uint8_t actuator_index{0};
  std::uint8_t excitation{0};
  std::uint32_t trial_id{0};
  float target_current_a{0.0F};
  std::uint32_t step_delay_ms{0};
  std::uint32_t step_duration_ms{0};
};

struct NormalState {
  std::uint32_t stm_tick_ms{0};
  float roll_rad{0.0F};
  float pitch_rad{0.0F};
  float yaw_rad{0.0F};
  std::array<float, 3> angular_velocity_rad_s{};
  std::array<float, 3> linear_acceleration_m_s2{};
  std::array<float, kJointCount> position_rad{};
  std::array<float, kJointCount> velocity_rad_s{};
  std::array<float, kJointCount> feedback_torque_nm{};
  std::uint8_t online_mask{0};
  std::uint8_t safety_state{0};
  bool last_command_timeout{false};
  std::uint8_t knee_limit_flag{0};
  std::uint32_t comm_rx_error_count{0};
  std::uint32_t comm_crc_error_count{0};
  std::uint32_t can_error_count{0};
};

struct IdentificationTelemetry {
  std::uint8_t protocol_version{0};
  std::uint8_t actuator_index{0};
  std::uint8_t actuator_type{0};
  std::uint8_t excitation{0};
  std::uint8_t safety_state{0};
  std::uint8_t flags{0};
  bool selected_online{false};
  std::uint8_t step_state{0};
  std::uint32_t sample_seq{0};
  std::uint32_t stm_tick_ms{0};
  std::uint32_t trial_id{0};
  float current_requested_a{0.0F};
  float current_applied_a{0.0F};
  std::uint32_t driver_command_raw{0};
  std::uint32_t driver_feedback_raw{0};
  float feedback_current_a{0.0F};
  float position_rad{0.0F};
  float velocity_rad_s{0.0F};
  std::uint32_t command_age_ms{0};
  std::uint32_t comm_rx_error_count{0};
};

std::uint16_t crc16Ccitt(const std::uint8_t *data, std::size_t size);
std::vector<std::uint8_t> encodeFrame(
  std::uint8_t type, std::uint16_t sequence,
  const std::vector<std::uint8_t> &payload);
std::vector<std::uint8_t> encodeNormalCommand(const NormalCommand &command);
std::vector<std::uint8_t> encodeIdentificationCommand(
  const IdentificationCommand &command);
bool decodeNormalState(const std::vector<std::uint8_t> &payload, NormalState &state);
bool decodeIdentificationTelemetry(
  const std::vector<std::uint8_t> &payload,
  IdentificationTelemetry &telemetry);

class StreamParser {
 public:
  std::optional<Frame> consume(std::uint8_t byte);
  const ParserStats &stats() const { return stats_; }
  void reset();

 private:
  enum class State {
    kHead0,
    kHead1,
    kType,
    kLength,
    kSequenceLow,
    kSequenceHigh,
    kPayload,
    kCrcLow,
    kCrcHigh,
  };

  void resetFrame();
  State state_{State::kHead0};
  Frame frame_{};
  std::size_t payload_index_{0};
  std::uint16_t received_crc_{0};
  ParserStats stats_{};
};

}  // namespace wheel_leg_stm32_bridge::protocol
