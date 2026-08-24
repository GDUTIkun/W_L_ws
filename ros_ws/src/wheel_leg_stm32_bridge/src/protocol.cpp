#include "wheel_leg_stm32_bridge/protocol.hpp"

#include <cstring>

namespace wheel_leg_stm32_bridge::protocol {
namespace {

std::uint16_t readU16Le(const std::uint8_t *data) {
  return static_cast<std::uint16_t>(data[0]) |
         (static_cast<std::uint16_t>(data[1]) << 8U);
}

std::uint32_t readU32Le(const std::uint8_t *data) {
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

float readF32Le(const std::uint8_t *data) {
  const std::uint32_t bits = readU32Le(data);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits), "float32 protocol requires 32-bit float");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void appendU16Le(std::vector<std::uint8_t> &data, std::uint16_t value) {
  data.push_back(static_cast<std::uint8_t>(value & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void appendU32Le(std::vector<std::uint8_t> &data, std::uint32_t value) {
  data.push_back(static_cast<std::uint8_t>(value & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

void appendF32Le(std::vector<std::uint8_t> &data, float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  appendU32Le(data, bits);
}

}  // namespace

std::uint16_t crc16Ccitt(const std::uint8_t *data, std::size_t size) {
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
    for (std::uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
              ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
              : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<std::uint8_t> encodeFrame(
    std::uint8_t type, std::uint16_t sequence,
    const std::vector<std::uint8_t> &payload) {
  if (payload.size() > kMaxPayloadSize) {
    return {};
  }

  std::vector<std::uint8_t> frame;
  frame.reserve(payload.size() + 8U);
  frame.push_back(kFrameHead0);
  frame.push_back(kFrameHead1);
  frame.push_back(type);
  frame.push_back(static_cast<std::uint8_t>(payload.size()));
  appendU16Le(frame, sequence);
  frame.insert(frame.end(), payload.begin(), payload.end());
  const auto crc = crc16Ccitt(frame.data() + 2U, 4U + payload.size());
  appendU16Le(frame, crc);
  return frame;
}

std::vector<std::uint8_t> encodeNormalCommand(const NormalCommand &command) {
  std::vector<std::uint8_t> payload;
  payload.reserve(kNormalCommandPayloadSize);
  payload.push_back(command.enable ? 1U : 0U);
  payload.push_back(command.estop ? 1U : 0U);
  for (const float effort : command.efforts_nm) {
    appendF32Le(payload, effort);
  }
  return payload;
}

std::vector<std::uint8_t> encodeIdentificationCommand(
    const IdentificationCommand &command) {
  std::vector<std::uint8_t> payload;
  payload.reserve(kIdentificationCommandPayloadSize);
  payload.push_back(kIdentificationProtocolVersion);
  payload.push_back(command.enable ? 1U : 0U);
  payload.push_back(command.estop ? 1U : 0U);
  payload.push_back(command.actuator_index);
  payload.push_back(command.excitation);
  payload.push_back(command.c620_threshold_current_enabled ? 1U : 0U);
  appendU16Le(payload, 0U);
  appendU32Le(payload, command.trial_id);
  appendF32Le(payload, command.target_torque_nm);
  appendU32Le(payload, command.step_delay_ms);
  appendU32Le(payload, command.step_duration_ms);
  return payload;
}

bool decodeNormalState(
    const std::vector<std::uint8_t> &payload, NormalState &state) {
  if (payload.size() != kNormalStatePayloadSize) {
    return false;
  }

  std::size_t offset = 0U;
  state.stm_tick_ms = readU32Le(payload.data() + offset);
  offset += 4U;
  state.roll_rad = readF32Le(payload.data() + offset);
  offset += 4U;
  state.pitch_rad = readF32Le(payload.data() + offset);
  offset += 4U;
  state.yaw_rad = readF32Le(payload.data() + offset);
  offset += 4U;
  for (float &value : state.angular_velocity_rad_s) {
    value = readF32Le(payload.data() + offset);
    offset += 4U;
  }
  for (float &value : state.linear_acceleration_m_s2) {
    value = readF32Le(payload.data() + offset);
    offset += 4U;
  }

  // Current STM firmware encodes position/velocity/effort per actuator.
  for (std::size_t i = 0; i < kJointCount; ++i) {
    state.position_rad[i] = readF32Le(payload.data() + offset);
    offset += 4U;
    state.velocity_rad_s[i] = readF32Le(payload.data() + offset);
    offset += 4U;
    state.feedback_torque_nm[i] = readF32Le(payload.data() + offset);
    offset += 4U;
  }

  state.online_mask = payload[offset++];
  state.safety_state = payload[offset++];
  state.last_command_timeout = payload[offset++] != 0U;
  state.knee_limit_flag = payload[offset++];
  state.comm_rx_error_count = readU32Le(payload.data() + offset);
  offset += 4U;
  state.comm_crc_error_count = readU32Le(payload.data() + offset);
  offset += 4U;
  state.can_error_count = readU32Le(payload.data() + offset);
  offset += 4U;
  return offset == payload.size();
}

bool decodeIdentificationTelemetry(
    const std::vector<std::uint8_t> &payload,
    IdentificationTelemetry &telemetry) {
  if (payload.size() != kIdentificationStatePayloadSize) {
    return false;
  }

  std::size_t offset = 0U;
  telemetry.protocol_version = payload[offset++];
  telemetry.actuator_index = payload[offset++];
  telemetry.actuator_type = payload[offset++];
  telemetry.excitation = payload[offset++];
  telemetry.safety_state = payload[offset++];
  telemetry.flags = payload[offset++];
  telemetry.selected_online = payload[offset++] != 0U;
  telemetry.step_state = payload[offset++];
  telemetry.sample_seq = readU32Le(payload.data() + offset);
  offset += 4U;
  telemetry.stm_tick_ms = readU32Le(payload.data() + offset);
  offset += 4U;
  telemetry.trial_id = readU32Le(payload.data() + offset);
  offset += 4U;
  telemetry.tau_requested_nm = readF32Le(payload.data() + offset);
  offset += 4U;
  telemetry.tau_applied_nm = readF32Le(payload.data() + offset);
  offset += 4U;
  telemetry.driver_command_raw = readU32Le(payload.data() + offset);
  offset += 4U;
  telemetry.driver_feedback_raw = readU32Le(payload.data() + offset);
  offset += 4U;
  telemetry.feedback_torque_nm = readF32Le(payload.data() + offset);
  offset += 4U;
  telemetry.position_rad = readF32Le(payload.data() + offset);
  offset += 4U;
  telemetry.velocity_rad_s = readF32Le(payload.data() + offset);
  offset += 4U;
  telemetry.command_age_ms = readU32Le(payload.data() + offset);
  offset += 4U;
  telemetry.comm_rx_error_count = readU32Le(payload.data() + offset);
  offset += 4U;
  return offset == payload.size();
}

std::optional<Frame> StreamParser::consume(std::uint8_t byte) {
  ++stats_.rx_bytes;
  switch (state_) {
    case State::kHead0:
      if (byte == kFrameHead0) {
        state_ = State::kHead1;
      }
      break;
    case State::kHead1:
      if (byte == kFrameHead1) {
        state_ = State::kType;
      } else {
        ++stats_.sync_losses;
        state_ = byte == kFrameHead0 ? State::kHead1 : State::kHead0;
      }
      break;
    case State::kType:
      frame_.type = byte;
      state_ = State::kLength;
      break;
    case State::kLength:
      if (byte > kMaxPayloadSize) {
        ++stats_.length_errors;
        resetFrame();
      } else {
        frame_.payload.assign(byte, 0U);
        payload_index_ = 0U;
        state_ = State::kSequenceLow;
      }
      break;
    case State::kSequenceLow:
      frame_.sequence = byte;
      state_ = State::kSequenceHigh;
      break;
    case State::kSequenceHigh:
      frame_.sequence |= static_cast<std::uint16_t>(byte) << 8U;
      state_ = frame_.payload.empty() ? State::kCrcLow : State::kPayload;
      break;
    case State::kPayload:
      frame_.payload[payload_index_++] = byte;
      if (payload_index_ == frame_.payload.size()) {
        state_ = State::kCrcLow;
      }
      break;
    case State::kCrcLow:
      received_crc_ = byte;
      state_ = State::kCrcHigh;
      break;
    case State::kCrcHigh: {
      received_crc_ |= static_cast<std::uint16_t>(byte) << 8U;
      std::vector<std::uint8_t> crc_data;
      crc_data.reserve(4U + frame_.payload.size());
      crc_data.push_back(frame_.type);
      crc_data.push_back(static_cast<std::uint8_t>(frame_.payload.size()));
      appendU16Le(crc_data, frame_.sequence);
      crc_data.insert(crc_data.end(), frame_.payload.begin(), frame_.payload.end());
      if (received_crc_ == crc16Ccitt(crc_data.data(), crc_data.size())) {
        ++stats_.frames_ok;
        Frame completed = frame_;
        resetFrame();
        return completed;
      }
      ++stats_.crc_errors;
      resetFrame();
      break;
    }
  }
  return std::nullopt;
}

void StreamParser::reset() {
  stats_ = {};
  resetFrame();
}

void StreamParser::resetFrame() {
  state_ = State::kHead0;
  frame_ = {};
  payload_index_ = 0U;
  received_crc_ = 0U;
}

}  // namespace wheel_leg_stm32_bridge::protocol
