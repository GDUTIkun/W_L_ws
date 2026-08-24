#include <cstdint>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"

#include "wheel_leg_stm32_bridge/protocol.hpp"

namespace protocol = wheel_leg_stm32_bridge::protocol;

namespace {

void appendU32(std::vector<std::uint8_t> &data, std::uint32_t value) {
  data.push_back(static_cast<std::uint8_t>(value));
  data.push_back(static_cast<std::uint8_t>(value >> 8U));
  data.push_back(static_cast<std::uint8_t>(value >> 16U));
  data.push_back(static_cast<std::uint8_t>(value >> 24U));
}

void appendFloat(std::vector<std::uint8_t> &data, float value) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  appendU32(data, bits);
}

}  // namespace

TEST(Protocol, NormalCommandRoundTripThroughStreamParser) {
  protocol::NormalCommand command;
  command.enable = true;
  command.efforts_nm = {1.0F, -1.0F, 0.25F, -0.25F, 2.0F, -2.0F};
  const auto frame = protocol::encodeFrame(
      protocol::kNormalCommandType, 42U,
      protocol::encodeNormalCommand(command));

  protocol::StreamParser parser;
  std::optional<protocol::Frame> decoded;
  for (const auto byte : frame) {
    const auto candidate = parser.consume(byte);
    if (candidate.has_value()) {
      decoded = candidate;
    }
  }

  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->type, protocol::kNormalCommandType);
  EXPECT_EQ(decoded->sequence, 42U);
  EXPECT_EQ(decoded->payload.size(), protocol::kNormalCommandPayloadSize);
  EXPECT_EQ(parser.stats().frames_ok, 1U);
}

TEST(Protocol, RejectsCorruptedFrameCrc) {
  protocol::NormalCommand command;
  auto frame = protocol::encodeFrame(
      protocol::kNormalCommandType, 7U,
      protocol::encodeNormalCommand(command));
  frame[6] ^= 0x01U;

  protocol::StreamParser parser;
  for (const auto byte : frame) {
    EXPECT_FALSE(parser.consume(byte).has_value());
  }
  EXPECT_EQ(parser.stats().frames_ok, 0U);
  EXPECT_EQ(parser.stats().crc_errors, 1U);
}

TEST(Protocol, DecodesIdentificationTelemetryV1) {
  std::vector<std::uint8_t> payload;
  payload.push_back(2U);
  payload.push_back(2U);
  payload.push_back(2U);
  payload.push_back(1U);
  payload.push_back(1U);
  payload.push_back(0U);
  payload.push_back(1U);
  payload.push_back(2U);
  appendU32(payload, 123U);
  appendU32(payload, 456U);
  appendU32(payload, 99U);
  appendFloat(payload, 1.5F);
  appendFloat(payload, 1.25F);
  appendU32(payload, 0x00000123U);
  appendU32(payload, 0x00000456U);
  appendFloat(payload, 1.2F);
  appendFloat(payload, 0.3F);
  appendFloat(payload, -0.4F);
  appendU32(payload, 5U);
  appendU32(payload, 6U);
  ASSERT_EQ(payload.size(), protocol::kIdentificationStatePayloadSize);

  protocol::IdentificationTelemetry telemetry;
  ASSERT_TRUE(protocol::decodeIdentificationTelemetry(payload, telemetry));
  EXPECT_EQ(telemetry.protocol_version, 2U);
  EXPECT_EQ(telemetry.actuator_index, 2U);
  EXPECT_EQ(telemetry.sample_seq, 123U);
  EXPECT_EQ(telemetry.trial_id, 99U);
  EXPECT_FLOAT_EQ(telemetry.current_applied_a, 1.25F);
  EXPECT_EQ(telemetry.driver_feedback_raw, 0x00000456U);
  EXPECT_FLOAT_EQ(telemetry.velocity_rad_s, -0.4F);
}

TEST(Protocol, DecodesCurrentInterleavedNormalStateLayout) {
  std::vector<std::uint8_t> payload;
  appendU32(payload, 1000U);
  for (int i = 0; i < 9; ++i) {
    appendFloat(payload, static_cast<float>(i));
  }
  for (int i = 0; i < 6; ++i) {
    appendFloat(payload, static_cast<float>(i) + 0.1F);
    appendFloat(payload, static_cast<float>(i) + 0.2F);
    appendFloat(payload, static_cast<float>(i) + 0.3F);
  }
  payload.push_back(0x3FU);
  payload.push_back(1U);
  payload.push_back(0U);
  payload.push_back(1U);
  appendU32(payload, 11U);
  appendU32(payload, 12U);
  appendU32(payload, 13U);
  ASSERT_EQ(payload.size(), protocol::kNormalStatePayloadSize);

  protocol::NormalState state;
  ASSERT_TRUE(protocol::decodeNormalState(payload, state));
  EXPECT_EQ(state.stm_tick_ms, 1000U);
  EXPECT_FLOAT_EQ(state.position_rad[4], 4.1F);
  EXPECT_FLOAT_EQ(state.velocity_rad_s[4], 4.2F);
  EXPECT_FLOAT_EQ(state.feedback_torque_nm[4], 4.3F);
  EXPECT_EQ(state.online_mask, 0x3FU);
  EXPECT_EQ(state.can_error_count, 13U);
}
