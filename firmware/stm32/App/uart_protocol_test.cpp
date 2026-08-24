#include "uart_protocol_test.h"

#include <cstring>

#include "Car.h"
#include "usart.h"

static const uint8_t kFrameHead0 = 0xA5;
static const uint8_t kFrameHead1 = 0x5A;
static const uint8_t kFrameTypeCommand = 0x01;
static const uint8_t kFrameTypeIdentificationCommand = 0x02;
static const uint8_t kFrameTypeState = 0x81;
static const uint8_t kFrameTypeIdentificationState = 0x82;
static const uint8_t kIdentificationProtocolVersion = 2u;
static const uint8_t kSafetyStateDisabled = 0;
static const uint8_t kSafetyStateEnabled = 1;
static const uint8_t kSafetyStateTimeout = 2;
static const uint8_t kSafetyStateEstop = 3;
static const uint8_t kSafetyStateFault = 4;
static const uint8_t kJointCount = 6;
static const uint8_t kCommandPayloadLen = 2u + kJointCount * 4u;
static const uint8_t kIdentificationCommandPayloadLen = 24u;
static const uint8_t kIdentificationStatePayloadLen = 56u;
static const uint8_t kMaxPayloadLen = 160u;
static const uint32_t kStatusPeriodMs = 5u;
static const uint32_t kIdentificationStatusPeriodMs = 2u;
static const uint32_t kCommandTimeoutMs = 100u;
static const uint32_t kIdentificationMaxDelayMs = 5000u;
static const uint32_t kIdentificationMaxDurationMs = 2000u;
static const float kHipKneeEffortLimit = 12.0f;
static const float kWheelEffortLimit = 6.0f;
static const float kHipKneeSlewPerCycle = 0.10f;
static const float kWheelSlewPerCycle = 0.06f;
// C620 CAN current command full scale: +/-16384 raw counts = +/-20 A.
// Do not add an identification-specific derating here: the requested range
// must cover the driver command range for current-to-torque identification.
static const float kC620CommandCurrentLimitA = 20.0f;
static const float kIdentificationHoldCurrentSlewPerCycle = 0.05f;
static const float kDegreesToRadians = 3.14159265358979323846f / 180.0f;
static const float kGravityMps2 = 9.80665f;
static const uint8_t kIdentificationFlagC620ThresholdCurrent = 1u << 0;

enum IdentificationExcitation {
    kIdentificationExcitationHold = 0,
    kIdentificationExcitationStep = 1,
};

enum IdentificationStepState {
    kIdentificationStepIdle = 0,
    kIdentificationStepDelay = 1,
    kIdentificationStepActive = 2,
    kIdentificationStepComplete = 3,
};

enum IdentificationActuatorType {
    kIdentificationActuatorGIM6010 = 1,
    kIdentificationActuatorC620 = 2,
};

enum RxState {
    kHead0,
    kHead1,
    kType,
    kLen,
    kSeqLo,
    kSeqHi,
    kPayload,
    kCrcLo,
    kCrcHi,
};

extern Class_Motor_DJI_C620 motor_3508_L;
extern Class_Motor_DJI_C620 motor_3508_R;
extern Class_Motor_DJI_GIM6010 motor_GIM6010_L_hip;
extern Class_Motor_DJI_GIM6010 motor_GIM6010_L_knee;
extern Class_Motor_DJI_GIM6010 motor_GIM6010_R_hip;
extern Class_Motor_DJI_GIM6010 motor_GIM6010_R_knee;

extern "C" {
volatile UartProtocolTestStats uart2_protocol_test_stats = {};
}

namespace {

struct StatePayload {
    uint32_t stm_tick_ms;
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float joint_position[kJointCount];
    float joint_velocity[kJointCount];
    float joint_effort[kJointCount];
    uint8_t online_mask;
    uint8_t safety_state;
    uint8_t last_command_timeout;
    uint8_t reserved0;
    uint32_t comm_rx_error_count;
    uint32_t comm_crc_error_count;
    uint32_t can_error_count;
};

uint8_t g_rx_byte = 0u;
RxState g_state = kHead0;
uint8_t g_frame_type = 0u;
uint8_t g_payload_len = 0u;
uint8_t g_payload_index = 0u;
uint16_t g_seq = 0u;
uint16_t g_rx_crc = 0u;
uint16_t g_tx_seq = 0u;
uint32_t g_last_status_tick_ms = 0u;
uint32_t g_last_identification_status_tick_ms = 0u;
uint8_t g_payload[kMaxPayloadLen] = {0};
uint8_t g_status_frame[2 + 4 + sizeof(StatePayload) + 2] = {0};
StatePayload g_state_payload = {};
volatile uint8_t g_tx_in_flight = 0u;
uint16_t g_tx_frame_len = 0u;
uint16_t g_tx_in_flight_seq = 0u;
float g_target_efforts[kJointCount] = {0.0f};
float g_applied_efforts[kJointCount] = {0.0f};
uint8_t g_command_enable = 0u;
uint8_t g_command_estop = 0u;
uint8_t g_last_command_timeout = 1u;
uint8_t g_safety_state = kSafetyStateDisabled;
uint32_t g_last_valid_command_tick_ms = 0u;
uint8_t g_identification_mode = 0u;
uint8_t g_identification_selected_actuator = 0u;
uint8_t g_identification_excitation = kIdentificationExcitationHold;
uint8_t g_identification_flags = 0u;
uint8_t g_identification_step_state = kIdentificationStepIdle;
uint32_t g_identification_trial_id = 0u;
uint32_t g_identification_start_tick_ms = 0u;
uint32_t g_identification_delay_ms = 0u;
uint32_t g_identification_duration_ms = 0u;
uint32_t g_identification_sample_seq = 0u;
float g_identification_target_current_a = 0.0f;
float g_identification_current_requested_a = 0.0f;
float g_identification_current_applied_a = 0.0f;

uint16_t Crc16CcittUpdate(uint16_t crc, uint8_t data)
{
    crc ^= static_cast<uint16_t>(data) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if ((crc & 0x8000u) != 0u) {
            crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
        } else {
            crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t Crc16Ccitt(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < size; ++i) {
        crc = Crc16CcittUpdate(crc, data[i]);
    }
    return crc;
}

uint16_t CalculateFrameCrc()
{
    uint16_t crc = 0xFFFFu;
    crc = Crc16CcittUpdate(crc, g_frame_type);
    crc = Crc16CcittUpdate(crc, g_payload_len);
    crc = Crc16CcittUpdate(crc, static_cast<uint8_t>(g_seq & 0xFFu));
    crc = Crc16CcittUpdate(crc, static_cast<uint8_t>((g_seq >> 8) & 0xFFu));
    for (uint8_t i = 0; i < g_payload_len; ++i) {
        crc = Crc16CcittUpdate(crc, g_payload[i]);
    }
    return crc;
}

void ResetParser()
{
    g_state = kHead0;
    g_frame_type = 0u;
    g_payload_len = 0u;
    g_payload_index = 0u;
    g_seq = 0u;
    g_rx_crc = 0u;
}

void WriteU16Le(uint8_t *dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFFu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void WriteU32Le(uint8_t *dst, uint32_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFFu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

uint32_t ReadU32Le(const uint8_t *src)
{
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

void WriteF32Le(uint8_t *dst, float value)
{
    uint32_t raw = 0u;
    std::memcpy(&raw, &value, sizeof(raw));
    WriteU32Le(dst, raw);
}

float ReadF32Le(const uint8_t *src)
{
    const uint32_t raw = ReadU32Le(src);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

bool IsFiniteFloat(float value)
{
    uint32_t raw = 0u;
    std::memcpy(&raw, &value, sizeof(raw));
    return (raw & 0x7F800000u) != 0x7F800000u;
}

float ConstrainSymmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

float ApplySlewLimit(float current, float target, float delta_limit)
{
    const float delta = target - current;
    if (delta > delta_limit) {
        return current + delta_limit;
    }
    if (delta < -delta_limit) {
        return current - delta_limit;
    }
    return target;
}

uint8_t BuildOnlineMask()
{
    uint8_t mask = 0u;
    if (motor_GIM6010_L_hip.Get_Status() == Motor_DJI_Status_ENABLE) {
        mask |= (1u << 0);
    }
    if (motor_GIM6010_L_knee.Get_Status() == Motor_DJI_Status_ENABLE) {
        mask |= (1u << 1);
    }
    if (motor_3508_L.Get_Status() == Motor_DJI_Status_ENABLE) {
        mask |= (1u << 2);
    }
    if (motor_GIM6010_R_hip.Get_Status() == Motor_DJI_Status_ENABLE) {
        mask |= (1u << 3);
    }
    if (motor_GIM6010_R_knee.Get_Status() == Motor_DJI_Status_ENABLE) {
        mask |= (1u << 4);
    }
    if (motor_3508_R.Get_Status() == Motor_DJI_Status_ENABLE) {
        mask |= (1u << 5);
    }
    return mask;
}

uint32_t BuildCanErrorCount(uint8_t online_mask)
{
    uint32_t offline_count = 0u;
    for (uint8_t i = 0; i < kJointCount; ++i) {
        if ((online_mask & (1u << i)) == 0u) {
            offline_count++;
        }
    }
    return offline_count;
}

bool IsC620Actuator(uint8_t actuator_index)
{
    return actuator_index == 2u || actuator_index == 5u;
}

uint8_t GetActuatorType(uint8_t actuator_index)
{
    return IsC620Actuator(actuator_index) ?
        kIdentificationActuatorC620 : kIdentificationActuatorGIM6010;
}

float GetIdentificationCurrentLimitA(uint8_t actuator_index)
{
    return IsC620Actuator(actuator_index) ?
        kC620CommandCurrentLimitA : 0.0f;
}

float GetActuatorPosition(uint8_t actuator_index)
{
    switch (actuator_index) {
    case 0u: return motor_GIM6010_L_hip.Get_Now_Angle();
    case 1u: return motor_GIM6010_L_knee.Get_Now_Angle();
    case 2u: return motor_3508_L.Get_Now_Angle();
    case 3u: return motor_GIM6010_R_hip.Get_Now_Angle();
    case 4u: return motor_GIM6010_R_knee.Get_Now_Angle();
    case 5u: return motor_3508_R.Get_Now_Angle();
    default: return 0.0f;
    }
}

float GetActuatorVelocity(uint8_t actuator_index)
{
    switch (actuator_index) {
    case 0u: return motor_GIM6010_L_hip.Get_Now_Omega();
    case 1u: return motor_GIM6010_L_knee.Get_Now_Omega();
    case 2u: return motor_3508_L.Get_Now_Omega();
    case 3u: return motor_GIM6010_R_hip.Get_Now_Omega();
    case 4u: return motor_GIM6010_R_knee.Get_Now_Omega();
    case 5u: return motor_3508_R.Get_Now_Omega();
    default: return 0.0f;
    }
}

float GetActuatorFeedbackCurrent(uint8_t actuator_index)
{
    switch (actuator_index) {
    case 2u: return motor_3508_L.Get_Now_Current();
    case 5u: return motor_3508_R.Get_Now_Current();
    default: return 0.0f;
    }
}

uint32_t GetActuatorCommandRaw(uint8_t actuator_index)
{
    switch (actuator_index) {
    case 0u: return motor_GIM6010_L_hip.Get_Command_Raw();
    case 1u: return motor_GIM6010_L_knee.Get_Command_Raw();
    case 2u: return motor_3508_L.Get_Command_Raw();
    case 3u: return motor_GIM6010_R_hip.Get_Command_Raw();
    case 4u: return motor_GIM6010_R_knee.Get_Command_Raw();
    case 5u: return motor_3508_R.Get_Command_Raw();
    default: return 0u;
    }
}

uint32_t GetActuatorFeedbackRaw(uint8_t actuator_index)
{
    switch (actuator_index) {
    case 0u: return motor_GIM6010_L_hip.Get_Feedback_Raw();
    case 1u: return motor_GIM6010_L_knee.Get_Feedback_Raw();
    case 2u: return motor_3508_L.Get_Feedback_Raw();
    case 3u: return motor_GIM6010_R_hip.Get_Feedback_Raw();
    case 4u: return motor_GIM6010_R_knee.Get_Feedback_Raw();
    case 5u: return motor_3508_R.Get_Feedback_Raw();
    default: return 0u;
    }
}

void ExitIdentificationMode()
{
    g_identification_mode = 0u;
    g_identification_step_state = kIdentificationStepIdle;
    g_identification_current_requested_a = 0.0f;
    g_identification_current_applied_a = 0.0f;
}

void DecodeNormalCommandPayload()
{
    if (g_payload_len != kCommandPayloadLen) {
        uart2_protocol_test_stats.length_errors++;
        return;
    }

    ExitIdentificationMode();
    g_command_enable = g_payload[0];
    g_command_estop = g_payload[1];
    for (uint8_t i = 0; i < kJointCount; ++i) {
        g_target_efforts[i] = ReadF32Le(g_payload + 2u + 4u * i);
    }
    g_last_valid_command_tick_ms = HAL_GetTick();
    g_last_command_timeout = 0u;
}

void DecodeIdentificationCommandPayload()
{
    if (g_payload_len != kIdentificationCommandPayloadLen ||
        g_payload[0] != kIdentificationProtocolVersion) {
        uart2_protocol_test_stats.length_errors++;
        return;
    }

    const uint8_t selected_actuator = g_payload[3];
    const uint8_t excitation = g_payload[4];
    const uint32_t trial_id = ReadU32Le(g_payload + 8u);
    const float target_current_a = ReadF32Le(g_payload + 12u);
    const uint32_t delay_ms = ReadU32Le(g_payload + 16u);
    const uint32_t duration_ms = ReadU32Le(g_payload + 20u);

    const bool invalid_step =
        excitation == kIdentificationExcitationStep && duration_ms == 0u;
    if (!IsC620Actuator(selected_actuator) ||
        excitation > kIdentificationExcitationStep ||
        !IsFiniteFloat(target_current_a) ||
        delay_ms > kIdentificationMaxDelayMs ||
        duration_ms > kIdentificationMaxDurationMs || invalid_step) {
        uart2_protocol_test_stats.length_errors++;
        return;
    }

    const uint32_t now = HAL_GetTick();
    const bool new_trial =
        g_identification_mode == 0u || trial_id != g_identification_trial_id;

    g_identification_mode = 1u;
    g_command_enable = g_payload[1];
    g_command_estop = g_payload[2];
    g_last_valid_command_tick_ms = now;
    g_last_command_timeout = 0u;

    if (new_trial) {
        g_identification_selected_actuator = selected_actuator;
        g_identification_excitation = excitation;
        // Direct-current identification forbids Threshold_Current so the
        // excitation is not contaminated by firmware dead-zone compensation.
        g_identification_flags = 0u;
        g_identification_trial_id = trial_id;
        g_identification_target_current_a = target_current_a;
        g_identification_delay_ms = delay_ms;
        g_identification_duration_ms = duration_ms;
        g_identification_start_tick_ms = now;
        g_identification_step_state =
            excitation == kIdentificationExcitationStep ?
            (delay_ms == 0u ? kIdentificationStepActive :
                             kIdentificationStepDelay) :
            kIdentificationStepActive;
        for (uint8_t i = 0u; i < kJointCount; ++i) {
            g_applied_efforts[i] = 0.0f;
        }
    }
}

void DecodeCommandPayload()
{
    if (g_frame_type == kFrameTypeCommand) {
        DecodeNormalCommandPayload();
    } else if (g_frame_type == kFrameTypeIdentificationCommand) {
        DecodeIdentificationCommandPayload();
    } else {
        uart2_protocol_test_stats.length_errors++;
    }
}

void MarkFrameOk()
{
    const uint32_t now = HAL_GetTick();
    if (uart2_protocol_test_stats.frames_ok != 0u) {
        const uint16_t expected_seq =
            static_cast<uint16_t>(uart2_protocol_test_stats.last_seq + 1u);
        if (g_seq != expected_seq) {
            uart2_protocol_test_stats.rx_seq_gaps++;
        }
    }

    if (uart2_protocol_test_stats.last_frame_tick_ms != 0u) {
        const uint32_t gap = now - uart2_protocol_test_stats.last_frame_tick_ms;
        if (gap < uart2_protocol_test_stats.min_frame_gap_ms) {
            uart2_protocol_test_stats.min_frame_gap_ms = gap;
        }
        if (gap > uart2_protocol_test_stats.max_frame_gap_ms) {
            uart2_protocol_test_stats.max_frame_gap_ms = gap;
        }
    }

    uart2_protocol_test_stats.frames_ok++;
    uart2_protocol_test_stats.last_seq = g_seq;
    uart2_protocol_test_stats.last_type = g_frame_type;
    uart2_protocol_test_stats.last_len = g_payload_len;
    uart2_protocol_test_stats.last_frame_tick_ms = now;

    DecodeCommandPayload();
}

void ConsumeByte(uint8_t byte)
{
    uart2_protocol_test_stats.rx_bytes++;

    switch (g_state) {
    case kHead0:
        if (byte == kFrameHead0) {
            g_state = kHead1;
        } else {
            uart2_protocol_test_stats.sync_losses++;
        }
        break;

    case kHead1:
        if (byte == kFrameHead1) {
            g_state = kType;
        } else {
            uart2_protocol_test_stats.sync_losses++;
            g_state = (byte == kFrameHead0) ? kHead1 : kHead0;
        }
        break;

    case kType:
        g_frame_type = byte;
        g_state = kLen;
        break;

    case kLen:
        g_payload_len = byte;
        if (g_payload_len > kMaxPayloadLen) {
            uart2_protocol_test_stats.length_errors++;
            ResetParser();
        } else {
            g_payload_index = 0u;
            g_state = kSeqLo;
        }
        break;

    case kSeqLo:
        g_seq = byte;
        g_state = kSeqHi;
        break;

    case kSeqHi:
        g_seq |= static_cast<uint16_t>(byte) << 8;
        g_state = (g_payload_len == 0u) ? kCrcLo : kPayload;
        break;

    case kPayload:
        g_payload[g_payload_index++] = byte;
        if (g_payload_index >= g_payload_len) {
            g_state = kCrcLo;
        }
        break;

    case kCrcLo:
        g_rx_crc = byte;
        g_state = kCrcHi;
        break;

    case kCrcHi:
        g_rx_crc |= static_cast<uint16_t>(byte) << 8;
        if (g_rx_crc == CalculateFrameCrc()) {
            MarkFrameOk();
        } else {
            uart2_protocol_test_stats.crc_errors++;
        }
        ResetParser();
        break;
    }
}

void RestartReceive()
{
    if (HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1) != HAL_OK) {
        uart2_protocol_test_stats.uart_errors++;
    }
}

void FillStatePayload(StatePayload *payload, uint32_t now)
{
    if (payload == 0) {
        return;
    }

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
    uint8_t safety_state = 0u;
    uint8_t last_command_timeout = 0u;

    __disable_irq();
    roll = imu_roll;
    pitch = imu_pitch;
    yaw = imu_yaw;
    gyro_x = gyx;
    gyro_y = gyy;
    gyro_z = gyz;
    acc_x = accx;
    acc_y = accy;
    acc_z = accz;
    safety_state = g_safety_state;
    last_command_timeout = g_last_command_timeout;
    __enable_irq();

    const uint8_t online_mask = BuildOnlineMask();
    const uint32_t can_error_count = BuildCanErrorCount(online_mask);
    const uint32_t comm_rx_error_count =
        uart2_protocol_test_stats.length_errors +
        uart2_protocol_test_stats.sync_losses +
        uart2_protocol_test_stats.rx_seq_gaps +
        uart2_protocol_test_stats.uart_errors;

    payload->stm_tick_ms = now;
    payload->roll = roll * kDegreesToRadians;
    payload->pitch = pitch * kDegreesToRadians;
    payload->yaw = yaw * kDegreesToRadians;
    payload->gyro_x = gyro_x * kDegreesToRadians;
    payload->gyro_y = gyro_y * kDegreesToRadians;
    payload->gyro_z = gyro_z * kDegreesToRadians;
    payload->acc_x = acc_x * kGravityMps2;
    payload->acc_y = acc_y * kGravityMps2;
    payload->acc_z = acc_z * kGravityMps2;

    payload->joint_position[0] = motor_GIM6010_L_hip.Get_Now_Angle();
    payload->joint_position[1] = motor_GIM6010_L_knee.Get_Now_Angle();
    payload->joint_position[2] = motor_3508_L.Get_Now_Angle();
    payload->joint_position[3] = motor_GIM6010_R_hip.Get_Now_Angle();
    payload->joint_position[4] = motor_GIM6010_R_knee.Get_Now_Angle();
    payload->joint_position[5] = motor_3508_R.Get_Now_Angle();

    payload->joint_velocity[0] = motor_GIM6010_L_hip.Get_Now_Omega();
    payload->joint_velocity[1] = motor_GIM6010_L_knee.Get_Now_Omega();
    payload->joint_velocity[2] = motor_3508_L.Get_Now_Omega();
    payload->joint_velocity[3] = motor_GIM6010_R_hip.Get_Now_Omega();
    payload->joint_velocity[4] = motor_GIM6010_R_knee.Get_Now_Omega();
    payload->joint_velocity[5] = motor_3508_R.Get_Now_Omega();

    payload->joint_effort[0] = motor_GIM6010_L_hip.Get_Now_Torque();
    payload->joint_effort[1] = motor_GIM6010_L_knee.Get_Now_Torque();
    payload->joint_effort[2] = motor_3508_L.Get_Now_Torque();
    payload->joint_effort[3] = motor_GIM6010_R_hip.Get_Now_Torque();
    payload->joint_effort[4] = motor_GIM6010_R_knee.Get_Now_Torque();
    payload->joint_effort[5] = motor_3508_R.Get_Now_Torque();

    payload->online_mask = online_mask;
    payload->safety_state = safety_state;
    payload->last_command_timeout = last_command_timeout;
    payload->reserved0 = knee_limit_flag;
    payload->comm_rx_error_count = comm_rx_error_count;
    payload->comm_crc_error_count = uart2_protocol_test_stats.crc_errors;
    payload->can_error_count = can_error_count;
}

void EncodeStatePayload(const StatePayload &payload, uint8_t *dst)
{
    uint16_t offset = 0u;
    WriteU32Le(dst + offset, payload.stm_tick_ms);
    offset += 4u;
    WriteF32Le(dst + offset, payload.roll);
    offset += 4u;
    WriteF32Le(dst + offset, payload.pitch);
    offset += 4u;
    WriteF32Le(dst + offset, payload.yaw);
    offset += 4u;
    WriteF32Le(dst + offset, payload.gyro_x);
    offset += 4u;
    WriteF32Le(dst + offset, payload.gyro_y);
    offset += 4u;
    WriteF32Le(dst + offset, payload.gyro_z);
    offset += 4u;
    WriteF32Le(dst + offset, payload.acc_x);
    offset += 4u;
    WriteF32Le(dst + offset, payload.acc_y);
    offset += 4u;
    WriteF32Le(dst + offset, payload.acc_z);
    offset += 4u;

    for (uint8_t i = 0u; i < kJointCount; ++i) {
        WriteF32Le(dst + offset, payload.joint_position[i]);
        offset += 4u;
        WriteF32Le(dst + offset, payload.joint_velocity[i]);
        offset += 4u;
        WriteF32Le(dst + offset, payload.joint_effort[i]);
        offset += 4u;
    }

    dst[offset++] = payload.online_mask;
    dst[offset++] = payload.safety_state;
    dst[offset++] = payload.last_command_timeout;
    dst[offset++] = payload.reserved0;
    WriteU32Le(dst + offset, payload.comm_rx_error_count);
    offset += 4u;
    WriteU32Le(dst + offset, payload.comm_crc_error_count);
    offset += 4u;
    WriteU32Le(dst + offset, payload.can_error_count);
}

void TrySendStatusFrame()
{
    if (g_identification_mode != 0u) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if ((now - g_last_status_tick_ms) < kStatusPeriodMs) {
        return;
    }
    g_last_status_tick_ms = now;

    if (g_tx_in_flight != 0u) {
        uart2_protocol_test_stats.tx_skip_in_flight++;
        return;
    }

    FillStatePayload(&g_state_payload, now);

    const uint8_t payload_len = static_cast<uint8_t>(sizeof(StatePayload));
    g_status_frame[0] = kFrameHead0;
    g_status_frame[1] = kFrameHead1;
    g_status_frame[2] = kFrameTypeState;
    g_status_frame[3] = payload_len;
    WriteU16Le(g_status_frame + 4, g_tx_seq);
    EncodeStatePayload(g_state_payload, g_status_frame + 6);

    const uint16_t crc =
        Crc16Ccitt(g_status_frame + 2, static_cast<uint16_t>(4u + payload_len));
    const uint16_t crc_index = static_cast<uint16_t>(6u + payload_len);
    WriteU16Le(g_status_frame + crc_index, crc);

    const uint16_t frame_len = static_cast<uint16_t>(crc_index + 2u);
    g_tx_in_flight = 1u;
    g_tx_frame_len = frame_len;
    g_tx_in_flight_seq = g_tx_seq;
    const HAL_StatusTypeDef tx_status =
        HAL_UART_Transmit_DMA(&huart2, g_status_frame, frame_len);

    if (tx_status == HAL_OK) {
    } else {
        g_tx_in_flight = 0u;
        uart2_protocol_test_stats.tx_errors++;
        if (tx_status == HAL_BUSY) {
            uart2_protocol_test_stats.tx_busy_errors++;
        } else if (tx_status == HAL_TIMEOUT) {
            uart2_protocol_test_stats.tx_timeout_errors++;
        } else if (tx_status == HAL_ERROR) {
            uart2_protocol_test_stats.tx_hal_error_errors++;
        }
    }
}

void TrySendIdentificationFrame()
{
    if (g_identification_mode == 0u) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if ((now - g_last_identification_status_tick_ms) <
        kIdentificationStatusPeriodMs) {
        return;
    }
    g_last_identification_status_tick_ms = now;
    g_identification_sample_seq++;

    if (g_tx_in_flight != 0u) {
        uart2_protocol_test_stats.tx_skip_in_flight++;
        return;
    }

    uint8_t selected = 0u;
    uint8_t excitation = 0u;
    uint8_t safety_state = 0u;
    uint8_t flags = 0u;
    uint8_t step_state = 0u;
    uint8_t online_mask = 0u;
    uint32_t sample_seq = 0u;
    uint32_t trial_id = 0u;
    uint32_t last_valid_command_tick_ms = 0u;
    uint32_t command_raw = 0u;
    uint32_t feedback_raw = 0u;
    uint32_t comm_rx_error_count = 0u;
    float current_requested_a = 0.0f;
    float current_applied_a = 0.0f;
    float feedback_current_a = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;

    // Keep one telemetry sample internally coherent if UART/CAN IRQs arrive
    // while the DMA frame is being assembled.
    __disable_irq();
    selected = g_identification_selected_actuator;
    excitation = g_identification_excitation;
    safety_state = g_safety_state;
    flags = g_identification_flags;
    step_state = g_identification_step_state;
    online_mask = BuildOnlineMask();
    sample_seq = g_identification_sample_seq;
    trial_id = g_identification_trial_id;
    last_valid_command_tick_ms = g_last_valid_command_tick_ms;
    current_requested_a = g_identification_current_requested_a;
    current_applied_a = g_identification_current_applied_a;
    command_raw = GetActuatorCommandRaw(selected);
    feedback_raw = GetActuatorFeedbackRaw(selected);
    feedback_current_a = GetActuatorFeedbackCurrent(selected);
    position = GetActuatorPosition(selected);
    velocity = GetActuatorVelocity(selected);
    comm_rx_error_count =
        uart2_protocol_test_stats.length_errors +
        uart2_protocol_test_stats.sync_losses +
        uart2_protocol_test_stats.rx_seq_gaps +
        uart2_protocol_test_stats.uart_errors;
    __enable_irq();

    const uint8_t selected_online =
        (online_mask & (1u << selected)) != 0u ? 1u : 0u;
    const uint32_t command_age_ms =
        last_valid_command_tick_ms == 0u ? 0xFFFFFFFFu :
        now - last_valid_command_tick_ms;

    uint16_t offset = 0u;
    g_status_frame[6u + offset++] = kIdentificationProtocolVersion;
    g_status_frame[6u + offset++] = selected;
    g_status_frame[6u + offset++] = GetActuatorType(selected);
    g_status_frame[6u + offset++] = excitation;
    g_status_frame[6u + offset++] = safety_state;
    g_status_frame[6u + offset++] = flags;
    g_status_frame[6u + offset++] = selected_online;
    g_status_frame[6u + offset++] = step_state;
    WriteU32Le(g_status_frame + 6u + offset, sample_seq);
    offset += 4u;
    WriteU32Le(g_status_frame + 6u + offset, now);
    offset += 4u;
    WriteU32Le(g_status_frame + 6u + offset, trial_id);
    offset += 4u;
    WriteF32Le(g_status_frame + 6u + offset, current_requested_a);
    offset += 4u;
    WriteF32Le(g_status_frame + 6u + offset, current_applied_a);
    offset += 4u;
    WriteU32Le(g_status_frame + 6u + offset, command_raw);
    offset += 4u;
    WriteU32Le(g_status_frame + 6u + offset, feedback_raw);
    offset += 4u;
    WriteF32Le(g_status_frame + 6u + offset, feedback_current_a);
    offset += 4u;
    WriteF32Le(g_status_frame + 6u + offset, position);
    offset += 4u;
    WriteF32Le(g_status_frame + 6u + offset, velocity);
    offset += 4u;
    WriteU32Le(g_status_frame + 6u + offset, command_age_ms);
    offset += 4u;
    WriteU32Le(g_status_frame + 6u + offset, comm_rx_error_count);

    g_status_frame[0] = kFrameHead0;
    g_status_frame[1] = kFrameHead1;
    g_status_frame[2] = kFrameTypeIdentificationState;
    g_status_frame[3] = kIdentificationStatePayloadLen;
    WriteU16Le(g_status_frame + 4, g_tx_seq);

    const uint16_t crc = Crc16Ccitt(
        g_status_frame + 2,
        static_cast<uint16_t>(4u + kIdentificationStatePayloadLen));
    const uint16_t crc_index =
        static_cast<uint16_t>(6u + kIdentificationStatePayloadLen);
    WriteU16Le(g_status_frame + crc_index, crc);

    const uint16_t frame_len = static_cast<uint16_t>(crc_index + 2u);
    g_tx_in_flight = 1u;
    g_tx_frame_len = frame_len;
    g_tx_in_flight_seq = g_tx_seq;
    const HAL_StatusTypeDef tx_status =
        HAL_UART_Transmit_DMA(&huart2, g_status_frame, frame_len);

    if (tx_status != HAL_OK) {
        g_tx_in_flight = 0u;
        uart2_protocol_test_stats.tx_errors++;
        if (tx_status == HAL_BUSY) {
            uart2_protocol_test_stats.tx_busy_errors++;
        } else if (tx_status == HAL_TIMEOUT) {
            uart2_protocol_test_stats.tx_timeout_errors++;
        } else if (tx_status == HAL_ERROR) {
            uart2_protocol_test_stats.tx_hal_error_errors++;
        }
    }
}

}  // namespace

extern "C" void UartProtocolTest_Init(void)
{
    ResetParser();
    UartProtocolTest_ResetStats();
    g_tx_seq = 0u;
    g_tx_in_flight = 0u;
    g_tx_frame_len = 0u;
    g_tx_in_flight_seq = 0u;
    g_last_status_tick_ms = HAL_GetTick();
    g_last_identification_status_tick_ms = g_last_status_tick_ms;
    g_last_valid_command_tick_ms = 0u;
    g_command_enable = 0u;
    g_command_estop = 0u;
    g_last_command_timeout = 1u;
    g_safety_state = kSafetyStateDisabled;
    g_identification_mode = 0u;
    g_identification_selected_actuator = 0u;
    g_identification_excitation = kIdentificationExcitationHold;
    g_identification_flags = 0u;
    g_identification_step_state = kIdentificationStepIdle;
    g_identification_trial_id = 0u;
    g_identification_start_tick_ms = 0u;
    g_identification_delay_ms = 0u;
    g_identification_duration_ms = 0u;
    g_identification_sample_seq = 0u;
    g_identification_target_current_a = 0.0f;
    g_identification_current_requested_a = 0.0f;
    g_identification_current_applied_a = 0.0f;
    for (uint8_t i = 0u; i < kJointCount; ++i) {
        g_target_efforts[i] = 0.0f;
        g_applied_efforts[i] = 0.0f;
    }
    RestartReceive();
}

extern "C" void UartProtocolTest_Process(void)
{
    TrySendStatusFrame();
}

extern "C" void UartProtocolTest_GetStats(UartProtocolTestStats *out)
{
    if (out == 0) {
        return;
    }

    __disable_irq();
    out->rx_bytes = uart2_protocol_test_stats.rx_bytes;
    out->frames_ok = uart2_protocol_test_stats.frames_ok;
    out->crc_errors = uart2_protocol_test_stats.crc_errors;
    out->length_errors = uart2_protocol_test_stats.length_errors;
    out->sync_losses = uart2_protocol_test_stats.sync_losses;
    out->rx_seq_gaps = uart2_protocol_test_stats.rx_seq_gaps;
    out->uart_errors = uart2_protocol_test_stats.uart_errors;
    out->tx_frames = uart2_protocol_test_stats.tx_frames;
    out->tx_bytes = uart2_protocol_test_stats.tx_bytes;
    out->tx_errors = uart2_protocol_test_stats.tx_errors;
    out->tx_busy_errors = uart2_protocol_test_stats.tx_busy_errors;
    out->tx_timeout_errors = uart2_protocol_test_stats.tx_timeout_errors;
    out->tx_hal_error_errors = uart2_protocol_test_stats.tx_hal_error_errors;
    out->last_seq = uart2_protocol_test_stats.last_seq;
    out->last_tx_seq = uart2_protocol_test_stats.last_tx_seq;
    out->last_type = uart2_protocol_test_stats.last_type;
    out->last_len = uart2_protocol_test_stats.last_len;
    out->last_frame_tick_ms = uart2_protocol_test_stats.last_frame_tick_ms;
    out->min_frame_gap_ms = uart2_protocol_test_stats.min_frame_gap_ms;
    out->max_frame_gap_ms = uart2_protocol_test_stats.max_frame_gap_ms;
    out->tx_skip_in_flight = uart2_protocol_test_stats.tx_skip_in_flight;
    __enable_irq();
}

extern "C" void UartProtocolTest_ResetStats(void)
{
    __disable_irq();
    uart2_protocol_test_stats.rx_bytes = 0u;
    uart2_protocol_test_stats.frames_ok = 0u;
    uart2_protocol_test_stats.crc_errors = 0u;
    uart2_protocol_test_stats.length_errors = 0u;
    uart2_protocol_test_stats.sync_losses = 0u;
    uart2_protocol_test_stats.rx_seq_gaps = 0u;
    uart2_protocol_test_stats.uart_errors = 0u;
    uart2_protocol_test_stats.tx_frames = 0u;
    uart2_protocol_test_stats.tx_bytes = 0u;
    uart2_protocol_test_stats.tx_errors = 0u;
    uart2_protocol_test_stats.tx_busy_errors = 0u;
    uart2_protocol_test_stats.tx_timeout_errors = 0u;
    uart2_protocol_test_stats.tx_hal_error_errors = 0u;
    uart2_protocol_test_stats.last_seq = 0u;
    uart2_protocol_test_stats.last_tx_seq = 0u;
    uart2_protocol_test_stats.last_type = 0u;
    uart2_protocol_test_stats.last_len = 0u;
    uart2_protocol_test_stats.last_frame_tick_ms = 0u;
    uart2_protocol_test_stats.min_frame_gap_ms = 0xFFFFFFFFu;
    uart2_protocol_test_stats.max_frame_gap_ms = 0u;
    uart2_protocol_test_stats.tx_skip_in_flight = 0u;
    __enable_irq();
}

extern "C" void UartProtocolTest_FillActuatorCommand(
    float *out_efforts,
    uint32_t effort_count)
{
    float target_efforts[kJointCount] = {0.0f};
    uint32_t last_valid_command_tick_ms = 0u;
    uint8_t command_enable = 0u;
    uint8_t command_estop = 0u;
    uint8_t identification_mode = 0u;
    uint8_t identification_selected_actuator = 0u;
    uint8_t identification_excitation = kIdentificationExcitationHold;
    uint32_t identification_start_tick_ms = 0u;
    uint32_t identification_delay_ms = 0u;
    uint32_t identification_duration_ms = 0u;
    float identification_target_current_a = 0.0f;

    __disable_irq();
    for (uint8_t i = 0u; i < kJointCount; ++i) {
        target_efforts[i] = g_target_efforts[i];
    }
    last_valid_command_tick_ms = g_last_valid_command_tick_ms;
    command_enable = g_command_enable;
    command_estop = g_command_estop;
    identification_mode = g_identification_mode;
    identification_selected_actuator = g_identification_selected_actuator;
    identification_excitation = g_identification_excitation;
    identification_start_tick_ms = g_identification_start_tick_ms;
    identification_delay_ms = g_identification_delay_ms;
    identification_duration_ms = g_identification_duration_ms;
    identification_target_current_a = g_identification_target_current_a;
    __enable_irq();

    const uint32_t now = HAL_GetTick();
    const bool timed_out =
        (last_valid_command_tick_ms == 0u) ||
        ((now - last_valid_command_tick_ms) > kCommandTimeoutMs);

    if (timed_out) {
        g_last_command_timeout = 1u;
    }

    const uint8_t online_mask = BuildOnlineMask();
    if (command_estop != 0u) {
        g_safety_state = kSafetyStateEstop;
    } else if (timed_out) {
        g_safety_state = kSafetyStateTimeout;
    } else if (command_enable == 0u) {
        g_safety_state = kSafetyStateDisabled;
    } else if (identification_mode != 0u &&
               (online_mask & (1u << identification_selected_actuator)) == 0u) {
        g_safety_state = kSafetyStateFault;
    } else {
        g_safety_state = kSafetyStateEnabled;
    }

    if (identification_mode != 0u) {
        float requested_current_a = 0.0f;
        uint8_t step_state = kIdentificationStepActive;

        if (identification_excitation == kIdentificationExcitationHold) {
            requested_current_a = identification_target_current_a;
        } else {
            const uint32_t elapsed = now - identification_start_tick_ms;
            if (elapsed < identification_delay_ms) {
                step_state = kIdentificationStepDelay;
            } else if ((elapsed - identification_delay_ms) <
                       identification_duration_ms) {
                step_state = kIdentificationStepActive;
                requested_current_a = identification_target_current_a;
            } else {
                step_state = kIdentificationStepComplete;
            }
        }

        for (uint8_t i = 0u; i < kJointCount; ++i) {
            g_applied_efforts[i] = 0.0f;
            if (out_efforts != 0 && i < effort_count) {
                out_efforts[i] = 0.0f;
            }
        }

        float applied_current_a = 0.0f;
        if (g_safety_state == kSafetyStateEnabled) {
            const float limited_current_a = ConstrainSymmetric(
                requested_current_a,
                GetIdentificationCurrentLimitA(
                    identification_selected_actuator));
            if (identification_excitation == kIdentificationExcitationStep) {
                applied_current_a = limited_current_a;
            } else {
                applied_current_a = ApplySlewLimit(
                    g_identification_current_applied_a,
                    limited_current_a,
                    kIdentificationHoldCurrentSlewPerCycle);
            }
            g_applied_efforts[identification_selected_actuator] = applied_current_a;
            if (out_efforts != 0 &&
                identification_selected_actuator < effort_count) {
                out_efforts[identification_selected_actuator] = applied_current_a;
            }
        }

        g_identification_step_state = step_state;
        g_identification_current_requested_a = requested_current_a;
        g_identification_current_applied_a = applied_current_a;
        return;
    }

    for (uint8_t i = 0u; i < kJointCount; ++i) {
        float target = 0.0f;
        if (g_safety_state == kSafetyStateEnabled) {
            target = target_efforts[i];
        }

        const float effort_limit = (i == 2u || i == 5u) ?
            kWheelEffortLimit : kHipKneeEffortLimit;
        const float slew_limit = (i == 2u || i == 5u) ?
            kWheelSlewPerCycle : kHipKneeSlewPerCycle;
        target = ConstrainSymmetric(target, effort_limit);
        g_applied_efforts[i] =
            ApplySlewLimit(g_applied_efforts[i], target, slew_limit);

        if (out_efforts != 0 && i < effort_count) {
            out_efforts[i] = g_applied_efforts[i];
        }
    }
}

extern "C" void UartProtocolTest_GetIdentificationControl(
    UartProtocolTestIdentificationControl *out)
{
    if (out == 0) {
        return;
    }

    __disable_irq();
    out->active = g_identification_mode;
    out->selected_actuator = g_identification_selected_actuator;
    out->c620_threshold_current_enabled =
        (g_identification_flags &
         kIdentificationFlagC620ThresholdCurrent) != 0u ? 1u : 0u;
    out->c620_direct_current_mode = g_identification_mode != 0u ? 1u : 0u;
    __enable_irq();
}

extern "C" void UartProtocolTest_ProcessIdentificationTelemetry(void)
{
    TrySendIdentificationFrame();
}

extern "C" uint8_t UartProtocolTest_GetSafetyState(void)
{
    return g_safety_state;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        ConsumeByte(g_rx_byte);
        RestartReceive();
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        g_tx_in_flight = 0u;
        uart2_protocol_test_stats.tx_frames++;
        uart2_protocol_test_stats.tx_bytes += g_tx_frame_len;
        uart2_protocol_test_stats.last_tx_seq = g_tx_in_flight_seq;
        g_tx_seq = static_cast<uint16_t>(g_tx_in_flight_seq + 1u);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (g_tx_in_flight != 0u) {
            g_tx_in_flight = 0u;
            uart2_protocol_test_stats.tx_errors++;
            uart2_protocol_test_stats.tx_hal_error_errors++;
        }
        uart2_protocol_test_stats.uart_errors++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        RestartReceive();
    }
}
