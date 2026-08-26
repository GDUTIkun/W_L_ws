/**
 * @file dvc_motor_dji.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆电机配置与操作
 * @version 1.2
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2024-03-07 1.2 新增功率控制接口与相关函数, 24赛季定稿
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

namespace
{
static const float kDegToRad = PI / 180.0f;
static const float kTwoPi = 2.0f * PI;

static float g_left_hip_world_angle = 0.0f;
static float g_right_hip_world_angle = 0.0f;
static float g_left_hip_world_omega = 0.0f;
static float g_right_hip_world_omega = 0.0f;

float NormalizeAnglePositive(float angle)
{
    while (angle < 0.0f)
    {
        angle += 2.0f * PI;
    }
    while (angle >= 2.0f * PI)
    {
        angle -= 2.0f * PI;
    }
    return angle;
}

int16_t QuantizePositionFeedforward(float value)
{
    const float scaled = value * 1000.0f;
    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }
    return static_cast<int16_t>(scaled);
}

void MapGIM6010AngleToLegFrame(const Enum_Motor_DJI_ID can_id,
                               const float motor_angle,
                               const float motor_omega,
                               float *mapped_angle,
                               float *mapped_omega)
{
    // Calibration from leg_data.txt:
    // World angle convention: horizontal +X is 0 deg, clockwise is positive.
    // Vertical downward is 90 deg and vertical upward is 270 deg.
    // Hip calibration pose maps both sides to 190.81 deg in world frame.
    // Knee calibration pose maps link angle magnitude 63.86 deg to -63.86 deg
    // in the current relative-angle convention.
    // Knee motors are mounted on hip_link, so knee world angle is:
    // hip_world_angle + knee_relative_angle + knee_offset.
    switch (can_id)
    {
    case CAN_Motor_ID_0x4E:
        *mapped_angle = NormalizeAnglePositive(-motor_angle + 190.81f * kDegToRad);
        *mapped_omega = -motor_omega;
        g_left_hip_world_angle = *mapped_angle;
        g_left_hip_world_omega = *mapped_omega;
        break;
    case CAN_Motor_ID_0x2E:
        *mapped_angle = NormalizeAnglePositive(motor_angle + 127.758f * kDegToRad);
        *mapped_omega = motor_omega;
        g_right_hip_world_angle = *mapped_angle;
        g_right_hip_world_omega = *mapped_omega;
        break;
    case CAN_Motor_ID_0x6E:
    {
        const float knee_relative_angle = -motor_angle - 68.491f * kDegToRad;
        *mapped_angle = NormalizeAnglePositive(g_left_hip_world_angle +
                                               knee_relative_angle);
        *mapped_omega = g_left_hip_world_omega - motor_omega;
        break;
    }
    case CAN_Motor_ID_0x8E:
    {
        const float knee_relative_angle = motor_angle - 68.538f * kDegToRad;
        *mapped_angle = NormalizeAnglePositive(g_right_hip_world_angle +
                                               knee_relative_angle);
        *mapped_omega = g_right_hip_world_omega + motor_omega;
        break;
    }
    default:
        *mapped_angle = motor_angle;
        *mapped_omega = motor_omega;
        break;
    }
}
} // namespace

/* Function prototypes -------------------------------------------------------*/

// TX buffer mapping for current project: FDCAN1 only.
uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan,
                          const Enum_Motor_DJI_ID &__CAN_ID)
{
    uint8_t *tmp_tx_data_ptr = 0;

    if (hcan != &hfdcan1)
    {
        return (0);
    }

    switch (__CAN_ID)
    {
    case Motor_DJI_ID_0x201:
        tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
        break;
    case Motor_DJI_ID_0x202:
        tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
        break;
    case (CAN_Motor_ID_0x2E):
        tmp_tx_data_ptr = &(CAN1_0x2E_Tx_Data[0]);
        break;
    case (CAN_Motor_ID_0x4E):
        tmp_tx_data_ptr = &(CAN1_0x4E_Tx_Data[0]);
        break;
    case (CAN_Motor_ID_0x6E):
        tmp_tx_data_ptr = &(CAN1_0x6E_Tx_Data[0]);
        break;
    case (CAN_Motor_ID_0x8E):
        tmp_tx_data_ptr = &(CAN1_0x8E_Tx_Data[0]);
    default:
        break;
    }

    return (tmp_tx_data_ptr);
}

uint16_t *allocate_tx_id(const FDCAN_HandleTypeDef *hcan,
                         const Enum_Motor_DJI_ID &__CAN_ID)
{
    if (hcan != &hfdcan1)
    {
        return 0;
    }

    switch (__CAN_ID)
    {
    case CAN_Motor_ID_0x2E:
        return &CAN1_0x2E_Tx_ID;
    case CAN_Motor_ID_0x4E:
        return &CAN1_0x4E_Tx_ID;
    case CAN_Motor_ID_0x6E:
        return &CAN1_0x6E_Tx_ID;
    case CAN_Motor_ID_0x8E:
        return &CAN1_0x8E_Tx_ID;
    default:
        return 0;
    }
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角度
 * @param __Angle_Offset 角度零点偏置, rad
 */
void Class_Motor_DJI_GIM6010::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const float &__Angle_Offset)
{
    CAN_Manage_Object = &CAN1_Manage_Object;
    CAN_Rx_ID = __CAN_Rx_ID;
    Angle_Offset = __Angle_Offset;
    Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID);
    Tx_ID = allocate_tx_id(hcan, __CAN_Rx_ID);
    // Force the initial torque-mode frame even though the constructor default is
    // also torque mode.
    Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA;
    Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
    uint8_t mode_data2[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN_Transmit_Data((FDCAN_HandleTypeDef *) hcan, __CAN_Rx_ID<<5|0x7, mode_data2, 8);

}

void Class_Motor_DJI_GIM6010::CAN_RxCpltCallback(uint8_t data_id)
{
    Flag += 1;
    Data_Process(data_id);
}

void Class_Motor_DJI_GIM6010::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    }
    else
    {
        Motor_DJI_Status = Motor_DJI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

void Class_Motor_DJI_GIM6010::TIM_Calculate_PeriodElapsedCallback()
{
    Output();
    
    Feedforward_Torque = 0.0f;
}

void Class_Motor_DJI_GIM6010::Set_Target_Position(
    float position_rev,
    float velocity_feedforward_rev_per_sec,
    float torque_feedforward_nm)
{
    Target_Position = position_rev;
    Position_Velocity_Feedforward =
        QuantizePositionFeedforward(velocity_feedforward_rev_per_sec);
    Position_Torque_Feedforward =
        QuantizePositionFeedforward(torque_feedforward_nm);
    Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
}

void Class_Motor_DJI_GIM6010::Set_Target_Angle(
    float angle_rad,
    float velocity_feedforward_rev_per_sec,
    float torque_feedforward_nm)
{
    Set_Target_Position(angle_rad / kTwoPi,
                        velocity_feedforward_rev_per_sec,
                        torque_feedforward_nm);
}

void Class_Motor_DJI_GIM6010::Set_Target_Velocity(
    float velocity_rev_per_sec,
    float torque_feedforward_nm)
{
    Target_Velocity = velocity_rev_per_sec;
    Velocity_Torque_Feedforward = torque_feedforward_nm;
    Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
}

void Class_Motor_DJI_GIM6010::Set_Control_Method(
    Enum_Motor_DJI_Control_Method control_method)
{
    if (Motor_DJI_Control_Method == control_method && Tx_ID != 0)
    {
        return;
    }

    Motor_DJI_Control_Method = control_method;
    if (CAN_Manage_Object == 0 || CAN_Manage_Object->CAN_Handler == 0 ||
        Tx_ID == 0)
    {
        return;
    }

    uint8_t mode_data[8] = {0};
    if (control_method == Motor_DJI_Control_Method_ANGLE)
    {
        mode_data[0] = 0x03;
        mode_data[4] = 0x03;
        *Tx_ID = (static_cast<uint16_t>(CAN_Rx_ID) << 5) | 0x0C;
    }
    else if (control_method == Motor_DJI_Control_Method_OMEGA)
    {
        mode_data[0] = 0x02;
        mode_data[4] = 0x02;
        *Tx_ID = (static_cast<uint16_t>(CAN_Rx_ID) << 5) | 0x0D;
    }
    else
    {
        mode_data[0] = 0x01;
        mode_data[4] = 0x01;
        *Tx_ID = (static_cast<uint16_t>(CAN_Rx_ID) << 5) | 0x0E;
    }
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler,
                      (static_cast<uint16_t>(CAN_Rx_ID) << 5) | 0x0B,
                      mode_data,
                      8);
}

void Class_Motor_DJI_GIM6010::Data_Process(uint8_t data_id)
{
    uint8_t *tmp_buffer = CAN_Manage_Object->Rx_Buffer;
    if ((data_id & 0x0F) == 0x09)
    {
        uint32_t angle_bits = (static_cast<uint32_t>(tmp_buffer[3]) << 24) |
                              (static_cast<uint32_t>(tmp_buffer[2]) << 16) |
                              (static_cast<uint32_t>(tmp_buffer[1]) << 8) |
                              static_cast<uint32_t>(tmp_buffer[0]);
        const float motor_angle =
            Math_BitsToFloat(angle_bits) / 4.0f * PI - Angle_Offset;
        const float motor_omega =
            Math_BitsToFloat((tmp_buffer[7] << 24) | (tmp_buffer[6] << 16) |
                             (tmp_buffer[5] << 8) | tmp_buffer[4]) *
            4.0f * PI / 15.0f;
        MapGIM6010AngleToLegFrame(CAN_Rx_ID, motor_angle, motor_omega,
                                  &Rx_Data.Now_Angle, &Rx_Data.Now_Omega);
    }
    else if ((data_id & 0x0F) == 0x0C)
    {
        Feedback_Raw = (static_cast<uint32_t>(tmp_buffer[7]) << 24) |
                       (static_cast<uint32_t>(tmp_buffer[6]) << 16) |
                       (static_cast<uint32_t>(tmp_buffer[5]) << 8) |
                       static_cast<uint32_t>(tmp_buffer[4]);
        Rx_Data.Now_Torque = Math_BitsToFloat(Feedback_Raw);
    }
}
    


/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_DJI_GIM6010::Output()
{
    if (Tx_Data == 0)
    {
        return;
    }

    if (Motor_DJI_Control_Method == Motor_DJI_Control_Method_ANGLE)
    {
        const uint16_t velocity_feedforward =
            static_cast<uint16_t>(Position_Velocity_Feedforward);
        const uint16_t torque_feedforward =
            static_cast<uint16_t>(Position_Torque_Feedforward);
        Command_Raw = Math_FloatToBits(Target_Position);
        Tx_Data[4] = static_cast<uint8_t>(velocity_feedforward & 0xFF);
        Tx_Data[5] = static_cast<uint8_t>((velocity_feedforward >> 8) & 0xFF);
        Tx_Data[6] = static_cast<uint8_t>(torque_feedforward & 0xFF);
        Tx_Data[7] = static_cast<uint8_t>((torque_feedforward >> 8) & 0xFF);
    }
    else if (Motor_DJI_Control_Method == Motor_DJI_Control_Method_OMEGA)
    {
        Command_Raw = Math_FloatToBits(Target_Velocity);
        const uint32_t torque_feedforward =
            Math_FloatToBits(Velocity_Torque_Feedforward);
        Tx_Data[4] = static_cast<uint8_t>(torque_feedforward & 0xFF);
        Tx_Data[5] = static_cast<uint8_t>((torque_feedforward >> 8) & 0xFF);
        Tx_Data[6] = static_cast<uint8_t>((torque_feedforward >> 16) & 0xFF);
        Tx_Data[7] = static_cast<uint8_t>((torque_feedforward >> 24) & 0xFF);
    }
    else
    {
        Out = Target_Torque / 8.0f;
        Command_Raw = Math_FloatToBits(Out);
        Tx_Data[4] = 0;
        Tx_Data[5] = 0;
        Tx_Data[6] = 0;
        Tx_Data[7] = 0;
    }
    Tx_Data[3] = (Command_Raw >> 24) & 0xFF;
    Tx_Data[2] = (Command_Raw >> 16) & 0xFF;
    Tx_Data[1] = (Command_Raw >> 8) & 0xFF;
    Tx_Data[0] = Command_Raw & 0xFF;
}




/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_Rx_ID CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Current_Max 最大电流
 */

void Class_Motor_DJI_C620::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID)
{
    CAN_Manage_Object = &CAN1_Manage_Object;
    CAN_Rx_ID = __CAN_Rx_ID;
    Tx_Data = allocate_tx_data(hcan,  __CAN_Rx_ID);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_DJI_C620::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DJI_C620::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        Motor_DJI_Status = Motor_DJI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Motor_DJI_C620::TIM_Calculate_PeriodElapsedCallback()
{
    if (Current_Command_Mode)
    {
        Out = Target_Current * CURRENT_TO_OUT;
    }
    else
    {
        Out = (Target_Torque + Feedforward_Torque) / Gearbox_Rate / CURRENT_TO_TORQUE * CURRENT_TO_OUT;
    }
    if(Threshold_Current_Enabled && Math_Abs(Out) < Threshold_Current)
        Out = Out + Math_Sign(Out)*Threshold_Current;
    Basic_Math_Constrain(&Out, -OUT_MAX, OUT_MAX);

    Output();

    Feedforward_Torque = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DJI_C620::Data_Process()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) CAN_Manage_Object->Rx_Buffer;

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);
    Feedback_Raw = static_cast<uint32_t>(static_cast<uint16_t>(tmp_current));

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -ENCODER_NUM_PER_ROUND / 2)
    {
        // 正方向转过了一圈
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > ENCODER_NUM_PER_ROUND / 2)
    {
        // 反方向转过了一圈
        Rx_Data.Total_Round--;
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * ENCODER_NUM_PER_ROUND + tmp_encoder;

    // 计算电机本身信息
    Rx_Data.Now_Angle = (float) Rx_Data.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI / Gearbox_Rate;
    Rx_Data.Now_Omega = (float) tmp_omega * BASIC_MATH_RPM_TO_RADPS / Gearbox_Rate;
    Rx_Data.Now_Torque = tmp_current / CURRENT_TO_OUT * CURRENT_TO_TORQUE * Gearbox_Rate;

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_DJI_C620::Output()
{
    if (Tx_Data == 0)
    {
        return;
    }

    const int16_t command = static_cast<int16_t>(Out);
    Command_Raw = static_cast<uint32_t>(static_cast<uint16_t>(command));
    Tx_Data[0] = command >> 8;
    Tx_Data[1] = command;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
