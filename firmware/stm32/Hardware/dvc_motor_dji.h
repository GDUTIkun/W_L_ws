/**
 * @file dvc_motor_dji.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆电机配置与操作
 * @version 1.2
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2024-03-07 1.2 新增功率控制接口与相关函数
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本, 功率控制接口移除, 后续建议通过上层或继承类实现
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

#ifndef DVC_MOTOR_DJI_H
#define DVC_MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆状态
 *
 */
enum Enum_Motor_DJI_Status
{
    Motor_DJI_Status_DISABLE = 0,
    Motor_DJI_Status_ENABLE,
};

/**
 * @brief 大疆电机的ID枚举类型
 *
 */
enum Enum_Motor_DJI_ID
{
    CAN_Motor_ID_0x2E = 1,
    CAN_Motor_ID_0x4E,
    CAN_Motor_ID_0x6E,
    CAN_Motor_ID_0x8E,
    Motor_DJI_ID_0x201,
    Motor_DJI_ID_0x202,
};

/**
 * @brief 大疆电机控制方式
 *
 */
enum Enum_Motor_DJI_Control_Method
{
    Motor_DJI_Control_Method_TORQUE = 0,
    Motor_DJI_Control_Method_OMEGA,
    Motor_DJI_Control_Method_ANGLE,
};

/**
 * @brief 大疆电机源数据
 *
 */
struct Struct_Motor_DJI_CAN_Rx_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 大疆电机经过处理的数据
 *
 */
struct Struct_Motor_DJI_Rx_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Temperature;
    float Now_Power;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief Reusable, GIM6010无刷电机, 单片机控制输出电压, 新驱动支持电流控制
 *
 */
class Class_Motor_DJI_GIM6010
{
public:
    Class_Motor_DJI_GIM6010();

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const float &__Angle_Offset = 0.0f);

    inline Enum_Motor_DJI_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline float Get_Now_Temperature() const;

    inline float Get_Now_Power() const;

    inline float Get_Target_Torque() const;

    inline float Get_Feedforward_Torque() const;

    inline void Set_Target_Torque(const float &__Target_Torque);

    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    void CAN_RxCpltCallback(uint8_t data_id);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 电机原始角度零点偏置, rad. GIM6010接收后还会按腿部安装方向映射到机构角.
    float Angle_Offset;

    // 常量

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND;

    // 最大输出刻度
    const float OUT_MAX;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;
    // 输出量
    float Out;

    // 读变量

    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method;
    // 目标的扭矩, Nm
    float Target_Torque;
    // 前馈的扭矩, Nm
    float Feedforward_Torque;

    // 内部函数

    void Data_Process(uint8_t data_id);

    void Output();
};


/**
 * @brief Reusable, C620无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_DJI_C620
{
public:
    Class_Motor_DJI_C620();

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID);

    inline Enum_Motor_DJI_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline float Get_Target_Torque() const;

    inline float Get_Feedforward_Omega() const;

    inline float Get_Feedforward_Torque() const;

    inline void Set_Target_Torque(const float &__Target_Torque);
    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    void CAN_RxCpltCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;

    // 常量

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND;
    // 扭矩电流常数, 减速前
    const float CURRENT_TO_TORQUE;
    // 电流到输出的转化系数
    const float CURRENT_TO_OUT;
    // 最大输出刻度
    const float OUT_MAX;
    const float Threshold_Current;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;
    // 输出量
    float Out;

    // 读变量

    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;

    // 读写变量
    // 目标的扭矩, Nm
    float Target_Torque;
    // 前馈的扭矩, Nm
    float Feedforward_Torque;

    // 内部函数

    void Data_Process();

    void Output();
};

/* Exported variables --------------------------------------------------------*/


//构造函数初始化
/* Exported function declarations --------------------------------------------*/

inline Class_Motor_DJI_GIM6010::Class_Motor_DJI_GIM6010()
    : CAN_Manage_Object(0), Angle_Offset(0.0f),
    ENCODER_NUM_PER_ROUND(8192), OUT_MAX(16384.0f), Flag(0), Pre_Flag(0), 
    Out(0.0f), Motor_DJI_Status(Motor_DJI_Status_DISABLE), 
   Target_Torque(0.0f), Feedforward_Torque(0.0f)
{
}


inline Class_Motor_DJI_C620::Class_Motor_DJI_C620()
    : CAN_Manage_Object(0), Tx_Data(0), Gearbox_Rate(3591.0f / 187.0f),
    ENCODER_NUM_PER_ROUND(8192), CURRENT_TO_TORQUE(0.3f),
    CURRENT_TO_OUT(16384.0f / 20.0f), OUT_MAX(16384.0f), 
    Threshold_Current(300),Flag(0), Pre_Flag(0), 
    Out(0.0f), Motor_DJI_Status(Motor_DJI_Status_DISABLE), 
    Target_Torque(0.0f), Feedforward_Torque(0.0f)
{
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DJI_Status 电机状态
 */
inline Enum_Motor_DJI_Status Class_Motor_DJI_GIM6010::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取当前的角度, rad. GIM6010返回按腿部标定映射后的机构角.
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_DJI_GIM6010::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s. GIM6010返回按腿部标定映射后的机构角速度.
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_DJI_GIM6010::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float Class_Motor_DJI_GIM6010::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DJI_GIM6010::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DJI_GIM6010::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}


/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DJI_GIM6010::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DJI_GIM6010::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DJI_Status 电机状态
 */
inline Enum_Motor_DJI_Status Class_Motor_DJI_C620::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_DJI_C620::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_DJI_C620::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float Class_Motor_DJI_C620::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DJI_C620::Get_Target_Torque() const
{
    return (Target_Torque);
}


/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DJI_C620::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DJI_C620::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DJI_C620::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
