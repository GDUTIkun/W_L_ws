/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行
 *
 */
enum Enum_PID_D_First
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

/**
 * @brief Reusable, PID算法
 *
 */
class Class_PID
{
public:
    Class_PID();

    void Init(const float &__K_P, const float &__K_I, const float &__K_D, const float &__K_F = 0.0f, const float &__I_Out_Max = 0.0f, const float &__Out_Max = 0.0f, const float &__D_T = 0.001f, const float &__Dead_Zone = 0.0f, const float &__I_Variable_Speed_A = 0.0f, const float &__I_Variable_Speed_B = 0.0f, const float &__I_Separate_Threshold = 0.0f, const Enum_PID_D_First &__D_First = PID_D_First_DISABLE, const float &__Derivative_LPF_RC = 0.0f, const float &__Output_LPF_RC = 0.0f);

    inline float Get_Integral_Error() const;

    inline float Get_Out() const;

    inline void Set_K_P(const float &__K_P);

    inline void Set_K_I(const float &__K_I);

    inline void Set_K_D(const float &__K_D);

    inline void Set_K_F(const float &__K_F);

    inline void Set_I_Out_Max(const float &__I_Out_Max);

    inline void Set_Out_Max(const float &__Out_Max);

    inline void Set_I_Variable_Speed_A(const float &__I_Variable_Speed_A);

    inline void Set_I_Variable_Speed_B(const float &__I_Variable_Speed_B);

    inline void Set_I_Separate_Threshold(const float &__I_Separate_Threshold);

    inline void Set_Derivative_LPF_RC(const float &__Derivative_LPF_RC);

    inline void Set_Output_LPF_RC(const float &__Output_LPF_RC);

    inline void Set_Target(const float &__Target);

    inline void Set_Now(const float &__Now);

    inline void Set_Integral_Error(const float &__Integral_Error);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // PID计时器周期, s
    float D_T;
    // 死区, Error在其绝对值内不输出
    float Dead_Zone;
    // 微分先行
    Enum_PID_D_First D_First;

    // 常量

    // 内部变量

    // 之前的当前值
    float Pre_Now;
    // 之前的目标值
    float Pre_Target;
    // 之前的输出值
    float Pre_Out;
    // 之前的微分输出值
    float Pre_D_Out;
    // 前向误差
    float Pre_Error;

    // 读变量

    // 输出值
    float Out;

    // 写变量

    // PID的P
    float K_P;
    // PID的I
    float K_I;
    // PID的D
    float K_D;
    // 前馈
    float K_F;

    // 积分限幅, 0为不限制
    float I_Out_Max;
    // 输出限幅, 0为不限制
    float Out_Max;

    // 变速积分定速内段阈值, 0为不限制
    float I_Variable_Speed_A;
    // 变速积分变速区间, 0为不限制
    float I_Variable_Speed_B;
    // 积分分离阈值，需为正数, 0为不限制
    float I_Separate_Threshold;

    // 微分低通滤波RC常数, 0为不限制
    float Derivative_LPF_RC;
    // 输出低通滤波RC常数, 0为不限制
    float Output_LPF_RC;

    // 目标值
    float Target;
    // 当前值
    float Now;

    // 读写变量

    // 积分值
    float Integral_Error;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline Class_PID::Class_PID()
    : D_T(0.001f), Dead_Zone(0.0f), D_First(PID_D_First_DISABLE), Pre_Now(0.0f), Pre_Target(0.0f), Pre_Out(0.0f), Pre_D_Out(0.0f), Pre_Error(0.0f), Out(0.0f), K_P(0.0f), K_I(0.0f), K_D(0.0f), K_F(0.0f), I_Out_Max(0.0f), Out_Max(0.0f), I_Variable_Speed_A(0.0f), I_Variable_Speed_B(0.0f), I_Separate_Threshold(0.0f), Derivative_LPF_RC(0.0f), Output_LPF_RC(0.0f), Target(0.0f), Now(0.0f), Integral_Error(0.0f)
{
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_PID::Get_Integral_Error() const
{
    return (Integral_Error);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_PID::Get_Out() const
{
    return (Out);
}

/**
 * @brief 设定PID的P
 *
 * @param __K_P PID的P
 */
inline void Class_PID::Set_K_P(const float &__K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定PID的I
 *
 * @param __K_I PID的I
 */
inline void Class_PID::Set_K_I(const float &__K_I)
{
    K_I = __K_I;
}

/**
 * @brief 设定PID的D
 *
 * @param __K_D PID的D
 */
inline void Class_PID::Set_K_D(const float &__K_D)
{
    K_D = __K_D;
}

/**
 * @brief 设定前馈
 *
 * @param __K_D 前馈
 */
inline void Class_PID::Set_K_F(const float &__K_F)
{
    K_F = __K_F;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param __I_Out_Max 积分限幅, 0为不限制
 */
inline void Class_PID::Set_I_Out_Max(const float &__I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param __Out_Max 输出限幅, 0为不限制
 */
inline void Class_PID::Set_Out_Max(const float &__Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param __I_Variable_Speed_A 定速内段阈值, 0为不限制
 */
inline void Class_PID::Set_I_Variable_Speed_A(const float &__I_Variable_Speed_A)
{
    I_Variable_Speed_A = __I_Variable_Speed_A;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param __I_Variable_Speed_B 变速区间, 0为不限制
 */
inline void Class_PID::Set_I_Variable_Speed_B(const float &__I_Variable_Speed_B)
{
    I_Variable_Speed_B = __I_Variable_Speed_B;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 *
 * @param __I_Separate_Threshold 积分分离阈值，需为正数, 0为不限制
 */
inline void Class_PID::Set_I_Separate_Threshold(const float &__I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

/**
 * @brief 设定微分低通滤波RC常数, 0为不限制
 *
 * @param __Derivative_LPF_RC 微分低通滤波RC常数, 0为不限制
 */
inline void Class_PID::Set_Derivative_LPF_RC(const float &__Derivative_LPF_RC)
{
    Derivative_LPF_RC = __Derivative_LPF_RC;
}

/**
 * @brief 设定输出低通滤波RC常数, 0为不限制
 *
 * @param __Output_LPF_RC 输出低通滤波RC常数, 0为不限制
 */
inline void Class_PID::Set_Output_LPF_RC(const float &__Output_LPF_RC)
{
    Output_LPF_RC = __Output_LPF_RC;
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
inline void Class_PID::Set_Target(const float &__Target)
{
    Target = __Target;
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
inline void Class_PID::Set_Now(const float &__Now)
{
    Now = __Now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param __Set_Integral_Error 积分值
 */
inline void Class_PID::Set_Integral_Error(const float &__Integral_Error)
{
    Integral_Error = __Integral_Error;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/