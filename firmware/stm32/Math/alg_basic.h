/**
 * @file alg_basic.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些极其简易的数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-10 1.1 修改成cpp
 * @date 2025-09-23 2.1 引入NaN判断
 *
 * @copyright Copyright (c) 2023-2025
 *
 */

#ifndef ALG_BASIC_H
#define ALG_BASIC_H

/* Includes ------------------------------------------------------------------*/
#include <float.h>
#include "arm_math.h"
#include "stm32h7xx_hal.h"


/* Exported macros -----------------------------------------------------------*/

extern const float BASIC_MATH_RPM_TO_RADPS;
extern const float BASIC_MATH_DEG_TO_RAD;
extern const float BASIC_MATH_RAD_TO_DEG;
extern const float BASIC_MATH_CELSIUS_TO_KELVIN;

/* Exported types ------------------------------------------------------------*/
template <typename Type> 
struct MAFilterType{
        Type *buf;
        uint16_t len;
        uint16_t idx;
        uint16_t count;
        Type sum;
};

// IEEE 754 单精度分解结构
struct IEEE754Single {
    uint32_t sign;      // 1 bit
    uint32_t exponent;  // 8 bits (偏置 127)
    uint32_t fraction;  // 23 bits
    uint32_t bits;      // 原始比特
};
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Basic_Math_Boolean_Logical_Not(bool *Value);

void Basic_Math_Endian_Reverse_16(void *Address);

uint16_t Basic_Math_Endian_Reverse_16(void *Source, void *Destination);

void Basic_Math_Endian_Reverse_32(void *Address);

uint32_t Basic_Math_Endian_Reverse_32(void *Source, void *Destination);

uint8_t Basic_Math_Sum_8(const uint8_t *Address, uint32_t Length);

uint16_t Basic_Math_Sum_16(const uint16_t *Address, uint32_t Length);

uint32_t Basic_Math_Sum_32(const uint32_t *Address, uint32_t Length);

float Basic_Math_Sinc(float x);

float Basic_Math_Deg_To_Rad(float deg);

float Basic_Math_Rad_To_Deg(float rad);

int32_t Basic_Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2);

float Basic_Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2);

bool Basic_Math_Is_Invalid_Float(float x);

float Basic_Math_Modulus_Normalization(float x, float modulus);

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 * @return 输出值
 */
template<typename Type>
Type Basic_Math_Constrain(Type x, Type Min, Type Max)
{
    if (x < Min)
    {
        x = Min;
    }
    else if (x > Max)
    {
        x = Max;
    }
    return (x);
}

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 * @return 输出值
 */
template<typename Type>
Type Basic_Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type Basic_Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}


// 返回原始 32 位比特（按 IEEE 754 内存布局）
static inline uint32_t Math_FloatToBits(float x) {
    uint32_t u = 0;
    memcpy(&u, &x, sizeof(u));
    return u;
}

//bit转float32
static inline float Math_BitsToFloat(uint32_t bits) {
    float x = 0.0f;
    memcpy(&x, &bits, sizeof(x));  // 避免别名问题
    return x;
}

template <typename Type>
inline void Math_Constrain(Type* x, Type Min, Type Max) {
    if (*x < Min)      *x = Min;
    else if (*x > Max) *x = Max;
}

template <typename Type>
inline Type Math_Abs(Type x) {
    return (x > 0) ? x : -x;
}

//符号函数
template <typename Type>
inline int Math_Sign(Type x) {
    return (Type(0) < x) - (x < Type(0));
}


template <typename Type>
Type LowPassFilter(Type now, Type pre, float alph)
{
    return pre*alph+(1.0f-alph)*now;
}

template <typename Type>
class MovingAverageFilter
{
public:
    void MA_InitFilter(Type *buffer, uint16_t length)
    {
        MAFilterType<Type>& s = struc;
        s.buf   = buffer;
        s.len   = length;
        s.idx   = 0;
        s.count = 0;
        s.sum   = 0.0f;
        for (uint16_t i = 0; i < length; i++) buffer[i] = 0.0f;
    };
    
    Type MA_UpdateFilter(Type sample)
    {
        MAFilterType<Type>& s = struc;
        if (s.count < s.len) {
            s.sum += sample;
            s.buf[s.idx] = sample;
            s.idx = (uint16_t)(s.idx + 1) % s.len;
            s.count++;
            return s.sum / (Type)s.count;
        } else {
            float old = s.buf[s.idx];
            s.sum += sample - old;
            s.buf[s.idx] = sample;
            s.idx = (uint16_t)(s.idx + 1) % s.len;
            return s.sum / (Type)s.len;
        }
    };
    Type MA_ResetF(void)
    {
        MAFilterType<Type>& s = struc;
        s.idx = 0;
        s.count = 0;
        s.sum = 0.0f;
        for (uint16_t i = 0; i < s.len; i++) s.buf[i] = 0.0f;
    }

private:
    MAFilterType<Type> struc;
};
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
