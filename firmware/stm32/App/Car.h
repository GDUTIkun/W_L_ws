#ifndef __CAR_H
#define __CAR_H
#include "FreeRTOS.h"
#include "task.h"
#include "jy901s.h"
#include "dvc_motor_dji.h"
#include "usart.h"

void Car_Init(void);

extern bool init_finished;
extern volatile float gyx, gyy, gyz;
extern volatile float accx, accy, accz;
extern volatile float imu_roll, imu_pitch, imu_yaw;
extern float r_hip_angle, r_knee_angle, l_hip_angle, l_knee_angle;
extern float r_knee_relative_angle, l_knee_relative_angle;
extern float r_hip_omega, r_knee_omega, l_hip_omega, l_knee_omega;
extern uint8_t knee_limit_flag;

#endif
