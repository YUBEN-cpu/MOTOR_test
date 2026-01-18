//
// Created by YUBEN on 2026/1/15.
//

#ifndef MOTOR_TEST_PID_H
#define MOTOR_TEST_PID_H

#include "stdint.h"

// PID结构体：存储参数、状态和限制值
// 在使用到pid的源文件里定义
typedef struct {
    // 核心参数
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数

    // 目标与反馈
    int16_t Target;   // 目标转速（RPM）
    int16_t Feedback; // 实际转速（RPM）

    // 中间变量
    int16_t Error;       // 当前误差 = Target - Feedback
    int16_t LastError;   // 上一次误差
    int32_t Integral;    // 积分项（累积误差）
    int16_t Derivative;  // 微分项（当前误差 - 上一次误差）

    // 输出限制（保护电机，如PWM占空比范围）
    int16_t OutMin;
    int16_t OutMax;

    // 积分限幅（防止积分饱和）
    int32_t IntegralMin;
    int32_t IntegralMax;

    // 微分限幅，防止放大噪声
    int8_t DerivativeMax;
} PID_TypeDef;


void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd) ;

void PID_SetTarget(PID_TypeDef *pid, int16_t target);

void PID_SetLimit(PID_TypeDef *pid, int16_t outMin, int16_t outMax,
                 int32_t integralMin, int32_t integralMax, int8_t DerivativeMax);

int16_t PID_Calc(PID_TypeDef *pid, int16_t feedback);



#endif //MOTOR_TEST_PID_H