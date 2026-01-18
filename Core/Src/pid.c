//
// Created by YUBEN on 2026/1/15.
//

#include "pid.h"
#include "motor.h"


/**
  * @brief  PID参数初始化
  * @param  pid: PID结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    // 初始化状态变量
    pid->Target = 0;
    pid->Feedback = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Derivative = 0;

    // 默认限制（可通过PID_SetLimit修改）
    pid->OutMin = -100;
    pid->OutMax = 100;
    pid->IntegralMin = -1000;
    pid->IntegralMax = 1000;
    pid->DerivativeMax = 50;
}

/**
  * @brief  设置目标转速
  * @param  pid: PID结构体指针
  * @param  target: 目标转速（RPM）
  */
void PID_SetTarget(PID_TypeDef *pid, int16_t target) {
    pid->Target = target;
}

/**
  * @brief  设置输出和积分和微分限幅
  * @param  outMin/outMax: 输出最小值/最大值（如PWM占空比）
  * @param  integralMin/integralMax: 积分最小值/最大值
  * @param  DerivativeMax: 微分最大值
  */
void PID_SetLimit(PID_TypeDef *pid, int16_t outMin, int16_t outMax,
                 int32_t integralMin, int32_t integralMax, int8_t DerivativeMax) {
    pid->OutMin = outMin;
    pid->OutMax = outMax;
    pid->IntegralMin = integralMin;
    pid->IntegralMax = integralMax;
    pid->DerivativeMax = DerivativeMax;
}

/**
  * @brief  PID计算核心函数
  * pid结构体
  * @param  feedback: 实际转速（编码器测量值）
  * @retval 输出值（如PWM占空比）
  */
int16_t PID_Calc(PID_TypeDef *pid, int16_t feedback) {
    // 更新反馈与误差
    pid->Feedback = feedback;
    pid->Error = pid->Target - pid->Feedback;

    // ✅ 优化1：误差为0时，积分清零，防止积分残留超调
    if(pid->Error == 0)
    {
        //pid->Integral = 0;
    }

    // 积分项（带限幅，防止饱和）
    pid->Integral += pid->Error;
    if (pid->Integral > pid->IntegralMax) {
        pid->Integral = pid->IntegralMax;
    } else if (pid->Integral < pid->IntegralMin) {
        pid->Integral = pid->IntegralMin;
    }

    // 微分项（当前误差 - 上一次误差）✅优化2：微分限幅，防止噪声放大
    pid->Derivative = pid->Error - pid->LastError;
    if(pid->Derivative > pid->DerivativeMax) pid->Derivative = pid->DerivativeMax;
    if(pid->Derivative < -pid->DerivativeMax) pid->Derivative = -pid->DerivativeMax;
    pid->LastError = pid->Error;  // 保存当前误差供下次使用

    // PID总输出计算
    int32_t output = (int32_t)(pid->Kp * pid->Error +
                              pid->Ki * pid->Integral +
                              pid->Kd * pid->Derivative);

    // 输出限幅（防止超出电机驱动能力）
    if (output > pid->OutMax) {
        output = pid->OutMax;
    } else if (output < pid->OutMin) {
        output = pid->OutMin;
    }

    return (int16_t)output;
}
