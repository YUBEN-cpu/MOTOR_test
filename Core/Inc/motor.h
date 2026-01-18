//
// Created by YUBEN on 2026/1/15.
//
#ifndef MOTOR_TEST_MOTOR_H
#define MOTOR_TEST_MOTOR_H

#include "main.h"
#include "stm32f1xx_hal.h"

extern volatile float Speed_R ;
extern volatile float Speed_L ;
extern uint32_t SAMPLE_TIME ;        // ✅ 采样时间 20ms，可改10/50/100

typedef enum {
    R,
    L,
    B        //both
}Motor_Side;

void Motor_Init() ;

void Motor_SetSpeed(int speed, Motor_Side side);

void Motor_Get_Speed();

//电机的pid参数设置函数
void Motor_PID_Init(float kp, float ki, float kd) ;

//积分和输出限幅设置函数
void Motor_PID_SetLimit(int16_t outMin, int16_t outMax,int32_t integralMin, int32_t integralMax, int8_t DerivativeMax) ;

//电机的目标转速设置函数
void Motor_PID_SetTarget(int16_t speed, Motor_Side side) ;

//pid执行函数
void Motor_PID_Enforce();

#endif //MOTOR_TEST_MOTOR_H