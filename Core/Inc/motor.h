//
// Created by YUBEN on 2026/1/15.
//
#ifndef MOTOR_TEST_MOTOR_H
#define MOTOR_TEST_MOTOR_H

#include "main.h"
#include "stm32f1xx_hal.h"

extern int PWM_MAX ;
extern int PWM_MIN ;
extern volatile float Speed_R ;
extern volatile float Speed_L ;
extern uint32_t SAMPLE_TIME ;        // ✅ 采样时间 20ms，可改10/50/100


// 电机参数（根据你的电机修改！）
#define ENCODER_LINE  11      // 编码器线数（电机参数）

void Motor_Init() ;

void Motor_SetSpeed_R(int speed) ;

void Motor_SetSpeed_L(int speed) ;

void Motor_Get_Speed();

#endif //MOTOR_TEST_MOTOR_H