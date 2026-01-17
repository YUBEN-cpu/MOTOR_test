//
// Created by YUBEN on 2026/1/15.
//

#include "motor.h"
#include "main.h"
#include "tim.h"

int PWM_MAX = 99;
int PWM_MIN = 0;
volatile float Speed_R = 0.0f;
volatile float Speed_L = 0.0f;
static uint32_t Last_tick = 0;       //上次滴答,静态变量不能声明
uint32_t SAMPLE_TIME = 100;        // ✅ 采样时间 20ms，可改10/50/100


void Motor_Init() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    // 2. 启动编码器测速定时器【关键新增】，TIM2=右电机，TIM3=左电机
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // 编码器计数器清零
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void Motor_SetSpeed_R(int speed) {
    if (speed >= 0) {
        if (speed >= PWM_MAX) speed = PWM_MAX;
        HAL_GPIO_WritePin(Right_wheel_F_GPIO_Port, Right_wheel_F_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin(Right_wheel_B_GPIO_Port, Right_wheel_B_Pin, GPIO_PIN_RESET );
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)speed);
    }else {
        if (speed <= -PWM_MAX) speed = -PWM_MAX;
        HAL_GPIO_WritePin(Right_wheel_F_GPIO_Port, Right_wheel_F_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin(Right_wheel_B_GPIO_Port, Right_wheel_B_Pin, GPIO_PIN_SET );
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -(uint32_t)speed);
    }
}

void Motor_SetSpeed_L(int speed) {
    if (speed >= 0) {
        if (speed >= PWM_MAX) speed = PWM_MAX;
        HAL_GPIO_WritePin(Left_wheel_F_GPIO_Port, Left_wheel_F_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin(Left_wheel_B_GPIO_Port, Left_wheel_B_Pin, GPIO_PIN_RESET );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)speed);
    }else {
        if (speed <= -PWM_MAX) speed = -PWM_MAX;
        HAL_GPIO_WritePin(Left_wheel_F_GPIO_Port, Left_wheel_F_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin(Left_wheel_B_GPIO_Port, Left_wheel_B_Pin, GPIO_PIN_SET );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -(uint32_t)speed);
    }
}

void Motor_Get_Speed() {
    uint32_t Now_tick = HAL_GetTick();
    if (Now_tick - Last_tick >= SAMPLE_TIME) {
        //速度计算，脉冲数/一圈的脉冲数/计时时间*60000（转每分钟）
        int16_t cnt_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t cnt_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        Speed_R = (float)cnt_R / (44.0 * (float)SAMPLE_TIME) * 60000.0;
        Speed_L = (float)cnt_L / (44.0 * (float)SAMPLE_TIME) * 60000.0;
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        Last_tick = Now_tick;
    }
}
