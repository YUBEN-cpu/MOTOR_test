//
// Created by YUBEN on 2026/1/15.
//

#include "motor.h"
#include "main.h"
#include "tim.h"
#include "PID.h"


PID_TypeDef pid_motor_R, pid_motor_L;   //电机的pid，后面应该还有超声波的pid

#define PWM_MAX 100                         //pwm输出最大值
volatile float Speed_R = 0.0f;
volatile float Speed_L = 0.0f;
uint32_t SAMPLE_TIME = 10;                // 测速周期10ms
#define REDUCTION_RATIO 4.4f              // 减速比，1:4.4
#define ENCODER_PULSE 44.0f                // 编码器单圈脉冲数

//各电机相关的定时器和通道的宏定义，便于以后移植
//PWM定时器的宏定义
#define R_Motor_PWM_TIM htim1
#define L_Motor_PWM_TIM htim1

//PWM通道
#define R_Motor_PWM_Channel TIM_CHANNEL_1
#define L_Motor_PWM_Channel TIM_CHANNEL_2

//编码器计时器
#define R_Motor_Encoder_TIM htim2
#define L_Motor_Encoder_TIM htim3




void Motor_Init() {
    HAL_TIM_PWM_Start(&R_Motor_PWM_TIM, R_Motor_PWM_Channel);
    __HAL_TIM_SET_COMPARE(&R_Motor_PWM_TIM, R_Motor_PWM_Channel, 0);
    HAL_TIM_PWM_Start(&L_Motor_PWM_TIM, L_Motor_PWM_Channel);
    __HAL_TIM_SET_COMPARE(&L_Motor_PWM_TIM, L_Motor_PWM_Channel, 0);
    // 2. 启动编码器测速定时器【关键新增】，TIM2=右电机，TIM3=左电机
    HAL_TIM_Encoder_Start(&R_Motor_Encoder_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&L_Motor_Encoder_TIM, TIM_CHANNEL_ALL);
    // 编码器计数器清零
    __HAL_TIM_SET_COUNTER(&R_Motor_Encoder_TIM, 0);
    __HAL_TIM_SET_COUNTER(&L_Motor_Encoder_TIM, 0);
}

//速度设置函数，加上左右参数,R右，L左， B一起
void Motor_SetSpeed(int speed, Motor_Side side) {
    if (side == R || side == B) {
        if (speed >= 0) {
            if (speed >= PWM_MAX) speed = PWM_MAX;
            HAL_GPIO_WritePin(Right_wheel_F_GPIO_Port, Right_wheel_F_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin(Right_wheel_B_GPIO_Port, Right_wheel_B_Pin, GPIO_PIN_RESET );
            __HAL_TIM_SET_COMPARE(&R_Motor_PWM_TIM, R_Motor_PWM_Channel, (uint32_t)speed);
        }else {
            if (speed <= -PWM_MAX) speed = -PWM_MAX;
            HAL_GPIO_WritePin(Right_wheel_F_GPIO_Port, Right_wheel_F_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin(Right_wheel_B_GPIO_Port, Right_wheel_B_Pin, GPIO_PIN_SET );
            __HAL_TIM_SET_COMPARE(&R_Motor_PWM_TIM, R_Motor_PWM_Channel, -(uint32_t)speed);
        }
    }
    if (side == L || side == B){
        if (speed >= 0) {
            if (speed >= PWM_MAX) speed = PWM_MAX;
            HAL_GPIO_WritePin(Left_wheel_F_GPIO_Port, Left_wheel_F_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin(Left_wheel_B_GPIO_Port, Left_wheel_B_Pin, GPIO_PIN_RESET );
            __HAL_TIM_SET_COMPARE(&L_Motor_PWM_TIM, L_Motor_PWM_Channel, (uint32_t)speed);
        }else {
            if (speed <= -PWM_MAX) speed = -PWM_MAX;
            HAL_GPIO_WritePin(Left_wheel_F_GPIO_Port, Left_wheel_F_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin(Left_wheel_B_GPIO_Port, Left_wheel_B_Pin, GPIO_PIN_SET );
            __HAL_TIM_SET_COMPARE(&L_Motor_PWM_TIM, L_Motor_PWM_Channel, -(uint32_t)speed);
        }
    }

}

void Motor_Get_Speed() {
    //速度计算，脉冲数/一圈的脉冲数/计时时间*60000（转每分钟）
    int16_t cnt_R = (int16_t)__HAL_TIM_GET_COUNTER(&R_Motor_Encoder_TIM);
    int16_t cnt_L = (int16_t)__HAL_TIM_GET_COUNTER(&L_Motor_Encoder_TIM);
    Speed_R = (float)cnt_R / (ENCODER_PULSE * (float)SAMPLE_TIME * REDUCTION_RATIO) * 60000.0;
    Speed_L = (float)cnt_L / (ENCODER_PULSE * (float)SAMPLE_TIME * REDUCTION_RATIO) * 60000.0;
    __HAL_TIM_SET_COUNTER(&R_Motor_Encoder_TIM, 0);
    __HAL_TIM_SET_COUNTER(&L_Motor_Encoder_TIM, 0);
}


//电机的pid参数设置函数
void Motor_PID_Init(float kp, float ki, float kd) {
    PID_Init(&pid_motor_R, kp, ki, kd);
    PID_Init(&pid_motor_L, kp, ki, kd);
}

//电机的目标转速设置函数
void Motor_PID_SetTarget(int16_t speed, Motor_Side side) {
    if (side == R || side == B) {
        PID_SetTarget(&pid_motor_R, speed);
    }
    if (side == L || side == B) {
        PID_SetTarget(&pid_motor_L, speed);
    }
}

void Motor_PID_SetLimit(int16_t outMin, int16_t outMax,int32_t integralMin, int32_t integralMax, int8_t DerivativeMax) {
    PID_SetLimit(&pid_motor_L, outMin, outMax, integralMin, integralMax, DerivativeMax);
    PID_SetLimit(&pid_motor_R, outMin, outMax, integralMin, integralMax, DerivativeMax);
}

//pid执行函数，10ms执行一次
void Motor_PID_Enforce() {
    int out_put_R = (int)PID_Calc(&pid_motor_R, (int16_t)Speed_R);
    int out_put_L = (int)PID_Calc(&pid_motor_L, (int16_t)Speed_L);
    Motor_SetSpeed(out_put_R,R);
    Motor_SetSpeed(out_put_L,L);
}



