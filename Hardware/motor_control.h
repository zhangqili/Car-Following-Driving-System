/*
 * motor_control.h
 *
 *  Created on: Aug 15, 2022
 *      Author: HP
 */

#ifndef MOTOR_CONTROL_MOTOR_CONTROL_H_
#define MOTOR_CONTROL_MOTOR_CONTROL_H_

#include "main.h"

typedef struct motor_param{
        int16_t Encoder;
        int16_t last_encoder;
        int16_t total_encoder;
}motor_param;

extern motor_param motor_l, motor_r,motor;

extern TIM_HandleTypeDef htim2;//声明TIM2的HAL库结构体
extern TIM_HandleTypeDef htim3;//声明TIM3的HAL库结构体
extern TIM_HandleTypeDef htim4;//声明TIM4的HAL库结构体
extern int8_t Speed_differ;
extern	float Angle_gx;    
extern	float Angle_gy;    
extern	float Angle_gz; 
void Get_Encoder(void);
void Get_Speed(void);
void Limit(float *PWMA,float *PWMB);
void Give_Motor_PWM(int MotorL_PWM,int MotorR_PWM);
int ABS(int a);

#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
