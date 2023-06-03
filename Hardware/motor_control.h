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
        uint8_t Encoder;
        uint8_t last_encoder;
        uint8_t total_encoder;
}motor_param;

extern motor_param motor_l, motor_r,motor;

extern TIM_HandleTypeDef htim2;//����TIM2��HAL��ṹ��
extern TIM_HandleTypeDef htim3;//����TIM3��HAL��ṹ��
extern TIM_HandleTypeDef htim4;//����TIM4��HAL��ṹ��

extern	float Angle_gx;    
extern	float Angle_gy;    
extern	float Angle_gz; 
void Get_Encoder(void);
void Get_Speed(void);
void Limit(int *PWMA,int *PWMB);
void Give_Motor_PWM(int MotorL_PWM,int MotorR_PWM);
int ABS(int a);

#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
