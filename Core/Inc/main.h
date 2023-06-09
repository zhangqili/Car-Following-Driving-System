/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define PWMA_Pin GPIO_PIN_0
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_1
#define PWMB_GPIO_Port GPIOA
#define Encoder_Left_A_Pin GPIO_PIN_6
#define Encoder_Left_A_GPIO_Port GPIOA
#define Encoder_Left_B_Pin GPIO_PIN_7
#define Encoder_Left_B_GPIO_Port GPIOA
#define BOOT_Pin GPIO_PIN_2
#define BOOT_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_8
#define INT_GPIO_Port GPIOA
#define INT_EXTI_IRQn EXTI9_5_IRQn
#define UP_Pin GPIO_PIN_15
#define UP_GPIO_Port GPIOA
#define DOWN_Pin GPIO_PIN_3
#define DOWN_GPIO_Port GPIOB
#define OK_Pin GPIO_PIN_4
#define OK_GPIO_Port GPIOB
#define BACK_Pin GPIO_PIN_5
#define BACK_GPIO_Port GPIOB
#define Encoder_Right_B_Pin GPIO_PIN_6
#define Encoder_Right_B_GPIO_Port GPIOB
#define Encoder_Right_A_Pin GPIO_PIN_7
#define Encoder_Right_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern float Up_balance_KP;   //С��ֱ����P����
extern float Up_balance_KD;  //С��ֱ����D����
extern float Velocity_KP;   //С���ٶȻ�P����
extern float Velocity_KI;    // С���ٶȻ�I����
extern float Turn_KP;          //С��ת��P����-20
extern float Turn_KD; 


extern float Mechanical_Angle;   //�ǶȻ�е��ֵ
extern int Mechanical_velocity;   //�ٶȻ�е��ֵ
extern int turn_speed;                   //ת����ֵ
extern int Encoder_Left,Encoder_Right;   //���ұ��������������?
extern int balance_up;
extern int velocity;
extern int turn_out;
extern short gx,gy,gz;   //����ԭʼ����
extern float pitch,roll,yaw;   //ŷ����
extern int PWMA,PWMB;   //������������ո��������PWM

extern uint8_t color_flag;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
