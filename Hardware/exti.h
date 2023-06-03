/*
 * exti.h
 *
 *  Created on: Aug 17, 2022
 *      Author: HP
 */

#ifndef EXTI_EXTI_H_
#define EXTI_EXTI_H_
#include "main.h"
#include "stm32f1xx_hal.h" //HAL库文件声明
extern UART_HandleTypeDef huart1;//声明USART2的HAL库结构体
extern int count;
#define USART1_REC_LEN  200//定义USART2最大接收字节数

extern uint8_t  USART1_RX_BUF[USART1_REC_LEN];//接收缓冲,最大USART_REC_LEN个字节.末字节为校验和
extern uint16_t USART1_RX_STA;//接收状态标记
extern uint8_t USART1_NewData;//当前串口中断接收的1个字节数据的缓存
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart);//串口中断回调函数声明

#endif /* EXTI_EXTI_H_ */
