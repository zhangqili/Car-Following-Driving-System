#include "us100.h"
#include "main.h"
#include "usart.h"

uint8_t US100_DistanceBuffer[2];
uint8_t US100_Temperature;
uint16_t US100_Distance;

uint8_t US100_Requset_Distance[1]={0x55};
uint8_t US100_Requset_Temperature[1]={0x50};

void US100_GetDistance()
{

	HAL_UART_Transmit(&huart3, US100_Requset_Distance, 1, 100);
	//HAL_UART_Receive_IT(&huart2, &(us100_dist.Pointer), 2);
	HAL_UART_Receive_DMA(&huart3, US100_DistanceBuffer, 2);
	//HAL_UART_Receive_IT(&huart2, (uint8_t *)&us100_distance, 2);
}
void US100_GetTemperature()
{
	HAL_UART_Transmit(&huart3, US100_Requset_Temperature, 1, 100);
	HAL_UART_Receive_DMA(&huart3, &US100_Temperature, 1);
}