/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu6050.h"
#include "exti.h"
#include "string.h"
#include "display.h"
#include "ui.h"
#include "string.h"
#include "display.h"
#include "stdlib.h"
#include "stdio.h"
#include "pid_control.h"
#include "motor_control.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Up_balance_KP=200;   //С��ֱ����P����216
float Up_balance_KD=1.1;  //С��ֱ����D����2
float Velocity_KP=-53;   //С���ٶȻ�P����-50
float Velocity_KI=-0.265;    // С���ٶȻ�I����-0.25
float Turn_KP=-20;          //С��ת��P����-20
float Turn_KD=-0.6;          //С��ת��D����-0.6
float Mechanical_Angle=-5.5;   //�Ƕ���ֵ
int Mechanical_velocity=0;   //�ٶ���ֵ
int turn_speed=0;
int Encoder_Left,Encoder_Right=0;   //���ұ��������������
int balance_up=0;
int velocity=0;
int turn_out=0;
short gx,gy,gz=0;   //���ٶ�
float pitch,roll,yaw=0;   //ŷ����
int PWMA,PWMB=0;   //������������ո��������PWM
float tempFloat;
uint8_t tempInt;
uint8_t* f_ptr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	/* UI PRESETS BEGIN */
	
	//Turn.pGain=15;
	//Turn.dGain=25;
	//motor_pid_l.pGain=20;
	//motor_pid_l.iGain=10;
	//motor_pid_r.pGain=20;
	//motor_pid_r.iGain=10;
	u8g2Init(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
	//while(1)
	Flash_Recovery();
	while(UI_Flag)
	{
		/*
		u8g2_ClearBuffer(&u8g2);
		if(OK_BUTTON)
			u8g2_DrawStr(&u8g2,5,ITEM_HEIGHT*1,"OK");
		if(BACK_BUTTON)
			u8g2_DrawStr(&u8g2,5,ITEM_HEIGHT*2,"BACK");
		if(UP_BUTTON)
			u8g2_DrawStr(&u8g2,5,ITEM_HEIGHT*3,"UP");
		if(DOWN_BUTTON)
			u8g2_DrawStr(&u8g2,5,ITEM_HEIGHT*4,"DOWN");
		u8g2_SendBuffer(&u8g2);
		*/
		UI_Update();
		UI_Render();
	}
	Flash_Save();
	UI_Menu=MONITOR;
	/* UI PRESETS END */
	
	
	HAL_UART_Receive_IT(&huart2,&USART_RX_BYTE,1);
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); //��NVIC�жϿ������йر�EXTI12�ж�
  //MPU6050_Init(); //���ʼ��mpu6050
//  __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //����TIM2��PWM
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //����TIM3 4�ı�����ģʽ
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&USART1_NewData,1);
	
	/*
  OLED_Init(); //OLED�ĳ�ʼ��
  OLED_Clear();//OLED����
  OLED_ShowString(0,1,"pitch:",12,0);
  OLED_ShowString(0,3,"gy:",12,0);
  OLED_ShowString(0,5,"goal v:",12,0);
  OLED_ShowString(0,7,"RX:",12,0);
	*/

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);//��NVIC�жϿ�����������EXTI12�ж�
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(USART_RX_FLG)
		{
			//sscanf(USART_RX_BUF,"%f",&bias_error);
			if(USART_RX_CNT==7)
			{
				f_ptr = (unsigned char*)&bias_error;
				sscanf(USART_RX_BUF,"\x2c\x12%c%c%c%c\x5b",f_ptr,f_ptr+1,f_ptr+2,f_ptr+3);
			}
			
			//sprintf(USART_RX_STR,"%f",bias_error);
			strcpy(USART_RX_STR,USART_RX_BUF);
			USART_RX_CNT=0;
			USART_RX_FLG=0;
		}
		UI_Update();
		UI_Render();
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(USART1_RX_STA&0x8000)              //�ж��жϽ��ձ�־λ������ģ��ʹ��USART1��
      {
    	 if((USART1_RX_STA&0x7FFF) ==3 	//�жϽ�������3��                    
    			&& USART1_RX_BUF[0]==0xA5 	//�жϽ��յ�1�������ǲ��ǰ�ͷ0xA5
    			&& USART1_RX_BUF[2]==(USART1_RX_BUF[1])%0x100)	//�жϽ���У�����ǲ���ԭ����֮�͵ĵ�8λ
 	
    	{
    		 switch(USART1_RX_BUF[1])      //���ղ���ȡ�������͹����ĵ�2������
    		 {
    		 case(0x01):Mechanical_velocity=-15;break;
    		 case(0x02):Mechanical_velocity=15;break;
    		 case(0x03):turn_speed=15;break;
    		 case(0x04):turn_speed=-15;break;
    		 case(0x00):Mechanical_velocity=0,turn_speed=0;break;
    		 default:break;

    		 }
    	 }
         USART1_RX_STA=0;//��־λ��0��׼���´ν���
      }
			/*
	  OLED_Showdecimal(36,1,pitch,5,2,12, 0); //OLED��ʾС��ֵ
	  OLED_Showdecimal(36,3,gy,5,2,12, 0);
	  OLED_Showdecimal(36,5,Mechanical_velocity,5,2,12, 0);
	  OLED_Showdecimal(36,7,Encoder_Left,5,2,12, 0);
		*/

		//Give_Motor_PWM(2500,2500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
