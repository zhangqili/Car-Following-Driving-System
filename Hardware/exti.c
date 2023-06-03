/*
 * exti.c
 *
 *  Created on: Aug 17, 2022
 *      Author: HP
 */

#include "exti.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "pid_control.h"
#include "usart.h"

uint8_t USART1_RX_BUF[USART1_REC_LEN];//接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART1_RX_STA=0;//接收状态标记//bit15：接收完成标志，bit14~0：接收到的有效字节数目
uint8_t USART1_NewData;//当前串口中断接收的1个字节数据的缓存
int count=0;
void  HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart)//串口中断回调函数
{
	if(huart ==&huart1)
	{
    if((USART1_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART1_NewData==0x5A)//接收到了0x5A
            {
                 USART1_RX_STA|=0x8000;   //接收完成了，将USART2_RX_STA中的bit15（15位）置1
            }
            else
            {
                   USART1_RX_BUF[USART1_RX_STA&0X7FFF]=USART1_NewData; /*将收到的数据放入数组，
                   例如按下按键1（前进）：
                   USART2_RX_BUF[0]=0xA5
                   USART2_RX_BUF[1]=0x01
                   USART2_RX_BUF[2]=0x01
                   虽然蓝牙模块发送的数据包有4个字节但是包尾0x5A不会存
    	           入USART2_RX_BUF中，当单片机接收到包尾的0x5A时会将USART2_RX_STA的最高位置1*/
                   USART1_RX_STA++;  //数据长度计数加1
                   if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收
            }
        }
			HAL_UART_Receive_IT(&huart1,(uint8_t *)&USART1_NewData,1); //因为每执行完一次中断回调函数会将接收中断功能关闭，所以最后需要再开启接收中断  
   }
     
}


//************************************************//

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//在inv_mpu.h设置的mpu6050采样率为100Hz,即10ms进入一次外部中断
//{
////	switch(GPIO_Pin)
////	{
////		case INT_Pin :

////		   MPU_DMP_Get_Data(&pitch,&roll,&yaw);//得到俯仰角pitch
////		   MPU_Get_Gyroscope(&gx,&gy,&gz);    //得到y轴加速度值
////		   Get_Encoder(); //得到左右编码器计数值即左右电机转速
////		   balance_up=Up_balance(pitch,gy,Mechanical_Angle); //直立环
////		   velocity=Velocity( Encoder_Left,Encoder_Right,Mechanical_velocity); //速度环
////		   turn_out=Turn_out(yaw,turn_speed);
////		   PWMA= balance_up+velocity-turn_out; //并联直立环与速度环
////		   PWMB= balance_up+velocity+turn_out; 
////		   Limit(&PWMA,&PWMB);        //PWM限幅
////		   if(pitch<-30||pitch>30)    //检查俯仰角的角度值，这里增加这个判断的目的是：当小车失去平衡后立即关闭电机
////		   {
////			   PWMA=0;
////			   PWMB=0;
////		   }
////		   Give_Motor_PWM(PWMA,PWMB); //赋值给PWM寄存器以及控制电机正反转

//////		 printf("pitch:%f\t,roll:%f\t,yaw:%f\r\n",pitch,roll,yaw);测试用的

////			break;
////		default:
////			break;
////  }
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM1)
	{	
		if(count>=500)
		{
			Turn.pGain=23;
			Turn.dGain=30;
			motor_pid_l.pGain=150;
			motor_pid_l.iGain=14;
			motor_pid_r.pGain=150;
			motor_pid_r.iGain=14;
			Track(15);
		}
		count++;
	}
			//send_dataF(Turn.errdat,speed_l,speed_r,motor_l.Encoder,motor_r.Encoder);
}
