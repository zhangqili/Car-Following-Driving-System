#include "flash.h"
#include <stdio.h>

#include "exti.h"
#include "string.h"
#include "string.h"
#include "display.h"
#include "stdlib.h"
#include "stdio.h"
#include "ui.h"
#include "pid_control.h"
#include "motor_control.h"

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PAGEError = 0;
uint32_t Flash_Buffer[16] = {0};
 /**********************************************************************************
  * 函数功能: 页擦除
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无 
  */
void Flash_Erase(void)
{  	
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / STM_SECTOR_SIZE;
    
     if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)  
	 {
		 HAL_FLASH_Lock();  
		 //printf(" Error...1\r\n");
         Error_Handler( );
	 }
}
 /**********************************************************************************
  * 函数功能: 数据写入
  * 输入参数: 写入数据缓存数组指针、写入数据数
  * 返 回 值: 无
  * 说    明：无 
  */    
void Flash_Write(uint32_t *pBuffer,uint32_t  NumToWrite)
{
 
    uint16_t  i=0;
    uint32_t Address = FLASH_USER_START_ADDR;
    HAL_FLASH_Unlock();	    //解锁
    Flash_Erase( );         //先擦除
                            //再写入
       //printf(" 擦除完成，准备写入......\r\n");
     while ( (Address < FLASH_USER_END_ADDR) && (i<NumToWrite)  )    
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, pBuffer[i]) == HAL_OK)
        {
            Address = Address + 4;  //地址后移4个字节
            i++;
        }
        else
		{  
			//printf(" Error...2\r\n"); 
            Error_Handler( );            
		}
    }
  
    HAL_FLASH_Lock();   //上锁
 
 
}
 
 /**********************************************************************************
  * 函数功能: 数据读取
  * 输入参数: 读取数据缓存数组指针、读出数据数
  * 返 回 值: 无
  * 说    明：无
  */
void Flash_Read(uint32_t  *pBuffer,uint32_t  NumToRead)
{
    uint16_t  i=0;
    uint32_t Address = FLASH_USER_START_ADDR;
    
    while ( (Address < FLASH_USER_END_ADDR) && (i<NumToRead)  )
  {
    pBuffer[i++]= *(__IO uint32_t *)Address;  
    Address = Address + 4;   //地址后移4个字节
  }
  
}

void Flash_Recovery(void)
{  	
	Flash_Read(Flash_Buffer,10);
	motor_pid_l.pGain=Flash_Buffer[0];
	motor_pid_l.iGain=Flash_Buffer[1];
	motor_pid_l.dGain=Flash_Buffer[2];
	motor_pid_r.pGain=Flash_Buffer[3];
	motor_pid_r.iGain=Flash_Buffer[4];
	motor_pid_r.dGain=Flash_Buffer[5];
	Turn.pGain=Flash_Buffer[6];
	Turn.iGain=Flash_Buffer[7];
	Turn.dGain=Flash_Buffer[8];
	UI_Interval=Flash_Buffer[9];
}

void Flash_Save(void)
{  	
	Flash_Buffer[0]=motor_pid_l.pGain;
	Flash_Buffer[1]=motor_pid_l.iGain;
	Flash_Buffer[2]=motor_pid_l.dGain;
	Flash_Buffer[3]=motor_pid_r.pGain;
	Flash_Buffer[4]=motor_pid_r.iGain;
	Flash_Buffer[5]=motor_pid_r.dGain;
	Flash_Buffer[6]=Turn.pGain;
	Flash_Buffer[7]=Turn.iGain;
	Flash_Buffer[8]=Turn.dGain;
	Flash_Buffer[9]=UI_Interval;
	Flash_Write(Flash_Buffer,10);
}
