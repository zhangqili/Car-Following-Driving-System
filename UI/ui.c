/*
 * ui.c
 *
 *  Created on: 2023年5月23日
 *      Author: xq123
 */
#include "main.h"
#include "ui.h"
#include "stdlib.h"
#include "stdio.h"
#include "pid_control.h"
#include "mpu6050.h"
#include "motor_control.h"
#include "usart.h"

Rectangle cursor={0,0,0};
UI_MENU UI_Menu=HOME;
UI_PID UI_pid=0;
int8_t UI_Selection = 0;
uint8_t UI_TempStr[16];
float UI_Interval=1;

uint8_t UI_Keys[4]={1};
uint8_t UI_Keys1[4]={1};
uint8_t UI_Flag=1;
uint8_t OK_Flag=0;
uint8_t BACK_Flag=0;
uint8_t UP_Flag=0;
uint8_t DOWN_Flag=0;
uint8_t PLUS_Flag=0;
uint8_t MINUS_Flag=0;

uint8_t UI_Alive_Flag=0;

PID* UI_PID_t;
//uint8_t home_str[][]={"Left Motor","Right Motor","Motor",};



void UI_Update()
{
    UI_Keys1[0]=UI_Keys[0];
    UI_Keys1[1]=UI_Keys[1];
    UI_Keys1[2]=UI_Keys[2];
    UI_Keys1[3]=UI_Keys[3];
    UI_Keys[0]=BACK_BUTTON;
    UI_Keys[1]=OK_BUTTON;
    UI_Keys[2]=UP_BUTTON;
    UI_Keys[3]=DOWN_BUTTON;
    if(UI_Keys[0]&&(UI_Keys1[0]!=UI_Keys[0]))
        BACK_Flag=1;
    if(UI_Keys[1]&&(UI_Keys1[1]!=UI_Keys[1]))
        OK_Flag=1;
    if((!UI_Keys[1])&&UI_Keys[2]&&(UI_Keys1[2]!=UI_Keys[2]))
        PLUS_Flag=1;
    if((!UI_Keys[1])&&UI_Keys[3]&&(UI_Keys1[3]!=UI_Keys[3]))
        MINUS_Flag=1;
    if((!PLUS_Flag)&&UI_Keys[2]&&(UI_Keys1[2]!=UI_Keys[2]))
        UP_Flag=1;
    if((!MINUS_Flag)&&UI_Keys[3]&&(UI_Keys1[3]!=UI_Keys[3]))
        DOWN_Flag=1;

    if(UP_Flag)
    {
        UP_Flag=0;
        UI_Selection++;
        if(UI_Selection>=UI_ITEM_MAX)
        {
            UI_Selection=0;
        }
    }
    if(DOWN_Flag)
    {
        DOWN_Flag=0;
        UI_Selection--;
        if(UI_Selection<0)
        {
            UI_Selection=UI_ITEM_MAX-1;
        }
    }
    switch (UI_Menu)
    {
    case HOME:
        if(OK_Flag)
        {
            OK_Flag=0;
            UI_Menu=MENU_PID;
            UI_pid = UI_Selection;
        }
        if(BACK_Flag)
        {
            BACK_Flag=0;
            UI_Menu=MONITOR;
						UI_Flag=0;
        }
        break;
    case MENU_PID:
				switch (UI_pid)
				{
				case PID_LEFT_MOTOR:
						UI_PID_t = &motor_pid_l;
						break;
				case PID_RIGHT_MOTOR:
						UI_PID_t = &motor_pid_r;
						break;
				case PID_TURN:
						UI_PID_t = &Turn;
						break;
				case PID_DISTANCE:
						UI_PID_t = &Turn;
						break;
				}
        if(BACK_Flag)
        {
            BACK_Flag=0;
            UI_Menu=HOME;
        }
        if(PLUS_Flag)
        {
            PLUS_Flag=0;
						switch(UI_Selection)
						{
							case 0:
								UI_PID_t->pGain+=UI_Interval;
								break;
							case 1:
								UI_PID_t->iGain+=UI_Interval;
								break;
							case 2:
								UI_PID_t->dGain+=UI_Interval;
								break;
							case 3:
								UI_Interval+=1;
								break;
						}
        }
        if(MINUS_Flag)
        {
            MINUS_Flag=0;
						switch(UI_Selection)
						{
							case 0:
								UI_PID_t->pGain-=UI_Interval;
								break;
							case 1:
								UI_PID_t->iGain-=UI_Interval;
								break;
							case 2:
								UI_PID_t->dGain-=UI_Interval;
								break;
							case 3:
								UI_Interval-=1;
								break;
						}
        }
        if(OK_Flag&&UI_Selection==3)
        {
            OK_Flag=0;
						UI_Menu=HOME;
        }
        UI_Menu_PID();
        break;
    default:
        break;
    }
}


void UI_Render()
{
    u8g2_ClearBuffer(&u8g2);

    switch (UI_Menu)
    {
    case HOME:
        UI_Menu_Home();
        break;
    case MENU_PID:
        UI_Menu_PID();
        break;
    case MONITOR:
        UI_Menu_Monitor();
        break;
    }
		if(UI_Menu!=MONITOR)
			u8g2_DrawFrame(&u8g2, 2, UI_Selection*ITEM_HEIGHT, 120, ITEM_HEIGHT+2);
		if(UI_Alive_Flag=!UI_Alive_Flag)
			u8g2_DrawStr(&u8g2, 120, 64,"A");
    u8g2_SendBuffer(&u8g2);
}

void UI_Menu_PID()
{
    sprintf(UI_TempStr,"P:%f",UI_PID_t->pGain);
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT, UI_TempStr);
    sprintf(UI_TempStr,"I:%f",UI_PID_t->iGain);
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT*2, UI_TempStr);
    sprintf(UI_TempStr,"D:%f",UI_PID_t->dGain);
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT*3, UI_TempStr);
    sprintf(UI_TempStr,"Interval:%f",UI_Interval);
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT*4, UI_TempStr);
}

void UI_Menu_Menu()
{

}

void UI_Menu_Home()
{
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT, "Left Motor");
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT*2, "Right Motor");
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT*3, "Turn");
    u8g2_DrawStr(&u8g2, 5, ITEM_HEIGHT*4, USART_RX_STR);
}


void UI_Menu_Monitor()
{

	  sprintf(UI_TempStr,"L:%d",motor_l.Encoder);
    u8g2_DrawStr(&u8g2, 0, ITEM_HEIGHT, UI_TempStr);
    sprintf(UI_TempStr,"R:%d",motor_r.Encoder);
    u8g2_DrawStr(&u8g2, 64, ITEM_HEIGHT, UI_TempStr);
    sprintf(UI_TempStr,"b_err:%f",bias_error);
    u8g2_DrawStr(&u8g2, 0, ITEM_HEIGHT*2, UI_TempStr);
    sprintf(UI_TempStr,"out:%f",Turn.pidout);
    u8g2_DrawStr(&u8g2, 0, ITEM_HEIGHT*3, UI_TempStr);
    sprintf(UI_TempStr,"err:%f",Turn.errdat);
    u8g2_DrawStr(&u8g2, 0, ITEM_HEIGHT*4, UI_TempStr);
    sprintf(UI_TempStr,"l_e:%d",speed_l);
    u8g2_DrawStr(&u8g2, 0, ITEM_HEIGHT*5, UI_TempStr);
    sprintf(UI_TempStr,"r_e:%d",speed_r);
    u8g2_DrawStr(&u8g2, 64, ITEM_HEIGHT*5, UI_TempStr);
}

/*
void UI_MPU6050()
{
    sprintf(UI_TempStr,"Ax:%3.2lf\n",MPU5050_Data.Ax);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 0, 12, UI_TempStr);
    sprintf(UI_TempStr,"Ay:%3.2lf\n",MPU5050_Data.Ay);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 0, 24, UI_TempStr);
    sprintf(UI_TempStr,"Az:%3.2lf\n",MPU5050_Data.Az);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 0, 36, UI_TempStr);
    sprintf(UI_TempStr,"Gx:%3.2lf\n",MPU5050_Data.Gx);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 64, 12, UI_TempStr);
    sprintf(UI_TempStr,"Gy:%3.2lf\n",MPU5050_Data.Gy);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 64, 24, UI_TempStr);
    sprintf(UI_TempStr,"Gz:%3.2lf\n",MPU5050_Data.Gz);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 64, 36, UI_TempStr);
    sprintf(UI_TempStr,"Kx:%3.2lf\n",MPU5050_Data.KalmanAngleX);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 64, 48, UI_TempStr);
    sprintf(UI_TempStr,"Ky:%3.2lf\n",MPU5050_Data.KalmanAngleY);
    printf(UI_TempStr);
    u8g2_DrawStr(&u8g2, 64, 60, UI_TempStr);
}
*/

void UI_DrawChart(u8g2_t * u8g2_ptr, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, uint8_t *list, uint8_t length, uint8_t begin, uint8_t max)
{
    int8_t index=0;
    float y1;
    float y2;
    float ymax = (float)max;
    for (int8_t i=0;i<w-1;i++)
    {
        index = begin-i;
        if(index>0)
        {
          y1=list[index];
          y2=list[index-1];
        }
        if(index==0)
        {
          y1=list[index];
          y2=list[length-1];
        }
        if(index<0)
        {
          y1=list[length+index];
          y2=list[length+index-1];
        }
        if(y1>ymax || y2>ymax)
          u8g2_DrawLine(u8g2_ptr,x+w-i,ymax,x+w-i-1,ymax);
        else
          u8g2_DrawLine(u8g2_ptr,
            x+w-i-2,h+y-(uint8_t)((float)(h-1)*(y1/ymax)),
            x+w-i-3,h+y-(uint8_t)((float)(h-1)*(y2/ymax)));
      }
}

void UI_DrawScatterPlot(u8g2_t * u8g2_ptr, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, uint8_t *list, uint8_t length, uint8_t begin, uint8_t max)
{
    int8_t index=0;
    uint8_t t;
    for (int8_t i=0;i<w-1;i++)
    {
        index = begin-i;
        if(index>0)
        {
          t=list[index];
        }
        if(index==0)
        {
         t=list[index];
        }
        if(index<0)
        {
          t=list[length+index];
        }
        if(t>h)
          u8g2_DrawPixel(u8g2_ptr, x+w-i-1, y+1);
        else
          u8g2_DrawPixel(u8g2_ptr, x+w-i-1, y+t);
      }
}


