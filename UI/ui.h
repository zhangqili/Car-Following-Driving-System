/*
 * ui.h
 *
 *  Created on: 2023年5月23日
 *      Author: xq123
 */

#ifndef UI_H_
#define UI_H_

#include "main.h"
#include "display.h"

#define ITEM_HEIGHT 14

#define OK_BUTTON HAL_GPIO_ReadPin(OK_GPIO_Port, OK_Pin)
#define BACK_BUTTON HAL_GPIO_ReadPin(BACK_GPIO_Port, BACK_Pin)
#define UP_BUTTON HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin)
#define DOWN_BUTTON HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin)

#define UI_ITEM_MAX 4

typedef enum
{
    HOME, MENU_PID, MONITOR
} UI_MENU;

typedef enum
{
    PID_LEFT_MOTOR, PID_RIGHT_MOTOR, PID_TURN, PID_DISTANCE
} UI_PID;


typedef struct
{
  uint8_t x1;
  uint8_t y1;
  uint8_t x2;
  uint8_t y2;
} Rectangle;


extern Rectangle cursor;
extern UI_MENU UI_Menu;

extern uint8_t UI_Keys[4];
extern uint8_t UI_Keys1[4];
extern uint8_t UI_Flag;
extern uint8_t OK_Flag;
extern uint8_t BACK_Flag;
extern uint8_t UP_Flag;
extern uint8_t DOWN_Flag;
extern uint8_t PLUS_Flag;
extern uint8_t MINUS_Flag;

extern uint8_t UI_TempStr[16];

void UI_Init();
void UI_Update();
void UI_Render();
void UI_Menu_Home();
void UI_Menu_Menu();
void UI_Menu_Oscilloscope();
void UI_DrawChart(u8g2_t * u8g2_ptr, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, uint8_t *list, uint8_t length, uint8_t begin, uint8_t max);
void UI_DrawScatterPlot(u8g2_t * u8g2_ptr, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, uint8_t *list, uint8_t length, uint8_t begin, uint8_t max);
void UI_Menu_PID();
void UI_Menu_Menu();
void UI_Menu_Home();
void UI_Menu_Monitor();

#endif /* UI_H_ */
