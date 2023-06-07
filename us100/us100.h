


#ifndef US100_H_
#define US100_H_

#include "main.h"

#define US100_Distance (US100_DistanceBuffer[0]*256+US100_DistanceBuffer[1])

extern uint8_t US100_DistanceBuffer[2];
extern uint8_t US100_Temperature;
//extern uint16_t US100_Distance;

void US100_GetDistance();
void US100_GetTemperature();

#endif /* DISPLAY_H_ */
