/* hc_sr04.h */

#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f4xx_hal.h"

void HCSR04_Init();
void HCSR04_Trigger(void);
uint16_t HCSR04_GetDistance(void);
void delay_us(uint16_t);
void HCSR04_SetTime(TIM_HandleTypeDef *htim);
void HCSR04_Reset();

extern volatile uint8_t hc_capture_done;

#endif
