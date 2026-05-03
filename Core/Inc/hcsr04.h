#ifndef __HCSR04_H
#define __HCSR04_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void HCSR04_Init(TIM_HandleTypeDef *htim);

void HCSR04_Update(void);
uint16_t HCSR04_GetDistance(void);
void HCSR04_ResetState();

void HCSR04_CaptureCallback(TIM_HandleTypeDef *htim);

#endif
