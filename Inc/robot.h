#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "string.h"
#include "motors.h"
#include "hcsr04.h"
#include "qtr8a.h"
#include "callbacks.h"
#include "led.h"
#include "flash_wl.h"
#include "pid.h"
#include "encoder.h"
#include "menu.h"
#include "robot.h"
#include "uart_ring.h"
#include "uart_cmd.h"


void ROBOT_Init(ADC_HandleTypeDef *, TIM_HandleTypeDef *);
void ROBOT_Update();

void ROBOT_Stop();
void ROBOT_CheckForRacing();

#endif
