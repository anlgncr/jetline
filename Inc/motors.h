#ifndef MOTORS_H
#define MOTORS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim3;

typedef enum {
    DIR_LEFT,
    DIR_RIGHT,
    DIR_FORWARD,
    DIR_BACKWARD
} Direction;

typedef enum {
    MT_LEFT,
	MT_RIGHT
} MotorId;

void MOTOR_Init();
void MOTOR_DriverOn();
void MOTOR_DriverOff();
void MOTOR_GoForward(int16_t speed, uint32_t dur);
void MOTOR_GoBackward(int16_t speed, uint32_t dur);
void MOTOR_TurnRight(int16_t speed, uint32_t dur);
void MOTOR_TurnLeft(int16_t speed, uint32_t dur);
void MOTOR_Set(uint8_t dir, int16_t speed, uint32_t dur);
void MOTOR_Update();
void MOTOR_SetSpeed(MotorId id, int speed);
void MOTOR_TurnLeftAndRight();
void MOTOR_Stop();

#endif
