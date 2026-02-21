#ifndef PID_H
#define PID_H

#include "main.h"
#include <stdbool.h>
#include "qtr8a.h"
#include "led.h"
#include "motors.h"
#include "encoder.h"
#include <stdbool.h>
#include "robot.h"

typedef enum{
	LINE_LEFT = 0,
	LINE_RIGHT = 1,
} LineSide;

typedef enum{
	PIST_A = 0,
	PIST_B = 1
} Pist;

void PID_Init();
int16_t PID_GetError();
uint16_t* PID_GetPIDPacket(void);

void PID_Update(void);
void PID_IncreaseKp();
void PID_DecreaseKp();
void PID_IncreaseKd();
void PID_DecreaseKd();
void PID_IncreaseSpeed();
void PID_DecreaseSpeed();
void PID_SetPoint(uint8_t index, uint16_t point);


void PID_SetP(uint16_t p);
void PID_SetI(uint16_t i);
void PID_SetD(uint16_t d);

void PID_TestDriveOn();
void PID_TestDriveOff();

void PID_GetString(uint8_t* str, size_t str_size);

void PID_SetBaseSpeed(uint16_t speed);
void PID_Enable();
void PID_Disable();

void PID_LoadFromRecord();
void PID_WriteToRecord(WL_Record *rec);

void PID_SetLineWhite();
void PID_SetLineBlack();
void PID_SetLineAuto();
void PID_SetCornerMode(bool val);

void PID_SetCornerSpeed(uint16_t speed);
void PID_SetTurningSpeed(uint16_t speed);
void PID_SetPist(Pist p);
void PID_SetPistCorners(uint16_t* cornerArray, size_t len);

uint8_t PID_GetPistCorners(uint16_t* buffer);

static inline float constrainf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

extern volatile bool pid_enabled;

#endif
