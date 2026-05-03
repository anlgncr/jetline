#ifndef PID_H
#define PID_H

#define MAX_SPEED 1000
#define MIN_SPEED 0
#define BRAKE_SPEED -50
#define DT_S 0.002f
#define STABLE_COUNT 5

#define SHARP_LEFT_MASK 0xF1
#define SHARP_RIGHT_MASK 0x8F
#define LINE_LEFT_MASK 0x80
#define LINE_RIGHT_MASK 0x01

#include "main.h"
#include <stdbool.h>
#include "qtr8a.h"
#include "led.h"
#include "motors.h"
#include "encoder.h"
#include <stdbool.h>
#include "robot.h"
#include <math.h>
#include "uart_ring.h"
#include <stdio.h>

typedef enum{
	LINE_LEFT = 0,
	LINE_RIGHT = 1,
} LineSide;

typedef enum{
	PIST_A = 0,
	PIST_B = 1
} Pist;

typedef enum {
    DRIVE_MODE_PID,
    DRIVE_MODE_SEARCH,
	DRIVE_MODE_STRAIGHT
} DriveMode;

typedef enum {
    CORNER_NONE,
    CORNER_LEFT,
    CORNER_RIGHT
} CornerState;


void PID_Init();
int16_t PID_GetError();
uint16_t* PID_GetPIDPacket(void);

void PID_Update(void);
void PID_SetPoint(uint8_t index, uint16_t point);

void PID_SetP(uint16_t p);
void PID_SetD(uint16_t d);

void PID_TestDriveOn();
void PID_TestDriveOff();

void PID_GetString(uint8_t* str, size_t str_size);

void PID_SetBaseSpeed(uint16_t speed);
void PID_Enable();
void PID_Disable();

void PID_LoadFromRecord();
void PID_WriteToRecord(WL_Record *rec);

void PID_SetLineAuto(bool val);
void PID_SetCornerMode(bool val);
void PID_SetAdaptiveSpeed(bool val);

void PID_SetCornerSpeed(uint16_t speed);
void PID_SetTurningSpeed(uint16_t speed);
void PID_SetSlowDownDistance(uint8_t distance); // pulse count
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
