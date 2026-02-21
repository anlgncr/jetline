/* ================= qtr8a.h ================= */
#ifndef QTR8A_H
#define QTR8A_H

#include "stm32f4xx_hal.h"
#include "flash_wl.h"

#define QTR8A_MAX_ERROR 350
#define QTR8A_CHANNEL_COUNT 8

#define LEDON_PORT GPIOB
#define LEDON_PIN GPIO_PIN_15

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint16_t raw[QTR8A_CHANNEL_COUNT];
    uint16_t minVal[QTR8A_CHANNEL_COUNT];
    uint16_t maxVal[QTR8A_CHANNEL_COUNT];
    int16_t norm[QTR8A_CHANNEL_COUNT];
} QTR8A_t;

typedef enum{
	LINE_BLACK = 0,
	LINE_WHITE = 1,
} lineType;

void QTR8A_Init(ADC_HandleTypeDef *hadc);

void QTR8A_LoadFromRecord();
void QTR8A_WriteToRecord(WL_Record *rec);

void QTR8A_GetSensorRaw(uint16_t* buffer);
void QTR8A_Measure();
void QTR8_CalibrateToBuffer();
void QTR8A_Calibrate();
void QTR8A_GetCalibratedString(char* str, size_t str_size);
void QTR8A_GetWeightedString(char* str, size_t str_size);
void QTR8A_Normalize(lineType);
void QTR8A_Reset();
void QTR8A_Ledon();
void QTR8A_Ledoff();
void QTR8A_StartCalibration();
void QTR8A_EndCalibration();
void QTR8A_Update();

int16_t QTR8A_GetPosition();
uint8_t QTR8A_GetLinePattern();
uint8_t QTR8A_GetWeight();

#endif
