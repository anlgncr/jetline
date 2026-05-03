#ifndef ENCODER_H
#define ENCODER_H

#include "led.h"

uint32_t ENCODER_GetCount();
uint32_t ENCODER_GetCountL();
uint32_t ENCODER_GetCountR();
uint32_t ENCODER_GetCountFromDistance(uint32_t cm);
uint32_t ENCODER_GetDistanceFromCount(uint32_t count);

uint32_t ENCODER_GetDistance();
uint32_t ENCODER_GetDistanceL();
uint32_t ENCODER_GetDistanceR();

void ENCODER_CountUpL();
void ENCODER_CountUpR();


void ENCODER_CountDownL();
void ENCODER_CountDownR();

void ENCODER_Reset();

#endif
