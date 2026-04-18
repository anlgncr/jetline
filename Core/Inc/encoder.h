#ifndef ENCODER_H
#define ENCODER_H

#include "led.h"

uint16_t ENCODER_GetCount();
uint16_t ENCODER_GetCountL();
uint16_t ENCODER_GetCountR();
uint16_t ENCODER_GetCountFromDistance(uint16_t cm);
uint16_t ENCODER_GetDistanceFromCount(uint16_t count);

uint16_t ENCODER_GetDistance();
uint16_t ENCODER_GetDistanceL();
uint16_t ENCODER_GetDistanceR();

void ENCODER_CountUpL();
void ENCODER_CountUpR();


void ENCODER_CountDownL();
void ENCODER_CountDownR();

void ENCODER_Reset();

#endif
