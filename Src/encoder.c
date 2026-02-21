#include "encoder.h"

static uint16_t counterL = 0;
static uint16_t counterR = 0;

//14 magnets
uint16_t ENCODER_GetCount(){
	return (counterL + counterR) / 2;
}

uint16_t ENCODER_GetCountL(){
	return counterL;
}

uint16_t ENCODER_GetCountR(){
	return counterR;
}

uint16_t ENCODER_GetDistance(){
	return (counterL + counterR) * 0.718; //cm
}

uint16_t ENCODER_GetCountFromDistance(uint16_t cm){
	return cm * 0.696f;
}

uint16_t ENCODER_GetDistanceFromCount(uint16_t count){
	return count * 1.436f;
}

uint16_t ENCODER_GetDistanceL(){
	return counterL * 1.436f; //cm
}

uint16_t ENCODER_GetDistanceR(){
	return counterR * 1.436f; //cm
}

void ENCODER_CountUpL(){
	counterL++;
	//LED_AnimateOnce(GREEN, SHORT_BLINK);
}

void ENCODER_CountUpR(){
	counterR++;
	//LED_AnimateOnce(YELLOW, SHORT_BLINK);
}


void ENCODER_CountDownL(){
	counterL--;
}

void ENCODER_CountDownR(){
	counterR--;
}

void ENCODER_Reset(){
	counterL = 0;
	counterR = 0;
}
