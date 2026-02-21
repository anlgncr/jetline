#include "led.h"

#define MAX_LEDS 4

static Led_t leds[MAX_LEDS];

const LedStep_t PATTERN_INIT[] = {
	{90, 40},   // dıt
	{60, 25},   // dı
	{60, 35},   // rı  (dirı = iki kısa ton)
	{0, 0}
};

const LedStep_t PATTERN_BLINK[] = {
    {150, 150}, {0, 0}
};

const LedStep_t PATTERN_SHORT_BLINK[] = {
    {10, 100}, {0, 0}
};

const LedStep_t PATTERN_LONG_BLINK[] = {
    {200, 200}, {0, 0}
};


// HEARTBEAT: 2 kısa blink + uzun pause
const LedStep_t PATTERN_HEARTBEAT[] = {
    {80, 120}, {80, 120}, {0, 800}, {0, 0}
};

const LedStep_t PATTERN_HEARTBEAT_REV[] = {
	{0, 800}, {80, 120}, {80, 120}, {0, 0}
};
// BURST: 3 hızlı blink
const LedStep_t PATTERN_BURST[] = {
    {50, 50}, {50, 50}, {50, 200}, {0, 0}
};
// ERROR CODE 3: 3 kısa blink
const LedStep_t PATTERN_ERROR[] = {
	{70, 40},   // dıt
	{200, 60},  // dıııt
	{0, 0}
};

const LedStep_t PATTERN_CONFIRM[] = {
	{70, 30},   // dıt
	{50, 20},   // dı
	{110, 40},  // rı  (hızlı yükselme etkisi)
	{0, 0}
};


const LedStep_t PATTERN_FINISH[] = {
	{50, 100},   // dıt
	{100, 30},   // dıııı
	{200, 30},  //  dııııııı
	{50, 0},  //  dıt
	{0, 0}
};

// ***-----****--****--***


void LED_Init(){
	leds[0].port = GREEN_LED_PORT;
	leds[0].pin = GREEN_LED_PIN;

	leds[1].port = YELLOW_LED_PORT;
	leds[1].pin = YELLOW_LED_PIN;

	leds[2].port = BLUE_LED_PORT;
	leds[2].pin = BLUE_LED_PIN;

	leds[3].port = BUZZER_PORT;
	leds[3].pin = BUZZER_PIN;
}

void LED_Animate(uint8_t i, Anim_t anim){
	 LED_AnimateFor(i, anim, 0);
}

void LED_AnimateOnce(uint8_t i, Anim_t anim){
	LED_AnimateFor(i, anim, 1);
}

void LED_On(uint8_t i){
	if(i >= MAX_LEDS) return;
	HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_SET);
	leds[i].active = 0;
}

void LED_Off(uint8_t i){
	if(i >= MAX_LEDS) return;
	HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_RESET);
	leds[i].active = 0;
}

void LED_AnimateFor(uint8_t i, Anim_t anim, uint16_t repeatCount){
	if(anim == BLINK){
		LED_SetPattern(i, PATTERN_BLINK, repeatCount);
	}
	if(anim == SHORT_BLINK){
		LED_SetPattern(i, PATTERN_SHORT_BLINK, repeatCount);
	}
	else if(anim == LONG_BLINK){
		LED_SetPattern(i, PATTERN_LONG_BLINK, repeatCount);
	}
	else if(anim == HEARTBEAT){
		LED_SetPattern(i, PATTERN_HEARTBEAT, repeatCount);
	}
	else if(anim == HEARTBEATREV){
		LED_SetPattern(i, PATTERN_HEARTBEAT_REV, repeatCount);
	}
	else if(anim == BURST){
		LED_SetPattern(i, PATTERN_BURST, repeatCount);
	}
	else if(anim == BUZZER_ERROR){
		LED_SetPattern(i, PATTERN_ERROR, repeatCount);
	}
	else if(anim == BUZZER_INIT){
		LED_SetPattern(i, PATTERN_INIT, repeatCount);
	}
	else if(anim == BUZZER_CONFIRM){
		LED_SetPattern(i, PATTERN_CONFIRM, repeatCount);
	}
	else if(anim == BUZZER_FINISH){
		LED_SetPattern(i, PATTERN_FINISH, repeatCount);
	}
}

void LED_SetPattern(uint8_t i, const LedStep_t *pattern, uint8_t repeatCount)
{
    if(i >= MAX_LEDS) return;

    leds[i].pattern = pattern;
    leds[i].stepIndex = 0;
    leds[i].currentRepeat = 0;
    leds[i].repeatCount = repeatCount;
    leds[i].startTick = HAL_GetTick();
    leds[i].state = 1;
    leds[i].active = 1;

    HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_SET);
}

void LED_Update(void)
{
    uint32_t now = HAL_GetTick();

    for(uint8_t i=0; i<MAX_LEDS; i++)
    {
        if(!leds[i].active || leds[i].pattern == NULL)
            continue;

        LedStep_t step = leds[i].pattern[leds[i].stepIndex];

        // pattern sonu kontrolü
        if(step.on_ms == 0 && step.off_ms == 0)
        {
            leds[i].stepIndex = 0;
            step = leds[i].pattern[0];

            // repeat count kontrolü
            if(leds[i].repeatCount > 0) {
                leds[i].currentRepeat++;
                if(leds[i].currentRepeat >= leds[i].repeatCount) {
                    leds[i].active = 0; // pattern tamamlandı → LED devre dışı
                    HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_RESET);
                    continue;
                }
            }
        }

        uint32_t delta = now - leds[i].startTick;

        if(leds[i].state)  // ON
        {
            if(delta >= step.on_ms)
            {
            	HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_RESET);

                leds[i].state = 0;
                leds[i].startTick = now;
            }
        }
        else // OFF
        {
            if(delta >= step.off_ms)
            {
              	HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_SET);

                leds[i].state = 1;
                leds[i].startTick = now;
                leds[i].stepIndex++; // bir sonraki adım
            }
        }
    }
}

