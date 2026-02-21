#ifndef LED_H
#define LED_H

#include "stm32f4xx_hal.h"

#define GREEN_LED_PORT GPIOB
#define GREEN_LED_PIN  GPIO_PIN_1

#define YELLOW_LED_PORT GPIOB
#define YELLOW_LED_PIN  GPIO_PIN_2

#define BLUE_LED_PORT GPIOC
#define BLUE_LED_PIN  GPIO_PIN_13

#define BUZZER_PORT GPIOA
#define BUZZER_PIN  GPIO_PIN_8

typedef struct {
    uint16_t on_ms;
    uint16_t off_ms;
} LedStep_t;


enum LedIndex {
    GREEN = 0,
    YELLOW = 1,
	BLUE = 2,
	BUZZER = 3
};

typedef enum {
    ST_OFF = 0,
    ST_ON,
    ST_PAUSE
} LedState;

typedef enum {
    BLINK,
	SHORT_BLINK,
	LONG_BLINK,
    HEARTBEAT,
	HEARTBEATREV,
	BURST,
	BUZZER_INIT,
	BUZZER_CONFIRM,
	BUZZER_ERROR,
	BUZZER_FINISH
} Anim_t;


typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;

    const LedStep_t *pattern;
    uint8_t stepIndex;
    uint8_t repeatCount;
    uint8_t currentRepeat;
    uint32_t startTick;

    uint8_t state;  // 0 = OFF, 1 = ON
    uint8_t active;
} Led_t;


void LED_Init();
void LED_On(uint8_t i);
void LED_Off(uint8_t i);
void LED_Animate(uint8_t i, Anim_t anim);
void LED_AnimateOnce(uint8_t i, Anim_t anim);
void LED_AnimateFor(uint8_t i, Anim_t anim, uint16_t repeatCount);
void LED_SetPattern(uint8_t i, const LedStep_t *pattern, uint8_t repeatCount);
void LED_Update(void);



#endif
