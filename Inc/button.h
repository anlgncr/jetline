#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BUTTON_COUNT 3

enum ButtonId {
    A = 0,
    B = 1,
	C = 2
};

typedef enum {
    BUTTON_IDLE = 0,
    BUTTON_DEBOUNCE,
    BUTTON_PRESSED,
    BUTTON_LONG_PRESSED
} ButtonState_t;

typedef struct Button Button;

typedef void (*ButtonCallback)();

struct Button {
	GPIO_TypeDef* port;
	uint16_t pin;

	ButtonState_t state;
	uint32_t lastTick;
	uint32_t pressTick;

    ButtonCallback onDown;
    ButtonCallback onUp;
    ButtonCallback onLong;
};

void BUTTON_Init();
void BUTTON_Update();

extern Button buttons[BUTTON_COUNT];

#endif
