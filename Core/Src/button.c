#include "button.h"

#define DEBOUNCE_TIME   40
#define LONG_PRESS_TIME 500
#define BUTTON_ACTIVE_LEVEL 0

Button buttons[BUTTON_COUNT];

void BUTTON_Set(Button* btn, GPIO_TypeDef* port, uint16_t pin){
    btn->port = port;
    btn->pin = pin;

    btn->state = BUTTON_IDLE;
    btn->lastTick = 0;
    btn->pressTick = 0;
}

void BUTTON_UpdateState(Button *btn){
	uint8_t rawState = HAL_GPIO_ReadPin(btn->port, btn->pin);
	uint32_t now = HAL_GetTick();

	switch(btn->state)
	{
		case BUTTON_IDLE:
			if(rawState == BUTTON_ACTIVE_LEVEL){
				btn->lastTick = now;
				btn->state = BUTTON_DEBOUNCE;
			}
			break;

		case BUTTON_DEBOUNCE:
			if((now - btn->lastTick) >= DEBOUNCE_TIME){

				if(rawState == BUTTON_ACTIVE_LEVEL){
					btn->pressTick = now;
					btn->state = BUTTON_PRESSED;
					if(btn->onDown) btn->onDown();
				}
				else{
					btn->state = BUTTON_IDLE;
				}
			}
			break;

		case BUTTON_PRESSED:
			if(rawState == BUTTON_ACTIVE_LEVEL){

			    if((now - btn->pressTick) >= LONG_PRESS_TIME){
			        if(btn->onLong) btn->onLong();
			        btn->state = BUTTON_LONG_PRESSED;
			    }
			}
			else{
				if(btn->onUp) btn->onUp();
			    btn->state = BUTTON_IDLE;
			}

			break;

		case BUTTON_LONG_PRESSED:
			if(rawState != BUTTON_ACTIVE_LEVEL){
				btn->state = BUTTON_IDLE;
			}
			break;
	}
}

void BUTTON_Init(){
	BUTTON_Set(&buttons[0], GPIOB, GPIO_PIN_13);
	BUTTON_Set(&buttons[1], GPIOB, GPIO_PIN_12);
	BUTTON_Set(&buttons[2], GPIOA, GPIO_PIN_0);
}

void BUTTON_Update(){
	for(uint8_t i = 0; i < BUTTON_COUNT; i++){
		BUTTON_UpdateState(buttons + i);
	}
}


