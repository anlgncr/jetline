#include "hcsr04.h"
#include "main.h"

volatile uint32_t ic_rising = 0;
volatile uint32_t ic_falling = 0;
volatile uint8_t  hc_capture_done = 0;
volatile uint8_t  last_edge_rising = 0;

static uint32_t last_trigger_tick = 0;

static TIM_HandleTypeDef *htim2 = NULL;

void HCSR04_Init(TIM_HandleTypeDef *_htim){
	htim2 = _htim;
	HCSR04_Reset();
}

void delay_us(uint16_t us){
    uint16_t start = TIM9->CNT;
    while ((uint16_t)(TIM9->CNT - start) < us);
}

void HCSR04_Trigger(void)
{
    uint32_t current_tick = HAL_GetTick();
    if(current_tick - last_trigger_tick < 30)
		return;

    if (hc_capture_done == 0){
        HCSR04_Reset();

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        delay_us(10);   // 10us pulse
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

        last_trigger_tick = current_tick;
    }
}

uint16_t HCSR04_GetDistance(void)
{
    if (hc_capture_done){
        hc_capture_done = 0;
        uint32_t diff = ic_falling - ic_rising;   // wrap-safe
        uint16_t distance = (uint16_t)(diff / 58);

        return (distance > 200)? 200 : distance;// us → cm
    }
    return 0;
}

void HCSR04_Reset(){
	ic_rising = 0;
	ic_falling = 0;
	last_edge_rising = 0;
	hc_capture_done = 0;

	__HAL_TIM_SET_CAPTUREPOLARITY(htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
}

void HCSR04_SetTime(TIM_HandleTypeDef *htim)
{
	uint32_t value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (!last_edge_rising){
        ic_rising = value;
        last_edge_rising = 1;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else{
        ic_falling = value;
        last_edge_rising = 0;
        hc_capture_done = 1;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}
