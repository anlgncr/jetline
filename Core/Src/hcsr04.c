#include "hcsr04.h"
#include "main.h"

#define MAX_DISTANCE       200
#define TRIGGER_INTERVAL   30      // ms
#define TIMEOUT_MS         30      // ms

typedef enum {
    HCSR04_IDLE,
    HCSR04_WAIT_ECHO
} HCSR04_State;

typedef struct {
    TIM_HandleTypeDef *htim;
    volatile uint32_t rising;
    volatile uint32_t falling;
    volatile uint8_t capture_done;
    uint32_t trigger_tick;
    uint16_t distance;
    HCSR04_State state;
    uint8_t edge;

} HCSR04_t;

static HCSR04_t hc;

static inline void delay_us(uint16_t us){
    uint16_t start = TIM9->CNT;
    while ((uint16_t)(TIM9->CNT - start) < us);
}

void HCSR04_Init(TIM_HandleTypeDef *htim){
    hc.htim = htim;
    HCSR04_ResetState();
}

void HCSR04_ResetState(){
	hc.state = HCSR04_IDLE;
	hc.distance = MAX_DISTANCE;
	hc.capture_done = 0;
	hc.edge = 0;

	__HAL_TIM_SET_CAPTUREPOLARITY(hc.htim,
	            TIM_CHANNEL_1,
	            TIM_INPUTCHANNELPOLARITY_FALLING);
}

void HCSR04_Update(void)
{
    uint32_t now = HAL_GetTick();

    switch(hc.state)
    {
        case HCSR04_IDLE:

            if(now - hc.trigger_tick >= TRIGGER_INTERVAL){

                // 🔥 trigger pulse
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                delay_us(10);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

                hc.capture_done = 0;
                hc.trigger_tick = now;

                hc.state = HCSR04_WAIT_ECHO;
            }
            break;

        case HCSR04_WAIT_ECHO:

            // ✅ echo geldi
            if(hc.capture_done){

                uint32_t diff = hc.falling - hc.rising;
                uint16_t dist = diff / 58;

                hc.distance = (dist > MAX_DISTANCE) ? MAX_DISTANCE : dist;

                hc.state = HCSR04_IDLE;
            }

            // ❌ echo gelmedi (timeout)
            else if(now - hc.trigger_tick > TIMEOUT_MS){
            	HCSR04_ResetState();
            }

            break;
    }
}

uint16_t HCSR04_GetDistance(void)
{
    return hc.distance;
}


void HCSR04_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim != hc.htim) return;

    uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if(hc.edge == 0){
        hc.rising = val;
        hc.edge = 1;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
            TIM_CHANNEL_1,
            TIM_INPUTCHANNELPOLARITY_RISING);
    }
    else{
        hc.falling = val;
        hc.edge = 0;
        hc.capture_done = 1;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
            TIM_CHANNEL_1,
            TIM_INPUTCHANNELPOLARITY_FALLING);
    }
}
