#include "callbacks.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM9)
    {
        PID_Update();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			HCSR04_SetTime(htim);
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			ENCODER_CountUpL();
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			ENCODER_CountUpR();
		}
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Callback(huart);
}
