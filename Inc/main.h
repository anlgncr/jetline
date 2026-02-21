/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRIGGER_Pin GPIO_PIN_13
#define TRIGGER_GPIO_Port GPIOC
#define USER_BUTTON_Pin GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_2
#define LED_YELLOW_GPIO_Port GPIOB
#define BUTTON_B_Pin GPIO_PIN_12
#define BUTTON_B_GPIO_Port GPIOB
#define BUTTON_A_Pin GPIO_PIN_13
#define BUTTON_A_GPIO_Port GPIOB
#define MOTOR_STANDBY_Pin GPIO_PIN_14
#define MOTOR_STANDBY_GPIO_Port GPIOB
#define QTR_LED_ON_Pin GPIO_PIN_15
#define QTR_LED_ON_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define ECHO_Pin GPIO_PIN_15
#define ECHO_GPIO_Port GPIOA
#define MotorB_PWM_Pin GPIO_PIN_4
#define MotorB_PWM_GPIO_Port GPIOB
#define MotorA_PWM_Pin GPIO_PIN_5
#define MotorA_PWM_GPIO_Port GPIOB
#define MotorA_IN1_Pin GPIO_PIN_6
#define MotorA_IN1_GPIO_Port GPIOB
#define MotorA_IN2_Pin GPIO_PIN_7
#define MotorA_IN2_GPIO_Port GPIOB
#define MotorB_IN1_Pin GPIO_PIN_8
#define MotorB_IN1_GPIO_Port GPIOB
#define MotorB_IN2_Pin GPIO_PIN_9
#define MotorB_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/*
typedef enum {
	MODE_IDLE,
    MODE_CALIBRATION,
    MODE_RACE,
	MODE_TEST_DRIVE,
	MODE_MENU,
	MODE_P,
	MODE_D,
	MODE_SPEED,
	MODE_SAVE
} SystemMode;

void setMode(SystemMode mode);
void setMenuEffect();
void setMenuIndexLED();
*/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
