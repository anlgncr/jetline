#include "motors.h"
#define MAX_SPEED 1000
#define CALIBRATING_SPEED 250

typedef enum {
	MOTOR_STATE_WAIT,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_TURN_LEFT_90,
    MOTOR_STATE_TURN_RIGHT_90,
	MOTOR_STATE_TURN_LEFT_180,
	MOTOR_STATE_TURN_RIGHT_180,
	MOTOR_STATE_STOP
} MotorState_t;

MotorState_t motorState = MOTOR_STATE_IDLE;

typedef void (*CallbackF)(void);
CallbackF callbackf = NULL;

static uint32_t duration;
static uint32_t startTime;
static uint8_t turnCount = 0;

void MOTOR_Init(){
	MOTOR_DriverOff();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void MOTOR_DriverOn(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
}

void MOTOR_DriverOff(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
}

void MOTOR_GoForward(int16_t speed, uint32_t dur){
	MOTOR_Set(DIR_FORWARD, speed, dur);
}

void MOTOR_GoBackward(int16_t speed, uint32_t dur){
	MOTOR_Set(DIR_BACKWARD, speed, dur);
}

void MOTOR_TurnRight(int16_t speed, uint32_t dur){
	MOTOR_Set(DIR_RIGHT, speed, dur);
}

void MOTOR_TurnLeft(int16_t speed, uint32_t dur){
	MOTOR_Set(DIR_LEFT, speed, dur);
}

void MOTOR_TurnLeftAndRight(CallbackF f){
	callbackf = f;
	turnCount = 0;
	motorState = MOTOR_STATE_WAIT;
	duration = 1000;
	startTime = HAL_GetTick();
}

void MOTOR_Stop(){
	MOTOR_SetSpeed(MT_LEFT, 0);
	MOTOR_SetSpeed(MT_RIGHT, 0);
	MOTOR_DriverOff();
	motorState = MOTOR_STATE_IDLE;

	if(callbackf != NULL){
		callbackf();
		callbackf = NULL;
	}
}

void MOTOR_Set(uint8_t dir, int16_t speed, uint32_t dur){
	switch(dir){
		case DIR_LEFT:
			MOTOR_SetSpeed(MT_LEFT, -speed);
			MOTOR_SetSpeed(MT_RIGHT, speed);
			break;
		case DIR_RIGHT:
			MOTOR_SetSpeed(MT_LEFT, speed);
			MOTOR_SetSpeed(MT_RIGHT, -speed);
			break;
		case DIR_FORWARD:
			MOTOR_SetSpeed(MT_LEFT, speed);
			MOTOR_SetSpeed(MT_RIGHT, speed);
			break;
		case DIR_BACKWARD:
			MOTOR_SetSpeed(MT_LEFT, -speed);
			MOTOR_SetSpeed(MT_RIGHT, -speed);
			break;
		default: return;
	}

	startTime = HAL_GetTick();
	duration = dur;
	MOTOR_DriverOn();
}

void MOTOR_Update(){
	if(motorState == MOTOR_STATE_IDLE){
		return;
	}

	uint32_t now = HAL_GetTick();
	uint32_t delta = now - startTime;

	switch(motorState)
	{
		case MOTOR_STATE_WAIT:
			if(delta >= duration){
				motorState = MOTOR_STATE_TURN_LEFT_90;
				startTime = HAL_GetTick();
				MOTOR_Set(DIR_LEFT, CALIBRATING_SPEED, 500);
			}
			break;

		case MOTOR_STATE_STOP:
			MOTOR_Stop();
			break;

		case MOTOR_STATE_TURN_LEFT_90:
			if(delta >= duration){
				turnCount++;
				if(turnCount >= 4) {
					motorState = MOTOR_STATE_STOP;
				}
				else{
					motorState = MOTOR_STATE_TURN_RIGHT_180;
					startTime = HAL_GetTick();
					MOTOR_Set(DIR_RIGHT, CALIBRATING_SPEED, 1000);
				}
			}
			break;

		case MOTOR_STATE_TURN_RIGHT_90:
			if(delta >= duration){
				turnCount++;
				if(turnCount >= 4) {
					motorState = MOTOR_STATE_STOP;
				}
				else{
					motorState = MOTOR_STATE_TURN_LEFT_180;
					startTime = HAL_GetTick();
					MOTOR_Set(DIR_RIGHT, CALIBRATING_SPEED, 1000);
				}
			}
			break;

		case MOTOR_STATE_TURN_LEFT_180:
			if(delta >= duration){
				turnCount++;
				if(turnCount >= 4) {
					motorState = MOTOR_STATE_STOP;
				}
				else{
					motorState = MOTOR_STATE_TURN_RIGHT_90;
					startTime = HAL_GetTick();
					MOTOR_Set(DIR_RIGHT, CALIBRATING_SPEED, 500);
				}
			}
			break;

		case MOTOR_STATE_TURN_RIGHT_180:
			if(delta >= duration){
				turnCount++;
				if(turnCount >= 4) {
					motorState = MOTOR_STATE_STOP;
				}
				else{
					motorState = MOTOR_STATE_TURN_LEFT_180;
					startTime = HAL_GetTick();
					MOTOR_Set(DIR_LEFT, CALIBRATING_SPEED, 1000);
				}
			}
			break;

		default: break;
	}
}

void MOTOR_SetSpeed(MotorId id, int speed){
	if(speed > MAX_SPEED) speed = MAX_SPEED;
	if(speed < -MAX_SPEED) speed = -MAX_SPEED;

	uint16_t pin1, pin2;

	if(id == MT_LEFT){
		pin1 = GPIO_PIN_8;
		pin2 = GPIO_PIN_9;
	}
	else{
		pin1 = GPIO_PIN_7;
		pin2 = GPIO_PIN_6;
	}

	if(speed < 0){
		HAL_GPIO_WritePin(GPIOB, pin2, SET);
		HAL_GPIO_WritePin(GPIOB, pin1, RESET);
	}
	else if(speed > 0){
		HAL_GPIO_WritePin(GPIOB, pin2, RESET);
		HAL_GPIO_WritePin(GPIOB, pin1, SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, pin2, RESET);
		HAL_GPIO_WritePin(GPIOB, pin1, RESET);
	}

	if(id == MT_LEFT){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(speed));
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(speed));
	}
}
