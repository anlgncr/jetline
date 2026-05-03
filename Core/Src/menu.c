#include "menu.h"
#include "stdio.h"
#include "string.h"
#include "button.h"
#include "motors.h"
#include "hcsr04.h"
#include "qtr8a.h"
#include "callbacks.h"
#include "led.h"
#include "flash_wl.h"
#include "pid.h"
#include "usbd_cdc_if.h"
#include "robot.h"

static SystemMode currentMode = MODE_IDLE;

static void setMode(SystemMode mode);
static void setMenuEffect();

SystemMode MENU_GetCurrentMode(){
	return currentMode;
}

void MENU_SetCurrentMode(SystemMode mode){
	currentMode = mode;
	setMode(mode);
}

//-----------------------------> MODE_IDLE
void MENU_StartRacing(){
	setMode(MODE_RACE);
	HCSR04_ResetState();
	PID_TestDriveOff();
}

void MENU_StartTestDriving(){
	setMode(MODE_TEST_DRIVE);
	PID_TestDriveOn();
	HCSR04_ResetState();
	LED_AnimateFor(BUZZER, BLINK, 2);
}

void MENU_StartTestDrivingD(){ // Direkt komutla başlar
	setMode(MODE_DRIVE);
	PID_TestDriveOn();
	PID_Enable();
}

void MENU_StartDriving(){ // Direkt komutla başlar
	setMode(MODE_DRIVE);
	PID_TestDriveOff();
	PID_Enable();
}

static void calibrationEndCallback(){
	setMode(MODE_IDLE);
}

void MENU_StartCalibration(){
	setMode(MODE_CALIBRATION);
	QTR8A_StartCalibration();
	MOTOR_TurnLeftAndRight(calibrationEndCallback);
}

static void encoderTestEndedCallback(){
	setMode(MODE_IDLE);
}

void MENU_StartTestEncoder(){
	ENCODER_Reset();
	setMode(MODE_TEST_ENCODER);
	MOTOR_GoForwardAndBackward(encoderTestEndedCallback);
}

void MENU_GoHome(){
	if(currentMode != MODE_IDLE){
		MENU_GoUp();
		MENU_GoHome();
	}
}

void MENU_GoUp(){
	switch(currentMode){
		case MODE_CALIBRATION:
			QTR8A_EndCalibration();
			MOTOR_Stop();
			setMode(MODE_IDLE);
			break;

		case MODE_TEST_ENCODER:
			MOTOR_Stop();
			setMode(MODE_IDLE);
			break;

		case MODE_RACE:
			ROBOT_Stop();
			setMode(MODE_IDLE);
			break;

		case MODE_TEST_DRIVE:
			ROBOT_Stop();
			setMode(MODE_IDLE);
			break;

		case MODE_DRIVE:
			PID_Disable();
			setMode(MODE_IDLE);
			break;

		default:
			setMode(MODE_IDLE);
			break;
	}
}


void MENU_Save(){
	WL_Record record = {};

	QTR8A_WriteToRecord(&record);
	PID_WriteToRecord(&record);
	WL_WriteRecord(&record);

	LED_AnimateOnce(BUZZER, LONG_BLINK);
	MENU_GoHome();
}

void MENU_SelectPistA(){
	PID_SetPist(PIST_A);
	LED_On(GREEN);
}

void MENU_SelectPistB(){
	PID_SetPist(PIST_B);
	LED_Off(GREEN);
}

static void noAction(){}

static void setButton(uint8_t btn, ButtonCallback up, ButtonCallback longPress){
    buttons[btn].onUp = up;
    buttons[btn].onLong = longPress;
}

static void setMode(SystemMode mode){
	switch(mode){
		case MODE_IDLE:
			setButton(A, MENU_SelectPistA, MENU_StartRacing);
			setButton(B, MENU_SelectPistB, MENU_StartCalibration);
			setButton(C, noAction, MENU_StartTestDriving);
			break;

		case MODE_CALIBRATION:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, MENU_GoUp, MENU_GoUp);
			setButton(C, MENU_GoUp, MENU_GoUp);
			break;

		case MODE_TEST_ENCODER:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, MENU_GoUp, MENU_GoUp);
			setButton(C, MENU_GoUp, MENU_GoUp);
			break;

		case MODE_RACE:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, MENU_GoUp, MENU_GoUp);
			setButton(C, MENU_GoUp, MENU_GoUp);
			break;

		case MODE_DRIVE:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, MENU_GoUp, MENU_GoUp);
			setButton(C, MENU_GoUp, MENU_GoUp);
			break;

		case MODE_TEST_DRIVE:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, MENU_GoUp, MENU_GoUp);
			setButton(C, MENU_GoUp, MENU_GoUp);
			break;

		default: return;
	}

	currentMode = mode;
	setMenuEffect();
}

static void setMenuEffect(){
	switch(currentMode){
		case MODE_IDLE:
			LED_Off(GREEN);
			LED_On(BLUE);
			break;

		case MODE_CALIBRATION:
			LED_Animate(GREEN, HEARTBEAT);
			break;

		case MODE_TEST_ENCODER:
			LED_Animate(GREEN, HEARTBEAT);
			break;

		case MODE_RACE:
			LED_Animate(GREEN, BLINK);
			break;

		case MODE_TEST_DRIVE:
			LED_Animate(GREEN, BLINK);
			break;

		case MODE_DRIVE:
			break;


		default: return; break;
	}
}
