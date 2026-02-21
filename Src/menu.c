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
static uint8_t menuIndex = 0;

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
	PID_TestDriveOff();
}

void MENU_StartDriving(){
	setMode(MODE_DRIVE);
	PID_TestDriveOff();
	PID_Enable();
}

void MENU_StartTestDriving(){
	setMode(MODE_DRIVE);
	PID_TestDriveOn();
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

static void enterMenu(){
	menuIndex = 0;
	setMode(MODE_MENU);
}


//------------------------------------------> MODE_MENU
static void setMenuIndexLED(){
	switch(menuIndex){
		case 0: LED_On(GREEN); LED_Off(YELLOW); LED_On(BLUE); break; //blue ters
		case 1: LED_On(YELLOW); LED_Off(GREEN); LED_On(BLUE); break;
		case 2: LED_On(GREEN); LED_On(YELLOW); LED_On(BLUE); break;
		case 3: LED_Off(BLUE); LED_Off(YELLOW); LED_Off(GREEN); break;
		case 4: LED_Off(BLUE); LED_Off(YELLOW); LED_On(GREEN); break;
		default: break;
	}
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

		case MODE_RACE:
			ROBOT_Stop();
			setMode(MODE_IDLE);
			break;

		case MODE_DRIVE:
			PID_Disable();
			setMode(MODE_IDLE);
			break;

		case MODE_MENU:
			setMode(MODE_IDLE);
			break;

		case MODE_P:
			setMode(MODE_MENU);
			break;

		case MODE_D:
			setMode(MODE_MENU);
			break;

		case MODE_SPEED:
			setMode(MODE_MENU);
			break;

		case MODE_SAVE:
			setMode(MODE_MENU);
			break;

		case MODE_TEST_DRIVE:
			ROBOT_Stop();
			setMode(MODE_MENU);
			break;

		default:
			setMode(MODE_IDLE);
			break;
	}
}

static void nextMenu(){
	if(menuIndex < 4){
		menuIndex++;
		setMenuIndexLED();
	}
}

static void prevMenu(){
	if(menuIndex > 0){
		menuIndex--;
		setMenuIndexLED();
	}
}

static void enterSubMenu(){
	switch(menuIndex){
		case 0:
			setMode(MODE_P);
			break;
		case 1:
			setMode(MODE_D);
			break;
		case 2:
			setMode(MODE_SPEED);
			break;
		case 3:
			setMode(MODE_SAVE);
			break;
		case 4:
			setMode(MODE_TEST_DRIVE);
			//PID_SetSpeed(20);
			PID_TestDriveOn();
			break;
		default: return; break;
	}
}

//----------------------------------------

//--------------------------------> SUB_MENU
static void increaseP(){
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
	PID_IncreaseKp();
}

static void decreaseP(){
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
	PID_DecreaseKp();
}

static void increaseD(){
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
	PID_IncreaseKd();
}

static void decreaseD(){
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
	PID_DecreaseKd();
}

static void increaseS(){
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
	PID_IncreaseSpeed();
}

static void decreaseS(){
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
	PID_DecreaseSpeed();
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
	LED_Off(YELLOW);
}

void MENU_SelectPistB(){
	PID_SetPist(PIST_B);
	LED_On(YELLOW);
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
			setButton(C, noAction, enterMenu);
			break;

		case MODE_CALIBRATION:
			setButton(A, noAction, noAction);
			setButton(B, MENU_GoUp, noAction);
			setButton(C, noAction, noAction);
			break;

		case MODE_RACE:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, noAction, noAction);
			setButton(C, noAction, noAction);
			break;

		case MODE_DRIVE:
			setButton(A, MENU_GoUp, MENU_GoUp);
			setButton(B, MENU_GoUp, MENU_GoUp);
			setButton(C, MENU_GoUp, MENU_GoUp);
			break;

		case MODE_MENU:
			setButton(A, prevMenu, noAction);
			setButton(B, nextMenu, noAction);
			setButton(C, enterSubMenu, MENU_GoUp);
			break;

		case MODE_P:
			setButton(A, decreaseP, noAction);
			setButton(B, increaseP, noAction);
			setButton(C, MENU_GoUp, noAction);
			break;

		case MODE_D:
			setButton(A, decreaseD, noAction);
			setButton(B, increaseD, noAction);
			setButton(C, MENU_GoUp, noAction);
			break;

		case MODE_SPEED:
			setButton(A, decreaseS, noAction);
			setButton(B, increaseS, noAction);
			setButton(C, MENU_GoUp, noAction);
			break;

		case MODE_SAVE:
			setButton(A, noAction, noAction);
			setButton(B, noAction, noAction);
			setButton(C, MENU_GoUp, MENU_Save);
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
			LED_Off(YELLOW);
			LED_On(BLUE);
			break;

		case MODE_CALIBRATION:
			LED_Animate(GREEN, HEARTBEAT);
			LED_Animate(YELLOW, HEARTBEATREV);
			break;

		case MODE_RACE:
			LED_Animate(YELLOW, HEARTBEAT);
			LED_Animate(GREEN, HEARTBEATREV);
			break;

		case MODE_MENU:
			setMenuIndexLED();
			break;

		case MODE_P:
			LED_Off(YELLOW);
			LED_Animate(GREEN, BLINK);
			break;

		case MODE_D:
			LED_Off(GREEN);
			LED_Animate(YELLOW, BLINK);
			break;

		case MODE_SPEED:
			LED_Animate(GREEN, BLINK);
			LED_Animate(YELLOW, BLINK);
			break;

		case MODE_SAVE:
			LED_Animate(BLUE, BLINK);
			break;

		case MODE_TEST_DRIVE:
			LED_Animate(BLUE, BLINK);
			LED_Animate(GREEN, BLINK);
			break;

		case MODE_DRIVE:
			LED_Animate(YELLOW, HEARTBEAT);
			break;


		default: return; break;
	}
}
