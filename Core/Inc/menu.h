#ifndef MENU_H
#define MENU_H

typedef enum {
	MODE_IDLE,
    MODE_CALIBRATION,
	MODE_TEST_ENCODER,
    MODE_RACE,
	MODE_DRIVE,
	MODE_TEST_DRIVE,
} SystemMode;

SystemMode MENU_GetCurrentMode();
void MENU_SetCurrentMode(SystemMode mode);
void MENU_GoUp();
void MENU_StartCalibration();
void MENU_StartTestEncoder();
void MENU_Save();

void MENU_StartRacing();
void MENU_StartTestDriving();
void MENU_StartTestDrivingD(); //Directly
void MENU_StartDriving();

void MENU_GoHome();



#endif
