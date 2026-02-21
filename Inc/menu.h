#ifndef MENU_H
#define MENU_H

typedef enum {
	MODE_IDLE,
    MODE_CALIBRATION,
    MODE_RACE,
	MODE_DRIVE,
	MODE_TEST_DRIVE,
	MODE_MENU,
	MODE_P,
	MODE_D,
	MODE_SPEED,
	MODE_SAVE,
} SystemMode;

SystemMode MENU_GetCurrentMode();
void MENU_SetCurrentMode(SystemMode mode);
void MENU_GoUp();
void MENU_StartCalibration();
void MENU_Save();
void MENU_StartRacing();
void MENU_StartDriving();
void MENU_StartTestDriving();
void MENU_GoHome();



#endif
