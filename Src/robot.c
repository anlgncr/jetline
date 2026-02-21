#include "robot.h"

#define READY_THRESHOLD      40
#define START_THRESHOLD      60
#define SAMPLE_PERIOD_MS     30
#define START_CONFIRM_COUNT   2

typedef enum {
    START_WAIT_FOR_OBJECT = 0,
    START_WAIT_FOR_REMOVAL,
    START_RACE_STARTED
} StartState;

static bool readyForRace = false;
static bool driving = false;
static SystemMode currentMode;
static StartState startState = START_WAIT_FOR_OBJECT;
static uint8_t startConfirmCounter = 0;

void ROBOT_Init(ADC_HandleTypeDef *hadc1, TIM_HandleTypeDef *echoTimer){
	LED_Init();
	MENU_SetCurrentMode(MODE_IDLE);
	QTR8A_Init(hadc1);
	PID_Init();
	BUTTON_Init();
	MOTOR_Init();
	HCSR04_Init(echoTimer);

	LED_AnimateOnce(BUZZER, BUZZER_INIT);
}

void ROBOT_Update(){
	BUTTON_Update();
	LED_Update();
	QTR8A_Update();
	MOTOR_Update();
	UART_CMD_Process();
	ROBOT_CheckForRacing();
}

void ROBOT_CheckForRacing()
{
    static uint32_t lastTick = 0;

    if(driving)
        return;

    SystemMode currentMode = MENU_GetCurrentMode();
    if(currentMode != MODE_RACE && currentMode != MODE_TEST_DRIVE)
        return;

    uint32_t currentTick = HAL_GetTick();
    if(currentTick - lastTick < SAMPLE_PERIOD_MS)
        return;

    lastTick = currentTick;

    HCSR04_Trigger();
    uint16_t d = HCSR04_GetDistance();

    if(d == 0)
    	return;

    switch(startState)
    {
        case START_WAIT_FOR_OBJECT:
            if(d < READY_THRESHOLD){
                startConfirmCounter++;

                if(startConfirmCounter >= START_CONFIRM_COUNT){
                    startConfirmCounter = 0;
                    startState = START_WAIT_FOR_REMOVAL;
                }
            }
            else{
                startConfirmCounter = 0;
            }

        break;

        case START_WAIT_FOR_REMOVAL:

            if(d > START_THRESHOLD){
                startConfirmCounter++;

                if(startConfirmCounter >= START_CONFIRM_COUNT){
                    startState = START_RACE_STARTED;
                    driving = true;

                    PID_Enable();
                    LED_Animate(GREEN, BURST);
                    LED_Animate(YELLOW, BURST);
                }
            }
            else{
                startConfirmCounter = 0;
            }

        break;

        case START_RACE_STARTED:
        default:
            // hiçbir şey yapma
        break;
    }
}


void ROBOT_Stop(){
	PID_Disable();
	readyForRace = false;
	driving = false;
	startState = START_WAIT_FOR_OBJECT;
	startConfirmCounter = 0;

	if(currentMode == MODE_TEST_DRIVE){
		PID_TestDriveOff();
	}
}
