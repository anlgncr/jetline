#include "pid.h"
#include <stdio.h>
#include "uart_ring.h"

typedef enum {
    DRIVE_MODE_NORMAL,
    DRIVE_MODE_TURN_LEFT_90,
    DRIVE_MODE_TURN_RIGHT_90,
    DRIVE_MODE_SEARCH,
    DRIVE_MODE_STOP,
	DRIVE_MODE_STRAIGHT
} DriveMode;

Pist selectedPist = PIST_A;

#define MAX_SPEED 1000
#define BRAKE_SPEED -50
#define DT_S 0.002f

// flags: bit3:cornerMode, bit2:autoDetectLine, bit1 = lineType
DriveMode driveMode = DRIVE_MODE_NORMAL;
static lineType line = LINE_BLACK;
static bool autoDetectLine = false;
static bool cornerMode = true;
volatile bool pidEnabled = false;
static bool testDrive = false;
static bool encoderReset = false;
static float lineWeight = 0;
static bool atCorner = false;
static bool leftCorner = false;
static bool rightCorner = false;
static uint8_t nextPoint = 0;
static uint32_t timer = 0;

static float Kp = 1;
static float Ki = 1;
static float Kd = 0.01f;

static int16_t prevError;
//static float integral;
static int16_t baseSpeed = 1000;
static int16_t baseSpeedActive;
static int16_t cornerSpeed;
static int16_t turningSpeed;
static float dFiltered = 0;

static uint16_t cornerPoint[2][CORNER_COUNT] = {};
static LineSide lineSide;

void PID_Init(){
	PID_LoadFromRecord();
}

void PID_Enable(){
	baseSpeedActive = cornerSpeed;
	prevError = 0;
	dFiltered = 0;
	encoderReset = false;
	lineWeight = 0;
	atCorner = false;
	leftCorner = false;
	rightCorner = false;
	nextPoint = 0;

	MOTOR_DriverOn();
	pidEnabled = true;
}

void PID_Disable(){
	pidEnabled = false;
	MOTOR_Stop();
}

// 8 bitlik pattern sensörler ile aynı sırada MSB->soldaki ir sensor
void PID_Update(void)
{
	if(!pidEnabled) return;

	QTR8A_Measure();
	QTR8A_Normalize(line);

	// --------------------------------------------------
	// AUTO LINE DETECT
	// --------------------------------------------------
	if(autoDetectLine){
	    uint8_t weight = QTR8A_GetWeight();

	    if(weight < 90){
	        lineWeight = weight * 0.02f + lineWeight * 0.98f;

	        if(lineWeight > 70){
	            line = (line == LINE_WHITE) ? LINE_BLACK : LINE_WHITE;
	            lineWeight = 0;
	            return;
	        }
	    }
	}

	// --------------------------------------------------
	// SENSOR ANALYSIS
	// --------------------------------------------------
	uint8_t linePattern = QTR8A_GetLinePattern();
	bool allBlack = (linePattern == 0xFF);
	bool lineLost = (linePattern == 0x00);

	// Köşe maskeleri
	bool sharpTurnLeft  = ((linePattern & 0xE0) == 0xE0) && !(linePattern & 0x07);
	bool sharpTurnRight = ((linePattern & 0x07) == 0x07) && !(linePattern & 0xE0);

	// --------------------------------------------------
	// DRIVE MODE DECISION
	// --------------------------------------------------
	if(lineLost){
	    driveMode = DRIVE_MODE_SEARCH;
	}
	else if(allBlack){
	    driveMode = DRIVE_MODE_STRAIGHT;

	    if(!encoderReset){ // Yarış başladı!
	        baseSpeedActive = testDrive ? cornerSpeed : baseSpeed;

	        ENCODER_Reset();
	        encoderReset = true;
	        timer = HAL_GetTick();
	        LED_AnimateOnce(BUZZER, LONG_BLINK);
	    }
	}
	else{
	    // Sharp turns
	    if(sharpTurnLeft)
	        driveMode = DRIVE_MODE_TURN_LEFT_90;
	    else if(sharpTurnRight)
	        driveMode = DRIVE_MODE_TURN_RIGHT_90;
	    else
	    	driveMode = DRIVE_MODE_NORMAL;

	    // Line side detect
		if(linePattern & 0xC0)
			lineSide = LINE_LEFT;
		else if(linePattern & 0x03)
			lineSide = LINE_RIGHT;
	}

	// --------------------------------------------------
	// TEST DRIVE MODE
	// --------------------------------------------------
	if(testDrive){
	    if(encoderReset && ENCODER_GetDistance() > 10 && !allBlack){
	        if(sharpTurnLeft && !leftCorner){
	            leftCorner = true;
	            PID_SetPoint(nextPoint++, ENCODER_GetCount());
	            LED_AnimateOnce(BUZZER, SHORT_BLINK);
	        }
	        else if(sharpTurnRight && !rightCorner){
	            rightCorner = true;
	            PID_SetPoint(nextPoint++, ENCODER_GetCount());
	            LED_AnimateOnce(BUZZER, SHORT_BLINK);
	        }
	        else if(linePattern == 0x18 || linePattern == 0x1C || linePattern == 0x38){
	            leftCorner = false;
	            rightCorner = false;
	        }
	    }
	}

	// --------------------------------------------------
	// NORMAL MODE CORNER CONTROL
	// --------------------------------------------------
	else if(encoderReset){
	    uint16_t slowPoint   = cornerPoint[selectedPist][nextPoint];
	    uint16_t currentPoint = ENCODER_GetCount();

	    if(slowPoint == 0){ // Yarış bitti!
	    	PID_Disable();
	        LED_AnimateFor(BUZZER, SHORT_BLINK, 3);
	        UART_CMD_SendFinishTime(HAL_GetTick() - timer);
	        MENU_GoHome();
	        return;
	    }


	    if(!atCorner && currentPoint >= (slowPoint - 8)){
	    	if(cornerMode)
	            baseSpeedActive = cornerSpeed;

	         atCorner = true;
	    }
	    else if(atCorner && currentPoint >= (slowPoint + 8)){
	        baseSpeedActive = baseSpeed;
	        nextPoint++;
	        atCorner = false;
	    }
	}

	if(driveMode == DRIVE_MODE_NORMAL){
		float error = QTR8A_GetPosition();

		if(abs(error) < 5)	error = 0;

		float speedRatio = baseSpeedActive / MAX_SPEED;
		if(speedRatio < 0.3f)
			speedRatio = 0.3f;

		float derivative = (error - prevError) / DT_S;
		dFiltered = 0.9f * dFiltered + 0.1f * derivative;
		prevError = error;

		int32_t output = (Kp * speedRatio) * error + Kd * dFiltered;

		if(output > 200) output = 200;

		int16_t leftSpeed = baseSpeedActive + output;
		int16_t rightSpeed = baseSpeedActive - output;

		// Tekerlekleri geri döndürmüyoruz sadece frenliyoruz
		if(leftSpeed < BRAKE_SPEED){
			leftSpeed = BRAKE_SPEED;
		}

		if(rightSpeed < BRAKE_SPEED){
			rightSpeed = BRAKE_SPEED;
		}

		MOTOR_SetSpeed(MT_LEFT, leftSpeed);
		MOTOR_SetSpeed(MT_RIGHT, rightSpeed);
	}
	else{
		prevError = 0;
		dFiltered = 0;

		if(driveMode == DRIVE_MODE_SEARCH){
			if(lineSide == LINE_LEFT){
				MOTOR_SetSpeed(MT_LEFT, BRAKE_SPEED);
				MOTOR_SetSpeed(MT_RIGHT, turningSpeed);
			}
			else{
				MOTOR_SetSpeed(MT_LEFT, turningSpeed);
				MOTOR_SetSpeed(MT_RIGHT, -50);
			}
		}
		else if(driveMode == DRIVE_MODE_TURN_LEFT_90){
			MOTOR_SetSpeed(MT_LEFT, BRAKE_SPEED);
			MOTOR_SetSpeed(MT_RIGHT, turningSpeed);
		}
		else if(driveMode == DRIVE_MODE_TURN_RIGHT_90){
			MOTOR_SetSpeed(MT_LEFT, turningSpeed);
			MOTOR_SetSpeed(MT_RIGHT, BRAKE_SPEED);
		}
		else if(driveMode == DRIVE_MODE_STRAIGHT){
			MOTOR_SetSpeed(MT_LEFT,  baseSpeedActive);
			MOTOR_SetSpeed(MT_RIGHT, baseSpeedActive);
		}
	}
}

void PID_LoadFromRecord(){
	WL_Record record;

	uint8_t success = WL_ReadLatest(&record);
	if(!success){
		LED_On(YELLOW);
		LED_AnimateOnce(BUZZER, LONG_BLINK);
		return;
	}

	Kp = record.Kp / 1000.0f;
	Kd = record.Kd / 1000.0f;
	baseSpeed = record.basespeed;
	cornerSpeed = record.cornerSpeed;
	turningSpeed = record.turningSpeed;

	line = (record.flags & 0x01)? LINE_BLACK : LINE_WHITE;
	autoDetectLine = (record.flags & 0x02);
	cornerMode = (record.flags & 0x04);
	selectedPist = (record.flags & 0x08)? PIST_B : PIST_A;

	memcpy(cornerPoint, record.corners, sizeof cornerPoint);
}

void PID_WriteToRecord(WL_Record *rec)
{
    rec->Kp = (int32_t)(Kp * 1000.0f);
    rec->Kd = (int32_t)(Kd * 1000.0f);
    rec->basespeed = baseSpeed;
    rec->cornerSpeed = cornerSpeed;
    rec->turningSpeed = turningSpeed;
    rec->flags = ((line == LINE_BLACK) << 0) | ((autoDetectLine) << 1) | ((cornerMode) << 2) | ((selectedPist == PIST_B) << 3);

    memcpy(rec->corners, cornerPoint, sizeof cornerPoint);
}


uint16_t* PID_GetPIDPacket(void) {
    static uint16_t packet[8];

	int i = 0;
	packet[i++] = (uint16_t)(Kp * 1000);
	packet[i++] = (uint16_t)(Kd * 1000);
	packet[i++] = baseSpeed;
	packet[i++] = turningSpeed;
	packet[i++] = cornerSpeed;
	packet[i++] = ((line == LINE_BLACK) << 0) | ((autoDetectLine) << 1) |	((cornerMode) << 2) | ((selectedPist == PIST_B) << 3);

    return packet;
}

uint8_t PID_GetPistCorners(uint16_t* buffer){
	if(selectedPist >= 2)
	   return 0;

	uint8_t count = 0;
	for(uint8_t i=0; i< CORNER_COUNT; i++){
		uint16_t pulseCount = cornerPoint[selectedPist][i];
		if(pulseCount == 0)
			break;

		buffer[i] = pulseCount;
		count++;
	}

	return count;
}

void PID_SetPistCorners(uint16_t* cornerArray, size_t len){
	if(selectedPist >= 2)
	   return;

	if(len > CORNER_COUNT)
		len = CORNER_COUNT - 1;

	//cornerArray içerisindeki veriler cm cinsinden
	memset(cornerPoint[selectedPist], 0, sizeof cornerPoint[0]);

	for(uint16_t i=0; i<len; i++){
		cornerPoint[selectedPist][i] = cornerArray[i];
	}
}

void PID_SetPoint(uint8_t index, uint16_t point){
	if(index < (CORNER_COUNT - 1)){ // sondaki alan 0 kalsın. Bitişi anlamak için
		cornerPoint[selectedPist][index] = point;
		LED_AnimateOnce(BUZZER, SHORT_BLINK);
		return;
	}

	LED_AnimateOnce(BUZZER, LONG_BLINK);
}

void PID_SetLineBlack(){
	line = LINE_BLACK;
	autoDetectLine = false;
}

void PID_SetLineWhite(){
	line = LINE_WHITE;
	autoDetectLine = false;
}

void PID_SetLineAuto(){
	autoDetectLine = true;
}

void PID_SetCornerMode(bool val){
	cornerMode = val;
}

void PID_SetBaseSpeed(uint16_t speed){
	baseSpeed = speed;
}

void PID_SetP(uint16_t p){
	Kp = p / 1000.0f;
}

void PID_SetD(uint16_t d){
	Kd = d / 1000.0f;
}

void PID_SetI(uint16_t i){
	Ki = i / 1000.0f;
}

void PID_IncreaseKp(){
	if(Kp < 2)
		Kp += 0.1f;
	else
		LED_AnimateOnce(BUZZER, LONG_BLINK);
}

void PID_DecreaseKp(){
	if(Kp > 0)
		Kp -= 0.1f;
	else
		LED_AnimateOnce(BUZZER, LONG_BLINK);
}

void PID_IncreaseKd(){
	if(Kd < 0.2f)
		Kd += 0.01f;
	else
		LED_AnimateOnce(BUZZER, LONG_BLINK);
}

void PID_DecreaseKd(){
	if(Kd > 0)
		Kd -= 0.01f;
	else
		LED_AnimateOnce(BUZZER, LONG_BLINK);
}

void PID_IncreaseSpeed(){
	if((baseSpeed + 50) <= 1000){
		baseSpeed += 50;
	}
	else{
		baseSpeed = MAX_SPEED;
		LED_AnimateOnce(BUZZER, LONG_BLINK);
	}
}

void PID_DecreaseSpeed(){
	if((baseSpeed - 50) >= 0){
		baseSpeed -= 50;
	}
	else{
		baseSpeed = 0;
		LED_AnimateOnce(BUZZER, LONG_BLINK);
	}
}

void PID_TestDriveOn(){
	for (int i = 0; i < CORNER_COUNT; i++) {
		cornerPoint[selectedPist][i] = 0;
	}
	testDrive = true;
}

void PID_TestDriveOff(){
	testDrive = false;
}

void PID_SetCornerSpeed(uint16_t speed){
	cornerSpeed = speed;
}

void PID_SetTurningSpeed(uint16_t speed){
	turningSpeed = speed;
}

void PID_SetPist(Pist p){
	selectedPist = p;
}



