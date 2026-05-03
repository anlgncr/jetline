/*
 *
 * Written with anxiety and pleasure by Anıl
 *
 * This system has two modes:
 *  - TEST_MODE: records corner positions like a responsible engineer
 *  - RACE_MODE: trusts PID, ignores fear, prays to sensors,
 *               and slows down near corners
 *
 * Both modes are activated by crossing the start line.
 * Success depends equally on code and luck.
 *
 */


#include "pid.h"


static DriveMode driveMode = DRIVE_MODE_PID;
static CornerState cornerState = CORNER_NONE;
static LineSide lineSide;
static Pist selectedPist = PIST_A;

volatile bool pidEnabled = false;
static float Kp = 1.5f;
static float Kd = 0.060f;
static float dFiltered = 0;
static float prevError;
static float lineWeight = 0;

static uint32_t cornerStartEncoder = 0;
static uint8_t cornerExitStableCount = 0;
static uint8_t sharpLeftCounter = 0;
static uint8_t sharpRightCounter = 0;
static uint8_t allBlackCounter = 0;
static uint8_t allWhiteCounter = 0;

static uint32_t lastCount = 0;
static uint32_t distance = 0;

static bool waitForFinishLine = false;
static bool nearCorner = false;
static int16_t fastDistance = 0;
static uint8_t slowdownDistance = 24;

static bool autoDetectLine = false;
static bool cornerMode = false; // no need corner mode
static bool adaptiveSpeedEnabled = false;

static bool testDrive = false;
static bool encoderReset = false;

static uint16_t cornerPoint[2][CORNER_COUNT] = {};
static uint16_t currentPoint = 0;
static uint16_t nextPoint = 0;
static uint32_t timer = 0;

static int16_t baseSpeed = 800;
static int16_t baseSpeedActive;
static int16_t cornerSpeed = 600;
static int16_t turningSpeed = 600;


void PID_Init(){
	PID_LoadFromRecord();
}

void PID_Enable(){
	baseSpeedActive = cornerSpeed;
	prevError = 0;
	dFiltered = 0;
	lineWeight = 0;

	encoderReset = false;
	waitForFinishLine = false;
	nearCorner = false;

	currentPoint = 0;
	nextPoint = 0;
	fastDistance = 0;

	sharpLeftCounter = 0;
	sharpRightCounter = 0;
	allBlackCounter = 0;
	allWhiteCounter = 0;
	lastCount = 0;
	distance = 0;

	QTR8A_Ledon();
	MOTOR_DriverOn();
	pidEnabled = true;
}

void PID_Disable(){
	pidEnabled = false;
	MOTOR_Stop();
	QTR8A_Ledoff();
}

// 8 bitlik pattern sensörler ile aynı sırada MSB->soldaki ir sensor
void PID_Update(void)
{
	if(!pidEnabled) return;

	uint32_t currentCounter = ENCODER_GetCount();
	uint32_t delta = currentCounter - lastCount;

	lastCount = currentCounter;
	distance += delta;

	// --------------------------------------------------
	// AUTO LINE DETECT
	// --------------------------------------------------
	if(autoDetectLine){
	    uint8_t weight = QTR8A_GetWeight();

	    if(weight < 90){
	        lineWeight = weight * 0.02f + lineWeight * 0.98f;
	        if(lineWeight > 70){
	        	QTR8A_ToggleLineType();
	            lineWeight = 0;
	            return;
	        }
	    }
	}


	uint8_t linePattern = QTR8A_GetLinePattern();

	if(linePattern == 0xFF)
		allBlackCounter++;
	else
		allBlackCounter = 0;


	if(linePattern == 0x00)
		allWhiteCounter++;
	else
		allWhiteCounter = 0;


	// --------------------------------------------------
	// DRIVE MODE DECISION
	// --------------------------------------------------
	if(allWhiteCounter > STABLE_COUNT){
	    driveMode = DRIVE_MODE_SEARCH;
	}
	else if(allBlackCounter > STABLE_COUNT){

		if(waitForFinishLine && distance > 12 && cornerState == CORNER_NONE){
			PID_Disable(); // goHome içinde de var
			LED_AnimateFor(BUZZER, SHORT_BLINK, 3);
			UART_CMD_SendFinishTime(HAL_GetTick() - timer);
			MENU_GoHome();
			return;
		}

	    driveMode = DRIVE_MODE_STRAIGHT;

	    if(!encoderReset){ // Yarış başladı!
	        baseSpeedActive = testDrive ? cornerSpeed : baseSpeed;

	        ENCODER_Reset();
	        distance = 0;
	        lastCount = 0;
	        encoderReset = true;
	        timer = HAL_GetTick();
	        LED_AnimateOnce(BUZZER, LONG_BLINK);

	        if(testDrive)
	        	memset(cornerPoint[selectedPist], 0, sizeof cornerPoint[0]); // köşe dizisini sıfırla
	    }
	}
	else{
	    driveMode = DRIVE_MODE_PID;

	    // Line side detect
		if(linePattern & LINE_LEFT_MASK)
			lineSide = LINE_LEFT;
		else if(linePattern & LINE_RIGHT_MASK)
			lineSide = LINE_RIGHT;
	}


	if (encoderReset && ENCODER_GetDistance() > 12) {

		if (!testDrive)
		{
			if(currentPoint == nextPoint)
			{
				uint16_t pointA = cornerPoint[selectedPist][currentPoint];
				uint16_t pointB = cornerPoint[selectedPist][currentPoint + 1];
				int16_t diff = pointB - pointA;

				if(pointB == 0){
					waitForFinishLine = true;
					fastDistance = -1; // bitiş çizgisine kadar hızlı git
				}
				else if(diff > slowdownDistance){
					fastDistance = diff - slowdownDistance;
				}
				else{
					fastDistance = 0;
				}

				nextPoint = currentPoint + 1;
			}


			if(cornerState == CORNER_NONE && (distance < fastDistance || fastDistance == -1)){
				baseSpeedActive = baseSpeed;
				nearCorner = false;
			}else{
				nearCorner = true;
				baseSpeedActive = cornerSpeed;
			}
		}


		if (cornerState == CORNER_NONE) {
			if((linePattern & SHARP_LEFT_MASK) == 0xF0){
				sharpLeftCounter++;
			}else{
				sharpLeftCounter = 0;
			}

			if((linePattern & SHARP_RIGHT_MASK) == 0x0F){
				sharpRightCounter++;
			}else{
				sharpRightCounter=0;
			}

			if (sharpLeftCounter > STABLE_COUNT) {
				cornerState = CORNER_LEFT;
				cornerStartEncoder = ENCODER_GetCount();
				cornerExitStableCount = 0;
				LED_AnimateOnce(BUZZER, SHORT_BLINK);

				if (testDrive && currentPoint < (CORNER_COUNT - 2)){
					PID_SetPoint(currentPoint++, cornerStartEncoder);
				}else if(!waitForFinishLine /*&& nearCorner*/){
					nearCorner = false;
					currentPoint++;
					distance = 0;
				}
			}
			else if (sharpRightCounter > STABLE_COUNT) {
				cornerState = CORNER_RIGHT;
				cornerStartEncoder = ENCODER_GetCount();
				cornerExitStableCount = 0;
				LED_AnimateOnce(BUZZER, SHORT_BLINK);

				if (testDrive && currentPoint < (CORNER_COUNT - 2)){
					PID_SetPoint(currentPoint++, cornerStartEncoder);
				}else if(!waitForFinishLine /*&& nearCorner*/){
					nearCorner = false;
					currentPoint++;
					distance = 0;
				}
			}
		}
		// CORNER EXIT
		else {

			bool center = ((linePattern & 0x18) == 0x18);
			uint16_t delta = ENCODER_GetCount() - cornerStartEncoder;

			if (center) {
				cornerExitStableCount++;
			} else {
				cornerExitStableCount = 0;
			}

			if (cornerExitStableCount >= STABLE_COUNT && delta > 12) {
				cornerState = CORNER_NONE;
				cornerExitStableCount = 0;
				sharpLeftCounter = 0;
				sharpRightCounter = 0;
			}
		}
	}


	if(driveMode == DRIVE_MODE_PID){
		float error = QTR8A_GetPosition();
		float errorAbs = fabs(error);

		if(errorAbs < 5)
			error = 0;

		float brakeRate = 0;
		if(adaptiveSpeedEnabled && errorAbs > 100){
			brakeRate = (errorAbs - 100) / 250.0f;
		}

		int16_t adaptiveSpeed = baseSpeedActive - (baseSpeedActive * brakeRate);

		if(adaptiveSpeed < 200){
			adaptiveSpeed = 200;
		}

		float speedRatio = (float)adaptiveSpeed / (float)MAX_SPEED;

		float derivative = (error - prevError) / DT_S;
		dFiltered = 0.85f * dFiltered + 0.15f * derivative;
		prevError = error;

		float Kd_eff = Kd * (0.2f + 0.8f * speedRatio);

		int32_t output = Kp * (0.2f + 0.8f * speedRatio) * error + Kd_eff * dFiltered;

		if(output > 400) output = 400;
		else if(output < -400) output = -400;

		int16_t leftSpeed = adaptiveSpeed + output;
		int16_t rightSpeed = adaptiveSpeed - output;


		if(leftSpeed < MIN_SPEED){
			leftSpeed = MIN_SPEED;
		}

		if(rightSpeed < MIN_SPEED){
			rightSpeed = MIN_SPEED;
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
				MOTOR_SetSpeed(MT_RIGHT, BRAKE_SPEED);
			}
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
		LED_On(GREEN);
		LED_AnimateOnce(BUZZER, LONG_BLINK);
		return;
	}

	Kp = record.Kp / 1000.0f;
	Kd = record.Kd / 1000.0f;
	baseSpeed = record.basespeed;
	cornerSpeed = record.cornerSpeed;
	turningSpeed = record.turningSpeed;
	slowdownDistance = record.slowdownDistance;

	if(record.flags & 0x01)
		QTR8A_SetLineType(LINE_BLACK);
	else
		QTR8A_SetLineType(LINE_WHITE);


	autoDetectLine = (record.flags & 0x02);
	cornerMode = (record.flags & 0x04);
	selectedPist = (record.flags & 0x08)? PIST_B : PIST_A;
	adaptiveSpeedEnabled = (record.flags & 0x10);

	memcpy(cornerPoint, record.corners, sizeof cornerPoint);
}

void PID_WriteToRecord(WL_Record *rec)
{
    rec->Kp = (int32_t)(Kp * 1000.0f);
    rec->Kd = (int32_t)(Kd * 1000.0f);
    rec->basespeed = baseSpeed;
    rec->cornerSpeed = cornerSpeed;
    rec->turningSpeed = turningSpeed;
    rec->slowdownDistance = slowdownDistance;
    rec->flags = 	((QTR8A_GetLineType() == LINE_BLACK) << 0) |
    				((autoDetectLine) << 1) |
					((cornerMode) << 2) |
					((selectedPist == PIST_B) << 3) |
					((adaptiveSpeedEnabled) << 4);

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
	packet[i++] = slowdownDistance;
	packet[i++] = 	((QTR8A_GetLineType() == LINE_BLACK) << 0) |
					((autoDetectLine) << 1) |
					((cornerMode) << 2) |
					((selectedPist == PIST_B) << 3) |
					((adaptiveSpeedEnabled) << 4);

    return packet;
}

uint8_t PID_GetPistCorners(uint16_t* buffer){
	if(selectedPist >= 2)
	   return 0;

	uint8_t count = 0;
	for(uint8_t i=1; i< CORNER_COUNT; i++){
		uint16_t pulseCount = cornerPoint[selectedPist][i];
		if(pulseCount == 0)
			break;

		buffer[count++] = pulseCount;
	}

	return count;
}

void PID_SetPistCorners(uint16_t* cornerArray, size_t len){
	if(selectedPist >= 2)
	   return;

	if(len > CORNER_COUNT - 2)
		len = CORNER_COUNT - 2;

	//cornerArray içerisindeki veriler cm cinsinden
	memset(cornerPoint[selectedPist], 0, sizeof cornerPoint[0]);

	for(uint16_t i=0; i<len; i++){
		cornerPoint[selectedPist][i + 1] = cornerArray[i];
	}
}

void PID_SetPoint(uint8_t index, uint16_t point){
	uint8_t i = index + 1; // 0. index her zaman 0

	if(i < (CORNER_COUNT - 2)){ // sondaki alan 0 kalsın. Bitişi anlamak için
		cornerPoint[selectedPist][i] = point;
		LED_AnimateOnce(BUZZER, SHORT_BLINK);
		return;
	}

	LED_AnimateOnce(BUZZER, LONG_BLINK);
}

void PID_SetLineAuto(bool val){
	autoDetectLine = val;
}

void PID_SetCornerMode(bool val){
	cornerMode = val;
}

void PID_SetAdaptiveSpeed(bool val){
	adaptiveSpeedEnabled = val;
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

void PID_TestDriveOn(){
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

void PID_SetSlowDownDistance(uint8_t distance){
	if(distance > 70){ // 70 => 100.5 cm
		slowdownDistance = 70;
		return;
	}

	slowdownDistance = distance;
}
