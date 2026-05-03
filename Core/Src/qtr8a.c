/* ================= qtr8a.c ================= */
#include "qtr8a.h"
#include "stdio.h"
#include "led.h"

#define INTERVAL 1
#define VOLTAGE_INDEX 8

static uint16_t dma_buffer[QTR8A_CHANNEL_COUNT + 1]; // voltaj ölçümü için 51K, 10K voltaj bölücü devre 12.6V (3S pil) 2482 = Tam dolu
static QTR8A_t sensor;
static uint8_t calibrationMode = 0;
static uint32_t startTime = 0;

static uint16_t minValBuf[QTR8A_CHANNEL_COUNT];
static uint16_t maxValBuf[QTR8A_CHANNEL_COUNT];

volatile bool adcReady = false;
lineType line = LINE_BLACK;

static const int16_t weights[QTR8A_CHANNEL_COUNT] = {
    -350, -250, -150, -50, 50, 150, 250, 350
};

uint32_t QTR8A_GetBatVoltage(){
	return dma_buffer[VOLTAGE_INDEX];
}

void QTR8A_Init(ADC_HandleTypeDef *hadc)
{
	QTR8A_Ledoff();
	QTR8A_LoadFromRecord();

    sensor.hadc = hadc;
    for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
        sensor.raw[i] = 0;
        sensor.norm[i] = 0;
    }

    HAL_ADC_Start_DMA(hadc, (uint32_t*)dma_buffer, QTR8A_CHANNEL_COUNT + 1);
}

void QTR8A_SetLineType(lineType l){
	line = l;
}

lineType QTR8A_GetLineType(){
	return line;
}

void QTR8A_ToggleLineType(){
	line = (line == LINE_WHITE) ? LINE_BLACK : LINE_WHITE;
}

void QTR8A_StartCalibration(){
	for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		minValBuf[i] = 0x0FFF;
		maxValBuf[i] = 0;
	}

	QTR8A_Ledon();
	calibrationMode = 1;
	startTime = HAL_GetTick();
	LED_AnimateOnce(BUZZER, SHORT_BLINK);
}

void QTR8A_EndCalibration(){
	calibrationMode = 0;
	QTR8A_Ledoff();
}

void QTR8A_Update()
{
    if(adcReady)
    {
        memcpy(sensor.raw, dma_buffer, sizeof(sensor.raw));
        QTR8A_Normalize();
        adcReady = false;

        if(calibrationMode)
            QTR8_CalibrateToBuffer();

    }

    if(!calibrationMode) return;

    uint32_t currentTick = HAL_GetTick();

    if(currentTick - startTime >= 4000)
    {
        memcpy(sensor.minVal, minValBuf, sizeof(sensor.minVal));
        memcpy(sensor.maxVal, maxValBuf, sizeof(sensor.maxVal));
        QTR8A_EndCalibration();

        if(QTR8A_CheckSensorValues() == 1){
        	LED_AnimateOnce(BUZZER, LONG_BLINK);
        }
    }
}

uint8_t QTR8A_CheckSensorValues(){
	uint8_t errorCount = 0;

	for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		int16_t min = sensor.minVal[i];
		int16_t max = sensor.maxVal[i];

		if(max <= min || (max - min) < 1000){
			sensor.minVal[i] = 500;
			sensor.maxVal[i] = 3500;
			errorCount++;
		}
	}

	if(errorCount > 0){
		LED_AnimateFor(BUZZER, BLINK, errorCount);
		return 0;
	}

	return 1;
}

void QTR8A_LoadFromRecord(){
	WL_Record record;

	uint8_t success = WL_ReadLatest(&record);
	if(!success){
		LED_On(GREEN);
		QTR8A_Reset();
		return;
	}

	for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		sensor.minVal[i] = record.sensorMin[i];
		sensor.maxVal[i] = record.sensorMax[i];
	}
}

void QTR8A_WriteToRecord(WL_Record *rec)
{
    for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
        rec->sensorMin[i] = sensor.minVal[i];
        rec->sensorMax[i] = sensor.maxVal[i];
    }
}

void QTR8A_Ledon(){
	HAL_GPIO_WritePin(LEDON_PORT, LEDON_PIN, GPIO_PIN_SET);
}

void QTR8A_Ledoff(){
	HAL_GPIO_WritePin(LEDON_PORT, LEDON_PIN, GPIO_PIN_RESET);
}

void QTR8A_Reset(){
	for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		sensor.minVal[i] = 4095;
		sensor.maxVal[i] = 0;
	}
}

void QTR8A_GetCalibratedString(char* str, size_t str_size)
{
    int offset = 0;
    for(int i = 0; i < QTR8A_CHANNEL_COUNT; i++)
    {
        offset += snprintf(str + offset, str_size - offset, "%u:%u", sensor.minVal[i], sensor.maxVal[i]);
        if(i < QTR8A_CHANNEL_COUNT - 1)
        {
            offset += snprintf(str + offset, str_size - offset, " | ");
        }
    }
}

void QTR8A_GetWeightedString(char* str, size_t str_size){
	int offset = 0;
	    for(int i = 0; i < QTR8A_CHANNEL_COUNT; i++)
	    {
	    	offset += snprintf(str + offset, str_size - offset, "%u", weights[i]);
			if(i < QTR8A_CHANNEL_COUNT - 1)
			{
				offset += snprintf(str + offset, str_size - offset, " | ");
			}
	    }
}

uint8_t QTR8A_GetWeight(){
	uint16_t sumWeight = 0;
	for (uint8_t i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		sumWeight += sensor.norm[i];
	}
	return sumWeight / 8;
}

//---------------------------------------

void QTR8_CalibrateToBuffer(){
	for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		if (sensor.raw[i] < minValBuf[i]) minValBuf[i] = sensor.raw[i];
		if (sensor.raw[i] > maxValBuf[i]) maxValBuf[i] = sensor.raw[i];
	}
}


void QTR8A_GetNormalizedSensorValues(uint16_t* buffer){
	for(uint8_t i=0; i<8; i++){
		buffer[i] = sensor.norm[i];
	}
}

void QTR8A_Normalize() // siyah:high, 4095
{
    for (int i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
        uint16_t min = sensor.minVal[i];
        uint16_t max = sensor.maxVal[i];

        // raw değeri değiştirmeden clamp uygula
        uint16_t raw_val = sensor.raw[i];
        if(raw_val < min) raw_val = min;
        else if(raw_val > max) raw_val = max;

        if (max <= min) {
            sensor.norm[i] = 0;
        } else {
            uint32_t temp = (uint32_t)(raw_val - min) * 100;

            if(line == LINE_WHITE)
            	sensor.norm[i] = 100 - (uint16_t)(temp / (max - min));
            else
            	sensor.norm[i] = (uint16_t)(temp / (max - min));
        }
    }
}

uint8_t QTR8A_GetLinePattern(){ //Beyaz 0, Siyah 1
	uint8_t linePattern = 0;

	for (uint8_t i = 0; i < QTR8A_CHANNEL_COUNT; i++) {
		if(sensor.norm[i] > 60){
			linePattern |= 128 >> i;
		}
	}
	return linePattern;
}

int16_t QTR8A_GetPosition() //max 350
{
    int32_t sumVal = 0;
    int32_t sumWeight = 0;

    for (uint8_t i = 0; i < QTR8A_CHANNEL_COUNT; i++) {

    	if(sensor.norm[i] > 10){
    		sumVal += sensor.norm[i];
			sumWeight += (int32_t)sensor.norm[i] * (int32_t)weights[i];
    	}
    }

    if (sumVal == 0) return 0;

    return sumWeight / sumVal;
}

