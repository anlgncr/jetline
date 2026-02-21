#include "uart_cmd.h"

#define CMD_PIST 0x01
#define CMD_SETTINGS 0x02
#define CMD_FINISH_TIME 0x03
#define CMD_QTR8A 0x04
#define CMD_HCSR 0x05

// --- Binary profile storage ---
static uint16_t tempProfile[CORNER_COUNT];

// --- Packet parser state ---
static PacketState_t pktState = PKT_WAIT_START;

static uint8_t  pktCount = 0;
static uint8_t  pktIndex = 0;
static uint8_t  bytePair = 0;

static uint16_t calculatedCRC = 0;
static uint16_t receivedCRC = 0;

// --- Text command parser ---
static char rxBuffer[128];
uint8_t bufferIndex = 0;

static uint16_t CRC16_Update(uint16_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc >>= 1;
    }
    return crc;
}


static uint16_t buildPacket(uint16_t* data, uint8_t cmd, uint8_t count, uint8_t* outBuffer)
{
    uint16_t crc = 0xFFFF;
    uint16_t pos = 0;

    outBuffer[pos++] = START_BYTE;
    crc = CRC16_Update(crc, START_BYTE);

    outBuffer[pos++] = cmd;
    crc = CRC16_Update(crc, cmd);

    outBuffer[pos++] = count;
    crc = CRC16_Update(crc, count);

    for(uint8_t i = 0; i < count; i++)
    {
        uint16_t value = data[i];
        if(value > 65535) value = 65535; // clamp

        uint8_t low = value & 0xFF;
        uint8_t high = (value >> 8) & 0xFF;

        outBuffer[pos++] = low;
        crc = CRC16_Update(crc, low);

        outBuffer[pos++] = high;
        crc = CRC16_Update(crc, high);
    }

    // CRC ekle
    outBuffer[pos++] = crc & 0xFF;        // CRC low
    outBuffer[pos++] = (crc >> 8) & 0xFF; // CRC high

    outBuffer[pos++] = END_BYTE;

    return pos; // toplam byte sayısı
}

// ----------------- UART ile gönderme -----------------
void sendCornersUART(uint16_t* corners , uint8_t count){
	if(corners == NULL || count == 0)
		return;

	uint8_t packet[3 + count * 2 + 3]; // (START+CMD+COUNT =>3) + DATA LENGTH + (CRC+END => 3)
	uint16_t packetLen = buildPacket(corners, CMD_PIST, count, packet);
    UART_PrintBytes(packet, packetLen);
}

void UART_CMD_SendFinishTime(uint32_t time){
	uint8_t packet[3 + 4 + 3]; // (START+CMD+COUNT =>3) + DATA LENGTH + (CRC+END => 3)

	uint16_t timeArray[2] = {
	    (uint16_t)(time),
	    (uint16_t)(time >> 16)
	};

	uint16_t packetLen = buildPacket(timeArray, CMD_FINISH_TIME, 2, packet);
	UART_PrintBytes(packet, packetLen);
}

void UART_CMD_SendQTR8A(){
	uint8_t packet[3 + 16 + 3]; // (START+CMD+COUNT =>3) + DATA LENGTH + (CRC+END => 3)
	uint16_t sensorData[8];
	QTR8A_GetSensorRaw(sensorData);
	uint16_t packetLen = buildPacket(sensorData, CMD_QTR8A, 8, packet);
	UART_PrintBytes(packet, packetLen);
}


void UART_CMD_SendHCSR(){
	uint8_t packet[3 + 2 + 3]; // (START+CMD+COUNT =>3) + DATA LENGTH + (CRC+END => 3)
	uint16_t d = HCSR04_GetDistance();

	uint16_t packetLen = buildPacket(&d, CMD_HCSR, 1, packet);
	UART_PrintBytes(packet, packetLen);
}

void UART_CMD_Process(){

    while (UART_Available())
    {
        uint8_t c = UART_Read();

        /* ---------- BINARY PROFIL PARSER ---------- */

        if(pktState != PKT_WAIT_START || c == START_BYTE)
        {
            switch(pktState)
            {
                case PKT_WAIT_START:
                    if(c == START_BYTE)
                    {
                        pktState = PKT_READ_COUNT;
                        calculatedCRC = 0xFFFF;
                        calculatedCRC = CRC16_Update(calculatedCRC, c);
                    }
                    continue;

                case PKT_READ_COUNT:
                    if(c <= CORNER_COUNT)
                    {
                        pktCount = c;
                        pktIndex = 0;
                        bytePair = 0;
                        calculatedCRC = CRC16_Update(calculatedCRC, c);
                        pktState = PKT_READ_DATA;
                    }
                    else
                        pktState = PKT_WAIT_START;
                    continue;

                case PKT_READ_DATA:
                    calculatedCRC = CRC16_Update(calculatedCRC, c);

                    if(bytePair == 0)
                    {
                        tempProfile[pktIndex] = c;
                        bytePair = 1;
                    }
                    else
                    {
                        tempProfile[pktIndex] |= (c << 8);
                        bytePair = 0;
                        pktIndex++;

                        if(pktIndex >= pktCount)
                            pktState = PKT_READ_CRC_L;
                    }
                    continue;

                case PKT_READ_CRC_L:
                    receivedCRC = c;
                    pktState = PKT_READ_CRC_H;
                    continue;

                case PKT_READ_CRC_H:
                    receivedCRC |= (c << 8);
                    pktState = PKT_WAIT_END;
                    continue;

                case PKT_WAIT_END:
                    if(c == END_BYTE)
                    {
                        if(receivedCRC == calculatedCRC)
                        {
                            PID_SetPistCorners(tempProfile, pktCount);
                        	LED_AnimateOnce(BUZZER, LONG_BLINK);
                            UART_Println("OK");
                        }
                        else
                        {
                        	LED_AnimateOnce(BUZZER, BUZZER_ERROR);
                            UART_Println("ERR");
                        }
                    }
                    pktState = PKT_WAIT_START;
                    continue;
            }
        }

        /* ---------- TEXT COMMAND PARSER ---------- */
        if (c == '\n'){
        	LED_AnimateOnce(BLUE, BLINK);
            rxBuffer[bufferIndex] = '\0';

            bool beep = false;
            if(strcmp((char*)rxBuffer, "GET_PID") == 0){
            	uint8_t packet[1 + 1 + 1 + 12 + 2 + 1]; // START+CMD+COUNT+DATA+CRC+END
				uint16_t packetLen = buildPacket(PID_GetPIDPacket(), CMD_SETTINGS, 6, packet);

				UART_PrintBytes(packet, packetLen);
			}
            else if(strcmp((char*)rxBuffer, "PIST") == 0){
            	uint16_t buffer[CORNER_COUNT];
				uint8_t count = PID_GetPistCorners(buffer);
				sendCornersUART(buffer, count);
				beep = true;
            }
            else if(strcmp((char*)rxBuffer, "QTR8A") == 0){
				UART_CMD_SendQTR8A();
			}
            else if(strcmp((char*)rxBuffer, "TRIG") == 0){
				HCSR04_Trigger();
			}
            else if(strcmp((char*)rxBuffer, "HCSR") == 0){
            	UART_CMD_SendHCSR();
			}
            else if(strcmp((char*)rxBuffer, "ECHO") == 0){
            	UART_Println("ECHO");
			}
			else if(strcmp((char*)rxBuffer, "DRIVE") == 0){
				MENU_GoHome();
				MENU_StartDriving();
			}
			else if(strcmp((char*)rxBuffer, "TESTD") == 0){
				MENU_GoHome();
				MENU_StartTestDriving();
			}
			else if(strcmp((char*)rxBuffer, "STOP") == 0){
				MENU_GoHome();
				beep = true;
			}
			else if(strcmp((char*)rxBuffer, "CLBRT") == 0){
				MENU_GoHome();
				MENU_StartCalibration();
			}
			else if(strcmp((char*)rxBuffer, "SAVE") == 0){
				MENU_GoHome();
				MENU_Save();
			}
			else{
				int val = 0;
				char cmd = 0;

				uint8_t result = sscanf(rxBuffer, "%c:%d", &cmd, &val);

				if(result == 2){
					if(cmd == 'P'){
						PID_SetP(val);
						beep = true;
					}
					else if(cmd == 'I'){
						PID_SetI(val);
						beep = true;
					}
					else if(cmd == 'D'){
						PID_SetD(val);
						beep = true;
					}
					else if(cmd == 'S'){
						PID_SetBaseSpeed(val);
						beep = true;
					}
					else if(cmd == 'T'){
						PID_SetTurningSpeed(val);
						beep = true;
					}
					else if(cmd == 'C'){
						PID_SetCornerSpeed(val);
						beep = true;
					}
					else if(cmd == 'F'){
						if(val == 1){
							PID_SetLineBlack();
							beep = true;
						}else if(val == 2){
							PID_SetLineWhite();
							beep = true;
						}else if(val == 3){
							PID_SetLineAuto();
							beep = true;
						}else if(val == 4){
							PID_SetCornerMode(true);
							beep = true;
						}else if(val == 5){
							PID_SetCornerMode(false);
							beep = true;
						}
						else if(val == 6){
							PID_SetPist(PIST_A);
							uint16_t buffer[CORNER_COUNT];
							uint8_t count = PID_GetPistCorners(buffer);
							sendCornersUART(buffer, count);
							beep = true;
						}
						else if(val == 7){
							PID_SetPist(PIST_B);
							uint16_t buffer[CORNER_COUNT];
							uint8_t count = PID_GetPistCorners(buffer);
							sendCornersUART(buffer, count);
							beep = true;
						}
					}


				}
			}

            if(beep)
            	LED_AnimateOnce(BUZZER, SHORT_BLINK);

            bufferIndex = 0;
        }else if(bufferIndex < sizeof(rxBuffer)-1){
        	rxBuffer[bufferIndex++] = c;
        }
    }
}



