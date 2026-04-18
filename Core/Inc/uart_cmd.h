#ifndef UART_CMD
#define UART_CMD

#define START_BYTE 0xAA
#define END_BYTE   0x55


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
#include "encoder.h"
#include "menu.h"
#include "robot.h"
#include "uart_ring.h"
#include <stdbool.h>

typedef enum {
    PKT_WAIT_START,
    PKT_READ_COUNT,
    PKT_READ_DATA,
    PKT_READ_CRC_L,
    PKT_READ_CRC_H,
    PKT_WAIT_END
} PacketState_t;

void UART_CMD_Process();
void UART_CMD_SendFinishTime(uint32_t time);

#endif
