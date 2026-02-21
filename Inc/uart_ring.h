#ifndef UART_RING_H
#define UART_RING_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define UART_RX_BUF_SIZE 128

void     UART_Ring_Init(UART_HandleTypeDef *huart);
uint8_t  UART_Available(void);
uint8_t  UART_Read(void);

void UART_Print(const char *str);
void UART_Println(const char *str);
void UART_PrintBytes(uint8_t *data, uint16_t len);
void UART_Callback(UART_HandleTypeDef *huart);

#endif
