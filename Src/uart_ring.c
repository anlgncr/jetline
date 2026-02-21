#include "uart_ring.h"
#include "string.h"

static UART_HandleTypeDef *uart;

static uint8_t rxBuf[UART_RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;

static uint8_t rxByte;   // interrupt için 1 byte

void UART_Ring_Init(UART_HandleTypeDef *huart)
{
    uart = huart;
    HAL_UART_Receive_IT(uart, &rxByte, 1);
}

uint8_t UART_Available(void)
{
    return (rxHead != rxTail);
}

uint8_t UART_Read(void)
{
    uint8_t data = 0;

    if (rxHead != rxTail)
    {
        data = rxBuf[rxTail];
        rxTail = (rxTail + 1) % UART_RX_BUF_SIZE;
    }

    return data;
}

void UART_Print(const char *str)
{
    if (str == NULL) return;
    HAL_UART_Transmit(uart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void UART_Println(const char *str)
{
    if (str != NULL)
    {
        HAL_UART_Transmit(uart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
    }

    uint8_t lf = '\n';
    HAL_UART_Transmit(uart, &lf, 1, HAL_MAX_DELAY);
}

void UART_PrintBytes(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;
    HAL_UART_Transmit(uart, data, len, HAL_MAX_DELAY);
}

/* HAL RX Callback */
void UART_Callback(UART_HandleTypeDef *huart)
{
    if (huart == uart)
    {
        uint16_t nextHead = (rxHead + 1) % UART_RX_BUF_SIZE;

        if (nextHead != rxTail)
        {
            rxBuf[rxHead] = rxByte;
            rxHead = nextHead;
        }
        // else: buffer dolu, byte düşer

        HAL_UART_Receive_IT(uart, &rxByte, 1);
    }
}
