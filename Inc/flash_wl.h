#ifndef FLASH_WL_H
#define FLASH_WL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

#define FLASH_WL_BASE_ADDR 0x08060000 // Sector 7 Base Address
#define FLASH_WL_SECTOR FLASH_SECTOR_7
#define FLASH_WL_SECTOR_SIZE 131072 // 128 KB
#define FLASH_WL_RECORD_SIZE 256 // Fixed record size
#define FLASH_WL_MAX_RECORDS (FLASH_WL_SECTOR_SIZE / FLASH_WL_RECORD_SIZE)
#define CRC_LEN_BYTES 248 // CRC ve HEADER Hariç
#define CORNER_COUNT 50
// 54 byte crc ve header haric değişkenler için yer ayrıldı

#define FLASH_WL_HEADER 0xCAFEBABE
typedef struct __attribute__((packed)) {
    uint32_t header;          // 4
    uint32_t crc;             // 4
    uint16_t sensorMin[8];    // 16
    uint16_t sensorMax[8];    // 16
    uint32_t Kp;            // 4
    uint32_t Kd;            // 4
    uint16_t basespeed;			//2
    uint8_t flags;				// 1
    uint16_t cornerSpeed;		// 2
    uint16_t turningSpeed;		//2
    uint8_t distanceToCorner;	// 1
    uint16_t corners[2][CORNER_COUNT];	// 200
} WL_Record;


uint32_t WL_FindNextAddress(void);
void WL_EraseSector(void);
uint8_t WL_WriteRecord(WL_Record *rec);
uint8_t WL_ReadLatest(WL_Record *rec);
uint32_t WL_CalcCRC32(uint8_t *data, uint32_t len);

#endif
