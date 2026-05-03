#include "flash_wl.h"

uint32_t WL_CalcCRC32(uint8_t *data, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFF;
	for (uint32_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
		crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
	}
	return ~crc;
}


uint32_t WL_FindNextAddress(void)
{
	for (uint32_t i = 0; i < FLASH_WL_MAX_RECORDS; i++) {
		uint32_t addr = FLASH_WL_BASE_ADDR + (i * FLASH_WL_RECORD_SIZE);
		uint32_t header = *(uint32_t*)addr;
		if (header != FLASH_WL_HEADER) return addr;
	}
	return 0xFFFFFFFF;
}


void WL_EraseSector(void)
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_WL_SECTOR, VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}


uint8_t WL_WriteRecord(WL_Record *rec)
{
	uint32_t addr = WL_FindNextAddress();

	HAL_FLASH_Unlock();

	if (addr == 0xFFFFFFFF) {
		WL_EraseSector();
		addr = FLASH_WL_BASE_ADDR;
	}

	rec->header = FLASH_WL_HEADER;
	rec->crc = WL_CalcCRC32((uint8_t*)rec->sensorMin, CRC_LEN_BYTES);

	uint8_t *ptr = (uint8_t*)rec;
	HAL_StatusTypeDef halres = HAL_OK;

	for (uint32_t i = 0; i < FLASH_WL_RECORD_SIZE; i += 4) {
		uint32_t word;
		memcpy(&word, ptr + i, 4);
		halres = HAL_FLASH_Program(TYPEPROGRAM_WORD, addr + i, word);
		if (halres != HAL_OK) {
			HAL_FLASH_Lock();
			return 0; // yazma başarısız
		}
	}

	HAL_FLASH_Lock();
	return 1;
}


uint8_t WL_ReadLatest(WL_Record *rec)
{
	uint32_t lastValid = 0;

	for (uint32_t i = 0; i < FLASH_WL_MAX_RECORDS; i++) {
		uint32_t addr = FLASH_WL_BASE_ADDR + (i * FLASH_WL_RECORD_SIZE);
		uint32_t header = *(uint32_t*)addr;

		if (header == FLASH_WL_HEADER)
			lastValid = addr;
		else
			break;
	}

	if (lastValid == 0) return 0;

	memcpy(rec, (void*)lastValid, sizeof(WL_Record));

	uint32_t calc = WL_CalcCRC32((uint8_t*)rec->sensorMin, CRC_LEN_BYTES);

	if (calc != rec->crc) return 0;


	return 1;
}
