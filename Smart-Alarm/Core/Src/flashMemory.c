#include "flashMemory.h"
#include <stdint.h>           // <-- Don't forget him here too!
#include "stm32f4xx_hal.h"    // Required for HAL_FLASH functions

void SaveToFlash(uint32_t data) {
    uint32_t address = Address1;


    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    uint32_t SectorError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = FLASH_SECTOR_6;
    eraseInit.NbSectors = 1;

    HAL_FLASHEx_Erase(&eraseInit, &SectorError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);

    HAL_FLASH_Lock();
}

/**
 * @brief Reads a 32-bit unsigned integer from flash memory.
 * @return The 32-bit unsigned integer retrieved from flash memory.
 */
uint32_t ReadFromFlash(void) {
	 return *(uint32_t*) 0x08040000;
}
