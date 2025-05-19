#ifndef PN532_H
#define PN532_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Default I2C address for the PN532 (7-bit address shifted for STM32 HAL).
 */
#define PN532_I2C_ADDR         (0x24 << 1)

/**
 * @brief PN532 command definitions.
 */
#define PN532_COMMAND_GETFIRMWAREVERSION   0x02
#define PN532_COMMAND_SAMCONFIGURATION     0x14
#define PN532_COMMAND_INLISTPASSIVETARGET  0x4A

/**
 * @brief Initializes the PN532 module.
 * @param hi2c Pointer to the I2C handle.
 * @retval true if initialization is successful, false otherwise.
 */
bool PN532_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Reads an NFC tag and retrieves its UID.
 * @param uid Pointer to buffer where the UID will be stored.
 * @param uidLength Pointer to variable where the UID length will be stored.
 * @retval true if a tag is detected and UID is read, false otherwise.
 */
bool PN532_ReadPassiveTarget(uint8_t *uid, uint8_t *uidLength);

/**
 * @brief Test function for PN532. Waits for an NFC tag and prints its UID to the serial output.
 */
void PN532_Test(void);

bool PN532_ReleaseTarget(void);

#endif // PN532_H
