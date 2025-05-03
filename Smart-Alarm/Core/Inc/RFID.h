#ifndef __RFID_H
#define __RFID_H

#include "stm32f4xx_hal.h"

// RFID Command Registers
#define RFID_IDLE              0x00   // Command: No action; Cancel current command
#define RFID_AUTHENT           0x0E   // Command: Perform authentication using KeyA or KeyB
#define RFID_RECEIVE           0x08   // Command: Receive data
#define RFID_TRANSMIT          0x04   // Command: Transmit data
#define RFID_TRANSCEIVE        0x0C   // Command: Transmit and receive data
#define RFID_RESET             0x0F   // Command: Reset chip
#define RFID_CALCCRC           0x03   // Command: CRC Calculation

// RFID FIFO Size
#define MAX_LEN 16

// Function prototypes
void RFID_Init(SPI_HandleTypeDef *spiHandle);
void RFID_WriteRegister(uint8_t addr, uint8_t val);
uint8_t RFID_ReadRegister(uint8_t addr);
void RFID_AntennaOn(void);
void RFID_Reset(void);
uint8_t RFID_Request(uint8_t reqMode, uint8_t *TagType);
uint8_t RFID_Anticoll(uint8_t *serNum);
uint8_t RFID_SelectTag(uint8_t *serNum);
void RFID_Halt(void);

#endif /* __RFID_H */
