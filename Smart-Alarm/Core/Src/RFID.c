#include "rfid.h"

// Local SPI handle
static SPI_HandleTypeDef *hspi;

// RFID SPI communication functions
void RFID_WriteRegister(uint8_t addr, uint8_t val) {
    uint8_t data[2] = {(addr << 1) & 0x7E, val};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Pull CS low
    HAL_SPI_Transmit(hspi, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Pull CS high
}

uint8_t RFID_ReadRegister(uint8_t addr) {
    uint8_t data[2] = {((addr << 1) & 0x7E) | 0x80, 0x00};
    uint8_t response;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Pull CS low
    HAL_SPI_Transmit(hspi, data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &response, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Pull CS high
    return response;
}

// RFID initialization
void RFID_Init(SPI_HandleTypeDef *spiHandle) {
    hspi = spiHandle;

    RFID_Reset();
    RFID_WriteRegister(0x2A, 0x8D); // Timer: TPrescaler*TreloadVal/6.78MHz = 25ms
    RFID_WriteRegister(0x2B, 0x3E);
    RFID_WriteRegister(0x2C, 30);
    RFID_WriteRegister(0x2D, 0);
    RFID_WriteRegister(0x15, 0x40); // 100% ASK
    RFID_WriteRegister(0x11, 0x3D); // CRC initial value 0x6363
    RFID_AntennaOn();
}

void RFID_Reset(void) {
    RFID_WriteRegister(0x01, RFID_RESET);
}

void RFID_AntennaOn(void) {
    uint8_t temp = RFID_ReadRegister(0x14);
    if (!(temp & 0x03)) {
        RFID_WriteRegister(0x14, temp | 0x03);
    }
}

uint8_t RFID_Request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status;
    uint8_t backBits; // The received bit count

    RFID_WriteRegister(0x0D, 0x07); // TxControlReg - Transmit the data in the FIFO
    RFID_WriteRegister(0x09, reqMode);
    status = RFID_Communicate(RFID_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != 0) || (backBits != 0x10)) {
        status = 1; // Error
    }
    return status;
}

uint8_t RFID_Anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint8_t unLen;

    RFID_WriteRegister(0x0D, 0x00); // Clear FIFO buffer
    serNum[0] = 0x93;               // Request Standard Mode
    serNum[1] = 0x20;
    status = RFID_Communicate(RFID_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == 0) {
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[4]) {
            status = 1; // Error
        }
    }
    return status;
}

uint8_t RFID_SelectTag(uint8_t *serNum) {
    uint8_t status;
    uint8_t recvBits;
    uint8_t buffer[9];
    uint8_t i;

    buffer[0] = 0x93;
    buffer[1] = 0x70;
    for (i = 0; i < 5; i++) {
        buffer[i + 2] = serNum[i];
    }
    RFID_CalculateCRC(buffer, 7, &buffer[7]);
    status = RFID_Communicate(RFID_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status != 0) || (recvBits != 0x18)) {
        status = 1; // Error
    }

    return status;
}

void RFID_Halt(void) {
    uint8_t buffer[4];

    buffer[0] = 0x50; // Command: Halt
    buffer[1] = 0x00;
    RFID_CalculateCRC(buffer, 2, &buffer[2]);

    RFID_Communicate(RFID_TRANSCEIVE, buffer, 4, buffer, NULL);
}
