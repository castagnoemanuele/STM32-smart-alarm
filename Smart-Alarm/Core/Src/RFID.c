#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "RFID.h"
#include "pinCode.h"
#include "display.h"

// Global variables
uint8_t RFIDstatus = 0;
uint8_t str[MAX_LEN + 1] = {0};
int RFIDtempcount = 0;
extern PincodeState pincodeState;
volatile uint8_t isI2CBusy = 0;  // Flag to track I2C activity

/**
 * @brief Write data to a register on the MFRC522
 * @param addr - Register address
 * @param val - Value to write
 */
// Sostituisci le funzioni principali con versioni semplificate

void Write_MFRC522(uint8_t addr, uint8_t val) {
    uint8_t tx_data[2];
    tx_data[0] = (addr << 1) & 0x7E;
    tx_data[1] = val;
    
    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_SPI_Transmit(&hspi3, tx_data, 2, 200);
    HAL_Delay(2);
    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
}

uint8_t Read_MFRC522(uint8_t addr) {
    uint8_t tx_data[2] = {((addr << 1) & 0x7E) | 0x80, 0};
    uint8_t rx_data[2] = {0, 0};

    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_SPI_TransmitReceive(&hspi3, tx_data, rx_data, 2, 200);
    HAL_Delay(2);
    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    return rx_data[1];
}

void MFRC522_Init(void) {
    printf("RFID minimal init start\n");

    // Hardware reset
    HAL_GPIO_WritePin(RC522_Rst_GPIO_Port, RC522_Rst_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(RC522_Rst_GPIO_Port, RC522_Rst_Pin, GPIO_PIN_SET);
    HAL_Delay(500);

    // Read version register
    uint8_t version = Read_MFRC522(VersionReg);
    printf("Version: 0x%02X\n", version);

    if(version == 0) {
        printf("RFID not responding, check hardware\n");
        return;
    }

    // Minimal initialization
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
    HAL_Delay(200);

    Write_MFRC522(TModeReg, 0x8D);
    Write_MFRC522(TPrescalerReg, 0x3E);
    Write_MFRC522(TReloadRegL, 30);
    Write_MFRC522(TReloadRegH, 0);
    Write_MFRC522(TxAutoReg, 0x40);
    Write_MFRC522(ModeReg, 0x3D);

    // Turn on antenna
    uint8_t temp = Read_MFRC522(TxControlReg);
    Write_MFRC522(TxControlReg, temp | 0x03);
    
    printf("RFID init complete\n");
}

/**
 * @brief Set bits in a register
 * @param reg - Register address
 * @param mask - Bit mask to set
 */
void SetBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);
}

/**
 * @brief Clear bits in a register
 * @param reg - Register address
 * @param mask - Bit mask to clear
 */
void ClearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));
}

/**
 * @brief Turn on the antenna
 */
void AntennaOn(void) {
    uint8_t temp = Read_MFRC522(TxControlReg);
    if (!(temp & 0x03)) {
        SetBitMask(TxControlReg, 0x03);
    }
}

/**
 * @brief Turn off the antenna
 */
void AntennaOff(void) {
    ClearBitMask(TxControlReg, 0x03);
}

/**
 * @brief Reset the MFRC522
 */
void MFRC522_Reset(void) {
    printf("Performing software reset...\r\n");

    // Ensure CS is in high state
    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // Send reset command
    Write_MFRC522(CommandReg, PCD_RESETPHASE);

    // Wait for reset to complete
    HAL_Delay(200);

    // Verify reset was completed
    uint8_t val = Read_MFRC522(CommandReg);
    printf("After reset, CommandReg = 0x%02X (should be 0x00)\r\n", val);
}

/**
 * @brief Initialize the MFRC522
 */


/**
 * @brief Request a card in the antenna field
 * @param reqMode - Request mode (REQIDL or REQALL)
 * @param TagType - Card type
 * @return MI_OK if successful
 */
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status;
    uint8_t backBits = 0;
    
    Write_MFRC522(BitFramingReg, 0x07);  // TxLastBits = BitFramingReg[2..0]
    
    TagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
    
    if ((status != MI_OK) || (backBits != 0x10)) {
        status = MI_ERR;
    }
    
    return status;
}

/**
 * @brief Communicate with a card with retry mechanism
 * @param command - MFRC522 command
 * @param sendData - Data to send
 * @param sendLen - Length of data to send
 * @param backData - Received data
 * @param backLen - Length of received data
 * @return MI_OK if successful
 */
uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;
    uint8_t retry_count = 2;  // Retry up to 2 times
    
    while (retry_count--) {
        switch (command) {
            case PCD_AUTHENT:
                irqEn = 0x12;
                waitIRq = 0x10;
                break;
            case PCD_TRANSCEIVE:
                irqEn = 0x77;
                waitIRq = 0x30;
                break;
            default:
                break;
        }

        Write_MFRC522(CommIEnReg, irqEn | 0x80);  // Interrupt request
        ClearBitMask(CommIrqReg, 0x80);           // Clear all interrupt request bits
        SetBitMask(FIFOLevelReg, 0x80);           // FlushBuffer=1, FIFO initialization

        Write_MFRC522(CommandReg, PCD_IDLE);      // Cancel current command

        // Write data to the FIFO
        for (i = 0; i < sendLen; i++) {
            Write_MFRC522(FIFODataReg, sendData[i]);
        }

        // Execute the command
        Write_MFRC522(CommandReg, command);
        if (command == PCD_TRANSCEIVE) {
            SetBitMask(BitFramingReg, 0x80);      // StartSend=1, transmission of data starts
        }

        // Wait for the command to complete (with timeout)
        i = 1000;  // Appropriate timeout value
        do {
            // Short delay to let the chip process
            HAL_Delay(1);
            n = Read_MFRC522(CommIrqReg);
            i--;
        } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

        ClearBitMask(BitFramingReg, 0x80);        // StartSend=0

        if (i != 0) {
            if (!(Read_MFRC522(ErrorReg) & 0x1B)) {  // BufferOvfl, ColErr, CRCErr, ProtocolErr
                status = MI_OK;
                if (n & irqEn & 0x01) {
                    status = MI_NOTAGERR;
                }
                
                if (command == PCD_TRANSCEIVE) {
                    n = Read_MFRC522(FIFOLevelReg);
                    lastBits = Read_MFRC522(ControlReg) & 0x07;
                    if (lastBits) {
                        *backLen = (n - 1) * 8 + lastBits;
                    } else {
                        *backLen = n * 8;
                    }

                    if (n == 0) {
                        n = 1;
                    }
                    if (n > MAX_LEN) {
                        n = MAX_LEN;
                    }

                    // Read the received data from FIFO
                    for (i = 0; i < n; i++) {
                        backData[i] = Read_MFRC522(FIFODataReg);
                    }
                }
                
                // Operation successful, exit loop
                break;
            } else {
                printf("Error register: 0x%02X, retrying...\r\n", Read_MFRC522(ErrorReg));
                status = MI_ERR;

                // Reset the command register before retrying
                if (retry_count > 0) {
                    Write_MFRC522(CommandReg, PCD_RESETPHASE);
                    HAL_Delay(10);
                }
            }
        } else {
            printf("Timeout waiting for command completion\r\n");
            if (retry_count > 0) {
                // Try to recover from timeout
                Write_MFRC522(CommandReg, PCD_RESETPHASE);
                HAL_Delay(10);
                printf("Retrying operation...\r\n");
            }
        }
    }
    
    return status;
}

/**
 * @brief Anti-collision detection
 * @param serNum - Card serial number
 * @return MI_OK if successful
 */
uint8_t MFRC522_Anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint8_t unLen;
    
    Write_MFRC522(BitFramingReg, 0x00);  // TxLastBits = BitFramingReg[2..0]
    
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
    
    if (status == MI_OK) {
        // Check card serial number
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i]) {
            status = MI_ERR;
        }
    }
    
    return status;
}

/**
 * @brief Calculate CRC
 * @param pIndata - Input data
 * @param len - Data length
 * @param pOutData - CRC result
 */
void CalulateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData) {
    uint8_t i, n;
    
    ClearBitMask(DivIrqReg, 0x04);      // CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);     // Clear the FIFO pointer
    
    // Write data to FIFO
    for (i = 0; i < len; i++) {
        Write_MFRC522(FIFODataReg, *(pIndata + i));
    }
    Write_MFRC522(CommandReg, PCD_CALCCRC);
    
    // Wait for CRC calculation to complete
    i = 255;
    do {
        n = Read_MFRC522(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));  // CRCIrq = 1
    
    // Read CRC calculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}

/**
 * @brief Select a card
 * @param serNum - Card serial number
 * @return Card capacity
 */
uint8_t MFRC522_SelectTag(uint8_t *serNum) {
    uint8_t i;
    uint8_t status;
    uint8_t size;
    uint8_t recvBits;
    uint8_t buffer[9];
    
    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i = 0; i < 5; i++) {
        buffer[i + 2] = *(serNum + i);
    }
    CalulateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    
    if ((status == MI_OK) && (recvBits == 0x18)) {
        size = buffer[0];
    } else {
        size = 0;
    }
    
    return size;
}

/**
 * @brief Authenticate a card
 * @param authMode - Authentication mode (0x60: Key A, 0x61: Key B)
 * @param BlockAddr - Block address
 * @param Sectorkey - Sector key
 * @param serNum - Card serial number
 * @return MI_OK if successful
 */
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t *Sectorkey, uint8_t *serNum) {
    uint8_t status;
    uint8_t recvBits;
    uint8_t i;
    uint8_t buff[12];
    
    // Build command buffer
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i = 0; i < 6; i++) {
        buff[i + 2] = *(Sectorkey + i);
    }
    for (i = 0; i < 4; i++) {
        buff[i + 8] = *(serNum + i);
    }
    
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);
    
    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08))) {
        status = MI_ERR;
    }
    
    return status;
}

/**
 * @brief Read data from a card block
 * @param blockAddr - Block address
 * @param recvData - Received data
 * @return MI_OK if successful
 */
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t *recvData) {
    uint8_t status;
    uint8_t unLen;
    
    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData, 2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
    
    if ((status != MI_OK) || (unLen != 0x90)) {
        status = MI_ERR;
    }
    
    return status;
}

/**
 * @brief Write data to a card block
 * @param blockAddr - Block address
 * @param writeData - Data to write
 * @return MI_OK if successful
 */
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t *writeData) {
    uint8_t status;
    uint8_t recvBits;
    uint8_t i;
    uint8_t buff[18];
    
    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);
    
    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
        status = MI_ERR;
    }
    
    if (status == MI_OK) {
        // Write data to FIFO
        for (i = 0; i < 16; i++) {
            buff[i] = *(writeData + i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
        
        if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
            status = MI_ERR;
        }
    }
    
    return status;
}

/**
 * @brief Halt a PICC
 */
void MFRC522_Halt(void) {
    uint8_t unLen;
    uint8_t buff[4];
    
    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);
    MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

/**
 * @brief Stop Crypto1 encryption
 */
void MFRC522_StopCrypto1(void) {
    ClearBitMask(Status2Reg, 0x08);
}

/**
 * @brief Test SPI communication with the MFRC522
 * @return 1 if communication is working, 0 otherwise
 */
uint8_t MFRC522_SelfTest(void) {
    uint8_t testVal = 0x55;
    uint8_t readVal;
    
    // Write a test value to a safe register (FIFODataReg)
    Write_MFRC522(FIFODataReg, testVal);
    
    // Read back the value
    readVal = Read_MFRC522(FIFODataReg);
    
    // Check if the read value matches the written value
    if (readVal == testVal) {
        printf("SPI communication test passed!\r\n");
        return 1;
    } else {
        printf("SPI communication test failed! Wrote 0x%02X, read 0x%02X\r\n", testVal, readVal);
        return 0;
    }
}

/**
 * @brief Reinitialize RFID module after potential issues
 * @return 1 if reinitialization succeeded, 0 otherwise
 */
uint8_t MFRC522_Reinitialize(void) {
    printf("Reinitializing RFID module...\r\n");

    // Full reset sequence
    MFRC522_Init();

    // Test if communication is working
    return MFRC522_SelfTest();
}

/**
 * @brief Check for RFID cards
 */
void checkRFID(void) {
    uint8_t status;
    uint8_t uid[5] = {0};
    uint8_t cardType[2];
    static uint32_t lastCheckTime = 0;
    static uint8_t initDone = 0;
    static uint8_t errorCounter = 0;
    
    // Perform SPI communication test only once
    if (!initDone) {
        printf("Testing RFID module communication...\r\n");
        if (MFRC522_SelfTest()) {
            printf("RFID module ready!\r\n");
            initDone = 1;
        } else {
            printf("RFID module not responding correctly. Check connections.\r\n");
            // Try to initialize again if we haven't tried too many times
            if (errorCounter < 3) {
                errorCounter++;
                HAL_Delay(500);  // Wait before retry
                return;
            } else {
                printf("Failed to initialize RFID after multiple attempts.\r\n");
                initDone = 1;  // Mark as initialized to avoid endless retries
            }
        }
    }
    
    // Non-blocking check for cards (don't run too frequently)
    if (HAL_GetTick() - lastCheckTime < 200) {
        return;
    }
    lastCheckTime = HAL_GetTick();
    
    // Step 1: Look for cards
    status = MFRC522_Request(PICC_REQIDL, cardType);
    if (status != MI_OK) {
        // Reset error counter if no card is detected (normal condition)
        errorCounter = 0;
        return;  // No card found
    }
    
    // Step 2: Get the card's UID
    status = MFRC522_Anticoll(uid);
    if (status != MI_OK) {
        // If anti-collision failed, increment error counter
        errorCounter++;
        if (errorCounter > 10) {  // After 10 consecutive errors
            if (MFRC522_Reinitialize()) {
                printf("RFID module successfully reinitialized!\r\n");
            } else {
                printf("RFID reinitialization failed!\r\n");
            }
            errorCounter = 0;
        }
        return;  // Anti-collision failed
    }
    
    // Reset error counter on successful detection
    errorCounter = 0;

    printf("Card detected! UID: %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
    
    // When a card is detected, display pin entry screen
    printf("Card detected. Please enter PIN.\r\n");
    Display_EnterPincode(pincodeState.pincode_position, pincodeState.entered_pincode);
    
    // Step 3: Select the card
    uint8_t size = MFRC522_SelectTag(uid);
    printf("Card size: %d bytes\r\n", size);
    
    // Halt PICC and stop encryption
    MFRC522_Halt();
    MFRC522_StopCrypto1();
}
