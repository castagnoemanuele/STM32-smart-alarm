#include "pn532.h"
#include "stdio.h"
#include "string.h"

static I2C_HandleTypeDef *pn532_hi2c = NULL;

// PN532 frame structure constants
#define PN532_PREAMBLE      0x00
#define PN532_STARTCODE1    0x00
#define PN532_STARTCODE2    0xFF
#define PN532_POSTAMBLE     0x00
#define PN532_HOSTTOPN532   0xD4
#define PN532_PN532TOHOST   0xD5

static bool pn532_writeCommand(const uint8_t *cmd, uint8_t cmdlen);
static bool pn532_readResponse(uint8_t *response, uint8_t len, uint32_t timeout);

/**
 * @brief  Initializes the PN532 module by requesting firmware version and configuring SAM.
 * @param  hi2c: Pointer to the I2C handle to be used for communication.
 * @retval true if initialization is successful, false otherwise.
 */
bool PN532_Init(I2C_HandleTypeDef *hi2c) {
    pn532_hi2c = hi2c;
    uint8_t cmd[2] = {PN532_COMMAND_GETFIRMWAREVERSION};
    uint8_t resp[12];

    HAL_Delay(100);

    if (!pn532_writeCommand(cmd, 1)) return false;
    if (!pn532_readResponse(resp, 12, 100)) return false;

    // Configure SAM (Secure Access Module)
    uint8_t sam[4] = {PN532_COMMAND_SAMCONFIGURATION, 0x01, 0x14, 0x01};
    if (!pn532_writeCommand(sam, 4)) return false;
    if (!pn532_readResponse(resp, 8, 100)) return false;

    return true;
}

/**
 * @brief  Reads an NFC tag and retrieves its UID (up to 7 bytes).
 * @param  uid: Pointer to buffer where the UID will be stored.
 * @param  uidLength: Pointer to variable where the UID length will be stored.
 * @retval true if a tag is detected and UID is read, false otherwise.
 */
bool PN532_ReadPassiveTarget(uint8_t *uid, uint8_t *uidLength) {
    uint8_t cmd[3] = {PN532_COMMAND_INLISTPASSIVETARGET, 0x01, 0x00};
    uint8_t resp[24];

    memset(resp, 0, sizeof(resp));

    if (!pn532_writeCommand(cmd, 3)) return false;
    if (!pn532_readResponse(resp, sizeof(resp), 300)) return false;

    // Check number of targets found
    if (resp[8] != 1 || resp[12] == 0) return false; // No tag present
    // Debug: print raw response
    printf("PN532 raw response: ");
    for (int i = 0; i < 24; i++) printf("%02X ", resp[i]);
    printf("\r\n");



    *uidLength = resp[12];
    if (*uidLength < 4 || *uidLength > 7) return false;

    memcpy(uid, &resp[13], *uidLength);
    return true;
}

/**
 * @brief  Test function for PN532. Waits for an NFC tag and prints its UID to the serial output.
 */
void PN532_Test(void) {
    uint8_t uid[7];
    uint8_t uidLen = 0;
    printf("Initializing PN532...\r\n");

    printf("Place an NFC tag near the reader...\r\n");
    while (1) {
        if (PN532_ReadPassiveTarget(uid, &uidLen)) {
            printf("NFC tag detected! UID: ");
            for (uint8_t i = 0; i < uidLen; i++) {
                printf("%02X ", uid[i]);
            }
            printf("\r\n");
            break;
        }
        HAL_Delay(500);
    }
}

/**
 * @brief  Constructs and transmits a command frame to the PN532 over I2C.
 * @param  cmd: Pointer to the command buffer.
 * @param  cmdlen: Length of the command.
 * @retval true if transmission is successful, false otherwise.
 */
static bool pn532_writeCommand(const uint8_t *cmd, uint8_t cmdlen) {
    uint8_t frame[32] = {0};
    uint8_t len = cmdlen + 1;
    uint8_t checksum = 0;
    uint8_t i = 0, idx = 0;

    frame[idx++] = PN532_PREAMBLE;
    frame[idx++] = PN532_STARTCODE1;
    frame[idx++] = PN532_STARTCODE2;
    frame[idx++] = len;
    frame[idx++] = ~len + 1;
    frame[idx++] = PN532_HOSTTOPN532;
    checksum += PN532_HOSTTOPN532;

    for (i = 0; i < cmdlen; i++) {
        frame[idx++] = cmd[i];
        checksum += cmd[i];
    }
    frame[idx++] = ~checksum + 1;
    frame[idx++] = PN532_POSTAMBLE;

    // The PN532 I2C protocol requires a leading 0x00 byte ("host to PN532" indicator)
    uint8_t i2cbuf[40];
    i2cbuf[0] = 0x00;
    memcpy(&i2cbuf[1], frame, idx);

    if (HAL_I2C_Master_Transmit(pn532_hi2c, PN532_I2C_ADDR, i2cbuf, idx + 1, 100) != HAL_OK)
        return false;
    return true;
}

/**
 * @brief  Receives a response frame from the PN532 over I2C.
 * @param  response: Pointer to the buffer where the response will be stored.
 * @param  len: Expected length of the response.
 * @param  timeout: Timeout duration in milliseconds.
 * @retval true if a valid response is received, false otherwise.
 */
static bool pn532_readResponse(uint8_t *response, uint8_t len, uint32_t timeout) {
    uint32_t tickstart = HAL_GetTick();
    uint8_t buf[32];

    while (HAL_GetTick() - tickstart < timeout) {
        HAL_Delay(10);
        if (HAL_I2C_Master_Receive(pn532_hi2c, PN532_I2C_ADDR, buf, len, 100) == HAL_OK) {
            // Search for the start code in the received buffer
            for (uint8_t i = 0; i < len - 2; i++) {
                if (buf[i] == PN532_STARTCODE1 && buf[i + 1] == PN532_STARTCODE2) {
                    memcpy(response, buf, len);
                    return true;
                }
            }
        }
    }
    return false;
}


bool PN532_ReleaseTarget(void) {
    uint8_t cmd[2] = {0x52, 0x01}; // InRelease, target 1
    uint8_t resp[8];
    if (!pn532_writeCommand(cmd, 2)) return false;
    if (!pn532_readResponse(resp, sizeof(resp), 100)) return false;
    return true;
}
