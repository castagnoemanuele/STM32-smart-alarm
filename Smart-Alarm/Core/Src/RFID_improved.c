/**
  * Miglioramento del modulo RFID RC522 per il sistema di allarme intelligente
  */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "RFID.h"
#include "config.h"

/**
  * @brief  Inizializza il modulo RFID MFRC522 con impostazioni ottimizzate
  * @retval None
  */
void MFRC522_Init_Improved(void)
{
  // Hardware reset sequence con tempi più lunghi
  printf("Resetting RFID module...\n");
  HAL_GPIO_WritePin(RC522_Rst_GPIO_Port, RC522_Rst_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);  // Increased from 50ms to 100ms for more reliable reset
  HAL_GPIO_WritePin(RC522_Rst_GPIO_Port, RC522_Rst_Pin, GPIO_PIN_SET);
  HAL_Delay(100);  // Increased from 50ms to 100ms for proper initialization

  // Software reset
  Write_MFRC522(CommandReg, PCD_RESETPHASE);
  HAL_Delay(100); // Wait for the reset to complete
  
  // Check if device responds with proper version
  RFIDstatus = Read_MFRC522(VersionReg);
  printf("RFID Module Version: 0x%02X\n", RFIDstatus);
  
  // Expected versions: 0x91 or 0x92 for MFRC522
  if (RFIDstatus != 0x91 && RFIDstatus != 0x92 && RFIDstatus != 0x12) {
    printf("[WARNING] RFID module reports unexpected version. Check connections.\n");
  }
  
  // Timer for 100kHz
  Write_MFRC522(TModeReg, 0x8D);     // Auto Timer restart
  Write_MFRC522(TPrescalerReg, 0x3E); // Timer Frequency f = 13.56MHz/(2*TPrescaler+1)
  Write_MFRC522(TReloadRegL, 30);    // 30 reload timer
  Write_MFRC522(TReloadRegH, 0);     // Higher 8 bits = 0
  
  // Enhanced settings for better performance
  //Write_MFRC522(TxASKReg, 0x40);     // 100% ASK modulation
  Write_MFRC522(ModeReg, 0x3D);      // CRC preset value 0x6363
  Write_MFRC522(RFCfgReg, 0x70);     // Set receiver gain to maximum
  
  // Force 100% ASK modulation
  Write_MFRC522(TxAutoReg, 0x40);    // Force 100% ASK
  
  // Open the antenna with better power
  AntennaOn();
  
  // Verify communication by reading version again
  RFIDstatus = Read_MFRC522(VersionReg);
  if (RFIDstatus == 0x91 || RFIDstatus == 0x92) {
    printf("[OK] MFRC522 successfully initialized. Version: 0x%02X\n", RFIDstatus);
  } else {
    printf("[ERROR] MFRC522 communication problem. Version: 0x%02X\n", RFIDstatus);
  }
}

/**
  * @brief  Verifica se la carta è presente con un miglior controllo degli errori
  * @param  reqMode: Modalità di richiesta
  *         TagType: Tipo di tag
  * @retval Status
  */
uint8_t MFRC522_Request_Improved(uint8_t reqMode, uint8_t *TagType)
{
  uint8_t status;
  uint8_t backBits; // The received data bits

  Write_MFRC522(BitFramingReg, 0x07);   // TxLastBists = BitFramingReg[2..0]

  TagType[0] = reqMode;

  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
  
  if ((status != MI_OK) || (backBits != 0x10)) {
    uint8_t error = Read_MFRC522(ErrorReg);
    if (error) {
      // Debug: print error register if there's an issue
      printf("RFID Error: 0x%02X\n", error);
    }
    status = MI_ERR;
  }

  return status;
}

/**
  * @brief  Migliore gestione della rilevazione delle carte
  * @retval None
  */
void checkRFID_Improved(void) {
    static uint32_t lastTick = 0;
    static uint8_t messagePrinted = 0;
    static uint8_t rfidInitialized = 0;
    static uint8_t resetAttempts = 0;
    
    uint8_t status;
    uint8_t uid[5] = {0};
    uint8_t sak;
    uint8_t sectorKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t cardData[16];
    
    // Check if RFID module is properly initialized
    if (!rfidInitialized) {
        // Initialize the module with improved function
        MFRC522_Init_Improved();
        rfidInitialized = 1;
        messagePrinted = 0;
        resetAttempts = 0;
    }

    // Step 1: Look for cards (non-blocking)
    if (!messagePrinted) {
        printf("\nNo card found, waiting for card...");
        messagePrinted = 1;
    }

    // Try to detect card
    status = MFRC522_Request_Improved(PICC_REQIDL, cardstr);
    if (status == MI_OK) {
        printf("\r\nCard detected!\r\n");
        
        // Step 2: Anti-collision to get UID
        status = MFRC522_Anticoll(uid);
        if (status != MI_OK) {
            printf("Anti-collision failed.\r\n");
            return;
        }
        
        printf("UID: %02X %02X %02X %02X\r\n", uid[0], uid[1], uid[2], uid[3]);
        
        // Step 3: Select the tag
        status = MFRC522_SelectTag(uid);
        if (status == 0) {
            printf("Card select failed.\r\n");
            return;
        }
        
        // Step 4: Authentication
        status = MFRC522_Auth(0x60, 3, sectorKey, uid);
        if (status != MI_OK) {
            printf("Authentication failed.\r\n");
            MFRC522_StopCrypto1();
            return;
        }
        
        printf("Authentication OK!\r\n");
        
        // Stop crypto and halt the card
        MFRC522_StopCrypto1();
        MFRC522_Halt();
        
        // Reset the message flag to start over next time
        messagePrinted = 0;
        return;
    }
    
    // Periodic check of RFID module health
    if (HAL_GetTick() - lastTick >= 500) {
        printf(".");
        lastTick = HAL_GetTick();
        
        // Check RFID version register
        uint8_t version = Read_MFRC522(VersionReg);
        if (version == 0 || version == 0xFF) {
            resetAttempts++;
            printf("\n[WARN] RFID module not responding (version: 0x%02X). Reset attempt: %d\n", version, resetAttempts);
            
            // Reset module if we've lost communication
            if (resetAttempts <= 3) {
                rfidInitialized = 0; // Force reinitialization
            } else if (resetAttempts == 4) {
                // After 3 reset attempts, show final error message
                printf("[ERROR] RFID module failed to respond after multiple reset attempts.\n");
                printf("Check power connections and SPI wiring, then restart the device.\n");
            }
        } else if (version != RFIDstatus) {
            // Module responded but version changed
            printf("\nRFID module status changed: 0x%02X -> 0x%02X\n", RFIDstatus, version);
            RFIDstatus = version;
        }
    }
    
    // Non-blocking return
}

// Questa funzione può essere chiamata dall'applicazione principale per ottenere 
// una versione migliorata della rilevazione delle carte
void RFID_Process(void) {
    checkRFID_Improved();
}
