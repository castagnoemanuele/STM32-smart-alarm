#include "display.h"
#include <stdio.h>

/* Private variables */
extern I2C_HandleTypeDef hi2c1;

void Display_Init(void) {
    ssd1306_Init();
}

void Display_BootMessage(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("Smart Alarm System", Font_7x10, White);

    ssd1306_SetCursor(30, 30);
    ssd1306_WriteString("Booting up...", Font_6x8, White);
    Display_Update();
    HAL_Delay(1500);

    ssd1306_Fill(White);
    ssd1306_SetCursor(30, 26);
    ssd1306_WriteString("System Ready", Font_7x10, Black);
    Display_Update();
    HAL_Delay(1000);

    Display_Clear();
}

void Display_SystemStatus(bool system_armed) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("System Status:", Font_7x10, White);

    ssd1306_SetCursor(10, 25);
    if (system_armed) {
        ssd1306_WriteString("ARMED", Font_7x10, White);
    } else {
        ssd1306_WriteString("DISARMED", Font_7x10, White);
    }

    ssd1306_SetCursor(10, 40);
    ssd1306_WriteString("Press A to arm", Font_6x8, White);
    ssd1306_SetCursor(10, 50);
    ssd1306_WriteString("Press B to disarm", Font_6x8, White);

    Display_Update();
}

void Display_LockedScreen(uint32_t remaining_seconds) {
    char msg[30];

    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("SYSTEM LOCKED", Font_7x10, White);

    sprintf(msg, "Wait %lu seconds", remaining_seconds);
    ssd1306_SetCursor(10, 30);
    ssd1306_WriteString(msg, Font_7x10, White);

    Display_Update();
}

void Display_EnterPincode(uint8_t pincode_position, const char* entered_pincode) {
    char display[PINCODE_LENGTH + 1] = {0};

    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("Enter PINCODE:", Font_7x10, White);

    for (uint8_t i = 0; i < pincode_position; i++) {
        display[i] = '*';
    }

    ssd1306_SetCursor(50, 30);
    ssd1306_WriteString(display, Font_11x18, White);

    Display_Update();
}

void Display_AccessGranted(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 20);
    ssd1306_WriteString("ACCESS", Font_11x18, White);
    ssd1306_SetCursor(30, 40);
    ssd1306_WriteString("GRANTED", Font_11x18, White);
    Display_Update();
    HAL_Delay(2000);
}

void Display_AccessDenied(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 20);
    ssd1306_WriteString("ACCESS", Font_11x18, White);
    ssd1306_SetCursor(30, 40);
    ssd1306_WriteString("DENIED", Font_11x18, White);
    Display_Update();
    HAL_Delay(2000);
}

void Display_AlarmScreen(bool toggle) {
    ssd1306_Fill(toggle ? Black : White);
    ssd1306_SetCursor(35, 26);
    ssd1306_WriteString("ALARM!", Font_7x10, toggle ? White : Black);
    Display_Update();
}

void Display_IPAddress(const char* ip) {
    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("IP:", Font_7x10, White);
    ssd1306_SetCursor(30, 40);
    ssd1306_WriteString(ip, Font_7x10, White);
    Display_Update();
    HAL_Delay(1000);
}

void Display_I2CScanResult(uint8_t address, uint8_t position) {
    char buffer[16];
    sprintf(buffer, "Found: 0x%02X", address);
    ssd1306_SetCursor(0, 16 + position * 10);
    ssd1306_WriteString(buffer, Font_6x8, White);
    Display_Update();
    HAL_Delay(1000);
}

void Display_Clear(void) {
    ssd1306_Fill(Black);
}

void Display_Update(void) {
    ssd1306_UpdateScreen();
}
