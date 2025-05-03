#ifndef CONFIG_H
#define CONFIG_H

/* System Configuration */
#define ALARM_DURATION_MS 5000
#define PINCODE_LENGTH 4
#define MAX_PIN_ATTEMPTS 3
#define LOCKOUT_TIME_MS 30000
#define KEY_DEBOUNCE_MS 500
#define REED_DEBOUNCE_MS 200

//ESP32 Communication related
#define MAX_CONNECTION_ATTEMPTS 3
#define DATA_SIZE 20
#define I2C_ADDR 0x5C

/* Pin Definitions */
#define BUTTON_PIN         GPIO_PIN_13
#define REED_SWITCH_PIN    GPIO_PIN_8
#define BUZZER_PIN         GPIO_PIN_10
#define ESP32_EN           GPIO_PIN_5

// Integrated LED
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA

// Keypad Rows (PB0, PB1, PB2, PB10)
#define ROW1_PIN   GPIO_PIN_0
#define ROW2_PIN   GPIO_PIN_1
#define ROW3_PIN   GPIO_PIN_2
#define ROW4_PIN   GPIO_PIN_10
#define ROW_PORT   GPIOB

// Keypad Columns (PC6, PC7, PC4, PC5)
#define COL1_PIN   GPIO_PIN_6
#define COL2_PIN   GPIO_PIN_7
#define COL3_PIN   GPIO_PIN_4
#define COL4_PIN   GPIO_PIN_5
#define COL_PORT   GPIOC

// Keypad matrix definition
static const char keypad[4][4] = { { '1', '2', '3', 'A' },
		{ '4', '5', '6', 'B' }, { '7', '8', '9', 'C' }, { '*', '0', '#', 'D' } };

#endif /* CONFIG_H */
