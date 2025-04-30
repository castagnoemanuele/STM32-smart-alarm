/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Smart Alarm System with Reed Switch
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "display.h"
#include "flashMemory.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t alarm_flag = 0;
volatile uint8_t key_pressed = 0;
volatile uint8_t reed_triggered = 0;
char last_key = '\0';
uint32_t last_key_time = 0;
uint32_t last_reed_time = 0;

char entered_pincode[PINCODE_LENGTH + 1] = { 0 };
uint8_t pincode_position = 0;
uint8_t pin_attempts = 0;
uint32_t lockout_start = 0;
bool system_locked = false;
bool system_armed = false;

char rx_data[DATA_SIZE];
char device_ip[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int ScanI2CDevices(void);
bool is_valid_ip(const char *ip);
char scan_keypad(void);
void handle_keypress(char key);
void check_pincode(void);
void TriggerAlarm(void);
void StopAlarm(void);
void HandleReedSwitch(void);
void esp32getIP(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	Display_Init();
	Display_BootMessage();

	// Enable ESP32
	HAL_GPIO_WritePin(GPIOB, ESP32_EN, GPIO_PIN_SET);
	HAL_Delay(5000); // Give time to the ESP to boot

	// Read saved pincode from flash
	uint32_t saved_pincode = ReadFromFlash(address1);
	char pincode_str[12];
	sprintf(pincode_str, "%lu", (unsigned long) saved_pincode);

	// Scan I2C devices
	ScanI2CDevices();
	HAL_Delay(1000);

	// Get IP from ESP32
	esp32getIP();

	HAL_Delay(1000);
	// Show system status
	Display_SystemStatus(system_armed);
	uint32_t start_time = 0;
	bool toggle = false;
	char pressed_key = '\0';
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* Main loop */

	while (1) {
		// Check if system is locked
		if (system_locked) {
			uint32_t remaining = (LOCKOUT_TIME_MS
					- (HAL_GetTick() - lockout_start)) / 1000;
			Display_LockedScreen(remaining);

			if ((HAL_GetTick() - lockout_start) >= LOCKOUT_TIME_MS) {
				system_locked = false;
				pin_attempts = 0;
				Display_SystemStatus(system_armed);
			}
			HAL_Delay(100);
			continue;
		}

		// Scan keypad
		pressed_key = scan_keypad();
		if (pressed_key != '\0') {
			handle_keypress(pressed_key);
		}

		// Check reed switch status
		HandleReedSwitch();

		// Handle alarm
		if (alarm_flag) {
			if (start_time == 0) {
				start_time = HAL_GetTick();
			}

			if (HAL_GetTick() - start_time < ALARM_DURATION_MS) {
				HAL_GPIO_TogglePin(GPIOA, BUZZER_PIN);
				Display_AlarmScreen(toggle);
				toggle = !toggle;
				HAL_Delay(100);
			} else {
				StopAlarm();
				start_time = 0;
			}
		}

		HAL_Delay(10);
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_5,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC4 PC5 PC6 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* Configure PA8 as input for reed switch with interrupt */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge (reed switch closed)
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Pull-down when reed switch is open
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Enable and set EXTI interrupt priority */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(BUTTON_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);
		if (system_armed) {
			TriggerAlarm();
		}
	}
}

void EXTI9_5_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(REED_SWITCH_PIN) != RESET) {
		if ((HAL_GetTick() - last_reed_time) > REED_DEBOUNCE_MS) {
			last_reed_time = HAL_GetTick();
			__HAL_GPIO_EXTI_CLEAR_IT(REED_SWITCH_PIN);
			reed_triggered = 1;
		}
	}
}

void HandleReedSwitch(void) {
	if (reed_triggered && system_armed) {
		reed_triggered = 0;
		TriggerAlarm();
	}
}

void TriggerAlarm(void) {
	alarm_flag = 1;
	HAL_GPIO_WritePin(GPIOA, BUZZER_PIN, GPIO_PIN_SET);
}

void StopAlarm(void) {
	alarm_flag = 0;
	HAL_GPIO_WritePin(GPIOA, BUZZER_PIN, GPIO_PIN_RESET);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	Display_SystemStatus(system_armed);
}

int ScanI2CDevices(void) {
	const char StartMSG[] = "Starting I2C scan...\r\n";
	const char EndMSG[] = "I2C scan complete.\r\n";
	const uint8_t Space[] = " ";
	char Buffer[32];

	uint8_t i, ret;
	char buffer[16];
	int found = 0;

	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("I2C Scan:", Font_7x10, White);
	ssd1306_UpdateScreen();

	HAL_Delay(500);

	printf("%s", StartMSG);

	for (i = 1; i < 128; i++) {
		ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 3, 5);
		if (ret == HAL_OK) {
			sprintf(Buffer, "0x%X", i);
			HAL_UART_Transmit(&huart2, Buffer, sizeof(Buffer), 10000);

			sprintf(buffer, "Found: 0x%02X", i);
			ssd1306_SetCursor(0, 16 + found * 10);
			ssd1306_WriteString(buffer, Font_6x8, White);
			ssd1306_UpdateScreen();

			HAL_Delay(300);
			found++;
			if (found > 4)
				break;
		} else {
			HAL_UART_Transmit(&huart2, Space, sizeof(Space), 10000);
		}
	}

	HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
	return found;
}

void esp32getIP(void) {
	for (int attempt = 1; attempt <= MAX_CONNECTION_ATTEMPTS; attempt++) {
		HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1,
				(I2C_ADDR << 1), (uint8_t*) rx_data, DATA_SIZE, HAL_MAX_DELAY);

		if (status == HAL_OK) {
			rx_data[DATA_SIZE - 1] = '\0';
			strncpy(device_ip, rx_data, sizeof(device_ip));
			device_ip[sizeof(device_ip) - 1] = '\0';

			if (is_valid_ip(device_ip)) {
				printf("\nReceived valid IP: %s\n", device_ip);
				Display_IPAddress(device_ip);
				HAL_GPIO_TogglePin(GPIOA, 6);
				HAL_Delay(100);
				break;
			}
		}
	}
}

bool is_valid_ip(const char *ip) {
	int num, dots = 0;
	char *ptr;
	char ip_copy[32];
	strncpy(ip_copy, ip, sizeof(ip_copy));
	ip_copy[sizeof(ip_copy) - 1] = '\0';

	ptr = strtok(ip_copy, ".");
	if (ptr == NULL)
		return false;

	while (ptr) {
		if (!isdigit(*ptr))
			return false;

		num = atoi(ptr);
		if (num < 0 || num > 255)
			return false;

		ptr = strtok(NULL, ".");
		if (ptr != NULL)
			dots++;
	}

	return dots == 3;
}

char scan_keypad(void) {
	char key = '\0';
	uint8_t row, col;

	for (row = 0; row < 4; row++) {
		HAL_GPIO_WritePin(ROW_PORT, ROW1_PIN | ROW2_PIN | ROW3_PIN | ROW4_PIN,
				GPIO_PIN_SET);
		switch (row) {
		case 0:
			HAL_GPIO_WritePin(ROW_PORT, ROW1_PIN, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(ROW_PORT, ROW2_PIN, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(ROW_PORT, ROW3_PIN, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(ROW_PORT, ROW4_PIN, GPIO_PIN_RESET);
			break;
		}

		HAL_Delay(1);

		if (HAL_GPIO_ReadPin(COL_PORT, COL1_PIN) == GPIO_PIN_RESET) {
			key = keypad[row][0];
		} else if (HAL_GPIO_ReadPin(COL_PORT, COL2_PIN) == GPIO_PIN_RESET) {
			key = keypad[row][1];
		} else if (HAL_GPIO_ReadPin(COL_PORT, COL3_PIN) == GPIO_PIN_RESET) {
			key = keypad[row][2];
		} else if (HAL_GPIO_ReadPin(COL_PORT, COL4_PIN) == GPIO_PIN_RESET) {
			key = keypad[row][3];
		}

		if (key != '\0') {
			if (key
					== last_key&& (HAL_GetTick() - last_key_time) < KEY_DEBOUNCE_MS) {
				return '\0';
			}
			last_key = key;
			last_key_time = HAL_GetTick();
			return key;
		}
	}

	return '\0';
}

void handle_keypress(char key) {
	if (key == '\0')
		return;

	printf("Key pressed: %c\r\n", key);

	switch (key) {
	case 'A':
		if (!system_armed && !system_locked) {
			pincode_position = 0;
			memset(entered_pincode, 0, sizeof(entered_pincode));
			Display_EnterPincode(pincode_position, entered_pincode);
		}
		break;

	case 'B':
		if (system_armed && !system_locked) {
			pincode_position = 0;
			memset(entered_pincode, 0, sizeof(entered_pincode));
			Display_EnterPincode(pincode_position, entered_pincode);
		}
		break;

	case 'C':
		pincode_position = 0;
		memset(entered_pincode, 0, sizeof(entered_pincode));
		Display_SystemStatus(system_armed);
		break;

	case 'D':
		if (pincode_position == PINCODE_LENGTH) {
			check_pincode();
		}
		break;

	case '*':
		pincode_position = 0;
		memset(entered_pincode, 0, sizeof(entered_pincode));
		Display_EnterPincode(pincode_position, entered_pincode);
		break;

	case '#':
		break;

	default:
		if (pincode_position < PINCODE_LENGTH && isdigit(key)) {
			entered_pincode[pincode_position++] = key;
			Display_EnterPincode(pincode_position, entered_pincode);
		}
		break;
	}
}

void check_pincode(void) {
	uint32_t stored_pincode = ReadFromFlash(address1);
	char stored_pincode_str[PINCODE_LENGTH + 1];
	sprintf(stored_pincode_str, "%04lu", stored_pincode);

	if (strncmp(entered_pincode, stored_pincode_str, PINCODE_LENGTH) == 0) {
		pin_attempts = 0;
		system_armed = !system_armed;
		Display_AccessGranted();
		Display_SystemStatus(system_armed);
	} else {
		pin_attempts++;
		if (pin_attempts >= MAX_PIN_ATTEMPTS) {
			system_locked = true;
			lockout_start = HAL_GetTick();
			Display_LockedScreen(LOCKOUT_TIME_MS / 1000);
		} else {
			Display_AccessDenied();
			pincode_position = 0;
			memset(entered_pincode, 0, sizeof(entered_pincode));
			Display_EnterPincode(pincode_position, entered_pincode);
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
