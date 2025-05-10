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
#include "stdio.h"
#include "string.h"
#include "ctype.h"
#include "pinCode.h"
#include "RFID.h"
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

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t alarm_flag = 0;
volatile uint8_t key_pressed = 0;
volatile uint8_t reed_triggered = 0;
char last_key = '\0';
uint32_t last_key_time = 0;
uint32_t last_reed_time = 0;

PincodeState pincodeState;
bool wifi_connected = false;
char rx_data[DATA_SIZE];
char device_ip[32];

uint8_t value = 0;
char str1[17] = { '\0' };
char str2[17] = { '\0' };
char str3[17] = { '\0' };
char str4[17] = { '\0' };
char tmp_str[65] = { '\0' };

// Delay function:
#define	Precise_Delay(x) {\
	uint32_t x1 = x * 72;\
	DWT->CYCCNT = 0;\
	while (DWT->CYCCNT < x1);\
}



////
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
int ScanI2CDevices(void);
bool is_valid_ip(const char *ip);
char scan_keypad(void);
void handle_keypress(char key);
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
int fputc(int ch, FILE *f) {
	return __io_putchar(ch);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t data[] = { 0xFF, 0x0 };
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	setvbuf(stdout, NULL, _IONBF, 0); //disable waiting for newline
	printf("Booting STM32...\r\n");
	Display_Init();
	Display_BootMessage();
	printf("Display initialized...\r\n");
	Pincode_Init(&pincodeState);

	//Initialize RFID
	MFRC522_Init();

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
	printf("\nRequesting IP address to ESP32");
	esp32getIP();

	HAL_Delay(1000);
	// Show system status
	Display_SystemStatus(pincodeState.system_armed);
	uint32_t start_time = 0;
	bool toggle = false;
	char pressed_key = '\0';
	if (wifi_connected) {
		HAL_TIM_Base_Start(&htim1);
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t) data, (uint32_t) &GPIOA->ODR,
				5);
		__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
	}


	//
	printf("\nSearching for an RC522\r\n");
	RFIDstatus = Read_MFRC522(VersionReg);
	sprintf(str1, "\nRunning RC522\r\n");
	sprintf(str2, "\nver:%x\r\n", RFIDstatus);
	printf("%s\r\n", str1);
	printf("%s\r\n", str2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* Main loop */

	while (1) {

		// Check if system is locked
		if (pincodeState.system_locked) {
			uint32_t remaining = (LOCKOUT_TIME_MS
					- (HAL_GetTick() - pincodeState.lockout_start)) / 1000;
			Display_LockedScreen(remaining);

			if ((HAL_GetTick() - pincodeState.lockout_start) >= LOCKOUT_TIME_MS) {
				pincodeState.system_locked = false;
				pincodeState.pin_attempts = 0;
				Display_SystemStatus(pincodeState.system_armed);
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
		checkRFID();
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
		if (pincodeState.system_armed) {
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
	if (reed_triggered && pincodeState.system_armed) {
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
	Display_SystemStatus(pincodeState.system_armed);
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
		uint8_t send_byte = '1';
		HAL_StatusTypeDef tx_status = HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR << 1), &send_byte, 1, HAL_MAX_DELAY);
		if (tx_status != HAL_OK) {
			printf("[ERROR] I2C Transmit failed on attempt %d\n", attempt);
			continue;
		}

		HAL_Delay(10);

		uint8_t rx_raw[DATA_SIZE] = {0};
		HAL_StatusTypeDef rx_status = HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDR << 1), rx_raw, DATA_SIZE, HAL_MAX_DELAY);

		if (rx_status == HAL_OK) {
			// Convert to string safely
			char ip_str[DATA_SIZE + 1] = {0}; // null terminator
			for (int i = 0; i < DATA_SIZE; i++) {
				if (rx_raw[i] == '\n' || rx_raw[i] == '\r') {
					break;
				}
				if (isprint(rx_raw[i])) {
					ip_str[i] = rx_raw[i];
				} else {
					ip_str[i] = 0;
					break;
				}
			}
			ip_str[DATA_SIZE] = '\0';

			if (is_valid_ip(ip_str)) {
				strncpy(device_ip, ip_str, sizeof(device_ip));
				device_ip[sizeof(device_ip) - 1] = '\0';

				printf("[OK] Received valid IP: %s\n", device_ip);
				Display_IPAddress(device_ip);
				HAL_Delay(100);
				wifi_connected = true;
				break;
			} else {
				printf("[WARN] Invalid IP received: %s\n", ip_str);
			}
		} else {
			printf("[ERROR] No IP received on attempt %d\n", attempt);
		}

		strcpy(device_ip, "ERROR");
		Display_IPAddress(device_ip);
		HAL_Delay(200);
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
		if (!pincodeState.system_armed && !pincodeState.system_locked) {
			pincodeState.pincode_position = 0;
			memset(pincodeState.entered_pincode, 0,
					sizeof(pincodeState.entered_pincode));
			Display_EnterPincode(pincodeState.pincode_position,
					pincodeState.entered_pincode);
		}
		break;

	case 'B':
		if (pincodeState.system_armed && !pincodeState.system_locked) {
			pincodeState.pincode_position = 0;
			memset(pincodeState.entered_pincode, 0,
					sizeof(pincodeState.entered_pincode));
			Display_EnterPincode(pincodeState.pincode_position,
					pincodeState.entered_pincode);
		}
		break;

	case 'C':
		pincodeState.pincode_position = 0;
		memset(pincodeState.entered_pincode, 0,
				sizeof(pincodeState.entered_pincode));
		Display_SystemStatus(pincodeState.system_armed);
		break;

	case 'D':
		if (pincodeState.pincode_position == PINCODE_LENGTH) {
			Pincode_Check(&pincodeState);
		}
		break;

	case '*':
		pincodeState.pincode_position = 0;
		memset(pincodeState.entered_pincode, 0,
				sizeof(pincodeState.entered_pincode));
		Display_EnterPincode(pincodeState.pincode_position,
				pincodeState.entered_pincode);
		break;

	case '#':
		break;

	default:
		if (pincodeState.pincode_position < PINCODE_LENGTH && isdigit(key)) {
			pincodeState.entered_pincode[pincodeState.pincode_position++] = key;
			Display_EnterPincode(pincodeState.pincode_position,
					pincodeState.entered_pincode);
		}
		break;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		printf("DMA transfer completed for TIM2_CH1.\n");
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
