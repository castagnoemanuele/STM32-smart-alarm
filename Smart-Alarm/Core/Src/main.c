/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Smart Alarm System
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
#include "mxconstants.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdbool.h"
#include "flashMemory.h"
#include <stdio.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ALARM_DURATION_MS 5000

/* Pin Definitions */
#define BUTTON_PIN         GPIO_PIN_13
#define BUZZER_PIN         GPIO_PIN_10
#define ESP32_EN	       GPIO_PIN_5
#define LD2_PIN            LD2_Pin

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

uint8_t Buffer[25] = { 0 };
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";

#define data_size 14
uint8_t i2c_addr = 0x5c;

char rx_data[data_size];

char device_ip[32]; // Declare this somewhere globally or within scope
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
int ScanI2CDevices(void);
bool is_valid_ip(const char *ip);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect printf to UART
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	uint8_t i = 0, ret;
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

	/* GPIOA and GPIOC Configuration----------------------------------------------*/
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* USER CODE BEGIN 2 */

	/*Configure SSD1306*/
	ssd1306_Init();
	DisplayBootMessage();
	/*Configure GPIO pin : BUTTON_PIN (blue pushbutton)*/
	GPIO_InitStruct.Pin = BUTTON_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; /* Try also rising */
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : BUZZER_PIN (buzzer)*/
	GPIO_InitStruct.Pin = BUZZER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	//////////////////////////////////PINCODE TEST SECTION//////////////////////////////
	//Write a string to memory
	uint32_t data = 10000000; //test data
	//SaveToFlash(data, address1); //enable to save a new value
	uint32_t value = ReadFromFlash(address1);
	char str[12];  // max per 32-bit unsigned int (10 cifre + '\0')
	sprintf(str, "%lu", (unsigned long) value); //convert int to string for printing
	ssd1306_SetCursor(35, 26); // Adjust as needed for centering
	ssd1306_WriteString(str, Font_7x10, White);
	ssd1306_UpdateScreen();
	///////////////////////////////////////////////////////////////////////////////////

	HAL_GPIO_WritePin(GPIOB, ESP32_EN, 1);
	HAL_Delay(5000); // Give time to the ESP to boot and connect to wifi

	/*-[ I2C Bus Scanning ]-*/
	ScanI2CDevices();
	HAL_Delay(500);
	/*--[ Scanning Done ]--*/

	// Ask For the IP Address
	int max_attempts = 3;
	int attempt;
	HAL_StatusTypeDef status;

	for (attempt = 1; attempt <= max_attempts; ++attempt) {
	    status = HAL_I2C_Master_Receive(&hi2c1, (i2c_addr << 1),
	                                    (uint8_t*) rx_data, data_size - 1, HAL_MAX_DELAY);

	    if (status == HAL_OK) {
	        rx_data[data_size - 1] = '\0';
	        strncpy(device_ip, rx_data, sizeof(device_ip));
	        device_ip[sizeof(device_ip) - 1] = '\0';

	        if (is_valid_ip(device_ip)) {
	            printf("\nReceived valid IP: %s\n", device_ip);

	            ssd1306_SetCursor(0, 40);
	            ssd1306_WriteString("IP:", Font_7x10, White);
	            ssd1306_SetCursor(30, 40);
	            ssd1306_WriteString(device_ip, Font_7x10, White);
	            ssd1306_UpdateScreen();

	            HAL_GPIO_TogglePin(GPIOA, BUZZER_PIN);
	            HAL_Delay(100);
	            break;
	        } else {
	            printf("\nInvalid IP format: %s\n", device_ip);
	        }
	    } else {
	        printf("\nAttempt %d failed to receive IP.\n", attempt);
	    }
	}

	if (attempt > max_attempts || !is_valid_ip(device_ip)) {
	    ssd1306_SetCursor(0, 40);
	    ssd1306_WriteString("IP ERROR!", Font_7x10, White);
	    ssd1306_UpdateScreen();
	    printf("\nFailed to receive valid IP after %d attempts.\n", max_attempts);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	printf("Configuration Completed. Starting Loop");

	uint32_t start_time = 0;
	bool toggle = 0;
	while (1) {
		if (alarm_flag) {
			if (start_time == 0)
				start_time = HAL_GetTick();
			if (HAL_GetTick() - start_time < ALARM_DURATION_MS) {
				HAL_GPIO_TogglePin(GPIOA, BUZZER_PIN);
				ssd1306_Fill(toggle ? Black : White);
				ssd1306_SetCursor(35, 26);
				ssd1306_WriteString("ALARM!", Font_7x10,
						toggle ? White : Black);
				ssd1306_UpdateScreen();
				toggle = !toggle;
				HAL_Delay(100);
			} else {
				alarm_flag = 0;
				start_time = 0;
				HAL_GPIO_WritePin(GPIOA, BUZZER_PIN, GPIO_PIN_RESET);
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();
			}

		}
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(BUTTON_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);
		alarm_flag = 1;
	}
}

// Boot Message Display Function
void DisplayBootMessage(void) {
	// Clear screen (black background)
	ssd1306_Fill(Black);

	// Welcome title
	ssd1306_SetCursor(10, 10);
	ssd1306_WriteString("Smart Alarm System", Font_7x10, White);

	// Booting line
	ssd1306_SetCursor(30, 30);
	ssd1306_WriteString("Booting up...", Font_6x8, White);

	ssd1306_UpdateScreen();
	HAL_Delay(1500);  // Dramatic pause

	// Flash to white background, black text
	ssd1306_Fill(White);
	ssd1306_SetCursor(30, 26);
	ssd1306_WriteString("System Ready", Font_7x10, Black);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);

	// Clear again to prepare for runtime display
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
}


int ScanI2CDevices(void) {
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
        ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5);
        if (ret == HAL_OK) {
            sprintf(Buffer, "0x%X", i);
            HAL_UART_Transmit(&huart2, Buffer, sizeof(Buffer), 10000);

            // Display the address found
            sprintf(buffer, "Found: 0x%02X", i);
            ssd1306_SetCursor(0, 16 + found * 10);
            ssd1306_WriteString(buffer, Font_6x8, White);
            ssd1306_UpdateScreen();

            HAL_Delay(300);
            found++;
            if (found > 4) break; // prevent OLED overflow
        } else {
            HAL_UART_Transmit(&huart2, Space, sizeof(Space), 10000);
        }
    }

    HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
    return found;
}

// Function to check if we have a valid response from ESP32
bool is_valid_ip(const char *ip) {
    int num, dots = 0;
    char *ptr;
    char ip_copy[32];
    strncpy(ip_copy, ip, sizeof(ip_copy));
    ip_copy[sizeof(ip_copy) - 1] = '\0';

    ptr = strtok(ip_copy, ".");
    if (ptr == NULL) return false;

    while (ptr) {
        if (!isdigit(*ptr)) return false;

        num = atoi(ptr);
        if (num < 0 || num > 255) return false;

        ptr = strtok(NULL, ".");
        if (ptr != NULL) dots++;
    }

    return dots == 3;
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
