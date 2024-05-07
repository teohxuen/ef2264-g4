/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stdarg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINT
//#define BINARY_WRITE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2; // SD card
SPI_HandleTypeDef hspi3; // Temperature Sensor

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Memory Address for Light Sensor
uint8_t BH1721_ADDR_read = 0x47; // Use 7-bit address + R/W bit
uint8_t BH1721_ADDR_write = 0x46; // Use 7-bit address + R/W bit
uint8_t BH1721_start = 0x01; // Power On
uint8_t BH1721_LRC = 0x13; // continuous low resolution (largest range)

// For Temperature Sensor
uint8_t TC72_WR = 0x80;
uint8_t TC72_TEMP = 0x15;
uint8_t TC72_OP = 0x14;
uint8_t TC72_RR = 0x03;

uint8_t unmount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Interrrupt Handler for Blue Button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		unmount = 1;
	}
}

void myprintf(const char *fmt, ...) {
	static char buffer[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, -1);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;
	uint8_t buf[8];

	uint8_t spi_buf[8];

	uint16_t light_val;
	uint8_t flag;
	float lux;
	float temp;

	uint32_t prev_tick;
	uint32_t curr_tick;
	uint32_t delta_tick;
	uint32_t init_tick;
	uint32_t led_tick;
	float time;
	uint8_t led_state;

	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FRESULT fres; //Result after operations
	FIL fil;
	UINT bytesWrote;
	char str_buf[100];
	float write_buf[3];
	int str_buf_len;

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	// Tell BH1721 to start measurements
	ret = HAL_I2C_Master_Transmit(&hi2c1, BH1721_ADDR_write, &BH1721_start, 1,
	HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		myprintf("Error!\n"); // error message in UART
	}

	// Mount SD Card
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		while (1)
			;
	}

	// Check available space on SD Card
	DWORD free_clusters, free_sectors, total_sectors;
	FATFS *getFreeFs;

	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		myprintf("f_getfree error (%i)\r\n", fres);
		while (1)
			;
	}
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;

	myprintf(
			"SD card stats:\r\n%ld kB total drive space.\r\n%ld kB available.\r\n",
			total_sectors / 2, free_sectors / 2);

#ifdef BINARY_WRITE
	// Write File Header
	fres = f_open(&fil, "data.bin", FA_CREATE_ALWAYS | FA_WRITE); // new file is created
	f_close(&fil); // close file
#else
	// Write File Header
	fres = f_open(&fil, "data.txt", FA_CREATE_ALWAYS | FA_WRITE); // new file is created
	f_lseek(&fil, f_size(&fil)); //put the file pointer to end of file
	str_buf_len = sprintf(str_buf, "Time; Temperature; Luminosity\r\n"); // generate string to write
	fres = f_write(&fil, str_buf, str_buf_len, &bytesWrote); //write
	f_close(&fil); // close file
#endif
	// LED on
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //PA5 is Green LED
	led_tick = HAL_GetTick();
	led_state = ~0;

	// Set SPI3 CS Low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	// Configure BH1721 to do low resolution continuously
	ret = HAL_I2C_Master_Transmit(&hi2c1, BH1721_ADDR_write, &BH1721_LRC, 1,
	HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		myprintf("Error!\n"); // error message in UART
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // SPI3 CS on
	HAL_SPI_Transmit(&hspi3, (uint8_t*) &TC72_WR, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) &TC72_OP, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // SPI3 CS off

	init_tick = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		// Read 2 bytes from BH1721 into a buffer
		ret = HAL_I2C_Master_Receive(&hi2c1, BH1721_ADDR_read, buf, 2,
		HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			myprintf("Error!\n"); // error message in UART
		}

		// Convert into Lux
		light_val = (buf[0] << 8) | buf[1];
		lux = light_val / 1.2;

		// Send 0x80 and 0x15 to Temp Sensor
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // SPI3 CS on
		HAL_SPI_Transmit(&hspi3, (uint8_t*) &TC72_WR, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi3, (uint8_t*) &TC72_TEMP, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // SPI3 CS off

		HAL_Delay(150);

		// Send 0x03 to Temp Sensor and Read 4 Bytes into SPI Buffer
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // SPI3 CS on
		HAL_SPI_Transmit(&hspi3, (uint8_t*) &TC72_RR, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi3, (uint8_t*) spi_buf, 4, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // SPI3 CS off

		// Convert Temp Data into Celsius
		flag = spi_buf[2] >> 6; //right shift by 6 bits

		if (spi_buf[1] >= 128) { //cause 128 means -0
			temp = -((spi_buf[1]) - 128) - (flag * 0.25);
		} else {
			temp = spi_buf[1] + (flag * 0.25);
		}

		curr_tick = HAL_GetTick();
		time = (float) (curr_tick - init_tick) / 1000;

#ifdef DEBUG_PRINT
		myprintf("Light: %.2f lux\r\n", lux);

		myprintf("Temp Val: %x %x %x %x\r\n", spi_buf[0], spi_buf[1],
				spi_buf[2], spi_buf[3]);

		myprintf("Temp : %.2f\r\n", temp);
#endif

		// write data to SD card
#ifdef BINARY_WRITE
		fres = f_open(&fil, "data.bin", FA_WRITE | FA_OPEN_APPEND);
		// 4 Byte 4 Byte 4 Byte
		// <Time><Temp><Luminosity>
		write_buf[0] = time;
		write_buf[1] = temp;
		write_buf[2] = lux;
		fres = f_write(&fil, write_buf, sizeof(write_buf), &bytesWrote);
		f_close(&fil); // close file
#else
		fres = f_open(&fil, "data.txt", FA_WRITE | FA_OPEN_APPEND);
		f_lseek(&fil, f_size(&fil)); //put the file pointer to end of file
		str_buf_len = sprintf(str_buf, "%.3f;%.2f;%.2f\r\n", time, temp, lux); // generate string to write
		fres = f_write(&fil, str_buf, str_buf_len, &bytesWrote); //write
		f_close(&fil); // close file
#endif

		delta_tick = HAL_GetTick() - prev_tick;
		prev_tick = HAL_GetTick();

		myprintf("Time between measurement: %d\r\n", delta_tick);

		// check if writing worked
#ifdef DEBUG_PRINT
		if (fres == FR_OK) {
			myprintf("Wrote %i bytes to 'data.txt'!\r\n", bytesWrote);
		} else {
			myprintf("f_write error\r\n");
		}
#endif

		// If blue button is pressed, unmount SD Card
		if (unmount) {
			fres = f_mount(NULL, "", 1);
			if (fres == FR_OK) {
				myprintf("SD CARD UNMOUNTED successfully...\r\n");
			}
			while (1)
				; // Halt the programme
		}

		// Turn off LED
		if (HAL_GetTick() - led_tick > 500) {
			if (led_state) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			}
			led_state = ~led_state; // Invert LED State
			led_tick = HAL_GetTick();
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
	hi2c1.Init.Timing = 0x10909CEC;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

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
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
