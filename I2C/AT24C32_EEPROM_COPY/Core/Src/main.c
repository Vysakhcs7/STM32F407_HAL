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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define DS1307_ADDRESS (0x68 << 1)

#define AT24C32_I2C_ADDRESS  					(0x50 << 1)
#define MASK 									0x0FFF
#define AT24C32_MEM_WRITE_START_ADDR 			0x000
#define AT24C32_MEM_READ_START_ADDR				(AT24C32_MEM_WRITE_START_ADDR)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef status;

/* USER CODE BEGIN PV */
uint8_t hour, minute, second;
uint8_t time[3];
uint8_t read_data[3];
uint8_t read_date[3];
int i = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*-----------------------------------------------------------------------------------*/
/* Write data to EEPROM */
void AT24C32_EEPROM_Page_Write(uint16_t mem_wrt_strt_addr, uint8_t *pwrite_data,
		uint16_t size) {
	uint16_t bytes_written = 0;
	uint16_t page_offset = mem_wrt_strt_addr % 32; // Calculate the offset within the current page

	while (bytes_written < size) {
		// Calculate the number of bytes to write in the current page
		uint16_t bytes_to_write = 32 - page_offset; // Bytes remaining in the current page

		// Adjust bytes_to_write if we have fewer bytes left to write
		if ((size - bytes_written) < bytes_to_write) {
			bytes_to_write = size - bytes_written;
		}

		// Write data to EEPROM
		status = HAL_I2C_Mem_Write(&hi2c1, AT24C32_I2C_ADDRESS,
				mem_wrt_strt_addr & MASK, I2C_MEMADD_SIZE_16BIT, pwrite_data,
				bytes_to_write, HAL_MAX_DELAY);
		if (status != HAL_OK) {
			// Handle the error (e.g., log it, blink an LED, etc.)
			printf("EEPROM Write Error\r\n");
			return;
		}

		// Update counters and pointers
		bytes_written += bytes_to_write;
		pwrite_data += bytes_to_write;
		mem_wrt_strt_addr += bytes_to_write;

		// Wait for the EEPROM to complete the write cycle
		HAL_Delay(5); // EEPROM write cycle time (typically 5ms)

		// Reset page_offset for the next page
		page_offset = 0;
	}
}

/*-----------------------------------------------------------------------------------*/
/* Read data from EEPROM */
void AT24C32_EEPROM_Page_Read(uint16_t mem_read_strt_addr, uint8_t *pread_data) {
	// Read the data from the EEPROM
	status = HAL_I2C_Mem_Read(&hi2c1, AT24C32_I2C_ADDRESS,
			(mem_read_strt_addr & MASK), I2C_MEMADD_SIZE_16BIT, pread_data, 3,
			HAL_MAX_DELAY);
	if (status != HAL_OK) {
		// Handle the error (e.g., log it, blink an LED, etc.)
		printf("EEPROM Read Error\r\n");
	}
}
/*-----------------------------------------------------------------------------------*/
uint8_t DEC2BCD(uint8_t dec) {
	return ((dec / 10) << 4) | (dec % 10);
}

uint8_t BCD2DEC(uint8_t bcd) {
	return ((bcd >> 4) * 10) + (bcd & 0x0F);
}
/*-----------------------------------------------------------------------------------*/

void DS1307_SetTime(uint8_t hour, uint8_t minute, uint8_t second) {
	uint8_t data[3];
	data[0] = DEC2BCD(second);
	data[1] = DEC2BCD(minute);
	data[2] = DEC2BCD(hour);

	HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data,
			3, HAL_MAX_DELAY);
}
/*-----------------------------------------------------------------------------------*/

void DS1307_GetTime(uint8_t *hour, uint8_t *minute, uint8_t *second) {
	uint8_t data[3];

	HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data,
			3, HAL_MAX_DELAY);

	*second = BCD2DEC(data[0]);
	*minute = BCD2DEC(data[1]);
	*hour = BCD2DEC(data[2]);
	time[0] = BCD2DEC(data[2]);
	time[1] = BCD2DEC(data[1]);
	time[2] = BCD2DEC(data[0]);
}
/*-----------------------------------------------------------------------------------*/
void DS1307_SetDate(uint8_t day, uint8_t month, uint8_t year) {
	uint8_t data[3];
	data[0] = DEC2BCD(day);
	data[1] = DEC2BCD(month);
	data[2] = DEC2BCD(year);
	HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, data,
			3, HAL_MAX_DELAY);

}
/*-----------------------------------------------------------------------------------*/
void DS1307_GetDate(void) {
	uint8_t data[3];

	HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, data,
			3, HAL_MAX_DELAY);
	read_date[0] = BCD2DEC(data[0]);
	read_date[1] = BCD2DEC(data[1]);
	read_date[2] = BCD2DEC(data[2]);
}
/*-----------------------------------------------------------------------------------*/

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
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	// Set time to 0x12:0x34:0x56 (example)
	DS1307_SetTime(23, 59, 00);
	DS1307_SetDate(31, 12, 23);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		DS1307_GetTime(&hour, &minute, &second);
		DS1307_GetDate();

		if (second >= 0 && second <= 10) {
			AT24C32_EEPROM_Page_Write(AT24C32_MEM_WRITE_START_ADDR, time,
					sizeof(time));
			HAL_Delay(500);
			AT24C32_EEPROM_Page_Read(AT24C32_MEM_READ_START_ADDR, read_data);

			printf("Timestamp : %u:%u:%u\r\n\n", read_data[0], read_data[1],
					read_data[2]);
			printf("Datestamp : %u:%u:%u\r\n\n", read_data[0], read_data[1],
					read_data[2]);

		}
		// printf("Time - %u:%u:%u\r\n",hour,minute,second);
		// printf("Date - %u-%u-%u\r\n",read_date[0],read_date[1],read_date[2]);
		HAL_Delay(1000);
//AT24C32_EEPROM_Page_Write(AT24C32_MEM_WRITE_START_ADDR, write_data,sizeof(write_data));
		// HAL_Delay(100);
		//	AT24C32_EEPROM_Page_Read(AT24C32_MEM_READ_START_ADDR, read_data);
		//	printf("Read Data: %s\r\n", read_data);  // Debug output

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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : PD12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the UART3 and Loop until the end of transmission */

	//PB10 - TX
	//PB11 - RX
	//PA2  - TX
	//PA3  - RX
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);

	return ch;
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
