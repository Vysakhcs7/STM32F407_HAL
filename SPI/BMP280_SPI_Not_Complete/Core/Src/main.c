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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_TEMP_XLSB 		0XFC
#define BMP280_TEMP_LSB 		0XFB
#define BMP280_TEMP_MSB 		0XFA
#define BMP280_PRESS_XLSB 		0XF9
#define BMP280_PRESS_LSB 		0XF8
#define BMP280_PRESS_MSB 		0XF7
#define BMP280_CONFIG 			0XF5
#define BMP280_CTRL_MEAS 		0XF4
#define BMP280_STATUS 			0XF3
#define BMP280_RESET 			0XE0
#define BMP280_ID 				0XD0

/* Oversampling set byte osrs_t (x2)
 Oversampling set byte osrs_p (x16)
 Normal mode - (11) */
#define BMP280_CTRL_MEAS_VALUE  0x57
#define BMP280_RESET_VALUE 		0xB6
#define TIMEOUT_DELAY 			100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
uint8_t data;
uint8_t comp_val_lsb;
uint8_t comp_val_msb;
uint16_t dig_T1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void bmp280_reg_config(uint8_t reg_addr, uint8_t data);
void bmp280_reg_read(uint8_t reg_addr);
void bmp280_read_compensation_val(uint8_t reg_addr_lsb, uint8_t reg_addr_msb);
void bmp280_read_dig_T1(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	//IIR Filter coefficient 16 - reg 0xf5

	//Data readout is done by starting a burst read from 0xF7 to
	//0xFC. The data are read out in an unsigned 20-bit format both for pressure and for temperature.

void bmp280_reg_config(uint8_t reg_addr, uint8_t data) {
	reg_addr = reg_addr & ~(1 << 7);
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg_addr, 1, TIMEOUT_DELAY);
	HAL_SPI_Transmit(&hspi2, &data, 1, TIMEOUT_DELAY);
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_SET);
}

void bmp280_reg_read(uint8_t reg_addr) {
	reg_addr = reg_addr | (1 << 7);
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg_addr, 1, TIMEOUT_DELAY);
	HAL_SPI_Receive(&hspi2, &data, 1, TIMEOUT_DELAY);
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_SET);
}

void bmp280_read_compensation_val(uint8_t reg_addr_lsb, uint8_t reg_addr_msb) {
	// Set MSB to indicate read operation
	// Pull CS low to select the BMP280
	// Read LSB
	// Read MSB
	// Pull CS high to deselect the BMP280
	// Combine the MSB and LSB to form the compensation value
	reg_addr_lsb = reg_addr_lsb | (1 << 7);
	reg_addr_msb = reg_addr_msb | (1 << 7);
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg_addr_lsb, 1, TIMEOUT_DELAY);
	HAL_SPI_Receive(&hspi2, &comp_val_lsb, 1, TIMEOUT_DELAY);
	HAL_SPI_Transmit(&hspi2, &reg_addr_msb, 1, TIMEOUT_DELAY);
	HAL_SPI_Receive(&hspi2, &comp_val_msb, 1, TIMEOUT_DELAY);
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_SET);
    dig_T1 = (comp_val_msb << 8) | comp_val_lsb;
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
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_SET);
	//bmp280_reg_config(BMP280_RESET, BMP280_RESET_VALUE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
bmp280_reg_read(BMP280_ID);
		bmp280_read_compensation_val(0x8E,0x8F);

		HAL_Delay(3000);
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
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
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
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SLAVE_SELECT_Pin */
	GPIO_InitStruct.Pin = SLAVE_SELECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SLAVE_SELECT_GPIO_Port, &GPIO_InitStruct);

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
