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

#define DS1307_ADDRESS (0x68 << 1)  // Define DS1307 I2C address

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;  // I2C handle for communication

/* USER CODE BEGIN PV */

uint8_t  write_time_date[7];    // Buffer to store time data for writing
uint8_t read_time_date[7];    // Buffer to store time data read from DS1307
uint8_t bcd_time_date[7];    // Buffer to store time data in BCD format

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);    // Function to configure system clock
static void MX_GPIO_Init(void);   // Function to initialize GPIO
static void MX_I2C1_Init(void);    // Function to initialize I2C1

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Convert decimal to BCD (Binary-Coded Decimal)
uint8_t DS1307_DEC2BCD(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}


// Convert BCD to decimal
uint8_t DS1307_BCD2DEC(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Set the time on the DS1307 RTC
void DS1307_SetTime(uint8_t year, uint8_t month, uint8_t date,uint8_t day,uint8_t hour, uint8_t minute, uint8_t second)
{
    write_time_date[0] = DS1307_DEC2BCD(second);  // Convert seconds to BCD
    write_time_date[1] = DS1307_DEC2BCD(minute);  // Convert minutes to BCD
    write_time_date[2] = DS1307_DEC2BCD(hour);    // Convert hours to BCD
    write_time_date[3] = DS1307_DEC2BCD(day);    // Convert hours to BCD
    write_time_date[4] = DS1307_DEC2BCD(date);    // Convert hours to BCD
    write_time_date[5] = DS1307_DEC2BCD(month);    // Convert hours to BCD
    write_time_date[6] = DS1307_DEC2BCD(year);    // Convert hours to BCD

    HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, write_time_date, 7, HAL_MAX_DELAY);  // Write time data to DS1307
}

// Get the time from the DS1307 RTC
void DS1307_GetTime() {
    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, bcd_time_date, 7, HAL_MAX_DELAY);  // Read time data from DS1307
    for(int i = 0; i < 7; i++)
    {
        read_time_date[i] = DS1307_BCD2DEC(bcd_time_date[i]);  // Convert BCD data to decimal
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();  // Initialize the HAL Library

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // Configure system clock

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  MX_GPIO_Init();  // Initialize GPIO
  MX_I2C1_Init();  // Initialize I2C1

  /* USER CODE BEGIN 2 */

  // Set initial date and time to - 2024-12-31 3(Tuesday) 23:59:40
  DS1307_SetTime(24, 12, 31, 3, 23, 59, 40);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    DS1307_GetTime();  // Get time

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();  // Enable Power Control Clock
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // Set voltage scaling

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // Use High-Speed Internal oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;  // Turn on HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  // Default calibration value
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  // Turn on PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;  // Use HSI as PLL source
  RCC_OscInitStruct.PLL.PLLM = 8;  // PLL M divider
  RCC_OscInitStruct.PLL.PLLN = 168;  // PLL N multiplier
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // PLL P divider
  RCC_OscInitStruct.PLL.PLLQ = 7;  // PLL Q divider
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)  // Initialize oscillators
  {
    Error_Handler();  // Call error handler if initialization fails
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  // Configure clock types
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Set PLL as SYSCLK source
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // AHB clock divider
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  // APB1 clock divider
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  // APB2 clock divider

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)  // Configure CPU, AHB and APB clocks
  {
    Error_Handler();  // Call error handler if configuration fails
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
  hi2c1.Instance = I2C1;  // Initialize I2C1
  hi2c1.Init.ClockSpeed = 100000;  // Set clock speed
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;  // Set duty cycle
  hi2c1.Init.OwnAddress1 = 0;  // Set own address
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // Set addressing mode
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;  // Disable dual address mode
  hi2c1.Init.OwnAddress2 = 0;  // Set second own address
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  // Disable general call mode
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;  // Disable clock stretching
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)  // Initialize I2C1
  {
    Error_Handler();  // Call error handler if initialization fails
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable GPIOB clock

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();  // Disable interrupts
  while (1)
  {
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
