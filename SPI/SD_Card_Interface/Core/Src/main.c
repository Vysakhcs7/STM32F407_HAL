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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define SD Card commands
#define CMD0        0
#define CMD0_ARG    0x00000000
#define CMD0_CRC    0x94
// Define SPI chip select control for SD card (assuming NSS is on PA4)
#define CS_DISABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_ENABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t SD_Check_CMD0(void);
void SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_ReceiveResponse(void);
void SD_SendDummyClock(void);
uint8_t SD_WaitReady(void);
uint8_t SD_Check_CMD8(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SD_powerUpSeq()
{
    // make sure card is deselected
	CS_DISABLE();

    // give SD card time to power up
    HAL_Delay(1);

    // send 80 clock cycles to synchronize
    for(uint8_t i = 0; i < 10; i++)
        //SPI_transfer(0xFF);
    	HAL_SPI_Transmit(&hspi1, (uint8_t *)0xFF, 1, HAL_MAX_DELAY);

    // deselect SD card
    CS_DISABLE();
    //SPI_transfer(0xFF);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0xFF, 1, HAL_MAX_DELAY);

}


/*void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    // transmit command to sd card
    SPI_transfer(cmd|0x40);

    // transmit argument
    SPI_transfer((uint8_t)(arg >> 24));
    SPI_transfer((uint8_t)(arg >> 16));
    SPI_transfer((uint8_t)(arg >> 8));
    SPI_transfer((uint8_t)(arg));

    // transmit crc
    SPI_transfer(crc|0x01);
}*/

void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t cmdFrame[6];

    // Prepare the command frame
    cmdFrame[0] = cmd | 0x40;                // Command byte (CMD + 0x40)
    cmdFrame[1] = (uint8_t)(arg >> 24);      // Argument byte 1
    cmdFrame[2] = (uint8_t)(arg >> 16);      // Argument byte 2
    cmdFrame[3] = (uint8_t)(arg >> 8);       // Argument byte 3
    cmdFrame[4] = (uint8_t)(arg);            // Argument byte 4
    cmdFrame[5] = crc | 0x01;                // CRC byte (always ending with 0x01)

    // Send the command frame via SPI
    HAL_SPI_Transmit(&hspi1, cmdFrame, 6, HAL_MAX_DELAY);
    printf("Sent CMD: %d, Arg: 0x%08X, CRC: 0x%02X\n", cmd, arg, crc);

}


/*
uint8_t SD_readRes1()
{
    uint8_t i = 0, res1;
    // keep polling until actual data received
    while((res1 = SPI_transfer(0xFF)) == 0xFF)
    {
        i++;

        // if no data received for 8 bytes, break
        if(i > 8) break;
    }

    return res1;
}
*/


uint8_t SD_readRes1() {
    uint8_t i = 0, res1;
    uint8_t dummyByte = 0xFF;

    // keep polling until actual data is received
    do {
        HAL_SPI_TransmitReceive(&hspi1, &dummyByte, &res1, 1, HAL_MAX_DELAY);
        i++;
    } while (res1 == 0xFF && i <= 8);  // If no response for 8 iterations, break the loop

    printf("Response received: 0x%02X after %d attempts\n", res1, i);
    return res1;
}


uint8_t SD_goIdleState()
{
    // assert chip select
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0xFF, 1, HAL_MAX_DELAY);
    //SPI_transfer(0xFF);

	CS_ENABLE();
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0xFF, 1, HAL_MAX_DELAY);
    //SPI_transfer(0xFF);

    // send CMD0
    SD_command(CMD0, CMD0_ARG, CMD0_CRC);

    // read response
    uint8_t res1 = SD_readRes1();

    // deassert chip select
    //SPI_transfer(0xFF);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0xFF, 1, HAL_MAX_DELAY);
    CS_DISABLE();
    //SPI_transfer(0xFF);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)0xFF, 1, HAL_MAX_DELAY);


    return res1;
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  // start power up sequence

     // command card to idle

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
	  SD_powerUpSeq();

	     uint8_t response = SD_goIdleState();
	       printf("CMD0 Response: 0x%02X\n", response);

HAL_Delay(2000);
    /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
  printf("Assert failed in file %s on line %d\n", file, line);
}
#endif /* USE_FULL_ASSERT */
