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
#define CMD0  (0x40 + 0)  // GO_IDLE_STATE
#define CMD8  (0x40 + 8)  // SEND_IF_COND
#define CMD55 (0x40 + 55) // APP_CMD
#define ACMD41 (0x40 + 41) // SD_SEND_OP_COND (after CMD55)

// Define SPI chip select control for SD card (assuming NSS is on PA4)
#define SD_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SD_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
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

// Function to check CMD8 response (voltage check)
uint8_t SD_Check_CMD8(void) {
    uint8_t response;

    // Send CMD8 with argument 0x1AA (check pattern and voltage)
    SD_CS_LOW();
    SD_SendCommand(CMD8, 0x000001AA, 0x87);  // CMD8 with CRC = 0x87
    response = SD_ReceiveResponse();
    SD_CS_HIGH();

    // Debug output via printf
    if (response == 0x01) {
        printf("CMD8 Success: SD card supports required voltage.\n");
        return 0;  // CMD8 success
    } else {
        printf("CMD8 Failed: Response = 0x%02X\n", response);
        return 1;  // CMD8 failed
    }
}

// Function to send ACMD41 and wait until card is ready
uint8_t SD_WaitReady(void) {
    uint8_t response;
    uint16_t timeout = 0xFFFF;

    // Repeat CMD55 (APP_CMD) + ACMD41 until SD card leaves idle state
    do {
        // Send CMD55
        SD_CS_LOW();
        SD_SendCommand(CMD55, 0x00000000, 0x01);  // CMD55 with CRC = 0x01
        response = SD_ReceiveResponse();
        SD_CS_HIGH();

        if (response > 0x01) {
            printf("CMD55 Failed: Response = 0x%02X\n", response);
            return 1;
        }

        // Send ACMD41 (SD_SEND_OP_COND)
        SD_CS_LOW();
        SD_SendCommand(ACMD41, 0x40000000, 0x01);  // ACMD41 with HCS set
        response = SD_ReceiveResponse();
        SD_CS_HIGH();

        if (response == 0x00) {
            printf("ACMD41 Success: SD card is ready.\n");
            return 0;  // Card is ready
        }

        timeout--;
    } while ((response != 0x00) && timeout);

    printf("ACMD41 Timeout: SD card did not leave idle state.\n");
    return 1;
}

// SD card CMD0 check function with debugging
uint8_t SD_Check_CMD0(void) {
    uint8_t response;

    // Send dummy clocks with CS high (80 cycles), SD card needs time to wake up
    SD_CS_HIGH();
    for (int i = 0; i < 20; i++) {  // Increased dummy cycles to ensure SD card wakes up
        SD_SendDummyClock();
    }

    // Send CMD0 (GO_IDLE_STATE) to reset the SD card
    SD_CS_LOW();
    SD_SendCommand(CMD0, 0x00000000, 0x95);  // CMD0 with CRC = 0x95 for first command
    response = SD_ReceiveResponse();
    SD_CS_HIGH();

    // Debug output via printf
    if (response == 0x01) {
        printf("CMD0 Success: SD card in idle state.\n");
        return 0;  // CMD0 success
    } else if (response == 0x00) {
        printf("CMD0 Response: SD card already in ready state (0x00).\n");
        return 2;  // CMD0 indicates card is ready
    } else {
        printf("CMD0 Failed: Response = 0x%02X\n", response);
        return 1;  // CMD0 failed
    }
}

// Send an SPI command to the SD card
void SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t cmdFrame[6];

    // Create the command frame
    cmdFrame[0] = cmd;                       // Command byte
    cmdFrame[1] = (uint8_t)(arg >> 24);      // Argument byte 1
    cmdFrame[2] = (uint8_t)(arg >> 16);      // Argument byte 2
    cmdFrame[3] = (uint8_t)(arg >> 8);       // Argument byte 3
    cmdFrame[4] = (uint8_t)(arg);            // Argument byte 4
    cmdFrame[5] = crc;                       // CRC byte (only important for CMD0, CMD8)

    // Send the command frame via SPI
    HAL_SPI_Transmit(&hspi1, cmdFrame, 6, HAL_MAX_DELAY);
}

// Receive a response from the SD card
uint8_t SD_ReceiveResponse(void) {
    uint8_t response;
    uint8_t timeout = 0xFF;

    // Loop until we get a response byte that is not 0xFF
    do {
        HAL_SPI_Receive(&hspi1, &response, 1, HAL_MAX_DELAY);
        timeout--;
    } while ((response == 0xFF) && timeout);

    return response;
}

// Send 0xFF via SPI to generate dummy clocks
void SD_SendDummyClock(void) {
    uint8_t dummyByte = 0xFF;
    HAL_SPI_Transmit(&hspi1, &dummyByte, 1, HAL_MAX_DELAY);
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
  // Step 1: Check CMD0 (already working)
      uint8_t cmd0Status = SD_Check_CMD0();
      if (cmd0Status != 0) {
          printf("CMD0 failed. Exiting...\n");
          return 1;
      }

      // Step 2: Check CMD8 (voltage check)
      uint8_t cmd8Status = SD_Check_CMD8();
      if (cmd8Status != 0) {
          printf("CMD8 failed. Exiting...\n");
          return 1;
      }

      // Step 3: Send ACMD41 until card is ready
      uint8_t acmd41Status = SD_WaitReady();
      if (acmd41Status != 0) {
          printf("ACMD41 failed. Exiting...\n");
          return 1;
      }

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
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
