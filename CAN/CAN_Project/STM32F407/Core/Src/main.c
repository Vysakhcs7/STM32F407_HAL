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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef txHeader;
uint8_t txData[1] = { 'V' };  // Data to send
uint32_t txMailbox;
uint8_t txData1[1] = { 'Y' };  // Data to send
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */

	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterBank = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.SlaveStartFilterBank = 14;

	txHeader.DLC = 1;  // Data length code: 1 byte
	txHeader.IDE = CAN_ID_STD;  // Standard identifier
	txHeader.StdId = 0x123;  // Identifier
	txHeader.RTR = CAN_RTR_DATA;  // Data frame
	txHeader.ExtId = 0x01;   // Extended Identifier (not used in this case)

	if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK) {
		// Filter configuration Error
		Error_Handler();
	}

	// Start CAN
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		// Start Error
		Error_Handler();
	}

	// Activate the notification for message reception
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		// Notification Error
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Set PD13 to High

			}
		}
			else {
				if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
							{
								HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData1, &txMailbox);
								//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // Set PD13 to High

							}
			}

			//		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
			//		{
			//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
			//		            if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK) {
			//		                Error_Handler();
			//		            }
			//
			//		        }
			// Check if a CAN message is received
			//		    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
			//		    {
			//		      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			//		      {
			//		        // Process the received message (RxData)
			//		        // You can add your code here to handle the received data
			//		      }
			//		      else
			//		      {
			//		        // Reception Error
			//		        Error_Handler();
			//		      }
			//		    }

			// HAL_Delay(500);

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
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
		RCC_OscInitStruct.HSEState = RCC_HSE_ON;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
		RCC_OscInitStruct.PLL.PLLM = 4;
		RCC_OscInitStruct.PLL.PLLN = 72;
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
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)
				!= HAL_OK) {
			Error_Handler();
		}
	}

	/**
	 * @brief CAN1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_CAN1_Init(void) {

		/* USER CODE BEGIN CAN1_Init 0 */

		/* USER CODE END CAN1_Init 0 */

		/* USER CODE BEGIN CAN1_Init 1 */

		/* USER CODE END CAN1_Init 1 */
		hcan1.Instance = CAN1;
		hcan1.Init.Prescaler = 4;
		hcan1.Init.Mode = CAN_MODE_NORMAL;
		hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
		hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
		hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
		hcan1.Init.TimeTriggeredMode = DISABLE;
		hcan1.Init.AutoBusOff = DISABLE;
		hcan1.Init.AutoWakeUp = DISABLE;
		hcan1.Init.AutoRetransmission = DISABLE;
		hcan1.Init.ReceiveFifoLocked = DISABLE;
		hcan1.Init.TransmitFifoPriority = DISABLE;
		if (HAL_CAN_Init(&hcan1) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN CAN1_Init 2 */

		/* USER CODE END CAN1_Init 2 */

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
		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

		/*Configure GPIO pin : PA0 */
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin : PD13 */
		GPIO_InitStruct.Pin = GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
