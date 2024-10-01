/*
 * main2.c
 *  Created on: Sep 29, 2024
 *  Author: vysakh
*/


#include "main.h"

GPIO_InitTypeDef GPIO_InitStruct;
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;

void GPIO_Init(void);
void Error(void);
void CAN1_Init(void);
void CAN1_Tx(void);
void SystemClock_Config(void);
void CAN_MspInit(void);


int main(void) {
	HAL_Init();
	SystemClock_Config();
	CAN_MspInit();
	CAN1_Init();
	if( HAL_CAN_Start(&hcan1) != HAL_OK){
		Error();
	}
    while (1) {
    	CAN1_Tx();
    	HAL_Delay(1000);
    }
}

void CAN1_Init(void){
	hcan1.Instance= CAN1;
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	hcan1.Init.Prescaler = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	if( HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error();
	}
}


void CAN1_Tx(void){
	uint32_t TxMailBox;
	uint8_t msg[6] = {'V','Y','S','A','K','H'};
	TxHeader.DLC = 6;
	TxHeader.StdId = 0X65D;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan1, &TxHeader, msg, &TxMailBox) != HAL_OK){
		Error();
	}
	while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailBox));
}




void CAN_MspInit(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	//PB8 - CAN1_Rx
	//PB9 - CAN1_Tx
	// 1001: AF9
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error();
  }
}

void Error(void){
	while(1){

	}
}
