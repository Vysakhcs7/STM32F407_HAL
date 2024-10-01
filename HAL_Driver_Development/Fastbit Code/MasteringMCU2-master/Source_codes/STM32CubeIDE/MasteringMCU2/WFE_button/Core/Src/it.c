/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */
#include "main_app.h"

extern UART_HandleTypeDef huart2;

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt.
  */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}