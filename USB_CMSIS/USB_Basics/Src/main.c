/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
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

/*HCLK = 72 MHz
PLL: M = 4, N = 72, P = 2, Q = 3
AHB Prescalar = 1
APB Prescalar1 = 2, APB Precalar2 = 1
MCO1 Prescalar = 2
*/

#include <stdint.h>
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx.h"

uint32_t SystemCoreClock = 72000000;

static void configure_clock()
{
	MODIFY_REG(FLASH->ACR,
			FLASH_ACR_LATENCY,
			FLASH_ACR_LATENCY_2WS << FLASH_ACR_LATENCY_Pos);
	// _VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS);
	
}

void SystemInit(void){
	configure_clock();	
}

int main(void) {
	while (1) {

	}
}
