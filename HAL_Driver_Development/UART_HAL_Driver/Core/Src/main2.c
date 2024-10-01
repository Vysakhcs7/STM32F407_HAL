/*
 * main2.c
 *
 *  Created on: Sep 29, 2024
 *  Author: vysakh
 *  Start Transmission: HAL_UART_Transmit_IT() is called, initiating the transmission of data.
 *  Transmit Data: The UART peripheral transmits data in the background while the CPU continues executing other code.
 *  Interrupt Triggered: When the transmission is complete, an interrupt is triggered.
 *  Interrupt Handler: The interrupt is handled by UART5_IRQHandler(), which calls HAL_UART_IRQHandler().
 *  Callback Execution: HAL_UART_IRQHandler() calls HAL_UART_TxCpltCallback(), which sets uart_ready to 1 to signal that the UART is available again.
 *  Main Loop Check: The main loop checks uart_ready and initiates the next transmission if it is 1.
 */


#include "main.h"

UART_HandleTypeDef huart5;
GPIO_InitTypeDef uart_io;
volatile uint8_t uart_ready = 1;


void UART_MspInit(void);
void GPIO_Init(void);
void UART5_Init(void);
void Error(void);
void UART5_Interrupt_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

int main(void) {
	HAL_Init();
	UART_MspInit();
	UART5_Init();
	UART5_Interrupt_Init();
    while (1) {
        if (uart_ready) {
            uart_ready = 0;
            HAL_UART_Transmit_IT(&huart5, (uint8_t*) "Hello World!\r\n", 14);
        }
    }
}


void GPIO_Init(void) {
	uart_io.Mode = GPIO_MODE_AF_PP;
	uart_io.Alternate = GPIO_AF8_UART5;
	uart_io.Pin = GPIO_PIN_12;
	uart_io.Pull = GPIO_PULLUP;
	uart_io.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOC, &uart_io);

	uart_io.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOD, &uart_io);

}

void UART5_Init(void) {
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 9600;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		Error();
	}
}

void UART_MspInit(void) {
//Enable the UART5 interface clock
__HAL_RCC_UART5_CLK_ENABLE();

//Enable the clock for the UART GPIOs.
//PC12 - Tx
//PD2 - Rx
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();

//Configure the UART TX/RX pins as alternate function pull-up.
GPIO_Init();

}
void UART5_IRQHandler(void)
{
	//To identify the cause of the interrupt
	HAL_UART_IRQHandler(&huart5);
}

void UART5_Interrupt_Init(void){
/* UART5 interrupt Init */
//Configure the USARTx interrupt priority.
HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
//Enable the NVIC USART IRQ handle.
HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        uart_ready = 1; // Set the flag to indicate UART is ready for new transmission
    }
}

void Error(void){
	while(1){

	}
}
