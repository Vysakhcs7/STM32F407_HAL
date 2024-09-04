#include "main.h"

int main(void)
{
	//HAL_Init();
	HAL_RCC_DeInit();

	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef myPort = {0};
	myPort.Mode =  GPIO_MODE_OUTPUT_PP;
	myPort.Pin = GPIO_PIN_13;
	myPort.Pull =  GPIO_NOPULL;
	myPort.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &myPort);

	while(1)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		HAL_Delay(1000);

	}
}
