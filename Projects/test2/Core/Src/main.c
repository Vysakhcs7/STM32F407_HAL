#include "stm32f4xx.h"

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

void SystemClock_Config(void);
void GPIO_Init(void);
void DHT11_Init(void);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_Read(void);
void DHT11_GetData(uint8_t *humidity, uint8_t *temperature);
void delay_us(uint32_t us);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    DHT11_Init();

    uint8_t humidity = 0, temperature = 0;

    while (1)
    {
        DHT11_GetData(&humidity, &temperature);
        HAL_Delay(1000);  // Delay 1 second
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_Start(void)
{
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);  // 20ms delay
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);
}

uint8_t DHT11_CheckResponse(void)
{
    uint8_t response = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
    {
        delay_us(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) response = 1;
        else response = -1;
    }
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));

    return response;
}

uint8_t DHT11_Read(void)
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
        delay_us(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
            data &= ~(1 << (7 - i));
        else
            data |= (1 << (7 - i));
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
    }
    return data;
}

void DHT11_GetData(uint8_t *humidity, uint8_t *temperature)
{
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;

    DHT11_Start();
    if (DHT11_CheckResponse())
    {
        Rh_byte1 = DHT11_Read();
        Rh_byte2 = DHT11_Read();
        Temp_byte1 = DHT11_Read();
        Temp_byte2 = DHT11_Read();
        SUM = DHT11_Read();

        if (SUM == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
        {
            *humidity = Rh_byte1;
            *temperature = Temp_byte1;
        }
    }
}

void delay_us(uint32_t us)
{
    uint32_t start = SysTick->VAL;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((SysTick->VAL - start) < ticks);
}
