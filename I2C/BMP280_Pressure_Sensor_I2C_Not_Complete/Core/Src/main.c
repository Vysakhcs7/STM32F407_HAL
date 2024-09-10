#include "stm32f4xx_hal.h"

// BMP280 I2C address
#define BMP280_ADDR            (0x76 << 1) // I2C address of BMP280 (Shifted left by 1 for HAL)

// BMP280 Registers
#define BMP280_REG_TEMP_XLSB   0xFC
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0

// BMP280 Calibration Registers
#define BMP280_REG_DIG_T1      0x88
#define BMP280_REG_DIG_T2      0x8A
#define BMP280_REG_DIG_T3      0x8C
#define BMP280_REG_DIG_P1      0x8E
#define BMP280_REG_DIG_P2      0x90
#define BMP280_REG_DIG_P3      0x92
#define BMP280_REG_DIG_P4      0x94
#define BMP280_REG_DIG_P5      0x96
#define BMP280_REG_DIG_P6      0x98
#define BMP280_REG_DIG_P7      0x9A
#define BMP280_REG_DIG_P8      0x9C
#define BMP280_REG_DIG_P9      0x9E

// BMP280 Reset value
#define BMP280_RESET_VALUE    0xB6

// I2C handle declaration
I2C_HandleTypeDef hi2c1;

// Calibration coefficients
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

int32_t raw_temp;
int32_t raw_press;
int32_t t_fine;

// Global variables for debugging
int32_t calibrated_temp;
int32_t calibrated_pressure;

// LED GPIO pin
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA

void SystemClock_Config(void);
static void MX_I2C1_Init(void);
static void MX_GPIO_Init(void);

void BMP280_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t reset_cmd = BMP280_RESET_VALUE;
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_RESET, 1, &reset_cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(200); // Increased delay for reset

    uint8_t ctrl_meas = 0x27; // Control measurement register value (oversampling x1, normal mode)
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, HAL_MAX_DELAY);

    uint8_t config = 0x00; // Configuration register value (standby time = 0.5ms, filter coefficient = 1)
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CONFIG, 1, &config, 1, HAL_MAX_DELAY);

    // Read calibration coefficients
    uint8_t calib[24];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_DIG_T1, 1, calib, 24, HAL_MAX_DELAY);

    dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

int32_t BMP280_ReadRawTemperature(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_TEMP_MSB, 1, data, 3, HAL_MAX_DELAY);
    return (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
}

int32_t BMP280_ReadRawPressure(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_PRESS_MSB, 1, data, 3, HAL_MAX_DELAY);
    return (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
}

uint8_t BMP280_ReadID(I2C_HandleTypeDef *hi2c) {
    uint8_t id;
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_ID, 1, &id, 1, HAL_MAX_DELAY);
    return id;
}

int32_t BMP280_CalibrateTemperature(int32_t adc_T) {
    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)dig_T1 << 1)) * (int32_t)dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)dig_T1) * ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8; // Temperature in Celsius multiplied by 100
    return T;
}

int32_t BMP280_CalibratePressure(int32_t adc_P, int32_t t_fine) {
    int32_t var1, var2, p;

    var1 = (((int32_t)t_fine >> 1) - (int32_t)64000);
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)dig_P6;
    var2 += ((var1 * (int32_t)dig_P5) << 1);
    var2 = var2 >> 2;
    var1 = (((((int32_t)dig_P4 * 65536) + var2) >> 2) + (int32_t)dig_P3) >> 16;
    var1 = (((((var1 * ((int32_t)dig_P2 >> 1)) >> 16) + (int32_t)dig_P1) >> 1) + 32768) >> 16;
    p = (((uint32_t)adc_P - 16384) - var1) * 3125;
    var1 = (((uint32_t)dig_P8 * (((p >> 13) * (p >> 13)) >> 11)) >> 14);
    var2 = (((uint32_t)dig_P7 * p) >> 13);
    p = (p + var1 + var2 + 8192) >> 14;
    return p;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    BMP280_Init(&hi2c1);

    // Main loop
    while (1) {
        // Allow sensor time to complete measurement
        HAL_Delay(100); // Wait for measurement to be processed

        raw_temp = BMP280_ReadRawTemperature(&hi2c1);
        raw_press = BMP280_ReadRawPressure(&hi2c1);

        calibrated_temp = BMP280_CalibrateTemperature(raw_temp);
        calibrated_pressure = BMP280_CalibratePressure(raw_press, t_fine);

        // Toggle LED to indicate successful reading
        if (calibrated_pressure > 100000) {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }

        HAL_Delay(1000); // Delay 1 second
    }
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  __disable_irq();
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
