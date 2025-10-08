/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ms5611.h"
#include "usbd_cdc_if.h"
#include "bmi088.h"
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
extern SPI_HandleTypeDef hspi2;
MS5611_Handle_t ms5611;
BMI088_Handle_t bmi088;
// Global variables for Live Expressions / Data Trace
volatile float sensor_pressure = 0.0f;
volatile float sensor_temperature = 0.0f;
volatile float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
volatile float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
volatile float imu_temperature = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);

  char buffer[200];
  int len;

  /* Initialize MS5611 Barometer */
  len = sprintf(buffer, "\r\n=== Sensor Initialization ===\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  len = sprintf(buffer, "Step 1: Init MS5611...\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  if (!MS5611_Init(&ms5611, &hspi2, GPIOC, GPIO_PIN_8, MS5611_OSR_4096)) {
      len = sprintf(buffer, "MS5611: FAILED!\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
      HAL_Delay(200);

      // Rapid blink = MS5611 error (100ms)
      while (1) {
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
          HAL_Delay(100);
      }
  }
  len = sprintf(buffer, "Step 2: MS5611 OK!\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  len = sprintf(buffer, "Step 3: Checking BMI088 CS pins...\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  // Test CS pins are working
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(10);

  len = sprintf(buffer, "Step 4: Init BMI088...\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  if (!BMI088_Init(&bmi088, &hspi2,
                   GPIOC, GPIO_PIN_7,   // Accel CS
                   GPIOB, GPIO_PIN_12)) { // Gyro CS
      len = sprintf(buffer, "BMI088: FAILED!\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
      HAL_Delay(200);

      // Medium blink = BMI088 error (200ms)
      while (1) {
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
          HAL_Delay(200);
      }
  }
  len = sprintf(buffer, "Step 5: BMI088 OK!\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  len = sprintf(buffer, "\r\n=== All Sensors Ready ===\r\n\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);
  HAL_Delay(200);

  uint32_t last_led_toggle = 0;
  uint32_t last_sensor_read = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    // Read all sensors every 200ms
	    if (HAL_GetTick() - last_sensor_read >= 200) {
	        last_sensor_read = HAL_GetTick();

	        // Read MS5611
	        float pressure, temperature;
	        if (MS5611_ReadSensor(&ms5611, &pressure, &temperature)) {
	            sensor_pressure = pressure;
	            sensor_temperature = temperature;
	        }

	        // Read BMI088 Accelerometer
	        BMI088_AccelData_t accel;
	        if (BMI088_ReadAccel(&bmi088, &accel)) {
	            accel_x = accel.x;
	            accel_y = accel.y;
	            accel_z = accel.z;
	        }

	        // Read BMI088 Gyroscope
	        BMI088_GyroData_t gyro;
	        if (BMI088_ReadGyro(&bmi088, &gyro)) {
	            gyro_x = gyro.x;
	            gyro_y = gyro.y;
	            gyro_z = gyro.z;
	        }

	        // Format and send data
	        int p_int = (int)sensor_pressure;
	        int p_frac = (int)((sensor_pressure - p_int) * 100);
	        int t_int = (int)sensor_temperature;
	        int t_frac = (int)((sensor_temperature - t_int) * 100);
	        if (sensor_temperature < 0 && t_frac != 0) t_frac = -t_frac;

	        len = sprintf(buffer, "P:%d.%02d mbar T:%d.%02d C | ",
	                     p_int, p_frac, t_int, t_frac);

	        // Accelerometer
	        int ax_int = (int)accel_x;
	        int ax_frac = (int)((accel_x - ax_int) * 100);
	        if (accel_x < 0 && ax_frac != 0) ax_frac = -ax_frac;

	        int ay_int = (int)accel_y;
	        int ay_frac = (int)((accel_y - ay_int) * 100);
	        if (accel_y < 0 && ay_frac != 0) ay_frac = -ay_frac;

	        int az_int = (int)accel_z;
	        int az_frac = (int)((accel_z - az_int) * 100);
	        if (accel_z < 0 && az_frac != 0) az_frac = -az_frac;

	        len += sprintf(buffer + len, "A:[%d.%02d,%d.%02d,%d.%02d]g | ",
	                      ax_int, ax_frac, ay_int, ay_frac, az_int, az_frac);

	        // Gyroscope
	        int gx_int = (int)gyro_x;
	        int gx_frac = (int)((gyro_x - gx_int) * 100);
	        if (gyro_x < 0 && gx_frac != 0) gx_frac = -gx_frac;

	        int gy_int = (int)gyro_y;
	        int gy_frac = (int)((gyro_y - gy_int) * 100);
	        if (gyro_y < 0 && gy_frac != 0) gy_frac = -gy_frac;

	        int gz_int = (int)gyro_z;
	        int gz_frac = (int)((gyro_z - gz_int) * 100);
	        if (gyro_z < 0 && gz_frac != 0) gz_frac = -gz_frac;

	        len += sprintf(buffer + len, "G:[%d.%02d,%d.%02d,%d.%02d]dps\r\n",
	                      gx_int, gx_frac, gy_int, gy_frac, gz_int, gz_frac);

	        CDC_Transmit_FS((uint8_t*)buffer, len);
	    }

	    // Toggle LED every second
	    if (HAL_GetTick() - last_led_toggle >= 1000) {
	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	        last_led_toggle = HAL_GetTick();
	    }

	    HAL_Delay(10);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
