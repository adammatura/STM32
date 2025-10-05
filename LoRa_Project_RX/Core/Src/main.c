/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LoRa Sniffer for SX1262
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <sx1262_B_common.h>
#include <sx126x.h>
#include <sx126x_hal.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
LoRaConfig LoRa;

uint8_t txdata[10];
uint8_t rxdata[10];
uint32_t last_print_time = 0;
uint32_t last_led_time = 0;
static volatile bool irq_fired = false;
static sx126x_pkt_type_t pkt_type ;
static sx126x_chip_status_t radio_status;
static sx126x_stats_lora_t stats;
static sx126x_errors_mask_t errors;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // Set initial RTC time
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  sTime.Hours = 20;
  sTime.Minutes = 22;
  sTime.Seconds = 0;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  sDate.Year = 25;  // 2025
  sDate.Month = 10;
  sDate.Date = 5;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  /****************************LoRa Module Initialization *******************************/
  printf_ts("AMA ENGINEERING LoRa RECEIVER APPLICATION\n\r");
  init_LoRa_parm();
  sx126x_clear_device_errors(&LoRa);
  sx126x_init(&LoRa);
  Radio_init(&LoRa);

  sx126x_clear_irq_status(&LoRa, SX126X_IRQ_ALL);
  sx126x_set_dio_irq_params(
      &LoRa, SX126X_IRQ_ALL,
      SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,
      SX126X_IRQ_NONE, SX126X_IRQ_NONE);

  sx126x_clear_irq_status(&LoRa, SX126X_IRQ_ALL);
  /****************************LoRa Module Initialization END *******************************/

  sx126x_get_pkt_type(&LoRa, &pkt_type);
  sx126x_get_status(&LoRa, &radio_status);
  sx126x_get_lora_stats(&LoRa, &stats);
  sx126x_get_device_errors(&LoRa, &errors);

  printf_ts("Error mask: 0x%04X\n\r", errors);

  HAL_Delay(2000);

  printf_ts("Starting RX mode...\n\r");
  sx126x_set_rx(&LoRa, 0);  // Enter continuous receive mode
  printf_ts("Ready to receive\n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  apps_common_sx126x_irq_process(&LoRa);
	  uint32_t current_time = HAL_GetTick();

	     // Print every 4000ms (4 seconds)
	     if(current_time - last_print_time >= 4000)
	     {
	         printf_ts("LoRa APPLICATION RUNNING...\n\r");
	         last_print_time = current_time;
	     }

	     // Toggle LED every 250ms
	     if(current_time - last_led_time >= 250)
	     {
	         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	         last_led_time = current_time;
	     }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x20;
  sTime.Minutes = 0x15;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_OCTOBER;
  DateToUpdate.Date = 0x5;
  DateToUpdate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LoRa_TX_EN_Pin|LoRa_RX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin LD2_Pin */
  GPIO_InitStruct.Pin = RST_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LoRa_TX_EN_Pin LoRa_RX_EN_Pin */
  GPIO_InitStruct.Pin = LoRa_TX_EN_Pin|LoRa_RX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void printf_ts(const char* format, ...)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    printf("SYS: 20%02d-%02d-%02d - %02d:%02d:%02d  ",
           sDate.Year, sDate.Month, sDate.Date,
           sTime.Hours, sTime.Minutes, sTime.Seconds);

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

void init_LoRa_parm(void)
{
	txdata[0] = 'M';
	txdata[1] = 'L';
	txdata[2] = 'o';
	txdata[3] = 'R';
	txdata[4] = 'a';

	LoRa.BUSY_port = BUSY_GPIO_Port;
	LoRa.BUSY_pin  = BUSY_Pin;
	LoRa.NSS_port  = SPI2_CS_GPIO_Port;
	LoRa.NSS_pin   = SPI2_CS_Pin;
	LoRa.RST_port  = RST_GPIO_Port;
	LoRa.RST_pin   = RST_Pin;
	LoRa.DIO1_port = DIO1_GPIO_Port;
	LoRa.DIO1_pin  = DIO1_Pin;

	LoRa.hSPIx     = &hspi2;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == LoRa.DIO1_pin)
  {
	  irq_fired = true;
  }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
void apps_common_sx126x_irq_process( const void* context )
{
    if( irq_fired == true )
    {
        irq_fired = false;

        sx126x_irq_mask_t irq_regs;
        sx126x_get_and_clear_irq_status( context, &irq_regs );

        if( ( irq_regs & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
        {
        	printf_ts( "Tx done\n\r" );
            //on_tx_done( );
        }

        if( ( irq_regs & SX126X_IRQ_RX_DONE ) == SX126X_IRQ_RX_DONE )
        {
        	printf_ts( "Rx done\n\r" );
            sx126x_handle_rx_done( context );
            on_rx_done( );

        }

        if( ( irq_regs & SX126X_IRQ_PREAMBLE_DETECTED ) == SX126X_IRQ_PREAMBLE_DETECTED )
        {
        	printf_ts( "Preamble detected\n\r" );
            //on_preamble_detected( );
        }

        if( ( irq_regs & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID )
        {
        	printf_ts( "Syncword valid\n\r" );
            //on_syncword_valid( );
        }

        if( ( irq_regs & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID )
        {
        	printf_ts( "Header valid\n\r" );
            //on_header_valid( );
        }

        if( ( irq_regs & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
        {
        	printf_ts( "Header error\n\r" );
            //on_header_error( );
        }

        if( ( irq_regs & SX126X_IRQ_CRC_ERROR ) == SX126X_IRQ_CRC_ERROR )
        {
        	printf_ts( "CRC error\n\r" );
            //on_crc_error( );
        }

        if( ( irq_regs & SX126X_IRQ_CAD_DONE ) == SX126X_IRQ_CAD_DONE )
        {
        	printf_ts( "CAD done\n\r" );
            if( ( irq_regs & SX126X_IRQ_CAD_DETECTED ) == SX126X_IRQ_CAD_DETECTED )
            {
            	printf_ts( "Channel activity detected\n\r" );
                //on_cad_done_detected( );
            }
            else
            {
            	printf_ts( "No channel activity detected\n\r" );
                //on_cad_done_undetected( );
            }
        }

        if( ( irq_regs & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT )
        {
        	printf_ts( "Rx timeout\n\r" );
            //on_rx_timeout( );
        }

        if( ( irq_regs & SX126X_IRQ_LR_FHSS_HOP ) == SX126X_IRQ_LR_FHSS_HOP )
        {
        	printf_ts( "FHSS hop done\n\r" );
            //on_fhss_hop_done( );
        }
    }
}
void on_rx_done(void)
{
    sx126x_rx_buffer_status_t rx_buffer_status;
    sx126x_pkt_status_lora_t pkt_status;
    char received[17];
    uint8_t raw_buffer[17];

    sx126x_get_lora_pkt_status(&LoRa, &pkt_status);
    sx126x_get_rx_buffer_status(&LoRa, &rx_buffer_status);

    uint8_t bytes_to_read = rx_buffer_status.pld_len_in_bytes;
    if(bytes_to_read > 16) bytes_to_read = 16;

    sx126x_read_buffer(&LoRa, 0, raw_buffer, bytes_to_read);

    uint8_t actual_len = raw_buffer[0];
    if(actual_len > 15) actual_len = 15;

    memcpy(received, raw_buffer + 1, actual_len);
    received[actual_len] = '\0';


    printf_ts("RX: %d bytes (actual: %d)\n\r", bytes_to_read, actual_len);

        printf_ts("Received: %s\n\r", received);

        printf_ts("RSSI: %d dBm\n\r", pkt_status.rssi_pkt_in_dbm);

        printf_ts("SNR: %d dB\n\r", pkt_status.snr_pkt_in_db);

        printf_ts("Signal RSSI: %d dBm\n\r", pkt_status.signal_rssi_pkt_in_dbm);

        printf_ts("-------------------\n\r");

        sx126x_set_rx(&LoRa, 0);
}


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
