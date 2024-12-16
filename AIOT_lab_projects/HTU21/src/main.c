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
#include "wifi.h"
#include "stdio.h"
#include "string.h"
#include "HTU21.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFFER_SIZE 2048
#define RX_BUFFER_SIZE 2048
#define TX_TIMEOUT 2000
#define RX_TIMEOUT 2000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
WIFI_HandleTypeDef hwifi;
char ssid[] = "your_hotspot_SSID";
char passphrase[] = "your_hotspot_password";
__IO FlagStatus cmdDataReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void wifi_init(void);
static void mqtt_init(void);
static void mqtt_publish_temp(void);
static void mqtt_publish_hum(void);


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
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize Wi-Fi and MQTT functionalities
  wifi_init();
  mqtt_init();
  // Initialize HTU21 Sensor
  HAL_Delay(500);
  HTU21_Reset();

  while(HTU21_Init() != 0x2) {
      printf("Initializing HTU21 Sensor...\r\n");
      HAL_Delay(100);
  }
  printf("Initialization Successful \r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  mqtt_publish_temp();
	  HAL_Delay(500);
	  mqtt_publish_hum();
	  HAL_Delay(500);
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, WIFI_RESET_Pin|WIFI_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WIFI_RESET_Pin WIFI_NSS_Pin */
  GPIO_InitStruct.Pin = WIFI_RESET_Pin|WIFI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_CMD_DATA_READY_Pin */
  GPIO_InitStruct.Pin = WIFI_CMD_DATA_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WIFI_CMD_DATA_READY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void wifi_init(void) {
    hwifi.handle = &hspi3;
    hwifi.ssid = ssid;
    hwifi.passphrase = passphrase;
    hwifi.securityType = WPA_MIXED;
    hwifi.DHCP = SET;
    hwifi.ipStatus = IP_V4;
    hwifi.transportProtocol = WIFI_TCP_PROTOCOL;
    hwifi.port = 8080;

    HAL_UART_Transmit(&huart1, (uint8_t *)"Initializing WiFi...\r\n", strlen("Initializing WiFi...\r\n"), HAL_MAX_DELAY);

    // Reset WiFi module and check for CMD/Data ready
    WIFI_RESET_MODULE();
    HAL_Delay(100);  // Short delay to allow module to reset
    HAL_UART_Transmit(&huart1, (uint8_t *)"WiFi module reset.\r\n", strlen("WiFi module reset.\r\n"), HAL_MAX_DELAY);

    if (!WIFI_IS_CMDDATA_READY()) {
        HAL_UART_Transmit(&huart1, (uint8_t *)"CMD/Data not ready.\r\n", strlen("CMD/Data not ready.\r\n"), HAL_MAX_DELAY);
        return;  // Early exit if CMD/Data not ready
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t *)"CMD/Data is ready, proceeding with initialization.\r\n", strlen("CMD/Data is ready, proceeding with initialization.\r\n"), HAL_MAX_DELAY);
    }

    if (WIFI_Init(&hwifi) != WIFI_OK) {
        HAL_UART_Transmit(&huart1, (uint8_t *)"WiFi init failed.\r\n", strlen("WiFi init failed.\r\n"), HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t *)"WiFi Initialized successfully.\r\n", strlen("WiFi Initialized successfully.\r\n"), HAL_MAX_DELAY);
    }

    if (WIFI_JoinNetwork(&hwifi) != WIFI_OK) {
        HAL_UART_Transmit(&huart1, (uint8_t *)"Failed to join network.\r\n", strlen("Failed to join network.\r\n"), HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t *)"Connected to WiFi network successfully.\r\n", strlen("Connected to WiFi network successfully.\r\n"), HAL_MAX_DELAY);
    }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == WIFI_CMD_DATA_READY_Pin) {
        cmdDataReady = HAL_GPIO_ReadPin(WIFI_CMD_DATA_READY_GPIO_Port, WIFI_CMD_DATA_READY_Pin);
    }
}

static void mqtt_init(void) {
    // Initialize MQTT client settings for temperature
    printf("Initializing MQTT client for temperature.\r\n");
    strcpy(hwifi.mqtt.publishTopic, "your_feed");
    hwifi.mqtt.securityMode = WIFI_MQTT_SECURITY_USER_PW;
    strcpy(hwifi.mqtt.userName, "your_username");
    strcpy(hwifi.mqtt.password, "your_AIO_key");
    hwifi.mqtt.keepAlive = 60;
    hwifi.port = 1883;
    strcpy(hwifi.remoteAddress, "io.adafruit.com");

    if (WIFI_MQTTClientInit(&hwifi) != WIFI_OK) {
        printf("MQTT client initialization for temperature failed.\n");
        Error_Handler();
    }
    printf("MQTT client for temperature initialized successfully.\r\n");

    // Initialize MQTT client settings for humidity
    printf("Initializing MQTT client for humidity.\r\n");
    strcpy(hwifi.mqtt.publishTopic, "your_feed");
    if (WIFI_MQTTClientInit(&hwifi) != WIFI_OK) {
        printf("MQTT client initialization for humidity failed.\n");
        Error_Handler();
    }
    printf("MQTT client for humidity initialized successfully.\r\n");
}

static void mqtt_publish_temp(void) {
    printf("Entering MQTT publish temperature function.\r\n");

    // Read temperature data
    float HTU21_Temp_Value = HTU21_GetTemp();

    // Format the data into a message string
    char message[256];
    snprintf(message, sizeof(message), "%.2f", HTU21_Temp_Value);

    // Set topic to temperature feed
    strcpy(hwifi.mqtt.publishTopic, "your_feed");
    printf("Set topic to: %s\r\n", hwifi.mqtt.publishTopic);

    // Reinitialize MQTT client with the new topic
    if (WIFI_MQTTClientInit(&hwifi) != WIFI_OK) {
        printf("MQTT client reinitialization failed for temperature.\n");
        Error_Handler();
    }

    // Publish the message
    if (WIFI_MQTTPublish(&hwifi, message, strlen(message)) != WIFI_OK) {
        printf("MQTT publish failed for temperature.\n");
        Error_Handler();
    }
    printf("Temperature message published successfully. Published to topic: %s\r\n", hwifi.mqtt.publishTopic);
}

static void mqtt_publish_hum(void) {
    printf("Entering MQTT publish humidity function.\r\n");

    // Read humidity data
    float HTU21_Hum_Value = HTU21_GetHum();

    // Format the data into a message string
    char message[256];
    snprintf(message, sizeof(message), "%.2f", HTU21_Hum_Value);

    // Set topic to humidity feed
    strcpy(hwifi.mqtt.publishTopic, "your_feed");
    printf("Set topic to: %s\r\n", hwifi.mqtt.publishTopic);

    // Reinitialize MQTT client with the new topic
    if (WIFI_MQTTClientInit(&hwifi) != WIFI_OK) {
        printf("MQTT client reinitialization failed for humidity.\n");
        Error_Handler();
    }

    // Publish the message
    if (WIFI_MQTTPublish(&hwifi, message, strlen(message)) != WIFI_OK) {
        printf("MQTT publish failed for humidity.\n");
        Error_Handler();
    }
    printf("Humidity message published successfully. Published to topic: %s\r\n", hwifi.mqtt.publishTopic);
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
