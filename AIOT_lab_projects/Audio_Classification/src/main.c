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
#include "NanoEdgeAI.h"
#include "knowledge.h"
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

#define DATA_INPUT_USER 256 //BUFFER SIZE
#define AXIS_NUMBER 1
#define CONFIRMATIONS_NB   (uint32_t)(3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
WIFI_HandleTypeDef hwifi;
char ssid[] = "your_hotspot_SSID";
char passphrase[] = "your_hotspot_password";
__IO FlagStatus cmdDataReady = 0;

float mic=0.0;
float mic_buffer[DATA_INPUT_USER*AXIS_NUMBER]={0};
volatile uint32_t buffer_size=0;
float output_class_buffer[CLASS_NUMBER]; // Buffer of class probabilities
const char *id2class[CLASS_NUMBER + 1] = { // Buffer for mapping class id to class name
			"unknown",
			"BABY_CRY",
			"AMBIENT",
			"AMBULANCE",
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Inference(void);
void fill_mic_buffer(void);
int __io_putchar(int ch);
static void wifi_init(void);
static void mqtt_init(void);
static void mqtt_publish(const char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc-> Instance == ADC1)
	{
		mic=HAL_ADC_GetValue(&hadc1);
		mic_buffer[buffer_size]=mic;
		buffer_size++;
		if(buffer_size>=DATA_INPUT_USER)
		{
			buffer_size=0;
		}
	}
}
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
  enum neai_state error_code = neai_classification_init(knowledge);
  	if (error_code != NEAI_OK)
 {
  		/* This happens if the knowledge does not correspond to the library or if the library works into a not supported board. */

  	printf("Knowledge initialization ERROR");
  	printf("%d", error_code);
  	}
  	else
  	{
  		printf("Knowledge initialization done");
  	}
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize Wi-Fi and MQTT functionalities
  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start(&htim2);
  wifi_init();
  mqtt_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Inference();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 32;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
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
void fill_mic_buffer()
{
    for(int i = 0; i < DATA_INPUT_USER; i++)
    {
        mic_buffer[AXIS_NUMBER * i] = mic;
        HAL_Delay(3);
    }
}

void Inference()
{
    uint16_t i, id_class_t0, id_class_tn;
    fill_mic_buffer();
    neai_classification(mic_buffer, output_class_buffer, &id_class_t0);
    for(i = 0; i < CONFIRMATIONS_NB - 1; i++)
    {
        fill_mic_buffer();
        neai_classification(mic_buffer, output_class_buffer, &id_class_tn);
        if(id_class_t0 != id_class_tn)
        {
            break;
        }
    }

    if(id_class_t0 == id_class_tn)
    {
        printf("Detected Class: %s\r\n", id2class[id_class_t0]);
        if (strcmp(id2class[id_class_t0], "BABY_CRY") == 0) {
            mqtt_publish("BABY_CRY");
        }
    }
    else
    {
        printf("?\r\n");
    }
}


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
    strcpy(hwifi.mqtt.publishTopic, "your_feed");
    // If you don't want to subscribe to any topic, you can skip this line or set it to an empty string.
    strcpy(hwifi.mqtt.subscribeTopic, "your_feed");
    hwifi.mqtt.securityMode = WIFI_MQTT_SECURITY_USER_PW;
    strcpy(hwifi.mqtt.userName, "your_username");
    strcpy(hwifi.mqtt.password, "your_AIO_key");
    hwifi.mqtt.keepAlive = 60;
    hwifi.port = 1883;
    strcpy(hwifi.remoteIpAddress, "io.adafruit.com");

    if (WIFI_MQTTClientInit(&hwifi) != WIFI_OK) {
        printf("MQTT client initialization failed.\n");
        Error_Handler();
    }
    printf("MQTT client initialized successfully.\r\n");
}

void mqtt_publish(const char* message) {
    printf("Entering MQTT publish function.\r\n");

    if (WIFI_MQTTPublish(&hwifi, (char *)message, strlen(message)) != WIFI_OK) {
        printf("MQTT publish failed.\n");
        Error_Handler();
    }
    printf("MQTT message published successfully.\n");
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
