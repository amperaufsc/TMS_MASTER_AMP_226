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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "errors.h"
#include "adc.h"
#include <stdbool.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* Definitions for xSendCAN */
osThreadId_t xSendCANHandle;
const osThreadAttr_t xSendCAN_attributes = {
  .name = "xSendCAN",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for xCheckComms */
osThreadId_t xCheckCommsHandle;
const osThreadAttr_t xCheckComms_attributes = {
  .name = "xCheckComms",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for xReadTemp */
osThreadId_t xReadTempHandle;
const osThreadAttr_t xReadTemp_attributes = {
  .name = "xReadTemp",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
osMutexId_t tempBufferMutexHandle;
const osMutexAttr_t tempBufferMutex_attributes = {.name = "tempBufferMutex"};

extern uint8_t FDCAN1RxData[8]; //slaves
extern FDCAN_RxHeaderTypeDef FDCAN1RxHeader;
extern uint8_t FDCAN1TxData[8];
extern FDCAN_TxHeaderTypeDef FDCAN1TxHeader;

extern uint8_t FDCAN2RxData[8]; //slaves
extern FDCAN_RxHeaderTypeDef FDCAN2RxHeader;
extern uint8_t FDCAN2TxData[8];
extern FDCAN_TxHeaderTypeDef FDCAN2TxHeader;

CAN_RxMsg_t lastRx1Msg;
CAN_TxStatus_t lastTx1Status = CAN_TX_OK;

CAN_RxMsg_t lastRx2Msg;
CAN_TxStatus_t lastTx2Status = CAN_TX_OK;

extern volatile int tmsErrorCode;

extern float slaveTempBuffers[numberOfSlaves][thermistorsRecieved];

extern uint32_t slaveLastMessageTicks[numberOfSlaves];

uint32_t rawAdcBuffer[numberOfThermistors] = {0}; 
extern uint16_t filteredAdcBuffer[numberOfThermistors];
float voltageBuffer[numberOfThermistors] = {0}, tempBuffer[numberOfThermistors] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_ADC1_Init(void);
void xSendCANFunction(void *argument);
void xCheckCommsFuncion(void *argument);
void xReadTempFunction(void *argument);

/* USER CODE BEGIN PFP */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  tempBufferMutexHandle = osMutexNew(&tempBufferMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of xSendCAN */
  xSendCANHandle = osThreadNew(xSendCANFunction, NULL, &xSendCAN_attributes);

  /* creation of xCheckComms */
  xCheckCommsHandle = osThreadNew(xCheckCommsFuncion, NULL, &xCheckComms_attributes);

  /* creation of xReadTemp */
  xReadTempHandle = osThreadNew(xReadTempFunction, NULL, &xReadTemp_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
#ifdef testLoopbackCAN1
  hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
#else
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
#endif
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 26;
  hfdcan1.Init.NominalTimeSeg2 = 7;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0;
  sFilterConfig.FilterID2 = 0;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
	  Error_Handler();
  }

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);

#ifdef testLoopbackCAN1
  FDCAN1TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  FDCAN1TxHeader.IdType = FDCAN_STANDARD_ID;
  FDCAN1TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
#endif
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
#ifdef testLoopbackCAN2
  hfdcan2.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
#else
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
#endif
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 26;
  hfdcan2.Init.NominalTimeSeg2 = 7;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  FDCAN_FilterTypeDef sFilterConfig2;

  sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig2.FilterIndex = 0;
  sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig2.FilterID1 = 0;
  sFilterConfig2.FilterID2 = 0;

  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2) != HAL_OK) {
	  Error_Handler();
  }

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan2);

  FDCAN2TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  FDCAN2TxHeader.IdType = FDCAN_EXTENDED_ID;
  FDCAN2TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, shutdownPin_Pin|userLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : shutdownPin_Pin userLED_Pin */
  GPIO_InitStruct.Pin = shutdownPin_Pin|userLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (hfdcan->Instance == FDCAN1) {
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1RxHeader,
					FDCAN1RxData) == HAL_OK) {
				receiveCANFromSlaves();
			}
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	if (hfdcan->Instance == FDCAN2) {
		if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN2RxHeader,
					FDCAN2RxData) == HAL_OK) {
				receiveCANFromGeral();
			}
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
#ifdef simulateSlave
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(xReadTempHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_xSendCANFunction */
/**
  * @brief  Function implementing the xSendCAN thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_xSendCANFunction */
void xSendCANFunction(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  float maxTemps[numberOfSlaves] = {0};
	  for(int i = 0; i < numberOfSlaves; i++){
		  maxTemps[i] = findMaxVal(slaveTempBuffers[i]);
	  }

	  sendMasterInfoToCAN(maxTemps, tmsErrorCode);

#ifdef testLoopbackCAN1
	  simulateSlaveBurst(0, 0);
#endif

	  injectFault(&maxTemps[0]);

	  for(int i = 0; i < numberOfSlaves; i++){
		  if(maxTemps[i] > maxTemperatureThreshold){
			  tmsErrorCode |= overTemperatureFault;
			  Error_Handler();
		  }
	  }
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_xCheckCommsFuncion */
/**
* @brief Function implementing the xCheckComms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_xCheckCommsFuncion */
void xCheckCommsFuncion(void *argument)
{
  /* USER CODE BEGIN xCheckCommsFuncion */
	static uint32_t atualTick = 0;
  /* Infinite loop */
  for(;;)
  {
	  atualTick = HAL_GetTick();

	  for(int i = 0; i < numberOfSlaves; i++){
		  if(atualTick - slaveLastMessageTicks[i] > 2000){
			  tmsErrorCode |= commFault;
			  Error_Handler();
		  }
	  }
    osDelay(10); // Frequência de 100Hz para processamento CAN
  }
  /* USER CODE END xCheckCommsFuncion */
}

/* USER CODE BEGIN Header_xReadTempFunction */
/**
* @brief Function implementing the xReadTemp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_xReadTempFunction */
void xReadTempFunction(void *argument)
{
  /* USER CODE BEGIN xReadTempFunction */
	static bool filtersInitialized = false;

	/* Infinite loop */
	for(;;)
	{
#ifdef simulateSlave
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)rawAdcBuffer, numberOfThermistors);

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		HAL_ADC_Stop_DMA(&hadc1);

		if (!filtersInitialized) {
			initTemperatureFilters(rawAdcBuffer);
			filtersInitialized = true;
		}

		for (int i = 0; i < numberOfThermistors; i++) {

			/* Etapa 1: Filtro de mediana (janela 3) — elimina spikes */
			uint16_t medianADC = applyMedianFilter(rawAdcBuffer[i], i);

			/* Etapa 2: Filtro IIR (alpha=1/8) — suaviza o sinal */
			filteredAdcBuffer[i] = applyIIRFilter(medianADC, i);

			/* Etapa 3: Diagnóstico de integridade do sensor */
			readStatus = checkThermistorConnection(filteredAdcBuffer[i]);

			/* Etapa 4: Conversão e armazenamento (protegido por mutex) */
			if(readStatus == OK){
				if (osMutexAcquire(tempBufferMutexHandle, osWaitForever) == osOK) {
					tempBuffer[i] = convertVoltageToTemperature(
							convertBitsToVoltage(filteredAdcBuffer[i]));
					osMutexRelease(tempBufferMutexHandle);
				}
			}
			else{
				thermistorFault = 1;
				sendReadingErrorInfoIntoCAN();
				osDelay(5); // Garante que o hardware CAN transmita o erro antes de prosseguir
				Error_Handler();
			}
		}
#endif
		/* Aguarda 100ms antes da próxima leitura (taxa de aquisição: ~10 Hz) */
		osDelay(100);
	}
  /* USER CODE END xReadTempFunction */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
	sendMasterInfoToCAN(0, tmsErrorCode);
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
