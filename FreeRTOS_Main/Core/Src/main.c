/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "semphr.h"
#include "stdio.h"
#include "stream_buffer.h"
#include "FreeRTOS_CLI.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cliTask */
osThreadId_t cliTaskHandle;
const osThreadAttr_t cliTask_attributes = {
  .name = "cliTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for cliPrintTask */
osThreadId_t cliPrintTaskHandle;
const osThreadAttr_t cliPrintTask_attributes = {
  .name = "cliPrintTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for uart2TxSem */
osSemaphoreId_t uart2TxSemHandle;
const osSemaphoreAttr_t uart2TxSem_attributes = {
  .name = "uart2TxSem"
};
/* Definitions for uart2RxSem */
osSemaphoreId_t uart2RxSemHandle;
const osSemaphoreAttr_t uart2RxSem_attributes = {
  .name = "uart2RxSem"
};
/* USER CODE BEGIN PV */
StreamBufferHandle_t xStreamBufferCli = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void cliTaskFunction(void *argument);
void cliPrintTaskFunction(void *argument);

/* USER CODE BEGIN PFP */
static inline void vSendStringByStreamBuffer(StreamBufferHandle_t xStreamBuffer,
																						 const void *pvTxData,
																						 size_t xDataLengthBytes,
																						 TickType_t xTicksToWait);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Create and reset stream buffer for printing CLI output with trigger level 1
  xStreamBufferCli = xStreamBufferCreate(OUTPUT_BUFFER_LEN, 1);
  xStreamBufferReset(xStreamBufferCli);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart2TxSem */
  uart2TxSemHandle = osSemaphoreNew(1, 1, &uart2TxSem_attributes);

  /* creation of uart2RxSem */
  uart2RxSemHandle = osSemaphoreNew(1, 1, &uart2RxSem_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of cliTask */
  cliTaskHandle = osThreadNew(cliTaskFunction, NULL, &cliTask_attributes);

  /* creation of cliPrintTask */
  cliPrintTaskHandle = osThreadNew(cliPrintTaskFunction, NULL, &cliPrintTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

// UART transmission complete ISR
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *husart) {

	BaseType_t xStatus = pdFALSE;
	BaseType_t xYieldRequired = pdFALSE;

	//Check UART and give semaphore
	if (husart->Instance == USART2) {
		xStatus = xSemaphoreGiveFromISR(uart2TxSemHandle, &xYieldRequired);
		if (xStatus == pdTRUE) {
			portYIELD_FROM_ISR(xYieldRequired);
		}
	}
}

// UART data received ISR
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *husart) {

	BaseType_t xStatus = pdFALSE;
	BaseType_t xYieldRequired = pdFALSE;

	//Check UART and give semaphore
	if (husart->Instance == USART2) {
		xStatus = xSemaphoreGiveFromISR(uart2RxSemHandle, &xYieldRequired);
		if (xStatus == pdTRUE) {
			portYIELD_FROM_ISR(xYieldRequired);
		}
	}
}


// Wrapper function for sending data using a stream buffer
static inline void vSendStringByStreamBuffer(StreamBufferHandle_t xStreamBuffer,
																						 const void *pvTxData,
																						 size_t xDataLengthBytes,
																						 TickType_t xTicksToWait) {
	size_t xBytesToSend = xDataLengthBytes;
	size_t xBytesSent = 0;

	do {

		// Try to send all bytes
		xBytesSent += xStreamBufferSend(xStreamBuffer, pvTxData + xBytesSent, xBytesToSend, xTicksToWait);

		// Decrement bytes to send by number of bytes already sent
		if (xBytesSent != xBytesToSend) {
			xBytesToSend = xDataLengthBytes - xBytesSent;
		}

	} while (xBytesSent != xDataLengthBytes);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_cliTaskFunction */
/**
* @brief Function implementing the cliTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cliTaskFunction */
void cliTaskFunction(void *argument)
{
  /* USER CODE BEGIN cliTaskFunction */

	// Task variables
	BaseType_t xStatus = pdFALSE;
	uint8_t cInputBufferIndex = 0;
	char cInputBuffer[INPUT_BUFFER_LEN] = {0};
	char cOutputBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cRxChar = 0;
	char cTxChar = NEWPAGE_CHAR;
	const char *cWelcomeMessage = "FreeRTOS Command Line Interface. \r\ntype \"help\" to view a list of commands.\r\n";
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;

	// Take Rx semaphore and initiate DMA Rx transfer
	xSemaphoreTake(uart2RxSemHandle, portMAX_DELAY);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)&cRxChar, 1);

	// Send new-page character and welcome message to cliPrintTask
	vSendStringByStreamBuffer(xStreamBufferCli, &cTxChar, 1, pdMS_TO_TICKS(SHORT_DELAY));
	vSendStringByStreamBuffer(xStreamBufferCli, cWelcomeMessage, strlen(cWelcomeMessage), pdMS_TO_TICKS(SHORT_DELAY));

	for(;;)
  {
		// Take Rx semaphore
		xStatus = xSemaphoreTake(uart2RxSemHandle, pdMS_TO_TICKS(STD_DELAY));
		if (xStatus == pdTRUE)
		{
			// Copy received character and initiate next DMA transfer
			cTxChar = cRxChar;
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)&cRxChar, 1);

			// Check input
			if (cTxChar == '\r' || cTxChar == '\n')
			{
				// Newline received, echo carriage return and newline
				vSendStringByStreamBuffer(xStreamBufferCli, "\r\n", strlen("\r\n"), pdMS_TO_TICKS(STD_DELAY));

				// Check input buffer
				if (cInputBufferIndex > 0)
				{
					do
					{
						// Process next command in input buffer
						xStatus = FreeRTOS_CLIProcessCommand(cInputBuffer, cOutputBuffer, OUTPUT_BUFFER_LEN);

						// Check if command was processed correctly
						if (cOutputBuffer[0] != '\0')
						{
							// Send command output to the terminal and clear output buffer
							vSendStringByStreamBuffer(xStreamBufferCli, cOutputBuffer, strnlen(cOutputBuffer, OUTPUT_BUFFER_LEN), pdMS_TO_TICKS(STD_DELAY));
							memset(cOutputBuffer, '\0', OUTPUT_BUFFER_LEN);
						}
					}
					while (xStatus != pdFALSE);

					// Command processed, clear input buffer
					memset(cInputBuffer, '\0', INPUT_BUFFER_LEN);
					cInputBufferIndex = 0;
				}
			}
			else if ((cTxChar == DEL_CHAR) && (cInputBufferIndex > 0))
			{
				// 'Delete' character received, remove entry from input buffer
				cInputBuffer[--cInputBufferIndex] = '\0';
				vSendStringByStreamBuffer(xStreamBufferCli, &cTxChar, 1, pdMS_TO_TICKS(STD_DELAY));
			}
			else if (cInputBufferIndex < INPUT_BUFFER_LEN)
			{
				// Other character received
				cInputBuffer[cInputBufferIndex++] = cTxChar;
				vSendStringByStreamBuffer(xStreamBufferCli, &cTxChar, 1, pdMS_TO_TICKS(STD_DELAY));
			}
		}
		else
		{
			// Check USART state and call generic error handler in case of errors
			uartState = HAL_UART_GetState(&huart2);
			if (HAL_UART_STATE_ERROR == uartState)
				Error_Handler();
		}
  }
  /* USER CODE END cliTaskFunction */
}

/* USER CODE BEGIN Header_cliPrintTaskFunction */
/**
* @brief Function implementing the cliPrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cliPrintTaskFunction */
void cliPrintTaskFunction(void *argument)
{
  /* USER CODE BEGIN cliPrintTaskFunction */

	BaseType_t xStatus = pdFALSE;
	size_t xReceiveSize = 0;
	char cReceiveBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cSendBuffer[OUTPUT_BUFFER_LEN] = {0};
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;
	HAL_StatusTypeDef halStatus = HAL_OK;

  for(;;)
  {
  	// Read stream buffer
  	xReceiveSize = xStreamBufferReceive(xStreamBufferCli, cReceiveBuffer, OUTPUT_BUFFER_LEN, portMAX_DELAY);

  	if (xReceiveSize > 0) {

    	// Wait for previous transmission to complete
    	xStatus = xSemaphoreTake(uart2TxSemHandle, pdMS_TO_TICKS(STD_DELAY));

    	// Transmit bytes read from the stream buffer
    	if (pdTRUE == xStatus)
    	{
    		memcpy(cSendBuffer, cReceiveBuffer, xReceiveSize);
    		halStatus = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cSendBuffer, (uint16_t)xReceiveSize);

    		// Ensure that API call was successful
    		if (halStatus != HAL_OK)
    			xSemaphoreGive(uart2TxSemHandle);
    	}
    	else
    	{
				// Check USART state and call generic error handler in case of errors
				uartState = HAL_UART_GetState(&huart2);
				if (HAL_UART_STATE_ERROR == uartState)
					Error_Handler();
    	}
  	}
  }
  /* USER CODE END cliPrintTaskFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
