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
#include "stdlib.h"
#include "queue.h"
#include "stdarg.h"
#include "math.h"
#include "limits.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Enum with different data types sent via queue
typedef enum {
	BLINK_PERIOD,
	PITCH,
	ROLL,
} eDataType;

// Union for sending different data types via queue
typedef union {
	uint32_t ui;
	float f;
} unDataValue;

// Structure that encapsulates data sent via queue
typedef struct {
	eDataType type;				// enum
	unDataValue value;		// union
} Data_t;

// Structure that encapsulates strings sent via queue
typedef struct {
	char *pcString;									// pointer to string buffer
	size_t xStringSize;							// actual size of string
	size_t xStringMaxSize; 					// size of string buffer
	SemaphoreHandle_t pStringLock;	// semaphore used to guard access to string
} StringData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

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
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cpuTempTask */
osThreadId_t cpuTempTaskHandle;
const osThreadAttr_t cpuTempTask_attributes = {
  .name = "cpuTempTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for accelGyroTask */
osThreadId_t accelGyroTaskHandle;
const osThreadAttr_t accelGyroTask_attributes = {
  .name = "accelGyroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for eventTask */
osThreadId_t eventTaskHandle;
const osThreadAttr_t eventTask_attributes = {
  .name = "eventTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blinkPeriodQueue */
osMessageQueueId_t blinkPeriodQueueHandle;
const osMessageQueueAttr_t blinkPeriodQueue_attributes = {
  .name = "blinkPeriodQueue"
};
/* Definitions for printStringQueue */
osMessageQueueId_t printStringQueueHandle;
const osMessageQueueAttr_t printStringQueue_attributes = {
  .name = "printStringQueue"
};
/* Definitions for printDataQueue */
osMessageQueueId_t printDataQueueHandle;
const osMessageQueueAttr_t printDataQueue_attributes = {
  .name = "printDataQueue"
};
/* Definitions for cpuTempQueue */
osMessageQueueId_t cpuTempQueueHandle;
const osMessageQueueAttr_t cpuTempQueue_attributes = {
  .name = "cpuTempQueue"
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
/* Definitions for buttonSem */
osSemaphoreId_t buttonSemHandle;
const osSemaphoreAttr_t buttonSem_attributes = {
  .name = "buttonSem"
};
/* Definitions for buttonPrintSem */
osSemaphoreId_t buttonPrintSemHandle;
const osSemaphoreAttr_t buttonPrintSem_attributes = {
  .name = "buttonPrintSem"
};
/* Definitions for cpuTempPrintSem */
osSemaphoreId_t cpuTempPrintSemHandle;
const osSemaphoreAttr_t cpuTempPrintSem_attributes = {
  .name = "cpuTempPrintSem"
};
/* Definitions for adc1Sem */
osSemaphoreId_t adc1SemHandle;
const osSemaphoreAttr_t adc1Sem_attributes = {
  .name = "adc1Sem"
};
/* Definitions for accelGyroSem */
osSemaphoreId_t accelGyroSemHandle;
const osSemaphoreAttr_t accelGyroSem_attributes = {
  .name = "accelGyroSem"
};
/* Definitions for accelGyroPrintSem */
osSemaphoreId_t accelGyroPrintSemHandle;
const osSemaphoreAttr_t accelGyroPrintSem_attributes = {
  .name = "accelGyroPrintSem"
};
/* Definitions for eventPrintSem */
osSemaphoreId_t eventPrintSemHandle;
const osSemaphoreAttr_t eventPrintSem_attributes = {
  .name = "eventPrintSem"
};
/* USER CODE BEGIN PV */

// Stream buffer for passing messages between CLI tasks
StreamBufferHandle_t xStreamBufferCli = NULL;

// Array for storing jostick ADC readings
uint16_t sAdc2Read[2] = {0};

// Global variables for accelerometer/gyroscope readings
float gfAccelX = 0.0;
float gfAccelY = 0.0;
float gfAccelZ = 0.0;
float gfGyroX = 0.0;
float gfGyroY = 0.0;
float gfGyroZ = 0.0;
float gfPitch = 0.0;
float gfRoll = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void cliTaskFunction(void *argument);
void cliPrintTaskFunction(void *argument);
void ledTaskFunction(void *argument);
void printTaskFunction(void *argument);
void buttonTaskFunction(void *argument);
void cpuTempTaskFunction(void *argument);
void accelGyroTaskFunction(void *argument);
void eventTaskFunction(void *argument);

/* USER CODE BEGIN PFP */

// Wrapper for writing to the CLI stream buffer
static inline void vSendStringByStreamBuffer(StreamBufferHandle_t xStreamBuffer,
																						 const void *pvTxData,
																						 size_t xDataLengthBytes,
																						 TickType_t xTicksToWait);

// FreeRTOS CLI callback functions
BaseType_t prvClearCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvPrintCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvBlinkCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvCpuTempCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvPitchRollCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

// Utility functions for sending strings via queue
static void vInitPrintStringData(StringData_t *pPrintStringData, char *pcString, size_t xStringMaxSize, SemaphoreHandle_t pStringLock);
static inline BaseType_t xSendStringByQueue(StringData_t *pPrintStringData, QueueHandle_t printStringQueue, char *pcPrintString, ...);

// Utility functions for MPU6050 accelerometer communication
static BaseType_t xAccelGyroMemRead(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static BaseType_t xAccelGyroMemWrite(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static BaseType_t xAccelGyroSetValue(uint16_t MemAddress, uint8_t setValue, uint8_t valueMask);
static BaseType_t xAccelGyroInit(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FreeRTOS CLI command structs
const CLI_Command_Definition_t xClearCommand = {.pcCommand = "clear",
																								.pcHelpString = "clear: \r\n Clears screen\r\n",
																								.pxCommandInterpreter = prvClearCommand,
																								.cExpectedNumberOfParameters = 0 };
const CLI_Command_Definition_t xPrintCommand = {.pcCommand = "print",
																								.pcHelpString = "print: \r\n Prints <word> <number of times>\r\n",
																								.pxCommandInterpreter = prvPrintCommand,
																								.cExpectedNumberOfParameters = 2 };
const CLI_Command_Definition_t xLedCommand = {.pcCommand = "led",
																								.pcHelpString = "led:\r\n Led blinking control: led [on|off]\r\n",
																								.pxCommandInterpreter = prvLedCommand,
																								.cExpectedNumberOfParameters = 1 };
const CLI_Command_Definition_t xBlinkCommand = {.pcCommand = "blink",
																								.pcHelpString = "blink:\r\n Set led blinking period in ms: blink <time>\r\n",
																								.pxCommandInterpreter = prvBlinkCommand,
																								.cExpectedNumberOfParameters = 1 };
const CLI_Command_Definition_t xTempCommand = {.pcCommand = "temp",
																								.pcHelpString = "temp:\r\n Measures CPU temperature\r\n",
																								.pxCommandInterpreter = prvCpuTempCommand,
																								.cExpectedNumberOfParameters = 0 };
const CLI_Command_Definition_t xPitchRollCommand = {.pcCommand = "pitchroll",
																								.pcHelpString = "pitchroll:\r\n Prints latest pitch and roll values\r\n",
																								.pxCommandInterpreter = prvPitchRollCommand,
																								.cExpectedNumberOfParameters = 0 };

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

  // Register FreeRTOS CLI commands
  FreeRTOS_CLIRegisterCommand(&xClearCommand);
  FreeRTOS_CLIRegisterCommand(&xPrintCommand);
  FreeRTOS_CLIRegisterCommand(&xLedCommand);
  FreeRTOS_CLIRegisterCommand(&xBlinkCommand);
  FreeRTOS_CLIRegisterCommand(&xTempCommand);
  FreeRTOS_CLIRegisterCommand(&xPitchRollCommand);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Create and reset stream buffer for printing CLI output with trigger level 1
  xStreamBufferCli = xStreamBufferCreate(OUTPUT_BUFFER_LEN, 1);
  xStreamBufferReset(xStreamBufferCli);

  // Start continuous ADC conversions for joystick
  // HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&sAdc2Read[0], 2);

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

  /* creation of buttonSem */
  buttonSemHandle = osSemaphoreNew(1, 1, &buttonSem_attributes);

  /* creation of buttonPrintSem */
  buttonPrintSemHandle = osSemaphoreNew(1, 1, &buttonPrintSem_attributes);

  /* creation of cpuTempPrintSem */
  cpuTempPrintSemHandle = osSemaphoreNew(1, 1, &cpuTempPrintSem_attributes);

  /* creation of adc1Sem */
  adc1SemHandle = osSemaphoreNew(1, 1, &adc1Sem_attributes);

  /* creation of accelGyroSem */
  accelGyroSemHandle = osSemaphoreNew(1, 1, &accelGyroSem_attributes);

  /* creation of accelGyroPrintSem */
  accelGyroPrintSemHandle = osSemaphoreNew(1, 1, &accelGyroPrintSem_attributes);

  /* creation of eventPrintSem */
  eventPrintSemHandle = osSemaphoreNew(1, 1, &eventPrintSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of blinkPeriodQueue */
  blinkPeriodQueueHandle = osMessageQueueNew (1, sizeof(Data_t), &blinkPeriodQueue_attributes);

  /* creation of printStringQueue */
  printStringQueueHandle = osMessageQueueNew (10, sizeof(StringData_t), &printStringQueue_attributes);

  /* creation of printDataQueue */
  printDataQueueHandle = osMessageQueueNew (10, sizeof(Data_t), &printDataQueue_attributes);

  /* creation of cpuTempQueue */
  cpuTempQueueHandle = osMessageQueueNew (1, 4, &cpuTempQueue_attributes);

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

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(ledTaskFunction, NULL, &ledTask_attributes);

  /* creation of printTask */
  printTaskHandle = osThreadNew(printTaskFunction, NULL, &printTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(buttonTaskFunction, NULL, &buttonTask_attributes);

  /* creation of cpuTempTask */
  cpuTempTaskHandle = osThreadNew(cpuTempTaskFunction, NULL, &cpuTempTask_attributes);

  /* creation of accelGyroTask */
  accelGyroTaskHandle = osThreadNew(accelGyroTaskFunction, NULL, &accelGyroTask_attributes);

  /* creation of eventTask */
  eventTaskHandle = osThreadNew(eventTaskFunction, NULL, &eventTask_attributes);

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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

  /*Configure GPIO pin : MOTION_INT_Pin */
  GPIO_InitStruct.Pin = MOTION_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MOTION_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// UART data transferred ISR
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

// I2C data received ISR
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	BaseType_t xStatus = pdFALSE;
	BaseType_t xYieldRequired = pdFALSE;
	BaseType_t xYieldRequiredSem = pdFALSE;

	if (hi2c->Instance == I2C1) {

		// Notify accelerometer task and give semaphore
		xTaskNotifyFromISR(accelGyroTaskHandle, I2C_MEM_READ_CPLT, eSetBits, &xYieldRequired);
		xStatus = xSemaphoreGiveFromISR(accelGyroSemHandle, &xYieldRequiredSem);

		// Check if any of the calls caused a higher priority task to wake up
		if (xStatus == pdTRUE) {
			xYieldRequired |= xYieldRequiredSem;
		}

		// Yield if required
		portYIELD_FROM_ISR(xYieldRequired);
	}
}

// I2C data transferred ISR
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {

	BaseType_t xStatus = pdFALSE;
	BaseType_t xYieldRequired = pdFALSE;
	BaseType_t xYieldRequiredSem = pdFALSE;

	if (hi2c->Instance == I2C1) {

		// Notify accelerometer task and give semaphore
		xTaskNotifyFromISR(accelGyroTaskHandle, I2C_MEM_WRITE_CPLT, eSetBits, &xYieldRequired);
		xStatus = xSemaphoreGiveFromISR(accelGyroSemHandle, &xYieldRequiredSem);

		// Check if any of the calls caused a higher priority task to wake up
		if (xStatus == pdTRUE) {
			xYieldRequired |= xYieldRequiredSem;
		}

		// Yield if required
		portYIELD_FROM_ISR(xYieldRequired);
	}
}

// Blue button pushed ISR
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	BaseType_t xStatus = pdTRUE;
	BaseType_t xYieldRequired = 0;

	//Check UART and give semaphore
	if (GPIO_Pin == B1_Pin) {
		xStatus = xSemaphoreGiveFromISR(buttonSemHandle, &xYieldRequired);
	} else if (GPIO_Pin == MOTION_INT_Pin) {
		vTaskNotifyGiveFromISR(eventTaskHandle, &xYieldRequired);
	}

	if (pdTRUE == xStatus) {
		portYIELD_FROM_ISR(xYieldRequired);
	}
}

// ADC1 conversion complete ISR
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	BaseType_t xStatus = pdFALSE;
	BaseType_t xYieldRequired = pdFALSE;

	//Check UART and give semaphore
	if (hadc->Instance == ADC1) {
		xStatus = xSemaphoreGiveFromISR(adc1SemHandle, &xYieldRequired);
		vTaskNotifyGiveFromISR(cpuTempTaskHandle, &xYieldRequired);
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
		if (xBytesSent != xDataLengthBytes) {
			xBytesToSend = xDataLengthBytes - xBytesSent;
		}

	} while (xBytesSent != xDataLengthBytes);
}

static BaseType_t xAccelGyroMemRead(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	BaseType_t xStatus = pdFALSE;
	HAL_StatusTypeDef i2cStatus = HAL_OK;
	uint32_t xNotification = 0;

	// Take semaphore, which protects access to gyroscope (I2C1)
	xStatus = xSemaphoreTake(accelGyroSemHandle, pdMS_TO_TICKS(STD_DELAY));

	if (pdTRUE == xStatus) {

		//Start I2C DMA transfer
		i2cStatus = HAL_I2C_Mem_Read_DMA(&hi2c1, ACCELGYRO_DEVICE, MemAddress, MemAddSize, pData, Size);
		if (HAL_OK != i2cStatus) {
			xSemaphoreGive(accelGyroSemHandle);
			return pdFAIL;
		}

		// Wait for notification from ISR
		xStatus = xTaskNotifyWait(0, 0xffffffff, &xNotification, pdMS_TO_TICKS(SHORT_DELAY));
		if (pdPASS == xStatus && xNotification == I2C_MEM_READ_CPLT)
			return pdPASS;
	}

	return pdFAIL;
}

static BaseType_t xAccelGyroMemWrite(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	BaseType_t xStatus = pdFALSE;
	HAL_StatusTypeDef i2cStatus = HAL_OK;
	uint32_t xNotification = 0;

	// Take semaphore, which protects access to gyroscope (I2C1)
	xStatus = xSemaphoreTake(accelGyroSemHandle, pdMS_TO_TICKS(STD_DELAY));

	if (pdTRUE == xStatus) {

		//Start I2C DMA transfer
		i2cStatus = HAL_I2C_Mem_Write_DMA(&hi2c1, ACCELGYRO_DEVICE, MemAddress, MemAddSize, pData, Size);
		if (HAL_OK != i2cStatus) {
			xSemaphoreGive(accelGyroSemHandle);
			return pdFAIL;
		}

		// Wait for notification from ISR
		xStatus = xTaskNotifyWait(0, 0xffffffff, &xNotification, pdMS_TO_TICKS(SHORT_DELAY));
		if (pdPASS == xStatus && xNotification == I2C_MEM_WRITE_CPLT)
			return pdPASS;
	}

	return pdFAIL;
}

static BaseType_t xAccelGyroSetValue(uint16_t MemAddress, uint8_t setValue, uint8_t valueMask) {

	static uint8_t value = 0;
	BaseType_t xStatus = pdPASS;

	// Read single byte from register
	xStatus = xAccelGyroMemRead(MemAddress, 1, &value, 1);

	if (pdPASS == xStatus) {

		// Clear masked bits and set using new value
		value &= ~valueMask;
		value |= (setValue & valueMask);

		// Write value to register
		xStatus = xAccelGyroMemWrite(MemAddress, 1, &value, 1);
	}

	return xStatus;
}

static BaseType_t xAccelGyroInit(void) {

	BaseType_t xStatus = pdPASS;

	// Reset accelerometer
	xStatus = xAccelGyroSetValue(ACCELGYRO_PWR_MGMT_1_ADDR, ACCELGYRO_DEVICE_RESET_SET, ACCELGYRO_DEVICE_RESET_MASK);

	// Wait after reset
	if (pdPASS == xStatus)
		vTaskDelay(pdMS_TO_TICKS(SHORT_DELAY));

	// Disable energy saving mode
	if (pdPASS == xStatus)
		xStatus = xAccelGyroSetValue(ACCELGYRO_PWR_MGMT_1_ADDR, ACCELGYRO_SLEEP_RESET, ACCELGYRO_SLEEP_MASK);

	// Enable motion interrupt
	if (pdPASS == xStatus)
		xStatus = xAccelGyroSetValue(ACCELGYRO_INT_ENABLE_ADDR, ACCELGYRO_MOTION_INT_SET, ACCELGYRO_MOTION_INT_MASK);

	// Set counter threshold
	if (pdPASS == xStatus)
		xStatus = xAccelGyroSetValue(ACCELGYRO_MOTION_DUR_ADDR, ACCELGYRO_MOTION_DUR_5, ACCELGYRO_MOTION_DUR_MASK);

	// Set acceleration threshold
	if (pdPASS == xStatus)
		xStatus = xAccelGyroSetValue(ACCELGYRO_MOTION_THR_ADDR, ACCELGYRO_MOTION_THR_10, ACCELGYRO_MOTION_THR_MASK);

	return xStatus;
}

// FreeRTOS CLI "clear" command
BaseType_t prvClearCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	// Copy new-page character to output
	if (xWriteBufferLen > 0)
		strcpy(pcWriteBuffer, (char[]){NEWPAGE_CHAR, '\0'});

	return pdFALSE;
}

// FreeRTOS CLI "print" command
BaseType_t prvPrintCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	BaseType_t xParameter2StringLength = 0;
	const char *pcParameter1 = NULL;
	const char *pcParameter2 = NULL;
	static int32_t lCounter = 0;

	// Get first parameter
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameter2StringLength);

	// First iteration
	if (lCounter == 0) {

		// Get number of print iterations
		lCounter = atoi(pcParameter2);
		if (lCounter < 1) {
			lCounter = 0;
			strcpy(pcWriteBuffer, "[cli] Invalid command, value must be > 0\r\n");
			return pdFALSE;
		}
	}

	// Update command output
	strncpy(pcWriteBuffer, pcParameter1, xParameter1StringLength);
	lCounter--;

	// Check for final output
	if (lCounter == 0) {
		strcat(pcWriteBuffer, "\r\n");
		return pdFALSE;
	}

	// Continue processing
	return pdTRUE;
}

// FreeRTOS CLI "led" command
BaseType_t prvLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;

	// Get first parameter - LED on/off
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);

	// Interpret command
	if (strncmp(pcParameter1, "on", xParameter1StringLength) == 0)
	{
		strcpy(pcWriteBuffer, "[cli] Resuming LED task\r\n");
		vTaskResume(ledTaskHandle);
	}
	else if (strncmp(pcParameter1, "off", xParameter1StringLength) == 0)
	{
		strcpy(pcWriteBuffer, "[cli] Suspending LED task\r\n");
		vTaskSuspend(ledTaskHandle);
	}
	else
	{
		strcpy(pcWriteBuffer, "[cli] Invalid command\r\nsyntax: led [on|off]\r\n");
	}

	return pdFALSE;
}

// FreeRTOS CLI "blink" command
BaseType_t prvBlinkCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;
	Data_t blinkPeriodData = {.type = BLINK_PERIOD, .value.ui = 0};

	// Get first parameter - LED blink period in ms
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);

	// Get LED blink period
	blinkPeriodData.value.ui = (uint32_t)atoi(pcParameter1);

	// Reset queue and send new blink period
	xQueueReset(blinkPeriodQueueHandle);
	xQueueSend(blinkPeriodQueueHandle, &blinkPeriodData, 0);

	// Abort LED task delay
	xTaskAbortDelay(ledTaskHandle);
	snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Setting blinking period to %lu ms\r\n", blinkPeriodData.value.ui);

	// Resume LED task if it was suspended
	if (eTaskGetState(ledTaskHandle) == eSuspended) {
		vTaskResume(ledTaskHandle);
		strncat(pcWriteBuffer, "[cli] Resuming led blink task\r\n", xWriteBufferLen - strnlen(pcWriteBuffer, xWriteBufferLen));
	}

	return pdFALSE;
}

// FreeRTOS CLI "temp" command
BaseType_t prvCpuTempCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	vTaskResume(cpuTempTaskHandle);
	strncpy(pcWriteBuffer, "[cli] Resuming cpuTempTask\r\n", xWriteBufferLen);
	return pdFALSE;
}

// FreeRTOS CLI "temp" command
BaseType_t prvPitchRollCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	Data_t printValue = {0};

	// Print pitch value
	printValue.type = PITCH;
	printValue.value.f = gfPitch;
	xQueueSend(printDataQueueHandle, &printValue, 0);

	// Print roll value
	printValue.type = ROLL;
	printValue.value.f = gfRoll;
	xQueueSend(printDataQueueHandle, &printValue, 0);

	return pdFALSE;
}

// Initializes string data to be sent via queue
static void vInitPrintStringData(StringData_t *pPrintStringData, char *pcString, size_t xStringMaxSize, SemaphoreHandle_t pStringLock) {
	pPrintStringData->pcString = pcString;
	pPrintStringData->xStringMaxSize = xStringMaxSize;
	pPrintStringData->pStringLock = pStringLock;
}

// Constructs string data object from multiple arguments and sends via queue
static inline BaseType_t xSendStringByQueue(StringData_t *pPrintStringData, QueueHandle_t printStringQueue, char *pcPrintString, ...) {

	BaseType_t xStatus = pdFALSE;

	// Take semaphore to protect data in source string
	xStatus = xSemaphoreTake(pPrintStringData->pStringLock, pdMS_TO_TICKS(STD_DELAY));
	if (xStatus == pdTRUE) {

		// Construct string using additional function arguments
		va_list pAarg;
		va_start(pAarg, pcPrintString);
		pPrintStringData->xStringSize = vsnprintf(pPrintStringData->pcString, pPrintStringData->xStringMaxSize, pcPrintString, pAarg);
		va_end(pAarg);

		// Send data to print task queue
		xStatus = xQueueSend(printStringQueue, pPrintStringData, pdMS_TO_TICKS(STD_DELAY));
	}

	return xStatus;
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
		// Wait for new character and take Rx semaphore
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
						// Process CLI command
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
				// 'Delete' character received, remove from input buffer
				cInputBuffer[--cInputBufferIndex] = '\0';
				vSendStringByStreamBuffer(xStreamBufferCli, &cTxChar, 1, pdMS_TO_TICKS(STD_DELAY));
			}
			else if (cInputBufferIndex < INPUT_BUFFER_LEN)
			{
				// Save new character and echo to terminal
				cInputBuffer[cInputBufferIndex++] = cTxChar;
				vSendStringByStreamBuffer(xStreamBufferCli, &cTxChar, 1, pdMS_TO_TICKS(STD_DELAY));
			}
		}
		else
		{
			// Monitor USART state and call generic error handler in case of issues
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

    	// Wait for previous transmission to complete and take Tx semaphore
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

/* USER CODE BEGIN Header_ledTaskFunction */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledTaskFunction */
void ledTaskFunction(void *argument)
{
  /* USER CODE BEGIN ledTaskFunction */
  /* Infinite loop */
	BaseType_t xStatus = pdFALSE;
	uint32_t uiBlinkPeriod = STD_DELAY;
	TickType_t xLastWakeTime = 0;
	Data_t receiveData = {0};

	vTaskSuspend(NULL);

	for(;;)
  {
		// Save wake time in ticks
		xLastWakeTime = xTaskGetTickCount();

		// Read data from queue and update blink period
		xStatus = xQueueReceive(blinkPeriodQueueHandle, &receiveData, 0);
		if (xStatus == pdTRUE) {
			if (receiveData.type == BLINK_PERIOD) {
				uiBlinkPeriod = receiveData.value.ui;
				xQueueSend(printDataQueueHandle, &receiveData, portMAX_DELAY);
			}
		}

		// Toggle LED
  	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  	// Delay next toggle until precise time
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(uiBlinkPeriod));
  }
  /* USER CODE END ledTaskFunction */
}

/* USER CODE BEGIN Header_printTaskFunction */
/**
* @brief Function implementing the printTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printTaskFunction */
void printTaskFunction(void *argument)
{
  /* USER CODE BEGIN printTaskFunction */

	// Status variables
	BaseType_t xStatus = pdFALSE;
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;
	HAL_StatusTypeDef halStatus = HAL_OK;

	// Local buffers
	size_t xReceiveSize = 0;
	char cSendBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cReceiveBuffer[OUTPUT_BUFFER_LEN] = {0};

	// Queue data objects
	StringData_t printString = {0};
	Data_t printData = {0};

	// Queue set handles
	QueueSetHandle_t xPrintQueueSet = NULL;
	QueueSetMemberHandle_t xActivatedQueue = NULL;

	// Initialize queue set
	xPrintQueueSet = xQueueCreateSet(QUEUE_SIZE * 2);
	xQueueAddToSet(printStringQueueHandle, xPrintQueueSet);
	xQueueAddToSet(printDataQueueHandle, xPrintQueueSet);

  for(;;)
  {
  	// Read queue set
  	xActivatedQueue = xQueueSelectFromSet(xPrintQueueSet, portMAX_DELAY);

  	// Handle different queue types
  	if (xActivatedQueue == printStringQueueHandle)
  	{
  		// Read string queue
    	xStatus = xQueueReceive(printStringQueueHandle, &printString, 0);
    	if (xStatus == pdTRUE) {

    		// Copy string to local buffer and give semaphore
    		xReceiveSize = printString.xStringSize;
    		memcpy(cReceiveBuffer, printString.pcString, xReceiveSize);
    		xStatus = xSemaphoreGive(printString.pStringLock);
    	}
  	}
  	else if (xActivatedQueue == printDataQueueHandle)
  	{
  		// Read string queue
    	xStatus = xQueueReceive(printDataQueueHandle, &printData, 0);
    	if (xStatus == pdTRUE) {
    		switch (printData.type) {
    			case BLINK_PERIOD:
    				xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "[printTask] Blink period %li\r\n", printData.value.ui);
    				break;
    			case PITCH:
    				xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "[printTask] Pitch: %.2f\r\n", printData.value.f);
    				break;
    			case ROLL:
    				xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "[printTask] Roll: %.2f\r\n", printData.value.f);
    				break;
    			default:
    				// Unsupported data type
    				xStatus = pdFALSE;
    		}
    	}
  	}
  	else
  	{
  		// Unknown queue type, skip printing data via UART
  		xStatus = pdFALSE;
  	}

  	// Transmit data via UART
  	if (xStatus == pdTRUE)
  	{
  		do {
				// Wait for previous transmission to complete
				xStatus = xSemaphoreTake(uart2TxSemHandle, pdMS_TO_TICKS(STD_DELAY));
				if (xStatus == pdTRUE)
				{
					memcpy(cSendBuffer, cReceiveBuffer, xReceiveSize);
					halStatus = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cSendBuffer, (uint16_t)xReceiveSize);
					if (halStatus != HAL_OK)
						xSemaphoreGive(uart2TxSemHandle);
				}
				else
				{
					// Check UART for errors
					uartState = HAL_UART_GetState(&huart2);
					if (HAL_UART_STATE_ERROR == uartState)
						Error_Handler();
				}
			} while (xStatus != pdTRUE);
		}
  }
  /* USER CODE END printTaskFunction */
}

/* USER CODE BEGIN Header_buttonTaskFunction */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buttonTaskFunction */
void buttonTaskFunction(void *argument)
{
  /* USER CODE BEGIN buttonTaskFunction */
  BaseType_t xStatus = pdFALSE;
  StringData_t printString = {0};
  char cText[OUTPUT_BUFFER_LEN] = {0};

  // Initialize string data structure
  vInitPrintStringData(&printString, cText, sizeof(cText), buttonPrintSemHandle);

  // Initialize button pressed semaphore
  xSemaphoreTake(buttonSemHandle, 0);

  for(;;)
  {
  	// Wait for button press
  	xStatus = xSemaphoreTake(buttonSemHandle, pdMS_TO_TICKS(STD_DELAY));
   	if (xStatus == pdTRUE) {

   		// Suspend/resume LED task and send message to terminal
   		if (eTaskGetState(ledTaskHandle) != eSuspended) {
   			vTaskSuspend(ledTaskHandle);
   			xSendStringByQueue(&printString, printStringQueueHandle, "buttonTask - LED blink task suspended\r\n");
   		} else {
   			vTaskResume(ledTaskHandle);
   			xSendStringByQueue(&printString, printStringQueueHandle, "buttonTask - LED blink task resumed\r\n");
   		}

   		// Delay task to minimize noise from button
   		vTaskDelay(pdMS_TO_TICKS(SHORT_DELAY));
  	}
  }
  /* USER CODE END buttonTaskFunction */
}

/* USER CODE BEGIN Header_cpuTempTaskFunction */
/**
* @brief Function implementing the cpuTempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cpuTempTaskFunction */
void cpuTempTaskFunction(void *argument)
{
  /* USER CODE BEGIN cpuTempTaskFunction */

	// Local variables and buffers
	BaseType_t xStatus = pdFALSE;
	uint16_t sAdc1Value[2] = {0};
	uint16_t *psVRefRead = &sAdc1Value[0];
	uint16_t *psTempSenRead = &sAdc1Value[1];
	BaseType_t xNotification = 0;

	// Reference registers
	uint16_t *psVddRefCalVal = (uint16_t *) 0x1FFF7A2A;
	uint16_t *psAdcTempRefVal30 = (uint16_t *) 0x1FFF7A2C;
	uint16_t *psAdcTempRefVal110 = (uint16_t *) 0x1FFF7A2E;

	// Temperature calculations
	float fVAtTemp30 = (*psAdcTempRefVal30 * VDD) / MAX_ADC_VAL_12_BITS;
	float fVAtTemp110 = (*psAdcTempRefVal110 * VDD) / MAX_ADC_VAL_12_BITS;
	float fVTempSenSlope = (fVAtTemp30 - fVAtTemp110) / TEMP_110_MINUS_30;
	float fVdda = 0.0;
	float fVTempSens = 0.0;
	float fCpuTemp = 0.0;

	// Text buffer and string data
	char cText[OUTPUT_BUFFER_LEN] = {0};
	StringData_t printString = {0};

	vInitPrintStringData(&printString, cText, sizeof(cText), cpuTempPrintSemHandle);

  for(;;)
  {
    vTaskSuspend(NULL);
    xStatus = xSemaphoreTake(adc1SemHandle, pdMS_TO_TICKS(STD_DELAY));

    if (xStatus == pdTRUE) {

    	// Start ADC readings and wait for notification from ISR
    	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) sAdc1Value, 2);
    	xNotification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SHORT_DELAY));

    	// Calculate temperature and print results
    	if (xNotification != 0) {
				fVdda = (VDD * (*psVddRefCalVal)) / *psVRefRead;
				fVTempSens = (*psTempSenRead * fVdda) / MAX_ADC_VAL_12_BITS;
				fCpuTemp = ((fVAtTemp30 - fVTempSens) / fVTempSenSlope) + TEMP_30;
				xSendStringByQueue(&printString, printStringQueueHandle, "[cpuTempTask] CPU temperature: %.2fC\r\n", fCpuTemp);
    	}
    }
  }
  /* USER CODE END cpuTempTaskFunction */
}

/* USER CODE BEGIN Header_accelGyroTaskFunction */
/**
* @brief Function implementing the accelGyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_accelGyroTaskFunction */
void accelGyroTaskFunction(void *argument)
{
  /* USER CODE BEGIN accelGyroTaskFunction */

  BaseType_t xStatus = pdFALSE;
  char cText[OUTPUT_BUFFER_LEN] = {0};
  StringData_t printString = {0};

  // Accelerometer readings
	uint16_t sAccel[3] = {0};
	uint16_t sGyro[3] = {0};
	int16_t sAxisXRaw = 0;
	int16_t sAxisYRaw = 0;
	int16_t sAxisZRaw = 0;
	float fAxisX = 0.0;
	float fAxisY = 0.0;
	float fAxisZ = 0.0;
	float fPitch = 0.0;
	float fRoll = 0.0;

	// Initialize struct used for printing data
	vInitPrintStringData(&printString, cText, sizeof(cText), accelGyroPrintSemHandle);

	// Initialize accelerometer
	xStatus = xAccelGyroInit();
	if (pdFAIL == xStatus) {
		xSendStringByQueue(&printString, printStringQueueHandle, "[accelGyroTask] accelGyro init failed\r\n");
	}

	/* Infinite loop */
  for(;;)
  {
  	// Read X, Y and Z axis accelerometer values (2x 1-byte register per axis, 6 bytes total)
  	xAccelGyroMemRead(ACCELGYRO_ACCEL_ADDR, 1, (uint8_t *)&sAccel[0], sizeof(sAccel));

  	// Swap big endian for little endian in each register
  	sAxisXRaw = SWAP_UINT16(sAccel[0]);
  	sAxisYRaw = SWAP_UINT16(sAccel[1]);
  	sAxisZRaw = SWAP_UINT16(sAccel[2]);

  	// Convert to acceleration readings
  	gfAccelX = fAxisX = (float) (sAxisXRaw * ACCELGYRO_DEFAULT_ACCEL) / SHRT_MAX;
  	gfAccelY = fAxisY = (float) (sAxisYRaw * ACCELGYRO_DEFAULT_ACCEL) / SHRT_MAX;
  	gfAccelZ = fAxisZ = (float) (sAxisZRaw * ACCELGYRO_DEFAULT_ACCEL) / SHRT_MAX;

  	// Calculate pitch and roll
  	gfPitch = fPitch = atan2(fAxisY, fAxisZ) * 57.3;
  	gfRoll = fRoll = atan2((-fAxisX), sqrt(fAxisY * fAxisY + fAxisZ * fAxisZ)) * 57.3;

  	// Read X, Y and Z axis gyroscope values (2x 1-byte register per axis, 6 bytes total)
  	xAccelGyroMemRead(ACCELGYRO_GYRO_ADDR, 1, (uint8_t *)&sGyro[0], sizeof(sGyro));

  	// Swap big endian for little endian in each register
  	sAxisXRaw = SWAP_UINT16(sGyro[0]);
  	sAxisYRaw = SWAP_UINT16(sGyro[1]);
  	sAxisZRaw = SWAP_UINT16(sGyro[2]);

  	// Convert to gyroscope readings
  	gfGyroX = fAxisX = (float) (sAxisXRaw * ACCELGYRO_DEFAULT_GYRO) / SHRT_MAX;
  	gfGyroY = fAxisY = (float) (sAxisYRaw * ACCELGYRO_DEFAULT_GYRO) / SHRT_MAX;
  	gfGyroZ = fAxisZ = (float) (sAxisZRaw * ACCELGYRO_DEFAULT_GYRO) / SHRT_MAX;

  	// Sample every 50ms
  	vTaskDelay(pdMS_TO_TICKS(50));

  }
  /* USER CODE END accelGyroTaskFunction */
}

/* USER CODE BEGIN Header_eventTaskFunction */
/**
* @brief Function implementing the eventTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_eventTaskFunction */
void eventTaskFunction(void *argument)
{
  /* USER CODE BEGIN eventTaskFunction */

	uint32_t xNotification = 0;
	char cText[OUTPUT_BUFFER_LEN] = {0};
	StringData_t printString = {0};

	vInitPrintStringData(&printString, cText, sizeof(cText), eventPrintSemHandle);

  for(;;)
  {
    xNotification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xNotification != 0) {
    	xSendStringByQueue(&printString, printStringQueueHandle, "eventTask: Motion interrupt!\r\n");
    }
  }
  /* USER CODE END eventTaskFunction */
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
