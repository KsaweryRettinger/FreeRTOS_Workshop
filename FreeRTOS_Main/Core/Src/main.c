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
#include "stdbool.h"
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
#include "event_groups.h"
#include "SSD1331.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Enum with different data types sent via queue
typedef enum {
	BLINK_PERIOD,
	PITCH,
	ROLL,
	CPUTEMP,
	MOTION,
	DISTANCE,
	TEMPERATURE,
	HUMIDITY,
	PITCHANDROLL,
	JOYSTICK,
	MOTORS,
	TILT
} eDataType;

// Union for sending different data types via queue
typedef union {
	uint32_t ui;
	uint8_t uc;
	float f;
	float pr[2];
	uint32_t joy[2];
	uint32_t motors[2];
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

// Timer callback struct
typedef struct {
  osTimerFunc_t func;
  void         *arg;
} TimerCallback_t;

// Configuration struct, saved to / loaded from flash memory
typedef struct {
	TickType_t ulLedTimPeriod;
	TickType_t ulOledTimPeriod;
} Config_t;

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

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim5_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
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
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for streamPrintTask */
osThreadId_t streamPrintTaskHandle;
const osThreadAttr_t streamPrintTask_attributes = {
  .name = "streamPrintTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for distTask */
osThreadId_t distTaskHandle;
const osThreadAttr_t distTask_attributes = {
  .name = "distTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for distTriggerTask */
osThreadId_t distTriggerTaskHandle;
const osThreadAttr_t distTriggerTask_attributes = {
  .name = "distTriggerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tempHumTask */
osThreadId_t tempHumTaskHandle;
const osThreadAttr_t tempHumTask_attributes = {
  .name = "tempHumTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for saveLoadTask */
osThreadId_t saveLoadTaskHandle;
const osThreadAttr_t saveLoadTask_attributes = {
  .name = "saveLoadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bleCliTask */
osThreadId_t bleCliTaskHandle;
const osThreadAttr_t bleCliTask_attributes = {
  .name = "bleCliTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for blePrintTask */
osThreadId_t blePrintTaskHandle;
const osThreadAttr_t blePrintTask_attributes = {
  .name = "blePrintTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for bleStrPrintTask */
osThreadId_t bleStrPrintTaskHandle;
const osThreadAttr_t bleStrPrintTask_attributes = {
  .name = "bleStrPrintTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for blePanelTask */
osThreadId_t blePanelTaskHandle;
const osThreadAttr_t blePanelTask_attributes = {
  .name = "blePanelTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorsTask */
osThreadId_t motorsTaskHandle;
const osThreadAttr_t motorsTask_attributes = {
  .name = "motorsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tiltCheckTask */
osThreadId_t tiltCheckTaskHandle;
const osThreadAttr_t tiltCheckTask_attributes = {
  .name = "tiltCheckTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for oledDataQueue */
osMessageQueueId_t oledDataQueueHandle;
const osMessageQueueAttr_t oledDataQueue_attributes = {
  .name = "oledDataQueue"
};
/* Definitions for blePrintDataQueue */
osMessageQueueId_t blePrintDataQueueHandle;
const osMessageQueueAttr_t blePrintDataQueue_attributes = {
  .name = "blePrintDataQueue"
};
/* Definitions for joyDataQueue */
osMessageQueueId_t joyDataQueueHandle;
const osMessageQueueAttr_t joyDataQueue_attributes = {
  .name = "joyDataQueue"
};
/* Definitions for tiltDataQueue */
osMessageQueueId_t tiltDataQueueHandle;
const osMessageQueueAttr_t tiltDataQueue_attributes = {
  .name = "tiltDataQueue"
};
/* Definitions for tempHumTim */
osTimerId_t tempHumTimHandle;
const osTimerAttr_t tempHumTim_attributes = {
  .name = "tempHumTim"
};
/* Definitions for oledTim */
osTimerId_t oledTimHandle;
const osTimerAttr_t oledTim_attributes = {
  .name = "oledTim"
};
/* Definitions for ledTim */
osTimerId_t ledTimHandle;
const osTimerAttr_t ledTim_attributes = {
  .name = "ledTim"
};
/* Definitions for printStringMutex */
osMutexId_t printStringMutexHandle;
const osMutexAttr_t printStringMutex_attributes = {
  .name = "printStringMutex"
};
/* Definitions for blePrintStringMutex */
osMutexId_t blePrintStringMutexHandle;
const osMutexAttr_t blePrintStringMutex_attributes = {
  .name = "blePrintStringMutex"
};
/* Definitions for uart2TxSem */
osSemaphoreId_t uart2TxSemHandle;
const osSemaphoreAttr_t uart2TxSem_attributes = {
  .name = "uart2TxSem"
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
/* Definitions for eventPrintSem */
osSemaphoreId_t eventPrintSemHandle;
const osSemaphoreAttr_t eventPrintSem_attributes = {
  .name = "eventPrintSem"
};
/* Definitions for uart1TxSem */
osSemaphoreId_t uart1TxSemHandle;
const osSemaphoreAttr_t uart1TxSem_attributes = {
  .name = "uart1TxSem"
};
/* Definitions for commonEvent */
osEventFlagsId_t commonEventHandle;
const osEventFlagsAttr_t commonEvent_attributes = {
  .name = "commonEvent"
};
/* USER CODE BEGIN PV */

// CLI stream buffers
StreamBufferHandle_t xStreamBufferString = NULL;
StreamBufferHandle_t xStreamBufferStringBle = NULL;

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

// OLED screen variables
uint16_t screenBuffer[OLED_SCREEN_HEIGHT][OLED_SCREEN_WIDTH] = {0};
extern uint8_t mobicaLogo[];
extern size_t mobicaLogoSize;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void cliTaskFunction(void *argument);
void streamPrintTaskFunction(void *argument);
void printTaskFunction(void *argument);
void cpuTempTaskFunction(void *argument);
void accelGyroTaskFunction(void *argument);
void eventTaskFunction(void *argument);
void oledTaskFunction(void *argument);
void distTaskFunction(void *argument);
void distTriggerFunction(void *argument);
void tempHumTaskFunction(void *argument);
void saveLoadTaskFunction(void *argument);
void bleCliTaskFunction(void *argument);
void blePrintTaskFunction(void *argument);
void bleStrPrintTaskFunction(void *argument);
void blePanelTaskFunction(void *argument);
void motorsTaskFunction(void *argument);
void tiltCheckTaskFunction(void *argument);
void tempHumTimCallback(void *argument);
void oledTimCallback(void *argument);
void ledTimCallback(void *argument);

/* USER CODE BEGIN PFP */

// Wrapper for writing to the CLI stream buffer
static inline BaseType_t xSendStringByStreamBuffer(StreamBufferHandle_t xStreamBuffer,
																						 SemaphoreHandle_t printStringMutex,
																						 const void *pvTxData,
																						 size_t xDataLengthBytes,
																						 TickType_t xTicksToWait);

// Functions for sending strings via queue
static void vInitPrintStringData(StringData_t *pPrintStringData, char *pcString, size_t xStringMaxSize, SemaphoreHandle_t pStringLock);
static inline BaseType_t xSendStringByQueue(StringData_t *pPrintStringData, QueueHandle_t printStringQueue, char *pcPrintString, ...);

// Functions for MPU6050 accelerometer communication
static BaseType_t xAccelGyroMemRead(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static BaseType_t xAccelGyroMemWrite(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static BaseType_t xAccelGyroSetValue(uint16_t MemAddress, uint8_t setValue, uint8_t valueMask);
static BaseType_t xAccelGyroInit(void);

// Functions for saving/loading data from flash memory
static void vSaveToFlash(uint8_t *src, size_t size);
static void vLoadFromFlash(uint8_t *dst, size_t size);

// Function for calculating jostick vector length
static int32_t vectorLength(int32_t X, int32_t Y);

// FreeRTOS CLI callback functions
BaseType_t prvClearCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvBlinkCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvCpuTempCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvPitchRollCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvOledCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvResetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvOledPeriodCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvSaveCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvLoadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvPanelCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvJoyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvListCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FreeRTOS CLI command structs
const CLI_Command_Definition_t xClearCommand = {.pcCommand = "clear",
																								.pcHelpString = "clear: \r\n Clears screen\r\n",
																								.pxCommandInterpreter = prvClearCommand,
																								.cExpectedNumberOfParameters = 0 };
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
const CLI_Command_Definition_t xOledCommand = {.pcCommand = "oled",
																								.pcHelpString = "oled:\r\n Turn display on/off: oled [on|off]\r\n",
																								.pxCommandInterpreter = prvOledCommand,
																								.cExpectedNumberOfParameters = 1 };
const CLI_Command_Definition_t xResetCommand = {.pcCommand = "reset",
																								.pcHelpString = "reset:\r\n Reset nucleo device\r\n",
																								.pxCommandInterpreter = prvResetCommand,
																								.cExpectedNumberOfParameters = 0 };
const CLI_Command_Definition_t xOledPeriodCommand = {.pcCommand = "oledperiod",
																								.pcHelpString = "oledperiod:\r\n Set oled timeout in ms: oledperiod <time>\r\n",
																								.pxCommandInterpreter = prvOledPeriodCommand,
																								.cExpectedNumberOfParameters = 1 };
const CLI_Command_Definition_t xSaveCommand = {.pcCommand = "save",
																								.pcHelpString = "save:\r\n Save configuration to flash memory\r\n",
																								.pxCommandInterpreter = prvSaveCommand,
																								.cExpectedNumberOfParameters = 0 };
const CLI_Command_Definition_t xLoadCommand = {.pcCommand = "load",
																								.pcHelpString = "load:\r\n Load configuration from flash memory\r\n",
																								.pxCommandInterpreter = prvLoadCommand,
																								.cExpectedNumberOfParameters = 0 };
const CLI_Command_Definition_t xPanelCommand = {.pcCommand = "panel",
																								.pcHelpString = "panel:\r\n Sends panel configuration to BLE phone app\r\n",
																								.pxCommandInterpreter = prvPanelCommand,
																								.cExpectedNumberOfParameters = 0 };
const CLI_Command_Definition_t xJoyCommand = {.pcCommand = "joy",
																								.pcHelpString = "joy:\r\n Controls motors power: joy X<value>Y<value>\r\n",
																								.pxCommandInterpreter = prvJoyCommand,
																								.cExpectedNumberOfParameters = 1 };
const CLI_Command_Definition_t xListCommand = {.pcCommand = "list",
																								.pcHelpString = "list:\r\n Lists all FreeRTOS tasks\r\n",
																								.pxCommandInterpreter = prvListCommand,
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
  FreeRTOS_CLIRegisterCommand(&xLedCommand);
  FreeRTOS_CLIRegisterCommand(&xBlinkCommand);
  FreeRTOS_CLIRegisterCommand(&xTempCommand);
  FreeRTOS_CLIRegisterCommand(&xPitchRollCommand);
  FreeRTOS_CLIRegisterCommand(&xOledCommand);
  FreeRTOS_CLIRegisterCommand(&xResetCommand);
  FreeRTOS_CLIRegisterCommand(&xOledPeriodCommand);
  FreeRTOS_CLIRegisterCommand(&xSaveCommand);
  FreeRTOS_CLIRegisterCommand(&xLoadCommand);
  FreeRTOS_CLIRegisterCommand(&xPanelCommand);
  FreeRTOS_CLIRegisterCommand(&xJoyCommand);
  FreeRTOS_CLIRegisterCommand(&xListCommand);

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
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Create and reset stream buffer for printing CLI output with trigger level 1
  xStreamBufferString = xStreamBufferCreate(OUTPUT_BUFFER_LEN, 1);
  xStreamBufferStringBle = xStreamBufferCreate(OUTPUT_BUFFER_LEN, 1);
  xStreamBufferReset(xStreamBufferString);
  xStreamBufferReset(xStreamBufferStringBle);

  // Start continuous ADC conversions for joystick
  // HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&sAdc2Read[0], 2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of printStringMutex */
  printStringMutexHandle = osMutexNew(&printStringMutex_attributes);

  /* creation of blePrintStringMutex */
  blePrintStringMutexHandle = osMutexNew(&blePrintStringMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart2TxSem */
  uart2TxSemHandle = osSemaphoreNew(1, 1, &uart2TxSem_attributes);

  /* creation of adc1Sem */
  adc1SemHandle = osSemaphoreNew(1, 1, &adc1Sem_attributes);

  /* creation of accelGyroSem */
  accelGyroSemHandle = osSemaphoreNew(1, 1, &accelGyroSem_attributes);

  /* creation of eventPrintSem */
  eventPrintSemHandle = osSemaphoreNew(1, 1, &eventPrintSem_attributes);

  /* creation of uart1TxSem */
  uart1TxSemHandle = osSemaphoreNew(1, 1, &uart1TxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of tempHumTim */
  tempHumTimHandle = osTimerNew(tempHumTimCallback, osTimerPeriodic, NULL, &tempHumTim_attributes);

  /* creation of oledTim */
  oledTimHandle = osTimerNew(oledTimCallback, osTimerOnce, NULL, &oledTim_attributes);

  /* creation of ledTim */
  ledTimHandle = osTimerNew(ledTimCallback, osTimerPeriodic, NULL, &ledTim_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of printStringQueue */
  printStringQueueHandle = osMessageQueueNew (10, sizeof(StringData_t), &printStringQueue_attributes);

  /* creation of printDataQueue */
  printDataQueueHandle = osMessageQueueNew (10, sizeof(Data_t), &printDataQueue_attributes);

  /* creation of oledDataQueue */
  oledDataQueueHandle = osMessageQueueNew (10, sizeof(Data_t), &oledDataQueue_attributes);

  /* creation of blePrintDataQueue */
  blePrintDataQueueHandle = osMessageQueueNew (10, sizeof(Data_t), &blePrintDataQueue_attributes);

  /* creation of joyDataQueue */
  joyDataQueueHandle = osMessageQueueNew (10, sizeof(Data_t), &joyDataQueue_attributes);

  /* creation of tiltDataQueue */
  tiltDataQueueHandle = osMessageQueueNew (10, sizeof(Data_t), &tiltDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of cliTask */
  cliTaskHandle = osThreadNew(cliTaskFunction, NULL, &cliTask_attributes);

  /* creation of streamPrintTask */
  streamPrintTaskHandle = osThreadNew(streamPrintTaskFunction, NULL, &streamPrintTask_attributes);

  /* creation of printTask */
  printTaskHandle = osThreadNew(printTaskFunction, NULL, &printTask_attributes);

  /* creation of cpuTempTask */
  cpuTempTaskHandle = osThreadNew(cpuTempTaskFunction, NULL, &cpuTempTask_attributes);

  /* creation of accelGyroTask */
  accelGyroTaskHandle = osThreadNew(accelGyroTaskFunction, NULL, &accelGyroTask_attributes);

  /* creation of eventTask */
  eventTaskHandle = osThreadNew(eventTaskFunction, NULL, &eventTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(oledTaskFunction, NULL, &oledTask_attributes);

  /* creation of distTask */
  distTaskHandle = osThreadNew(distTaskFunction, NULL, &distTask_attributes);

  /* creation of distTriggerTask */
  distTriggerTaskHandle = osThreadNew(distTriggerFunction, NULL, &distTriggerTask_attributes);

  /* creation of tempHumTask */
  tempHumTaskHandle = osThreadNew(tempHumTaskFunction, NULL, &tempHumTask_attributes);

  /* creation of saveLoadTask */
  saveLoadTaskHandle = osThreadNew(saveLoadTaskFunction, NULL, &saveLoadTask_attributes);

  /* creation of bleCliTask */
  bleCliTaskHandle = osThreadNew(bleCliTaskFunction, NULL, &bleCliTask_attributes);

  /* creation of blePrintTask */
  blePrintTaskHandle = osThreadNew(blePrintTaskFunction, NULL, &blePrintTask_attributes);

  /* creation of bleStrPrintTask */
  bleStrPrintTaskHandle = osThreadNew(bleStrPrintTaskFunction, NULL, &bleStrPrintTask_attributes);

  /* creation of blePanelTask */
  blePanelTaskHandle = osThreadNew(blePanelTaskFunction, NULL, &blePanelTask_attributes);

  /* creation of motorsTask */
  motorsTaskHandle = osThreadNew(motorsTaskFunction, NULL, &motorsTask_attributes);

  /* creation of tiltCheckTask */
  tiltCheckTaskHandle = osThreadNew(tiltCheckTaskFunction, NULL, &tiltCheckTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of commonEvent */
  commonEventHandle = osEventFlagsNew(&commonEvent_attributes);

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_RES_Pin|M1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_CS_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_RES_Pin */
  GPIO_InitStruct.Pin = OLED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_CS_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTION_INT_Pin */
  GPIO_InitStruct.Pin = MOTION_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MOTION_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_EN_Pin */
  GPIO_InitStruct.Pin = M1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(M1_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_EN_Pin */
  GPIO_InitStruct.Pin = M2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(M2_EN_GPIO_Port, &GPIO_InitStruct);

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
	} else if (husart->Instance == USART1) {
		xStatus = xSemaphoreGiveFromISR(uart1TxSemHandle, &xYieldRequired);
		if (xStatus == pdTRUE) {
			portYIELD_FROM_ISR(xYieldRequired);
		}
	}
}

// UART data received ISR
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *husart) {

	BaseType_t xYieldRequired = pdFALSE;

	//Check UART and give semaphore
	if (husart->Instance == USART2) {
		vTaskNotifyGiveFromISR(cliTaskHandle, &xYieldRequired);
		portYIELD_FROM_ISR(xYieldRequired);
	} else if (husart->Instance == USART1) {
		vTaskNotifyGiveFromISR(bleCliTaskHandle, &xYieldRequired);
		portYIELD_FROM_ISR(xYieldRequired);
	}
}

// I2C data received ISR
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	BaseType_t xStatus = pdFALSE;
	BaseType_t xYieldRequired = pdFALSE;
	BaseType_t xYieldRequiredSem = pdFALSE;
	EventBits_t xBitsToSet = I2C_MEM_READ_EVENT;

	if (hi2c->Instance == I2C1) {

		// Notify accelerometer task and give semaphore
		xEventGroupSetBitsFromISR(commonEventHandle, xBitsToSet, &xYieldRequired);
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

	if (hi2c->Instance == I2C1) {
		xStatus = xSemaphoreGiveFromISR(accelGyroSemHandle, &xYieldRequired);
		if (xStatus == pdTRUE) {
			portYIELD_FROM_ISR(xYieldRequired);
		}
	}
}

// GPIO event ISR
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	EventBits_t xBitsToSet = 0;
	BaseType_t xYieldRequired = 0;

	//Check UART and give semaphore
	if (GPIO_Pin == B1_Pin) {
		xBitsToSet = BUTTON_EVENT;
		xEventGroupSetBitsFromISR(commonEventHandle, xBitsToSet, &xYieldRequired);
	} else if (GPIO_Pin == MOTION_INT_Pin) {
		xBitsToSet = MOTION_INT_EVENT;
		xEventGroupSetBitsFromISR(commonEventHandle, xBitsToSet, &xYieldRequired);
	}

	portYIELD_FROM_ISR(xYieldRequired);
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

// Timer input capture ISR
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	BaseType_t xYieldRequired = pdFALSE;

	// Check for echo from distance sensor
	if (htim->Instance == TIM5)
		vTaskNotifyGiveFromISR(distTaskHandle, &xYieldRequired);
	portYIELD_FROM_ISR(xYieldRequired);

	// Check for echo from distance sensor
	if (htim->Instance == TIM1)
		xTaskNotifyFromISR(tempHumTaskHandle, TEMP_HUM_IC_CPLT, eSetBits, &xYieldRequired);
	portYIELD_FROM_ISR(xYieldRequired);
}

// Wrapper function for sending data using a stream buffer
static inline BaseType_t xSendStringByStreamBuffer(StreamBufferHandle_t xStreamBuffer,
																						 SemaphoreHandle_t printStringMutex,
																						 const void *pvTxData,
																						 size_t xDataLengthBytes,
																						 TickType_t xTicksToWait) {
	size_t xBytesToSend = xDataLengthBytes;
	size_t xBytesSent = 0;
	BaseType_t xStatus = pdFALSE;

	if (0 == xDataLengthBytes) {
		xDataLengthBytes = strlen(pvTxData);
		xBytesToSend = xDataLengthBytes;
	}

	// Take mutex
	xStatus = xSemaphoreTake(printStringMutex, xTicksToWait);

	//  Send string
	if (pdPASS == xStatus) {
		do {
			xBytesSent += xStreamBufferSend(xStreamBuffer, pvTxData + xBytesSent, xBytesToSend, xTicksToWait);
			if (xBytesSent != xDataLengthBytes) {
				xBytesToSend = xDataLengthBytes - xBytesSent;
			}
		} while (xBytesSent != xDataLengthBytes);

		// Release mutex
		xStatus = xSemaphoreGive(printStringMutex);

		// Yield to same or lower priority task
		taskYIELD();
	}

	return xStatus;
}

static BaseType_t xAccelGyroMemRead(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	BaseType_t xStatus = pdPASS;
	HAL_StatusTypeDef i2cStatus = HAL_OK;
	EventBits_t xEventGroupValue = 0;
	EventBits_t xBitsToWaitFor = I2C_MEM_READ_EVENT;

	// Take semaphore, which protects access to gyroscope (I2C1)
	xStatus = xSemaphoreTake(accelGyroSemHandle, pdMS_TO_TICKS(STD_DELAY));

	// Read from gyroscope
	if (pdTRUE == xStatus) {
		i2cStatus = HAL_I2C_Mem_Read_DMA(&hi2c1, ACCELGYRO_DEVICE, MemAddress, MemAddSize, pData, Size);
		if (HAL_OK != i2cStatus) {
			xSemaphoreGive(accelGyroSemHandle);
			return pdFAIL;
		}

		// Wait for I2C event bit to be set
		xEventGroupValue = xEventGroupWaitBits(commonEventHandle, xBitsToWaitFor, pdTRUE, pdTRUE, pdMS_TO_TICKS(SHORT_DELAY));
		if (pdPASS == xStatus && (xEventGroupValue & xBitsToWaitFor))
			return pdPASS;
	}

	return pdFAIL;
}

static BaseType_t xAccelGyroMemWrite(uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	BaseType_t xStatus = pdPASS;
	HAL_StatusTypeDef i2cStatus = HAL_OK;

	// Take semaphore, which protects access to gyroscope (I2C1)
	xStatus = xSemaphoreTake(accelGyroSemHandle, pdMS_TO_TICKS(STD_DELAY));

	// Write to gyroscope
	if (pdTRUE == xStatus) {
		i2cStatus = HAL_I2C_Mem_Write_DMA(&hi2c1, ACCELGYRO_DEVICE, MemAddress, MemAddSize, pData, Size);
		if (HAL_OK != i2cStatus) {
			xSemaphoreGive(accelGyroSemHandle);
			return pdFAIL;
		}
	}

	return pdPASS;
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

static void vSaveToFlash(uint8_t *src, size_t size) {

	// Structure used to erase flash sector 7
	FLASH_EraseInitTypeDef flashErase = {.TypeErase = FLASH_TYPEERASE_SECTORS,
																			 .Sector = FLASH_SECTOR_7,
																			 .NbSectors = 1,
																			 .VoltageRange = FLASH_VOLTAGE_RANGE_3};

	// Start address of flash sector 7
	void *pvSector7 = (void *)FLASH_SECTOR7_ADDR;
	uint32_t SectorErr = 0;

	// Start of critical section
	taskENTER_CRITICAL();

	// Unlock flash and clear all flags
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
												 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_RDERR | FLASH_FLAG_BSY);

	// Erase flash sector 7
	HAL_FLASHEx_Erase(&flashErase, &SectorErr);

	// Write all bytes to sector 7
	for (size_t i = 0; i < size; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)pvSector7 + i, (uint64_t)src[i]);
	}

	// Lock flash
	HAL_FLASH_Lock();

	// End of critical section
	taskEXIT_CRITICAL();
}

static void vLoadFromFlash(uint8_t *dst, size_t size) {

	// pointer to the beginning of sector 7
	void *pvSector7 = (void *)FLASH_SECTOR7_ADDR;

	// copy the data from flash to the buffer
	memcpy(dst, pvSector7, size);
}

// FreeRTOS CLI "clear" command
BaseType_t prvClearCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	// Copy new-page character to output
	if (xWriteBufferLen > 0)
		strncpy(pcWriteBuffer, (char[]){NEWPAGE_CHAR, '\0'}, xWriteBufferLen);

	return pdFALSE;
}

static int32_t vectorLength(int32_t X, int32_t Y) {

	int32_t length = sqrt((X * X) + (Y * Y));

	if (length > 100)
		return 100;
	else
		return length;
}

// FreeRTOS CLI "led" command
BaseType_t prvLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;
	TimerCallback_t *pLedTimerCallback = NULL;
	BaseType_t *pxLedStatus = NULL;

	// Get first parameter - LED on/off
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);

	// Get LED timer status from timer callback argument
	pLedTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(ledTimHandle);
	pxLedStatus = pLedTimerCallback->arg;

	// Interpret command
	if (strncmp(pcParameter1, "on", xParameter1StringLength) == 0)
	{
		// Start LED timer
		if (!xTimerIsTimerActive(ledTimHandle)) {
			strncpy(pcWriteBuffer, "[cli] Starting LED timer\r\n", xWriteBufferLen);
			if (pdFAIL == xTimerStart(ledTimHandle, 0))
				strncat(pcWriteBuffer, "[cli] Starting LED timer failed\r\n", xWriteBufferLen - strnlen(pcWriteBuffer, xWriteBufferLen));

			// Update LED status
			*pxLedStatus = pdTRUE;
		}
	}
	else if (strncmp(pcParameter1, "off", xParameter1StringLength) == 0)
	{
		// Stop LED timer
		if (xTimerIsTimerActive(ledTimHandle)) {
			strncpy(pcWriteBuffer, "[cli] Stopping LED timer\r\n", xWriteBufferLen);
			if (pdFAIL == xTimerStop(ledTimHandle, 0))
				strncat(pcWriteBuffer, "[cli] Stopping LED timer failed\r\n", xWriteBufferLen - strnlen(pcWriteBuffer, xWriteBufferLen));

			// Update LED status and disable PWM
			*pxLedStatus = pdFALSE;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		}
	}
	else
	{
		strncpy(pcWriteBuffer, "[cli] Invalid command\r\nsyntax: led [on|off]\r\n", xWriteBufferLen);
	}

	return pdFALSE;
}

// FreeRTOS CLI "blink" command
BaseType_t prvBlinkCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;
	BaseType_t xBlinkPeriod = 0;
	Data_t printValue = {0};
	TimerCallback_t *pLedTimerCallback = NULL;
	BaseType_t *pxLedStatus = NULL;

	// Get first parameter - LED blink period in ms
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	xBlinkPeriod = (uint32_t)atoi(pcParameter1);

	// Get LED timer status from timer callback argument
	pLedTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(ledTimHandle);
	pxLedStatus = pLedTimerCallback->arg;

	if (xBlinkPeriod > 0) {

		// Abort LED task delay
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Setting LED blinking timer period to %lu ms\r\n", xBlinkPeriod);

		// Set new timer period
		if (pdPASS == xTimerChangePeriod(ledTimHandle, pdMS_TO_TICKS(xBlinkPeriod), 0)) {
			*pxLedStatus = pdTRUE;
			printValue.type = BLINK_PERIOD;
			printValue.value.ui = xBlinkPeriod;
			xQueueSend(blePrintDataQueueHandle, &printValue, 0);
		} else {
			strncat(pcWriteBuffer, "[cli] Changing LED timer period failed\r\n", xWriteBufferLen - strnlen(pcWriteBuffer, xWriteBufferLen));
		}
	} else {
		strncat(pcWriteBuffer, "[cli] Invalid setting, timer period must be > 0\r\n", xWriteBufferLen - strnlen(pcWriteBuffer, xWriteBufferLen));
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

BaseType_t prvOledCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;

	// Read command parameter
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);

	if (0 == strncmp(pcParameter1, "on", xParameter1StringLength)) {

		//Resume task
		if (eSuspended == eTaskGetState(oledTaskHandle)) {
			strncpy(pcWriteBuffer, "[cli] Resuming OLED task\r\n", xWriteBufferLen);
			vTaskResume(oledTaskHandle);
		} else {
			strncpy(pcWriteBuffer, "[cli] OLED task is already running\r\n", xWriteBufferLen);
		}

		// Stop one-shot timer
		if (pdTRUE == xTimerIsTimerActive(oledTimHandle)) {
			if (xTimerStop(oledTimHandle, 0))
				strncat(pcWriteBuffer,  "[cli] OLED timeout disabled\r\n", xWriteBufferLen - strlen(pcWriteBuffer));
			else
				strncat(pcWriteBuffer,  "[cli] Failed to disable OLED timeout\r\n", xWriteBufferLen - strlen(pcWriteBuffer));
		}
	} else if (0 == strncmp(pcParameter1, "off", xParameter1StringLength)) {

		//Suspend task
		if (eSuspended != eTaskGetState(oledTaskHandle)) {
			strncpy(pcWriteBuffer, "[cli] Suspending OLED task\r\n", xWriteBufferLen);
			vTaskSuspend(oledTaskHandle);
			memset(screenBuffer, 0x0, sizeof(screenBuffer));
		} else {
			strncpy(pcWriteBuffer, "[cli] OLED task is already suspended\r\n", xWriteBufferLen);
		}

	} else {

		//Invalid command
		strncpy(pcWriteBuffer, "[cli] Invalid command\r\nsyntax: oled [on|off]\r\n", xWriteBufferLen);
	}

	return pdFALSE;
}

BaseType_t prvResetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	HAL_NVIC_SystemReset();
	return pdFALSE;
}

BaseType_t prvOledPeriodCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xStatus = pdFALSE;
	TickType_t xOledPeriod = {0};
	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;

	// Get first parameter - OLED timeout in seconds
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);

	// Convert to integer and set new timer period
	xOledPeriod = (TickType_t)atoi(pcParameter1);
	xStatus = xTimerChangePeriod(oledTimHandle, xOledPeriod, 0);

	if (pdPASS == xStatus)
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Setting OLED timeout to %lu ms\r\n", xOledPeriod);
	else
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Setting OLED timeout failed\r\n");

	return pdFALSE;
}

BaseType_t prvSaveCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xStatus = pdFALSE;
	xStatus = xTaskNotify(saveLoadTaskHandle, SAVE_TO_FLASH, eSetBits);

	if (pdPASS == xStatus)
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Sending 'save' notification to saveLoadTask\r\n");
	else
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Failed sending 'save' notification to saveLoadTask\r\n");

	return pdFALSE;
}

BaseType_t prvLoadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xStatus = pdFALSE;
	xStatus = xTaskNotify(saveLoadTaskHandle, LOAD_FROM_FLASH, eSetBits);

	if (pdPASS == xStatus)
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Sending 'load' notification to saveLoadTask\r\n");
	else
		snprintf(pcWriteBuffer, xWriteBufferLen, "[cli] Failed sending 'load' notification to saveLoadTask\r\n");

	return pdFALSE;
}

BaseType_t prvPanelCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	vTaskResume(blePanelTaskHandle);
	strncpy(pcWriteBuffer, "[cli] Resuming blePanelTask\r\n", xWriteBufferLen);
	return pdFALSE;
}

BaseType_t prvJoyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	BaseType_t xParameter1StringLength = 0;
	const char *pcParameter1 = NULL;
	char cXStringValue[4] = {0};
	char cYStringValue[4] = {0};
	char *pcValue = &cXStringValue[0];
	Data_t joyValue = {0};

	// Get jostick X and Y positions
	if (eSuspended != eTaskGetState(motorsTaskHandle)) {
		pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
		for (size_t i = 1; i < xParameter1StringLength; i++) {
			if (pcParameter1[i] == 'Y') {
				pcValue = &cYStringValue[0];
				continue;
			}
			*pcValue++ = pcParameter1[i];
		}

		// Save to data structure and send to motors task
		joyValue.type = JOYSTICK;
		joyValue.value.joy[0] = atoi(cXStringValue) - 100;
		joyValue.value.joy[1] = -(atoi(cYStringValue) - 100);
		xQueueSend(joyDataQueueHandle, &joyValue, 0);
	}

	return pdFALSE;
}

BaseType_t prvListCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	static BaseType_t showHeader = pdTRUE;

	if (showHeader == pdTRUE) {
		strncpy(pcWriteBuffer,
						"Name          State  Priority  Stack   Num\r\n"
						"*********************************************\n\r",
						xWriteBufferLen);
		showHeader = pdFALSE;
		return pdTRUE;
	}
	else
	{
		showHeader = pdTRUE;
		vTaskList(pcWriteBuffer);
		return pdFALSE;
	}
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
	BaseType_t xNotification = 0;
	uint8_t cInputBufferIndex = 0;
	char cInputBuffer[INPUT_BUFFER_LEN] = {0};
	char cOutputBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cRxChar = 0;
	char cTxChar = NEWPAGE_CHAR;
	const char *cWelcomeMessage = "FreeRTOS Command Line Interface. \r\ntype \"help\" to view a list of commands.\r\n";
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;

	// Take Rx semaphore and initiate DMA Rx transfer
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)&cRxChar, 1);

	// Send new-page character and welcome message to cliPrintTask
	xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, &cTxChar, 1, pdMS_TO_TICKS(SHORT_DELAY));
	xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, cWelcomeMessage, strlen(cWelcomeMessage), pdMS_TO_TICKS(SHORT_DELAY));

	for(;;)
  {
		// Wait for new character
		xNotification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SHORT_DELAY));
		if (xNotification != 0)
		{
			// Copy received character and initiate next DMA transfer
			cTxChar = cRxChar;
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)&cRxChar, 1);

			// Check input
			if (cTxChar == '\r' || cTxChar == '\n')
			{
				// Newline received, echo carriage return and newline
				xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "\r\n", strlen("\r\n"), pdMS_TO_TICKS(STD_DELAY));

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
							xSendStringByStreamBuffer(xStreamBufferString,
																				printStringMutexHandle,
																				cOutputBuffer,
																				strnlen(cOutputBuffer, OUTPUT_BUFFER_LEN),
																				pdMS_TO_TICKS(STD_DELAY));
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
				xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, &cTxChar, 1, pdMS_TO_TICKS(STD_DELAY));
			}
			else if (cInputBufferIndex < INPUT_BUFFER_LEN)
			{
				// Save new character and echo to terminal
				cInputBuffer[cInputBufferIndex++] = cTxChar;
				xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, &cTxChar, 1, pdMS_TO_TICKS(STD_DELAY));
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

/* USER CODE BEGIN Header_streamPrintTaskFunction */
/**
* @brief Function implementing the streamPrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_streamPrintTaskFunction */
void streamPrintTaskFunction(void *argument)
{
  /* USER CODE BEGIN streamPrintTaskFunction */

	BaseType_t xStatus = pdFALSE;
	size_t xReceiveSize = 0;
	char cReceiveBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cSendBuffer[OUTPUT_BUFFER_LEN] = {0};
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;
	HAL_StatusTypeDef halStatus = HAL_OK;

	for(;;)
	{
		// Read stream buffer
		xReceiveSize = xStreamBufferReceive(xStreamBufferString, cReceiveBuffer, OUTPUT_BUFFER_LEN, portMAX_DELAY);

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

  /* USER CODE END streamPrintTaskFunction */
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

	// Task synchronization bits
	EventBits_t uxThisTasksSyncBits = CPU_TEMP_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 EVENT_HANDLER_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 OLED_INIT_EVENT |
																 DIST_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	// Text buffer and string data
	char cText[OUTPUT_BUFFER_LEN] = {0};
	Data_t printData = {0};

	// Synchronize task initialization
	xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

  for(;;)
  {
    vTaskSuspend(NULL);
    xStatus = xSemaphoreTake(adc1SemHandle, pdMS_TO_TICKS(STD_DELAY));

    if (xStatus == pdTRUE) {

    	// Start ADC readings and wait for notification from ISR
    	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) sAdc1Value, 2);
    	xNotification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SHORT_DELAY));

    	if (xNotification != 0) {

    		// Calculate temperature
    		fVdda = (VDD * (*psVddRefCalVal)) / *psVRefRead;
				fVTempSens = (*psTempSenRead * fVdda) / MAX_ADC_VAL_12_BITS;
				fCpuTemp = ((fVAtTemp30 - fVTempSens) / fVTempSenSlope) + TEMP_30;

				// Send to terminal
				snprintf(cText, sizeof(cText), "[cpuTempTask] CPU temperature: %.2fC\r\n", fCpuTemp);
				xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, cText, strnlen(cText, sizeof(cText)), pdMS_TO_TICKS(STD_DELAY));

				// Send to OLED screen
				printData.type = CPUTEMP;
				printData.value.f = fCpuTemp;
				if (eSuspended != eTaskGetState(oledTaskHandle))
					xQueueSend(oledDataQueueHandle, &printData, pdMS_TO_TICKS(SHORT_DELAY));

				// Send to BT task
				xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(SHORT_DELAY));
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
  Data_t printData = {0};

  // Accelerometer readings
	uint16_t sAccel[3] = {0};
	uint16_t sGyro[3] = {0};
	int16_t sAxisXRaw = 0;
	int16_t sAxisYRaw = 0;
	int16_t sAxisZRaw = 0;
	uint8_t sampleNum = 0;
	bool bDataReady = false;
	float fAxisX = 0.0;
	float fAxisY = 0.0;
	float fAxisZ = 0.0;
	float fPitch = 0.0;
	float fRoll = 0.0;
	float fPitchWin[ACCELGYRO_WIN_SIZE] = {0.0};
	float fRollWin[ACCELGYRO_WIN_SIZE] = {0.0};

	// Task synchronization bits
	EventBits_t uxThisTasksSyncBits = ACCEL_GYRO_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 EVENT_HANDLER_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT |
																 OLED_INIT_EVENT |
																 DIST_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	// Synchronize task initialization
	xEventGroupWaitBits(commonEventHandle, OLED_INIT_EVENT, pdFALSE, pdTRUE, pdMS_TO_TICKS(STD_DELAY));

	// Initialize accelerometer
	xStatus = xAccelGyroInit();
	if (pdFAIL == xStatus) {
		xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[accelGyroTask] accelGyro init failed\r\n", 0, pdMS_TO_TICKS(STD_DELAY));
	} else {
		xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));
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
  	fPitchWin[sampleNum] = atan2(fAxisY, fAxisZ) * 57.3;
  	fRollWin[sampleNum] = atan2((-fAxisX), sqrt(fAxisY * fAxisY + fAxisZ * fAxisZ)) * 57.3;
  	sampleNum = (sampleNum + 1) % ACCELGYRO_WIN_SIZE;
  	if (!bDataReady && (sampleNum == 0))
  		bDataReady = true;

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

  	if (bDataReady) {

  		// Sum readings
  	  gfPitch = fPitch = 0;
  		gfRoll = fRoll = 0;
  		for (uint8_t i = 0; i < ACCELGYRO_WIN_SIZE; i++) {
  			fPitch += fPitchWin[i];
  			fRoll += fRollWin[i];
  		}

  		// Calculate average
  		gfPitch = fPitch = fPitch / ACCELGYRO_WIN_SIZE;
  		gfRoll = fRoll = fRoll / ACCELGYRO_WIN_SIZE;

  		// Send data to other tasks
    	printData.type = PITCH;
    	printData.value.f = fPitch;
    	xQueueSend(oledDataQueueHandle, &printData, pdMS_TO_TICKS(STD_DELAY));
    	printData.type = ROLL;
    	printData.value.f = fRoll;
    	xQueueSend(oledDataQueueHandle, &printData, pdMS_TO_TICKS(STD_DELAY));
    	printData.type = PITCHANDROLL;
    	printData.value.pr[0] = fRoll;
    	printData.value.pr[1] = fPitch;
    	xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(STD_DELAY));
    	xQueueSend(tiltDataQueueHandle, &printData, pdMS_TO_TICKS(STD_DELAY));
  	}

  	// Delay next sample
  	vTaskDelay(pdMS_TO_TICKS(SHORT_DELAY));
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
	char cText[OUTPUT_BUFFER_LEN] = {0};
	StringData_t printString = {0};
	uint8_t cValue = 0;
	EventBits_t xBitsToWaitFor = (BUTTON_EVENT | MOTION_INT_EVENT | TILT_ALERT_EVENT | TILT_OK_EVENT);
	EventBits_t xEventGroupValue = 0;
	Data_t printData = {0};

	// Task synchronization bits
	EventBits_t uxThisTasksSyncBits = EVENT_HANDLER_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 DIST_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 OLED_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	// OLED timer callback and number of timeout events
	TimerCallback_t *pOledTimerCallback = NULL;
	uint32_t *pxOledTimerExecuted = NULL;

	// LED timer callback and status
	TimerCallback_t *pLedTimerCallback = NULL;
	BaseType_t *pxLedStatus = NULL;

	// Get LED status
	pLedTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(ledTimHandle);
	pxLedStatus = pLedTimerCallback->arg;

	// Initialize string data
	vInitPrintStringData(&printString, cText, sizeof(cText), eventPrintSemHandle);

	// Synchronize task initialization
	xEventGroupValue = xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

	if (!(xEventGroupValue & CPU_TEMP_INIT_EVENT))
		xSendStringByQueue(&printString, printStringQueueHandle, "[%s] cpuTempTask not ready!\r\n", pcTaskGetName(NULL));
	if (!(xEventGroupValue & DIST_INIT_EVENT))
		xSendStringByQueue(&printString, printStringQueueHandle, "[%s] distTask not ready!\r\n", pcTaskGetName(NULL));
	if (!(xEventGroupValue & ACCEL_GYRO_INIT_EVENT))
		xSendStringByQueue(&printString, printStringQueueHandle, "[%s] accelGyroTask not ready!\r\n", pcTaskGetName(NULL));
	if (!(xEventGroupValue & OLED_INIT_EVENT))
		xSendStringByQueue(&printString, printStringQueueHandle, "[%s] oledTask not ready!\r\n", pcTaskGetName(NULL));

  for(;;)
  {
    xEventGroupValue = xEventGroupWaitBits(commonEventHandle, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);

    // Motion event
    if (xEventGroupValue & MOTION_INT_EVENT) {

    	// Read interrupt status register
    	xAccelGyroMemRead(ACCELGYRO_INT_STATUS_ADDR, 1, &cValue, 1);

    	// Check if motion interrupt is set
    	if (cValue & ACCELGYRO_INT_STATUS_MOTION) {
    		xSendStringByQueue(&printString, printStringQueueHandle, "[eventTask] Motion interrupt!\r\n");

    		// Read motion status register and print motion information to terminal
    		xAccelGyroMemRead(ACCELGYRO_MOTION_STATUS_ADDR, 1, &cValue, 1);
    		xSendStringByQueue(&printString,
    											 printStringQueueHandle,
													 "[eventTask] Motion status: -X:%d +X:%d -Y:%d +Y:%d -Z:%d +Z:%d\r\n",
													 (cValue & ACCELGYRO_MOTION_STATUS_X_NEG) ? 1 : 0,
													 (cValue & ACCELGYRO_MOTION_STATUS_X_POS) ? 1 : 0,
													 (cValue & ACCELGYRO_MOTION_STATUS_Y_NEG) ? 1 : 0,
													 (cValue & ACCELGYRO_MOTION_STATUS_Y_POS) ? 1 : 0,
													 (cValue & ACCELGYRO_MOTION_STATUS_Z_NEG) ? 1 : 0,
													 (cValue & ACCELGYRO_MOTION_STATUS_Z_POS) ? 1 : 0);

				// Send to OLED screen
				printData.type = MOTION;
				printData.value.uc = cValue;
				xQueueSend(oledDataQueueHandle, &printData, pdMS_TO_TICKS(SHORT_DELAY));

				// Add delay
				vTaskDelay(pdMS_TO_TICKS(SHORT_DELAY));
    	}
    }

    // Button event
    if (xEventGroupValue & BUTTON_EVENT) {

    	// Resume OLED task if suspended
  		if (eSuspended == eTaskGetState(oledTaskHandle)) {
  			vTaskResume(oledTaskHandle);
  			xSendStringByQueue(&printString, printStringQueueHandle, "[eventTask] Resuming OLED task\r\n");
  		}

  		// Restart OLED timer
  		if (xTimerStart(oledTimHandle, pdMS_TO_TICKS(100)))
  			xSendStringByQueue(&printString, printStringQueueHandle, \
  					"[eventTask] Enabled OLED screen timeout, period: %lu ms\r\n", xTimerGetPeriod(oledTimHandle));
  		else
  			xSendStringByQueue(&printString, printStringQueueHandle, \
  					"[eventTask] Unable to set OLED screen timeout\r\n");

  		// Get OLED timer callback argument, containing number of timeout events
			pOledTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(oledTimHandle);
			pxOledTimerExecuted = (uint32_t *)pOledTimerCallback->arg;
			xSendStringByQueue(&printString, printStringQueueHandle, "[eventTask] Number of OLED timeouts: %lu\r\n", *pxOledTimerExecuted);
    }

    if (xEventGroupValue & TILT_ALERT_EVENT) {

    	// Suspend motors task, reset data queue and turn off all PWM channels
    	vTaskSuspend(motorsTaskHandle);
    	xQueueReset(joyDataQueueHandle);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    	// Send data to android app
    	printData.type = MOTORS;
    	printData.value.motors[0] = 100;
    	printData.value.motors[1] = 100;
    	xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(SHORT_DELAY));
    	printData.type = TILT;
    	printData.value.ui = TILT_ALERT_EVENT;
    	xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(SHORT_DELAY));

      // Restore LED blinking
      if (*pxLedStatus == pdTRUE && xTimerIsTimerActive(ledTimHandle) == pdFALSE)
      	xTimerStart(ledTimHandle, pdMS_TO_TICKS(SHORT_DELAY));

      xSendStringByQueue(&printString, printStringQueueHandle, "[eventTask] Tilt alert - suspending motors task\r\n");
    }

    if (xEventGroupValue & TILT_OK_EVENT) {
    	vTaskResume(motorsTaskHandle);
    	printData.type = TILT;
    	printData.value.ui = TILT_OK_EVENT;
    	xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(SHORT_DELAY));
    	xSendStringByQueue(&printString, printStringQueueHandle, "[eventTask] Tilt ok - resuming motors task\r\n");
    }
  }
  /* USER CODE END eventTaskFunction */
}

/* USER CODE BEGIN Header_oledTaskFunction */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oledTaskFunction */
void oledTaskFunction(void *argument)
{
  /* USER CODE BEGIN oledTaskFunction */
	BaseType_t xStatus = pdFALSE;
	char cText[TEXT_ARRAY] = {0};
	Data_t printData = {0};

	// Task synchronization bits
	EventBits_t uxThisTasksSyncBits = OLED_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 EVENT_HANDLER_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 DIST_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	// Timer callback and counter of timeout events
	TimerCallback_t* pOledTimerCallback = NULL;
	uint32_t xOledTimerExecuted = 0;

	// Initialize screen and start continuous DMA transfer
	ssd1331_init();
	HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)screenBuffer, sizeof(screenBuffer));

	// Synchronize task initialization
	xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

	// Cycle screen with different colors
  ssd1331_clear_screen(BLACK);
  vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
  ssd1331_clear_screen(WHITE);
	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
  ssd1331_clear_screen(RED);
	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
  ssd1331_clear_screen(BLUE);
	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
  ssd1331_clear_screen(GREEN);
	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
  ssd1331_clear_screen(BLACK);
	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));

	// Display Mobica logo
	memcpy(screenBuffer, mobicaLogo, mobicaLogoSize);
	vTaskDelay(3000);

	// Clear screen
  ssd1331_clear_screen(BLACK);
	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));

	// Get OLED timer callback and set callback argument
	pOledTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(oledTimHandle);
	pOledTimerCallback->arg = &xOledTimerExecuted;

	// Set default timer period to 10s
	xStatus = xTimerChangePeriod(oledTimHandle, pdMS_TO_TICKS(10000), pdMS_TO_TICKS(100));
	xTimerStop(oledTimHandle, 0);
	if (pdFAIL == xStatus)
		xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[oledTask] Unable to set OLED screen timeout\r\n", 0, pdMS_TO_TICKS(STD_DELAY));

	/* Infinite loop */
  for(;;)
  {
  	xStatus = xQueueReceive(oledDataQueueHandle, &printData, portMAX_DELAY);
  	if (pdTRUE == xStatus) {
  		switch (printData.type) {

  			case PITCH:
  				// Clear 12 rows starting from row 0 and print pitch value at 0,0
  				snprintf(cText, sizeof(cText), "Pitch: %.1f", printData.value.f);
  				memset(&screenBuffer[0], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 0, (uint8_t *)cText, FONT_1206, RED);
  				break;

  			case ROLL:
  				// Clear 12 rows starting from row 13 and print roll value at 0,13
  				snprintf(cText, sizeof(cText), "Roll: %.1f", printData.value.f);
  				memset(&screenBuffer[13], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 13, (uint8_t *)cText, FONT_1206, BLUE);
  				break;

  			case DISTANCE:
  				// Clear 12 rows starting from row 26 and distance sensor reading at 0,26
  				snprintf(cText, sizeof(cText), "Distance: %lucm", printData.value.ui);
  				memset(&screenBuffer[26], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 26, (uint8_t *)cText, FONT_1206, GREEN);
  				break;

  			case TEMPERATURE:
  				// Clear 12 rows starting from row 39 and print temperature at 0,39
  				snprintf(cText, sizeof(cText), "Temp: %.1f", printData.value.f);
  				memset(&screenBuffer[39], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 39, (uint8_t *)cText, FONT_1206, YELLOW);
  				break;

  			case HUMIDITY:
  				// Clear 12 rows starting from row 52 and print humidity at 0,52
  				snprintf(cText, sizeof(cText), "Humidity: %.1f", printData.value.f);
  				memset(&screenBuffer[52], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 52, (uint8_t *)cText, FONT_1206, PURPLE);
  				break;

  			case BLINK_PERIOD:
  				// Clear 12 rows starting from row 26 and (temporarily) print blink rate at 0,26
  				snprintf(cText, sizeof(cText), "Blink: %lu", printData.value.ui);
  				memset(&screenBuffer[26], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 26, (uint8_t *)cText, FONT_1206, GREEN);
  				vTaskDelay(pdMS_TO_TICKS(2000));
  				break;

  			case CPUTEMP:
  				// Clear 12 rows starting from row 39 and (temporarily) print CPU temperature at 0,39
  				snprintf(cText, sizeof(cText), "CPU Temp: %.1f", printData.value.f);
  				memset(&screenBuffer[39], 0x0, OLED_SCREEN_WIDTH * 12 * sizeof(uint16_t));
  				ssd1331_display_string(0, 39, (uint8_t *)cText, FONT_1206, YELLOW);
  				vTaskDelay(pdMS_TO_TICKS(2000));
  				break;

  			case MOTION:

  				// Motion detected - flash screen
  				for (uint8_t i = 0; i < 3; i++) {
    			  ssd1331_clear_screen(RED);
    				vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
    				ssd1331_clear_screen(BLACK);
    				vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
  				}

  				//Print motion status
  				snprintf(cText, sizeof(cText),
  								 "Motion detected:-X:%d +X:%d -Y:%d  +Y:%d -Z:%d +Z:%d",
  								 (printData.value.uc & ACCELGYRO_MOTION_STATUS_X_NEG) ? 1 : 0,
  								 (printData.value.uc & ACCELGYRO_MOTION_STATUS_X_POS) ? 1 : 0,
  								 (printData.value.uc & ACCELGYRO_MOTION_STATUS_Y_NEG) ? 1 : 0,
  								 (printData.value.uc & ACCELGYRO_MOTION_STATUS_Y_POS) ? 1 : 0,
  								 (printData.value.uc & ACCELGYRO_MOTION_STATUS_Z_NEG) ? 1 : 0,
  								 (printData.value.uc & ACCELGYRO_MOTION_STATUS_Z_POS) ? 1 : 0);
  				ssd1331_display_string(0, 13, (uint8_t *)cText, FONT_1206, RED);

  				// Add delay and clear screen
  			  vTaskDelay(pdMS_TO_TICKS(3000));
  				ssd1331_clear_screen(BLACK);
  				break;

  			case PITCHANDROLL:
  				break;
  			case JOYSTICK:
  				break;
  			case MOTORS:
  				break;
  			case TILT:
  				break;
  		}
  	}
  }
  /* USER CODE END oledTaskFunction */
}

/* USER CODE BEGIN Header_distTaskFunction */
/**
* @brief Function implementing the distTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_distTaskFunction */
void distTaskFunction(void *argument)
{
  /* USER CODE BEGIN distTaskFunction */

	BaseType_t xNotification = 0;
	uint32_t uiTimerTicksToUs = (SystemCoreClock / 2) / 1000000;
	uint32_t xEchoRead[2] = {0};
	uint32_t uiDistance = 0;
	Data_t printValue = {0};

	// Task synchronization bits
	EventBits_t uxThisTasksSyncBits = DIST_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 EVENT_HANDLER_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 OLED_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	// Start circular DMA transfer for input capture timer
	HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_1, xEchoRead, 2);

	// Synchronize task initialization
	xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

	/* Infinite loop */
	for(;;)
	{
		// Wait for sensor echo
		xNotification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STD_DELAY));

		if (0 != xNotification) {

			// Calculate distance in cm
			uiDistance = (uint32_t) ((xEchoRead[1] - xEchoRead[0]) / uiTimerTicksToUs) / TIME_TO_DIST_CM;

			// Send to OLED screen
			printValue.type = DISTANCE;
			printValue.value.ui = uiDistance;

			if (eSuspended != eTaskGetState(oledTaskHandle))
				xQueueSend(oledDataQueueHandle, &printValue, pdMS_TO_TICKS(SHORT_DELAY));

			// Send to BT task
			xQueueSend(blePrintDataQueueHandle, &printValue, pdMS_TO_TICKS(SHORT_DELAY));

			// Set echo received event
			xEventGroupSetBits(commonEventHandle, DIST_ECHO_EVENT);
		}

		vTaskDelay(pdMS_TO_TICKS(SHORT_DELAY));
	}

  /* USER CODE END distTaskFunction */
}

/* USER CODE BEGIN Header_distTriggerFunction */
/**
* @brief Function implementing the distTriggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_distTriggerFunction */
void distTriggerFunction(void *argument)
{
  /* USER CODE BEGIN distTriggerFunction */

	uint32_t uiTimerTicksToUs = (SystemCoreClock / 2) / 1000000;
	uint32_t xTriggerTicks = uiTimerTicksToUs * 20;

	// Task synchronization bits
	EventBits_t xEventGroupValue = 0;
	EventBits_t uxThisTasksSyncBits = DIST_TRIGGER_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 EVENT_HANDLER_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 OLED_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT |
																 DIST_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	// Setup trigger timer (one pulse PWM)
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim4, xTriggerTicks + 1);

	// Synchronize task initialization
	xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

  /* Infinite loop */
  for(;;)
  {
		// Send 20us trigger pulse
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

		// Wait for echo
		xEventGroupValue = xEventGroupWaitBits(commonEventHandle, DIST_ECHO_EVENT, pdTRUE, pdTRUE, pdMS_TO_TICKS(STD_DELAY));

		//Stop timer and add delay between trigger pulses
		if (xEventGroupValue & DIST_ECHO_EVENT) {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			vTaskDelay(pdMS_TO_TICKS(SHORT_DELAY));
		}
  }
  /* USER CODE END distTriggerFunction */
}

/* USER CODE BEGIN Header_tempHumTaskFunction */
/**
* @brief Function implementing the tempHumTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tempHumTaskFunction */
void tempHumTaskFunction(void *argument)
{
  /* USER CODE BEGIN tempHumTaskFunction */

	// Status variables
	BaseType_t xStatus = pdFALSE;
	uint32_t xNotification = 0;

	// Data printing and display
	Data_t printData = {0};

	// Task priority
	UBaseType_t uxTaskPriority = uxTaskPriorityGet(NULL);

	// GPIO reconfiguration structure
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	// Data capture
	uint32_t xCounterValueForBit = 0;
	uint16_t sInputRead[42] = {0};
	uint8_t cSensorData[5] = {0};

	// Results
	uint8_t cChecksum = 0;
	uint16_t sRawTemperature = 0;
	uint16_t sRawHumidity = 0;
	float fTemperature = 0.0f;
	float fHumidity = 0.0f;

	// GPIO reconfiguration settings
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
	GPIO_InitStructure.Pin = TEMP_HUM_SENS_Pin;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;

	// 100us timer counter value
	xCounterValueForBit = ((SystemCoreClock / 1000000) / TIM1->PSC) * 100;

	xStatus = xTimerChangePeriod(tempHumTimHandle, pdMS_TO_TICKS(500), pdMS_TO_TICKS(100));
	if (pdFAIL == xStatus)
		xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[tempHumTask] Timer change period failed\r\n", 0, pdMS_TO_TICKS(STD_DELAY));

  for(;;)
  {
  	// Suspend task and wait for periodic timer
  	vTaskSuspend(NULL);

  	// Set pin to standard output mode
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  	HAL_GPIO_Init(TEMP_HUM_SENS_GPIO_Port, &GPIO_InitStructure);

  	// Increase task priority to ensure that all incoming data is captured
  	vTaskPrioritySet(NULL, uxTaskPriority + 1);

  	// Trigger sensor with 1ms low pulse
  	HAL_GPIO_WritePin(TEMP_HUM_SENS_GPIO_Port, TEMP_HUM_SENS_Pin, GPIO_PIN_RESET);
  	vTaskDelay(pdMS_TO_TICKS(1));
  	HAL_GPIO_WritePin(TEMP_HUM_SENS_GPIO_Port, TEMP_HUM_SENS_Pin, GPIO_PIN_SET);

  	// Set GPIO to input capture mode
  	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
  	HAL_GPIO_Init(TEMP_HUM_SENS_GPIO_Port, &GPIO_InitStructure);

  	// Reset timer to 0
  	__HAL_TIM_SetCounter(&htim1, 0);

  	// Capture time between 42 falling edges
  	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)sInputRead, 42);

  	// Reset to original task priority
  	vTaskPrioritySet(NULL, uxTaskPriority);

  	// Wait for all data
  	xStatus = xTaskNotifyWait(0xffffffff, 0xffffffff, &xNotification, pdMS_TO_TICKS(6));

  	if (pdTRUE == xStatus && (xNotification == TEMP_HUM_IC_CPLT)) {

  		// Save incoming data (MSB first) and check timer readings
  		for (int i = 2, j = 0; i < 42; i++, j++) {
  			cSensorData[j / 8] <<= 1;
  			cSensorData[j / 8] |= ((sInputRead[i] - sInputRead[i - 1]) > xCounterValueForBit) ? 1 : 0;
  		}

  		// Received checksum shoud be the summation of the first 4 bytes
  		cChecksum = (cSensorData[0] + cSensorData[1] + cSensorData[2] + cSensorData[3]);
  		if (cChecksum != cSensorData[4])
  			xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[tempHumTask] Wrong checksum!\r\n", 0, pdMS_TO_TICKS(STD_DELAY));

  		// Calculate temperature and humidity
  		sRawHumidity = (uint16_t)(cSensorData[0] << 8) | cSensorData[1];
  		sRawTemperature = (uint16_t)(cSensorData[2] << 8) | cSensorData[3];
  		fHumidity = sRawHumidity / 10.0;
  		fTemperature = sRawTemperature / 10.0;

  		// Send data to OLED and BT tasks
  		printData.type = TEMPERATURE;
  		printData.value.f = fTemperature;
  		if (eSuspended != eTaskGetState(oledTaskHandle))
  			xQueueSend(oledDataQueueHandle, &printData, pdMS_TO_TICKS(50));
  		xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(50));

  		printData.type = HUMIDITY;
  		printData.value.f = fHumidity;
  		if (eSuspended != eTaskGetState(oledTaskHandle))
  			xQueueSend(oledDataQueueHandle, &printData, pdMS_TO_TICKS(50));
  		xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(50));
  	}

  	// Stop input capture
  	HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);
  }
  /* USER CODE END tempHumTaskFunction */
}

/* USER CODE BEGIN Header_saveLoadTaskFunction */
/**
* @brief Function implementing the saveLoadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_saveLoadTaskFunction */
void saveLoadTaskFunction(void *argument)
{
  /* USER CODE BEGIN saveLoadTaskFunction */

	uint32_t xNotification = 0;
	BaseType_t xStatus = pdFAIL;
	Config_t config = {0};
	Data_t printData = {0};
	EventBits_t uxThisTasksSyncBits = FLASH_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (EVENT_HANDLER_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 OLED_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT |
																 DIST_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 MOTORS_INIT_EVENT);

	BaseType_t xLedStatus = pdFALSE;
	TimerCallback_t *pLedTimerCallback = NULL;

	// Get LED timer status from timer callback argument
	pLedTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(ledTimHandle);
	pLedTimerCallback->arg = &xLedStatus;

	// Synchronize task initialization
	xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

	for(;;)
  {
		xStatus = xTaskNotifyWait(0, 0xffffffff, &xNotification, pdMS_TO_TICKS(500));

		// Save configuration to flash memory
		if (pdTRUE == xStatus && xNotification == SAVE_TO_FLASH) {
			config.ulLedTimPeriod = xTimerGetPeriod(ledTimHandle);
			config.ulOledTimPeriod = xTimerGetPeriod(oledTimHandle);
			vSaveToFlash((uint8_t *)&config, sizeof(config));
			xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[saveLoadTask] Config saved to flash\r\n", 0, pdMS_TO_TICKS(STD_DELAY));
		}

		// Load configuration from flash memory
		if (pdTRUE == xStatus && xNotification == LOAD_FROM_FLASH) {

			// Load current configuration from flash
			vLoadFromFlash((uint8_t *)&config, sizeof(config));

			// Update LED timer period
			if (pdFAIL == xTimerChangePeriod(ledTimHandle, pdMS_TO_TICKS(config.ulLedTimPeriod), pdMS_TO_TICKS(100)))
				xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[saveLoadTask] Changing LED timer period failed\r\n", 0, pdMS_TO_TICKS(STD_DELAY));
			else {
				xLedStatus = pdTRUE;
	  		printData.type = BLINK_PERIOD;
	  		printData.value.ui = config.ulLedTimPeriod;
	  		xQueueSend(blePrintDataQueueHandle, &printData, pdMS_TO_TICKS(50));
			}

			// Update OLED timer period
			if (pdFAIL == xTimerChangePeriod(ledTimHandle, pdMS_TO_TICKS(config.ulOledTimPeriod), pdMS_TO_TICKS(100)))
					xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[saveLoadTask] Changing OLED timer period failed\r\n", 0, pdMS_TO_TICKS(STD_DELAY));

			// Reset OLED timer
			if (pdFAIL == xTimerReset(oledTimHandle, pdMS_TO_TICKS(100)))
				xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[saveLoadTask] OLED timer reset failed\r\n", 0, pdMS_TO_TICKS(STD_DELAY));

			xSendStringByStreamBuffer(xStreamBufferString, printStringMutexHandle, "[saveLoadTask] Config loaded from flash\r\n", 0, pdMS_TO_TICKS(STD_DELAY));
		}
  }
  /* USER CODE END saveLoadTaskFunction */
}

/* USER CODE BEGIN Header_bleCliTaskFunction */
/**
* @brief Function implementing the bleCliTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bleCliTaskFunction */
void bleCliTaskFunction(void *argument)
{
  /* USER CODE BEGIN bleCliTaskFunction */

	// Task variables
	BaseType_t xStatus = pdFALSE;
	BaseType_t xNotification = 0;
	uint8_t cInputBufferIndex = 0;
	char cInputBuffer[INPUT_BUFFER_LEN] = {0};
	char cOutputBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cRxChar = 0;
	char cTxChar = 0;
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;

	// Initiate DMA Rx transfer
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)&cRxChar, 1);

	for(;;)
	{
		xNotification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SHORT_DELAY));
		if (xNotification != 0)
		{
			cTxChar = cRxChar;
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)&cRxChar, 1);

			if ('#' == cTxChar)
			{
				if (cInputBufferIndex > 0)
				{
					do {
						xStatus = FreeRTOS_CLIProcessCommand(cInputBuffer, cOutputBuffer, OUTPUT_BUFFER_LEN);
					} while (xStatus != pdFALSE);
					memset(cInputBuffer, '\0', INPUT_BUFFER_LEN);
					cInputBufferIndex = 0;
				}
			}
			else {
				if ('*' == cTxChar) {
					cInputBufferIndex = 0;
					memset(cInputBuffer, '\0', INPUT_BUFFER_LEN);
				} else {
					if (cInputBufferIndex < INPUT_BUFFER_LEN)
						cInputBuffer[cInputBufferIndex++] = cTxChar;
				}
			}
		}
		else
		{
			uartState = HAL_UART_GetState(&huart1);
			if (HAL_UART_STATE_ERROR == uartState)
				Error_Handler();
		}
	}

  /* USER CODE END bleCliTaskFunction */
}

/* USER CODE BEGIN Header_blePrintTaskFunction */
/**
* @brief Function implementing the blePrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blePrintTaskFunction */
void blePrintTaskFunction(void *argument)
{
  /* USER CODE BEGIN blePrintTaskFunction */

	// Status variables
	BaseType_t xStatus = pdFALSE;
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;
	HAL_StatusTypeDef halStatus = HAL_OK;

	// Local buffers and data container
	size_t xReceiveSize = 0;
	char cSendBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cReceiveBuffer[OUTPUT_BUFFER_LEN] = {0};
	Data_t printData = {0};

  for(;;)
  {
		xStatus = xQueueReceive(blePrintDataQueueHandle, &printData, portMAX_DELAY);
		if (pdTRUE == xStatus) {
			memset(cReceiveBuffer, '\0', OUTPUT_BUFFER_LEN);

			switch (printData.type) {
				case BLINK_PERIOD:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*B%lims*", printData.value.ui);
					break;
				case CPUTEMP:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*C%.2fC*", printData.value.f);
					break;
				case DISTANCE:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*D%limm*", printData.value.ui);
					break;
				case TEMPERATURE:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*T%.1fC*", printData.value.f);
					break;
				case HUMIDITY:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*H%.1f%%*", printData.value.f);
					break;
				case PITCHANDROLL:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*PX%.1fY%.1f*", printData.value.pr[0], printData.value.pr[1]);
					break;
				case MOTORS:
					xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*L%lu**R%lu*", printData.value.motors[0], printData.value.motors[1]);
					break;
				case TILT:
					if (printData.value.ui == TILT_ALERT_EVENT)
						xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*IR255G0B0*");
					else if (printData.value.ui == TILT_OK_EVENT)
						xReceiveSize = snprintf(cReceiveBuffer, OUTPUT_BUFFER_LEN, "*IR0G255B0*");
					break;
				default:
					xStatus = pdFALSE;
			}
		}

		// Transmit data to BT transmitter
		if (pdTRUE == xStatus)
		{
			do {
				xStatus = xSemaphoreTake(uart1TxSemHandle, pdMS_TO_TICKS(STD_DELAY));

				if (xStatus == pdTRUE)
				{
					memset(cSendBuffer, '\0', OUTPUT_BUFFER_LEN);
					memcpy(cSendBuffer, cReceiveBuffer, xReceiveSize);
					halStatus = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cSendBuffer, (uint16_t)xReceiveSize);
					if (halStatus != HAL_OK)
						xSemaphoreGive(uart1TxSemHandle);
				}
				else
				{
					// Check UART for errors
					uartState = HAL_UART_GetState(&huart1);
					if (HAL_UART_STATE_ERROR == uartState)
						Error_Handler();
				}
			} while (xStatus != pdTRUE);
		}
  }

  /* USER CODE END blePrintTaskFunction */
}

/* USER CODE BEGIN Header_bleStrPrintTaskFunction */
/**
* @brief Function implementing the bleStrPrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bleStrPrintTaskFunction */
void bleStrPrintTaskFunction(void *argument)
{
  /* USER CODE BEGIN bleStrPrintTaskFunction */

	BaseType_t xStatus = pdFALSE;
	size_t xReceiveSize = 0;
	char cReceiveBuffer[OUTPUT_BUFFER_LEN] = {0};
	char cSendBuffer[OUTPUT_BUFFER_LEN] = {0};
	HAL_UART_StateTypeDef uartState = HAL_UART_STATE_READY;
	HAL_StatusTypeDef halStatus = HAL_OK;

	for(;;)
	{
		// Read stream buffer
		xReceiveSize = xStreamBufferReceive(xStreamBufferStringBle, cReceiveBuffer, OUTPUT_BUFFER_LEN, portMAX_DELAY);

		if (xReceiveSize > 0) {

			// Wait for previous transmission to complete and take Tx semaphore
			xStatus = xSemaphoreTake(uart1TxSemHandle, pdMS_TO_TICKS(STD_DELAY));

			// Transmit bytes read from the stream buffer
			if (pdTRUE == xStatus)
			{
				memcpy(cSendBuffer, cReceiveBuffer, xReceiveSize);
				halStatus = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cSendBuffer, (uint16_t)xReceiveSize);

				// Ensure that API call was successful
				if (halStatus != HAL_OK)
					xSemaphoreGive(uart1TxSemHandle);
			}
			else
			{
				// Check USART state and call generic error handler in case of errors
				uartState = HAL_UART_GetState(&huart1);
				if (HAL_UART_STATE_ERROR == uartState)
					Error_Handler();
			}
		}
	}
  /* USER CODE END bleStrPrintTaskFunction */
}

/* USER CODE BEGIN Header_blePanelTaskFunction */
/**
* @brief Function implementing the blePanelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blePanelTaskFunction */
void blePanelTaskFunction(void *argument)
{
  /* USER CODE BEGIN blePanelTaskFunction */
	char *pcPanelSetup[] = {
		"*.kwl\n",
		"clear_panel()\n",
		"set_grid_size(21,9)\n",
		"add_text(7,0,large,L,Humidity,245,240,245,)\n",
		"add_text_box(7,6,2,C,37.3%,0,0,245,H)\n",
		"add_text_box(9,6,2,C,23.8C,245,0,0,T)\n",
		"add_text(8,8,large,L,Reset system,245,240,245,)\n",
		"add_text_box(0,5,3,C,585mm,206,172,92,D)\n",
		"add_text(17,0,large,C,Right motor power,245,240,245,)\n",
		"add_text(13,0,large,C,Left motor power,245,240,245,)\n",
		"add_text(9,0,large,L,Temperature,245,240,245,)\n",
		"add_text(3,8,large,L,Oled On/Off,245,240,245,)\n",
		"add_text(3,5,large,L,Distance,245,240,245,)\n",
		"add_text(12,4,large,L,Terminal,245,240,245,)\n",
		"add_text(18,8,large,C,Steering Joy,245,240,245,)\n",
		"add_text(16,4,large,L,Tilt status,245,240,245,)\n",
		"add_button(7,8,14,*reset#,)\n",
		"add_switch(1,8,1,*oled on#,*oled off#,0,0)\n",
		"add_free_pad(17,5,0,200,0,50,*joy ,#)\n",
		"add_gauge(7,1,3,0,100,37,H,0%,100%,10,5)\n",
		"add_gauge(9,1,2,0,100,23,T,OC,100C,10,2)\n",
		"add_gauge(0,6,4,0,2000,585,D,0mm,2000mm,10,5)\n",
		"add_gauge(12,1,1,0,200,100,R,BW,FW,10,5)\n",
		"add_gauge(16,1,1,0,200,100,L,BW,FW,10,5)\n",
		"add_led(15,4,1,I,0,255,0)\n",
		"add_xy_graph(0,0,6,-90.0,90.0,-180.0,180.0,5,P,Pitch & Roll,Roll,Pitch,0,0,0,0,1,1,0,1,1,wide,large,1,PR,255,0,0)\n",
		"add_monitor(12,5,5,,1)\n",
		"add_send_box(12,8,5,led off,*,#)\n",
		"set_panel_notes(-,,,)\n",
		"run()\n",
		"*\n"
	};

  /* Infinite loop */
  for(;;)
  {
    vTaskSuspend(NULL);

    // Pause sending data to app during setup
    vTaskSuspend(blePrintTaskHandle);

    for (size_t i = 0; i < (sizeof(pcPanelSetup)/sizeof(char *)); i++) {
    	xSendStringByStreamBuffer(xStreamBufferStringBle, blePrintStringMutexHandle, pcPanelSetup[i], 0, pdMS_TO_TICKS(LONG_DELAY));
    	vTaskDelay(pdMS_TO_TICKS(STD_DELAY));
    }

    // Resume printing to app
    vTaskResume(blePrintTaskHandle);
  }
  /* USER CODE END blePanelTaskFunction */
}

/* USER CODE BEGIN Header_motorsTaskFunction */
/**
* @brief Function implementing the motorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorsTaskFunction */
void motorsTaskFunction(void *argument)
{
  /* USER CODE BEGIN motorsTaskFunction */

	uint32_t freq = PWM_FREQ;
	uint32_t arr = freq - 1;
	uint32_t psc = ((SystemCoreClock / (arr + 1)) / freq) - 1; // 100 timer ticks = 1 PWM period
	Data_t blePrintData = {.type = MOTORS};
	Data_t joyData = {0};
	int32_t X = 0;
	int32_t Y = 0;
	BaseType_t xStatus = pdFALSE;
	int32_t leftMotors = 0;
	int32_t leftMotorsDir = 0;
	int32_t rightMotors = 0;
	int32_t rightMotorsDir = 0;

	// Task synchronization bits
	EventBits_t uxThisTasksSyncBits = MOTORS_INIT_EVENT;
	EventBits_t uxBitsToWaitFor = (FLASH_INIT_EVENT |
																 EVENT_HANDLER_INIT_EVENT |
																 ACCEL_GYRO_INIT_EVENT |
																 OLED_INIT_EVENT |
																 DIST_INIT_EVENT |
																 DIST_TRIGGER_INIT_EVENT |
																 CPU_TEMP_INIT_EVENT);

	// LED timer callback and status
	TimerCallback_t *pLedTimerCallback = NULL;
	BaseType_t *pxLedStatus = NULL;

	// Setup PWM presacler and autoreload
	__HAL_TIM_SET_PRESCALER(&htim2, psc);
	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);

	// Start PWM on all 3 channels
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	// Synchronize task initialization
	xEventGroupSync(commonEventHandle, uxThisTasksSyncBits, uxBitsToWaitFor, pdMS_TO_TICKS(STD_DELAY));

	// Get LED status
	pLedTimerCallback = (TimerCallback_t *)pvTimerGetTimerID(ledTimHandle);
	pxLedStatus = pLedTimerCallback->arg;

	for(;;)
  {
    xStatus = xQueueReceive(joyDataQueueHandle, &joyData, pdMS_TO_TICKS(100));

    if (pdTRUE == xStatus) {

    	// Joystick readings
    	X = joyData.value.joy[0];
    	Y = joyData.value.joy[1];

    	// Calculate motor power
    	if (Y >= 0) {
    		if (X >= 0) {
    			// Top right quarter
    			leftMotors = vectorLength(X, Y);
    			rightMotors = Y - X;
    		} else {
    			// Top left quarter
    			leftMotors = Y - (-X);
    			rightMotors = vectorLength(X, Y);
    		}
    	} else {
    		if (X >= 0) {
    			// Bottom right quarter
    			leftMotors = Y + X;
    			rightMotors = -(vectorLength(X, Y));
    		} else {
    			// Bottom left quarter
    			leftMotors = -(vectorLength(X, Y));
    			rightMotors = Y + -(X);
    		}
    	}

    	// Set motor direction
    	if (leftMotors >= 0) {
    		leftMotorsDir = GPIO_PIN_RESET;
    		blePrintData.value.motors[0] = 100 + leftMotors;
    	} else {
    		leftMotors = -leftMotors;
    		leftMotorsDir = GPIO_PIN_SET;
    		blePrintData.value.motors[0] = 100 - leftMotors;
    	}

    	if (rightMotors >= 0) {
    		rightMotorsDir = GPIO_PIN_RESET;
    		blePrintData.value.motors[1] = 100 + rightMotors;
    	} else {
    		rightMotors = -rightMotors;
    		rightMotorsDir = GPIO_PIN_SET;
    		blePrintData.value.motors[1] = 100 - rightMotors;
    	}

      // Set motor PWM
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, leftMotors);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, rightMotors);

      // Set motor direction
      HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin, leftMotorsDir);
      HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, rightMotorsDir);

      // Update LED timer and PWM
      if (*pxLedStatus == pdTRUE && xTimerIsTimerActive(ledTimHandle) == pdTRUE)
      	xTimerStop(ledTimHandle, pdMS_TO_TICKS(SHORT_DELAY));

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (leftMotors + rightMotors) / 2);

			} else {

				// Switch off motors
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

				// Set motor values to default
				blePrintData.value.motors[0] = 100;
				blePrintData.value.motors[1] = 100;

				// Switch LED blinking back on
	      if (*pxLedStatus == pdTRUE && xTimerIsTimerActive(ledTimHandle) == pdFALSE)
	      	xTimerStart(ledTimHandle, pdMS_TO_TICKS(SHORT_DELAY));
			}

		// Send motor data to BT app
		xQueueSend(blePrintDataQueueHandle, &blePrintData, pdMS_TO_TICKS(SHORT_DELAY));
	}

  /* USER CODE END motorsTaskFunction */
}

/* USER CODE BEGIN Header_tiltCheckTaskFunction */
/**
* @brief Function implementing the tiltCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tiltCheckTaskFunction */
void tiltCheckTaskFunction(void *argument)
{
  /* USER CODE BEGIN tiltCheckTaskFunction */

	Data_t tiltData = {0};
	TickType_t xTicksToWait = pdMS_TO_TICKS(500);
	TimeOut_t xTimeOut = {0};
	BaseType_t xStatus = pdFALSE;

  for(;;)
  {
  	xStatus = xQueueReceive(tiltDataQueueHandle, &tiltData, pdMS_TO_TICKS(LONG_DELAY));

  	if (pdTRUE == xStatus) {

  		if (tiltData.value.pr[0] > MAXIMUM_TILT || tiltData.value.pr[0] < -MAXIMUM_TILT ||
  			  tiltData.value.pr[1] > MAXIMUM_TILT || tiltData.value.pr[1] < -MAXIMUM_TILT) {

  				// Initialize timeout structure with current tick value
  				vTaskSetTimeOutState(&xTimeOut);

  				do {

  					// Wait for timeout
  					if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {

  						// Set tilt alert bit
  						xEventGroupSetBits(commonEventHandle, TILT_ALERT_EVENT);

  						// Continue checking tilt until value is acceptable
  						do {
  							xQueueReceive(tiltDataQueueHandle, &tiltData, pdMS_TO_TICKS(SHORT_DELAY));
  						} while (tiltData.value.pr[0] > MAXIMUM_TILT || tiltData.value.pr[0] < -MAXIMUM_TILT ||
 	  			       tiltData.value.pr[1] > MAXIMUM_TILT || tiltData.value.pr[1] < -MAXIMUM_TILT);

  						// Set tilt alert bit
  						xEventGroupSetBits(commonEventHandle, TILT_OK_EVENT);
  					}

  					// Timeout not reached, continue checking tilt value
  					xQueueReceive(tiltDataQueueHandle, &tiltData, pdMS_TO_TICKS(SHORT_DELAY));

  				} while (tiltData.value.pr[0] > MAXIMUM_TILT || tiltData.value.pr[0] < -MAXIMUM_TILT ||
  	  			       tiltData.value.pr[1] > MAXIMUM_TILT || tiltData.value.pr[1] < -MAXIMUM_TILT);
  		}
  	}
  }
  /* USER CODE END tiltCheckTaskFunction */
}

/* tempHumTimCallback function */
void tempHumTimCallback(void *argument)
{
  /* USER CODE BEGIN tempHumTimCallback */
	vTaskResume(tempHumTaskHandle);
  /* USER CODE END tempHumTimCallback */
}

/* oledTimCallback function */
void oledTimCallback(void *argument)
{
  /* USER CODE BEGIN oledTimCallback */

	// Increment counter, pointed to by argument
	(*(uint32_t*)argument)++;

	// Suspend OLED task and clear screen
	vTaskSuspend(oledTaskHandle);
	memset(screenBuffer, 0x0, sizeof(screenBuffer));

  /* USER CODE END oledTimCallback */
}

/* ledTimCallback function */
void ledTimCallback(void *argument)
{
  /* USER CODE BEGIN ledTimCallback */
	if (__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1))
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	else
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_FREQ);

  /* USER CODE END ledTimCallback */
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
