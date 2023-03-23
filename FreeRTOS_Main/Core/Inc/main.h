/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern SPI_HandleTypeDef hspi2;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STD_DELAY 250
#define SHORT_DELAY 50
#define DEL_CHAR 127
#define INPUT_BUFFER_LEN 100
#define OUTPUT_BUFFER_LEN 500
#define NEWPAGE_CHAR 12
#define QUEUE_SIZE 10
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Joy_X_Pin GPIO_PIN_0
#define Joy_X_GPIO_Port GPIOC
#define Joy_Y_Pin GPIO_PIN_1
#define Joy_Y_GPIO_Port GPIOC
#define OLED_MOSI_Pin GPIO_PIN_3
#define OLED_MOSI_GPIO_Port GPIOC
#define DIST_ECHO_Pin GPIO_PIN_0
#define DIST_ECHO_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_4
#define OLED_RES_GPIO_Port GPIOC
#define OLED_CS_Pin GPIO_PIN_0
#define OLED_CS_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_1
#define OLED_DC_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_13
#define OLED_SCK_GPIO_Port GPIOB
#define MOTION_INT_Pin GPIO_PIN_7
#define MOTION_INT_GPIO_Port GPIOC
#define MOTION_INT_EXTI_IRQn EXTI9_5_IRQn
#define TEMP_HUM_SENS_Pin GPIO_PIN_8
#define TEMP_HUM_SENS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DIST_TRIGGER_Pin GPIO_PIN_6
#define DIST_TRIGGER_GPIO_Port GPIOB
#define ACCELGYRO_SDA_Pin GPIO_PIN_7
#define ACCELGYRO_SDA_GPIO_Port GPIOB
#define ACCELGYRO_SCL_Pin GPIO_PIN_8
#define ACCELGYRO_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Common event bits
#define BUTTON_EVENT (0x01)
#define MOTION_INT_EVENT (0x02)
#define I2C_MEM_READ_EVENT (0x04)

// Task synchronization (initialization)
#define EVENT_HANDLER_INIT_EVENT (0x08)
#define CPU_TEMP_INIT_EVENT (0x10)
#define ACCEL_GYRO_INIT_EVENT (0x20)
#define OLED_INIT_EVENT (0x40)
#define DIST_INIT_EVENT (0x80)
#define DIST_TRIGGER_INIT_EVENT (0x100)
#define DIST_ECHO_EVENT (0x200)
#define TEMP_HUM_IC_CPLT (0x400)
#define SAVE_TO_FLASH (0x800)
#define LOAD_FROM_FLASH (0x1000)
#define FLASH_INIT_EVENT (0x2000)


// CPU temperature readings
#define VDD (3.3)
#define MAX_ADC_VAL_12_BITS (4095)
#define TEMP_30 (30.0)
#define TEMP_110_MINUS_30 (80.0)

// Accelerometer/gyroscope registers and bits
#define ACCELGYRO_WIN_SIZE (10)
#define ACCELGYRO_DEVICE (0xD0)
#define ACCELGYRO_WHO_AM_I_ADDR (0x75)
#define ACCELGYRO_DEFAULT_ACCEL (2)
#define ACCELGYRO_DEFAULT_GYRO (250)
#define ACCELGYRO_PWR_MGMT_1_ADDR (0x6B)
#define ACCELGYRO_DEVICE_RESET_SET (0x80)
#define ACCELGYRO_DEVICE_RESET_RESET (0x80)
#define ACCELGYRO_DEVICE_RESET_MASK (0x80)
#define ACCELGYRO_SLEEP_SET (0x40)
#define ACCELGYRO_SLEEP_RESET (0x00)
#define ACCELGYRO_SLEEP_MASK (0x40)
#define ACCELGYRO_ACCEL_ADDR (0x3B)
#define ACCELGYRO_GYRO_ADDR (0x43)

// Motion detection registers and bits
#define ACCELGYRO_INT_ENABLE_ADDR (0x38)
#define ACCELGYRO_MOTION_INT_SET (0x40)
#define ACCELGYRO_MOTION_INT_RESET (0x00)
#define ACCELGYRO_MOTION_INT_MASK (0x40)
#define ACCELGYRO_MOTION_DUR_ADDR (0x20)
#define ACCELGYRO_MOTION_DUR_5	(0x05)
#define ACCELGYRO_MOTION_DUR_MASK (0xFF)
#define ACCELGYRO_MOTION_THR_ADDR (0x1F)
#define ACCELGYRO_MOTION_THR_10	(0x0A)
#define ACCELGYRO_MOTION_THR_MASK (0xFF)

// Motion status registers and bits
#define ACCELGYRO_INT_STATUS_ADDR (0x3A)
#define ACCELGYRO_INT_STATUS_MOTION (0x40)
#define ACCELGYRO_MOTION_STATUS_ADDR (0x61)
#define ACCELGYRO_MOTION_STATUS_ADDR (0x61)
#define ACCELGYRO_MOTION_STATUS_X_NEG (0x80)
#define ACCELGYRO_MOTION_STATUS_X_POS (0x40)
#define ACCELGYRO_MOTION_STATUS_Y_NEG (0x20)
#define ACCELGYRO_MOTION_STATUS_Y_POS (0x10)
#define ACCELGYRO_MOTION_STATUS_Z_NEG (0x08)
#define ACCELGYRO_MOTION_STATUS_Z_POS (0x04)

// Big endian to little endian conversion
#define SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))

// Minimum value macro
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

// OLED screen
#define OLED_SCREEN_WIDTH (96)
#define OLED_SCREEN_HEIGHT (64)
#define TEXT_ARRAY (100)
#define __SSD1331_RES_SET()           HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET)
#define __SSD1331_RES_CLR()           HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET)
#define __SSD1331_DC_SET()            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define __SSD1331_DC_CLR()            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define __SSD1331_CS_SET()            HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)
#define __SSD1331_CS_CLR()            HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
#define __SSD1331_WRITE_BYTE(__DATA)  HAL_SPI_Transmit(&hspi2, __DATA, 1, 1000)

// Distance sensor
#define TIME_TO_DIST_CM (58)

// Flash memory
#define FLASH_SECTOR7_ADDR (0x08060000);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
