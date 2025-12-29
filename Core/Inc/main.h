/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "app.h"
#include "modbus.h"
#include "mmodbus_hal.h"
#include "smodbus_hal.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_DB1_Pin GPIO_PIN_3
#define LED_DB1_GPIO_Port GPIOE
#define LED_DB2_Pin GPIO_PIN_4
#define LED_DB2_GPIO_Port GPIOE
#define LED_DB3_Pin GPIO_PIN_5
#define LED_DB3_GPIO_Port GPIOE
#define LED_DB4_Pin GPIO_PIN_6
#define LED_DB4_GPIO_Port GPIOE
#define LED_DB5_Pin GPIO_PIN_13
#define LED_DB5_GPIO_Port GPIOC
#define EEPR_WP_Pin GPIO_PIN_2
#define EEPR_WP_GPIO_Port GPIOF
#define MCP2515_CS_Pin GPIO_PIN_3
#define MCP2515_CS_GPIO_Port GPIOC
#define Relay_Out_1_Pin GPIO_PIN_3
#define Relay_Out_1_GPIO_Port GPIOA
#define Relay_Out_2_Pin GPIO_PIN_4
#define Relay_Out_2_GPIO_Port GPIOA
#define Relay_Out_3_Pin GPIO_PIN_5
#define Relay_Out_3_GPIO_Port GPIOA
#define Relay_Out_4_Pin GPIO_PIN_6
#define Relay_Out_4_GPIO_Port GPIOA
#define Relay_out_5_Pin GPIO_PIN_0
#define Relay_out_5_GPIO_Port GPIOB
#define Relay_Out_6_Pin GPIO_PIN_1
#define Relay_Out_6_GPIO_Port GPIOB
#define Relay_Out_7_Pin GPIO_PIN_11
#define Relay_Out_7_GPIO_Port GPIOF
#define Relay_Out_14_Pin GPIO_PIN_12
#define Relay_Out_14_GPIO_Port GPIOE
#define Relay_Out_13_Pin GPIO_PIN_13
#define Relay_Out_13_GPIO_Port GPIOE
#define Relay_Out_12_Pin GPIO_PIN_14
#define Relay_Out_12_GPIO_Port GPIOE
#define Relay_Out_11_Pin GPIO_PIN_15
#define Relay_Out_11_GPIO_Port GPIOE
#define Relay_Out_10_Pin GPIO_PIN_10
#define Relay_Out_10_GPIO_Port GPIOB
#define Relay_Out_9_Pin GPIO_PIN_12
#define Relay_Out_9_GPIO_Port GPIOB
#define Relay_Out_8_Pin GPIO_PIN_8
#define Relay_Out_8_GPIO_Port GPIOD
#define Flash_Reset_Pin GPIO_PIN_10
#define Flash_Reset_GPIO_Port GPIOD
#define Out_Cmd2_Pin GPIO_PIN_8
#define Out_Cmd2_GPIO_Port GPIOG
#define Out_Cmd3_Pin GPIO_PIN_6
#define Out_Cmd3_GPIO_Port GPIOC
#define Out_Cmd4_Pin GPIO_PIN_7
#define Out_Cmd4_GPIO_Port GPIOC
#define Out_Cmd5_Pin GPIO_PIN_9
#define Out_Cmd5_GPIO_Port GPIOC
#define Out_Cmd7_Pin GPIO_PIN_4
#define Out_Cmd7_GPIO_Port GPIOD
#define Out_Cmd8_Pin GPIO_PIN_5
#define Out_Cmd8_GPIO_Port GPIOD
#define Out_Cmd9_Pin GPIO_PIN_6
#define Out_Cmd9_GPIO_Port GPIOD
#define Out_Cmd10_Pin GPIO_PIN_10
#define Out_Cmd10_GPIO_Port GPIOG
#define Out_Cmd11_Pin GPIO_PIN_14
#define Out_Cmd11_GPIO_Port GPIOG
#define SPI_TO_CAN_INT_Pin GPIO_PIN_15
#define SPI_TO_CAN_INT_GPIO_Port GPIOG
#define Out_Cmd12_Pin GPIO_PIN_9
#define Out_Cmd12_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
