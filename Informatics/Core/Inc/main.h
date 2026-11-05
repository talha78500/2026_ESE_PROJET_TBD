/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_SLEEP_Pin GPIO_PIN_13
#define MOTOR_SLEEP_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOC
#define M1_ENCA_Pin GPIO_PIN_0
#define M1_ENCA_GPIO_Port GPIOA
#define M1_ENCB_Pin GPIO_PIN_1
#define M1_ENCB_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define M2_ENCA_Pin GPIO_PIN_4
#define M2_ENCA_GPIO_Port GPIOA
#define M2_ENCB_Pin GPIO_PIN_6
#define M2_ENCB_GPIO_Port GPIOA
#define USART1_UNUSED_Pin GPIO_PIN_4
#define USART1_UNUSED_GPIO_Port GPIOC
#define LIDAR_RX_Pin GPIO_PIN_5
#define LIDAR_RX_GPIO_Port GPIOC
#define TOF1_INT_Pin GPIO_PIN_10
#define TOF1_INT_GPIO_Port GPIOB
#define TOF1_XST_Pin GPIO_PIN_11
#define TOF1_XST_GPIO_Port GPIOB
#define TOF2_INT_Pin GPIO_PIN_12
#define TOF2_INT_GPIO_Port GPIOB
#define TOF2_XST_Pin GPIO_PIN_13
#define TOF2_XST_GPIO_Port GPIOB
#define NEOPIXEL_DATA_Pin GPIO_PIN_14
#define NEOPIXEL_DATA_GPIO_Port GPIOB
#define LIDAR_CTRL_Pin GPIO_PIN_6
#define LIDAR_CTRL_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_11
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_12
#define SERVO2_GPIO_Port GPIOA
#define FG_GPIO_Pin GPIO_PIN_2
#define FG_GPIO_GPIO_Port GPIOD
#define CHARGEUR_INT_Pin GPIO_PIN_4
#define CHARGEUR_INT_GPIO_Port GPIOB
#define CHARGEUR_PG_Pin GPIO_PIN_5
#define CHARGEUR_PG_GPIO_Port GPIOB
#define CHARGEUR_QON_Pin GPIO_PIN_6
#define CHARGEUR_QON_GPIO_Port GPIOB
#define COLOR_INT_Pin GPIO_PIN_9
#define COLOR_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
