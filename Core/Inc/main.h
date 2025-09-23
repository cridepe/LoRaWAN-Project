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
#include "stm32l4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIO5_Pin GPIO_PIN_9
#define DIO5_GPIO_Port GPIOE
#define DIO5_EXTI_IRQn EXTI9_5_IRQn
#define RESET_RFM95_Pin GPIO_PIN_11
#define RESET_RFM95_GPIO_Port GPIOE
#define NSS_Pin GPIO_PIN_12
#define NSS_GPIO_Port GPIOE
#define DIO1_Pin GPIO_PIN_10
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI15_10_IRQn
#define DIO0_Pin GPIO_PIN_11
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI15_10_IRQn
#define CAMERA_RESET_Pin GPIO_PIN_6
#define CAMERA_RESET_GPIO_Port GPIOD
#define CAMERA_PWDN_Pin GPIO_PIN_7
#define CAMERA_PWDN_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
