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
#define ENA_2_Pin GPIO_PIN_13
#define ENA_2_GPIO_Port GPIOC
#define ENA_1_Pin GPIO_PIN_14
#define ENA_1_GPIO_Port GPIOC
#define STEP_3_Pin GPIO_PIN_1
#define STEP_3_GPIO_Port GPIOC
#define SWITCH1_Pin GPIO_PIN_2
#define SWITCH1_GPIO_Port GPIOC
#define SWITCH1_EXTI_IRQn EXTI2_IRQn
#define SWITCH2_Pin GPIO_PIN_3
#define SWITCH2_GPIO_Port GPIOC
#define SWITCH2_EXTI_IRQn EXTI3_IRQn
#define LEVEL_VBAT_Pin GPIO_PIN_1
#define LEVEL_VBAT_GPIO_Port GPIOA
#define DIR_4_Pin GPIO_PIN_4
#define DIR_4_GPIO_Port GPIOA
#define STEP_4_Pin GPIO_PIN_5
#define STEP_4_GPIO_Port GPIOA
#define DIR_3_Pin GPIO_PIN_6
#define DIR_3_GPIO_Port GPIOA
#define DIR_2_Pin GPIO_PIN_4
#define DIR_2_GPIO_Port GPIOC
#define STEP_2_Pin GPIO_PIN_5
#define STEP_2_GPIO_Port GPIOC
#define DIR_1_Pin GPIO_PIN_0
#define DIR_1_GPIO_Port GPIOB
#define STEP_1_Pin GPIO_PIN_1
#define STEP_1_GPIO_Port GPIOB
#define XSHUT_1_Pin GPIO_PIN_12
#define XSHUT_1_GPIO_Port GPIOB
#define XSHUT_2_Pin GPIO_PIN_13
#define XSHUT_2_GPIO_Port GPIOB
#define XSHUT_3_Pin GPIO_PIN_14
#define XSHUT_3_GPIO_Port GPIOB
#define XSHUT_4_Pin GPIO_PIN_15
#define XSHUT_4_GPIO_Port GPIOB
#define M2_Pin GPIO_PIN_6
#define M2_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_7
#define M1_GPIO_Port GPIOC
#define M0_Pin GPIO_PIN_8
#define M0_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define DIR_5_Pin GPIO_PIN_11
#define DIR_5_GPIO_Port GPIOA
#define STEP_5_Pin GPIO_PIN_12
#define STEP_5_GPIO_Port GPIOA
#define ENA_5_Pin GPIO_PIN_15
#define ENA_5_GPIO_Port GPIOA
#define ENA_4_Pin GPIO_PIN_10
#define ENA_4_GPIO_Port GPIOC
#define ENA_3_Pin GPIO_PIN_11
#define ENA_3_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
