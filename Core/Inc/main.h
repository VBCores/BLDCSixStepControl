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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t micros_k;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_DIV_Pin GPIO_PIN_0
#define BAT_DIV_GPIO_Port GPIOC
#define I_A_Pin GPIO_PIN_1
#define I_A_GPIO_Port GPIOC
#define I_B_Pin GPIO_PIN_2
#define I_B_GPIO_Port GPIOC
#define I_C_Pin GPIO_PIN_3
#define I_C_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define INLA_Pin GPIO_PIN_13
#define INLA_GPIO_Port GPIOB
#define INLB_Pin GPIO_PIN_14
#define INLB_GPIO_Port GPIOB
#define INLC_Pin GPIO_PIN_15
#define INLC_GPIO_Port GPIOB
#define ENC2_4_Pin GPIO_PIN_7
#define ENC2_4_GPIO_Port GPIOC
#define ENC2_4_EXTI_IRQn EXTI9_5_IRQn
#define ENC2_5_Pin GPIO_PIN_8
#define ENC2_5_GPIO_Port GPIOC
#define ENC2_5_EXTI_IRQn EXTI9_5_IRQn
#define INHA_Pin GPIO_PIN_8
#define INHA_GPIO_Port GPIOA
#define INHB_Pin GPIO_PIN_9
#define INHB_GPIO_Port GPIOA
#define INHC_Pin GPIO_PIN_10
#define INHC_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define EN_GATE_Pin GPIO_PIN_3
#define EN_GATE_GPIO_Port GPIOB
#define FAULT_Pin GPIO_PIN_5
#define FAULT_GPIO_Port GPIOB
#define ENC2_3_Pin GPIO_PIN_6
#define ENC2_3_GPIO_Port GPIOB
#define ENC2_3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
