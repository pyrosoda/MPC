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
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define DIG1_Pin GPIO_PIN_0
#define DIG1_GPIO_Port GPIOC
#define DIG2_Pin GPIO_PIN_1
#define DIG2_GPIO_Port GPIOC
#define DIG3_Pin GPIO_PIN_2
#define DIG3_GPIO_Port GPIOC
#define DIG4_Pin GPIO_PIN_3
#define DIG4_GPIO_Port GPIOC
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define SEG_D_Pin GPIO_PIN_1
#define SEG_D_GPIO_Port GPIOB
#define SEG_E_Pin GPIO_PIN_11
#define SEG_E_GPIO_Port GPIOB
#define SEG_F_Pin GPIO_PIN_12
#define SEG_F_GPIO_Port GPIOB
#define SEG_A_Pin GPIO_PIN_13
#define SEG_A_GPIO_Port GPIOB
#define SEG_B_Pin GPIO_PIN_14
#define SEG_B_GPIO_Port GPIOB
#define SEG_C_Pin GPIO_PIN_15
#define SEG_C_GPIO_Port GPIOB
#define GREEN1_Pin GPIO_PIN_6
#define GREEN1_GPIO_Port GPIOC
#define RED_Pin GPIO_PIN_8
#define RED_GPIO_Port GPIOC
#define GREEN3_Pin GPIO_PIN_9
#define GREEN3_GPIO_Port GPIOC
#define SEG_G_Pin GPIO_PIN_11
#define SEG_G_GPIO_Port GPIOA
#define SEG_DP_Pin GPIO_PIN_12
#define SEG_DP_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define GREEN2_Pin GPIO_PIN_8
#define GREEN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
