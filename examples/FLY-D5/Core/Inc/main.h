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
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_dma.h"

#include "stm32f0xx_ll_exti.h"

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
#define X_UART_Pin GPIO_PIN_13
#define X_UART_GPIO_Port GPIOC
#define X_DIR_Pin GPIO_PIN_14
#define X_DIR_GPIO_Port GPIOC
#define X_STEP_Pin GPIO_PIN_15
#define X_STEP_GPIO_Port GPIOC
#define BED_temp_Pin GPIO_PIN_0
#define BED_temp_GPIO_Port GPIOC
#define E_temp_Pin GPIO_PIN_1
#define E_temp_GPIO_Port GPIOC
#define X_EN_Pin GPIO_PIN_2
#define X_EN_GPIO_Port GPIOC
#define Y_UART_Pin GPIO_PIN_3
#define Y_UART_GPIO_Port GPIOC
#define Y_DIR_Pin GPIO_PIN_0
#define Y_DIR_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_1
#define Y_STEP_GPIO_Port GPIOA
#define Y_EN_Pin GPIO_PIN_2
#define Y_EN_GPIO_Port GPIOA
#define Z_UART_Pin GPIO_PIN_3
#define Z_UART_GPIO_Port GPIOA
#define Z_DIR_Pin GPIO_PIN_4
#define Z_DIR_GPIO_Port GPIOA
#define Z_STEP_Pin GPIO_PIN_5
#define Z_STEP_GPIO_Port GPIOA
#define Z_EN_Pin GPIO_PIN_6
#define Z_EN_GPIO_Port GPIOA
#define E0_UART_Pin GPIO_PIN_7
#define E0_UART_GPIO_Port GPIOA
#define E0_DIR_Pin GPIO_PIN_4
#define E0_DIR_GPIO_Port GPIOC
#define E0_STEP_Pin GPIO_PIN_5
#define E0_STEP_GPIO_Port GPIOC
#define E0_EN_Pin GPIO_PIN_0
#define E0_EN_GPIO_Port GPIOB
#define E1_UART_Pin GPIO_PIN_1
#define E1_UART_GPIO_Port GPIOB
#define E1_DIR_Pin GPIO_PIN_2
#define E1_DIR_GPIO_Port GPIOB
#define E1_STEP_Pin GPIO_PIN_10
#define E1_STEP_GPIO_Port GPIOB
#define E1_EN_Pin GPIO_PIN_11
#define E1_EN_GPIO_Port GPIOB
#define T0_HEAT_Pin GPIO_PIN_6
#define T0_HEAT_GPIO_Port GPIOC
#define BED_HEAT_Pin GPIO_PIN_7
#define BED_HEAT_GPIO_Port GPIOC
#define FAN0_Pin GPIO_PIN_8
#define FAN0_GPIO_Port GPIOC
#define FAN1_Pin GPIO_PIN_9
#define FAN1_GPIO_Port GPIOC
#define SWDIO_LCD_D5_Pin GPIO_PIN_13
#define SWDIO_LCD_D5_GPIO_Port GPIOA
#define SW_CLK_LCD_RS_Pin GPIO_PIN_14
#define SW_CLK_LCD_RS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
