/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define KEYPAD1_B_Pin GPIO_PIN_13
#define KEYPAD1_B_GPIO_Port GPIOC
#define KEYPAD1_A_Pin GPIO_PIN_14
#define KEYPAD1_A_GPIO_Port GPIOC
#define LCD1_back_light_Pin GPIO_PIN_15
#define LCD1_back_light_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define pb1_Pin GPIO_PIN_2
#define pb1_GPIO_Port GPIOC
#define pb1_EXTI_IRQn EXTI2_IRQn
#define KEYPAD2_A_Pin GPIO_PIN_0
#define KEYPAD2_A_GPIO_Port GPIOA
#define KEYPAD2_B_Pin GPIO_PIN_1
#define KEYPAD2_B_GPIO_Port GPIOA
#define KEYPAD2_C_Pin GPIO_PIN_2
#define KEYPAD2_C_GPIO_Port GPIOA
#define KEYPAD2_D_Pin GPIO_PIN_3
#define KEYPAD2_D_GPIO_Port GPIOA
#define KEYPAD2_EXT_Pin GPIO_PIN_4
#define KEYPAD2_EXT_GPIO_Port GPIOA
#define KEYPAD2_EXT_EXTI_IRQn EXTI4_IRQn
#define LCD2_RS_Pin GPIO_PIN_5
#define LCD2_RS_GPIO_Port GPIOA
#define LCD2_RW_Pin GPIO_PIN_6
#define LCD2_RW_GPIO_Port GPIOA
#define LCD2_EN_Pin GPIO_PIN_7
#define LCD2_EN_GPIO_Port GPIOA
#define LCD2_back_light_Pin GPIO_PIN_4
#define LCD2_back_light_GPIO_Port GPIOC
#define LCD2_D4_Pin GPIO_PIN_1
#define LCD2_D4_GPIO_Port GPIOB
#define LCD2_D5_Pin GPIO_PIN_2
#define LCD2_D5_GPIO_Port GPIOB
#define LCD2_D6_Pin GPIO_PIN_10
#define LCD2_D6_GPIO_Port GPIOB
#define LCD2_D7_Pin GPIO_PIN_11
#define LCD2_D7_GPIO_Port GPIOB
#define SW8_Pin GPIO_PIN_12
#define SW8_GPIO_Port GPIOB
#define SW7_Pin GPIO_PIN_13
#define SW7_GPIO_Port GPIOB
#define SW6_Pin GPIO_PIN_14
#define SW6_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_15
#define SW5_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_6
#define SW4_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_7
#define SW3_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_8
#define SW2_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_9
#define SW1_GPIO_Port GPIOC
#define LCD1_EN_Pin GPIO_PIN_15
#define LCD1_EN_GPIO_Port GPIOA
#define LCD1_RW_Pin GPIO_PIN_12
#define LCD1_RW_GPIO_Port GPIOC
#define LCD1_RS_Pin GPIO_PIN_2
#define LCD1_RS_GPIO_Port GPIOD
#define LCD1_D4_Pin GPIO_PIN_4
#define LCD1_D4_GPIO_Port GPIOB
#define LCD1_D5_Pin GPIO_PIN_5
#define LCD1_D5_GPIO_Port GPIOB
#define LCD1_D6_Pin GPIO_PIN_6
#define LCD1_D6_GPIO_Port GPIOB
#define LCD1_D7_Pin GPIO_PIN_7
#define LCD1_D7_GPIO_Port GPIOB
#define KEYPAD1_D_Pin GPIO_PIN_8
#define KEYPAD1_D_GPIO_Port GPIOB
#define KEYPAD1_C_Pin GPIO_PIN_9
#define KEYPAD1_C_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
