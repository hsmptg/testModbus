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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint16_t spk_cnt;
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
#define LED_USER_Pin GPIO_PIN_13
#define LED_USER_GPIO_Port GPIOC
#define ADDR0_Pin GPIO_PIN_14
#define ADDR0_GPIO_Port GPIOC
#define SPEAKER_Pin GPIO_PIN_15
#define SPEAKER_GPIO_Port GPIOC
#define PN532_RST_Pin GPIO_PIN_0
#define PN532_RST_GPIO_Port GPIOA
#define PN532_IRQ_Pin GPIO_PIN_1
#define PN532_IRQ_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_0
#define SS_GPIO_Port GPIOB
#define Pin6_Pin GPIO_PIN_15
#define Pin6_GPIO_Port GPIOA
#define Pin5_Pin GPIO_PIN_4
#define Pin5_GPIO_Port GPIOB
#define Pin4_Pin GPIO_PIN_5
#define Pin4_GPIO_Port GPIOB
#define Pin3_Pin GPIO_PIN_6
#define Pin3_GPIO_Port GPIOB
#define PinCon_Pin GPIO_PIN_7
#define PinCon_GPIO_Port GPIOB
#define PinX_Pin GPIO_PIN_8
#define PinX_GPIO_Port GPIOB
#define PinX_EXTI_IRQn EXTI9_5_IRQn
#define ADDR1_Pin GPIO_PIN_9
#define ADDR1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
