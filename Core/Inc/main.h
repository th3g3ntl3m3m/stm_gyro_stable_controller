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
#define ICM20948_CS_Pin GPIO_PIN_2
#define ICM20948_CS_GPIO_Port GPIOE
#define BTN_PARK_UP_Pin GPIO_PIN_4
#define BTN_PARK_UP_GPIO_Port GPIOE
#define BTN_PARK_DOWN_Pin GPIO_PIN_5
#define BTN_PARK_DOWN_GPIO_Port GPIOE
#define ADC_6_Pin GPIO_PIN_0
#define ADC_6_GPIO_Port GPIOC
#define ADC_7_Pin GPIO_PIN_1
#define ADC_7_GPIO_Port GPIOC
#define ADC_8_Pin GPIO_PIN_2
#define ADC_8_GPIO_Port GPIOC
#define ADC_1_Pin GPIO_PIN_0
#define ADC_1_GPIO_Port GPIOA
#define ADC_2_Pin GPIO_PIN_1
#define ADC_2_GPIO_Port GPIOA
#define ADC_3_Pin GPIO_PIN_2
#define ADC_3_GPIO_Port GPIOA
#define DRIVER_STEP_Pin GPIO_PIN_4
#define DRIVER_STEP_GPIO_Port GPIOC
#define DRIVER_DIR_Pin GPIO_PIN_5
#define DRIVER_DIR_GPIO_Port GPIOC
#define ADC_4_Pin GPIO_PIN_0
#define ADC_4_GPIO_Port GPIOB
#define ADC_5_Pin GPIO_PIN_1
#define ADC_5_GPIO_Port GPIOB
#define DRIVER_EN_Pin GPIO_PIN_8
#define DRIVER_EN_GPIO_Port GPIOE
#define PWM_LED1_Pin GPIO_PIN_9
#define PWM_LED1_GPIO_Port GPIOE
#define PWM_LED2_Pin GPIO_PIN_11
#define PWM_LED2_GPIO_Port GPIOE
#define PWM_LED3_Pin GPIO_PIN_13
#define PWM_LED3_GPIO_Port GPIOE
#define HIGH_RX_Pin GPIO_PIN_11
#define HIGH_RX_GPIO_Port GPIOB
#define ICM20948_SCK_Pin GPIO_PIN_13
#define ICM20948_SCK_GPIO_Port GPIOB
#define ICM20948_MISO_Pin GPIO_PIN_14
#define ICM20948_MISO_GPIO_Port GPIOB
#define ICM20948_MOSI_Pin GPIO_PIN_15
#define ICM20948_MOSI_GPIO_Port GPIOB
#define HIGH_TX_Pin GPIO_PIN_8
#define HIGH_TX_GPIO_Port GPIOD
#define LED_0_Pin GPIO_PIN_12
#define LED_0_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOD
#define LOW_TX_Pin GPIO_PIN_5
#define LOW_TX_GPIO_Port GPIOD
#define LOW_RX_Pin GPIO_PIN_6
#define LOW_RX_GPIO_Port GPIOD
#define PWM_LED4_Pin GPIO_PIN_7
#define PWM_LED4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
