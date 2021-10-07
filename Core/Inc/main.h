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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ICM20948_CS_Pin GPIO_PIN_2
#define ICM20948_CS_GPIO_Port GPIOE
#define J11_RESERVED_Pin GPIO_PIN_3
#define J11_RESERVED_GPIO_Port GPIOE
#define J12_RESERVED_Pin GPIO_PIN_4
#define J12_RESERVED_GPIO_Port GPIOE
#define J13_RESERVED_Pin GPIO_PIN_5
#define J13_RESERVED_GPIO_Port GPIOE
#define J14_RESERVED_Pin GPIO_PIN_6
#define J14_RESERVED_GPIO_Port GPIOE
#define ADC_AMP_36V_Pin GPIO_PIN_0
#define ADC_AMP_36V_GPIO_Port GPIOC
#define BTN_2_PWR_ON_Pin GPIO_PIN_1
#define BTN_2_PWR_ON_GPIO_Port GPIOC
#define ADC_VOLT_BAT_Pin GPIO_PIN_2
#define ADC_VOLT_BAT_GPIO_Port GPIOC
#define ADC_VOLT_PWR_ST_Pin GPIO_PIN_3
#define ADC_VOLT_PWR_ST_GPIO_Port GPIOC
#define BTN_1_LED_Pin GPIO_PIN_0
#define BTN_1_LED_GPIO_Port GPIOA
#define LED_PWM_2_Pin GPIO_PIN_1
#define LED_PWM_2_GPIO_Port GPIOA
#define ADC_IK_CENTER_Pin GPIO_PIN_2
#define ADC_IK_CENTER_GPIO_Port GPIOA
#define ADC_IK_BACK_LEFT_Pin GPIO_PIN_3
#define ADC_IK_BACK_LEFT_GPIO_Port GPIOA
#define ADC_IK_BACK_RIGHT_Pin GPIO_PIN_4
#define ADC_IK_BACK_RIGHT_GPIO_Port GPIOA
#define ADC_IK_FRONT_LEFT_Pin GPIO_PIN_5
#define ADC_IK_FRONT_LEFT_GPIO_Port GPIOA
#define ADC_IK_FRONT_RIGHT_Pin GPIO_PIN_6
#define ADC_IK_FRONT_RIGHT_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_7
#define SERVO_PWM_GPIO_Port GPIOA
#define DRIVER_STEP_Pin GPIO_PIN_4
#define DRIVER_STEP_GPIO_Port GPIOC
#define DRIVER_DIR_Pin GPIO_PIN_5
#define DRIVER_DIR_GPIO_Port GPIOC
#define ADC_AMP_5V_Pin GPIO_PIN_0
#define ADC_AMP_5V_GPIO_Port GPIOB
#define ADC_AMP_12V_Pin GPIO_PIN_1
#define ADC_AMP_12V_GPIO_Port GPIOB
#define J1_SERVICE_LED_Pin GPIO_PIN_2
#define J1_SERVICE_LED_GPIO_Port GPIOB
#define J2_GYRO_SELECTOR_Pin GPIO_PIN_7
#define J2_GYRO_SELECTOR_GPIO_Port GPIOE
#define J3_SERVO_SELECTOR_Pin GPIO_PIN_8
#define J3_SERVO_SELECTOR_GPIO_Port GPIOE
#define J4_POWER_LOGIC_Pin GPIO_PIN_9
#define J4_POWER_LOGIC_GPIO_Port GPIOE
#define J5_LED_LINE_Pin GPIO_PIN_10
#define J5_LED_LINE_GPIO_Port GPIOE
#define J6_IK_SENSORS_Pin GPIO_PIN_11
#define J6_IK_SENSORS_GPIO_Port GPIOE
#define J7_UART_PROTO_Pin GPIO_PIN_12
#define J7_UART_PROTO_GPIO_Port GPIOE
#define J8_AUTO_PARKING_Pin GPIO_PIN_13
#define J8_AUTO_PARKING_GPIO_Port GPIOE
#define J9_GYRO_STABLE_Pin GPIO_PIN_14
#define J9_GYRO_STABLE_GPIO_Port GPIOE
#define J10_FACTORY_MODE_Pin GPIO_PIN_15
#define J10_FACTORY_MODE_GPIO_Port GPIOE
#define LED_PWM_3_Pin GPIO_PIN_10
#define LED_PWM_3_GPIO_Port GPIOB
#define LED_PWM_4_Pin GPIO_PIN_11
#define LED_PWM_4_GPIO_Port GPIOB
#define ARDUINO_MPU9250_TX_Pin GPIO_PIN_8
#define ARDUINO_MPU9250_TX_GPIO_Port GPIOD
#define ARDUINO_MPU9250_RX_Pin GPIO_PIN_9
#define ARDUINO_MPU9250_RX_GPIO_Port GPIOD
#define LED_0_Pin GPIO_PIN_12
#define LED_0_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOD
#define PLATFORM_PWR_ON_RELAY_Pin GPIO_PIN_7
#define PLATFORM_PWR_ON_RELAY_GPIO_Port GPIOC
#define PWR_STATION_RELAY_Pin GPIO_PIN_8
#define PWR_STATION_RELAY_GPIO_Port GPIOC
#define J15_RESERVED_Pin GPIO_PIN_9
#define J15_RESERVED_GPIO_Port GPIOC
#define JETSON_TX_Pin GPIO_PIN_9
#define JETSON_TX_GPIO_Port GPIOA
#define JETSON_RX_Pin GPIO_PIN_10
#define JETSON_RX_GPIO_Port GPIOA
#define LED_PWM_1_Pin GPIO_PIN_15
#define LED_PWM_1_GPIO_Port GPIOA
#define GYROSCOOTER_TX_Pin GPIO_PIN_5
#define GYROSCOOTER_TX_GPIO_Port GPIOD
#define GYROSCOOTER_RX_Pin GPIO_PIN_6
#define GYROSCOOTER_RX_GPIO_Port GPIOD
#define ICM20948_SCK_Pin GPIO_PIN_3
#define ICM20948_SCK_GPIO_Port GPIOB
#define ICM20948_MISO_Pin GPIO_PIN_4
#define ICM20948_MISO_GPIO_Port GPIOB
#define ICM20948_MOSI_Pin GPIO_PIN_5
#define ICM20948_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
