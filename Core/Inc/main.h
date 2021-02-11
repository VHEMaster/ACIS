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
#include "stm32f7xx_hal.h"

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
#define SENS_CSPS_Pin GPIO_PIN_13
#define SENS_CSPS_GPIO_Port GPIOC
#define SENS_CSPS_EXTI_IRQn EXTI15_10_IRQn
#define SENS_HALL_Pin GPIO_PIN_14
#define SENS_HALL_GPIO_Port GPIOC
#define SENS_HALL_EXTI_IRQn EXTI15_10_IRQn
#define SENS_ACC_Pin GPIO_PIN_15
#define SENS_ACC_GPIO_Port GPIOC
#define SENS_ACC_EXTI_IRQn EXTI15_10_IRQn
#define VDD3V3EN_Pin GPIO_PIN_0
#define VDD3V3EN_GPIO_Port GPIOC
#define ADC1_12_TEMP_Pin GPIO_PIN_2
#define ADC1_12_TEMP_GPIO_Port GPIOC
#define ADC1_13_BAT_Pin GPIO_PIN_3
#define ADC1_13_BAT_GPIO_Port GPIOC
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_4
#define ADC4_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_5
#define ADC5_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_6
#define ADC6_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_7
#define ADC7_GPIO_Port GPIOA
#define ADC_nCE_Pin GPIO_PIN_4
#define ADC_nCE_GPIO_Port GPIOC
#define ADC_nCS_Pin GPIO_PIN_0
#define ADC_nCS_GPIO_Port GPIOB
#define TIM8_CH3N_ADC_RW_Pin GPIO_PIN_1
#define TIM8_CH3N_ADC_RW_GPIO_Port GPIOB
#define ADC_FORMAT_Pin GPIO_PIN_2
#define ADC_FORMAT_GPIO_Port GPIOB
#define ADC_BPO_Pin GPIO_PIN_10
#define ADC_BPO_GPIO_Port GPIOB
#define ADC_STATUS_Pin GPIO_PIN_11
#define ADC_STATUS_GPIO_Port GPIOB
#define ADC_STATUS_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI2_WP_Pin GPIO_PIN_6
#define SPI2_WP_GPIO_Port GPIOC
#define LED_CHECK_Pin GPIO_PIN_7
#define LED_CHECK_GPIO_Port GPIOC
#define TIM3_CH3_CHOKE_Pin GPIO_PIN_8
#define TIM3_CH3_CHOKE_GPIO_Port GPIOC
#define TIM3_CH4_ACC_Pin GPIO_PIN_9
#define TIM3_CH4_ACC_GPIO_Port GPIOC
#define USART1_COMM_Pin GPIO_PIN_8
#define USART1_COMM_GPIO_Port GPIOA
#define MCU_IGN_Pin GPIO_PIN_11
#define MCU_IGN_GPIO_Port GPIOA
#define RELAY_START_Pin GPIO_PIN_15
#define RELAY_START_GPIO_Port GPIOA
#define RELAY_IGN_Pin GPIO_PIN_10
#define RELAY_IGN_GPIO_Port GPIOC
#define SENS_PARK_Pin GPIO_PIN_11
#define SENS_PARK_GPIO_Port GPIOC
#define SENS_GEAR_Pin GPIO_PIN_12
#define SENS_GEAR_GPIO_Port GPIOC
#define ECON_Pin GPIO_PIN_2
#define ECON_GPIO_Port GPIOD
#define PROPANE_IN_Pin GPIO_PIN_4
#define PROPANE_IN_GPIO_Port GPIOB
#define PETROL_IN_Pin GPIO_PIN_5
#define PETROL_IN_GPIO_Port GPIOB
#define IGN_23_Pin GPIO_PIN_6
#define IGN_23_GPIO_Port GPIOB
#define IGN_14_Pin GPIO_PIN_7
#define IGN_14_GPIO_Port GPIOB
#define PROPANE_OUT_Pin GPIO_PIN_8
#define PROPANE_OUT_GPIO_Port GPIOB
#define PETROL_OUT_Pin GPIO_PIN_9
#define PETROL_OUT_GPIO_Port GPIOB
#define TACHOMETER_GPIO_Port GPIOH
#define TACHOMETER_Pin GPIO_PIN_0
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
