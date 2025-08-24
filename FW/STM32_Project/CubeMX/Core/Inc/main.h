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
#include "stm32c0xx_hal.h"

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
#define MCU_PMIC_PWR_EN_Pin GPIO_PIN_0
#define MCU_PMIC_PWR_EN_GPIO_Port GPIOA
#define MCU_PMIC_PGOOD_Pin GPIO_PIN_1
#define MCU_PMIC_PGOOD_GPIO_Port GPIOA
#define MCU_PMIC_LDO_PGOOD_Pin GPIO_PIN_2
#define MCU_PMIC_LDO_PGOOD_GPIO_Port GPIOA
#define MCU_PMIC_nINT_Pin GPIO_PIN_3
#define MCU_PMIC_nINT_GPIO_Port GPIOA
#define MCU_PMIC_nWAKEUP_Pin GPIO_PIN_4
#define MCU_PMIC_nWAKEUP_GPIO_Port GPIOA
#define MCU_GPIO_PA5_Pin GPIO_PIN_5
#define MCU_GPIO_PA5_GPIO_Port GPIOA
#define MCU_GPIO_PA6_Pin GPIO_PIN_6
#define MCU_GPIO_PA6_GPIO_Port GPIOA
#define MCU_GPIO_PA7_Pin GPIO_PIN_7
#define MCU_GPIO_PA7_GPIO_Port GPIOA
#define MCU_GPIO_PA8_Pin GPIO_PIN_8
#define MCU_GPIO_PA8_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
