/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M3D_Pin GPIO_PIN_3
#define M3D_GPIO_Port GPIOE
#define M3P_Pin GPIO_PIN_5
#define M3P_GPIO_Port GPIOE
#define M4P_Pin GPIO_PIN_6
#define M4P_GPIO_Port GPIOE
#define M4D_Pin GPIO_PIN_14
#define M4D_GPIO_Port GPIOC
#define M2P_Pin GPIO_PIN_3
#define M2P_GPIO_Port GPIOA
#define M1P_Pin GPIO_PIN_5
#define M1P_GPIO_Port GPIOA
#define M1D_Pin GPIO_PIN_7
#define M1D_GPIO_Port GPIOE
#define EN1_CH1_Pin GPIO_PIN_9
#define EN1_CH1_GPIO_Port GPIOE
#define EN1_CH2_Pin GPIO_PIN_11
#define EN1_CH2_GPIO_Port GPIOE
#define M2D_Pin GPIO_PIN_14
#define M2D_GPIO_Port GPIOE
#define EN5_CH1_Pin GPIO_PIN_6
#define EN5_CH1_GPIO_Port GPIOC
#define EN5_CH2_Pin GPIO_PIN_7
#define EN5_CH2_GPIO_Port GPIOC
#define EN3_CH1_Pin GPIO_PIN_6
#define EN3_CH1_GPIO_Port GPIOB
#define EN3_CH2_Pin GPIO_PIN_7
#define EN3_CH2_GPIO_Port GPIOB
#define PWM_SERVO_Pin GPIO_PIN_8
#define PWM_SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
