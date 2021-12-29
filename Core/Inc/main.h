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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
/* USER CODE BEGIN Private defines */
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define LED_G8_Pin GPIO_PIN_8
#define LED_G8_GPIO_Port GPIOG
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define LED_G7_Pin GPIO_PIN_7
#define LED_G7_GPIO_Port GPIOG
#define LED_G6_Pin GPIO_PIN_6
#define LED_G6_GPIO_Port GPIOG
#define MPU_SPI_NSS_Pin GPIO_PIN_6
#define MPU_SPI_NSS_GPIO_Port GPIOF
#define LED_G5_Pin GPIO_PIN_5
#define LED_G5_GPIO_Port GPIOG
#define LED_G4_Pin GPIO_PIN_4
#define LED_G4_GPIO_Port GPIOG
#define LED_G3_Pin GPIO_PIN_3
#define LED_G3_GPIO_Port GPIOG
#define LED_G2_Pin GPIO_PIN_2
#define LED_G2_GPIO_Port GPIOG
#define LED_G1_Pin GPIO_PIN_1
#define LED_G1_GPIO_Port GPIOG
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOH
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOE
#define BUTTON_TRIG_Pin GPIO_PIN_0
#define BUTTON_TRIG_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOF
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
