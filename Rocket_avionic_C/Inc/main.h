/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define sl_red_Pin GPIO_PIN_4
#define sl_red_GPIO_Port GPIOA
#define sl_yellow_Pin GPIO_PIN_5
#define sl_yellow_GPIO_Port GPIOA
#define bmp280_scl_Pin GPIO_PIN_10
#define bmp280_scl_GPIO_Port GPIOB
#define bmp280_sda_Pin GPIO_PIN_11
#define bmp280_sda_GPIO_Port GPIOB
#define user_button_Pin GPIO_PIN_12
#define user_button_GPIO_Port GPIOB
#define lora_tx_Pin GPIO_PIN_9
#define lora_tx_GPIO_Port GPIOA
#define lora_rx_Pin GPIO_PIN_10
#define lora_rx_GPIO_Port GPIOA
#define deployment_1_Pin GPIO_PIN_3
#define deployment_1_GPIO_Port GPIOB
#define deployment_2_Pin GPIO_PIN_4
#define deployment_2_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_5
#define buzzer_GPIO_Port GPIOB
#define eeprom_scl_Pin GPIO_PIN_6
#define eeprom_scl_GPIO_Port GPIOB
#define eeprom_sda_Pin GPIO_PIN_7
#define eeprom_sda_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
