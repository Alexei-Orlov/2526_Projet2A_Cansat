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
#include "stm32g4xx_hal.h"

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
#define VBAT_ADC1_IN3_Pin GPIO_PIN_2
#define VBAT_ADC1_IN3_GPIO_Port GPIOA
#define HMIBTN_EXTI3_Pin GPIO_PIN_3
#define HMIBTN_EXTI3_GPIO_Port GPIOA
#define HMIBTN_EXTI3_EXTI_IRQn EXTI3_IRQn
#define BARO_EXTI_Pin GPIO_PIN_4
#define BARO_EXTI_GPIO_Port GPIOA
#define SD_SPI1_SCK_Pin GPIO_PIN_5
#define SD_SPI1_SCK_GPIO_Port GPIOA
#define SD_SPI1_MISO_Pin GPIO_PIN_6
#define SD_SPI1_MISO_GPIO_Port GPIOA
#define SD_SPI1_MOSI_Pin GPIO_PIN_7
#define SD_SPI1_MOSI_GPIO_Port GPIOA
#define SD_GPIO_CS_Pin GPIO_PIN_4
#define SD_GPIO_CS_GPIO_Port GPIOC
#define LED_GPIO_OUT_Pin GPIO_PIN_0
#define LED_GPIO_OUT_GPIO_Port GPIOB
#define SD_GPIO_DETECT_Pin GPIO_PIN_2
#define SD_GPIO_DETECT_GPIO_Port GPIOB
#define HMILED_GPIO_Pin GPIO_PIN_10
#define HMILED_GPIO_GPIO_Port GPIOB
#define LIDAR_USART3_RX_Pin GPIO_PIN_11
#define LIDAR_USART3_RX_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_12
#define LORA_NSS_GPIO_Port GPIOB
#define LORA_SPI2_SCK_Pin GPIO_PIN_13
#define LORA_SPI2_SCK_GPIO_Port GPIOB
#define LORA_SPI2_MISO_Pin GPIO_PIN_14
#define LORA_SPI2_MISO_GPIO_Port GPIOB
#define LORA_SPI2_MOSI_Pin GPIO_PIN_15
#define LORA_SPI2_MOSI_GPIO_Port GPIOB
#define MOTOR_TIM3_CH1_Pin GPIO_PIN_6
#define MOTOR_TIM3_CH1_GPIO_Port GPIOC
#define IMU_I2C3_SCL_Pin GPIO_PIN_8
#define IMU_I2C3_SCL_GPIO_Port GPIOA
#define STLINK_GPS_USART1_TX_Pin GPIO_PIN_9
#define STLINK_GPS_USART1_TX_GPIO_Port GPIOA
#define STLINK_GPS_USART1_RX_Pin GPIO_PIN_10
#define STLINK_GPS_USART1_RX_GPIO_Port GPIOA
#define BARO_I2C1_SCL_Pin GPIO_PIN_15
#define BARO_I2C1_SCL_GPIO_Port GPIOA
#define LIDAR_USART3_TX_Pin GPIO_PIN_10
#define LIDAR_USART3_TX_GPIO_Port GPIOC
#define IMU_I2C3_SDA_Pin GPIO_PIN_11
#define IMU_I2C3_SDA_GPIO_Port GPIOC
#define LORA_RST_Pin GPIO_PIN_4
#define LORA_RST_GPIO_Port GPIOB
#define VPOWER_EN_GPIO_OUT_Pin GPIO_PIN_5
#define VPOWER_EN_GPIO_OUT_GPIO_Port GPIOB
#define BARO_I2C1_SDA_Pin GPIO_PIN_9
#define BARO_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
