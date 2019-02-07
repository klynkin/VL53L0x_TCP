/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PE2_Pin GPIO_PIN_2
#define PE2_GPIO_Port GPIOE
#define PE3_Pin GPIO_PIN_3
#define PE3_GPIO_Port GPIOE
#define PE4_Pin GPIO_PIN_4
#define PE4_GPIO_Port GPIOE
#define E1_DIR_Pin GPIO_PIN_13
#define E1_DIR_GPIO_Port GPIOC
#define PF6_Pin GPIO_PIN_6
#define PF6_GPIO_Port GPIOF
#define RELAY_1_Pin GPIO_PIN_10
#define RELAY_1_GPIO_Port GPIOF
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define E2_HEAT_Pin GPIO_PIN_0
#define E2_HEAT_GPIO_Port GPIOB
#define BED_HEAT_Pin GPIO_PIN_1
#define BED_HEAT_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_11
#define RELAY_2_GPIO_Port GPIOF
#define E2_RESET_Pin GPIO_PIN_12
#define E2_RESET_GPIO_Port GPIOF
#define E2_DIR_Pin GPIO_PIN_13
#define E2_DIR_GPIO_Port GPIOF
#define Z_DIR_Pin GPIO_PIN_14
#define Z_DIR_GPIO_Port GPIOF
#define Y_DIR_Pin GPIO_PIN_15
#define Y_DIR_GPIO_Port GPIOF
#define LCD_D5_Pin GPIO_PIN_0
#define LCD_D5_GPIO_Port GPIOG
#define LCD_D6_Pin GPIO_PIN_1
#define LCD_D6_GPIO_Port GPIOG
#define USER_KEY_Pin GPIO_PIN_7
#define USER_KEY_GPIO_Port GPIOE
#define USER_KEY_EXTI_IRQn EXTI9_5_IRQn
#define USER_1_Pin GPIO_PIN_8
#define USER_1_GPIO_Port GPIOE
#define USER_2_Pin GPIO_PIN_9
#define USER_2_GPIO_Port GPIOE
#define E1_RESET_Pin GPIO_PIN_10
#define E1_RESET_GPIO_Port GPIOE
#define Z_RESET_Pin GPIO_PIN_11
#define Z_RESET_GPIO_Port GPIOE
#define Y_RESET_Pin GPIO_PIN_12
#define Y_RESET_GPIO_Port GPIOE
#define X_RESET_Pin GPIO_PIN_13
#define X_RESET_GPIO_Port GPIOE
#define X_DIR_Pin GPIO_PIN_15
#define X_DIR_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_2
#define LCD_D7_GPIO_Port GPIOG
#define SD2_DET_Pin GPIO_PIN_3
#define SD2_DET_GPIO_Port GPIOG
#define SD_DET_Pin GPIO_PIN_6
#define SD_DET_GPIO_Port GPIOG
#define USB_POWER_Pin GPIO_PIN_7
#define USB_POWER_GPIO_Port GPIOG
#define BEEPER_Pin GPIO_PIN_8
#define BEEPER_GPIO_Port GPIOG
#define FAN_1_Pin GPIO_PIN_6
#define FAN_1_GPIO_Port GPIOC
#define E1_HEAT_Pin GPIO_PIN_7
#define E1_HEAT_GPIO_Port GPIOC
#define UART2_CTS_Pin GPIO_PIN_3
#define UART2_CTS_GPIO_Port GPIOD
#define UART2_RTS_Pin GPIO_PIN_4
#define UART2_RTS_GPIO_Port GPIOD
#define ZERO_Pin GPIO_PIN_7
#define ZERO_GPIO_Port GPIOD
#define X_STOP_Pin GPIO_PIN_9
#define X_STOP_GPIO_Port GPIOG
#define X_STOP_EXTI_IRQn EXTI9_5_IRQn
#define U_STOP_Pin GPIO_PIN_10
#define U_STOP_GPIO_Port GPIOG
#define U_STOP_EXTI_IRQn EXTI15_10_IRQn
#define Y_STOP_Pin GPIO_PIN_11
#define Y_STOP_GPIO_Port GPIOG
#define Y_STOP_EXTI_IRQn EXTI15_10_IRQn
#define V_STOP_Pin GPIO_PIN_12
#define V_STOP_GPIO_Port GPIOG
#define V_STOP_EXTI_IRQn EXTI15_10_IRQn
#define Z_STOP_Pin GPIO_PIN_13
#define Z_STOP_GPIO_Port GPIOG
#define Z_STOP_EXTI_IRQn EXTI15_10_IRQn
#define I2C1_WP_Pin GPIO_PIN_15
#define I2C1_WP_GPIO_Port GPIOG
#define SPI2_NSS_Pin GPIO_PIN_9
#define SPI2_NSS_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_0
#define USER_LED_GPIO_Port GPIOE
#define FLAG_Pin GPIO_PIN_1
#define FLAG_GPIO_Port GPIOE
#define FLAG_EXTI_IRQn EXTI1_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
