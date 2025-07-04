/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "user_main.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// This is left here just to have it handy for copying when debugging a specific function
// Don't uncomment!!
//               __attribute__((optimize("O0")))

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// STM32CUBE IDE removed line printing in Error Handler. This macro restores it.
// Credits: https://community.st.com/s/question/0D50X00009XkffVSAR/stm32cubemx-v421-errorhandler-definition-issues-in-mainh
#ifdef DEBUG_ERROR
    #define GET_MACRO( _0, _1, NAME, ... ) NAME
    #define Error_Handler(...) GET_MACRO( _0, ##__VA_ARGS__, Error_Handler1, Error_Handler0 )()
    #define Error_Handler0() _Error_Handler(__BASE_FILE__, __LINE__ )
    #define Error_Handler1(unused) _Error_Handler( char * file, int line )
    void _Error_Handler(char *, int);
#endif

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EE_SCL_Pin GPIO_PIN_0
#define EE_SCL_GPIO_Port GPIOA
#define EE_SDA_Pin GPIO_PIN_1
#define EE_SDA_GPIO_Port GPIOA
#define TIP_Pin GPIO_PIN_2
#define TIP_GPIO_Port GPIOA
#define BUZ0_Pin GPIO_PIN_3
#define BUZ0_GPIO_Port GPIOA
#define BUZ1_Pin GPIO_PIN_4
#define BUZ1_GPIO_Port GPIOA
#define BUZ2_Pin GPIO_PIN_5
#define BUZ2_GPIO_Port GPIOA
#define WAKE_Pin GPIO_PIN_6
#define WAKE_GPIO_Port GPIOA
#define NTC_Pin GPIO_PIN_7
#define NTC_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_0
#define PWM_GPIO_Port GPIOB
#define VIN_Pin GPIO_PIN_1
#define VIN_GPIO_Port GPIOB
#define HW_SCL_Pin GPIO_PIN_10
#define HW_SCL_GPIO_Port GPIOB
#define HW_SDA_Pin GPIO_PIN_11
#define HW_SDA_GPIO_Port GPIOB
#define SW_SDA_Pin GPIO_PIN_12
#define SW_SDA_GPIO_Port GPIOB
#define SW_SCL_Pin GPIO_PIN_13
#define SW_SCL_GPIO_Port GPIOB
#define ENC_SW_Pin GPIO_PIN_15
#define ENC_SW_GPIO_Port GPIOA
#define ENC_R_Pin GPIO_PIN_3
#define ENC_R_GPIO_Port GPIOB
#define ENC_L_Pin GPIO_PIN_4
#define ENC_L_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
