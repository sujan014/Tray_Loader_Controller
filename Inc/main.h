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
  * Copyright (c) 2021 STMicroelectronics International N.V. 
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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

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

#include "stm32f4xx_hal.h"
#include "Uart.h"
#include <stdarg.h>

#define MSG1    Uart1_Printf
#define MSG2    Uart2_Printf

#define MANUAL_RESET     0
#define MANUAL_SET          1
#define MANUAL_UP           1
#define MANUAL_DOWN      2

#define LED1_ON     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED1_OFF     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define LED1_TG     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12)

#define LED2_ON     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED2_OFF     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED2_TG     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

#define BUZZER(x)       (x == 1)?HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, SET):HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, RESET)
#define BUZZER_TG       HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_4)

#define DOOR_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11)
#define LIM1_IN         HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10)
#define LIM2_IN     HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8)
#define IR1_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)
#define IR2_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1)
#define ECT1_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12)
#define ECT2_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13)
#define PHOTO1_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)
#define PHOTO2_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3)
#define CYLIN1_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4)
#define CYLIN2_IN     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5)

#define ENC_B     HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3)
#define ENC_Z     HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4)

#define MOTOR_UP            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define MOTOR_DOWN      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

void TImer2_Interrupt_Init(void);

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
