/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "main.h"
#include "Task_Main.h"
#include "Uart.h"

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  
  while (1)
  {
    
  }
  
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  
  while (1)
  {
    
  }
  
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  
  while (1)
  {
    
  }
  
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  
  while (1)
  {
    
  }
  
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
    SystemTickCount_ms++;
    SystemBlink_ms++;
    if (SystemBlink_ms >= 1000)
    {
        SystemBlink_ms = 0;
        LED1_TG;
    }
    if (reset == 1)
    {
        if (timer_tick > 0)    timer_tick--;
    }
    if (opCheck_Flag == 1)
    {
        if (opTimecheck > 0)    opTimecheck--;                
    }
    if (ocCheck_flag)
        ocTimer++;
    
    if (++msgTick_ms >= 1000)
    {
        msgTick_ms = 0;
        SendStatus();
    }
    
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();  
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  
  HAL_TIM_IRQHandler(&htim2);
  
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  
}

extern uint32_t encoder_cnt;
extern long int enc_A , enc_B , enc_Z;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
       //in2++;
        //encoder_cnt++;
        if (ENC_B == 1)
        {
            // reverse direction as encoder is fixed in opposite direction
            enc_A--;
        }
        else
        {
            enc_A++;
        }
    }
    /*
    if(GPIO_Pin == GPIO_PIN_3)
    {
       in3++;
    }
    if(GPIO_Pin == GPIO_PIN_4)
    {
       in4++;
    }
    */
}

// Duty cycle %
#define MAX_DUTY        80

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2)
    {
        if (duty_change == 1)
        {
            duty_change  = 0;
            if (dutycycle < MAX_DUTY)
            {
                dutycycle = dutycycle + 10;                               
                uint16_t dutypulse = (htim->Init.Period*dutycycle)/100 - 1;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutypulse);
            }
        }
    }    
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
