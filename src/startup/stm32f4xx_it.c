/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "log.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rtc.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "NMI HANDLER.\0");
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "HARD FAULT HANDLER.\0");
    log_Log(LOG, LOG_ERR_UNKNOWN, "Entering standby mode.\0");

    RTC_ClearFlag(RTC_FLAG_ALRAF);
    RTC_ClearFlag(RTC_FLAG_ALRBF);
    PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);
    PWR_EnterSTANDBYMode();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "MEMORY MANAGE HANDLER.\0");
    log_Log(LOG, LOG_ERR_UNKNOWN, "Entering standby mode.\0");

    RTC_ClearFlag(RTC_FLAG_ALRAF);
    RTC_ClearFlag(RTC_FLAG_ALRBF);
    PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);
    PWR_EnterSTANDBYMode();

  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "BUS FAULT HANDLER.\0");
    log_Log(LOG, LOG_ERR_UNKNOWN, "Entering standby mode.\0");

    RTC_ClearFlag(RTC_FLAG_ALRAF);
    RTC_ClearFlag(RTC_FLAG_ALRBF);
    PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);
    PWR_EnterSTANDBYMode();

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "USAGE FAULT HANDLER.\0");
    log_Log(LOG, LOG_ERR_UNKNOWN, "Entering standby mode.\0");

    RTC_ClearFlag(RTC_FLAG_ALRAF);
    RTC_ClearFlag(RTC_FLAG_ALRBF);
    PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);
    PWR_EnterSTANDBYMode();

  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "SCV HANDLER.\0");

}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "DEBUG MONITOR HANDLER.\0");
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "PENDSV HANDLER.\0");
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    log_Log(LOG, LOG_ERR_UNKNOWN, "SYSTICK HANDLER.\0");
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
