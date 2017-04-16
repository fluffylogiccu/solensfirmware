/** @file sleep.c
 *  @brief Implemenation of the sleep module.
 *
 *  This contains the implementations of the
 *  sleep file functions.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "err.h"
#include "sleep.h"
#include <stdint.h>
#include <time.h>

/**************************************
 * Private functions
 */

sleep_status_t sleep_rtcInit() {

    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_InitTypeDef RTC_InitStructure;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset RTC Domain */
    //RCC_BackupResetCmd(ENABLE);
    //RCC_BackupResetCmd(DISABLE);

    /* Enable the LSE OSC */
    //RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    //while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    //{
    //}

    /* Select the RTC Clock Source */
    //RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div31);

    /* Enable the RTC Clock */
     RCC_RTCCLKCmd(ENABLE);

     /* Wait for RTC APB registers synchronisation */
     RTC_WaitForSynchro();

    /* Configure the RTC data register and RTC prescaler */
    /* ck_spre(1Hz) = RTCCLK(LSI) /(AsynchPrediv + 1)*(SynchPrediv + 1)*/
    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);

    /* Get time from network */
    time_t ntptime = esp8266_GetTime();
    struct tm *info;
    info = localtime(&ntptime);

    time_t ntptime_next = ntptime + 10;
    struct tm *info_next;
    info_next = localtime(&ntptime_next);

    /* Set the alarm 05h:20min:30s */
    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 0;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 0;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0x10;
    RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
    RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;

    /* Configure the RTC Alarm A register */
    RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);

    /* Enable RTC Alarm A Interrupt */
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    /* Enable the alarm */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

    RTC_ClearFlag(RTC_FLAG_ALRAF);

    /* Set the time to 00h 00mn 00s AM */
    RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
    RTC_TimeStructure.RTC_Hours   = info->tm_hour;
    RTC_TimeStructure.RTC_Minutes = info->tm_min;
    RTC_TimeStructure.RTC_Seconds = info->tm_sec;
    RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

    RTC_DateStructure.RTC_Year = info->tm_year;
    RTC_DateStructure.RTC_Month = info->tm_mon;
    RTC_DateStructure.RTC_Date = info->tm_mday;
    RTC_DateStructure.RTC_WeekDay = info->tm_wday;
    RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);

    /* RTC Alarm A Interrupt Configuration */
    /* EXTI configuration *********************************************************/
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable the RTC Alarm Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return SLEEP_INFO_OK;
}

void RTC_Alarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);
  }
}

/**************************************
 * Public functions
 */

sleep_status_t sleep_Init() {
    sleep_rtcInit();


    return SLEEP_INFO_OK;

}
