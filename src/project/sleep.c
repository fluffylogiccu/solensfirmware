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
#include "log.h"
#include "cmd.h"
#include <stdint.h>
#include <time.h>

// Wakup time
static time_t sleep_alarmTime;

/**************************************
 * Private functions
 */

/*sleep_status_t sleep_timeInit() {

    sleep_alarmTime = esp8266_GetTime();
    struct tm *info;
    info = localtime(&sleep_alarmTime);

    return SLEEP_INFO_OK;
}*/

sleep_status_t sleep_rtcInit() {

    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_InitTypeDef RTC_InitStructure;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure2;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    log_Log(SLEEP, SLEEP_INFO_OK, "RTC initialization.\0");

    RTC_ClearITPendingBit(RTC_IT_WUT);
    EXTI_ClearITPendingBit(EXTI_Line22);

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset RTC Domain */
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);

    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    log_Log(SLEEP, SLEEP_INFO_OK, "LSE clock ready.\0");

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

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

    sleep_alarmTime = ntptime + 30;
    struct tm *info_next;
    info_next = localtime(&sleep_alarmTime);

    /* Disable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    /* go back to bed */
    if (info_next->tm_hour > 23 || info_next->tm_hour < 7) {
        /* Set the alarm to the morning */
        RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
        RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 0;
        RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 10;
        RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 10;
        RTC_AlarmStructure.RTC_AlarmDateWeekDay = info_next->tm_wday;
        RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
        RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours;

        /* Configure the RTC Alarm A register */
        RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

        /* Enable RTC Alarm A Interrupt */
        RTC_ITConfig(RTC_IT_ALRA, ENABLE);

        /* Enable the alarm */
        RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

        RTC_ClearFlag(RTC_FLAG_ALRAF);

        sleep_Standby();

    } else {

        /* Set the alarm for every minute at 0 sec */
        RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
        RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = info_next->tm_hour;
        RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = info_next->tm_min;
        RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0; //info_next->tm_sec;
        RTC_AlarmStructure.RTC_AlarmDateWeekDay = info_next->tm_wday;
        RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
        RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;

        /* Set the alarm for every minute at 30 sec */
        RTC_AlarmStructure2.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
        RTC_AlarmStructure2.RTC_AlarmTime.RTC_Hours   = info_next->tm_hour;
        RTC_AlarmStructure2.RTC_AlarmTime.RTC_Minutes = info_next->tm_min;
        RTC_AlarmStructure2.RTC_AlarmTime.RTC_Seconds = 30; //info_next->tm_sec;
        RTC_AlarmStructure2.RTC_AlarmDateWeekDay = info_next->tm_wday;
        RTC_AlarmStructure2.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
        RTC_AlarmStructure2.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;
    }

    /* Configure the RTC Alarm A register */
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    /* Enable RTC Alarm A Interrupt */
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    /* Enable the alarm */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

    RTC_ClearFlag(RTC_FLAG_ALRAF);

    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_B, &RTC_AlarmStructure2);

    /* Enable RTC Alarm A Interrupt */
    RTC_ITConfig(RTC_IT_ALRB, ENABLE);

    /* Enable the alarm */
    RTC_AlarmCmd(RTC_Alarm_B, ENABLE);

    RTC_ClearFlag(RTC_FLAG_ALRBF);

    return SLEEP_INFO_OK;
}

/**************************************
 * Public functions
 */

sleep_status_t sleep_Init() {
    sleep_rtcInit();
    //sleep_timeInit();

    return SLEEP_INFO_OK;
}

sleep_status_t sleep_Standby() {

    log_Log(SLEEP, SLEEP_INFO_OK, "Preparing to enter standby mode.\0");

    /* Clear WakeUp (WUTF) pending flag */
    RTC_ClearFlag(RTC_FLAG_ALRAF);
    RTC_ClearFlag(RTC_FLAG_ALRBF);
    PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);

    /* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
    PWR_EnterSTANDBYMode();

    // Should never enter this case
    return SLEEP_ERR_SLEEP;
}
