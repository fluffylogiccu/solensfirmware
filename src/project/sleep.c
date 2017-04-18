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

sleep_status_t sleep_timeInit() {

    sleep_alarmTime = esp8266_GetTime();
    struct tm *info;
    info = localtime(&sleep_alarmTime);

    return SLEEP_INFO_OK;
}

sleep_status_t sleep_rtcInit() {

/*    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_InitTypeDef RTC_InitStructure;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure; */

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    log_Log(SLEEP, SLEEP_INFO_OK, "RTC INIT");

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


    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC Clock */
     RCC_RTCCLKCmd(ENABLE);

     /* Wait for RTC APB registers synchronisation */
     RTC_WaitForSynchro();

     /* EXTI configuration *******************************************************/
     /*EXTI_ClearITPendingBit(EXTI_Line22);
     EXTI_InitStructure.EXTI_Line = EXTI_Line22;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);

     /* Enable the RTC Wakeup Interrupt */
     /*NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);

     /* Enable Wakeup Counter */
     //RTC_WakeUpCmd(DISABLE);

#if 0
     RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
     RTC_SetWakeUpCounter(0xA000-1);

     /* Enable the Wakeup Interrupt */
     RTC_ITConfig(RTC_IT_WUT, ENABLE);

     /* Enable Wakeup Counter */
     RTC_WakeUpCmd(ENABLE);

     /* Clear WakeUp (WUTF) pending flag */
     RTC_ClearFlag(RTC_FLAG_WUTF);
     PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);

     /* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
     PWR_EnterSTANDBYMode();
#endif


    /* Configure the RTC data register and RTC prescaler */
    /* ck_spre(1Hz) = RTCCLK(LSI) /(AsynchPrediv + 1)*(SynchPrediv + 1)*/
/*    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);
*/
    /* Get time from network */
/*    time_t ntptime = esp8266_GetTime();
    struct tm *info;
    info = localtime(&ntptime);
*/
    /* Set the time to 00h 00mn 00s AM */
    /*RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
    RTC_TimeStructure.RTC_Hours   = info->tm_hour;
    RTC_TimeStructure.RTC_Minutes = info->tm_min;
    RTC_TimeStructure.RTC_Seconds = info->tm_sec;
    RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

    RTC_DateStructure.RTC_Year = info->tm_year;
    RTC_DateStructure.RTC_Month = info->tm_mon;
    RTC_DateStructure.RTC_Date = info->tm_mday;
    RTC_DateStructure.RTC_WeekDay = info->tm_wday;
    RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);

    sleep_alarmTime = ntptime + 60 - info->tm_sec;
    struct tm *info_next;
    info_next = localtime(&sleep_alarmTime);
*/
    /* RTC Wakeup Interrupt Generation: Clock Source: RTCCLK_Div16, Wakeup Time Base: ~20s
     RTC Clock Source LSE 32.768 kHz
     Wakeup Time Base = (16 / (LSE)) * WakeUpCounter
  */




#if 0

    /* Disable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    /* Set the alarm 05h:20min:30s */
    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = info_next->tm_hour;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = info_next->tm_min;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = info_next->tm_sec;
    RTC_AlarmStructure.RTC_AlarmDateWeekDay = info_next->tm_wday;
    RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;

    /* Configure the RTC Alarm A register */
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    RTC_TimeTypeDef time_temp;
    RTC_AlarmTypeDef alarm_temp;
    RTC_GetTime(RTC_Format_BIN, &time_temp);
    RTC_GetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm_temp);

    /* Enable RTC Alarm A Interrupt */
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    /* Enable the alarm */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

    RTC_ClearFlag(RTC_FLAG_ALRAF);

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
#endif

    return SLEEP_INFO_OK;
}

void RTC_WKUP_IRQHandler(void)
{
  log_Log(SLEEP, SLEEP_INFO_OK, "ABOUT TO RESET RCT_IT_WUT");
  if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
  {

    RTC_ClearITPendingBit(RTC_IT_WUT);
    EXTI_ClearITPendingBit(EXTI_Line22);
  }
}

#if 0
void RTC_Alarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);

    RTC_AlarmTypeDef RTC_AlarmStructure;

    /* Disable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    log_Log(SLEEP, SLEEP_INFO_OK, "30 second interrupt\0");

    sleep_alarmTime = sleep_alarmTime + 30;
    struct tm *info_next;
    info_next = localtime(&sleep_alarmTime);

    /* Disable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    /* Set the alarm 05h:20min:30s */
    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = info_next->tm_hour;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = info_next->tm_min;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = info_next->tm_sec;
    RTC_AlarmStructure.RTC_AlarmDateWeekDay = info_next->tm_wday;
    RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;

    /* Configure the RTC Alarm A register */
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    /* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
       STANDBY mode (RTC Alarm IT not enabled in NVIC) */
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    /* Enable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

    /* Clear RTC Alarm Flag */
    RTC_ClearFlag(RTC_FLAG_ALRAF);

    /* queue capture command */
    cmd_cmd_t *capture;
    cmd_status_t st = cmd_CmdAllocate(&capture, 0);
    if (st != CMD_INFO_OK) {
        log_Log(CMD, st, "Could not queue image capture command.\0");
    }
    capture->cmd_module = CAM;
    capture->cmd_func = CAM_FUNC_CAPTURE;
    capture->cmd_dataLen = 0;
    st = cmd_QueuePut(capture);
    if (st != CMD_INFO_OK) {
        log_Log(CMD, st, "Could not add command to queue.\0");
    }

    /* queue transfer command */
    cmd_cmd_t *transfer;
    st = cmd_CmdAllocate(&transfer, 0);
    if (st != CMD_INFO_OK) {
        log_Log(CMD, st, "Could not queue image transfer command.\0");
    }
    transfer->cmd_module = CAM;
    transfer->cmd_func = CAM_FUNC_TRANSFER;
    transfer->cmd_dataLen = 0;
    st = cmd_QueuePut(transfer);
    if (st != CMD_INFO_OK) {
        log_Log(CMD, st, "Could not add command to queue.\0");
    }

  }
}
#endif

/**************************************
 * Public functions
 */

sleep_status_t sleep_Init() {
    sleep_rtcInit();
    sleep_timeInit();

    return SLEEP_INFO_OK;
}

sleep_status_t sleep_Standby() {

    log_Log(SLEEP, SLEEP_INFO_OK, "Preparing to enter standby mode.\0");

    /* Enable Wakeup Counter */
    RTC_WakeUpCmd(DISABLE);

    RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
    RTC_SetWakeUpCounter(0xA000-1);

    /* Enable the Wakeup Interrupt */
    RTC_ITConfig(RTC_IT_WUT, ENABLE);

    /* Enable Wakeup Counter */
    RTC_WakeUpCmd(ENABLE);

    /* Clear WakeUp (WUTF) pending flag */
    RTC_ClearFlag(RTC_FLAG_WUTF);
    PWR_ClearFlag(PWR_FLAG_WU | PWR_FLAG_SB);

    /* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
    PWR_EnterSTANDBYMode();

    // Should never enter this case
    return SLEEP_ERROR_SLEEP;
}
