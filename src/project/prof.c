/** @file prof.c
 *  @brief Implemenation of the profiler functions.
 *
 *  This contains the implementations of the profiler 
 *  functions. The main profiler function is defined as a 
 *  preprocessor macro in the prof.h file.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */
 
#ifdef __PROF
#include "prof.h"
#include "err.h"
#include "log.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include <stdint.h>

/* @brief Timer configuration parameters
 */
// 1 uS resolution
#define PROF_PRESCALE SystemCoreClock/1000000
// 100 second max
#define PROF_PERIOD 1000000*100

/* @brief flag to keep track of initialization
 */
static uint8_t prof_initialized = 0;
#endif

/**************************************
 * Private functions
 */

#ifdef __PROF
prof_status_t prof_itoa(uint8_t *str, uint32_t data) {
    uint8_t i = 0;
    uint8_t temp = 0;
    while (data > 0) {
        // find lowest digit, add ASCII base
        *(str+i) = data % 10 + '0';
        data /= 10;
        i++;
    }

    *(str+i) = '\0';
    // reverse string
    while(i/2 > 0) {
        temp = *str;
        *str = *(str+i-1);
        *(str++ + i-1) = temp;
        i -= 2;
    }

    return PROF_INFO_OK;
}

prof_status_t prof_concat(uint8_t *buf, uint8_t *str1, uint8_t *str2) {
    while(*str1 != '\0') {
        *buf++ = *str1++;
    }
    while(*str2 != '\0') {
        *buf++ = *str2++;
    }
    *buf = '\0';
    return PROF_INFO_OK;
}

prof_status_t prof_start() {
    TIM2->CNT = 0;
    return PROF_INFO_OK;
}

prof_status_t prof_stop(char *msg) {
    uint8_t mBuf[256];
    uint8_t *msgBuf = (uint8_t *) mBuf;
    uint8_t nBuf[6];
    uint8_t *numBuf = (uint8_t *) nBuf;
    uint32_t time = (uint32_t) TIM2->CNT;

    // Decide units to display
    if (time > 1000000000) {
        time /= 1000000;
        prof_itoa(numBuf, time);
        prof_concat(msgBuf, (uint8_t *) msg, (uint8_t *) "\t\0");
        prof_concat(msgBuf, msgBuf, numBuf);
        prof_concat(msgBuf, msgBuf, (uint8_t *) " s");
    } else if (time > 1000000) {
        time /= 1000;
        prof_itoa(numBuf, time);
        prof_concat(msgBuf, (uint8_t *) msg, (uint8_t *) "\t\0");
        prof_concat(msgBuf, msgBuf, numBuf);
        prof_concat(msgBuf, msgBuf, (uint8_t *) " ms\0");
    } else {
        prof_itoa(numBuf, time);
        prof_concat(msgBuf, (uint8_t *) msg, (uint8_t *) "\t\0");
        prof_concat(msgBuf, msgBuf, numBuf);
        prof_concat(msgBuf, msgBuf, (uint8_t *) " us\0");
    }

    log_Log(PROF, PROF_INFO_RESULTS, (char *) msgBuf);

    return PROF_INFO_OK;
}

#endif

/**************************************
 * Public functions
 */

#ifdef __PROF
prof_status_t prof_Init() {
    if (prof_initialized == 1) {
        return PROF_WARN_ALINIT;
    }

    TIM_TimeBaseInitTypeDef timeInit;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    timeInit.TIM_Prescaler = PROF_PRESCALE;
    timeInit.TIM_CounterMode = TIM_CounterMode_Up;
    timeInit.TIM_Period = PROF_PERIOD;
    timeInit.TIM_ClockDivision = TIM_CKD_DIV1;
    timeInit.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timeInit);

    TIM_Cmd(TIM2, ENABLE);

    // Initialization flag
    prof_initialized = 1;

    return PROF_INFO_OK;
}

/* Preprocessor macro prof_Profile(x) defined in prof.h
 * This will be the main profiler function called by
 * the user.
 */
// #define prof_Prof()... 
#endif

