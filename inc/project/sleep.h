/** @file sleep.h
 *  @brief Function prototypes for the sleep system.
 *
 *  This contains the prototypes, macros, constants,
 *  and global variables for the sleep file.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __SLEEP_H
#define __SLEEP_H

/*************************************
 * Includes and definitions
 */

#include "err.h"
#include "esp8266.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_pwr.h"
#include <stdint.h>

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */

/** @brief Initialize sleep module
 *
 *  This function initializes the sleep module. It
 *  Initializes the RTC clock and gets the time from
 *  the NTP server. The wifi should be configured before
 *  calling this function.
 *
 *  @return a status code of type sleep_status_t
 */
sleep_status_t sleep_Init();

# endif /* __SLEEP_H */
