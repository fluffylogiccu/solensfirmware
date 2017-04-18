/* @file main.c
 *  @brief This file contains the main routine.
 *
 *  This initializes the logger, command interface,
 *  SDRAM, and starts the main command loop based on
 *  compile-time flags.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "err.h"
#include "log.h"
#ifdef __PROF
#include "prof.h"
#endif
#include "cmd.h"
#include "sdram.h"
#ifdef __TEST
#include "test.h"
#endif
#ifdef __WIFI
#include "wifi.h"
#endif
#ifdef __ESP8266
#include "esp8266.h"
#endif
#include "cam.h"
#include "sleep.h"

#include <stdint.h>

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */

int main() {

    // Run optional tests before initialization
    #ifdef __TEST
    #ifdef __LOG
        test_log();
    #endif

    #ifdef __PROF
        test_prof();
    #endif
    #endif

    #ifdef __LOG
    // Optional logger
    log_status_t l_st = log_Init()  ;
    if (l_st == LOG_INFO_OK) {
        log_Log(LOG, LOG_INFO_OK, "Initialized logger.\0");
    } else if (l_st == LOG_WARN_ALINIT) {
        log_Log(LOG, LOG_WARN_ALINIT, "Already initialized logger.\0");
    } else {
        return -1;
    }
    #endif

    // Need command module for scheduling even without user commands
    cmd_status_t cm_st = cmd_Init();
    if (cm_st == CMD_INFO_OK) {
        log_Log(CMD, CMD_INFO_OK, "Initialized command module.\0");
    } else if (cm_st == CMD_WARN_ALINIT) {
        log_Log(LOG, CMD_WARN_ALINIT, "Already initialized logger.\0");
    } else {
        return -2;
    }

    #ifdef __PROF
    // Optional profiler
    prof_status_t p_st = prof_Init();
    if (p_st == PROF_INFO_OK) {
        log_Log(PROF, PROF_INFO_OK, "Initialized profiler.\0");
    } else if (p_st == PROF_WARN_ALINIT) {
        log_Log(PROF, p_st, "Profiler already initialized.\0");
    } else {
        log_Log(PROF, p_st, "Could not initialize profiler.\0");
    }
    #endif

    // Initialize SDRAM always
    sdram_status_t sd_st = sdram_Init();
    if (sd_st == SDRAM_INFO_OK) {
        log_Log(SDRAM, SDRAM_INFO_OK, "Initialized SDRAM interface.\0");
    } else if (sd_st == SDRAM_WARN_ALINIT) {
        log_Log(SDRAM, sd_st, "Already initialized SDRAM.\0");
    } else {
        log_Log(SDRAM, sd_st, "Could not initialize SDRAM.\0");
    }

    // Initialize camera always
    cam_status_t cam_st = cam_Init();
    if (cam_st == CAM_INFO_OK) {
        log_Log(CAM, CAM_INFO_OK, "Initialized camera module.\0");
    } else if (cam_st == CAM_WARN_ALINIT) {
        log_Log(CAM, CAM_WARN_ALINIT, "Camera already initialized.\0");
    } else {
        log_Log(CAM, cam_st, "Could not initialize camera module.\0");
    }

    cam_st = cam_Configure();
    if (cam_st == CAM_INFO_OK) {
        log_Log(CAM, CAM_INFO_OK, "Configured camera module.\0");
    } else {
        log_Log(CAM, cam_st, "Could not initialize camera module.\0");
    }

    #ifdef __WIFI
    wifi_status_t w_st = wifi_Init();
    if (w_st == WIFI_INFO_OK) {
        log_Log(WIFI, WIFI_INFO_OK, "Initialized wifi module.\0");
    } else if (w_st == WIFI_WARN_ALINIT) {
        log_Log(WIFI, WIFI_WARN_ALINIT, "Wifi module already initialized.\0");
    } else {
        log_Log(WIFI, w_st, "Could not initialize wifi module.\0");
    }
    #endif

    sleep_status_t sl_st = sleep_Init();
    if (sl_st == SLEEP_INFO_OK) {
        log_Log(SLEEP, SLEEP_INFO_OK, "Initialized sleep module.\0");
    } else {
        log_Log(SLEEP, sl_st, "Could not initialize sleep module.\0");
    }

    /* Capture and transfor */
    cam_st = cam_Capture();
    if (cam_st == CAM_INFO_OK) {
        log_Log(CAM, CAM_INFO_OK, "Captured image.\0");
    } else {
        log_Log(CAM, cam_st, "Could not capture image.\0");
    }

    cam_st = cam_Transfer();
    if (cam_st == CAM_INFO_OK) {
        log_Log(CAM, CAM_INFO_OK, "Transferred image.\0");
    } else {
        log_Log(CAM, cam_st, "Could not transfer image.\0");
    }

    /* Go to sleep */
    sl_st = sleep_Standby();
    if (sl_st == SLEEP_INFO_OK) {
        log_Log(SLEEP, SLEEP_INFO_OK, "Successful sleep.\0");
    } else {
        log_Log(SLEEP, sl_st, "Could not enter standby mode.\0");
    }

    // Start main loop, shouldn't occur in normal operation
    cmd_status_t cmd_st = cmd_Loop();
    if (cmd_st != CMD_INFO_OK) {
        log_Log(CMD, cmd_st, "Exiting main.\0");
        return -3;
    }

    return 0;
}
