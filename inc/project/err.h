/** @file err.h
 *  @brief Definitions for errors,
 *  warnings, and status.
 *
 *  This contains the definitions for errors, warnings, and
 *  status for every module. INFO enum values are reserved
 *  as 0-19, WARN enum values are reserved as 20-39, and
 *  ERR enum values are reserved as 40-59.
 *
 *  This file also handles the standard peripheral library
 *  assertion failure by logging the failure to the debug
 *  interface.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __ERR_H
#define __ERR_H

/**************************************
 * @name Includes and definitions
 */

#include <stdint.h>

/* @brief Error sections, INFO is 0-19,
 * WARN is 20-39, ERR is 40-59
 */
#define INFO 0
#define WARN 20
#define ERR  40
#define END  60

/* @brief General status type for module status typecast.
 */
typedef uint8_t gen_status_t;

/* @brief Log module status
 */
typedef enum log_status_e {
    LOG_INFO_OK = INFO,
    LOG_INFO_UNKNOWN = WARN-1,

    LOG_WARN_ALINIT = WARN,
    LOG_WARN_IGNORED = WARN+1,
    LOG_WARN_UNKNOWN = ERR-1,

    LOG_ERR_DATASIZE = ERR,
    LOG_ERR_MSGSIZE = ERR+1,
    LOG_ERR_LOGOFF = ERR+2,
    LOG_ERR_UNKNOWN = END-1
} log_status_t;

/* @brief CMD module status
 */
typedef enum cmd_status_e {
    CMD_INFO_OK = INFO,
    CMD_INFO_INTERRUPT = INFO+1,
    CMD_INFO_QUEUEEMPTY = INFO+2,
    CMD_INFO_QUEUEFULL = INFO+3,
    CMD_INFO_QUEUEPARTIAL = INFO+4,
    CMD_INFO_UNKNOWN = WARN-1,

    CMD_WARN_FREE = WARN,
    CMD_WARN_ALINIT = WARN+1,
    CMD_WARN_UNKNOWN = ERR-1,

    CMD_ERR_QUEUEEMPTY = ERR,
    CMD_ERR_QUEUEFULL = ERR+1,
    CMD_ERR_QUEUEINVALID = ERR+2,
    CMD_ERR_MALLOC = ERR+3,
    CMD_ERR_NULLPTR = ERR+4,
    CMD_ERR_DATA = ERR+5,
    CMD_ERR_NOFUNC = ERR+6,
    CMD_ERR_NOMOD = ERR+7,
    CMD_ERR_UNKNOWN = END-1,
} cmd_status_t;

/* @brief STDLIB status
 */
typedef enum stdlib_status_e {
    STDLIB_INFO_OK = INFO,
    STDLIB_INFO_UNKNOWN = WARN-1,

    STDLIB_WARN_UNKNOWN = ERR-1,

    STDLIB_ERR_UNKNOWN = END-1,
} stdlib_status_e;

/* @brief SDRAM status
 */
typedef enum sdram_status_e {
    SDRAM_INFO_OK = INFO,
    SDRAM_INFO_UNKNOWN = WARN-1,

    SDRAM_WARN_ALINIT = WARN,
    SDRAM_WARN_UNKNOWN = ERR-1,

    SDRAM_ERR_UNKNOWN = END-1,
} sdram_status_t;

/* @brief OV5642 status
 */
typedef enum ov5642_status_e {
    OV5642_INFO_OK = INFO,
    OV5642_INFO_IMAGE = INFO+1,
    OV5642_INFO_UNKNOWN = WARN-1,

    OV5642_WARN_ALINIT = WARN,
    OV5642_WARN_UNKNOWN = ERR-1,

    OV5642_ERR_I2CSTART = ERR,
    OV5642_ERR_I2CREAD = ERR+1,
    OV5642_ERR_I2CWRITE = ERR+2,
    OV5642_ERR_I2CTIMEOUT = ERR+3,
    OV5642_ERR_UNKNOWN = END-1,
} ov5642_status_t;

/* @brief OV7670 status
 */
typedef enum ov7670_status_e {
    OV7670_INFO_OK = INFO,
    OV7670_INFO_IMAGE = INFO+1,
    OV7670_INFO_UNKNOWN = WARN-1,

    OV7670_WARN_ALINIT = WARN,
    OV7670_WARN_UNKNOWN = ERR-1,

    OV7670_ERR_I2CSTART = ERR,
    OV7670_ERR_I2CREAD = ERR+1,
    OV7670_ERR_I2CWRITE = ERR+2,
    OV7670_ERR_I2CTIMEOUT = ERR+3,
    OV7670_ERR_UNKNOWN = END-1,
} ov7670_status_t;

/* @brief Profiler status
 */
typedef enum prof_status_e {
    PROF_INFO_OK = INFO,
    PROF_INFO_RESULTS = INFO+1,
    PROF_INFO_UNKNOWN = WARN-1,

    PROF_WARN_ALINIT = WARN,
    PROF_WARN_UNKNOWN = ERR-1,

    PROF_ERR_UNKNOWN = END-1,
} prof_status_t;

/* @brief Profiler status
 */
typedef enum test_status_e {
    TEST_INFO_OK = INFO,
    TEST_INFO_PASSED = INFO+1,
    TEST_INFO_CONFIRM = INFO+2,
    TEST_INFO_UNKNOWN = WARN-1,

    TEST_WARN_FAILED = WARN,
    TEST_WARN_UNKNOWN = ERR-1,

    TEST_ERR_UNKNOWN = END-1,
} test_status_t;

/* @brief Camera status
 */
typedef enum cam_status_e {
    CAM_INFO_OK = INFO,
    CAM_INFO_IMAGE = INFO+1,

    CAM_INFO_UNKNOWN = WARN-1,

    CAM_WARN_ALINIT = WARN,
    CAM_WARN_ALCONF = WARN+1,
    CAM_WARN_UNKNOWN = ERR-1,

    CAM_ERR_INIT = ERR,
    CAM_ERR_CONFIG = ERR+1,
    CAM_ERR_CAPTURE = ERR+2,
    CAM_ERR_TRANSFER = ERR+3,
    CAM_ERR_UNKNOWN = END-1,
} cam_status_t;

/* @brief ESP8266 status
 */
typedef enum esp8266_status_e {
    ESP8266_INFO_OK = INFO,
    ESP8266_INFO_UNKNOWN = WARN-1,

    ESP8266_WARN_UNKNOWN = ERR-1,

    ESP8266_ERR_UNKNOWN = END-1,
} esp8266_status_t;

/* @brief wifi status
 */
typedef enum wifi_status_e {
    WIFI_INFO_OK = INFO,
    WIFI_INFO_IDLE = INFO+1,
    WIFI_INFO_CONNECTING = INFO+2,
    WIFI_INFO_GOT_IP = INFO+3,
    WIFI_INFO_UNKNOWN = WARN-1,

    WIFI_WARN_ALINIT = WARN,
    WIFI_WARN_WRONG_PASSWORD = WARN+1,
    WIFI_WARN_NO_AP_FOUND = WARN+2,
    WIFI_WARN_UNKNOWN = ERR-1,

    WIFI_ERR_DATASIZE = ERR,
    WIFI_ERR_MSGSIZE = ERR+1,
    WIFI_ERR_SEND = ERR+2,
    WIFI_ERR_INIT = ERR+3,
    WIFI_ERR_CONNECT_FAIL = ERR+4,
    WIFI_ERR_UNKNOWN = END-1,
} wifi_status_t;

/* @brief sleep status
 */
typedef enum sleep_status_e {
    SLEEP_INFO_OK = INFO,
    SLEEP_INFO_UNKNOWN = WARN-1,

    SLEEP_WARN_UNKNOWN = ERR-1,

    SLEEP_ERR_SLEEP = ERR,
    SLEEP_ERR_UNKNOWN = END-1,
} sleep_status_t;

/**************************************
 * @name Public functions
 */

#ifdef  USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the
 * source line number where the assert_param error has
 * occured. Used by the STM32f4xx standard peripheral
 * library to report issues.
 *
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    log_Log(STDLIB, STDLIB_ERR_UNKNOWN, file, 4, &line);
}
#endif

# endif /* __ERR_H */
