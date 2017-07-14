/** @file cam.c
 *  @brief Implemenation of the camera functions.
 *
 *  This contains the implementations of the
 *  camera functions. Different camera drivers are
 *  selected based on compiler directives.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "cam.h"
#include "err.h"
#include "log.h"
#include "sdram.h"
#include "stm32f4xx_gpio.h"
#ifdef __OV7670
#include "ov7670.h"
#endif
#ifdef __OV5642
#include "ov5642.h"
#endif
#ifdef __WIFI
#include "wifi.h"
#endif
#include <stdint.h>
#include "string.h"
#include "stdio.h"

/* @brief Initialization flag
 */
static uint8_t cam_initialized = 0;

/* @brief Configuration flag
 */
static uint8_t cam_configured = 0;

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */

cam_status_t cam_Init() {
    // Check if initialized
    if (cam_initialized == 1) {
        return CAM_WARN_ALINIT;
    }

    #ifdef __OV7670
    ov7670_status_t st = ov7670_Init();
    if (st == OV7670_INFO_OK) {
        cam_initialized = 1;
        return CAM_INFO_OK;
    } else {
        return CAM_ERR_INIT;
    }
    #endif

    #ifdef __OV5642
    ov5642_status_t st = ov5642_Init();
    if (st == OV5642_INFO_OK) {
        cam_initialized = 1;
        return CAM_INFO_OK;
    } else {
        return CAM_ERR_INIT;
    }
    #endif

}

cam_status_t cam_Configure() {
    // Check if configured
    if (cam_configured == 1) {
        return CAM_WARN_ALCONF;
    }

    #ifdef __OV7670
    ov7670_status_t st = ov7670_Configure();
    if (st == OV7670_INFO_OK) {
        cam_configured = 1;
        return CAM_INFO_OK;
    } else {
        return CAM_ERR_CONFIG;
    }
    #endif

    #ifdef __OV5642
    ov5642_status_t st = ov5642_Configure();
    if (st == OV5642_INFO_OK) {
        cam_configured = 1;
        return CAM_INFO_OK;
    } else {
        return CAM_ERR_CONFIG;
    }
    #endif
}

cam_status_t cam_Capture() {
    // Check if initialized
    if (cam_initialized != 1) {
        return CAM_ERR_INIT;
    }

    // Check if configured
    if (cam_configured != 1) {
        return CAM_ERR_CONFIG;
    }

    #ifdef __OV7670
    ov7670_status_t st = ov7670_Capture();
    if (st == OV7670_INFO_OK) {
        return CAM_INFO_OK;
    } else {
        return CAM_ERR_CAPTURE;
    }
    #endif

    #ifdef __OV5642
    ov5642_status_t st = ov5642_Capture();
    if (st == OV5642_INFO_OK) {
        return CAM_INFO_OK;
    } else {
        return CAM_ERR_CAPTURE;
    }
    #endif
}

cam_status_t cam_Transfer() {
    // Check if initialized
    if (cam_initialized != 1) {
        return CAM_ERR_INIT;
    }

    // Check if configured
    if (cam_configured != 1) {
        return CAM_ERR_CONFIG;
    }


    // /* Power down the camera*/
    // GPIO_SetBits(GPIOG, GPIO_Pin_2);

    log_Log(CAM, CAM_INFO_OK, "Beginning image transfer.\0");

    #ifdef __WIFI

    //Boulder Lat Longs
    // double latitude = 40.0149856;
    // double longitude = -105.2705456;

    char* loc = "40.0149856,-105.27052456";

    #ifdef __OV7670
    wifi_Send(CAM, CAM_WARN_UNKNOWN, "abcdefghijuklmnopqrstuvwxyz\0", 0, 0);
    wifi_Send(CAM, CAM_INFO_IMAGE, loc, OV7670_IMAGE_BUFSIZE, (uint8_t *) SDRAM_IMAGEADDR);
    #endif

    #ifdef __OV5642
    wifi_Send(CAM, CAM_INFO_IMAGE, loc, OV5642_IMAGE_BUFSIZE*16, (uint8_t *) SDRAM_IMAGEADDR);
    #endif

    #else

    #ifdef __OV7670
    log_Log(CAM, CAM_INFO_IMAGE, loc, OV7670_IMAGE_BUFSIZE, (uint8_t *) SDRAM_IMAGEADDR);
    #endif

    #ifdef __OV5642
    log_Log(CAM, CAM_INFO_IMAGE, loc, OV5642_IMAGE_BUFSIZE, (uint8_t *) SDRAM_IMAGEADDR);
    #endif

    #endif

    return CAM_INFO_OK;
}
