/** @file wifi.h
 *  @brief Function prototypes for the wifi driver
 *
 *  This contains the prototypes, macros, constants,
 *  and global variables for the wifi driver. Compiler
 *  directives choose the wifi module.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __WIFI_H
#define __WIFI_H

/*************************************
 * @name Includes and definitions
 */

#include "err.h"
#include "mod.h"
#include <stdint.h>

#define WIFI_MAXDATASIZE 16777216

/* @brief Wifi channel enum
 */
typedef enum wifi_topic_e {IMAGE, HEALTH} wifi_topic_t;

/**************************************
 * @name Private functions
 */

/**************************************
 * @name Public functions
 */

/** @brief Send data over wifi
 *
 *  This function sends data over the wifi module.
 *
 *  @param channel where to send data 
 *  @param len optional data length paramter
 *  @param data optional pointer to data bufffer
 *  @return a status code of type wifi_status_t
 */
wifi_status_t wifi_Send(wifi_Channel_t channel, uint8_t *data, uint32_t len);

/** @brief Wifi initialization
 *
 *  This function initializes the wifi module.
 *
 *  @return a status code of type wifi_status_t
 */
wifi_status_t wifi_Init(); 

# endif /* __WIFI_H */
