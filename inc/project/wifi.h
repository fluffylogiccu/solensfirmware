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

#define WIFI_MAXMSGSIZE 255
#define WIFI_MAXDATASIZE 16777216

/* @brief Type for wifi packets
 */
#ifdef __S0LENS_A
typedef struct __attribute__ ((packed)) wifi_packet_s {
    char* wifi_packet_topic;
    uint32_t wifi_packet_dataLen;
    uint8_t *wifi_packet_data;
} wifi_packet_t;
#endif

#ifdef __STM32F429I_DISCOVERY
typedef struct __attribute__ ((packed)) wifi_packet_s {
    uint8_t wifi_packet_mod;
    uint8_t wifi_packet_status;
    uint8_t wifi_packet_msgLen;
    uint8_t *wifi_packet_msg;
    uint32_t wifi_packet_dataLen;
    uint8_t *wifi_packet_data;
} wifi_packet_t;
#endif
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
 *  @param module of the type mod_t
 *  @param status code to send
 *  @param msg message as a pointer to uint8_t
 *  @param len optional data length paramter
 *  @param data optional pointer to data bufffer
 *  @return a status code of type wifi_status_t
 */
wifi_status_t wifi_Send(mod_t mod, gen_status_t status, char *msg, uint32_t len, uint8_t *data);

/** @brief Wifi initialization
 *
 *  This function initializes the wifi module.
 *
 *  @return a status code of type wifi_status_t
 */
wifi_status_t wifi_Init();

# endif /* __WIFI_H */
