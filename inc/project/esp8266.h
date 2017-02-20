/** @file esp8266.h
 *  @brief Function declarateions for the esp8266 module.
 *
 *  This code is for the stm32f429 side. The actual NodeMCU
 *  code running on the esp8266 can be found in the periph
 *  folder in the root directory.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __ESP8266_H
#define __ESP8266_H

/*************************************
 * @name Includes and definitions
 */

#include "err.h"
#include "wifi.h"
#include <stdint.h>

#define ESP8266_BAUDRATE 9600

/**************************************
 * @name Private functions
 */

/**************************************
 * @name Public functions
 */

/** @brief Send data to the ESP8622 over UART
 *
 *  ESP866 should be initialized before calling this
 *  function. This function sends a single wifi packet
 *  over UART.
 *
 *  @param wifi_packet the packet to send
 *  @return a return code of type esp8266_status_t
 */
esp8266_status_t esp8266_Send(wifi_packet_t *wifi_packet);

/** @brief Initialize the ESP8266 driver
 *
 *  This function initializes UART1 to communicate with
 *  the ESP8622 wifi module.
 *
 *  @return a return code of type esp8266_status_t
 */
esp8266_status_t esp8266_Init();

# endif /* _ESP8266__H */
