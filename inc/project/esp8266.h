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

#define ESP8266_BAUDRATE 921600*2
#define ESP8266_QUEUE_CAP 256

typedef enum esp8266_topic_e {ESP8266_IMAGE, ESP8266_HEALTH} esp8266_topic_t;

/* @brief queue state enumeration
 */
typedef enum esp8266_queue_state_e {
    ESP8266_QUEUE_EMPTY,
    ESP8266_QUEUE_FULL,
    ESP8266_QUEUE_PARTIAL,
    ESP8266_QUEUE_INVALID
} esp8266_queue_state_t;

typedef struct __attribute__ ((packed)) esp8266_queue_s {
    uint8_t *esp8266_queue_buf;
    uint8_t *esp8266_queue_head;
    uint8_t *esp8266_queue_tail;

    uint16_t esp8266_queue_capacity;
    uint16_t esp8266_queue_size;

    esp8266_queue_state_t esp8266_queue_status;

} esp8266_queue_t;

/**************************************
 * @name Private functions
 */

/** @brief Read from the circular buffer
 *
 *  This function should be called through a function
 *  pointer in the MQTTPacket_read function to read data
 *  during MQTT handshaking. It should get UART data and
 *  put it in buf.
 *
 *  @param buf the buffer to put bytes in
 *  @param len the number of bytes to read
 *  return the return code
 */
int esp8266_uartBufRead(unsigned char *buf, int len);

/** @brief Send data to the ESP8622 over UART
 *
 *  ESP866 should be initialized before calling this
 *  function. This function sends a len number of bytes
 *  from the data buffer
 *
 *  @param data the data buffer
 *  @param len the number of bytes to send
 *  @return a return code of type esp8266_status_t
 */
esp8266_status_t esp8266_uartSend(uint8_t *data, uint32_t len);

/** @brief Initialize the UART
 *
 *  This function initializes the UART for bidrectional
 *  communication.
 *
 *  @return a return code of type esp6288_status_t
 **/
esp8266_status_t esp8266_uartInit();

/** @brief Initialize the MQTT client
 *
 *  This should be called from the ESP initialization
 *  function. This will initialize the MQTT connection
 *  over UART.
 *
 *  @return a return code of type esp6288_status_t
 **/
esp8266_status_t esp8266_mqttInit();

/** @brief Initialize the UART RX queue
 *
 *  This initializes the UART RX queue.
 *
 *  @return a return code of type esp6288_status_t
 **/
esp8266_status_t esp8266_queueInit();

/** @brief Puts a byte in the queue
 *
 *  This function puts a single byte in the queue. This
 *  should be called from the UART interrupt handler.
 *
 *  @param data the byte to put in the queue
 *  @return a return code of type esp6288_status_t
 **/
esp8266_status_t esp8266_queuePut(uint8_t data);

/** @brief Gets a byte in the queue
 *
 *  This function gets a single byte from the queue.
 *
 *  @param data the location to put the data in
 *  @return a return code of type esp6288_status_t
 **/
esp8266_status_t esp8266_queueGet(uint8_t *data);


/**************************************
 * @name Public functions
 */

/** @brief Send data to the ESP8622 over MQTT interface
 *
 *  ESP866 and MQTT should be initialized before calling
 *  this function. This will send the data the data buffer
 *  to the topic described by wifi_topic.
 *
 *  @param esp_topic the topic to send to 
 *  @param data the data buffer
 *  @param len the number of bytes to send
 *  @return a return code of type esp8266_status_t
 */
esp8266_status_t esp8266_Send(esp8266_topic_t esp_topic, uint8_t *data, uint32_t len);

/** @brief Initialize the ESP8266 driver
 *
 *  This function initializes UART1 to communicate with
 *  the ESP8622 wifi module.
 *
 *  @return a return code of type esp8266_status_t
 */
esp8266_status_t esp8266_Init();

# endif /* _ESP8266__H */
