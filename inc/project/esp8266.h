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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define ESP8266_BAUDRATE 230400
#define SLIP_END      0300    /**< Indicates end of packet */
#define SLIP_ESC      0333    /**< Indicates byte stuffing */
#define SLIP_ESC_END  0334    /**< ESC ESC_END means END data byte */
#define SLIP_ESC_ESC  0335    /**< ESC ESC_ESC means ESC data byte */
#define ESP_TIMEOUT   1000000 /**< Default timeout for TCP requests when waiting for a response */
#define MAX_PACKET_SIZE 950     /*This needs to be a multiple of the image width times 2*/
#define MAX_ATTEMPT 1000000

// Enumeration of commands supported by esp-link, this needs to match the definition in
// esp-link!
typedef enum {
  CMD_NULL = 0,     /**< null, mainly to prevent 0 from doing something bad */
  CMD_SYNC,         /**< Synchronize, starts the protocol */
  CMD_RESP_V,       /**< Response with a value */
  CMD_RESP_CB,      /**< Response with a callback */
  CMD_WIFI_STATUS,  /**< Get the wifi status */
  CMD_CB_ADD,       /**< Add a custom callback */
  CMD_CB_EVENTS,    /**< ??? */
  CMD_GET_TIME,     /**< Get current time in seconds since the unix epoch */
  //CMD_GET_INFO,

  CMD_MQTT_SETUP = 10, /**< Register callback functions */
  CMD_MQTT_PUBLISH,    /**< Publish MQTT topic */
  CMD_MQTT_SUBSCRIBE,  /**< Subscribe to MQTT topic */
  CMD_MQTT_LWT,        /**< Define MQTT last will */

  CMD_REST_SETUP = 20, /**< Setup REST connection */
  CMD_REST_REQUEST,    /**< Make request to REST server */
  CMD_REST_SETHEADER,  /**< Define HTML header */

  CMD_WEB_SETUP = 30,  /**< web-server setup */
  CMD_WEB_DATA,        /**< used for publishing web-server data */

  CMD_SOCKET_SETUP = 40,  /**< Setup socket connection */
  CMD_SOCKET_SEND,        /**< Send socket packet */
} CmdName; /**< Enumeration of commands supported by esp-link, this needs to match the definition in esp-link! */

enum WIFI_STATUS {
  STATION_IDLE = 0,        /**< Idle status */
  STATION_CONNECTING,      /**< Trying to connect */
  STATION_WRONG_PASSWORD,  /**< Connection error, wrong password */
  STATION_NO_AP_FOUND,     /**< Connection error, AP not found */
  STATION_CONNECT_FAIL,    /**< Connection error, connection failed */
  STATION_GOT_IP           /**< Connected, received IP */
}; /**< Enumeration of possible WiFi status */

typedef struct {
  uint8_t* buf;
  uint16_t bufSize;
  uint16_t dataLen;
  uint8_t isEsc;
} slip_protocol_t; /**< Protocol structure  */

typedef struct PACKED {
  //Fields from the el-client packet struct
  uint16_t cmd;            /**< Command to execute */
  uint16_t argc;           /**< Number of arguments */
  uint32_t value;          /**< Callback to invoke, NULL if none; or response value */
  uint8_t  args[0];        /**< Arguments */
} slip_packet_t; /**< Packet structure  */

typedef struct RESPONSE {
    uint16_t _arg_num;
    uint8_t* _arg_ptr;
    slip_packet_t* _cmd;
} slip_response_t;

//Circular buffer
typedef struct __attribute__ ((packed)) wifi_queue_s {
    uint8_t *buf;
    uint8_t *head;
    uint8_t *tail;
    uint8_t *bufEnd;
    uint16_t capacity;
    uint16_t count;
} uart_buffer_t;

/**************************************
 * @name Private functions
 */

 /** @brief Initialize SLIP
  *
  *  Sets protocol structure from decoding SLIP
  *  data form the ESP8266
  *
  *  @return nothing
  */
esp8266_status_t esp8266_Slip_Init(void);

/** @brief Initialize USART1 Rx Buffer
 *
 *  Sets up circular buffer for recieving data
 *  on USART1 from ESP8266
 *
 *  @return nothing
 */
esp8266_status_t USART_Buffer_Init(uint16_t capacity);

/** @brief Destroy the USART1 circular Rx buffer
 *
 *
 *  @return nothing
 */
esp8266_status_t USART_Buffer_Free(void);

/** @brief Get the number of bytes sitting in
 *  the circular buffer
 *
 *  @return number of bytes in the the buffer
 */
uint16_t USART_Buffer_Available(void);

/** @brief Push a byte onto the buffer
 *
 *  If the buffer is full, data will be lost
 *
 */
void USART_Buffer_Push(uint8_t data);

/** @brief Pop a byte off the buffer
 *
 *  Returns null if buffer is empty
 *
 *  @return first byte that was pushed on the buffer
 */
uint8_t USART_Buffer_Pop(void);


/** @brief Interrupt handler for USART1 Rx
 *
 */
 void USART1_IRQHandler(void) ;

//===== CRC helper functions

/*! crc16Add(unsigned char b, uint16_t acc)
@brief Create CRC for a byte add it to an existing CRC checksum and return the result
@param b
	Byte which CRC will be added
@param acc
	Existing CRC checksum
@return <code>uint16_t</code>
	New CRC checksum
@par Example
@code
	no example code yet
@endcode
*/
uint16_t crc16Add(unsigned char b, uint16_t acc);

/*! crc16Data(const unsigned char *data, uint16_t len, uint16_t acc)
@brief Create/add CRC for a data buffer
@param data
	The data buffer which will be CRCed
@param len
	Size of the data buffer
@param acc
	Existing CRC checksum
@return <code>uint16_t</code>
	New CRC checksum
@par Example
@code
	no example code yet
@endcode
*/
uint16_t crc16Data(const unsigned char *data, uint16_t len, uint16_t acc);

slip_packet_t *esp8266_protoCompletedCb(void);

slip_packet_t *esp8266_Process(void);

int16_t popArg(slip_response_t* packet, void* data, uint16_t maxLen);

char* popString(slip_response_t* response);

void packet_to_response(slip_packet_t *packet, slip_response_t *response);


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

/** @brief Handshake with the esp-link
 *
 */
 bool esp8266_Sync();

/** @brief Write byte data to UART1 using SLIP
 *
 *  This function sanitizes out special characters
 *  in the data and sends the correct special characters
 *  for the SLIP protocol
 *  @return none
 */
void esp8266_Write_Slip_Byte(uint8_t data);

/** @brief Write data to UART1 using SLIP
 *
 *  Wrapper function for SLIP USART1 communication
 *  if more than one byte needs to be written
 *
 *  @return none
 */
void esp8266_Write_Slip(void *data, uint16_t len);

/** @brief Write data to UART1 without encoding for SLIP
 *
 *  @return status
 */
esp8266_status_t esp8266_Raw_Send(uint16_t data);

/** @brief Modified Request function from el-client
 *
 *  Wrapper function for SLIP commands to be sent to
 *  the ESP8622
 *
 *  @return none
 */
void esp8266_Request(uint16_t cmd, uint32_t value, uint16_t argc);
void esp8266_Request2(const void* data, uint16_t len);
void esp8266_Request0(void);

/** @brief Modified WaitReturn function from el-client
 *
 *  Spins on a response from the ESP8266 with a timeout
 *  defined by ESP_TIMEOUT
 *
 *  @return none
 */
slip_packet_t *esp8266_Wait_Return(uint32_t timeout);


wifi_status_t esp8266_WifiCb(slip_response_t *response);

void esp8266_ResetCb(void);

/** @brief Modified GetTime function from el-client
 *
 *  Sends a request to the ESP8266 for the time
 *  The esp-link firmware will return the seconds
 *  if the sntp sync is successful
 *
 *  @return none
 */
uint32_t esp8266_GetTime();



/* MQTT */
void mqtt_setup(void);

void mqtt_lwt(const char* topic, const char* message, uint8_t qos, uint8_t retain);

void mqtt_subscribe(const char* topic, uint8_t qos);

void mqtt_publish(const char* topic, const uint8_t* data, const uint16_t len, uint8_t qos, uint8_t retain);

void mqtt_connected_callback(void *response);

void mqtt_disconnnected_callback(void *response);

void mqtt_published_callback(void *response);

void mqtt_data_callback(slip_response_t *response);

# endif /* _ESP8266__H */
