/** @file wifi.c
 *  @brief Implemenation of the wifi functions.
 *
 *  This function contains implementations of wifi
 *  functions depending on compiler directive selections.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "wifi.h"
#ifdef __ESP8266
#include "esp8266.h"
#endif
#include <stdint.h>

static uint8_t wifi_initialized = 0;

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */

wifi_status_t wifi_Send(mod_t module, gen_status_t status, char *msg, uint32_t len, uint8_t *data) {

	if (len > WIFI_MAXDATASIZE) {
        return WIFI_ERR_DATASIZE;
    }

    // Get msg length
    uint8_t msgLen = 0;
    while (*(msg+msgLen) != '\0') {
        msgLen++;

        // Check for message size
        if (msgLen == WIFI_MAXMSGSIZE) {
            return WIFI_ERR_MSGSIZE;
        }
    }

    // Construct packet
    wifi_packet_t wifi_packet;
    wifi_packet.wifi_packet_mod = module;
    wifi_packet.wifi_packet_status = status;
    wifi_packet.wifi_packet_msgLen = msgLen;
    wifi_packet.wifi_packet_msg = (uint8_t *) msg;
    wifi_packet.wifi_packet_dataLen = len;
    wifi_packet.wifi_packet_data = data;

	#ifdef __ESP8266
	esp8266_status_t st = esp8266_Send(&wifi_packet);
	if (st == ESP8266_INFO_OK) {
		return WIFI_INFO_OK;
	} else {
		return WIFI_ERR_SEND;
	}
	#endif

	return WIFI_ERR_UNKNOWN;
}

wifi_status_t wifi_Init() {

	if (wifi_initialized == 1) {
		return WIFI_WARN_ALINIT;
	}

	#ifdef __ESP8266
	esp8266_status_t st = esp8266_Init();
	if (st == ESP8266_INFO_OK) {
		wifi_initialized = 1;
		return WIFI_INFO_OK;
	} else {
		return WIFI_ERR_INIT;
	}
	#endif

	return WIFI_ERR_UNKNOWN;
}
