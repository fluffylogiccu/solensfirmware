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

wifi_status_t wifi_Send(wifi_topic_t topic, uint8_t *data, uint32_t len) {


	if (len > WIFI_MAXDATASIZE) {
        return WIFI_ERR_DATASIZE;
    }

	#ifdef __ESP8266
	esp8266_status_t st = esp8266_Send((esp8266_topic_t) topic, data, len);
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
