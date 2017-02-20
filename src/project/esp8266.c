/** @file esp8266.c
 *  @brief Implemenation of the esp8266 driver
 *
 *  This contains the implementations of the 
 *  esp8266 driver functions.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "esp8266.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "wifi.h"
#include <stdint.h>

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */

esp8266_status_t esp8266_Send(wifi_packet_t *wifi_packet) {
	uint32_t i = 0;
    uint8_t firstSize = sizeof(wifi_packet->wifi_packet_mod) +
                        sizeof(wifi_packet->wifi_packet_status) +
                        sizeof(wifi_packet->wifi_packet_msgLen);
    uint16_t secondSize = wifi_packet->wifi_packet_msgLen;
    uint16_t thirdSize = sizeof(wifi_packet->wifi_packet_dataLen) +
                         sizeof(wifi_packet->wifi_packet_msg) +
                         firstSize;
    uint32_t fourthSize = wifi_packet->wifi_packet_dataLen;;

    // Send data serially
    while (i < firstSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, *(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < secondSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, *(wifi_packet->wifi_packet_msg+i));
        i++;
    }
    i = firstSize + sizeof(wifi_packet->wifi_packet_msg);
    while (i < thirdSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, *(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < fourthSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        // Delay of 100 works for lab computers
        if (i % 100 == 0) {        
            for (uint32_t j = 0; j < 1000; j++) {}
        }
        USART_SendData(USART1, *(wifi_packet->wifi_packet_data+i));
        i++;
    }
    return WIFI_INFO_OK;
}

esp8266_status_t esp8266_Init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Enable USART1 Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // Connect Pin A9 to USART1 Tx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

    // Configure USART Tx as alternate function
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Medium_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = ESP8266_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = (USART1->CR1 & (USART_CR1_RE | USART_CR1_TE)) | USART_Mode_Tx;

    // USART configuration
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART
    USART_Cmd(USART1, ENABLE);

	return ESP8266_INFO_OK;
}

