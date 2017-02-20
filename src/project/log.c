/** @file log.c
 *  @brief Implemenation of the log functions.
 *
 *  This contains the implementations of the 
 *  log functions.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#ifdef __LOG

#include "stm32f4xx.h"
#include <stdio.h>
#include "log.h"
#include "err.h"
#include "mod.h"
#include <stdint.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

/* @brief Initialization flag for logger
 */
static uint8_t log_initialized = 0;

#endif

/**************************************
 * Private functions
 */

#ifdef __LOG
log_status_t log_log2(mod_t module, gen_status_t status) {
    return log_log5(module, status, "\0", 0, NULL);
}

log_status_t log_log3(mod_t module, gen_status_t status, char *msg) {
    return log_log5(module, status, msg, 0, NULL);
}

log_status_t log_log4(mod_t module, gen_status_t status, uint32_t len, uint8_t *data) {
    return log_log5(module, status, "\0", len, data);
}

log_status_t log_log5(mod_t module, gen_status_t status, char *msg, uint32_t len, uint8_t *data) {

    // Check for compiler verbosity directives
    #ifdef __LOG_INFO
    if (status < INFO) {
        return LOG_WARN_IGNORED;
    }
    #endif
    #ifdef __LOG_WARN
    if (status < WARN) {
        return LOG_WARN_IGNORED;
    }
    #endif
    #ifdef __LOG_ERR
    if (status < ERR) {
        return LOG_WARN_IGNORED;
    }
    #endif

    // Check for data size  
    if (len > LOG_MAXDATASIZE) {
        return LOG_ERR_DATASIZE;
    }
   
    // Get msg length
    uint8_t msgLen = 0;
    while (*(msg+msgLen) != '\0') {
        msgLen++;

        // Check for message size
        if (msgLen == LOG_MAXMSGSIZE) {
            return LOG_ERR_MSGSIZE;
        }
    }

    // Construct packet
    log_packet_t log_packet;
    log_packet.log_packet_mod = module;
    log_packet.log_packet_status = status;
    log_packet.log_packet_msgLen = msgLen;
    log_packet.log_packet_msg = (uint8_t *) msg;
    log_packet.log_packet_dataLen = len;
    log_packet.log_packet_data = data;

    return log_send(&log_packet);
}

log_status_t log_send(log_packet_t *log_packet) {

    // Figure out sizes
    uint32_t i = 0;
    uint8_t firstSize = sizeof(log_packet->log_packet_mod) + 
                         sizeof(log_packet->log_packet_status) + 
                         sizeof(log_packet->log_packet_msgLen);
    uint16_t secondSize = log_packet->log_packet_msgLen;
    uint16_t thirdSize = sizeof(log_packet->log_packet_dataLen) +
                          sizeof(log_packet->log_packet_msg) +
                          firstSize;
    uint32_t fourthSize = log_packet->log_packet_dataLen;;
   
    // Send data serially
    while (i < firstSize) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, *(((uint8_t *)log_packet)+i));
        i++;
    }
    i = 0;
    while (i < secondSize) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, *(log_packet->log_packet_msg+i));
        i++;
    }
    i = firstSize + sizeof(log_packet->log_packet_msg);
    while (i < thirdSize) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, *(((uint8_t *)log_packet)+i));
        i++;
    }
    i = 0;
    while (i < fourthSize) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        // Delay
        if (i % 100 == 0) {
            // 100 cycles works fine for lab computer
            for (uint32_t j = 0; j < 1000; j++) {}
        }
        USART_SendData(USART2, *(log_packet->log_packet_data+i));
        i++;
    }

    return LOG_INFO_OK;
}
#endif

/**************************************
 * Public functions
 */

#ifdef __LOG
log_status_t log_Init() {
    
    if (log_initialized == 1) {
        return LOG_WARN_ALINIT;
    }

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
   
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Enable USART2 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // Connect Pin D5 to USART2 Tx
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);

    // Configure USART Tx as alternate function
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Medium_Speed;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = LOG_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = (USART2->CR1 & (USART_CR1_RE | USART_CR1_TE)) | USART_Mode_Tx;

    // USART configuration
    USART_Init(USART2, &USART_InitStructure);

    // Enable USART
    USART_Cmd(USART2, ENABLE);

    log_initialized = 1;

    return LOG_INFO_OK;
}
#endif
// Public macro function
// log_Log(...) {}
