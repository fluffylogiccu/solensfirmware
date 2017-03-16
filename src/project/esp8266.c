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

// UART queue
static esp8266_queue_t *esp8266_queue;

// MQTT connection initializer
static MQTTPacket_connectData esp8266_mqtt;

// MQTT topic initializer
static MQTTString esp8266_mqttTopic;

// MQTT message buffer
uint8_t esp8266_mqttBuf[200];

uint32_t esp8266_mqttLen;

/**************************************
 * Private functions
 */

int esp8266_uartBufRead(unsigned char *buf, int len) {
	// Read from circular buffer
    unit8_t ch = 0;
    esp8266_status_t ret = ESP8266_INFO_OK;
    uint32_t i = 0;
    while (i < len) {
        ret = esp8266_queueGet(&ch);
        if (ret == ESP8266_INFO_OK) {
            *buf = ch;
            buf++;
        } else {
            return -1;
        }
        i++;
    }
    
	return 0;
}

esp8266_status_t esp8266_uartSend(uint8_t *data, uint32_t len) {
    // Send data over USART1
    int i = 0
    while (i < len) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, *(data+i));
        i++;
    }
}

esp8266_status_t esp8266_mqttInit() {
    esp8266_mqtt = MQTTPacket_connectData_initializer;
    esp8266_mqttLen = sizeof(esp8266_mqttBuf);
    esp8266_mqttTopic = MQTTString_initializer;
    
    esp8266_mqtt.clientID.cstring = "Ben";
    esp8266_mqtt.keepAliveInterVal = 20;
    esp8266_mqtt.cleansession = 1;
    esp8266_mqtt.username.cstring = "admin";
    esp8266_mqtt.password.cstring = "password";
   
    uint32_t len = MQTTSerializeConnect(esp8266_mqttBuf, esp8266_mqttLen, &esp8266_mqtt);
    esp8266_status_t ret = esp8266_uartSend(esp8266_mqttBuf, len);
}

esp8266_status_t esp8266_queueInit() {

    // Allocate space for queue
    esp8266_queue = NULL;
    esp8266_queue = (esp8266_queue_t *) malloc(sizeof(esp8266_queue_t));

    if (esp8266_queue == NULL) {
        log_Log(ESP8266, ESP8266_ERR_MALLOC, "Couldn't initialize esp8266_queue memory.\0");
        return ESP8266_ERR_MALLOC;
    }

    esp8266_queue->esp8266_queue_buf = NULL;
    esp8266_queue->esp8266_queue_buf = (uint8_t *) malloc(sizeof(esp8266_esp8266_t *)*ESP8266_QUEUE_CAP);
    if (esp8266_queue->esp8266_queue_buf == NULL) {
        log_Log(ESP8266, ESP8266_ERR_MALLOC, "Couldn't initialize esp8266_queue_buf memory.\0");
        return ESP8266_ERR_MALLOC;
    }

    esp8266_queue->esp8266_queue_head = esp8266_queue->esp8266_queue_buf;
    esp8266_queue->esp8266_queue_tail = esp8266_queue->esp8266_queue_buf;
    esp8266_queue->esp8266_queue_capacity = ESP8266_QUEUE_CAP;
    esp8266_queue->esp8266_queue_size = 0;
    esp8266_queue->esp8266_queue_status = ESP8266_QUEUE_EMPTY;

    #ifndef __ESP8266
    esp8266_initialized = 1;
    #endif

    return ESP8266_INFO_OK;
}

esp8266_status_t esp8266_queuePut(uint8_t data) {
    // Check for full queue
    if (esp8266_queue->esp8266_queue_status == ESP8266_QUEUE_FULL) {
        log_Log(ESP8266, ESP8266_ERR_QUEUEFULL);
        return ESP8266_ERR_QUEUEFULL;
    }
    if (esp8266_queue->esp8266_queue_status == ESP8266_QUEUE_INVALID) {
        log_Log(ESP8266, ESP8266_ERR_QUEUEINVALID);
        return ESP8266_ERR_QUEUEINVALID;
    }

    // Set data
    *esp8266_queue->esp8266_queue_head = data;

    // Increment head and check for buffer wrap
    esp8266_queue->esp8266_queue_head++;
    if ((esp8266_queue->esp8266_queue_head - esp8266_queue->esp8266_queue_buf) >=
         esp8266_queue->esp8266_queue_capacity) {

        esp8266_queue->esp8266_queue_head -= esp8266_queue->esp8266_queue_capacity;
    }
    esp8266_queue->esp8266_queue_size++;

    // Set new state
    if (esp8266_queue->esp8266_queue_size == esp8266_queue->esp8266_queue_capacity ||
        esp8266_queue->esp8266_queue_head == esp8266_queue->esp8266_queue_tail) {

        esp8266_queue->esp8266_queue_status = ESP8266_QUEUE_FULL;
    } else {
        esp8266_queue->esp8266_queue_status = ESP8266_QUEUE_PARTIAL;
    }

    return ESP8266_INFO_OK;
}

esp8266_status_t esp8266_queueGet(uint8_t *data) {
    // Check for valud esp8266 pointer
    if (esp8266 == NULL) {
        log_Log(ESP8266, ESP8266_ERR_NULLPTR, "Invalid command pointer in parameter list for esp8266_queueGet.\0");
        return ESP8266_ERR_NULLPTR;
    }

    // Check if queue is empty
    if (esp8266_queue->esp8266_queue_status == ESP8266_QUEUE_EMPTY) {
        log_Log(ESP8266, ESP8266_ERR_QUEUEEMPTY, "Trying to get from an empty command queue.\0");
        return ESP8266_ERR_QUEUEEMPTY;
    }

    // Set command
    *data = *esp8266_queue->esp8266_queue_tail;

    // Increment tail and check for wrap
    esp8266_queue->esp8266_queue_tail++;
    if ((esp8266_queue->esp8266_queue_tail - esp8266_queue->esp8266_queue_buf) >=
         esp8266_queue->esp8266_queue_capacity) {

        esp8266_queue->esp8266_queue_tail -= esp8266_queue->esp8266_queue_capacity;
    }
    esp8266_queue->esp8266_queue_size--;

    // Set new state
    if (esp8266_queue->esp8266_queue_size == 0) {
        esp8266_queue->esp8266_queue_status = ESP8266_QUEUE_EMPTY;
    } else {
        esp8266_queue->esp8266_queue_status = ESP8266_QUEUE_PARTIAL;
    }

    return ESP8266_INFO_OK;
}

/**************************************
 * Public functions
 */
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
        uint8_t ch = USART1->DR;
        esp8266_queuePut(ch);
		// Save to circular buffer
    }
}

esp8266_status_t esp8266_Send(wifi_topic_t *wifi_topic, uint8_t *data, uint32_t len) {
	uint32_t i = 0;

    #ifdef __STM32F429I_DISCOVERY
    // Send data serially
    while (i < len) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, *(((uint8_t *)wifi_)+i));
        i++;
    }
    #endif

    #ifdef __S0LENS_A
    // Send data serially
    while (i < len) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, *(((uint8_t *)wifi_packet)+i));
        i++;
    }
    #endif

    return WIFI_INFO_OK;
}

esp8266_status_t esp8266_Init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    #ifdef __STM32F429I_DISCOVERY
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

    #endif

    #ifdef __S0LENS_A

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

    #endif

	return ESP8266_INFO_OK;
}

