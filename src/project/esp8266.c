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


slip_protocol_t *proto;
uint8_t protoBuf[128];
uart_buffer_t *rxBuffer;
uint16_t crc;
bool syncing = false;

/**************************************
 * Private functions
 */


void Slip_Init(){
	proto->buf = protoBuf;
	proto->bufSize = sizeof(protoBuf);
	proto->dataLen = 0;
	proto->isEsc = 0;
}
void Slip_Write_Byte(uint8_t data){
	switch(data){
		case SLIP_END:
		Safe_Send(SLIP_ESC);
		Safe_Send(SLIP_ESC_END);
		break;
		case SLIP_ESC:
		Safe_Send(SLIP_ESC);
		Safe_Send(SLIP_ESC_ESC);
		break;
		default:
		Safe_Send(data);
	}
}

void Slip_Write(void *data, uint16_t len){
	uint8_t *d = (uint8_t*)data;
	while(len--){
		Slip_Write_Byte(*d++);
	}
}

void USART_Buffer_Init(uint16_t capacity){
	rxBuffer->capacity = capacity;
	rxBuffer->count = 0;
	rxBuffer->buf = (uint8_t*)malloc(capacity * 8);
	rxBuffer->head = rxBuffer->buf;
	rxBuffer->tail = rxBuffer->buf;
	rxBuffer->bufEnd = rxBuffer->buf + capacity * 8;
}

void USART_Buffer_Free(){
	free(rxBuffer->buf);
	rxBuffer->capacity = 0;
	rxBuffer->count = 0;
	rxBuffer->head = NULL;
	rxBuffer->tail = NULL;
	rxBuffer->bufEnd = NULL;
}

uint16_t USART_Buffer_Available(){
	if(rxBuffer->buf != NULL){
		return rxBuffer->count;
	}
	else{
		//Buffer not initialized
		return 0;
	}
}

void USART_Buffer_Push(uint8_t data){
	if(rxBuffer->count == rxBuffer->capacity){
		//Buffer is full
		return;
	}
	*(rxBuffer->head) = data;
	rxBuffer->head++;
	if(rxBuffer->head == rxBuffer->bufEnd){
		rxBuffer->head = rxBuffer->buf;
	}
	rxBuffer->count++;
}

uint8_t USART_Buffer_Pop8(void){
	if(rxBuffer->count == 0){
		//Buffer is empty
		return -1;
	}
	uint8_t item = *(rxBuffer->tail);
	rxBuffer->tail++;
	if(rxBuffer->tail == rxBuffer->bufEnd){
		rxBuffer->tail = rxBuffer->buf;
	}
	rxBuffer->count--;
	return item;
}

uint16_t USART_Buffer_Pop16(void){
	if(rxBuffer->count == 0){
		//Buffer is empty
		return -1;
	}
	uint16_t item = (uint16_t)*(rxBuffer->tail);
	rxBuffer->tail++;
	if(rxBuffer->tail == rxBuffer->bufEnd){
		rxBuffer->tail = rxBuffer->buf;
	}
	rxBuffer->count = rxBuffer->count - 1;
	if(rxBuffer->count == 0){
		//Buffer only had one byte
		return item;
	} else {
		item |= (*(rxBuffer->tail) << 8);
		rxBuffer->tail++;
		if(rxBuffer->tail == rxBuffer->bufEnd){
			rxBuffer->tail = rxBuffer->buf;
		}
		rxBuffer->count--;
		return item;
	}
}

void USART1_IRQHandler(void) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE)){
		uint8_t ch = USART1->DR;
		USART_Buffer_Push(ch);
    }
}


uint16_t crc16Add(unsigned char b, uint16_t acc)
{
  acc ^= b;
  acc = (acc >> 8) | (acc << 8);
  acc ^= (acc & 0xff00) << 4;
  acc ^= (acc >> 8) >> 4;
  acc ^= (acc & 0xff00) >> 5;
  return acc;
}

uint16_t crc16Data(const unsigned char *data, uint16_t len, uint16_t acc)
{
  for (uint16_t i=0; i<len; i++)
    acc = crc16Add(*data++, acc);
  return acc;
}

slip_packet_t *protoCompletedCb(void){
    slip_packet_t *packet = (slip_packet_t*)proto->buf;
    uint16_t crc = crc16Data(proto->buf, proto->dataLen-2, 0);

    uint16_t resp_crc = *(uint16_t*)(proto->buf+proto->dataLen-2);
    if(crc != resp_crc){
        return NULL;
    }
    switch(packet->cmd){
        case CMD_RESP_V:
            return packet;
        case CMD_RESP_CB:
            //Here is where they return a callback function
            return NULL;
        case CMD_SYNC:
            //esp-link not in sync
            Esp_ResetCb();
            return NULL;
        default:
            //command not implemented
            return NULL;
    }
}

slip_packet_t *Slip_Process(){
    uint16_t value;
    while(USART_Buffer_Available() > 0){
        value = USART_Buffer_Pop16();
        if(value == SLIP_ESC){
            proto->isEsc = 1;
        } else if (value == SLIP_END){
            slip_packet_t *packet = proto->dataLen >= 8 ? protoCompletedCb() : 0;
            proto->dataLen = 0;
            proto->isEsc = 0;
            if(packet != NULL)return packet;
        } else {
            if(proto->isEsc){
                if(value == SLIP_ESC_END) value = SLIP_END;
                if(value == SLIP_ESC_ESC) value = SLIP_ESC;
                proto->isEsc = 0;
            }
            if(proto->dataLen < proto->bufSize){
                proto->buf[proto->dataLen++] = value;
            }
        }
    }
    return NULL;
}

void Safe_Send(uint16_t data){
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
	USART_SendData(USART1, data);
}


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

    #ifdef __STM32F429I_DISCOVERY
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
    #endif

    #ifdef __S0LENS_A
    // Send data serially
    while (i < firstSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        Slip_Write_Byte(*(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < secondSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        Slip_Write_Byte(*(wifi_packet->wifi_packet_msg+i));
        i++;
    }
    i = firstSize + sizeof(wifi_packet->wifi_packet_msg);
    while (i < thirdSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        Slip_Write_Byte(*(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < fourthSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        // Delay of 100 works for lab computers
        if (i % 100 == 0) {
            for (uint32_t j = 0; j < 1000; j++) {}
        }
        Slip_Write_Byte( *(wifi_packet->wifi_packet_data+i));
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
    USART_InitStructure.USART_Mode = (USART1->CR1 & (USART_CR1_RE | USART_CR1_TE)) | USART_Mode_Tx | USART_Mode_Rx;

    // USART configuration
    USART_Init(USART1, &USART_InitStructure);

	// Enable Rx interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	// Initialize interrupts
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    // Enable USART
    USART_Cmd(USART1, ENABLE);

	//Init WiFi Rx circular buffer
	USART_Buffer_Init(100);

	//Setup SLIP
	Slip_Init();

	//Sync with esp-link
	bool esp_status = Esp_Sync();

    #endif

	return ESP8266_INFO_OK;
}

bool Esp_Sync(){
	if(!syncing){
		Safe_Send(SLIP_END);

		//Sync request
		Slip_Request(CMD_SYNC, (uint32_t)&Esp_WifiCb, 0);
		Slip_Write((uint8_t*)&crc, 2);
		Safe_Send(SLIP_END);

		syncing = true;

		slip_packet_t *packet;
		while((packet = Slip_Wait_Return()) != NULL){
			if(packet->value == (uint32_t)&Esp_WifiCb){
				syncing = false;
				return true;
			}
		}
		syncing = false;
		return false;
	}
	return false;
}

void Slip_Request(uint16_t cmd, uint32_t value, uint16_t argc){
	crc = 0;
	Safe_Send(SLIP_END);
	Slip_Write(&cmd, 2);
	crc = crc16Data((unsigned const char*)&cmd, 2, crc);

	Slip_Write(&argc, 2);
	crc = crc16Data((unsigned const char*)&argc, 2, crc);

	Slip_Write(&value, 4);
	crc = crc16Data((unsigned const char*)&value, 4, crc);
}

slip_packet_t *Slip_Wait_Return(void){
	uint32_t count = 0;
	while(count < ESP_TIMEOUT){
		slip_packet_t *packet = Slip_Process();
		if(packet != NULL) return packet;
		count++;
	}
	return NULL;
}

void Esp_WifiCb(void *response){
	//Function that will be called when the wifi changes state
}

void Esp_ResetCb(void){
	//Callback that gets called when the esp gets reset and we need to sync
	bool ok = false;
	do {
		ok = Esp_Sync();
	} while(!ok);
}

uint32_t GetTime(){
	Slip_Request(CMD_GET_TIME, 0, 0);
	Slip_Write((uint8_t*)&crc, 2);
	Safe_Send(SLIP_END);


	slip_packet_t *pkt = Slip_Wait_Return();
	return pkt ? pkt->value : 0;
}
