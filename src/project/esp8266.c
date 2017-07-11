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
#include "sleep.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "wifi.h"
#include <stdint.h>
#include "log.h"
#include <string.h>


slip_protocol_t proto;
uint8_t protoBuf[128];
uart_buffer_t rxBuffer;
uint16_t crc;
bool syncing = false;
bool wifiConnected = false;
bool mqtt_connected = false;
bool recieved_start_ack = false;
bool recieved_end_ack = false;
bool recieved_data_ack = false;
bool initial_callback_flag = false;

/**************************************
 * Private functions
 */

esp8266_status_t esp8266_setRTC() {
	RTC_TimeTypeDef RTC_TimeStructure;
    RTC_InitTypeDef RTC_InitStructure;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    log_Log(SLEEP, SLEEP_INFO_OK, "RTC initialization.\0");

    RTC_ClearITPendingBit(RTC_IT_WUT);
    EXTI_ClearITPendingBit(EXTI_Line22);

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset RTC Domain */
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);

    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    log_Log(SLEEP, SLEEP_INFO_OK, "LSE clock ready.\0");

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC Clock */
     RCC_RTCCLKCmd(ENABLE);

     /* Wait for RTC APB registers synchronisation */
     RTC_WaitForSynchro();

    /* Configure the RTC data register and RTC prescaler */
    /* ck_spre(1Hz) = RTCCLK(LSI) /(AsynchPrediv + 1)*(SynchPrediv + 1)*/
    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);

    /* Set the time to 00h 00mn 00s AM */
    RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
    RTC_TimeStructure.RTC_Hours   = 0;
    RTC_TimeStructure.RTC_Minutes = 0;
    RTC_TimeStructure.RTC_Seconds = 30;
    RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

    /*RTC_DateStructure.RTC_Year = 117;
    RTC_DateStructure.RTC_Month = 7;
    RTC_DateStructure.RTC_Date = 1;
    RTC_DateStructure.RTC_WeekDay = info->tm_wday;
    RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure); */

    /* Disable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    /* Set the alarm for every minute at 0 sec */
    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 6;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 7;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0; //info_next->tm_sec;
    RTC_AlarmStructure.RTC_AlarmDateWeekDay = 1;
    RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours | RTC_AlarmMask_Minutes;

    /* Configure the RTC Alarm A register */
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    /* Enable RTC Alarm A Interrupt */
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    /* Enable the alarm */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

    RTC_ClearFlag(RTC_FLAG_ALRAF);

	return ESP8266_INFO_OK;

}

esp8266_status_t esp8266_Slip_Init(){
	proto.buf = protoBuf;
	proto.bufSize = sizeof(protoBuf);
	proto.dataLen = 0;
	proto.isEsc = 0;
	return ESP8266_INFO_OK;
}

void esp8266_Write_Slip_Byte(uint8_t data){
	switch(data){
		case SLIP_END:
		esp8266_Raw_Send(SLIP_ESC);
		esp8266_Raw_Send(SLIP_ESC_END);
		break;
		case SLIP_ESC:
		esp8266_Raw_Send(SLIP_ESC);
		esp8266_Raw_Send(SLIP_ESC_ESC);
		break;
		default:
		esp8266_Raw_Send(data);
	}
}

void esp8266_Write_Slip(void *data, uint16_t len){
	uint8_t *d = (uint8_t*)data;
	while(len--){
		esp8266_Write_Slip_Byte(*d++);
	}
}

esp8266_status_t USART_Buffer_Init(uint16_t capacity){
	rxBuffer.capacity = capacity;
	rxBuffer.count = 0;
	rxBuffer.buf = (uint8_t*)malloc(capacity * 8);
	rxBuffer.head = rxBuffer.buf;
	rxBuffer.tail = rxBuffer.buf;
	rxBuffer.bufEnd = rxBuffer.buf + capacity * sizeof(uint8_t);

	return ESP8266_INFO_OK;
}

esp8266_status_t USART_Buffer_Free(){
	free(rxBuffer.buf);
	rxBuffer.capacity = 0;
	rxBuffer.count = 0;
	rxBuffer.head = NULL;
	rxBuffer.tail = NULL;
	rxBuffer.bufEnd = NULL;

	return ESP8266_INFO_OK;
}

uint16_t USART_Buffer_Available(){
	if(rxBuffer.buf != NULL){
		return rxBuffer.count;
	}
	else{
		//Buffer not initialized
		return 0;
	}
}

void USART_Buffer_Push(uint8_t data){
	if(rxBuffer.count == rxBuffer.capacity){
		//Buffer is full
		return;
	}
	*(rxBuffer.head) = data;
	rxBuffer.head++;
	if(rxBuffer.head == rxBuffer.bufEnd){
		rxBuffer.head = rxBuffer.buf;
	}
	rxBuffer.count++;
}

uint8_t USART_Buffer_Pop(void){
	if(rxBuffer.count == 0){
		//Buffer is empty
		return -1;
	}
	uint8_t item = *(rxBuffer.tail);
	rxBuffer.tail++;
	if(rxBuffer.tail == rxBuffer.bufEnd){
		rxBuffer.tail = rxBuffer.buf;
	}
	rxBuffer.count--;
	return item;
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

slip_packet_t *esp8266_protoCompletedCb(void){
    slip_packet_t *packet = (slip_packet_t*)proto.buf;
    uint16_t crc = crc16Data(proto.buf, proto.dataLen-2, 0);
	slip_response_t resp;
    uint16_t resp_crc = *(uint16_t*)(proto.buf+proto.dataLen-2);
    if(crc != resp_crc){
        return NULL;
    }
    switch(packet->cmd){
        case CMD_RESP_V:
            return packet;
        case CMD_RESP_CB:
            //Here is where they return a callback function
			//They do some crazy stuff with function pointers and I am
			//content with this if statement since we have limited funciton
			//callbacks to keep track of

			packet_to_response(packet, &resp);
			if(packet->value == (uint32_t)&esp8266_WifiCb){
				wifi_status_t wifi_status;
				if (initial_callback_flag == 0) {
					wifi_status = esp8266_WifiCb(&resp);
					initial_callback_flag = 1;
				} else {
				/* if we lose connectivity go back to sleep */
					sleep_Standby();
				}
				log_Log(WIFI, wifi_status);
			} else if(packet->value == (uint32_t)&mqtt_connected_callback){
				mqtt_connected_callback(&resp);
			} else if(packet->value == (uint32_t)&mqtt_disconnnected_callback){
				mqtt_disconnnected_callback(&resp);
			} else if(packet->value == (uint32_t)&mqtt_published_callback){
				mqtt_published_callback(&resp);
			} else if(packet->value == (uint32_t)&mqtt_data_callback){
				mqtt_data_callback(&resp);
			}
            return NULL;
        case CMD_SYNC:
            //esp-link not in sync
            esp8266_ResetCb();
            return NULL;
        default:
            //command not implemented
			log_Log(ESP8266, ESP8266_ERR_UNKNOWN, "Unknown Command\0");
            return NULL;
    }
}

slip_packet_t *esp8266_Process(){
    uint16_t value;
    while(USART_Buffer_Available() > 0){
        value = USART_Buffer_Pop();
        if(value == SLIP_ESC){
            proto.isEsc = 1;
        } else if (value == SLIP_END){
            slip_packet_t *packet = proto.dataLen >= 8 ? esp8266_protoCompletedCb() : 0;
            proto.dataLen = 0;
            proto.isEsc = 0;
            if(packet != NULL)return packet;
        } else {
            if(proto.isEsc){
                if(value == SLIP_ESC_END) value = SLIP_END;
                if(value == SLIP_ESC_ESC) value = SLIP_ESC;
                proto.isEsc = 0;
            }
            if(proto.dataLen < proto.bufSize){
                proto.buf[proto.dataLen++] = value;
            }
        }
    }
    return NULL;
}

void packet_to_response(slip_packet_t *packet, slip_response_t *resp){
	resp->_cmd = packet;
	resp->_arg_ptr = packet->args;
	resp->_arg_num = 0;
}

int16_t popArg(slip_response_t* resp, void* d, uint16_t maxLen){
	if(resp->_arg_num >= resp->_cmd->argc) return -1;

	uint16_t len = *(uint16_t*)resp->_arg_ptr;
	uint16_t pad = (4-((len+2)&3))&3;
	resp->_arg_ptr += 2;
	resp->_arg_num++;

	uint8_t *data = (uint8_t *)d;
	uint16_t l = len > maxLen ? maxLen : len;
	uint8_t *p = resp->_arg_ptr;
	while(l--)
		*data++ = *p++;

	resp->_arg_ptr += len + pad;
	return len;
}

char* popString(slip_response_t *resp){

	uint16_t len = *(uint16_t*)resp->_arg_ptr;
	uint16_t pad = (4-((len+2)&3))&3;
	resp->_arg_ptr += 2;
	resp->_arg_num++;

	char* ret = (char*)malloc(sizeof(char)*len);

	strcpy(ret, (char*)resp->_arg_ptr);

	resp->_arg_ptr += pad;
	return ret;

}

esp8266_status_t esp8266_Raw_Send(uint16_t data){
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
	USART_SendData(USART1, data);
	return ESP8266_INFO_OK;
}


/**************************************
 * Public functions
 */

esp8266_status_t esp8266_Send(wifi_packet_t *wifi_packet) {
	#ifdef __STM32F429I_DISCOVERY
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
        
        // if (i % 10 == 0) {
        //     for (uint32_t j = 0; j < 5000; j++) {}
        // }


        USART_SendData(USART1, *(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < secondSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}

        // if (i % 10 == 0) {
        //     for (uint32_t j = 0; j < 5000; j++) {}
        // }
            
        USART_SendData(USART1, *(wifi_packet->wifi_packet_msg+i));
        i++;
    }
    i = firstSize + sizeof(wifi_packet->wifi_packet_msg);
    while (i < thirdSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        

        // if (i % 10 == 0) {
        //     for (uint32_t j = 0; j < 5000; j++) {}
        // }
        
        USART_SendData(USART1, *(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < fourthSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        // Delay of 100 works for lab computers
        // How did this number happen?????
        if (i % 10 == 0) {
            for (uint32_t j = 0; j < 5000; j++) {}
        }
        USART_SendData(USART1, *(wifi_packet->wifi_packet_data+i));
        i++;
    }
    #endif

    #ifdef __S0LENS_A
    // Send data serially
	/*
    while (i < firstSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        esp8266_Write_Slip_Byte(*(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < secondSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        esp8266_Write_Slip_Byte(*(wifi_packet->wifi_packet_msg+i));
        i++;
    }
    i = firstSize + sizeof(wifi_packet->wifi_packet_msg);
    while (i < thirdSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        esp8266_Write_Slip_Byte(*(((uint8_t *)wifi_packet)+i));
        i++;
    }
    i = 0;
    while (i < fourthSize) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        // Delay of 100 works for lab computers
        if (i % 100 == 0) {
            for (uint32_t j = 0; j < 1000; j++) {}
        }
        esp8266_Write_Slip_Byte( *(wifi_packet->wifi_packet_data+i));
        i++;
    }
	*/

	//char meep = 'e';
	uint8_t meep = 0x55;
	recieved_start_ack = false;
	mqtt_publish("img/start", &meep, 1, 0, 0);
	int attempt = 0;
	while(true){
		esp8266_Wait_Return(100000);
		if(recieved_start_ack){
			break;
		} else if (attempt >= MAX_ATTEMPT){
			log_Log(WIFI, WIFI_WARN_NO_SERVER_RESPONSE, "No response. going to sleep\0");
			sleep_Standby();
		} else {
			attempt++;
		}
	}

	uint8_t *dataPointer = wifi_packet->wifi_packet_data;
	volatile uint32_t dataLen = 0;

	uint32_t i = 0;
	/*Make sure MAX_PACKET_SIZE is a multiple of 2 * image width*/
	for(i = 0; i < wifi_packet->wifi_packet_dataLen; i+=MAX_PACKET_SIZE){
		recieved_data_ack = false;
		dataLen = (wifi_packet->wifi_packet_dataLen - i ) > MAX_PACKET_SIZE ? MAX_PACKET_SIZE : (wifi_packet->wifi_packet_dataLen - i);
		mqtt_publish("img/data", dataPointer, dataLen, 0, 0);
		dataPointer += dataLen;
		attempt = 0;
		while(true){
			esp8266_Wait_Return(1000);
			if(recieved_data_ack){
				break;
			} else if (attempt >= MAX_ATTEMPT){
				log_Log(WIFI, WIFI_WARN_NO_SERVER_RESPONSE, "No response. going to sleep\0");
				sleep_Standby();
			} else if(attempt == RETRY_ATTEMPT){
				mqtt_publish("img/data", dataPointer, dataLen, 0, 0);
				attempt++;
			} else {
				attempt++;
			}
		}

	}
	char merp = 'f';
	mqtt_publish("img/end", (uint8_t*)&merp, 1, 0, 0);
	esp8266_Wait_Return(ESP_TIMEOUT);

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
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

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

    // Connect Pin B7 to USART1 Rx
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    // Configure USART Tx as alternate function
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Medium_Speed;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

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
	USART_Buffer_Init(1000);

	//Setup SLIP
	esp8266_Slip_Init();


	//Sync with esp-link
	bool ok;
	do{
		ok = esp8266_Sync();
		if(!ok) log_Log(WIFI, WIFI_ERR_UNKNOWN, "Unable to sync.\0");
		//sleep_Standby();
	} while(!ok);

	do{
		mqtt_setup();
		esp8266_Wait_Return(ESP_TIMEOUT);
	} while(!mqtt_connected);

	esp8266_Wait_Return(ESP_TIMEOUT);

	uint8_t buf[] = "Mornin'";
	mqtt_publish("wake-up", buf, sizeof(buf), 2, 0);
	esp8266_Wait_Return(ESP_TIMEOUT);

	mqtt_subscribe("sunrise", 1);
	//Ack channels for the image recieve
	mqtt_subscribe("img/start/ack", 0);
	mqtt_subscribe("img/data/ack", 0);
	mqtt_subscribe("img/end/ack", 0);

    #endif

	return ESP8266_INFO_OK;
}

bool esp8266_Sync(){
	if(!syncing){
		esp8266_Raw_Send(SLIP_END);

		//Sync request
		esp8266_Request(CMD_SYNC, (uint32_t)&esp8266_WifiCb, 0);
		esp8266_Write_Slip((uint8_t*)&crc, 2);
		esp8266_Raw_Send(SLIP_END);

		syncing = true;

		slip_packet_t *packet;
		while((packet = esp8266_Wait_Return(ESP_TIMEOUT)) != NULL){
			if(packet->value == (uint32_t)&esp8266_WifiCb){
				syncing = false;
				return true;
			}
		}
		syncing = false;
		return false;
	}
	return false;
}

void esp8266_Request(uint16_t cmd, uint32_t value, uint16_t argc){
	crc = 0;
	esp8266_Raw_Send(SLIP_END);
	esp8266_Write_Slip(&cmd, 2);
	crc = crc16Data((unsigned const char*)&cmd, 2, crc);

	esp8266_Write_Slip(&argc, 2);
	crc = crc16Data((unsigned const char*)&argc, 2, crc);

	esp8266_Write_Slip(&value, 4);
	crc = crc16Data((unsigned const char*)&value, 4, crc);
}

void esp8266_Request2(const void* data, uint16_t len){
	uint8_t *d = (uint8_t*)data;

	//write the length
	esp8266_Write_Slip(&len, 2);
	crc = crc16Data((unsigned const char*)&len, 2, crc);

	//output the data
	for(uint16_t l=len; l>0; l--){
		esp8266_Write_Slip_Byte(*d);
		crc = crc16Add(*d, crc);
		d++;
	}

	//output padding
	uint16_t pad = (4-(len&3))&3;
	uint8_t temp = 0;
	while(pad--){
		esp8266_Write_Slip_Byte(temp);
		crc = crc16Add(temp, crc);
	}

}

void esp8266_Request0(void){
	esp8266_Write_Slip((uint8_t*)&crc, 2);
	esp8266_Raw_Send(SLIP_END);
}

slip_packet_t *esp8266_Wait_Return(uint32_t timeout){
	uint32_t count = 0;
	while(count < timeout){
		slip_packet_t *packet = esp8266_Process();
		if(packet != NULL) return packet;
		count++;
	}
	return NULL;
}

wifi_status_t esp8266_WifiCb(slip_response_t *response){
	//Function that will be called when the wifi changes state
	if(response->_cmd->argc == 1){
		log_Log(WIFI, WIFI_INFO_OK, "Wifi callback\0");
		uint8_t status;
		popArg(response, &status, 1);
		switch (status) {
			case STATION_IDLE:
				wifiConnected = false;
				return WIFI_INFO_IDLE;
			case STATION_CONNECTING:
				wifiConnected = false;
				return WIFI_INFO_CONNECTING;
			case STATION_WRONG_PASSWORD:
				wifiConnected = false;
				return WIFI_WARN_WRONG_PASSWORD;
			case STATION_NO_AP_FOUND:
				wifiConnected = false;
				return WIFI_WARN_NO_AP_FOUND;
			case STATION_CONNECT_FAIL:
				wifiConnected = false;
				return WIFI_ERR_CONNECT_FAIL;
			case STATION_GOT_IP:
				wifiConnected = true;
				log_Log(WIFI, WIFI_INFO_GOT_IP, "Wifi connected!\0");
				return WIFI_INFO_GOT_IP;
			default:
				wifiConnected = false;
				return WIFI_INFO_UNKNOWN;
		}
	}
	return WIFI_INFO_UNKNOWN;
}

void esp8266_ResetCb(void){
	//Callback that gets called when the esp gets reset and we need to sync
	bool ok = false;
	//do {
		ok = esp8266_Sync();
		log_Log(WIFI, WIFI_INFO_UNKNOWN, "Esp needs sync!");
	//} while(!ok);
}

uint32_t esp8266_GetTime(){
	esp8266_Request(CMD_GET_TIME, 0, 0);
	esp8266_Request0();


	slip_packet_t *pkt = esp8266_Wait_Return(ESP_TIMEOUT);
	return pkt ? pkt->value : 0;
}

/* MQTT */
void mqtt_setup(void){
	esp8266_Request(CMD_MQTT_SETUP, 0, 4);
	uint32_t cb = (uint32_t)&mqtt_connected_callback;
	esp8266_Request2(&cb, 4);
	cb = (uint32_t)&mqtt_disconnnected_callback;
	esp8266_Request2(&cb, 4);
	cb = (uint32_t)&mqtt_published_callback;
	esp8266_Request2(&cb, 4);
	cb = (uint32_t)&mqtt_data_callback;
	esp8266_Request2(&cb, 4);
	esp8266_Request0();
}

void mqtt_lwt(const char* topic, const char* message, uint8_t qos, uint8_t retain){
	esp8266_Request(CMD_MQTT_LWT, 0, 4);
	esp8266_Request2(topic, strlen(topic));
	esp8266_Request2(message, strlen(message));
	esp8266_Request2(&qos, 1);
	esp8266_Request2(&retain, 1);
	esp8266_Request0();

}

void mqtt_subscribe(const char* topic, uint8_t qos){
	esp8266_Request(CMD_MQTT_SUBSCRIBE, 0, 2);
	esp8266_Request2(topic, strlen(topic));
	esp8266_Request2(&qos, 1);
	esp8266_Request0();
}

void mqtt_publish(const char* topic, const uint8_t* data, const uint16_t len, uint8_t qos, uint8_t retain){
	esp8266_Request(CMD_MQTT_PUBLISH, 0, 5);
	esp8266_Request2(topic, strlen(topic));
	esp8266_Request2(data, len);
	esp8266_Request2(&len, 2);
	esp8266_Request2(&qos, 1);
	esp8266_Request2(&retain, 1);
	esp8266_Request0();
}

/*	These are the client specific callback functions
*	They are business logic to deal with mqtt events
*	and most will just log information.
*/
void mqtt_connected_callback(void *response){
	mqtt_connected = true;
	log_Log(WIFI, WIFI_INFO_OK, "MQTT connected!\0");
}


void mqtt_disconnnected_callback(void *response){
	mqtt_connected = false;
	log_Log(WIFI, WIFI_INFO_OK, "MQTT disconnected :'(\0");
}

void mqtt_published_callback(void *response){
	log_Log(WIFI, WIFI_INFO_OK, "MQTT published.\0");
}

void mqtt_data_callback(slip_response_t *response){
	char* topic = popString(response);
	char* img_start_ack = "img/start/ack";
	char* img_data_ack = "img/data/ack";
	char* img_end_ack = "img/end/ack";
	if(strcmp(topic, img_start_ack) == 0){
		recieved_start_ack = true;
	} else if(strcmp(topic, img_data_ack) == 0){
		recieved_data_ack = true;
	} else if(strcmp(topic, img_end_ack)){
		recieved_end_ack = true;
	} else {
		log_Log(WIFI, WIFI_INFO_OK, topic);
	}
	free(topic);
}
