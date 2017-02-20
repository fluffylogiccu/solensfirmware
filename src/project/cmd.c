/** @file cmd.c
 *  @brief Implemenation of the command module.
 *
 *  This contains the implementations of the 
 *  command module functions.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "cmd.h"
#include "err.h"
#include "sdram.h"
#include "cam.h"
#include "prof.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>
#include <stdlib.h>

#ifdef __CMD
#include "stm32f4xx_usart.h"
#endif

#ifdef __STM32F429I_DISCOVERY
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_sdram.h"
#endif

/* @brief instance of queue
 */
static cmd_queue_t *cmd_queue;

/* @brief initialize flag
 */
static uint8_t cmd_initialized;

#ifdef __CMD
/* @brief UART2 Rx cmd buffer
 */
static cmd_cmd_t *cmd_uartBuf = 0;

/* @brief UART2 Rx counter
 */
static uint32_t cmd_uartCtr;

/* @brief UART2 Rx total size
 */
static uint32_t cmd_uartTotal;

/* @brief UART2 Rx data size
 */
static uint16_t cmd_uartDataSize;

/* @brief UART2 Rx data start
 */
#define CMD_DATASTART 4

/* @brief UART2 Rx data len start
 */
#define CMD_DATALENSTART 2
#endif

/**************************************
 * Private functions
 */

cmd_status_t cmd_queueInit() {

    // Allocate space for queue
    cmd_queue = NULL;
    cmd_queue = (cmd_queue_t *) malloc(sizeof(cmd_queue_t));

    if (cmd_queue == NULL) {
        log_Log(CMD, CMD_ERR_MALLOC, "Couldn't initialize cmd_queue memory.\0");
        return CMD_ERR_MALLOC;
    } 

    cmd_queue->cmd_queue_buf = NULL;
    cmd_queue->cmd_queue_buf = (cmd_cmd_t **) malloc(sizeof(cmd_cmd_t *)*CMD_QUEUE_CAP);
    if (cmd_queue->cmd_queue_buf == NULL) {
        log_Log(CMD, CMD_ERR_MALLOC, "Couldn't initialize cmd_queue_buf memory.\0");
        return CMD_ERR_MALLOC;
    } 

    cmd_queue->cmd_queue_head = cmd_queue->cmd_queue_buf;
    cmd_queue->cmd_queue_tail = cmd_queue->cmd_queue_buf;
    cmd_queue->cmd_queue_capacity = CMD_QUEUE_CAP;
    cmd_queue->cmd_queue_size = 0;
    cmd_queue->cmd_queue_status = CMD_QUEUE_EMPTY;

    #ifndef __CMD
    cmd_initialized = 1;
    #endif

    return CMD_INFO_OK;
}

#ifdef __CMD
cmd_status_t cmd_uartInit() {

   // Initialize command buffer
    cmd_uartBuf = NULL;
    cmd_uartBuf = (cmd_cmd_t *) malloc(sizeof(cmd_cmd_t));
    
    // Check for failure
    if (cmd_uartBuf == NULL) {
        log_Log(CMD, CMD_ERR_MALLOC, "Couldn't initialize UART command buffer memory\0");
        return CMD_ERR_MALLOC;
    }

    // Set to initial values
    cmd_uartBuf->cmd_module = 0;
    cmd_uartBuf->cmd_func = 0;
    cmd_uartBuf->cmd_dataLen = 0;
    cmd_uartBuf->cmd_data = NULL;
    cmd_uartCtr = 0;
    cmd_uartTotal = CMD_DATASTART;
    cmd_uartDataSize = 0;

    // Structures for configuring
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // Enable USART2 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // Connect Pin D6 to USART2 Rx
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    
    // Configure USART Rx as alternate function
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Medium_Speed;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = CMD_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = (USART2->CR1 & (USART_CR1_RE | USART_CR1_TE)) | USART_Mode_Rx;

    // USART configuration
    USART_Init(USART2, &USART_InitStructure);

    // Enable Rx interrupts
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    // Initialize interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable USART2
    USART_Cmd(USART2, ENABLE);
    
    cmd_initialized = 1;

    return CMD_INFO_OK;

}

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
        uint8_t ch = USART2->DR;      

        if (cmd_uartCtr >= CMD_DATASTART) {
            // Save character to data buffer
            *(cmd_uartBuf->cmd_data + cmd_uartCtr - CMD_DATASTART) = ch;
        } else {
            // Save character to buffer
            *((uint8_t *)(cmd_uartBuf) + cmd_uartCtr) = ch;
        }
        // Get the first byte of data length
        if (cmd_uartCtr == CMD_DATALENSTART) {
            cmd_uartDataSize += ch;
        // Get the second byte
        } else if (cmd_uartCtr == CMD_DATALENSTART + 1) {
            cmd_uartDataSize += ch << 8;
            // We're done if no data
            if (cmd_uartDataSize != 0) {
                cmd_uartBuf->cmd_data = (uint8_t *) malloc(cmd_uartDataSize);
                cmd_uartTotal += cmd_uartDataSize;
            }
        }

        // Increase and check if we're done
        cmd_uartCtr++;        
        if (cmd_uartCtr >= cmd_uartTotal) {
            cmd_uartCtr = 0;
            cmd_uartTotal = CMD_DATASTART;
            cmd_uartDataSize = 0;
            cmd_status_t st = cmd_QueueGetStatus();
                // Check if there's room
            if (st != CMD_INFO_QUEUEEMPTY && st != CMD_INFO_QUEUEPARTIAL) {
                log_Log(CMD, st, "Bad queue state. Could not write command to queue.\0");
            } else {
                // Copy queue buffer to new array
                cmd_cmd_t *cmdCopy;
                st = cmd_CmdAllocate(&cmdCopy, cmd_uartBuf->cmd_dataLen);
                if (st != CMD_INFO_OK) {
                    log_Log(CMD, st, "Could not copy command. Could not write command to queue.\0");
                }
                cmdCopy->cmd_module = cmd_uartBuf->cmd_module;
                cmdCopy->cmd_func = cmd_uartBuf->cmd_func;
                cmdCopy->cmd_dataLen = cmd_uartBuf->cmd_dataLen;

                // Don't need the data that CmdAllocate made for this
                free(cmdCopy->cmd_data);
                cmdCopy->cmd_data = cmd_uartBuf->cmd_data;
              
                // Add command to queue
                st = cmd_QueuePut(cmdCopy);
                if (st != CMD_INFO_OK) {
                    log_Log(CMD, st, "Could not add command to queue.\0");
                }

                // Zero out cmd buffer
                cmd_uartBuf->cmd_module = 0;
                cmd_uartBuf->cmd_func = 0;
                cmd_uartBuf->cmd_dataLen = 0;
                cmd_uartBuf->cmd_data = NULL;               
            }         
        }
    }
}

#endif

/**************************************
 * Public functions
 */

cmd_status_t cmd_CmdDeallocate(cmd_cmd_t *cmd) {
    // Check for null issues
    if (cmd == NULL) {
        log_Log(CMD, CMD_WARN_FREE, "Attempting to free command that has already been freed.\0");
        return CMD_WARN_FREE;
    }
    if (cmd->cmd_data == NULL) {
        free(cmd);
        // We don't log a warning here because this is expected on commands with no data
        return CMD_WARN_FREE;
    }
    
    // Free the data
    free(cmd->cmd_data);
    free(cmd);

    return CMD_INFO_OK;
}

cmd_status_t cmd_CmdAllocate(cmd_cmd_t **cmd, uint16_t dataLen) {     
    if (cmd == NULL) {
        log_Log(CMD, CMD_ERR_NULLPTR, "cmd_CmdAllocate parameter cannot be NULL.\0");
        return CMD_ERR_NULLPTR;
    }

    // Allocate struct
    *cmd = NULL;
    *cmd = (cmd_cmd_t *) malloc(sizeof(cmd_cmd_t));
    if (*cmd == NULL) {
        log_Log(CMD, CMD_ERR_MALLOC, "Could not allocate command structure.\0");
        return CMD_ERR_MALLOC;
    }

    // Allocate data in struct
    (*cmd)->cmd_data = NULL;
    if (dataLen != 0) {
        (*cmd)->cmd_data = (uint8_t *) malloc(dataLen);
        if ((*cmd)->cmd_data == NULL) {
            log_Log(CMD, CMD_ERR_MALLOC, "Could not allocate command data.\0");
            return CMD_ERR_MALLOC;
        }
    }

    // Set to defaults
    (*cmd)->cmd_module = 0;
    (*cmd)->cmd_func = 0;
    (*cmd)->cmd_dataLen = dataLen;

    return CMD_INFO_OK;
}

cmd_status_t cmd_QueueGetStatus() {
    switch (cmd_queue->cmd_queue_status) {
        case CMD_QUEUE_EMPTY:
            return CMD_INFO_QUEUEEMPTY;
        case CMD_QUEUE_FULL:
            return CMD_INFO_QUEUEFULL;
        case CMD_QUEUE_PARTIAL:
            return CMD_INFO_QUEUEPARTIAL;
        default:
            return CMD_ERR_QUEUEINVALID;
    }    
}

cmd_status_t cmd_QueuePut(cmd_cmd_t *cmd) {
    // Check for full queue
    if (cmd_queue->cmd_queue_status == CMD_QUEUE_FULL) {
        log_Log(CMD, CMD_ERR_QUEUEFULL);
        return CMD_ERR_QUEUEFULL;
    }
    if (cmd_queue->cmd_queue_status == CMD_QUEUE_INVALID) {
        log_Log(CMD, CMD_ERR_QUEUEINVALID);
        return CMD_ERR_QUEUEINVALID;
    }

    // Set data
    *cmd_queue->cmd_queue_head = cmd;

    // Increment head and check for buffer wrap
    cmd_queue->cmd_queue_head++;
    if ((cmd_queue->cmd_queue_head - cmd_queue->cmd_queue_buf) >= 
         cmd_queue->cmd_queue_capacity) {

        cmd_queue->cmd_queue_head -= cmd_queue->cmd_queue_capacity;
    }
    cmd_queue->cmd_queue_size++;

    // Set new state
    if (cmd_queue->cmd_queue_size == cmd_queue->cmd_queue_capacity ||
        cmd_queue->cmd_queue_head == cmd_queue->cmd_queue_tail) {

        cmd_queue->cmd_queue_status = CMD_QUEUE_FULL;
    } else {
        cmd_queue->cmd_queue_status = CMD_QUEUE_PARTIAL;
    }

    return CMD_INFO_OK;
}

cmd_status_t cmd_QueueGet(cmd_cmd_t **cmd) {
    // Check for valud cmd pointer
    if (cmd == NULL) {
        log_Log(CMD, CMD_ERR_NULLPTR, "Invalid command pointer in parameter list for cmd_QueueGet.\0");
        return CMD_ERR_NULLPTR;
    }

    // Check if queue is empty
    if (cmd_queue->cmd_queue_status == CMD_QUEUE_EMPTY) {
        log_Log(CMD, CMD_ERR_QUEUEEMPTY, "Trying to get from an empty command queue.\0");
        return CMD_ERR_QUEUEEMPTY;
    }

    // Set command
    *cmd = *cmd_queue->cmd_queue_tail;

    // Increment tail and check for wrap
    cmd_queue->cmd_queue_tail++;
    if ((cmd_queue->cmd_queue_tail - cmd_queue->cmd_queue_buf) >=
         cmd_queue->cmd_queue_capacity) {

        cmd_queue->cmd_queue_tail -= cmd_queue->cmd_queue_capacity;
    }
    cmd_queue->cmd_queue_size--;

    // Set new state
    if (cmd_queue->cmd_queue_size == 0) {
        cmd_queue->cmd_queue_status = CMD_QUEUE_EMPTY;
    } else {
        cmd_queue->cmd_queue_status = CMD_QUEUE_PARTIAL;
    }

    return CMD_INFO_OK;
}

cmd_status_t cmd_Init() {
    // Check initialization flag
    if (cmd_initialized == 1) {
        return CMD_WARN_ALINIT;
    }

#ifdef __CMD
    cmd_status_t ret = cmd_queueInit();
    if (ret != CMD_INFO_OK) {
        return ret;
    }
    return cmd_uartInit();
#else 
    return cmd_queueInit();
#endif

}

cmd_status_t cmd_Loop() {
    cmd_cmd_t *cmd;
    cmd_status_t st;
    while(1) {
        // Spin on empty queue
        while (cmd_QueueGetStatus() == CMD_INFO_QUEUEEMPTY) {}
        if (cmd_QueueGetStatus() == CMD_ERR_QUEUEINVALID) {
            log_Log(CMD, CMD_ERR_QUEUEINVALID, "Command Queue entered invalid state.\0");
            return CMD_ERR_QUEUEINVALID;
        } 

        // Get function
        cmd_QueueGet(&cmd);

        // Route function request
        switch (cmd->cmd_module) {
        case LOG:
            switch (cmd->cmd_func) {
                #ifdef __LOG
                case LOG_FUNC_INIT:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Initialization command should not have data.\0");
                    }
                    log_status_t log_st = log_Init();
                    if (log_st == LOG_INFO_OK) {
                        log_Log(LOG, LOG_INFO_OK, "Initialized Logger.\0");
                    } else if (log_st == LOG_WARN_ALINIT){
                        log_Log(LOG, LOG_WARN_ALINIT, "Logger is already initialized.\0");
                    } else {                       
                        log_Log(LOG, log_st, "Failed to initialize log.\0");
                    }
                    
                    break;
                #endif
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a log function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));
                    break;
 
            }
            break;
        case CMD:
            switch (cmd->cmd_func) {
                case CMD_FUNC_INIT:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Initialization command should not have data.\0");
                    }
                    cmd_status_t cmd_st = cmd_Init();
                    if (cmd_st == CMD_INFO_OK) {
                        log_Log(CMD, CMD_INFO_OK, "Initialized command module.\0");
                    } else if (cmd_st == CMD_WARN_ALINIT) {
                        log_Log(CMD, CMD_WARN_ALINIT, "Command module already initialized.\0");
                    } else {
                        log_Log(CMD, cmd_st, "Failed to initialize log\0");
                    }                    
                    break;
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a log function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));
                    break;
            }
            break;
        case STDLIB:
            switch (cmd->cmd_func) {
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a stdlib function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));        
                }
                break;
        case SDRAM:
            switch (cmd->cmd_func) {
                case SDRAM_FUNC_INIT:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Initialization command should not have data.\0");
                    }
                    sdram_status_t sd_st = sdram_Init();
                    if (sd_st == SDRAM_INFO_OK) {
                        log_Log(SDRAM, SDRAM_INFO_OK, "Initialized SDRAM interface.\0");
                    } else if (sd_st == SDRAM_WARN_ALINIT) {
                        log_Log(SDRAM, SDRAM_WARN_ALINIT, "SDRAM already initialized.\0");
                    } else {
                        log_Log(SDRAM, sd_st, "Failed to initialize SDRAM.\0");
                    }                    
                    break;
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call an SDRAM function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));
                    break;
 
            }
            break;
        case PROF:
            switch (cmd->cmd_func) {
                case PROF_FUNC_INIT:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Initialization command should not have data.\0");
                    }
                    prof_status_t p_st = prof_Init();
                    if (p_st == PROF_INFO_OK) {
                        log_Log(PROF, PROF_INFO_OK, "Initialized profiler.\0");
                    } else if (p_st == PROF_WARN_ALINIT) {
                        log_Log(PROF, PROF_WARN_ALINIT, "Profiler already initialized.\0");
                    } else {
                        log_Log(PROF, p_st, "Failed to initialize profiler.\0");
                    }                    
                    break;
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a profiler function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));
                    break;
            }
            break;
        case CAM:
            switch (cmd->cmd_func) {
                case CAM_FUNC_INIT:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Initialization command should not have data.\0");
                    }
                    cam_status_t c_st = cam_Init();
                    if (c_st == CAM_INFO_OK) {
                        log_Log(CAM, CAM_INFO_OK, "Initialized camera module.\0");
                    } else if (c_st == CAM_WARN_ALINIT) {
                        log_Log(CAM, CAM_WARN_ALINIT, "Camera module already initialized.\0");
                    } else {
                        log_Log(CAM, c_st, "Failed to initialize camera.\0");
                    }                    
                    break;
                case CAM_FUNC_CONFIG:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Configuration command should not have data.\0");
                    }
                    c_st = cam_Configure();
                    if (c_st == CAM_INFO_OK) {
                        log_Log(CAM, CAM_INFO_OK, "Configured camera module.\0");
                    } else if (c_st == CAM_WARN_ALCONF) {
                        log_Log(CAM, CAM_WARN_ALCONF, "Camera module already configured.\0");
                    } else {
                        log_Log(CAM, c_st, "Failed to configure camera.\0");
                    }                    
                    break;
                case CAM_FUNC_CAPTURE:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Camera capture command should not have data.\0");
                    }
                    c_st = cam_Capture();
                    if (c_st == CAM_INFO_OK) {
                        log_Log(CAM, CAM_INFO_OK, "Captured an image into SDRAM.\0");
                    } else {
                        log_Log(CAM, c_st, "Could not capture image.\0");
                    }                    
                    break;
                case CAM_FUNC_TRANSFER:
                    if (cmd->cmd_data != 0) {
                        log_Log(CMD, CMD_ERR_DATA, "Camera transfer command should not have data.\0");
                    }
                    c_st = cam_Transfer();
                    if (c_st == CAM_INFO_OK) {
                        log_Log(CAM, CAM_INFO_OK, "Successfully transferred image.\0");
                    } else {
                        log_Log(CAM, c_st, "Could not transfer image to debug interface.\0");
                    }                    
                    break;
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a camera function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));
                    break;
            }
            break;
        case ESP8266:
            switch (cmd->cmd_func) {
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call an ESP8266 function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));        
                }
                break;
        case WIFI:
            switch (cmd->cmd_func) {
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a wifi function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));        
                }
                break;
        case TEST:
            switch (cmd->cmd_func) {
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call a test function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));        
                }
                break;
        case OV5642:
            switch (cmd->cmd_func) {
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call an ov5642 function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));        
                }
                break;
         case OV7670:
            switch (cmd->cmd_func) {
                default:
                    log_Log(CMD, CMD_ERR_NOFUNC, "Tried to call an 0v7670 function that doesn't exist.\0", 
                            1, &(cmd->cmd_func));        
                }
                break;


        default:
            log_Log(CMD, CMD_ERR_NOMOD, "Tried to send a command to an unknown module.\0");
            break;
        }

        // Free memory, pass warning about freeing freed memory
        st = cmd_CmdDeallocate(cmd);
        if (st != CMD_INFO_OK && st != CMD_WARN_FREE) {
            log_Log(CMD, st);
            return st;
        }
    }
}

