/** @file cmd.h
 *  @brief Function prototypes for the command module.
 *
 *  This contains the prototypes, macros, constants,
 *  and global variables for the command module.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __CMD_H
#define __CMD_H

/*************************************
 * @name Includes and definitions
 */

#include <stdint.h>
#include "log.h"
#include "err.h"
#include "mod.h"

#define CMD_QUEUE_CAP 256
#define CMD_BAUDRATE 115200//921600

/* @brief general function that other function enums
 * can be cast to
 */
typedef uint8_t gen_func_t;

/* @brief log functions
 */
typedef enum log_func_e {
    LOG_FUNC_INIT,
} log_func_t;

/* @brief cmd functions
 */
typedef enum cmd_func_e {
    CMD_FUNC_INIT,
} cmd_func_t;

/* @brief stdlib functions
 */
typedef enum stdlib_func_e {
    STDLIB_FUNC_DUMMY,   
} stdlib_func_t;

/* @brief stdlib functions
 */
typedef enum sdram_func_e {
    SDRAM_FUNC_INIT,   
} sdram_func_t;

/* @brief ov5642 functions
 */
typedef enum ov5642_func_e {
    ov5642_FUNC_DUMMY,   
} ov5642_func_t;

/* @brief ov7670 functions
 */
typedef enum ov7670_func_e {
    OV7670_FUNC_DUMMY,   
} ov7670_func_t;

/* @brief profiler functions
 */
typedef enum prof_func_e {
    PROF_FUNC_INIT,   
} prof_func_t;

/* @brief test functions
 */
typedef enum test_func_e {
    TEST_FUNC_DUMMY,   
} test_func_t;

/* @brief camera functions
 */
typedef enum cam_func_e {
    CAM_FUNC_INIT,
    CAM_FUNC_CONFIG,
    CAM_FUNC_CAPTURE,
    CAM_FUNC_TRANSFER,   
} cam_func_t;

/* @brief command structure
 */
typedef struct __attribute__ ((packed)) cmd_cmd_s {
    mod_t cmd_module;
    gen_func_t cmd_func;
    uint16_t cmd_dataLen;
    uint8_t *cmd_data;
} cmd_cmd_t;

/* @brief queue state enumeration
 */
typedef enum cmd_queue_state_e {
    CMD_QUEUE_EMPTY,
    CMD_QUEUE_FULL,
    CMD_QUEUE_PARTIAL,
    CMD_QUEUE_INVALID
} cmd_queue_state_t;

/* @brief command queue structure
 */
typedef struct __attribute__ ((packed)) cmd_queue_s {
    cmd_cmd_t **cmd_queue_buf;
    cmd_cmd_t **cmd_queue_head;
    cmd_cmd_t **cmd_queue_tail;

    uint16_t cmd_queue_capacity;
    uint16_t cmd_queue_size;

    cmd_queue_state_t cmd_queue_status;

} cmd_queue_t;

/**************************************
 * @name Private functions
 */

/** @brief Initialize the queue
 *
 *  Initializes the cmd_queue 
 *
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_queueInit();

#ifdef __CMD
/** @brief This function initializes the UART Rx
 *
 *  The directive __CMD must be set to use this function.
 *
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_uartInit();

/** @brief UART Rx interrupt handler
 *
 *  The interrupt handler saves a command into a command
 *  buffer, checking if we have recieved the correct 
 *  number of bytes. It keeps track of the total, adjusting
 *  based on the data length parameter. When a full command
 *  is recieved, it is put into the command queue to be 
 *  executed.
 */
#ifdef __STM32F429I_DISCOVERY
void USART2_IRQHandler(void);
#endif
#ifdef __S0LENS_A
void UART4_IRQHandler(void);
#endif

#endif

/**************************************
 * @name Public functions
 */

/** @brief Deallocate command memory at the command pointer
 *
 *  This function will deallocate memory for the cmd_cmd_t
 *  structure and the cmd_data field.
 *
 *  @param cmd a pointer to the command to free
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_CmdDeallocate(cmd_cmd_t *cmd);

/** @brief Allocate command memory at the command pointer
 * 
 *  Note: When the command is finally executed, the memory will
 *  be freed. 
 *
 *  This function will allocate memory for the cmd_cmd_t
 *  structure and the cmd_data field, both of which will 
 *  be freed when the command is executed. You can free
 *  the command memory prematurely by calling 
 *  cmd_CmdDeallocate() but don't do this if you you are
 *  planning to put the command into the queue!
 *
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_CmdAllocate(cmd_cmd_t **cmd, uint16_t dataLen);

/** @brief Get the status of the queue
 *
 *  This will return a status code that will tell the 
 *  caller if the cmd_queue is empty, full, partial, or 
 *  in an invalid state. 
 *
 *  @param cmd the address of a pointer to a cmd
 *  @param dataLen data length to allocate
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_QueueGetStatus();

/** @brief Put a command in the queue.
 * 
 *  This function puts a command in the queue if the 
 *  queue is not full.
 *
 *  NOTE: For the command to get executed correctly, the
 *  caller must allocate space on the heap for both the 
 *  command and the command data! If the command passed in
 *  is not allocated, the command loop will try to execute
 *  on garbage memory!
 *
 *  Example call:
 *      cmd_cmd_t *cmd = (cmd_cmd_t *) malloc(sizeof(cmd_cmd_t));
 *      cmd->cmd_module = LOG;
 *      cmd->cmd_function = LOG_FUNC_LOG;
 *      cmd->cmd_dataLen = 2;
 *      cmd->cmd_data = (uint8_t *) malloc(cmd->cmd_dataLen);
 *      cmd->cmd_data[0] = 1;
 *      cmd->cmd_data[1] = 0;
 *
 *  or using cmd_Allocate():
 *      cmd_cmd_t cmd;
 *      // Address of pointer is important
 *      cmd_status_t ret = cmd_Allocate(&cmd, 2);
 *      cmd->module = LOG;
 *      cmd->function = LOG_FUNC_LOG;
 *      cmd->cmd_data[0] = 1;
 *      cmd->cmd_data[0] = 0;
 *
 *  This would log a message to the debug console
 *      CMD: CMD_INFO_OK
 *
 *  @param a command to put into the queue
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_QueuePut(cmd_cmd_t *cmd);

/** @brief Get a command from the queue
 * 
 *  This function gets a command from the queue, returning
 *  QUEUE_INFO_OK or an error if the queue is empty or in 
 *  an invalid state. The command is put in the pointer
 *  location cmd (passed by reference);
 *
 *  @param cmd the location to put the command
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_QueueGet(cmd_cmd_t **cmd);

/** @brief Initialize the command module
 * 
 *  This function initializes the main command queue, and
 *  if the compiler directive __CMD is set, initalizes
 *  UART2 for Rx mode to receive commands from the debug
 *  interface. It should be noted that the command queue
 *  and loop will still function without the __CMD
 *  directive. The directive only turns off the UART module
 *  and the connection to the debug interface.
 *
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_Init();

/** @brief The main command loop
 * 
 *  This function checks the main command queue for pending
 *  commands and executes them if there are any. It also 
 *  calls cmd_Deallocate to free the memory associated with
 *  commands.
 *
 *  @return a status value of the type cmd_status_t
 */
cmd_status_t cmd_Loop();

# endif /* __CMD_H */
