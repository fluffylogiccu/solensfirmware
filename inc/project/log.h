/** @file log.h
 *  @brief Logging functions for the debug serial terminal
 *  connection.
 *
 *  This file contains logging functions, macros, and
 *  initialization code for the serial logger. This code 
 *  can be toggled on or off with the __LOG preprocessor
 *  macro.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __LOG_H
#define __LOG_H

/*************************************
 * @name Includes and definitions
 */

#ifdef __LOG
#include <stdint.h>
#include "err.h"
#include "mod.h"

#define LOG_MAXMSGSIZE 255
#define LOG_MAXDATASIZE 16777216
#define LOG_BAUDRATE 115200//921600

/** @brief Type for log packets
 */
typedef struct __attribute__ ((packed)) log_packet_s {
    uint8_t log_packet_mod;
    uint8_t log_packet_status;
    uint8_t log_packet_msgLen;
    uint8_t *log_packet_msg;
    uint32_t log_packet_dataLen;
    uint8_t *log_packet_data;
} log_packet_t;
#endif

/**************************************
 * @name Private functions
 */

#ifdef __LOG
/** @brief Log with two parameters.
 *
 *  This function takes in a module code and a status code
 *  associated with the module. The function then logs the 
 *  data to the serial terminal.
 *
 *  @param module of the type mod_t
 *  @param status code to log
 *  @return return code with type log_status_t
 */
log_status_t log_log2(mod_t module, gen_status_t status);

/** @brief Log with three parameters.
 *
 *  This function takes in a module code, a status code
 *  associated with the module, and a message as a
 *  uint8_t pointer with a null terminated string. The
 *  function then logs the data to the serial terminal.
 *
 *  @param module of the type mod_t
 *  @param status code to log
 *  @param msg message as a pointer to uint8_t
 *  @return return code with type log_status_t
 */
log_status_t log_log3(mod_t module, gen_status_t status, char *msg);

/** @brief Log with four parameters.
 *
 *  This function takes in a module code, a status code
 *  associated with the module, a data length, and a  
 *  pointer to a data buffer.. The function then logs the 
 *  data to the serial terminal.
 *
 *  @param module of the type mod_t
 *  @param status code to log
 *  @param len optional data length paramter
 *  @param data optional pointer to data bufffer
 *  @return return code with type log_status_t
 */
log_status_t log_log4(mod_t module, gen_status_t status, uint32_t len, uint8_t *data);

/** @brief Log with five parameters.
 *
 *  This function takes in a module code, a status code
 *  associated with the module, a message as a uint8_t 
 *  pointer with a null terminated string, a data length,
 *  and a pointer to a data buffer.. The function then 
 *  logs the data to the serial terminal.
 *
 *  @param module of the type mod_t
 *  @param status code to log
 *  @param msg message as a pointer to uint8_t
 *  @param len optional data length paramter
 *  @param data optional pointer to data bufffer
 *  @return return code with type log_status_t
 */
log_status_t log_log5(mod_t module, gen_status_t status, char *msg, uint32_t len, uint8_t *data);

/** @brief Log send command.
 *
 *  This function sends the specified command packet with 
 *  the underlying UART configuration.
 *
 *  @param log_packet the log packet to send
 *  @return return code with type log_status_t
 */
log_status_t log_send(log_packet_t *log_packet);
#endif

/**************************************
 * @name Public functions
 */

#ifdef __LOG
/** @breif The log initialization function
 *
 *  This function should be called before using any log
 *  functions. It initializes the logger and returns
 *  a status code.
 *
 *  @return status code of type log_status_t
 */
log_status_t log_Init();

/** @brief The main log function
 *
 *  This function can take several forms.Depending on the 
 *  number of input parameters to the function and the type
 *  of message (warning, error, info, etc), this macro will
 *  call one of several sub-functions. Calling log with 0
 *  or 1 arguments should generate a compile time or 
 *  linker error.
 *
 *  The second parameter can be called with any value from
 *  the err.h module status types. This will get typecast 
 *  as a gen_status_t type (declared in this err.h).
 *
 *  The macro function is declared as log_Log()
 *
 *  @param module of the type mod_t
 *  @param status code to log
 *  @param msg message as a pointer to uint8_t
 *  @param len optional data length paramter
 *  @param data optional pointer to data bufffer
 *  @return return code with type log_status_t
 */
#define LOG_SELECT(_1,_2,_3,_4,_5,LOG_NAME,...) LOG_NAME
#define log_Log(...) LOG_SELECT(__VA_ARGS__,log_log5,log_log4,log_log3,log_log2,log_log1,log_log0)(__VA_ARGS__)

#else
#define log_Log(...) LOG_ERR_LOGOFF
#endif

# endif /* __LOG_H */
