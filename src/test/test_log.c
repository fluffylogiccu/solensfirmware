/** @file test_log.c
 *  @brief Test functions for the logger.
 *
 *  This contains the implementations of the logger test
 *  functions.
 *  
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "test.h"
#include "log.h"
#include "err.h"
#include <stdint.h>

#define NULL ((void *)0)

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */
#ifdef __LOG
test_status_t test_log() {
    test_Test(test_log_Init, "test_log_Init passed.\0");
    test_Test(test_log_Log, "test_log_Log passed.\0");
   
    log_Log(TEST, TEST_INFO_PASSED, "test_log passed all tests.\0"); 

    return TEST_INFO_PASSED;
}

char *test_log_Init() {

    log_status_t st = log_Init();
    test_Assert(st == LOG_INFO_OK, "log_Init couldn't initialize logger.\0");
    st = log_Init();
    test_Assert(st == LOG_WARN_ALINIT, "log_Init should issue a warning on second initialization call.\0");

    return NULL;
}

char * test_log_Log() {
    log_Log(TEST, TEST_INFO_OK, "Logging with two parameters.\0");
    log_Log(TEST, TEST_INFO_CONFIRM);
    log_Log(TEST, TEST_INFO_OK, "Logging with three parameters.\0");
    log_Log(TEST, TEST_INFO_CONFIRM, "Some message string.\0");
    log_Log(TEST, TEST_INFO_OK, "Logging with four parameters.\0");
    uint8_t testBuf[16];
    for (uint8_t i = 0; i < 16; i++) {
        testBuf[i] = i;
    }
    log_Log(TEST, TEST_INFO_CONFIRM, 16, testBuf);
    log_Log(TEST, TEST_INFO_OK, "Logging with five parameters.\0");
    log_Log(TEST, TEST_INFO_CONFIRM, "Some message string.\0", 16, testBuf);
    uint8_t msgBufTooLong[LOG_MAXMSGSIZE+1];
    for (uint32_t i = 0; i < LOG_MAXMSGSIZE+1; i++) {
        msgBufTooLong[i] = 'A';
    }
    msgBufTooLong[LOG_MAXMSGSIZE] = '\0';
    test_Assert(log_Log(TEST, TEST_INFO_OK, (char *) msgBufTooLong) == LOG_ERR_MSGSIZE, "log_Log failed to handle a message that was too long.\0");
    test_Assert(log_Log(TEST, TEST_INFO_OK, LOG_MAXDATASIZE+1, NULL) == LOG_ERR_DATASIZE, "log_Log failed to handle data that was too long.\0");

    return NULL;
}
#endif





