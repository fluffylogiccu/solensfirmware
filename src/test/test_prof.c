/** @file test_prof.c
 *  @brief Test functions for the profiler
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
#include "prof.h"
#include "err.h"
#include <stdint.h>

#define NULL ((void *)0)

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */
#ifdef __PROF
test_status_t test_prof() {
    test_Test(test_prof_Init, "test_prof_Init passed.\0");
    test_Test(test_prof_Profile, "test_prof_Profile passed.\0");
   
    log_Log(TEST, TEST_INFO_PASSED, "test_prof passed all tests.\0"); 

    return TEST_INFO_PASSED;
}

char *test_prof_Init() {

    prof_status_t st = prof_Init();
    test_Assert(st == PROF_INFO_OK, "prof_Init couldn't initialize profiler.\0");
    st = log_Init();
    test_Assert(st == PROF_WARN_ALINIT, "prof_Init should issue a warning on second initialization call.\0");

    return NULL;
}

char * test_prof_Profile() {
    
    prof_Profile(
        for (int i = 0; i < 1000; i++) {},
        "Test loop 1000\t\0"
    );

    prof_Profile(
        for (int i = 0; i < 10000; i++) {},
        "Test loop 10000\t\0"
    );

    prof_Profile(
        for (int i = 0; i < 100000; i++) {},
        "Test loop 100000\0"
    );

    prof_Profile(
        for (int i = 0; i < 1000000; i++) {},
        "Test loop 1000000\0"
    );

    prof_Profile(
        for (int i = 0; i < 10000000; i++) {},
        "Test loop 10000000\0"
    );

    return NULL;
}
#endif





