/** @file sdram.c
 *  @brief Implemenation of the SDRAM functions
 *
 *  This file implements the SDRAM functions
 *  depending on the flag __STM32F429I_DISCOVERY
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "sdram.h"
#include <stdint.h>
#include "err.h"

#ifdef __STM32F429I_DISCOVERY
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_sdram.h"
#endif

/* @ brief Initialization flag
 */
static uint8_t sdram_initialized = 0;

/**************************************
 * Public functions
 */

sdram_status_t sdram_Init() {
    if (sdram_initialized == 1) {
        return SDRAM_WARN_ALINIT;
    }

    #ifdef __STM32F429I_DISCOVERY
    // Call discovery board function
    SDRAM_Init();
    sdram_initialized = 1;
    return SDRAM_INFO_OK;
    #endif
    
    return SDRAM_ERR_UNKNOWN;
}

sdram_status_t sdram_write(uint32_t *buf, uint32_t addr, uint32_t size) {

    #ifdef __STM32F429I_DISCOVERY
    SDRAM_WriteBuffer(buf, addr, size);
    return SDRAM_INFO_OK;
    #endif

    return SDRAM_ERR_UNKNOWN;

}

sdram_status_t sdram_read(uint32_t *buf, uint32_t addr, uint32_t size) {

    #ifdef __STM32F429I_DISCOVERY
    SDRAM_ReadBuffer(buf, addr, size);
    return SDRAM_INFO_OK;
    #endif
    
    return SDRAM_ERR_UNKNOWN;
}
