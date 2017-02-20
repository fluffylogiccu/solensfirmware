/** @file sdram.h
 *  @brief Function prototypes for the SDRAM functions
 *
 *  This contains the prototypes for the SDRAM functions
 *  based on compiler directives.
 *
 *  If __STM32F429I_DISCOVERY is defined, we use the 
 *  implementation of the SDRAM interface from the
 *  discovery board libraries.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __SDRAM_H
#define __SDRAM_H

/*************************************
 * Includes and definitions
 */

#include <stdint.h>
#include "err.h"

/* @brief SDRAM addresses
 */
#define SDRAM_BASEADDR 0xD0100000
#define SDRAM_IMAGEADDR SDRAM_BASEADDR

/**************************************
 * Private functions
 */

/**************************************
 * Public functions
 */

/** @brief Initialize the SDRAM interface
 *
 *  This will initialize the SDRAM interface either
 *  using the discovery board library or a custom
 *  implementations         
 *
 *  @return a status code of the type sdram_status_t
 */
sdram_status_t sdram_Init();

/** @brief Writes to the SDRAM
 *
 *  This will write to the sdram at a specified address
 *  using the supplied buffer, address, and transfer size.
 *
 *  @param buf the buffer to read from
 *  @param addr the address to write to
 *  @param size size in bytes to write
 *  @return a status code of the type sdram_status_t
 */
sdram_status_t sdram_write(uint32_t *buf, uint32_t addr, uint32_t size);

/** @brief Reads from the SDRAM
 *
 *  This will read fram the sdram at a specified address
 *  with the supplied buffer, address, and transfer size.
 * 
 *  @param buf the buffer to write to
 *  @param addr the address to read from
 *  @param size size in bytes to read
 *  @return a status code of the type sdram_status_t
 */
sdram_status_t sdram_read(uint32_t *buf, uint32_t addr, uint32_t size);

# endif /* __SDRAM_H */
