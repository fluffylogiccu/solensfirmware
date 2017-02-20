/** @file template.h
 *  @brief Function prototypes for the template file.
 *
 *  This contains the prototypes, macros, constants,
 *  and global variables for the template file. 
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __TEMPLATE_H
#define __TEMPLATE_H

/*************************************
 * Includes and definitions
 */

#include <stdint.h>

/**************************************
 * Private functions
 */

/** @brief Short description of the function.
 *
 *  Long description of the function, and certain
 *  important use cases associated with the function.         
 *
 *  @param ch the input byte
 *  @return The return byte
 */
uint8_t template_privateFunction(uint8_t ch);

/**************************************
 * Public functions
 */

/** @brief Short description of the function.
 *
 *  Long description of the function, and certain
 *  important use cases associated with the function.         
 *
 *  @param ch the input byte
 *  @return The return byte
 */
uint8_t template_PublicFunction(uint8_t ch);

# endif /* __TEMPLATE_H */
