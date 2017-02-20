/** @file mod.h
 *  @brief Module definitions.
 *
 *  This file defines a module enumeration for use with
 *  the logger and the command modules. 
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __MOD_H
#define __MOD_H

/*************************************
 * Includes and definitions
 */

/** @brief Enumeration of module types. 
 */
typedef enum mod_e {
    LOG,
    CMD,
    STDLIB,
    SDRAM,
    OV5642,
    OV7670,
    PROF,
    TEST,
    CAM,
    ESP8266,
    WIFI,
} mod_t;

# endif /* __MOD_H */
