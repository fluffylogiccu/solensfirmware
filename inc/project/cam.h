/** @file cam.h
 *  @brief Function prototypes for the camera controller.
 *
 *  This contains the prototypes, macros, constants,
 *  and global variables for the camera controller.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __CAM_H
#define __CAM_H

/*************************************
 * @name Includes and definitions
 */

#include "err.h"
#include <stdint.h>

/**************************************
 * @name Private functions
 */

/**************************************
 * @name Public functions
 */

/** @brief Initialize the camera
 *
 *  This function initializes the camera. It should
 *  be called before attempting to configure or take
 *  an image.
 *
 *  @return a status of type cam_status_t
 */
cam_status_t cam_Init();

/** @brief Configure the camera module
 *  
 *  This function should initialize the camera to take
 *  images and store them in SDRAM.
 *
 *  @return a status of type cam_status_t
 */
cam_status_t cam_Configure();

/** @brief Capture an image
 *
 *  This function should capture and store an image in 
 *  the SDRAM.
 *
 *  @return a status of type cam_status_t
 */
cam_status_t cam_Capture();

/** @brief Transfer an image to the debug console
 *
 *  This function transfers an image from SDRAM to the 
 *  debug host interface. The logger must be enabled for
 *  this function to work.
 *
 *  @return a status of type cam_status_t
 */
cam_status_t cam_Transfer();

# endif /* __CAM_H */
