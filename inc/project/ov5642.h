/** @file ov5642.h
 *  @brief Function prototypes for the ov5642 camera
 *
 *  This contains the driver function prototypes, macros,
 *  constants, and global variables.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

#ifndef __OV5642_H
#define __OV5642_H

/*************************************
 * @name Includes and definitions
 */

#include "err.h"
#include "ov5642_regs.h"
#include <stdint.h>

/* @brief Base address of DCMI module and offset of DR 
 * register for DMA requests.
 */
#define OV5642_DCMI_BASEADDR ((uint32_t)0x50050000)
#define OV5642_DCMI_OFFSETDR 0x28
#define OV5642_DCMI_PERIPHADDR (OV5642_DCMI_BASEADDR | OV5642_DCMI_OFFSETDR)

/* @brief DMA Transfer size
 */
#define OV5642_IMAGE_BUFSIZE 320*240*2

/* @brief I2C clock speed
 */
#define OV5642_I2C2_SPEED 100000

/* @brief I2C ack and nack requests
 */
#define OV5642_I2C2_ACK 1
#define OV5642_I2C2_NACK 0

/* @brief I2C timeout
 */
#define OV5642_I2C2_TIMEOUT 0x4000

/* @brief OV5642 read and write addresses
 */
#define OV5642_I2C2_READADDR 0x79 //0x42
#define OV5642_I2C2_WRITEADDR 0x78 //0x42

/**************************************
 * @name Private functions
 */

/** @brief Initialize the clock with RCC.
 *
 *  This function initializes the clock used by the 
 *  ov5642 camera module.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_clockInit();

/** @brief Initialize the DMA controller
 *
 *  This function initializes and configures DMA for the 
 *  ov5642 camera module. DMA is configured with DMA2,
 *  Stream 1, Channel 1. See page 310 of the STM32F4 
 *  family reference manual RM0090.
 *
 *  Configured for circular mode, with maximum stream NDT
 *  transfer size. Configured to transfer into SDRAM.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_dmaInit();

/** @brief Initialize the DCMI module
 *
 *  This function initializes and configures DCMI. The 
 *  following pin mapping is used for the discovery board.
 *
 *  DCMI     | Pin
 *  --------------------------------
 *  VSYNC    | PB7
 *  HSYNC    | PA4
 *  PIXCLK   | PA6
 *  D7       | PB9
 *  D6       | PB8
 *  D5       | PD3
 *  D4       | PC11
 *  D3       | PC9
 *  D2       | PC8
 *  D1       | PC7
 *  D0       | PC6
 *  -----------------------------------
 *
 *  DCMI is configured for snapshot mode.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_dcmiInit();

/** @brief Initialize I2C
 *
 *  This function initializes I2C to set and control 
 *  ov5642 camera configuration.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_i2cInit();

/** @brief Start I2C transmission   
 *
 *  This function starts a transmission on 12C2 to the 
 *  address given with the specified direction. Use either
 *  I2C_Direction_Transmitter or I2C_Driection_Receiver
 *  for the direction parameter.
 *
 *  @param address address of the slave
 *  @param direction transfer direction
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_i2cStart(uint8_t address, uint8_t direction);

/** @brief Stop I2C transmission   
 *
 *  This function stops an I2C2 transaction by sending
 *  the STOP condition.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_i2cStop();

/** @brief Read a byte from the I2C bus     
 *
 *  This function reads a single byte and sends and ACK
 *  signal back to the sender. The byte is put in the 
 *  memory location of data
 *
 *  @param data the byte read
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_i2cRead(uint8_t *data, uint8_t ack);

/** @brief Write a byte to the I2C bus     
 *
 *  This function writes a single byte and sends and
 *  waits for the byte to be transmitted. If ack is true,
 *  request another byte. If ack is false, end the 
 *  do not request another byte (ending transmission).
 *
 *  @param data the byte to send
 *  @param ack if ack is true, request another byte
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_i2cWrite(uint8_t data);

/** @brief Write a register in the OV5642
 *
 *  This function writes a register in the OV5642 using
 *  the I2C interface. The I2C interface must be configured
 *  before using this function.
 *
 *  @param reg the register to write
 *  @param value the value to write to the register
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_regWrite(uint16_t reg, uint8_t value);

/** @brief Read a register in the OV5642
 *
 *  This function reads a register in the OV5642 using
 *  the I2C interface. The I2C interface must be configured
 *  before using this function.
 *
 *  @param reg the register to read
 *  @param value a pointer to the value that is read
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_regRead(uint16_t reg, uint8_t *value);

/** @brief Write an array of register values in the OV5642
 *
 *  This function writes the registers in the OV5642 using
 *  the I2C interface. The I2C interface must be configured
 *  before using this function. This function writes the 
 *  registers based on address-value mappings in the 
 *  ov5642_regs.h file.
 *
 *  @param reg the register mapping to write.
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_regWriteArray(const ov5642_reg_t *reg);

/** @brief Frame complete interrupt handler
 *
 *  This file implements an interrupt handler for the
 *  frame complete interrupt in the DCMI controller.
 *  Currently this handler does nothing.
 */ 
void DCMI_IRQHandler();

/**************************************
 * @name Public functions
 */

/** @brief Initialize the ov5642 camera
 *
 *  This function fully initializes the ov5642 camera.
 *  This should be called before attempting to configure
 *  or take an image.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_Init();

/** @brief Transfer a configuration to the camera over I2C
 *
 *  This function transfers a configuration from the 
 *  definitions in ov5642_regs.h to the camera module 
 *  over I2C.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_Configure();

/** @brief Capture an image and put it into SDRAM.
 *
 *  This function commands the ov5642 module to take an
 *  image, and transfers the image to SDRAM using DMA and 
 *  DCMI functionality.
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_Capture();

/** @brief Transfer an image from SDRAM to the host.
 *
 *  This function uses the logger to transfer a full
 *  image from SDRAM to the host computer. This function
 *  will not work if the logger is disabled (directive 
 *  __LOG needs to be on).
 *
 *  @return a status code of the type ov5642_status_t
 */
ov5642_status_t ov5642_Transfer();

# endif /* __OV5642_H */
