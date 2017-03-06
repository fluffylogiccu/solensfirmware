# Solens Firmware
[![Build Status](https://travis-ci.org/fluffylogiccu/solensfirmware.svg?branch=master)](https://travis-ci.org/fluffylogiccu/solensfirmware)

This project integrates the STM32F429I-DISC1 or S0LENS board
with an OV7670/OV5642 camera module and an ESP8266 wifi module.

For full functional documentation, explore the doc folder.

PIN MAP STM32F429I-DISC1

    /**********************************
     * DCMI     | Pin   | OVxxxx Pin
     * --------------------------------
     * VSYNC    | PB7   | 5
     * HSYNC    | PA4   | 6
     * PIXCLK   | PA6   | 7
     * D7       | PB9   | 9
     * D6       | PB8   | 10
     * D5       | PD3   | 11
     * D4       | PC11  | 12
     * D3       | PC9   | 13
     * D2       | PC8   | 14
     * D1       | PC7   | 15
     * D0       | PC6   | 16
     *********************************/
     

    /**********************************
     * I2C      | Pin   | OVxxxx Pin
     * --------------------------------
     * SCL      | PB10  | 3
     * SDA      | PB11  | 4
     *********************************/

    /**********************************
     * MCO1     | Pin   | OVxxxx Pin
     * --------------------------------
     * MCO1     | PA8   | 8
     *********************************/

    /**********************************
     * UART 2 Host       | Pin
     * --------------------------------
     * Tx                | D5
     * Rx                | D6
     * --------------------------------
     * UART cable connections
     * Black GND to GND
     * White Rx to D5 Tx
     * Green Tx to D6 Rx
     * Red NC
     *********************************/

    /**********************************
     * UART 1 ESP8266   | Pin
     * --------------------------------
     * Tx               | A9
     * --------------------------------
     * Connect to GPIO13 (D7) on the
     * ESP8266 board
     *********************************/
