/** @file ov5642.c
 * 
 *  @brief Implemenation of the ov5642 camera functions.
 *
 *  This file implements functionality of the ov5642
 *  camera module. For function descriptions, see the 
 *  ov5642.h file in the include directory.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "ov5642.h"
#include "ov5642_regs.h"
#include "sdram.h"
#include "log.h"
#include "err.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"

#include <stdint.h>

//uint8_t temp_buffer[320*240*3];

/**************************************
 * Private functions
 */

ov5642_status_t ov5642_clockInit() {
    GPIO_InitTypeDef gpioInit;

    #ifdef STM32F429I_DISCOVERY

    // Enable HSI clock
    RCC_HSICmd(ENABLE);

    // Configure for HSI clock, no prescaler
    RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);

    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Map alternate function 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

    // Configure PA8
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Pin = GPIO_Pin_8;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &gpioInit);

    #endif

    #ifdef S0LENS_A

    #endif

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_dmaInit() {
    DMA_InitTypeDef dmaInit;
    
    /* We are going to use DMA2, Stream 1, Channel 1
     * to handle DCMI DMA requests. See page 310 of
     * the STM32F4 family reference manual RM0090. */

    // Enable clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // Deinit existing stream
    DMA_Cmd(DMA2_Stream1, DISABLE);
    DMA_DeInit(DMA2_Stream1);

    // Construct initialization config
    dmaInit.DMA_Channel = DMA_Channel_1;
    dmaInit.DMA_PeripheralBaseAddr = OV5642_DCMI_PERIPHADDR;
    dmaInit.DMA_Memory0BaseAddr = (uint32_t) SDRAM_IMAGEADDR;
    dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInit.DMA_BufferSize = OV5642_IMAGE_BUFSIZE / 4;
    dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dmaInit.DMA_Mode = DMA_Mode_Circular;
    dmaInit.DMA_Priority = DMA_Priority_High;
    dmaInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
    dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;

    // Initialize
    DMA_Init(DMA2_Stream1, &dmaInit);

    // Enable
    DMA_Cmd(DMA2_Stream1, ENABLE);

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_dcmiInit() {
    GPIO_InitTypeDef gpioInit;

    #ifdef STM32F429I_DISCOVERY
    /**********************************
     * DCMI     | Pin
     * --------------------------------
     * VSYNC    | PB7
     * HSYNC    | PA4
     * PIXCLK   | PA6
     * D7       | PB9
     * D6       | PB8
     * D5       | PD3
     * D4       | PC11
     * D3       | PC9
     * D2       | PC8
     * D1       | PC7
     * D0       | PC6
     **********************************/

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC |
                           RCC_AHB1Periph_GPIOD, 
                           ENABLE);

    // Send clock to DCMI
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

    // Port A
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); // HSYNC
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); // PIXCLK

    gpioInit.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOA, &gpioInit); 

    // Port B
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); // VSYNC
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI); // D6
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI); // D7

    gpioInit.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOB, &gpioInit); 

    // Port C
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);  // D0
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);  // D1
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);  // D2
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);  // D3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI); // D4
    
    gpioInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |
                        GPIO_Pin_8 | GPIO_Pin_9 |
                        GPIO_Pin_11;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOC, &gpioInit); 

    // Port D
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_DCMI); // D5

    gpioInit.GPIO_Pin = GPIO_Pin_3;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &gpioInit); 

    #endif

    #ifdef S0LENS_A

    #endif
    
    // Initialize DCMI
    DCMI_InitTypeDef dcmiInit;

    // DCMI Init
    dcmiInit.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
    dcmiInit.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
    dcmiInit.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
    dcmiInit.DCMI_VSPolarity = DCMI_VSPolarity_High;
    dcmiInit.DCMI_HSPolarity = DCMI_HSPolarity_Low;
    dcmiInit.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
    dcmiInit.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

    // Initialize
    DCMI_Init(&dcmiInit);

    // Turn on JPEG mode
    // DCMI_JPEGCmd(ENABLE);

    // Enable interrupt on frame complete in DCMI
    DCMI_ITConfig(DCMI_IT_OVF, ENABLE);

    // NVIC Enable interrupt on frame complete
    NVIC_InitTypeDef nvicInit;
    nvicInit.NVIC_IRQChannel = DCMI_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);

    // Enable DCMI
    DCMI_Cmd(ENABLE);

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_i2cInit() {
    GPIO_InitTypeDef gpioInit;
    I2C_InitTypeDef i2cInit;

    #ifdef STM32F429I_DISCOVERY

    // Enable I2C clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    // Enable Port B GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Reset
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

    // Map alternate functions
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    // Configure pins
    gpioInit.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_OD;    // Open Drain for i2c
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpioInit);

    #endif

    #ifdef S0LENS_A

    #endif

    // Initialize I2C configuration
    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = 0x00;
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInit.I2C_ClockSpeed = OV5642_I2C2_SPEED;


    I2C_Init(I2C2, &i2cInit);

    I2C_Cmd(I2C2, ENABLE);

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_i2cStart(uint8_t address, uint8_t direction) {
    if (!(direction == I2C_Direction_Transmitter ||
          direction == I2C_Direction_Receiver)) {
        log_Log(OV5642, OV5642_ERR_I2CSTART, "Bad I2C direction.\0");
        return OV5642_ERR_I2CSTART;
    }
    
    uint32_t timeout = OV5642_I2C2_TIMEOUT;

    // Wait until I2C2 is not busy
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) {
        if (timeout-- == 0) {
            log_Log(OV5642, OV5642_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV5642_ERR_I2CTIMEOUT;
        }
    }

    // Send START
    I2C_GenerateSTART(I2C2, ENABLE);

    // Wait for slave acknowledge
    timeout = OV5642_I2C2_TIMEOUT;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        if (timeout-- == 0) {
            log_Log(OV5642, OV5642_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV5642_ERR_I2CTIMEOUT;
        }
    }

    // Send address
    I2C_Send7bitAddress(I2C2, address, direction);

    // Wait for acknowledgement
    timeout = OV5642_I2C2_TIMEOUT;
    if (direction == I2C_Direction_Transmitter) {
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            if (timeout-- == 0) {
                log_Log(OV5642, OV5642_ERR_I2CTIMEOUT, "I2C timed out.\0");
                return OV5642_ERR_I2CTIMEOUT;
            }   
        }
    } else if (direction == I2C_Direction_Receiver)  {
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
            if (timeout-- == 0) {
                log_Log(OV5642, OV5642_ERR_I2CTIMEOUT, "I2C timed out.\0");
                return OV5642_ERR_I2CTIMEOUT;
            }   
        }
    }

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_i2cStop() {
    I2C_GenerateSTOP(I2C2, ENABLE);
    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_i2cRead(uint8_t *data, uint8_t ack) {
    if (!(ack == OV5642_I2C2_ACK ||
          ack == OV5642_I2C2_NACK)) {
        log_Log(OV5642, OV5642_ERR_I2CREAD, "Bad value for ack parameter.\0");
        return OV5642_ERR_I2CREAD;
    }
   
    uint32_t timeout = OV5642_I2C2_TIMEOUT;            

    if (ack == OV5642_I2C2_ACK) { 
        I2C_AcknowledgeConfig(I2C2, ENABLE);
    } else {
        I2C_AcknowledgeConfig(I2C2, DISABLE);
    }
    
    // Wait for a byte
    timeout = OV5642_I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        if (timeout-- == 0) {
            log_Log(OV5642, OV5642_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV5642_ERR_I2CTIMEOUT;
        }   
    }

    // Read and return byte
    *data = I2C_ReceiveData(I2C2);

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_i2cWrite(uint8_t data) {
    I2C_SendData(I2C2, data);
    // Wait for transmission
    uint32_t timeout = OV5642_I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if (timeout-- == 0) {
            log_Log(OV5642, OV5642_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV5642_ERR_I2CTIMEOUT;
        }   
    }

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_regWrite(uint16_t address, uint8_t value) {
    // Start transaction
    ov5642_status_t ret = ov5642_i2cStart(OV5642_I2C2_WRITEADDR, I2C_Direction_Transmitter);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }   

    // Write address high order bits
    ret = ov5642_i2cWrite(address >> 8);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }

    // Write address low order bits
    ret = ov5642_i2cWrite(address & 0x00FF);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }

    // Write data
    ret = ov5642_i2cWrite(value & 0x00FF);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }

    // Stop transmission
    ret = ov5642_i2cStop();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret);
        return ret;
    }

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_regRead(uint16_t address, uint8_t *value) {
    // Start transaction
    ov5642_status_t ret = ov5642_i2cStart(OV5642_I2C2_WRITEADDR, I2C_Direction_Transmitter);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }   

    // Send address high order bits
    ret = ov5642_i2cWrite(address >> 8);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }

    // Write address low order bits
    ret = ov5642_i2cWrite(address & 0x00FF);
    if (ret != OV5642_INFO_OK) {
        ov5642_i2cStop();
        log_Log(OV5642, ret);
        return ret;
    }

    // Stop transmission
    ret = ov5642_i2cStop();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret);
        return ret;
    }

    // Start read transmission
    ret = ov5642_i2cStart(OV5642_I2C2_READADDR, I2C_Direction_Receiver);
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret);
        return ret;
    }

    // Read one byte
    ret = ov5642_i2cRead(value, OV5642_I2C2_NACK);
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret);
        return ret;
    }

    // Stop transmission
    ret = ov5642_i2cStop();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret);
        return ret;
    }

    return OV5642_INFO_OK;;

}

ov5642_status_t ov5642_regWriteArray(const ov5642_reg_t *reg) {
    // reg is terminated with [0xffff, 0xff]
    ov5642_status_t ret;
    while (reg->reg != 0xffff || reg->val != 0xff) {
        ret = ov5642_regWrite(reg->reg, reg->val);
        if (ret != OV5642_INFO_OK) {
            log_Log(OV5642, ret, "Couldn't write OV5642 register array.\0");
            return ret;
        }

        // Increase register pointer
        reg++;
    }

    return OV5642_INFO_OK;
}

void DCMI_IRQHandler() {
    log_Log(OV5642, OV5642_INFO_OK, "OVF IRQ.\0");
    return;
}

/**************************************
 * Public functions
 */

ov5642_status_t ov5642_Init() {
    ov5642_status_t ret = ov5642_dmaInit();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret, "Could not initialize OV5642 DMA.\0");
        return ret;
    }

    ret = ov5642_dcmiInit();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret, "Could not initialize OV5642 DCMI.\0");
        return ret;
    }

    ret = ov5642_i2cInit();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret, "Could not initialize OV5642 I2C.\0");
        return ret;
    }

    ret = ov5642_clockInit();
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret, "Could not initialize OV5642 clock.\0");
        return ret;
    }

    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_Configure() {
    ov5642_status_t ret = ov5642_regWriteArray(OV5642_QVGA_Preview);
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret, "Could not configure OV5642 registers.\0");
        return ret;
    }
/*    ret = ov5642_regWriteArray(ov5642_320x240);
    if (ret != OV5642_INFO_OK) {
        log_Log(OV5642, ret, "Could not configure OV5642 registers.\0");
    }*/
    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_Capture() {
    DCMI_CaptureCmd(ENABLE);
    return OV5642_INFO_OK;
}

ov5642_status_t ov5642_Transfer() {
    log_Log(OV5642, OV5642_INFO_OK, "Beginning image transfer.\0");
    log_Log(OV5642, OV5642_INFO_IMAGE, "\0", OV5642_IMAGE_BUFSIZE, (uint8_t *) SDRAM_IMAGEADDR);

    return OV5642_INFO_OK;
}
