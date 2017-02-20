/** @file ov7670.c
 *  @brief Implemenation of the ov7670 camera functions.
 *
 *  This file implements functionality of the ov7670
 *  camera module. For function descriptions, see the 
 *  ov7670.h file in the include directory.
 *
 *  @author Ben Heberlein
 *  @bug No known bugs.
 */

/*************************************
 * Includes and definitions
 */

#include "ov7670.h"
#include "ov7670_regs.h"
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

ov7670_status_t ov7670_clockInit() {
    GPIO_InitTypeDef gpioInit;
#if 0    
    // Configure for 24MHz clock
    RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_4);

    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    // Map alternate function 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);

    // Configure PC9
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Pin = GPIO_Pin_9;
    gpioInit.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(GPIOC, &gpioInit);
#endif

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
    gpioInit.GPIO_Speed = GPIO_Medium_Speed;
    GPIO_Init(GPIOA, &gpioInit);

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_dmaInit() {
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
    dmaInit.DMA_PeripheralBaseAddr = OV7670_DCMI_PERIPHADDR;
    dmaInit.DMA_Memory0BaseAddr = (uint32_t) SDRAM_IMAGEADDR;
    dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInit.DMA_BufferSize = OV7670_IMAGE_BUFSIZE/ 4;
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

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_dcmiInit() {
    GPIO_InitTypeDef gpioInit;

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
                           RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE,
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
                        GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_9; 
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
#if 0
    // Port E 
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); // D3

    gpioInit.GPIO_Pin = GPIO_Pin_1;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOE, &gpioInit); 
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

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_i2cInit() {
    GPIO_InitTypeDef gpioInit;
    I2C_InitTypeDef i2cInit;

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

    // Initialize I2C configuration
    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = 0x00;
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInit.I2C_ClockSpeed = OV7670_I2C2_SPEED;


    I2C_Init(I2C2, &i2cInit);

    I2C_Cmd(I2C2, ENABLE);

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_i2cStart(uint8_t address, uint8_t direction) {
    if (!(direction == I2C_Direction_Transmitter ||
          direction == I2C_Direction_Receiver)) {
        log_Log(OV7670, OV7670_ERR_I2CSTART, "Bad I2C direction.\0");
        return OV7670_ERR_I2CSTART;
    }
    
    uint32_t timeout = OV7670_I2C2_TIMEOUT;

    // Wait until I2C2 is not busy
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) {
        if (timeout-- == 0) {
            log_Log(OV7670, OV7670_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV7670_ERR_I2CTIMEOUT;
        }
    }

    // Send START
    I2C_GenerateSTART(I2C2, ENABLE);

    // Wait for slave acknowledge
    timeout = OV7670_I2C2_TIMEOUT;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        if (timeout-- == 0) {
            log_Log(OV7670, OV7670_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV7670_ERR_I2CTIMEOUT;
        }
    }

    // Send address
    I2C_Send7bitAddress(I2C2, address, direction);

    // Wait for acknowledgement
    timeout = OV7670_I2C2_TIMEOUT;
    if (direction == I2C_Direction_Transmitter) {
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            if (timeout-- == 0) {
                log_Log(OV7670, OV7670_ERR_I2CTIMEOUT, "I2C timed out.\0");
                return OV7670_ERR_I2CTIMEOUT;
            }   
        }
    } else if (direction == I2C_Direction_Receiver)  {
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
            if (timeout-- == 0) {
                log_Log(OV7670, OV7670_ERR_I2CTIMEOUT, "I2C timed out.\0");
                return OV7670_ERR_I2CTIMEOUT;
            }   
        }
    }

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_i2cStop() {
    I2C_GenerateSTOP(I2C2, ENABLE);
    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_i2cRead(uint8_t *data, uint8_t ack) {
    if (!(ack == OV7670_I2C2_ACK ||
          ack == OV7670_I2C2_NACK)) {
        log_Log(OV7670, OV7670_ERR_I2CREAD, "Bad value for ack parameter.\0");
        return OV7670_ERR_I2CREAD;
    }
   
    uint32_t timeout = OV7670_I2C2_TIMEOUT;            

    if (ack == OV7670_I2C2_ACK) { 
        I2C_AcknowledgeConfig(I2C2, ENABLE);
    } else {
        I2C_AcknowledgeConfig(I2C2, DISABLE);
    }
    
    // Wait for a byte
    timeout = OV7670_I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        if (timeout-- == 0) {
            log_Log(OV7670, OV7670_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV7670_ERR_I2CTIMEOUT;
        }   
    }

    // Read and return byte
    *data = I2C_ReceiveData(I2C2);

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_i2cWrite(uint8_t data) {
    I2C_SendData(I2C2, data);
    // Wait for transmission
    uint32_t timeout = OV7670_I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if (timeout-- == 0) {
            log_Log(OV7670, OV7670_ERR_I2CTIMEOUT, "I2C timed out.\0");
            return OV7670_ERR_I2CTIMEOUT;
        }   
    }

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_regWrite(uint8_t address, uint8_t value) {
    // Start transaction
    ov7670_status_t ret = ov7670_i2cStart(OV7670_I2C2_WRITEADDR, I2C_Direction_Transmitter);
    if (ret != OV7670_INFO_OK) {
        ov7670_i2cStop();
        log_Log(OV7670, ret);
        return ret;
    }   

    // Write address
    ret = ov7670_i2cWrite(address & 0x00FF);
    if (ret != OV7670_INFO_OK) {
        ov7670_i2cStop();
        log_Log(OV7670, ret);
        return ret;
    }

    // Write data
    ret = ov7670_i2cWrite(value & 0x00FF);
    if (ret != OV7670_INFO_OK) {
        ov7670_i2cStop();
        log_Log(OV7670, ret);
        return ret;
    }

    // Stop transmission
    ret = ov7670_i2cStop();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret);
        return ret;
    }

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_regRead(uint8_t address, uint8_t *value) {
    // Start transaction
    ov7670_status_t ret = ov7670_i2cStart(OV7670_I2C2_WRITEADDR, I2C_Direction_Transmitter);
    if (ret != OV7670_INFO_OK) {
        ov7670_i2cStop();
        log_Log(OV7670, ret);
        return ret;
    }   

    // Send address high order bits
    ret = ov7670_i2cWrite(address & 0x00FF);
    if (ret != OV7670_INFO_OK) {
        ov7670_i2cStop();
        log_Log(OV7670, ret);
        return ret;
    }

    // Stop transmission
    ret = ov7670_i2cStop();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret);
        return ret;
    }

    // Start read transmission
    ret = ov7670_i2cStart(OV7670_I2C2_READADDR, I2C_Direction_Receiver);
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret);
        return ret;
    }

    // Read one byte
    ret = ov7670_i2cRead(value, OV7670_I2C2_NACK);
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret);
        return ret;
    }

    // Stop transmission
    ret = ov7670_i2cStop();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret);
        return ret;
    }

    return OV7670_INFO_OK;;

}

ov7670_status_t ov7670_regWriteArray(const ov7670_reg_t *reg) {
    // reg is terminated with [0xff, 0xff]
    ov7670_status_t ret;
    while (reg->reg != 0xff || reg->val != 0xff) {
        ret = ov7670_regWrite(reg->reg, reg->val);
        if (ret != OV7670_INFO_OK) {
            log_Log(OV7670, ret, "Couldn't write OV7670 register array.\0");
            return ret;
        }

        // Wait
        for (uint32_t i = 0; i < 10000; i++) {}

        // Increase register pointer
        reg++;
    }

    return OV7670_INFO_OK;
}

void DCMI_IRQHandler() {
    log_Log(OV7670, OV7670_INFO_OK, "OVF IRQ.\0");
    return;
}

/**************************************
 * Public functions
 */

ov7670_status_t ov7670_Init() {
    ov7670_status_t ret = ov7670_dmaInit();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret, "Could not initialize OV7670 DMA.\0");
        return ret;
    }

    ret = ov7670_dcmiInit();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret, "Could not initialize OV7670 DCMI.\0");
        return ret;
    }

    ret = ov7670_i2cInit();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret, "Could not initialize OV7670 I2C.\0");
        return ret;
    }

    ret = ov7670_clockInit();
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret, "Could not initialize OV7670 clock.\0");
        return ret;
    }

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_Configure() {
    ov7670_status_t ret = ov7670_regWriteArray(ov7670_QVGA_YUV_regs);
    if (ret != OV7670_INFO_OK) {
        log_Log(OV7670, ret, "Could not configure OV7670 registers.\0");
        return ret;
    }

    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_Capture() {
    DCMI_CaptureCmd(ENABLE);
    return OV7670_INFO_OK;
}

ov7670_status_t ov7670_Transfer() {
    log_Log(OV7670, OV7670_INFO_OK, "Beginning image transfer.\0");
    log_Log(OV7670, OV7670_INFO_IMAGE, "\0", OV7670_IMAGE_BUFSIZE, (uint8_t *) SDRAM_IMAGEADDR);
    
    return OV7670_INFO_OK;
}
