# Sources
SRCS := main.c \
		log.c \
        cmd.c \
		sdram.c \
		prof.c \
		cam.c \
		sleep.c \
        system_stm32f4xx.c \
        startup_stm32f429_439xx.s \
        stm32f4xx_it.c \
		\
		misc.c \
		stm32f4xx_rtc.c \
		stm32f4xx_pwr.c \
		stm32f4xx_rcc.c \
		stm32f4xx_gpio.c \
		stm32f4xx_exti.c \
		stm32f4xx_usart.c

ifeq ($(DEBUG),TRUE)
  ifneq ($(LOG),NONE)
    SRCS += stm32f4xx_usart.c
  else
    ifeq ($(CMD),TRUE)
      SRCS += stm32f4xx_usart.c
    endif
  endif

  ifeq ($(PROF),TRUE)
    SRCS += stm32f4xx_tim.c
  endif

  ifeq ($(TEST),TRUE)
    ifneq ($(LOG),NONE)
      SRCS += test_log.c
    endif

    ifeq ($(PROF),TRUE)
      SRCS += test_prof.c
    endif
  endif

endif

ifeq ($(BOARD),STM32F429I_DISCOVERY)
  SRCS += stm32f429i_discovery_sdram.c \
		  stm32f4xx_fmc.c
else
  ifeq ($(BOARD),S0LENS_A)
    SRCS += solens_sdram.c \
			stm32f4xx_fmc.c
  endif
endif

ifeq ($(CAMERA),OV5642)
  SRCS += ov5642.c \
		  stm32f4xx_dma.c \
		  stm32f4xx_dcmi.c \
		  stm32f4xx_i2c.c
else
  ifeq ($(CAMERA),OV7670)
	SRCS += ov7670.c \
			stm32f4xx_dma.c \
            stm32f4xx_dcmi.c \
            stm32f4xx_i2c.c
  endif
endif

ifneq ($(WIFI),NONE)
  ifeq ($(WIFI),ESP8266)
	SRCS += wifi.c \
			esp8266.c
  endif
endif

# Object files
OBJS := $(SRCS:.c=.o)
OBJS := $(OBJS:.s=.o)

# Assembly files
ASMS := $(SRCS:.c=.s)

# Preprocessor files
PRES := $(SRCS:.c=.i)

# Search path for source files
VPATH = src:src/drivers:src/drivers/discovery:src/project:src/startup:src/test

# Include directory
INC_DIR := inc \
           inc/project \
           inc/drivers \
		   inc/drivers/discovery \
           inc/startup \
           inc/test

# Config directory
CONF_DIR := config

# Build directory
BUILD_DIR := build

# Binary directory
BIN_DIR := bin

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME := firmware_poc

# Linker file
LINKER_FILE := $(CONF_DIR)/STM32F429ZI_FLASH.ld
