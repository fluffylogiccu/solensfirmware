#OPTIONS
# -----------------

# Debug mode
# Debug mode will suppress the directives
# __LOG, __CMD, __PROF, __TEST, and __DEBUG
# Options are TRUE, FALSE
# Default is TRUE
DEBUG=TRUE

# Logger verbosity
# Options are INFO, WARN, ERR, NONE
# Default is ERR
LOG=INFO

# Command interface
# Options are TRUE, FALSE
# Default is TRUE
CMD=TRUE

# Profiler enable
# Options are TRUE, FALSE
# Default is TRUE
PROF=TRUE

# Unit tests enabled
# Options are TRUE, FALSE
# Default is FALSE
TEST=FALSE

# Camera module
# Options are OV5642, OV7670
# Default is OV5642
CAMERA=OV5642

# Wifi Module
# Options are ESP8266, NONE
# Default is NONE
WIFI=ESP8266

# Discovery board
# Options are STM32F429I_DISCOVERY, S0LENS_A
BOARD=S0LENS_A

# VARIABLES
# -----------------

COMP_FLAGS = STM32F429_439xx USE_STDPERIPH_DRIVER HSE_VALUE=8000000

ifeq ($(DEBUG),FALSE)
  DEBUG_FLAGS = -O0
else
  ifeq ($(DEBUG),TRUE)
    DEBUG_FLAGS = -g


    ifneq ($(LOG),NONE)
      ifeq ($(LOG),INFO)
        COMP_FLAGS += __LOG __LOG_INFO
      else
        ifeq ($(LOG),WARN)
          COMP_FLAGS += __LOG __LOG_WARN
        else
          ifeq ($(LOG),ERR)
            COMP_FLAGS += __LOG __LOG_ERR
          else
            $(error Bad value for LOG)
          endif
        endif
      endif
    endif


    ifeq ($(CMD),TRUE)
      COMP_FLAGS += __CMD
    else
      ifneq ($(CMD),FALSE)
        $(error Bad value for CMD)
      endif
    endif

    ifeq ($(PROF),TRUE)
      COMP_FLAGS += __PROF
    else
      ifneq ($(PROF),FALSE)
        $(error, Bad value for PROF)
      endif
    endif

    ifeq ($(TEST),TRUE)
	  COMP_FLAGS += __TEST
    else
      ifneq ($(TEST),FALSE)
	    $(error, Bad value for TEST)
      endif
    endif

  else
    $(error Bad value for DEBUG)
  endif
endif

ifeq ($(CAMERA),OV5642)
  COMP_FLAGS += __OV5642
else
  ifeq ($(CAMERA),OV7670)
	COMP_FLAGS += __OV7670
  else
  	$(error Bad value for CAMERA)
  endif
endif

ifneq ($(WIFI),NONE)
  ifeq ($(WIFI),ESP8266)
    COMP_FLAGS += __ESP8266 __WIFI
  else
    $(error Bad value for WIFI)
  endif
endif

ifeq ($(BOARD),STM32F429I_DISCOVERY)
  COMP_FLAGS += __STM32F429I_DISCOVERY
else
  ifeq ($(BOARD),S0LENS_A)
    COMP_FLAGS += __S0LENS_A
  endif
endif

# INCLUDES
# ----------------

include config/sources.mk
include config/tools.mk

# CONSTRUCTION
# -----------------

# Final CFLAGS with sources
CFLAGS  = $(DEBUG_FLAGS) --std=c99 -Wall -T$(LINKER_FILE)
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nosys.specs
CFLAGS += $(addprefix -I, $(INC_DIR))
CFLAGS += $(addprefix -D, $(COMP_FLAGS))
CFLAGS += -fvar-tracking -O0

RM_F = rm -f
MKDIR_P = mkdir -p
CP = cp

# TARGETS
# -----------------

# Default to build
all: build

# Build elf file
.PHONY: build
build: $(BIN_DIR)/$(PROJ_NAME).elf

$(BIN_DIR)/$(PROJ_NAME).elf: $(addprefix $(BUILD_DIR)/, $(OBJS))
	@$ $(MKDIR_P) $(BIN_DIR)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(BIN_DIR)/$(PROJ_NAME).elf $(BIN_DIR)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(BIN_DIR)/$(PROJ_NAME).elf $(BIN_DIR)/$(PROJ_NAME).bin

# Output object files from source
.PHONY: compile
compile: $(addprefix $(BUILD_DIR)/, $(OBJS))

$(BUILD_DIR)/%.o: %.c
	@$ $(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@$ $(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Output preprocessed files from source
.PHONY: preprocess
preprocess: $(addprefix $(BUILD_DIR)/, $(PRES))

$(BUILD_DIR)/%.i: %.c
	@$ $(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) -E $< -o $@

$(BUILD_DIR)/%.i: %.s
	@$ $(MKDIR_P) $(BUILD_DIR)
	# Do nothing

# Output assembly files from source
.PHONY: assemble
assemble: $(addprefix $(BUILD_DIR)/, $(ASMS))

$(BUILD_DIR)/%.s: %.c
	@$ $(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) -S $< -o $@

$(BUILD_DIR)/%.s: %.s
	@$ $(MKDIR_P) $(BUILD_DIR)
	@$ $(CP) $< $(BUILD_DIR)

# Clean build and bin folders
.PHONY: clean
clean:
	@$ $(RM_F) $(BIN_DIR)/* $(BUILD_DIR)/*

# Flash the STM32F4
burn: build
	st-flash write $(BIN_DIR)/$(PROJ_NAME).bin 0x8000000
