PROJECT_NAME     := ble_app_ancs_c_pca10028_s130
TARGETS          := nrf51422_xxac
OUTPUT_DIRECTORY := _build_nRF51

SDK_ROOT := ../../..
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/nrf51422_xxac.out: \
  LINKER_SCRIPT  := ble_app_ancs_c_gcc_nrf51.ld
  
include $(PROJ_DIR)/Makefile.files

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.S \
  $(SDK_ROOT)/components/toolchain/system_nrf51.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/softdevice/s130/headers \
  $(SDK_ROOT)/components/softdevice/s130/headers/nrf51 \
  $(PROJ_DIR)/pca10028/s130/config \

# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
#CFLAGS += -D__HEAP_SIZE=0
#CFLAGS += -D__STACK_SIZE=0
CFLAGS += -DNRF51422
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -DARDUINO=164
# CFLAGS += -DDEBUG
CFLAGS += -DNRF_LOG_ENABLED=0
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Ofast
CFLAGS += -mfloat-abi=soft
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums -nostdlib

# C++ flags common to all targets
CXXFLAGS += -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF51422
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys


.PHONY: $(TARGETS) default prod all force clean help flash flash_softdevice dfu

# Default target - first one defined
default: nrf51422_xxac

prod: CFLAGS += -DNRF_LOG_ENABLED=0
prod: nrf51422_xxac

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac
	
dfu:
	nrfutil --verbose pkg generate --hw-version 51 --sd-req 0x87 --application-version 1 --application $(OUTPUT_DIRECTORY)/nrf51422_xxac.hex --key-file $(SDK_ROOT)/vault/priv.pem s130_watch.zip  
	mv s130_watch.zip ..\..\..\..\..\Dropbox\

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf51422_xxac.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf51 --sectorerase
	nrfjprog --reset -f nrf51

# Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.1_softdevice.hex
	nrfjprog --program $(SDK_ROOT)/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex -f nrf51 --sectorerase 
	nrfjprog --reset -f nrf51

erase:
	nrfjprog --eraseall -f nrf52
