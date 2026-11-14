# Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
# Licensed under the MIT License
# https://opensource.org/licenses/MIT

# Shared HIL firmware build for STM32F411.
#
# The model-specific MCU compensator is NOT hardcoded here. The model's
# Makefile_HIL invokes this file and passes:
#   COMPENSATOR=<absolute path to models/<model>/compensator/compensator_STM32F411.c>
#   OUTPUT_DIR=<absolute path to build/<model>>
#
# Example (from a model directory):
#   $(MAKE) -C ../../hil/hil_STM32F4 \
#       COMPENSATOR=$(abspath compensator/compensator_STM32F411.c) \
#       OUTPUT_DIR=$(abspath ../../build/<model>)

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

GCCOPTIONS = -O0 -mcpu=cortex-m4 -g -mthumb -nostartfiles -mhard-float -mfpu=fpv4-sp-d16 -fno-exceptions -fno-unwind-tables

# This Makefile lives in hil/hil_STM32F4/; TinyUSB is vendored at the repo root.
TINYUSB_ROOT = ../../tinyusb-master

# Model-specific control law (supplied by models/<model>/Makefile_HIL).
COMPENSATOR ?=
# All build artifacts land here (normal case: build/<model>/).
OUTPUT_DIR ?= .
# Optional model/profile defines, e.g. -DHIL_TWO_OUTPUTS or -DFLYWHEEL_SPEED.
EXTRA_CFLAGS ?=

# Only require COMPENSATOR for actual builds, so `make clean` still works.
ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifeq ($(strip $(COMPENSATOR)),)
$(error COMPENSATOR is not set. Build through models/<model>/Makefile_HIL, or pass COMPENSATOR=<path>)
endif
endif

include_dirs = . include $(TINYUSB_ROOT)/src $(dir $(COMPENSATOR))
CFLAGS = $(addprefix -I, $(include_dirs)) $(EXTRA_CFLAGS)

sources = HIL_main.c \
          src/core_initialisation_STM32F411xe.c \
          src/startup_gcc.c \
          src/UART.c \
          src/printm.c \
          src/handlers_STM32F411xe.c \
          src/applications.c \
          src/usb_descriptors.c \
          src/usb_cdc_adapter.c \
          $(COMPENSATOR) \
          $(TINYUSB_ROOT)/src/tusb.c \
          $(TINYUSB_ROOT)/src/common/tusb_fifo.c \
          $(TINYUSB_ROOT)/src/device/usbd.c \
          $(TINYUSB_ROOT)/src/class/cdc/cdc_device.c \
          $(TINYUSB_ROOT)/src/portable/synopsys/dwc2/dcd_dwc2.c \
          $(TINYUSB_ROOT)/src/portable/synopsys/dwc2/dwc2_common.c

linker_script = MDCS_HIL.ld

OUT = $(OUTPUT_DIR)/MDCS_HIL.out
BIN = $(OUTPUT_DIR)/MDCS_HIL.bin
MAP = $(OUTPUT_DIR)/output.map

.PHONY: all clean

all: $(BIN)

$(OUTPUT_DIR):
	@mkdir -p $(OUTPUT_DIR)

$(BIN): $(OUT) | $(OUTPUT_DIR)
	$(OBJCOPY) -O binary $< $@

$(OUT): $(sources) | $(OUTPUT_DIR)
	$(CC) $(GCCOPTIONS) $(CFLAGS) $(sources) -T $(linker_script) -Wl,-Map=$(MAP) -o $@

clean:
	rm -f $(OUT) $(BIN) $(MAP)
