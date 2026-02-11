# Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
# Licensed under the MIT License
# https://opensource.org/licenses/MIT

# Path from models/inverted_rotary_pendulum/
ROOT = ../..
# Define the specific directory for this project's output
BUILD_DIR = $(ROOT)/build/furuta
COMMON = $(ROOT)/framework/common
CODECS = $(ROOT)/framework/transport/codecs


CC = gcc
# Added -I$(ROOT)/models/inverted_rotary_pendulum/simulator to be safe
CFLAGS = -Wall -O2 -I./simulator -I./compensator -I$(CODECS) -I$(COMMON) \
         -Wno-unused-variable -Wno-unused-result

# Sources
SRCS = simulator/furuta_sim.c \
       compensator/compensator.c \
       $(CODECS)/encoder_decoder_logic.c \
	   $(COMMON)/get_keypress.c

# The binary file itself sitting inside the directory
TARGET = $(BUILD_DIR)/furuta_sim

all: setup $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) -lm
	@echo "------------------------------------------------"
	@echo "Build successful!"
	@echo "Binary located at: $(TARGET)"
	@echo "------------------------------------------------"

setup:
	@mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all setup clean