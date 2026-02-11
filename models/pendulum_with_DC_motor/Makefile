# Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
# Licensed under the MIT License
# https://opensource.org/licenses/MIT

# Path from the root of the repository
ROOT = ../..
# Define the specific directory for this project's output
# Placed at the top level, under the directory named 'inverted_pendulum'
BUILD_DIR = $(ROOT)/build/inverted_pendulum
COMMON = $(ROOT)/framework/common
CODECS = $(ROOT)/framework/transport/codecs

CC = gcc
# Include paths for simulator and compensator
CFLAGS = -Wall -O2 -I./simulator -I./compensator -I$(CODECS) -I$(COMMON)\
         -Wno-unused-variable -Wno-unused-result

# Sources
SRCS = simulator/pendulum_sim.c \
       compensator/compensator.c \
       $(CODECS)/encoder_decoder_logic.c \
	   $(COMMON)/get_keypress.c  
		 
# The binary file sitting inside the 'inverted_pendulum' build directory
TARGET = $(BUILD_DIR)/inverted_pendulum_sim

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

# Shortcut to run the C-simulation
run: all
	./$(TARGET)

.PHONY: all setup clean run