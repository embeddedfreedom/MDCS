# 
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

ROOT = ../../..
# Define the specific directory for this project's output
BUILD_DIR = $(ROOT)/build/flywheel_control
COMMON = $(ROOT)/framework/common
CODECS = $(ROOT)/framework/transport/codecs

CC = gcc
CFLAGS = -Wall -O2 -I./simulator -I$(CODECS) -I$(COMMON) -Wno-unused-variable -Wno-unused-result

# Shared base sources (simulator + input helper)
CORE_SRCS = simulator/flywheel_sim.c \
            $(COMMON)/get_keypress.c

# Explicit Output Binary Destinations
TARGET_POS = $(BUILD_DIR)/flywheel_position_sim
TARGET_SPEED = $(BUILD_DIR)/flywheel_speed_sim

.PHONY: all position speed clean help setup

# Default rule displays an instruction overview
all: help

position: CFLAGS += -I./compensator/position_control -DPOSITION_PROFILE
position: setup $(TARGET_POS)

$(TARGET_POS): $(CORE_SRCS) $(CODECS)/encoder_decoder_logic.c compensator/position_control/compensator.c
	@echo ">> Compiling Modular Suite: POSITION CONTROL PROFILE..."
	$(CC) $(CFLAGS) $(CORE_SRCS) $(CODECS)/encoder_decoder_logic.c compensator/position_control/compensator.c -o $(TARGET_POS) -lm
	@echo "------------------------------------------------"
	@echo "Position Control Build Successful!"
	@echo "Run via: ./$(TARGET_POS)"
	@echo "------------------------------------------------"

speed: CFLAGS += -I./compensator/speed_control -DSPEED_PROFILE
speed: setup $(TARGET_SPEED)

$(TARGET_SPEED): $(CORE_SRCS) $(CODECS)/encoder_decoder_logic.c compensator/speed_control/compensator.c
	@echo ">> Compiling Modular Suite: SPEED CONTROL PROFILE..."
	$(CC) $(CFLAGS) $(CORE_SRCS) $(CODECS)/encoder_decoder_logic.c compensator/speed_control/compensator.c -o $(TARGET_SPEED) -lm
	@echo "------------------------------------------------"
	@echo "Speed Control Build Successful!"
	@echo "Run via: ./$(TARGET_SPEED)"
	@echo "------------------------------------------------"

setup:
	@mkdir -p $(BUILD_DIR)

clean:
	@echo "Cleaning up generated artifacts..."
	rm -rf $(BUILD_DIR)

help:
	@echo "================================================================="
	@echo "            Modular Dynamics Control Suite - Flywheel Engine      "
	@echo "================================================================="
	@echo "Usage:"
	@echo "  make position  - Compiles the isolated position tracking executable"
	@echo "  make speed     - Compiles the isolated continuous velocity executable"
	@echo "  make clean     - Wipes the output build directories"
	@echo "  make help      - Displays this reference guide"
	@echo ""
	@echo "  (For Hardware-in-the-Loop, use: make -f Makefile_HIL hil)"
	@echo "================================================================="
	@echo "Output Binaries will be safely generated under:"
	@echo "  $(BUILD_DIR)/"
	@echo "================================================================="
