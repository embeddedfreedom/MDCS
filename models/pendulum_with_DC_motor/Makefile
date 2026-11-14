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