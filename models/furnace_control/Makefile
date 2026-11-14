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
BUILD_DIR = $(ROOT)/build/furnace_control
COMMON = $(ROOT)/framework/common

CC = gcc
CFLAGS = -Wall -O2 -I./simulator -I./compensator -I$(COMMON) \
         -Wno-unused-variable -Wno-unused-result

# Sources (SIL: furnace simulator + LQI compensator + keypress)
SRCS = simulator/furnace_sim.c \
       compensator/compensator.c \
	   $(COMMON)/get_keypress.c

# The binary file itself sitting inside the directory
TARGET = $(BUILD_DIR)/furnace_sim

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
