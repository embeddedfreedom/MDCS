/* 
Copyright (C) 2026 Diptopal Basu (embeddedfreedom)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include "compensator.h"
#include "furnace_types.h"

#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       
    float    temp1;          
    float    temp2;          
    uint8_t  host_cmd;      
    uint8_t  checksum;      
} HostToMcuPacket;

_Static_assert(sizeof(HostToMcuPacket) == 12, "ERROR: HostToMcuPacket is not 12 bytes!");

typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       
    float    output1;       
    float    output2;       
    uint8_t  mcu_status;    
    uint8_t  checksum;      
} McuToHostPacket;

_Static_assert(sizeof(McuToHostPacket) == 12, "ERROR: McuToHostPacket is not 12 bytes!");
#pragma pack(pop)

static bool initialized = false;
static float t1_prev = 0, t2_prev = 0;
static int serial_fd = -1;

/* This variable is used from the simulator to suspend the simulation on packet errors */
extern bool runloop;

static void simulation_shutdown(void) {
    printf("CONNECTION ERROR >> COMPENSATOR DISCONNECTED PRESS 'S' TO RESTART SIMULATION\n");
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;  // Critical: enables open() to retry next cycle
    }
    runloop = false;
}

void run_thermal_compensator_cycle(float measured_T1, float measured_T2,
                                   const ThermalParams *p,
                                   float *u1, float *u2) {

    if (!initialized) {
        initialized = true;
    }

    (void)p; /* Host-side HIL: gains live on the MCU, parameters unused here */

    /* --- Open serial port to MCU on first cycle --- */
    if (serial_fd < 0) {
        serial_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd < 0) {
            simulation_shutdown();
            *u1 = 0.0f;
            *u2 = 0.0f;
            return;
        } else {
            struct termios tty;
            if (tcgetattr(serial_fd, &tty) != 0) {
                close(serial_fd);
                serial_fd = -1;
                simulation_shutdown();
                *u1 = 0.0f;
                *u2 = 0.0f;
                return;
            } else {
                cfmakeraw(&tty);
                cfsetispeed(&tty, B921600);
                cfsetospeed(&tty, B921600);
                if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
                    close(serial_fd);
                    serial_fd = -1;
                    simulation_shutdown();
                    *u1 = 0.0f;
                    *u2 = 0.0f;
                    return;
                }
            }
        }
    }

    /* --- Build and send packet to MCU: sync(2) + temp1 + temp2 + cmd + checksum = 12 bytes --- */
    HostToMcuPacket tx_pkt;
    tx_pkt.sync[0] = 0xAA;
    tx_pkt.sync[1] = 0x55;
    tx_pkt.temp1 = measured_T1;
    tx_pkt.temp2 = measured_T2;
    tx_pkt.host_cmd = 0x01;

    uint8_t *tx_bytes = (uint8_t *)&tx_pkt;
    uint8_t checksum = 0;
    for (int i = 0; i < 11; i++) {
        checksum ^= tx_bytes[i];
    }
    tx_pkt.checksum = checksum;

    ssize_t written = write(serial_fd, &tx_pkt, sizeof(HostToMcuPacket));
    if (written != (ssize_t)sizeof(HostToMcuPacket)) {
        simulation_shutdown();
        *u1 = 0.0f;
        *u2 = 0.0f;
        return;
    }

    /* --- Wait for response from MCU (3 ms poll timeout) --- */
    struct pollfd pfd;
    pfd.fd = serial_fd;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 3);
    if (ret <= 0 || !(pfd.revents & POLLIN)) {
        simulation_shutdown();
        *u1 = 0.0f;
        *u2 = 0.0f;
        return;
    }

    McuToHostPacket rx_pkt;
    ssize_t bytes_read = read(serial_fd, &rx_pkt, sizeof(McuToHostPacket));

    if (bytes_read != (ssize_t)sizeof(McuToHostPacket)) {
        simulation_shutdown();
        *u1 = 0.0f;
        *u2 = 0.0f;
        return;
    }

    /* --- Validate response: sync 0x55 0xAA, XOR checksum --- */
    uint8_t *rx_bytes = (uint8_t *)&rx_pkt;
    uint8_t rx_chk = 0;
    for (int i = 0; i < 11; i++) {
        rx_chk ^= rx_bytes[i];
    }

    if (rx_pkt.sync[0] != 0x55 || rx_pkt.sync[1] != 0xAA || rx_pkt.checksum != rx_chk) {
        simulation_shutdown();
        *u1 = 0.0f;
        *u2 = 0.0f;
        return;
    }

    /* --- Extract heater commands from MCU --- */
    *u1 = rx_pkt.output1;
    *u2 = rx_pkt.output2;

    /* --- Clamp to heater PWM limits [0, 1] (no active cooling) --- */
    if (*u1 > 1.0f) *u1 = 1.0f;
    if (*u1 < 0.0f) *u1 = 0.0f;
    if (*u2 > 1.0f) *u2 = 1.0f;
    if (*u2 < 0.0f) *u2 = 0.0f;
}
