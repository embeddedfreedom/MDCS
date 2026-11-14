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
#include "compensator_hil.h"
#include "flywheel_types.h"
#include "encoder_decoder_logic.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* A real index pulse occurs only at the revolution boundary. The simulator
   emits a one-shot pulse on window entry; gate the snap so a coarse window
   position can never erase real motion. */
#define Z_SNAP_TOL_COUNTS 64

#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];
    float    pos1;
    float    pos2;
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
static Decoder20 dec_flywheel;
static int serial_fd = -1;

/* This variable is used from the simulator to suspend the simulation on packet errors */
extern bool runloop;

static void wrap_angle(float *a) {
    while (*a >  M_PI) *a -= 2.0f * M_PI;
    while (*a < -M_PI) *a += 2.0f * M_PI;
}

static void simulation_shutdown(void) {
    printf("CONNECTION ERROR >> COMPENSATOR DISCONNECTED PRESS 'S' TO RESTART SIMULATION\n");
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;  /* Critical: enables open() to retry next cycle */
    }
    runloop = false;
}

float run_compensator_cycle(EncoderBus *bus, FlywheelParams *p, int *motor_dir) {
    if (!initialized) {
        decoder20_init(&dec_flywheel);
        initialized = true;
    }

    /* 1. Decode the 20 kHz quadrature stream accumulated over the last 1 ms */
    for (int i = 0; i < bus->n_pend; i++) {
        decoder20_process(&dec_flywheel, bus->pend_states[i]);
    }

    /* Z-pulse drift compensation (only near a real revolution boundary) */
    if (bus->z_pend) {
        int nearest = (int)(roundf((float)dec_flywheel.count /
                                   (float)ENCODER20_COUNTS_PER_REV) *
                            (float)ENCODER20_COUNTS_PER_REV);
        int delta = dec_flywheel.count - nearest;
        if (delta < 0) delta = -delta;
        if (delta <= Z_SNAP_TOL_COUNTS) {
            dec_flywheel.count = nearest;
            while (dec_flywheel.count >  ENCODER20_DECODER_WRAP - 1)
                dec_flywheel.count -= ENCODER20_COUNTS_PER_REV;
            while (dec_flywheel.count < -ENCODER20_DECODER_WRAP)
                dec_flywheel.count += ENCODER20_COUNTS_PER_REV;
        }
    }

    const float COUNT_TO_RAD = (2.0f * M_PI / (float)ENCODER20_COUNTS_PER_REV);
    float theta = (float)dec_flywheel.count * COUNT_TO_RAD;
    wrap_angle(&theta);

    /* 2. Lazy-open and configure the physical MCU link */
    if (serial_fd < 0) {
        serial_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd < 0) {
            simulation_shutdown();
        } else {
            struct termios tty;
            if (tcgetattr(serial_fd, &tty) != 0) {
                close(serial_fd);
                serial_fd = -1;
                simulation_shutdown();
            } else {
                cfmakeraw(&tty);
                cfsetispeed(&tty, B921600);
                cfsetospeed(&tty, B921600);
                if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
                    close(serial_fd);
                    serial_fd = -1;
                    simulation_shutdown();
                }
            }
        }
    }

    float rx_output = 0.0f;

    if (serial_fd >= 0) {
        /* Build and transmit: sync(2) + theta + 0 + cmd + checksum = 12 bytes */
        HostToMcuPacket tx_pkt;
        tx_pkt.sync[0] = 0xAA;
        tx_pkt.sync[1] = 0x55;
        tx_pkt.pos1 = theta;
        tx_pkt.pos2 = 0.0f;
        tx_pkt.host_cmd = 0x01;

        uint8_t *tx_bytes = (uint8_t *)&tx_pkt;
        uint8_t checksum = 0;
        for (int i = 0; i < (int)sizeof(HostToMcuPacket) - 1; i++) {
            checksum ^= tx_bytes[i];
        }
        tx_pkt.checksum = checksum;

        ssize_t written = write(serial_fd, &tx_pkt, sizeof(HostToMcuPacket));
        if (written != (ssize_t)sizeof(HostToMcuPacket)) {
            simulation_shutdown();
            if (motor_dir) *motor_dir = 0;
            return 0.0f;
        }

        /* Wait for the MCU response (3 ms timeout) */
        struct pollfd pfd;
        pfd.fd = serial_fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 3);
        if (ret <= 0 || !(pfd.revents & POLLIN)) {
            simulation_shutdown();
            if (motor_dir) *motor_dir = 0;
            return 0.0f;
        }

        McuToHostPacket rx_pkt;
        ssize_t bytes_read = read(serial_fd, &rx_pkt, sizeof(McuToHostPacket));

        if (bytes_read != (ssize_t)sizeof(McuToHostPacket)) {
            simulation_shutdown();
            if (motor_dir) *motor_dir = 0;
            return 0.0f;
        }

        /* Validate response: sync 0x55 0xAA, XOR checksum */
        uint8_t *rx_bytes = (uint8_t *)&rx_pkt;
        uint8_t rx_chk = 0;
        for (int i = 0; i < (int)sizeof(McuToHostPacket) - 1; i++) {
            rx_chk ^= rx_bytes[i];
        }

        if (rx_pkt.sync[0] != 0x55 || rx_pkt.sync[1] != 0xAA || rx_pkt.checksum != rx_chk) {
            simulation_shutdown();
            if (motor_dir) *motor_dir = 0;
            return 0.0f;
        }

        rx_output = rx_pkt.output1;
    }

    /* 3. Output constraints: MCU sends signed volts */
    if (rx_output > p->Vmax)  rx_output = p->Vmax;
    if (rx_output < -p->Vmax) rx_output = -p->Vmax;

    *motor_dir = (rx_output >= 0) ? 1 : -1;
    return fabsf(rx_output) / p->Vmax;
}
