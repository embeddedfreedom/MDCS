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
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include "compensator.h"   
#include "furuta_types.h"
#include "encoder_decoder_logic.h"
#include "mdcs_config.h"
#include "hil_protocol.h"

static bool initialized = false;
static Decoder dec_arm, dec_pend;
static float t1_prev = 0, t2_prev = 0;
static int serial_fd = -1;

static void wrap_angle(float *a) {
    while (*a >  M_PI) *a -= 2.0f * M_PI;
    while (*a < -M_PI) *a += 2.0f * M_PI;
}

// This variable is used from the simulator to suspend the simulation on packet errors
extern bool runloop;

static void simulation_shutdown(void) {
    printf("CONNECTION ERROR >> COMPENSATOR DISCONNECTED PRESS 'S' TO RESTART SIMULATION\n");
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;  // Critical: enables open() to retry next cycle
    }
    runloop = false;
}
float run_compensator_cycle(EncoderBus *bus, FurutaParams *p, int *motor_dir) {

    if (!initialized) {
        decoder_init(&dec_arm); 
        decoder_init(&dec_pend);
        initialized = true;
    }

    // 1. Process 20kHz samples to reach current 1kHz state
    for (int i = 0; i < bus->n_arm; i++){
        decoder_process(&dec_arm, bus->arm_states[i]);
        }

    for (int i = 0; i < bus->n_pend; i++){
        decoder_process(&dec_pend, bus->pend_states[i]);
        }

    // --- Z-PULSE DRIFT COMPENSATION ---
    if (bus->z_arm) {
        dec_arm.count = (int)(roundf((float)dec_arm.count / 4096.0f) * 4096.0f);
    }

    if (bus->z_pend) {
        dec_pend.count = (int)(roundf((float)dec_pend.count / 4096.0f) * 4096.0f);
    }

    const float COUNT_TO_RAD = (2.0f * M_PI / 4096.0f);
    float t1 = (float)dec_arm.count * COUNT_TO_RAD;
    float t2 = (float)dec_pend.count * COUNT_TO_RAD;
    

    if (serial_fd < 0) {
    serial_fd = open(MDCS_HIL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
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

    float rx_pos1 = 0.0f;
    float rx_pos2 = 0.0f;
    float status  = 0.0f;
    
    if (serial_fd >= 0) {
        HostToMcuPacket tx_pkt;
        tx_pkt.sync[0] = HIL_HOST_SYNC_BYTE0;
        tx_pkt.sync[1] = HIL_HOST_SYNC_BYTE1;
        tx_pkt.pos1 = t1;
        tx_pkt.pos2 = t2;
        tx_pkt.host_cmd = HIL_CMD_RUN;
        tx_pkt.checksum = hil_calculate_checksum((const unsigned char *)&tx_pkt,
                                                 HIL_PACKET_SIZE - 1);

        ssize_t written = write(serial_fd, &tx_pkt, sizeof(HostToMcuPacket));
        if (written != (ssize_t)sizeof(HostToMcuPacket)) {
            simulation_shutdown();
            if (motor_dir) {
                 *motor_dir = 0;
    }
            return 0.0f;
        }

        struct pollfd pfd;
        pfd.fd = serial_fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 3);
        if (ret <= 0 || !(pfd.revents & POLLIN)) {
            simulation_shutdown();
            if (motor_dir) {
                 *motor_dir = 0;
    }
            return 0.0f;
        }

        McuToHostPacket rx_pkt;
        ssize_t bytes_read = read(serial_fd, &rx_pkt, sizeof(McuToHostPacket));
        
        if (bytes_read != (ssize_t)sizeof(McuToHostPacket)) {
            simulation_shutdown();
            if (motor_dir) {
                 *motor_dir = 0;
    }
            return 0.0f;
        }

        uint8_t rx_chk = hil_calculate_checksum((const unsigned char *)&rx_pkt,
                                                HIL_PACKET_SIZE - 1);

        if (rx_pkt.sync[0] != HIL_MCU_SYNC_BYTE0 ||
            rx_pkt.sync[1] != HIL_MCU_SYNC_BYTE1 ||
            rx_pkt.checksum != rx_chk) {
            simulation_shutdown();
            if (motor_dir) {
                 *motor_dir = 0;
    }
            return 0.0f;
        }

        // Extract received packet fields while rx_pkt is in scope
        rx_pos1 = rx_pkt.output1;
        rx_pos2 = rx_pkt.output2;
        status  = (float)rx_pkt.mcu_status;
    }

    // 4. Output Processing
    if (rx_pos1 > p->Vmax){
        rx_pos1 = p->Vmax;
       }
    if (rx_pos1 < -p->Vmax){
        rx_pos1 = -p->Vmax;
       }
          
    *motor_dir = (rx_pos1 >= 0) ? 1 : -1;
    return fabsf(rx_pos1) / p->Vmax;
}
   
    
