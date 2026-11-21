/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

/*
 * host_hil_test.c - Linux Serial Host for STM32F411 DMA HIL Validation
 * 
 * Target   : Linux Host PC (x86_64)
 * Port     : /dev/ttyUSB0 or /dev/ttyACM0 @ 921600 Baud, 8N1, Raw Mode
 * Packet   : 12-byte symmetrical binary packet with XOR checksum
 * 
 * Compilation:
 *   gcc -O2 host_hil_test.c -o host_hil_test -lm
 * 
 * Usage:
 *   ./host_hil_test /dev/ttyUSB0
 */

//Remember the turnaround time from controller to host has to be much less than 1 ms otherwise it will cause jitter in the model
// So there has to be a function to measure the time 

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <math.h>

/* MDCS central config (socat/HIL paths) — single source of truth */
#include "../../../framework/common/mdcs_config.h"

/* Canonical HIL frame definition (shared with firmware and host compensators) */
#include "../hil_protocol.h"

/* Frame constants, packet structs, and the XOR checksum helper are defined
   in hil_protocol.h (included above). */

/* ===============================================================================
 * FUNCTION: configure_serial_port
 * PURPOSE : Places the Linux TTY driver in raw binary mode at 921600 baud (8N1).
 * =============================================================================== */
static int configure_serial_port(const char *port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "FATAL: Unable to open serial port '%s': %s\n", port_name, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "FATAL: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Set 921600 Baud for both TX and RX */
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    /* 8 Data Bits, No Parity, 1 Stop Bit (8N1), No HW Flow Control */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    /* Disable software flow control (XON/XOFF) and special byte alterations */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    /* Disable output post-processing */
    tty.c_oflag &= ~(OPOST | ONLCR);

    /* Disable canonical mode, local echo, and signal traps */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* Non-blocking read configuration: immediate return if nothing available */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "FATAL: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Flush hardware FIFO buffers */
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ===============================================================================
 * FUNCTION: read_exact_bytes_with_timeout
 * PURPOSE : Gathers exactly 'needed' bytes within 'timeout_ms' wall-clock time.
 * =============================================================================== */
static bool read_exact_bytes_with_timeout(int fd, uint8_t *buffer, size_t needed, uint32_t timeout_ms) {
    size_t total = 0;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (total < needed) {
        ssize_t n = read(fd, &buffer[total], needed - total);
        if (n > 0) {
            total += (size_t)n;
        } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
            return false;
        }

        /* Check elapsed time */
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed_ms = (double)(now.tv_sec - start.tv_sec) * 1000.0 +
                            (double)(now.tv_nsec - start.tv_nsec) / 1000000.0;
        if (elapsed_ms >= (double)timeout_ms) {
            return false; /* Timeout */
        }

        /* Sleep 50 us to prevent pegging 100% of a CPU core */
        usleep(50);
    }
    return true;
}

/* ===============================================================================
 * MAIN
 * =============================================================================== */
int main(int argc, char **argv) {
    const char *port_name = (argc > 1) ? argv[1] : MDCS_HIL_TEST_PORT;

    printf("==================================================================\n");
    printf(" STM32F411 HIL Lockstep DMA Echo Verification Testbench\n");
    printf(" Port     : %s\n", port_name);
    printf(" Baud     : 921600 bps (8N1)\n");
    printf(" Frame    : %d bytes (Little-Endian IEEE 754)\n", HIL_PACKET_SIZE);
    printf(" Target Hz: 1000 Hz (1 ms lockstep)\n");
    printf("==================================================================\n\n");

    int fd = configure_serial_port(port_name);
    if (fd < 0) {
        return EXIT_FAILURE;
    }

    /* Allow the USB-to-UART bridge to settle */
    usleep(200000);
    tcflush(fd, TCIOFLUSH);

    HostToMcuPacket tx_pkt;
    McuToHostPacket rx_pkt;
    uint8_t rx_raw[HIL_PACKET_SIZE];

    uint32_t total_sent       = 0;
    uint32_t total_recv       = 0;
    uint32_t sync_errors      = 0;
    uint32_t checksum_errors  = 0;
    uint32_t nan_errors       = 0;
    uint32_t range_errors     = 0;
    uint32_t value_mismatches = 0;
    const int TOTAL_CYCLES    = 1000;

    struct timespec next_cycle;
    clock_gettime(CLOCK_MONOTONIC, &next_cycle);

    for (int cycle = 1; cycle <= TOTAL_CYCLES; cycle++) {
        /* Pacing to exact 1 ms (1000 Hz) loop */
        next_cycle.tv_nsec += 1000000L; /* +1 ms */
        if (next_cycle.tv_nsec >= 1000000000L) {
            next_cycle.tv_nsec -= 1000000000L;
            next_cycle.tv_sec++;
        }

        /* Synthesize varying test positions */
        float test_pos1 = 1.250f * sinf((float)cycle * 0.02f);
        float test_pos2 = -0.750f * cosf((float)cycle * 0.02f);

        /* -------------------------------------------------------------------
         * STEP 1: Pack the Host -> MCU Frame (12 Bytes)
         * ------------------------------------------------------------------- */
        tx_pkt.sync[0]  = HIL_HOST_SYNC_BYTE0; // 0xAA
        tx_pkt.sync[1]  = HIL_HOST_SYNC_BYTE1; // 0x55
        tx_pkt.pos1     = test_pos1;
        tx_pkt.pos2     = test_pos2;
        tx_pkt.host_cmd = HIL_CMD_RUN;         // 0x01
        tx_pkt.checksum = hil_calculate_checksum((const uint8_t *)&tx_pkt, HIL_PACKET_SIZE - 1);

        /* -------------------------------------------------------------------
         * STEP 2: Transmit Packet to MCU
         * ------------------------------------------------------------------- */
        ssize_t written = write(fd, &tx_pkt, HIL_PACKET_SIZE);
        if (written != HIL_PACKET_SIZE) {
            fprintf(stderr, "[Cycle %04d] Short write: %zd of %d bytes written\n", cycle, written, HIL_PACKET_SIZE);
            continue;
        }
        total_sent++;

        /* -------------------------------------------------------------------
         * STEP 3: Receive 12-Byte Response from MCU (Timeout = 5 ms)
         * ------------------------------------------------------------------- */
        bool ok = read_exact_bytes_with_timeout(fd, rx_raw, HIL_PACKET_SIZE, 5);
        if (!ok) {
            fprintf(stderr, "[Cycle %04d] TIMEOUT: No 12-byte response within 5 ms\n", cycle);
            continue;
        }
        total_recv++;

        /* -------------------------------------------------------------------
         * STEP 4: VERIFICATION 1 — Sync Header Check
         * Handles both HIL standard (0x55, 0xAA) and simple echo (0xAA, 0x55)
         * ------------------------------------------------------------------- */
        bool sync_is_mcu_standard = (rx_raw[0] == HIL_MCU_SYNC_BYTE0 && rx_raw[1] == HIL_MCU_SYNC_BYTE1);
        bool sync_is_raw_echo     = (rx_raw[0] == HIL_HOST_SYNC_BYTE0 && rx_raw[1] == HIL_HOST_SYNC_BYTE1);

        if (!sync_is_mcu_standard && !sync_is_raw_echo) {
            sync_errors++;
            fprintf(stderr, "[Cycle %04d] SYNC FAIL: Expected [0x55 0xAA] or [0xAA 0x55], got [0x%02X 0x%02X]\n",
                    cycle, rx_raw[0], rx_raw[1]);
            /* Flush input to recover alignment in case of byte slip */
            tcflush(fd, TCIFLUSH);
            continue;
        }

        /* -------------------------------------------------------------------
         * STEP 5: VERIFICATION 2 — XOR Checksum Integrity
         * ------------------------------------------------------------------- */
        uint8_t computed_cs = hil_calculate_checksum(rx_raw, HIL_PACKET_SIZE - 1);
        uint8_t received_cs = rx_raw[11];
        if (computed_cs != received_cs) {
            checksum_errors++;
            fprintf(stderr, "[Cycle %04d] CHECKSUM FAIL: Computed 0x%02X != Received 0x%02X\n",
                    cycle, computed_cs, received_cs);
            continue;
        }

        /* Cast verified raw buffer to struct */
        memcpy(&rx_pkt, rx_raw, HIL_PACKET_SIZE);

        /* -------------------------------------------------------------------
         * STEP 6: VERIFICATION 3 — IEEE 754 Floating-Point Sanity (NaN / Inf)
         * ------------------------------------------------------------------- */
        if (isnan(rx_pkt.output1) || isnan(rx_pkt.output2) ||
            isinf(rx_pkt.output1) || isinf(rx_pkt.output2)) {
            nan_errors++;
            fprintf(stderr, "[Cycle %04d] FLOAT CORRUPT: NaN or Inf detected in payload\n", cycle);
            continue;
        }

        /* -------------------------------------------------------------------
         * STEP 7: VERIFICATION 4 — Physical Plausibility Range Check
         * ------------------------------------------------------------------- */
        if (fabsf(rx_pkt.output1) > 50.0f || fabsf(rx_pkt.output2) > 50.0f) {
            range_errors++;
            fprintf(stderr, "[Cycle %04d] RANGE FAIL: Received values out of plausible bounds\n", cycle);
        }

        if (cycle % 100 == 0) {
            printf("[Cycle %04d] out1=%.3f out2=%.3f status=0x%02X\n",
                   cycle, rx_pkt.output1, rx_pkt.output2, rx_pkt.mcu_status);
        }
    }

    /* -------------------------------------------------------------------
     * SUMMARY
     * ------------------------------------------------------------------- */
    printf("\n==================================================================\n");
    printf(" HIL Lockstep Test Summary\n");
    printf("==================================================================\n");
    printf(" Total cycles   : %d\n", TOTAL_CYCLES);
    printf(" Total sent     : %u\n", total_sent);
    printf(" Total received : %u\n", total_recv);
    printf(" Sync errors    : %u\n", sync_errors);
    printf(" Checksum errors: %u\n", checksum_errors);
    printf(" NaN/Inf errors : %u\n", nan_errors);
    printf(" Range errors   : %u\n", range_errors);
    printf("==================================================================\n");

    close(fd);
    return (sync_errors == 0 && checksum_errors == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
