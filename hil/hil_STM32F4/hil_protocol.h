/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

/**
 * @file hil_protocol.h
 * @brief Canonical 12-byte MDCS HIL frame definition.
 *
 * Single source of truth shared by:
 *   - the STM32F411 firmware transport
 *     (core_initialisation_STM32F411xe.h, src/usb_cdc_adapter.c)
 *   - every model's host-side compensator_hil.c
 *   - the host lockstep test (test/host_HIL_test.c)
 *
 * This header deliberately includes no libc headers. The bare-metal firmware
 * defines its own integer types, while host code includes <stdint.h>; using
 * plain `unsigned char` for the 1-byte fields keeps the layout identical in
 * both environments and avoids conflicting typedefs.
 *
 * Change the frame layout, sync bytes, command/status bits, or checksum
 * algorithm HERE and rebuild. The _Static_asserts below fail the build if
 * padding or a size mismatch appears.
 */

#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#define HIL_PACKET_SIZE        12

/* Host -> MCU sync */
#define HIL_HOST_SYNC_BYTE0    0xAA
#define HIL_HOST_SYNC_BYTE1    0x55

/* MCU -> host sync */
#define HIL_MCU_SYNC_BYTE0     0x55
#define HIL_MCU_SYNC_BYTE1     0xAA

/* Host commands */
#define HIL_CMD_STOP           0x00
#define HIL_CMD_RUN            0x01
#define HIL_CMD_RESET          0x02

/* MCU status flags */
#define HIL_STATUS_RUNNING     (1 << 0)  /* PA0 button state (1 = run) */
#define HIL_STATUS_TIMEOUT     (1 << 1)  /* watchdog tripped */

#pragma pack(push, 1)

/* Host -> MCU: sync(2) + pos1(4) + pos2(4) + cmd(1) + checksum(1) */
typedef struct __attribute__((packed)) {
    unsigned char sync[2];
    float         pos1;
    float         pos2;
    unsigned char host_cmd;
    unsigned char checksum;
} HostToMcuPacket;

/* MCU -> host: sync(2) + out1(4) + out2(4) + status(1) + checksum(1) */
typedef struct __attribute__((packed)) {
    unsigned char sync[2];
    float         output1;
    float         output2;
    unsigned char mcu_status;
    unsigned char checksum;
} McuToHostPacket;

#pragma pack(pop)

_Static_assert(sizeof(HostToMcuPacket) == HIL_PACKET_SIZE,
               "ERROR: HostToMcuPacket must be 12 bytes");
_Static_assert(sizeof(McuToHostPacket) == HIL_PACKET_SIZE,
               "ERROR: McuToHostPacket must be 12 bytes");

/**
 * XOR checksum over the first len bytes (normally HIL_PACKET_SIZE - 1).
 * The checksum byte itself is excluded.
 */
static inline unsigned char hil_calculate_checksum(const unsigned char *buf,
                                                   unsigned len) {
    unsigned char cs = 0;
    unsigned i;
    for (i = 0; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

#endif /* HIL_PROTOCOL_H */
