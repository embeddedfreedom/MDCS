/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"



#ifndef HIL_PACKET_DEFS_H
#define HIL_PACKET_DEFS_H


#define HIL_HOST_SYNC_BYTE0    0xAA
#define HIL_HOST_SYNC_BYTE1    0x55

#define HIL_MCU_SYNC_BYTE0     0x55
#define HIL_MCU_SYNC_BYTE1     0xAA

#define HIL_PACKET_SIZE        12

/* Host Commands */
#define HIL_CMD_STOP           0x00
#define HIL_CMD_RUN            0x01
#define HIL_CMD_RESET          0x02

/* MCU Status Flags */
#define HIL_STATUS_RUNNING     (1 << 0)  /* PA0 Button State (1=Run, 0=Stop) */
#define HIL_STATUS_TIMEOUT     (1 << 1)  /* Watchdog tripped (1=Failsafe active) */

#pragma pack(push, 1)

/*
 * =========================================================================
 * 1. HOST -> MCU PACKET (Received via DMA1 Stream 5)
 * =========================================================================
 * Total Size: Exactly 12 Bytes (Zero Padding)
 *
 * Byte Offsets:
 * [0..1]  sync[2]   (2 bytes) -> 0xAA, 0x55
 * [2..5]  pos1      (4 bytes) -> float (Primary angle/position, e.g., rad)
 * [6..9]  pos2      (4 bytes) -> float (Secondary angle/position, 0.0f if unused)
 * [10]    host_cmd  (1 byte)  -> 0x01 = RUN, 0x00 = STOP
 * [11]    checksum  (1 byte)  -> XOR of bytes 0..10
 * =========================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /* Offset 0..1  : Frame synchronization magic */
    float    pos1;          /* Offset 2..5  : Primary plant position/angle */
    float    pos2;          /* Offset 6..9  : Secondary plant position/angle */
    uint8_t  host_cmd;      /* Offset 10    : Run/Stop command */
    uint8_t  checksum;      /* Offset 11    : XOR of bytes 0..10 */
} HostToMcuPacket;

/* Compile-time verification: Build halts if compiler inserts padding */
_Static_assert(sizeof(HostToMcuPacket) == 12, "ERROR: HostToMcuPacket is not 12 bytes!");


/*
 * =========================================================================
 * 2. MCU -> HOST PACKET (Transmitted via DMA1 Stream 6)
 * =========================================================================
 * Total Size: Exactly 12 Bytes (Zero Padding)
 *
 * Byte Offsets:
 * [0..1]  sync[2]     (2 bytes) -> 0x55, 0xAA
 * [2..5]  output1     (4 bytes) -> float (Primary control effort: e.g. signed duty -1.0 to +1.0)
 * [6..9]  output2     (4 bytes) -> float (Secondary control effort, or telemetry/gain value)
 * [10]    mcu_status  (1 byte)  -> Bit 0: PA0 Run/Stop state, Bit 1: Watchdog
 * [11]    checksum    (1 byte)  -> XOR of bytes 0..10
 * =========================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /* Offset 0..1  : Frame synchronization magic */
    float    output1;       /* Offset 2..5  : Primary control effort */
    float    output2;       /* Offset 6..9  : Secondary output / live telemetry */
    uint8_t  mcu_status;    /* Offset 10    : Status flags */
    uint8_t  checksum;      /* Offset 11    : XOR of bytes 0..10 */
} McuToHostPacket;

/* Compile-time verification: Build halts if compiler inserts padding */
_Static_assert(sizeof(McuToHostPacket) == 12, "ERROR: McuToHostPacket is not 12 bytes!");

#pragma pack(pop)


/*
 * Fast 11-byte XOR Checksum calculation (reusable for both directions)
 */
static inline uint8_t calculate_hil_checksum(const uint8_t *buffer) {
    uint8_t cs = 0;
    for (int i = 0; i < (HIL_PACKET_SIZE - 1); i++) {
        cs ^= buffer[i];
    }
    return cs;
}

#endif /* HIL_PACKET_DEFS_H */