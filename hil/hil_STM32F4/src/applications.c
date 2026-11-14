/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"

/*
 * ===============================================================================
 * FUNCTION: Extract_HostToMcuPacket
 * PURPOSE : Validates sync/checksum on raw RX buffer and extracts payload variables.
 * RETURNS : true if packet is valid; false if corrupted or misaligned.
 * ===============================================================================
 */
bool Extract_HostToMcuPacket(const uint8_t *raw_buf, float *pos1_out, float *pos2_out, uint8_t *cmd_out) {
    /* Cast raw byte array to the packed struct format */
    const HostToMcuPacket *pkt = (const HostToMcuPacket *)raw_buf;

    /* ---------------------------------------------------------------------------
     * STEP 1: Verify Frame Synchronization Magic (0xAA, 0x55)
     * --------------------------------------------------------------------------- */
    if (pkt->sync[0] != 0xAA || pkt->sync[1] != 0x55) {
        return false;
    }

    /* ---------------------------------------------------------------------------
     * STEP 2: Compute and Verify XOR Checksum (Bytes 0 through 10)
     * --------------------------------------------------------------------------- */
    uint8_t computed_cs = 0;
    for (int i = 0; i < 11; i++) {
        computed_cs ^= raw_buf[i];
    }
    if (computed_cs != pkt->checksum) {
        return false;
    }

    /* ---------------------------------------------------------------------------
     * STEP 3: Extract Validated Payload Data
     * --------------------------------------------------------------------------- */
    *pos1_out = pkt->pos1;
    *pos2_out = pkt->pos2;
    *cmd_out  = pkt->host_cmd;

    return true;
}



/*
 * ===============================================================================
 * FUNCTION: Write_McuToHostPacket
 * PURPOSE : Packs control loop telemetry into the TX buffer and computes checksum.
 * ===============================================================================
 */
void Write_McuToHostPacket(uint8_t *tx_buf, float out1, float out2, uint8_t status_flags) {
    /* Cast raw byte array to the packed struct format for clean assignment */
    McuToHostPacket *pkt = (McuToHostPacket *)tx_buf;

    /* ---------------------------------------------------------------------------
     * STEP 1: Set Frame Synchronization Magic (0x55, 0xAA)
     * --------------------------------------------------------------------------- */
    pkt->sync[0] = 0x55;
    pkt->sync[1] = 0xAA;

    /* ---------------------------------------------------------------------------
     * STEP 2: Populate Control Effort & Status Fields
     * --------------------------------------------------------------------------- */
    pkt->output1    = out1;
    pkt->output2    = out2;
    pkt->mcu_status = status_flags;

    /* ---------------------------------------------------------------------------
     * STEP 3: Compute XOR Checksum across Bytes 0 through 10
     * --------------------------------------------------------------------------- */
    uint8_t computed_cs = 0;
    for (int i = 0; i < 11; i++) {
        computed_cs ^= tx_buf[i];
    }
    
    /* Store checksum in the final byte (Offset 11) */
    pkt->checksum = computed_cs;
}