/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#include "tusb.h"

#define HIL_PACKET_SIZE 12

/* Forward declarations from your existing codebase */
extern bool Extract_HostToMcuPacket(const uint8_t *raw_buf, float *pos1, float *pos2, uint8_t *cmd);
extern void Write_McuToHostPacket(uint8_t *tx_buf, float out1, float out2, uint8_t status_flags);
extern volatile int32_t hil_watchdog_counter;
float run_compensator_cycle_HIL(float t1, float t2);

#ifdef HIL_TWO_OUTPUTS
/* Dual-output models (e.g. furnace: two independent heater PWMs). */
void run_compensator_cycle_HIL_dual(float t1, float t2, float *out1, float *out2);
#endif

/* Frame parser state machine */
typedef enum {
    STATE_HUNT_SYNC0 = 0,
    STATE_HUNT_SYNC1,
    STATE_COLLECT_PAYLOAD
} parser_state_t;

/* Aligned buffers for 32-bit DMA Bus Master compliance */
static uint8_t rx_packet[HIL_PACKET_SIZE] __attribute__((aligned(4)));
static uint8_t tx_packet[HIL_PACKET_SIZE] __attribute__((aligned(4)));
static uint8_t rx_idx = 0;
static parser_state_t parser_state = STATE_HUNT_SYNC0;



//--------------------------------------------------------------------+
// Stack Lifecycle API
//--------------------------------------------------------------------+
void usb_cdc_init(void) {
    tusb_init();
}

void usb_cdc_task(void) {
    tud_task();
}


uint32_t tusb_time_millis_api(void) {
    return 0;
}

//--------------------------------------------------------------------+
// USB Hardware Interrupt (IRQ 67)
//--------------------------------------------------------------------+
void OTG_FS_IRQHandler(void) {
    tud_int_handler(0);
}

//--------------------------------------------------------------------+
// CDC RX Accumulator & Immediate Response Dispatcher
//--------------------------------------------------------------------+
void tud_cdc_rx_cb(uint8_t itf) {
    (void) itf;

    while (tud_cdc_available()) {
        uint8_t byte = (uint8_t) tud_cdc_read_char();

        switch (parser_state) {
            case STATE_HUNT_SYNC0:
                if (byte == 0xAA) {
                    rx_packet[0] = byte;
                    parser_state = STATE_HUNT_SYNC1;
                }
                break;

            case STATE_HUNT_SYNC1:
                if (byte == 0x55) {
                    rx_packet[1] = byte;
                    rx_idx = 2; // Next byte goes to offset 2
                    parser_state = STATE_COLLECT_PAYLOAD;
                } else if (byte == 0xAA) {
                    rx_packet[0] = 0xAA; // Re-align if consecutive 0xAA
                } else {
                    parser_state = STATE_HUNT_SYNC0;
                }
                break;

            case STATE_COLLECT_PAYLOAD:
                rx_packet[rx_idx++] = byte;

                if (rx_idx == HIL_PACKET_SIZE) {
                    // 12 bytes assembled: reset state for next cycle
                    parser_state = STATE_HUNT_SYNC0;

                    float pos1 = 0.0f;
                    float pos2 = 0.0f;
                    uint8_t cmd = 0;

                    /* Validate frame synchronization & XOR checksum */
                    if (Extract_HostToMcuPacket(rx_packet, &pos1, &pos2, &cmd)) {
                        /* 1. Reset 5 ms watchdog */
                        hil_watchdog_counter = 0;

                        /* 2. Execute 1 kHz Control Law / Compensator */
#ifdef HIL_TWO_OUTPUTS
                        float out1 = 0.0f;
                        float out2 = 0.0f;
                        run_compensator_cycle_HIL_dual(pos1, pos2, &out1, &out2);
#else
                        float out1 = run_compensator_cycle_HIL(pos1, pos2); // Compensator math
                        float out2 = pos2;
#endif

                        /* 3. Pack 12-byte response */
                        Write_McuToHostPacket(tx_packet, out1, out2, 0x01);

                        /* 4. Transmit and flush immediately to TxFIFO */
                        tud_cdc_write(tx_packet, HIL_PACKET_SIZE);
                        tud_cdc_write_flush();
                    }
                }
                break;
        }
    }
}