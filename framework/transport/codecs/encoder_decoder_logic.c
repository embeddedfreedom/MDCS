/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

/**
 * encoder_decoder_logic.c
 * -----------------------
 * Simulated quadrature encoder and decoder.
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>   // int8_t, uint8_t
#include <stdlib.h>
#include "encoder_decoder_logic.h"



/* Quadrature states mapping: Phase (0,1,2,3) to Gray (0,1,3,2) */
static const int phase_to_gray[] = {0, 1, 3, 2};

static const int8_t quad_table[4][4] = {
    /* to:  00  01  10  11 */
    /* 0   1   2   3  */
    /* 0 */ {  0, +1, -1,  0 },
    /* 1 */ { -1,  0,  0, +1 },
    /* 2 */ { +1,  0,  0, -1 },
    /* 3 */ {  0, -1, +1,  0 }
};

void encoder_process_angle(Encoder *e, float angle, int *out_states, int *num_states)
{
    // 1. Calculate the delta
    float delta = angle - e->prev_angle;
    e->prev_angle = angle;

    // 2. Accumulate the change
    e->remainder += delta;

    // 3. Determine pulses
    // We use a temporary integer to avoid multiple divisions
    int pulses = (int)(e->remainder / DEG_PER_COUNT);

    // 4. THE PRECISION FIX:
    // Update remainder by exactly what was converted to pulses.
    // This preserves the microscopic fractional degrees.
    e->remainder = e->remainder - ((float)pulses * DEG_PER_COUNT);

    *num_states = 0;
    
    // 5. THE DIRECTION FIX:
    // If pulses is 0, we don't move. If not 0, we check the sign.
    if (pulses == 0) return; 
    
    int direction = (pulses > 0) ? 1 : -1;
    int abs_pulses = abs(pulses);
    
    for (int i = 0; i < abs_pulses; i++) {
        if (*num_states >= MAX_ALLOWED_STATES) break; 

        if (direction > 0) {
            e->quad_state = (e->quad_state + 1) & 0x3;
        } else {
            e->quad_state = (e->quad_state + 3) & 0x3;
        }
        
        out_states[(*num_states)++] = phase_to_gray[e->quad_state];
    }
}

void decoder_process(Decoder *d, int new_state)
{
    int delta = quad_table[d->prev_state][new_state];
    d->count += delta;
    d->prev_state = new_state;

    // ROBUST NORMALIZATION: Keep count between -2048 and 2047
    // This matches the physics normalize_angle behavior.
    // 4096 counts = 360 degrees.
    while (d->count > 2047) {
        d->count -= 4096;
    }
    while (d->count < -2048) {
        d->count += 4096;
    }
}

void encoder_init(Encoder *e) {
    e->prev_angle = 0.0f;
    e->remainder  = 0.0f;
    e->quad_state = 0; 
}

void decoder_init(Decoder *d) {
    d->prev_state = 0;
    d->count = 0;
}




//DO NOT UNCOMMENT FOR THE PROJECT. THIS IS LEFT BEHIND FOR THE STUDENT TO UNDERSTAND THE INNER 
//WORKINGS OF THE ENCODER/DECODER
//
// int main() {
//     Encoder test_encoder;
//     Decoder test_decoder;
    
//     encoder_init(&test_encoder);
//     decoder_init(&test_decoder);

//     int state_array[MAX_ALLOWED_STATES];
//     int array_index = 0;

//     // --- TEST 1: Move CW to 5 degrees ---
//     float angle1 = 5.0f;
//     encoder_process_angle(&test_encoder, angle1, state_array, &array_index);
//     for(int i=0; i < array_index; i++) decoder_process(&test_decoder, state_array[i]);

//     printf("After CW Move: %f degrees\n", (float)test_decoder.count * DEG_PER_COUNT);

//     // --- TEST 2: Move CCW back to -5 degrees ---
//     // Total change is -10 degrees from current position
//     float angle2 = -5.0f;
//     encoder_process_angle(&test_encoder, angle2, state_array, &array_index);

//     printf("CCW Pulses Generated: %d\n", array_index);
//     printf("CCW Sequence: ");
    
//     for(int i = 0; i < array_index; i++) {
//         printf("%d ", state_array[i]);
//         decoder_process(&test_decoder, state_array[i]);
//     }

//     float final_angle = (float)test_decoder.count * DEG_PER_COUNT;
//     printf("\nFinal Decoded Position: %f degrees\n", final_angle);

//     return 0;
// }

