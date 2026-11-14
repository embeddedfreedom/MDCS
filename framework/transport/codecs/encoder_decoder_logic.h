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

#ifndef ENCODER_DECODER_LOGIC_H
#define ENCODER_DECODER_LOGIC_H

#include <stdint.h>
#include <unistd.h>

/* =========================
   Encoder (encoder only)
   ========================= */
#define MAX_ENCODER_PULSES 2048
#define MAX_PWM_CHANNELS 4
#define DEG_PER_COUNT 0.087890625f
#define MAX_ALLOWED_STATES 2048

/* ===============================
   Encoder state
   =============================== */
typedef struct {
    float prev_angle;
    float remainder;
    int   quad_state;
} Encoder;

/* ===============================
   Decoder state
   =============================== */
typedef struct {
    int prev_state;
    int count;
} Decoder;
                        

typedef struct {
    int8_t  pulses[MAX_ENCODER_PULSES];
    uint8_t count;
    uint8_t direction;
    float   voltage;
    float   current;
} EncoderMessage;


typedef struct {
    float pwm[MAX_PWM_CHANNELS];
    uint8_t count;
} ControllerCommand;       

void encoder_init(Encoder *e);
void decoder_init(Decoder *d);
void encoder_process_angle(Encoder *e,
                           float delta_angle,
                           int *out_states,
                           int *num_states);

void decoder_process(Decoder *d, int new_state);

/* =========================
   High-resolution encoder (20-bit)
   ========================= */
#define ENCODER20_COUNTS_PER_REV 1048576
#define ENCODER20_DEG_PER_COUNT (360.0 / (double)ENCODER20_COUNTS_PER_REV)
#define ENCODER20_DECODER_WRAP (ENCODER20_COUNTS_PER_REV / 2)
#define ENCODER20_MAX_STATES_PER_CALL 4096

typedef struct {
    double prev_angle;
    double remainder;
    int    quad_state;
} Encoder20;

typedef struct {
    int prev_state;
    int count;
} Decoder20;

void encoder20_init(Encoder20 *e);
void decoder20_init(Decoder20 *d);
void encoder20_process_angle(Encoder20 *e,
                             double angle,
                             int *out_states,
                             int *num_states);
void decoder20_process(Decoder20 *d, int new_state);

#endif 