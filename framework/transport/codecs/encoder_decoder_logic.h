/*
Encoder-Decoder Logic

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