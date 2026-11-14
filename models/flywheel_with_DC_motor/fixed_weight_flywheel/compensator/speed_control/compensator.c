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
#include <stdbool.h>
#include "compensator.h"
#include "flywheel_types.h"
#include "encoder_decoder_logic.h"

#define OMEGA_REF_RAD_S     15.0f
#define K2_SPEED  -1.0f
#define KI_SPEED  -0.05f
#define V_FF      4.7000f

static bool initialized = false;
static Decoder20 dec_flywheel;
static int prev_count = 0;
static float d_filt = 0.0f;
static float speed_error_sum = 0.0f;

float run_compensator_cycle(EncoderBus *bus, FlywheelParams *p, int *motor_dir) {
    if (!initialized) {
        decoder20_init(&dec_flywheel);
        prev_count = 0;
        initialized = true;
    }

    /* Decode the 20-bit quadrature stream produced by the local encoder. */
    for (int i = 0; i < bus->n_pend; i++) {
        decoder20_process(&dec_flywheel, bus->pend_states[i]);
    }

    /* The decoder count wraps at +/-ENCODER20_DECODER_WRAP. Unwrap the
     * 1 kHz delta so a wrap is not seen as a huge reverse movement. */
    int raw_delta = dec_flywheel.count - prev_count;
    prev_count = dec_flywheel.count;

    int delta_count;
    if (raw_delta > ENCODER20_DECODER_WRAP - 1) {
        delta_count = raw_delta - ENCODER20_COUNTS_PER_REV;
    } else if (raw_delta < -ENCODER20_DECODER_WRAP) {
        delta_count = raw_delta + ENCODER20_COUNTS_PER_REV;
    } else {
        delta_count = raw_delta;
    }

    const float COUNT_TO_RAD = (2.0f * PI / (float)ENCODER20_COUNTS_PER_REV);
    float d_raw = (float)delta_count * COUNT_TO_RAD / 0.001f;
    d_filt = (0.80f * d_filt) + (0.20f * d_raw);

    float speed_err = d_filt - OMEGA_REF_RAD_S;

    float u_unsat = (K2_SPEED * speed_err) + (KI_SPEED * speed_error_sum) + V_FF;

    if (speed_err > 0.0f && u_unsat < p->Vmax) {
        speed_error_sum += speed_err * 0.001f;
    } else if (speed_err < 0.0f && u_unsat > -p->Vmax) {
        speed_error_sum += speed_err * 0.001f;
    }

    if (speed_error_sum >  2.0f) speed_error_sum =  2.0f;
    if (speed_error_sum < -2.0f) speed_error_sum = -2.0f;

    float u = (K2_SPEED * speed_err) + (KI_SPEED * speed_error_sum) + V_FF;

    if (u > p->Vmax)  u = p->Vmax;
    if (u < -p->Vmax) u = -p->Vmax;

    *motor_dir = (u >= 0) ? 1 : -1;
    return fabsf(u) / p->Vmax;
}