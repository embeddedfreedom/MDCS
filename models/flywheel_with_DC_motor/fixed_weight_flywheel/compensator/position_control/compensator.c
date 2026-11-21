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

#ifndef PI
#define PI 3.141592653589793f
#endif

// LQR Gains computed from sim-matched parameters (M_disc=1.3872, R_disc=0.075,
// shaft_m=0.06, shaft_r=0.005, b1=0.002, kt=kb=0.12, R=2.5, I_tot=0.0039022)
// u = K_POS*error_pos + K_VEL*error_vel + K_INT*pos_integral  (all restoring, positive)
#define K_POS  11.3836f  // LQR position gain
#define K_VEL  1.9305f   // LQR velocity gain
#define K_INT  7.0711f   // LQR integral gain

// The simulator flags a +/-0.1 deg index window (~291 counts at 20-bit), but a
// real Z pulse occurs only at the revolution boundary. Snap only when the
// decoder is genuinely within this many counts of a boundary; otherwise the
// coarse window would erase up to 0.1 deg of real motion (a standing offset).
#define Z_SNAP_TOL_COUNTS 64

// Persistent tracking states
static bool initialized = false;
static Decoder20 dec_flywheel;
static float t_prev = 0.0f;
static float d_filt = 0.0f;       
static float pos_integral = 0.0f; 

float run_compensator_cycle(EncoderBus *bus, FlywheelParams *p, int *motor_dir) {
    if (!initialized) {
        decoder20_init(&dec_flywheel);
        initialized = true;
    }

    // PHASE A: Process accumulated 20kHz encoder pulse frames (1ms window)
    for (int i = 0; i < bus->n_pend; i++) {
        decoder20_process(&dec_flywheel, bus->pend_states[i]);
    }

    // Absolute Z-Pulse Calibration Alignment. Only accept the coarse index
    // window when the count is actually near a revolution boundary.
    if (bus->z_pend) {
        int nearest = (int)(roundf((float)dec_flywheel.count /
                                   (float)ENCODER20_COUNTS_PER_REV) *
                            (float)ENCODER20_COUNTS_PER_REV);
        int delta = dec_flywheel.count - nearest;
        if (delta < 0) delta = -delta;
        if (delta <= Z_SNAP_TOL_COUNTS) {
            dec_flywheel.count = nearest;
            // Wrap snapped value into the valid 20-bit decoder range (±180°).
            while (dec_flywheel.count >  ENCODER20_DECODER_WRAP - 1)
                dec_flywheel.count -= ENCODER20_COUNTS_PER_REV;
            while (dec_flywheel.count < -ENCODER20_DECODER_WRAP)
                dec_flywheel.count += ENCODER20_COUNTS_PER_REV;
        }
    }

    // PHASE B: Scale positional counts to physical radians
    const float COUNT_TO_RAD = (2.0f * PI / (float)ENCODER20_COUNTS_PER_REV);
    float t = (float)dec_flywheel.count * COUNT_TO_RAD;

    // PHASE C: Finite difference derivative for raw velocity tracking
    float d_raw = (t - t_prev) / 0.001f; // 1kHz control downsample window
    t_prev = t;

    // First-Order Low-Pass filter to erase sensor quantization jitter
    d_filt = (0.80f * d_filt) + (0.20f * d_raw); 

    // PHASE D: Error Tracking & Target Configuration
    // Example: Command a 90-degree step displacement target (Edit as needed)
    float theta_ref = 23.0f * (PI / 180.0f);
    float error_pos = theta_ref - t;
    // Wrap position error to shortest angular path [-PI, PI] so the controller
    // tracks the target across revolutions instead of unwinding the wrong way.
    while (error_pos >  PI) error_pos -= 2.0f * PI;
    while (error_pos < -PI) error_pos += 2.0f * PI;
    float error_vel = 0.0f - d_filt; // We want target speed at destination to be 0

    // Update position loop error integration accumulator
    pos_integral += error_pos * 0.001f;

    // Anti-Windup Clamping to prevent actuator saturation memory runaway
    if (pos_integral >  2.0f) pos_integral =  2.0f;
    if (pos_integral < -2.0f) pos_integral = -2.0f;

    // PHASE E: Execute State-Feedback Control Law
    float u = (K_POS * error_pos) + (K_VEL * error_vel) + (K_INT * pos_integral);

    // PHASE F: Feedforward Friction Compensation (Overcoming the 0.4V Stiction Barrier)
    if (fabsf(u) > 0.001f) {
        u += (u > 0.0f) ? p->deadzone : -p->deadzone;
    }

    // PHASE G: Actuator Mapping and Direction Inversion Checks
    if (u >= 0.0f) {
        *motor_dir = 1;  // Clockwise rotation
    } else {
        *motor_dir = -1; // Counter-Clockwise rotation
        u = -u;          // Strip sign for standard magnitude PWM conversion
    }

    // PHASE H: Enforce absolute power voltage rails
    if (u > p->Vmax) {
        u = p->Vmax;
    }
    if (u < -p->Vmax) {
        u = -p->Vmax;
    }

    // Normalize absolute output voltage to duty cycle ratio [0.0, 1.0]
    return u / p->Vmax;
}
