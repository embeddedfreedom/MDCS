/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

#include <math.h>
#include <stdbool.h>
#include "compensator.h"   
#include "pendulum_types.h"
#include "encoder_decoder_logic.h"

// Persistent state variables
static bool initialized = false;
static Decoder dec_pend;
static float t_prev = 0;
static float d_filt = 0;       
static float error_sum = 0.0f; 

float run_compensator_cycle(EncoderBus *bus, PendulumParams *p, int *motor_dir) {
    if (!initialized) {
        decoder_init(&dec_pend);
        initialized = true;
    }

    // 1. Process Raw Encoder Data
    for (int i = 0; i < bus->n_pend; i++) {
        decoder_process(&dec_pend, bus->pend_states[i]);
    }

    // Z-Pulse Sync at 0 rad (Down)
    if (bus->z_pend) {
        dec_pend.count = (int)(roundf((float)dec_pend.count / 4096.0f) * 4096.0f);
    }

    // 2. State Estimation
    const float COUNT_TO_RAD = (2.0f * PI / 4096.0f);
    float t = (float)dec_pend.count * COUNT_TO_RAD;
    // The value of 0.001 here is necessary, for the 20:1 plant where the dt is 50 micro seconds
    // this value must be 20 times that
    float d_raw = (t - t_prev) / 0.001f;
    t_prev = t;

    // Low-Pass Filter (0.8/0.2) to kill sensor jitter
    d_filt = (0.80f * d_filt) + (0.20f * d_raw); 

    // Error relative to PI (Top)
    float angle_err = t - PI;
    while (angle_err >  PI) angle_err -= 2.0f * PI;
    while (angle_err < -PI) angle_err += 2.0f * PI;

    float u = 0;

    // 3. Control Logic
    // Catch Condition: Is it near the top AND moving slow enough to stop?
    bool in_catch_zone = (fabsf(angle_err) < 0.50f);
    bool is_slow = (fabsf(d_filt) < 3.0f);

    if (in_catch_zone && is_slow) { 
        // --- BALANCE MODE (LQR + I) ---
        float k1 = -220.0f;
        float k2 = -26.0f;  
        float ki = -75.0f; // High gain to force 180.000 precision

        error_sum += angle_err * 0.001f;
        if (error_sum > 2.0f) error_sum = 2.0f; // Anti-windup
        if (error_sum < -2.0f) error_sum = -2.0f;

        u = (k1 * angle_err) + (k2 * d_filt) + (ki * error_sum);

        // Deadzone Compensation
        if (fabsf(angle_err) > 0.001f) {
            u += (u > 0) ? p->deadzone : -p->deadzone;
        }
    } 
    else {
        // --- ROBUST SWING-UP (Energy Pump) ---
        error_sum = 0; // Clear integrator memory

        // Calculate if we are in the bottom half (cos > 0) or top half (cos < 0)
        // This prevents the "propeller" by only pushing at the bottom.
        if (cosf(t) > 0) {
            // Push in the direction of motion to add energy
            u = (d_filt >= 0) ? p->Vmax : -p->Vmax;
        } else {
            // Coast in the top half to allow LQR to "catch" it cleanly
            u = 0; 
        }

        // Kickstart if perfectly still at the bottom
        if (fabsf(fmodf(t, 2*PI)) < 0.1f && fabsf(d_filt) < 0.1f) {
            u = p->Vmax;
        }
    }

    // 4. Output Constraints
    if (u > p->Vmax) u = p->Vmax;
    if (u < -p->Vmax) u = -p->Vmax;

    *motor_dir = (u >= 0) ? 1 : -1;
    return fabsf(u) / p->Vmax;
}
