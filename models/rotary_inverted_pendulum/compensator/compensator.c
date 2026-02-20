/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "compensator.h"   
#include "furuta_types.h"
#include "encoder_decoder_logic.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static bool initialized = false;
static Decoder dec_arm, dec_pend;
static float t1_prev = 0, t2_prev = 0;

static void wrap_angle(float *a) {
    while (*a >  M_PI) *a -= 2.0f * M_PI;
    while (*a < -M_PI) *a += 2.0f * M_PI;
}

float run_compensator_cycle(EncoderBus *bus, FurutaParams *p, int *motor_dir) {
    if (!initialized) {
        decoder_init(&dec_arm); 
        decoder_init(&dec_pend);
        initialized = true;
    }

    // 1. Process 20kHz samples to reach current 1kHz state
    for (int i = 0; i < bus->n_arm; i++) decoder_process(&dec_arm, bus->arm_states[i]);
    for (int i = 0; i < bus->n_pend; i++) decoder_process(&dec_pend, bus->pend_states[i]);

    // --- Z-PULSE DRIFT COMPENSATION ---
    // If the sensor detected the physical zero-point in the last 1ms:
    if (bus->z_arm) {
        // Snap the arm count to the nearest full rotation
        dec_arm.count = (int)(roundf((float)dec_arm.count / 4096.0f) * 4096.0f);
    }
    if (bus->z_pend) {
        // Snap the pendulum count to the nearest full rotation
        // This effectively kills the drift from NOISE_MISSING_PULSES
        dec_pend.count = (int)(roundf((float)dec_pend.count / 4096.0f) * 4096.0f);
    }

    const float COUNT_TO_RAD = (2.0f * M_PI / 4096.0f);
    float t1 = (float)dec_arm.count * COUNT_TO_RAD;
    float t2 = (float)dec_pend.count * COUNT_TO_RAD;
    
    // 2. Raw Velocity (No filtering to avoid Phase Lag)
    float d1 = (t1 - t1_prev) / 0.001f;
    float d2 = (t2 - t2_prev) / 0.001f;
    t1_prev = t1; 
    t2_prev = t2;

    float angle_err = t2 - M_PI;
    wrap_angle(&angle_err);

    float u = 0;

    // 3. CONTROL LOGIC
    if (fabsf(angle_err) < 0.45f) { // Slightly tighter catch window for lighter mass
        // --- STABILIZE (Gains scaled for 50g) ---
        // We reduce k1 and k2 because 50g reacts much faster to torque
        // float k1 = -45.01f; // Was -45.0f
        // float k2 = -9.23f;  // Was -9.0f
        // float k3 = 8.18f;   // Was 8.0f
        // float k4 = 3.09f;   // Was 3.0f
        float k1 = -101.01f; 
        float k2 = -12.41f;  
        float k3 = 10.00f;  
        float k4 = 7.33f;  

        u = (k1 * angle_err) + (k2 * d2) + (k3 * t1) + (k4 * d1);
        
        // Soften the center to prevent high-frequency noise vibration
        if (fabsf(angle_err) < 0.03f) {
            u *= 0.7f; 
        }
    } 
    else {
        // --- SWING-UP (The "Whip" Logic) ---
        // For 50g, we use slightly less voltage so the arm doesn't outrun the pendulum
        float swing_volts = p->Vmax * 0.85f; 

        if (fabsf(d2) < 0.5f) {
            u = (t1 > 0) ? -swing_volts : swing_volts; 
        } else {
            float direction = (d2 > 0) ? -1.0f : 1.0f;
            u = direction * swing_volts; 
        }
        // Centering force to keep the arm from wandering
        u -= 12.0f * t1; 
    }

    // 4. Output Processing
    if (u > p->Vmax) u = p->Vmax;
    if (u < -p->Vmax) u = -p->Vmax;

    *motor_dir = (u >= 0) ? 1 : -1;
    return fabsf(u) / p->Vmax;
}



