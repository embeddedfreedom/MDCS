/*
  compensator_STM32F411.c - Flywheel position LQI, MCU-side compensator
  Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
  Licensed under the MIT License

  Receives ONLY the flywheel angle from the host over USB CDC.
  Derives velocity at 1 kHz and runs the position LQI law from
  compensator/position_control/compensator.c.
  Returns the signed motor voltage.
*/

#include <math.h>
#include "core_initialisation_STM32F411xe.h"

#ifndef PI
#define PI 3.141592653589793f
#endif

#define VMAX       12.0f
#define DEADZONE    0.4f

#define K_POS      11.3836f
#define K_VEL       1.9305f
#define K_INT       7.0711f
#define THETA_REF_RAD (23.0f * (PI / 180.0f))
#define INT_CLAMP   2.0f

static bool  initialized = false;
static float theta_prev = 0.0f;
static float d_filt = 0.0f;
static float integral = 0.0f;

static float wrap_pi(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

float run_compensator_cycle_HIL(float t1, float t2) {
    (void)t2;   /* Only one sensor: the flywheel angle */

    /* 1. Velocity from the angle difference (1 kHz), unwrap ±PI */
    float d_theta = wrap_pi(t1 - theta_prev);
    theta_prev = t1;

    float d_raw = d_theta / 0.001f;
    d_filt = (0.80f * d_filt) + (0.20f * d_raw);

    /* 2. Position LQI */
    float error_pos = wrap_pi(THETA_REF_RAD - t1);
    float error_vel = 0.0f - d_filt;

    integral += error_pos * 0.001f;
    if (integral >  INT_CLAMP) integral =  INT_CLAMP;
    if (integral < -INT_CLAMP) integral = -INT_CLAMP;

    float u = (K_POS * error_pos) + (K_VEL * error_vel) + (K_INT * integral);

    /* Feedforward friction compensation */
    if (fabsf(u) > 0.001f) {
        u += (u > 0.0f) ? DEADZONE : -DEADZONE;
    }

    if (u >  VMAX) u =  VMAX;
    if (u < -VMAX) u = -VMAX;

    TIM3_Set_DutyCycle((fabsf(u) / VMAX) * 999.0f);

    return u;
}
