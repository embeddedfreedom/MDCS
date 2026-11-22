/*
  compensator_STM32F411.c - Pendulum with DC motor, MCU-side compensator
  Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
  Licensed under the MIT License

  Receives ONLY the pendulum angle from the host over USB CDC.
  Derives velocity at 1 kHz, runs the same LQR+I / energy swing-up law as
  the SIL compensator, and returns the signed motor voltage.
*/

#include <math.h>
#include "core_initialisation_STM32F411xe.h"

#ifndef PI
#define PI 3.141592653589793f
#endif

#define VMAX      12.0f
#define DEADZONE   0.4f

/* Balance gains (same as models/pendulum_with_DC_motor/compensator/compensator.c) */
#define K1      -220.0f
#define K2       -26.0f
#define KI       -75.0f

#define CATCH_ZONE       0.50f
#define SLOW_THRESHOLD   3.0f
#define INT_CLAMP        2.0f

static bool  initialized = false;
static float t_prev = 0.0f;
static float d_filt = 0.0f;
static float error_sum = 0.0f;

float run_compensator_cycle_HIL(float t1, float t2) {
    (void)t2;   /* Only one sensor: the pendulum angle */

    /* 1. Velocity from the angle difference (1 kHz) */
    float d_raw = (t1 - t_prev) / 0.001f;
    t_prev = t1;

    /* Low-pass filter (0.8 / 0.2) to kill sensor jitter */
    d_filt = (0.80f * d_filt) + (0.20f * d_raw);

    /* 2. Error relative to PI (upright) */
    float angle_err = t1 - PI;
    while (angle_err >  PI) angle_err -= 2.0f * PI;
    while (angle_err < -PI) angle_err += 2.0f * PI;

    float u = 0.0f;

    /* 3. Control logic */
    bool in_catch_zone = (fabsf(angle_err) < CATCH_ZONE);
    bool is_slow = (fabsf(d_filt) < SLOW_THRESHOLD);

    if (in_catch_zone && is_slow) {
        /* Balance mode: LQR + integral */
        error_sum += angle_err * 0.001f;
        if (error_sum >  INT_CLAMP) error_sum =  INT_CLAMP;
        if (error_sum < -INT_CLAMP) error_sum = -INT_CLAMP;

        u = (K1 * angle_err) + (K2 * d_filt) + (KI * error_sum);

        /* Deadzone compensation */
        if (fabsf(angle_err) > 0.001f) {
            u += (u > 0.0f) ? DEADZONE : -DEADZONE;
        }
    } else {
        /* Swing-up: pump energy only in the bottom half.
           t1 is wrapped to [-PI, PI], so cos(t1) > 0 <=> |t1| < PI/2. */
        error_sum = 0.0f;

        if (fabsf(t1) < (PI * 0.5f)) {
            u = (d_filt >= 0.0f) ? VMAX : -VMAX;
        } else {
            u = 0.0f;
        }

        /* Kickstart if perfectly still at the bottom.
           t1 is already in [-PI, PI], so fmodf(t1, 2*PI) == t1. */
        if (fabsf(t1) < 0.1f && fabsf(d_filt) < 0.1f) {
            u = VMAX;
        }
    }

    /* 4. Output constraints */
    if (u >  VMAX) u =  VMAX;
    if (u < -VMAX) u = -VMAX;

    TIM3_Set_DutyCycle((fabsf(u) / VMAX) * 999.0f);

    return u;
}
