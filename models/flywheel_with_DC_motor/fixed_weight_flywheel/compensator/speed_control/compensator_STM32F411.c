/*
  compensator_STM32F411.c - Flywheel speed PI, MCU-side compensator
  Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
  Licensed under the MIT License

  Receives ONLY the flywheel angle from the host over USB CDC.
  Derives velocity at 1 kHz and runs the speed PI + feedforward law from
  compensator/speed_control/compensator.c.
  Returns the signed motor voltage.
*/

#include <math.h>
#include "core_initialisation_STM32F411xe.h"

#ifndef PI
#define PI 3.141592653589793f
#endif

#define VMAX             12.0f
#define OMEGA_REF_RAD_S  15.0f
#define K2_SPEED          -1.0f
#define KI_SPEED          -0.05f
#define V_FF               2.8250f
#define INT_CLAMP          2.0f

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

    if (!initialized) {
        theta_prev = t1;
        d_filt = 0.0f;
        integral = 0.0f;
        initialized = true;
    }

    /* 1. Velocity from the angle difference (1 kHz), unwrap ±PI */
    float d_theta = wrap_pi(t1 - theta_prev);
    theta_prev = t1;

    float d_raw = d_theta / 0.001f;
    d_filt = (0.80f * d_filt) + (0.20f * d_raw);

    /* 2. Speed PI + feedforward */
    float speed_err = d_filt - OMEGA_REF_RAD_S;

    float u_unsat = (K2_SPEED * speed_err) + (KI_SPEED * integral) + V_FF;

    if (speed_err > 0.0f && u_unsat < VMAX) {
        integral += speed_err * 0.001f;
    } else if (speed_err < 0.0f && u_unsat > -VMAX) {
        integral += speed_err * 0.001f;
    }

    if (integral >  INT_CLAMP) integral =  INT_CLAMP;
    if (integral < -INT_CLAMP) integral = -INT_CLAMP;

    float u = (K2_SPEED * speed_err) + (KI_SPEED * integral) + V_FF;

    if (u >  VMAX) u =  VMAX;
    if (u < -VMAX) u = -VMAX;

    TIM3_Set_DutyCycle((fabsf(u) / VMAX) * 999.0f);

    return u;
}
