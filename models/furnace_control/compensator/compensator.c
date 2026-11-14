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
#include <stdio.h>
#include <stdbool.h>
#include "compensator.h"
#include "furnace_types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ----------------------------------------------------------------
 * Tuning — edit these to retune.
 * ---------------------------------------------------------------- */

/* Setpoints (°C) */
#define T1_SETPOINT   75.0f
#define T2_SETPOINT   75.0f

/* ---- INDUSTRIAL gains (C1=500, C2=450, K12=2.5, ha1=0.5, ha2=0.4) ---- */
/* Active when BENCH_TEST_ACCELERATED == 0 */
/* Computed via furnace_lqi_design.py with PWM input, linearized at
   operating point (Vmax=12V, R=4 ohm, pwm_sp=[0.8333, 0.7454]). */
#define IND_KX_00 20.766772f
#define IND_KX_01  0.044152f
#define IND_KX_10  0.043879f
#define IND_KX_11 20.767721f
#define IND_KI_00  2.000000f
#define IND_KI_01  0.000249f
#define IND_KI_10 -0.000249f
#define IND_KI_11  2.000000f

/* ---- BENCH gains (C1=5, C2=4.5) — commented out, enable if needed ----
#define BEN_KX_00  3.6f
#define BEN_KX_01  0.18f
#define BEN_KX_10  0.20f
#define BEN_KX_11  3.5f
#define BEN_KI_00  0.04f
#define BEN_KI_01 -0.00f
#define BEN_KI_10  0.00f
#define BEN_KI_11  0.04f
---- */

/* Actuator limits (PWM duty cycle: 0 = off, 1 = full on) */
#define PWM_MIN   0.0f
#define PWM_MAX   1.0f

/* Anti-windup: clamp integral states to this magnitude (°C·s) */
#define INT_CLAMP 200.0f

static bool initialized = false;
static float q1 = 0.0f, q2 = 0.0f;   /* integral error states */
static float pwm_sp1 = 0.0f, pwm_sp2 = 0.0f;

/* Select gain set based on thermal capacitance (industrial vs bench). */
static void select_gains(const ThermalParams *p,
                         float *kx00, float *kx01,
                         float *kx10, float *kx11,
                         float *ki00, float *ki01,
                         float *ki10, float *ki11) {
    if (p->C1 >= 100.0f) {
        *kx00 = IND_KX_00; *kx01 = IND_KX_01;
        *kx10 = IND_KX_10; *kx11 = IND_KX_11;
        *ki00 = IND_KI_00; *ki01 = IND_KI_01;
        *ki10 = IND_KI_10; *ki11 = IND_KI_11;
    } else {
        /* Fallback: use industrial gains scaled down for bench. */
        *kx00 = IND_KX_00 * 0.2f; *kx01 = IND_KX_01 * 0.2f;
        *kx10 = IND_KX_10 * 0.2f; *kx11 = IND_KX_11 * 0.2f;
        *ki00 = IND_KI_00; *ki01 = IND_KI_01;
        *ki10 = IND_KI_10; *ki11 = IND_KI_11;
    }
}

static void clampf(float *v, float lo, float hi) {
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}

/*
 * 1 kHz LQI thermal compensator for the coupled heat slab.
 *
 * measured_T1, measured_T2 : latest sensor readings (°C), already jittered
 * p                          : plant parameters (ThermalParams)
 * pwm1, pwm2                 : output PWM duty cycle commands [0, 1]
 *
 * Called once per 1 ms cycle from furnace_sim.c.
 */
void run_thermal_compensator_cycle(float measured_T1, float measured_T2,
                                   const ThermalParams *p,
                                   float *pwm1, float *pwm2) {
    if (p == NULL || pwm1 == NULL || pwm2 == NULL) {
        *pwm1 = 0.0f;
        *pwm2 = 0.0f;
        return;
    }

    float kx00, kx01, kx10, kx11;
    float ki00, ki01, ki10, ki11;
    select_gains(p, &kx00, &kx01, &kx10, &kx11, &ki00, &ki01, &ki10, &ki11);

    if (!initialized) {
        q1 = 0.0f;
        q2 = 0.0f;

        /* Feedforward steady-state PWM to hold setpoint against ambient losses.
           Steady-state power needed: P = (K12+ha)*Tsp - K12*T_other - ha*Tamb
           Convert to PWM: pwm = sqrt(P * R) / Vmax */
        float p_sp1 = (p->K12 + p->ha1_A1) * T1_SETPOINT
                    - p->K12 * T2_SETPOINT
                    - p->ha1_A1 * p->Tamb;
        float p_sp2 = (p->K12 + p->ha2_A2) * T2_SETPOINT
                    - p->K12 * T1_SETPOINT
                    - p->ha2_A2 * p->Tamb;

        if (p_sp1 > 0.0f)
            pwm_sp1 = sqrtf(p_sp1 * p->R) / p->Vmax;
        else
            pwm_sp1 = 0.0f;

        if (p_sp2 > 0.0f)
            pwm_sp2 = sqrtf(p_sp2 * p->R) / p->Vmax;
        else
            pwm_sp2 = 0.0f;

        clampf(&pwm_sp1, PWM_MIN, PWM_MAX);
        clampf(&pwm_sp2, PWM_MIN, PWM_MAX);
        initialized = true;
    }

    /* Tracking error */
    float e1 = measured_T1 - T1_SETPOINT;
    float e2 = measured_T2 - T2_SETPOINT;

    /* Integrate error with anti-windup: pause integration when saturated */
    float q1_next = q1 + e1 * 0.001f;
    float q2_next = q2 + e2 * 0.001f;
    clampf(&q1_next, -INT_CLAMP, INT_CLAMP);
    clampf(&q2_next, -INT_CLAMP, INT_CLAMP);

    /* Tentative control: pwm = pwm_sp - Kx*e - Ki*q */
    float out1 = pwm_sp1
               - (kx00 * e1 + kx01 * e2)
               - (ki00 * q1_next + ki01 * q2_next);
    float out2 = pwm_sp2
               - (kx10 * e1 + kx11 * e2)
               - (ki10 * q1_next + ki11 * q2_next);

    /* Output saturation (PWM duty cycle) */
    clampf(&out1, PWM_MIN, PWM_MAX);
    clampf(&out2, PWM_MIN, PWM_MAX);

    /* Anti-windup: only commit integral update if we did not saturate */
    bool sat1 = (out1 <= PWM_MIN || out1 >= PWM_MAX);
    bool sat2 = (out2 <= PWM_MIN || out2 >= PWM_MAX);
    if (!sat1) q1 = q1_next;
    if (!sat2) q2 = q2_next;

    *pwm1 = out1;
    *pwm2 = out2;
}
