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
#include "core_initialisation_STM32F411xe.h"

/* ----------------------------------------------------------------
 * Plant parameters (industrial set, matches furnace_sim.c)
 * ---------------------------------------------------------------- */
#define PLANT_C1      500.0f
#define PLANT_C2      450.0f
#define PLANT_K12       2.5f
#define PLANT_HA1_A1    0.5f
#define PLANT_HA2_A2    0.4f
#define PLANT_TAMB     25.0f
#define PLANT_VMAX     12.0f
#define PLANT_R         4.0f

/* Setpoints (degC) */
#define T1_SETPOINT    75.0f
#define T2_SETPOINT    75.0f

/* Industrial LQI gains (furnace_lqi_design.py, PWM input) */
#define KX_00  20.766772f
#define KX_01   0.044152f
#define KX_10   0.043879f
#define KX_11  20.767721f
#define KI_00   2.000000f
#define KI_01   0.000249f
#define KI_10  -0.000249f
#define KI_11   2.000000f

#define PWM_MIN    0.0f
#define PWM_MAX    1.0f
#define INT_CLAMP 200.0f

static bool  initialized = false;
static float q1 = 0.0f, q2 = 0.0f;
static float pwm_sp1 = 0.0f, pwm_sp2 = 0.0f;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Bare-metal sqrt (Newton): avoids pulling libm/newlib syscalls into the
   firmware. Only used once at init for the feedforward PWM. */
static float sqrt_pos(float x) {
    if (x <= 0.0f) return 0.0f;
    float r = x;
    for (int i = 0; i < 12; i++) {
        r = 0.5f * (r + x / r);
    }
    return r;
}

void run_compensator_cycle_HIL_dual(float in1, float in2, float *out1, float *out2) {
    if (out1 == 0 || out2 == 0) {
        return;
    }

    if (!initialized) {
        q1 = 0.0f;
        q2 = 0.0f;

        /* Feedforward steady-state PWM to hold the setpoints */
        float p_sp1 = (PLANT_K12 + PLANT_HA1_A1) * T1_SETPOINT
                    - PLANT_K12 * T2_SETPOINT
                    - PLANT_HA1_A1 * PLANT_TAMB;
        float p_sp2 = (PLANT_K12 + PLANT_HA2_A2) * T2_SETPOINT
                    - PLANT_K12 * T1_SETPOINT
                    - PLANT_HA2_A2 * PLANT_TAMB;

        pwm_sp1 = (p_sp1 > 0.0f) ? sqrt_pos(p_sp1 * PLANT_R) / PLANT_VMAX : 0.0f;
        pwm_sp2 = (p_sp2 > 0.0f) ? sqrt_pos(p_sp2 * PLANT_R) / PLANT_VMAX : 0.0f;

        pwm_sp1 = clampf(pwm_sp1, PWM_MIN, PWM_MAX);
        pwm_sp2 = clampf(pwm_sp2, PWM_MIN, PWM_MAX);
        initialized = true;
    }

    /* Tracking error */
    float e1 = in1 - T1_SETPOINT;
    float e2 = in2 - T2_SETPOINT;

    /* Integrate with clamping */
    float q1_next = clampf(q1 + e1 * 0.001f, -INT_CLAMP, INT_CLAMP);
    float q2_next = clampf(q2 + e2 * 0.001f, -INT_CLAMP, INT_CLAMP);

    /* pwm = pwm_sp - Kx*e - Ki*q */
    float pwm1 = pwm_sp1
               - (KX_00 * e1 + KX_01 * e2)
               - (KI_00 * q1_next + KI_01 * q2_next);
    float pwm2 = pwm_sp2
               - (KX_10 * e1 + KX_11 * e2)
               - (KI_10 * q1_next + KI_11 * q2_next);

    pwm1 = clampf(pwm1, PWM_MIN, PWM_MAX);
    pwm2 = clampf(pwm2, PWM_MIN, PWM_MAX);

    /* Anti-windup: only commit the integral when not saturated */
    if (!(pwm1 <= PWM_MIN || pwm1 >= PWM_MAX)) q1 = q1_next;
    if (!(pwm2 <= PWM_MIN || pwm2 >= PWM_MAX)) q2 = q2_next;

    /* Drive hardware PWM timer outputs (TIM3 CH1 = PA6, TIM3 CH2 = PA7) */
    TIM3_Set_DutyCycle((uint32_t)(pwm1 * 999.0f));
    TIM3_Set_DutyCycle2((uint32_t)(pwm2 * 999.0f));

    *out1 = pwm1;
    *out2 = pwm2;
}
