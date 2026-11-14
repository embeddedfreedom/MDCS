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

#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h> 
#include "compensator.h"
#include "get_keypress.h"
#include "furnace_types.h"

// --- NOISE TOGGLES (1=ON, 0=OFF) ---
#define NOISE_SENSOR_JITTER          1  // Thermal sensor measurement noise
#define JITTER_STRENGTH              0.05f // Degrees Celsius Gaussian-like noise

// --- TIME-SCALE TOGGLE (0 = Real Industrial Slow, 1 = Rapid Bench Test) ---
// Set to 1 if you want setpoints to settle in ~8 seconds instead of 15 minutes!
#define BENCH_TEST_ACCELERATED       0

// Global flag used by compensator to disconnect/suspend the simulation
bool runloop;

/**
 * @brief Computes the state derivatives for the Coupled Heat Slab.
 * Governing Equations:
 *   C1 * dT1/dt = u1 - K12*(T1 - T2) - ha1_A1*(T1 - Tamb)
 *   C2 * dT2/dt = u2 + K12*(T1 - T2) - ha2_A2*(T2 - Tamb)
 */
void compute_derivatives(const ThermalState *x, float pwm1, float pwm2, const ThermalParams *p, ThermalState *dx) {
    float t1 = x->T1;
    float t2 = x->T2;

    // --- HEATER MODEL: PWM duty cycle -> voltage -> power ---
    // P = V^2 / R, where V = Vmax * pwm (clamped to [0, Vmax])
    float v1 = p->Vmax * pwm1;
    float v2 = p->Vmax * pwm2;
    float p1 = (v1 * v1) / p->R;   // Heater 1 electrical power (W)
    float p2 = (v2 * v2) / p->R;   // Heater 2 electrical power (W)

    // Heat flux from node 1 to node 2
    float q12 = p->K12 * (t1 - t2);

    // Dynamic temperature derivatives
    float dT1_dt = (p1 - q12 - p->ha1_A1 * (t1 - p->Tamb)) / p->C1;
    float dT2_dt = (p2 + q12 - p->ha2_A2 * (t2 - p->Tamb)) / p->C2;

    dx->T1 = t1;
    dx->T2 = t2;
    dx->T1_dot = dT1_dt;
    dx->T2_dot = dT2_dt;
    dx->u1 = pwm1;   // Store PWM command for telemetry
    dx->u2 = pwm2;
    dx->q12 = q12;
}

/**
 * @brief 4th-Order Runge-Kutta numerical integrator
 */
void rk4_step(ThermalState *x, float u1, float u2, float dt, const ThermalParams *p) {
    ThermalState k1, k2, k3, k4, xt;
    
    // Step 1: Evaluate at start of interval
    compute_derivatives(x, u1, u2, p, &k1);
    
    // Step 2: Evaluate at midpoint using k1
    xt.T1 = x->T1 + 0.5f * dt * k1.T1_dot; 
    xt.T2 = x->T2 + 0.5f * dt * k1.T2_dot;
    compute_derivatives(&xt, u1, u2, p, &k2);
    
    // Step 3: Evaluate at midpoint using k2
    xt.T1 = x->T1 + 0.5f * dt * k2.T1_dot; 
    xt.T2 = x->T2 + 0.5f * dt * k2.T2_dot;
    compute_derivatives(&xt, u1, u2, p, &k3);
    
    // Step 4: Evaluate at end of interval using k3
    xt.T1 = x->T1 + dt * k3.T1_dot; 
    xt.T2 = x->T2 + dt * k3.T2_dot;
    compute_derivatives(&xt, u1, u2, p, &k4);
    
    // Weighted accumulation of states
    x->T1 += (dt / 6.0f) * (k1.T1_dot + 2.0f * k2.T1_dot + 2.0f * k3.T1_dot + k4.T1_dot);
    x->T2 += (dt / 6.0f) * (k1.T2_dot + 2.0f * k2.T2_dot + 2.0f * k3.T2_dot + k4.T2_dot);
    
    // Update auxiliary telemetry & derivatives
    x->T1_dot = k4.T1_dot;
    x->T2_dot = k4.T2_dot;
    x->u1 = k4.u1;
    x->u2 = k4.u2;
    x->q12 = k4.q12;
}

int main() {
    char key;
    runloop = false;
    
#if BENCH_TEST_ACCELERATED
    // Accelerated parameters: tau ~ 1.6s (Settles in ~7-8s for rapid desk testing)
    ThermalParams p = {
        .C1 = 5.0f,
        .C2 = 4.5f,
        .K12 = 2.5f,
        .ha1_A1 = 0.5f,
        .ha2_A2 = 0.4f,
        .Tamb = 25.0f,
        .Vmax = 12.0f,   // Max heater voltage (V)
        .R = 4.0f        // Heater resistance (ohms)
    };
#else
    // Exact physical parameters from derivation: tau ~ 167s (Industrial time constant)
    ThermalParams p = {
        .C1 = 500.0f,
        .C2 = 450.0f,
        .K12 = 2.5f,
        .ha1_A1 = 0.5f,
        .ha2_A2 = 0.4f,
        .Tamb = 25.0f,
        .Vmax = 12.0f,   // Max heater voltage (V)
        .R = 4.0f        // Heater resistance (ohms)
    };
#endif
    
    ThermalState s = {
        .T1 = 25.0f,
        .T2 = 25.0f,
        .T1_dot = 0.0f,
        .T2_dot = 0.0f,
        .u1 = 0.0f,
        .u2 = 0.0f,
        .q12 = 0.0f
    };
    
    float dt = 0.00005f; // 20kHz Physics simulation step (50 microseconds)
    int sim_3d_in = open("/home/dipto/sim_3d_in", O_WRONLY | O_NONBLOCK);
    int sim_graph_in = open("/home/dipto/sim_graph_in", O_WRONLY | O_NONBLOCK);
    
    int i = 0;
    float pwm1 = 0.0f;
    float pwm2 = 0.0f;

    // --- TIMING SETUP START ---
    struct timespec next_step, last_comp_time, current_time;
    const long NS_PER_STEP = 50000; // 50 microseconds
    clock_gettime(CLOCK_MONOTONIC, &next_step);
    last_comp_time = next_step; 
    // --- TIMING SETUP END ---

    while(1) {
        key = get_keypress();

        if (key == 's' || key == 'S') {
            runloop = true;
            printf("\n>> COMPENSATOR CONNECTED\n");
        } 
        else if (key == 't' || key == 'T') {
            runloop = false;
            printf("\n>> COMPENSATOR DISCONNECTED\n");
        } 
        else if (key == 'q' || key == 'Q') {
            printf("\nExiting...\n");
            break;
        }

        next_step.tv_nsec += NS_PER_STEP;
        if (next_step.tv_nsec >= 1000000000L) {
            next_step.tv_nsec -= 1000000000L;
            next_step.tv_sec++;
        }
        
        // ==================================================
        // 1. HIGH FREQUENCY: PLANT PHYSICS (20kHz)
        // ==================================================
        float active_pwm1 = pwm1;
        float active_pwm2 = pwm2;
        
        // Safety shutdown: zero power if compensator is detached
        if (!runloop) {
            active_pwm1 = 0.0f;
            active_pwm2 = 0.0f;
        }

        // Integrate thermal physics
        rk4_step(&s, active_pwm1, active_pwm2, dt, &p);

        i++; 
        // ==================================================
        // 2. LOW FREQUENCY: COMPENSATOR & TELEMETRY (1kHz)
        // ==================================================
        if (i >= 20) {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed_ms = (current_time.tv_sec - last_comp_time.tv_sec) * 1000.0 +
                                (current_time.tv_nsec - last_comp_time.tv_nsec) / 1000000.0;
            
            static int int_print_throttle = 0;                    
            if (elapsed_ms > 1.05 || elapsed_ms < 0.95) {
                if (++int_print_throttle > 200) { 
                    printf("\rTiming Jitter Alert: Cycle took %.4f ms     \r", elapsed_ms);
                    fflush(stdout);
                    int_print_throttle = 0;
                }
            }
            last_comp_time = current_time;

            // Generate sensor measurement noise for this 1ms cycle
            float jitter1 = 0.0f;
            float jitter2 = 0.0f;
#if NOISE_SENSOR_JITTER
            jitter1 = ((float)rand() / RAND_MAX - 0.5f) * JITTER_STRENGTH;
            jitter2 = ((float)rand() / RAND_MAX - 0.5f) * JITTER_STRENGTH;
#endif
            float measured_T1 = s.T1 + jitter1;
            float measured_T2 = s.T2 + jitter2;

            if (runloop) {
                /* --- COMPENSATOR HOOK (1 kHz) --- */
                run_thermal_compensator_cycle(measured_T1, measured_T2, &p, &pwm1, &pwm2);
            } else {
                pwm1 = 0.0f;
                pwm2 = 0.0f;
            }
            
            // Stream telemetry: <T1  T2  u1  u2  q12  Tamb>
            if (sim_3d_in >= 0 && sim_graph_in >= 0) {
                char buf[192]; 

                int n = snprintf(buf, 192, "<%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f>\n", 
                                          s.T1, s.T2, 
                                          s.u1, s.u2,
                                          s.q12, p.Tamb);
                write(sim_3d_in, buf, n);
                write(sim_graph_in, buf, n);
            }
            i = 0;
        }
    
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_step, NULL); 
    }
    return 0;
}