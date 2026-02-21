/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

#include <stdio.h>
#include <unistd.h>
#include<time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h> 
#include "encoder_decoder_logic.h"
#include "pendulum_types.h"
#include "compensator.h" // Note: Shared bus structure from your header
#include "get_keypress.h"

// --- NOISE TOGGLES (1=ON, 0=OFF) ---
#define NOISE_PWM_VIBRATION    1  // Voltage fluctuations
#define NOISE_SENSOR_JITTER    1  // Mechanical vibration (high freq)
#define NOISE_MISSING_PULSES   1  // Optical disk skips (causes drift)
#define NOISE_ENCODER_SPIKE    1  // Stray electrical pulses (causes drift)

// --- NOISE PARAMETERS ---
#define JITTER_STRENGTH        0.1f   // Degrees of vibration
#define SPIKE_PROBABILITY      2000   // 1 in 2000 chance per 50us
#define PWM_NOISE_STRENGTH     0.25f  // Volts

void normalize_angle(float *angle) {
    while (*angle >  PI) *angle -= 2.0f * PI;
    while (*angle < -PI) *angle += 2.0f * PI;
}

// This array maps the index (0-3) to the Gray code equivalent
// used by optical encoders.
static const uint8_t phase_to_gray[4] = {
    0b00, // 0: Both signals low
    0b01, // 1: Channel B rises
    0b11, // 2: Channel A rises (both high)
    0b10  // 3: Channel B falls
};

void compute_derivatives(const PendulumState *x, float motor_pwm, int motor_dir, const PendulumParams *p, PendulumState *dx) {
    float t = x->theta;
    float d = x->theta_dot;
    
    // Motor Model
    float V_requested = motor_pwm * p->Vmax * (float)motor_dir;
    float V_effective = 0.0f;

    if (fabsf(V_requested) > p->deadzone) {
        V_effective = (V_requested > 0) ? (V_requested - p->deadzone) : (V_requested + p->deadzone);
    }

    float tau_motor = (p->kt / p->R) * (V_effective - p->kb * d);

    // Rigid Body Inertia
    float I_shaft = 0.5f * p->motor_shaft_mass * p->motor_shaft_radius * p->motor_shaft_radius;
    float I_pend  = (1.0f/3.0f) * p->M1 * p->L1 * p->L1 + 0.25f * p->M1 * p->r3 * p->r3;
    float I_tot   = I_shaft + I_pend;

    // Equation of Motion: I*ddtheta + b*dtheta + mgl/2*sin(theta) = tau
    dx->theta = d;
    dx->theta_dot = (tau_motor - p->b1 * d - (p->M1 * p->g * p->L1 / 2.0f) * sinf(t)) / I_tot;
    dx->command_voltage = V_effective;
    dx->torque = tau_motor;
}

void rk4_step(PendulumState *x, float pwm, int dir, float dt, const PendulumParams *p) {
    PendulumState k1, k2, k3, k4, xt;
    compute_derivatives(x, pwm, dir, p, &k1);
    xt.theta = x->theta + 0.5f*dt*k1.theta; xt.theta_dot = x->theta_dot + 0.5f*dt*k1.theta_dot;
    compute_derivatives(&xt, pwm, dir, p, &k2);
    xt.theta = x->theta + 0.5f*dt*k2.theta; xt.theta_dot = x->theta_dot + 0.5f*dt*k2.theta_dot;
    compute_derivatives(&xt, pwm, dir, p, &k3);
    xt.theta = x->theta + dt*k3.theta; xt.theta_dot = x->theta_dot + dt*k3.theta_dot;
    compute_derivatives(&xt, pwm, dir, p, &k4);
    x->theta += (dt/6.0f)*(k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);
    x->theta_dot += (dt/6.0f)*(k1.theta_dot + 2*k2.theta_dot + 2*k3.theta_dot + k4.theta_dot);
    normalize_angle(&x->theta); 
    x->command_voltage = k4.command_voltage;
    x->torque = k4.torque;
}

EncoderBus bus = {0}; // Using your MAX_BUS_STATES structure

int main() {
    char key;
    bool runLoop = false;
    PendulumParams p = { .M1=0.2f, .L1=0.3f, .r3=0.01f, 
                         .motor_shaft_radius=0.005f, .motor_shaft_mass=0.06f,
                         .b1=0.008f, .g=9.81f, .kt=0.12f, .kb=0.12f, 
                         .Vmax=12.0f, .R=2.5f, .deadzone=0.4f};
    
    PendulumState s = {0.0f, 0.0f, 0.0f, 0.0f}; 
    Encoder e_pend;
    encoder_init(&e_pend);

    float cum_theta = s.theta * (180.0f / PI);
    e_pend.prev_angle = cum_theta;
    
    float dt = 0.00005f; // 20kHz
    //Change the path values as per your directory structure, modify the socat command accordingly
    int sim_3d_in = open("/home/dipto/sim_3d_in", O_WRONLY | O_NONBLOCK);
    int sim_graph_in = open("/home/dipto/sim_graph_in", O_WRONLY | O_NONBLOCK);
    
    int i = 0, dir = 1;
    float pwm = 0;

    // --- TIMING SETUP ---
    struct timespec next_step, last_comp_time, current_time;
    const long NS_PER_STEP = 50000; // 50us
    clock_gettime(CLOCK_MONOTONIC, &next_step);
    last_comp_time = next_step; 

    while(1) {
        // Check if a key has been pressed
        char key = get_keypress();

        if (key == 's' || key == 'S') {
            runLoop = true;
            printf("\n>> COMPENSATOR CONNECTED\n");
        } 
        else if (key == 't' || key == 'T') {
            runLoop = false;
            printf("\n>> COMPENSATOR DISCONNECTED\n");
        } 
        else if (key == 'q' || key == 'Q') {
            printf("\nExiting...\n");
            break;
        }
        // 1. Advance Schedule
        next_step.tv_nsec += NS_PER_STEP;
        if (next_step.tv_nsec >= 1000000000L) {
            next_step.tv_nsec -= 1000000000L;
            next_step.tv_sec++;
        }

        // 2. Physics Step
        float prev_theta = s.theta;
        float active_pwm = pwm;
        #if NOISE_PWM_VIBRATION
        if (runLoop == true) {
                active_pwm += ((float)rand() / RAND_MAX - 0.5f) * PWM_NOISE_STRENGTH;
                // SATURATION FIX: Prevent the value from going outside [0, 1]
                if (active_pwm > 1.0f){
                     active_pwm = 1.0f;
                }
                if (active_pwm < 0.0f) {
                 active_pwm = 0.0f;
               }
            }
            else if (runLoop == false)
            {
                 active_pwm = 0;   
            }
        
        #endif
        
        rk4_step(&s, active_pwm, dir, dt, &p);

        // 3. Encoder Processing with Restored Jitter
        int local_states[MAX_ALLOWED_STATES]; 
        int n_local;

        float d_theta = (s.theta - prev_theta) * (180.0f / PI);
        if (d_theta > 180.0f)  d_theta -= 360.0f;
        if (d_theta < -180.0f) d_theta += 360.0f;
        cum_theta += d_theta;

        float jitter = 0;
        #if NOISE_SENSOR_JITTER
        jitter = ((float)rand() / RAND_MAX - 0.5f) * JITTER_STRENGTH;
        #endif

        // Apply Jitter here
        encoder_process_angle(&e_pend, cum_theta + jitter, local_states, &n_local);
        
        for(int j = 0; j < n_local; j++) {
            #if NOISE_MISSING_PULSES
            if ((rand() % 1000) == 0) continue; 
            #endif
            if (bus.n_pend < MAX_BUS_STATES) bus.pend_states[bus.n_pend++] = local_states[j];
        }

        #if NOISE_ENCODER_SPIKE
        if ((rand() % SPIKE_PROBABILITY) == 0) {
            if (bus.n_pend < MAX_BUS_STATES) bus.pend_states[bus.n_pend++] = phase_to_gray[rand() % 4];
        }
        #endif

        // Z-Pulse Logic
        float wrap_t = fmodf(s.theta * (180.0f / PI) + 180.0f, 360.0f) - 180.0f;
        if (fabs(wrap_t) < 0.1f) {
            bus.z_pend = true;
            bus.z_idx_pend = bus.n_pend;
        }

        // 4. 1kHz Compensator Loop
        i++;
        if (i >= 20) { 
            // Timing Check
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed_ms = (current_time.tv_sec - last_comp_time.tv_sec) * 1000.0 +
                                (current_time.tv_nsec - last_comp_time.tv_nsec) / 1000000.0;

            if (elapsed_ms > 1.05 || elapsed_ms < 0.95) {
                printf("Timing Jitter Alert: Cycle took %.4f ms\n", elapsed_ms);
            }
            last_comp_time = current_time; 

            // Output to visualizers
            if (sim_3d_in >= 0 && sim_graph_in >= 0) {
                char buf[192]; 
                int n = snprintf(buf, 192, "<%.4f\t%.4f\t%.4f\t%.4f>\n", 
                                                          s.theta, s.theta_dot, 
                                                          s.command_voltage, s.torque);
                write(sim_3d_in, buf, n);
                write(sim_graph_in, buf, n);
            }

            if (runLoop == true) {
                //printf("Within Compensator loop");
                pwm = run_compensator_cycle(&bus, &p, &dir);
            }
            else if (runLoop == false)
            {
                 pwm = 0;   
            }
            bus.n_pend = 0;
            bus.z_pend = false;
            i = 0;
        }

        // 5. High-Precision Sync
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_step, NULL);
    }
    return 0;
}
