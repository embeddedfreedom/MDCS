/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

#include <time.h> // Ensure this is included at the top
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h> 
#include "encoder_decoder_logic.h"
#include "furuta_types.h"
#include "compensator.h"
#include "get_keypress.h"

#define PI 3.14159265358979323846f

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

/**
 * @brief Computes the state derivatives for the Furuta Pendulum.
 * * This follows the verified Full Rigid Body Derivation.
 * State x: [theta1, theta2, theta1_dot, theta2_dot]
 */
void compute_derivatives(const FurutaState *x, float motor_pwm, int motor_dir, const FurutaParams *p, FurutaState *dx) {
    // 1. Local variable setup for readability
    float t2 = x->theta2;
    float d1 = x->theta1_dot;
    float d2 = x->theta2_dot;
    float s2 = sinf(t2); 
    float c2 = cosf(t2);
    float s2c2 = s2 * c2; // Used for sin(2*theta2) = 2 * s2 * c2
    
    // 2. Motor Model: Converts PWM to Torque
    float V_requested = motor_pwm * p->Vmax * (float)motor_dir;
    float V_effective = 0.0f;

    // --- PWM Dead Zone Logic ---
    // Real motors often need ~0.3V to 0.8V just to start turning
    float deadzone = 0.4f; // Set this based on your motor's "stiction"

    if (fabsf(V_requested) > deadzone) {
        // If we are outside the dead zone, we subtract the 'dead' portion 
        // so the torque doesn't jump instantly from 0 to a high value.
        if (V_requested > 0) {
            V_effective = V_requested - deadzone;
        } else {
            V_effective = V_requested + deadzone;
        }
    } else {
        // Inside the dead zone, the motor simply does not produce torque
        V_effective = 0.0f;
    }

    // Now use V_effective to calculate the actual physical torque
    float tau_motor = (p->kt / p->R) * (V_effective - p->kb * d1);

    // 3. Inertia Calculation (Rigid Body Cylinders)
    float J_shaft = 0.5f * p->shaft_mass * p->shaft_radius * p->shaft_radius;
    float J_rod1  = 0.5f * p->M1 * p->r1 * p->r1; // Vertical Riser
    float J_rod2  = (1.0f/12.0f) * p->M2 * p->L2 * p->L2 + 0.25f * p->M2 * p->r2 * p->r2; // Horizontal Arm
    float J3_com  = (1.0f/12.0f) * p->M3 * p->L3 * p->L3 + 0.25f * p->M3 * p->r3 * p->r3; // Pendulum Link

    // 4. Lagrangian Constants
    // Alpha (a): Total base inertia + Steiner term for pendulum mass
    float alpha = J_shaft + J_rod1 + J_rod2 + p->M3 * (p->L2 * p->L2 / 4.0f);
    
    // Beta (b): Pendulum mass-shifting term (M * COM_dist^2)
    float beta  = p->M3 * (p->L3 * p->L3 / 4.0f);
    
    // Gamma (c): Total pendulum inertia about its own pivot
    float gamma = J3_com + beta;
    
    // Delta (d): Cross-coupling constant (M * L2/2 * L3/2)
    float delta = p->M3 * (p->L2 / 2.0f) * (p->L3 / 2.0f);

    // 5. Mass Matrix A(theta) elements
    float a11 = alpha + beta * s2 * s2;
    float a12 = delta * c2;
    float a22 = gamma;

    // 6. Right Hand Side (Generalized Forces)
    // RHS1 = tau_motor - friction - Coriolis/Centrifugal terms
    float rhs1 = tau_motor - p->b1 * d1 - 2.0f * beta * s2c2 * d1 * d2 + delta * s2 * d2 * d2;
    
    // RHS2 = -friction + centrifugal - gravity
    float rhs2 = -p->b2 * d2 + beta * s2c2 * d1 * d1 - p->M3 * p->g * (p->L3 / 2.0f) * s2;

    // 7. Solving for Accelerations (A * ddtheta = RHS)
    // Using Cramer's rule for the 2x2 matrix inversion
    float det = a11 * a22 - a12 * a12;

    dx->theta1 = d1;
    dx->theta2 = d2;
    dx->theta1_dot = (a22 * rhs1 - a12 * rhs2) / det;
    dx->theta2_dot = (a11 * rhs2 - a12 * rhs1) / det;
    dx->command_voltage = V_effective;
    dx->torque = tau_motor;
}



void rk4_step(FurutaState *x, float pwm, int dir, float dt, const FurutaParams *p) {
    FurutaState k1, k2, k3, k4, xt;
    compute_derivatives(x, pwm, dir, p, &k1);
    xt.theta1 = x->theta1 + 0.5f*dt*k1.theta1; xt.theta1_dot = x->theta1_dot + 0.5f*dt*k1.theta1_dot;
    xt.theta2 = x->theta2 + 0.5f*dt*k1.theta2; xt.theta2_dot = x->theta2_dot + 0.5f*dt*k1.theta2_dot;
    compute_derivatives(&xt, pwm, dir, p, &k2);
    xt.theta1 = x->theta1 + 0.5f*dt*k2.theta1; xt.theta1_dot = x->theta1_dot + 0.5f*dt*k2.theta1_dot;
    xt.theta2 = x->theta2 + 0.5f*dt*k2.theta2; xt.theta2_dot = x->theta2_dot + 0.5f*dt*k2.theta2_dot;
    compute_derivatives(&xt, pwm, dir, p, &k3);
    xt.theta1 = x->theta1 + dt*k3.theta1; xt.theta1_dot = x->theta1_dot + dt*k3.theta1_dot;
    xt.theta2 = x->theta2 + dt*k3.theta2; xt.theta2_dot = x->theta2_dot + dt*k3.theta2_dot;
    compute_derivatives(&xt, pwm, dir, p, &k4);
    x->theta1 += (dt/6.0f)*(k1.theta1 + 2*k2.theta1 + 2*k3.theta1 + k4.theta1);
    x->theta1_dot += (dt/6.0f)*(k1.theta1_dot + 2*k2.theta1_dot + 2*k3.theta1_dot + k4.theta1_dot);
    x->theta2 += (dt/6.0f)*(k1.theta2 + 2*k2.theta2 + 2*k3.theta2 + k4.theta2);
    x->theta2_dot += (dt/6.0f)*(k1.theta2_dot + 2*k2.theta2_dot + 2*k3.theta2_dot + k4.theta2_dot);
    normalize_angle(&x->theta1); 
    normalize_angle(&x->theta2);
    x->command_voltage = k4.command_voltage;
    x->torque = k4.torque;
}


// Global bus to be read by the compensator
EncoderBus bus = {{0}, 0, 0, false,
                  {0}, 0, 0, false};
//Change the pendulum weight back to 100g, this is just for testing 

int main() {
    
    char key;
    bool runLoop = false;
    FurutaParams p = { .M1=0.12f, .M2=0.15f, .M3=0.05f, .L1=0.2f, .L2=0.4f, .L3=0.2f, 
                       .r1=0.01f, .r2=0.01f, .r3=0.01f, .shaft_radius=0.005f, .shaft_mass=0.06f,
                       .b1=0.008f, .b2=0.001f, .g=9.81f, .kt=0.12f, .kb=0.12f, .Vmax=12.0f, .R=2.5f,
                       .deadzone=0.4f};
    
    FurutaState s = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
    
    Encoder e_arm, e_pend;
    encoder_init(&e_arm); 
    encoder_init(&e_pend);

    float start_t1 = s.theta1 * (180.0f / PI);
    float start_t2 = s.theta2 * (180.0f / PI);

    double cum_t1 = start_t1;
    double cum_t2 = start_t2;

    e_arm.prev_angle = start_t1; 
    e_pend.prev_angle = start_t2;
    
    float dt = 0.00005f; // 20kHz Physics
    int sim_3d_in = open("/home/dipto/sim_3d_in", O_WRONLY | O_NONBLOCK);
    int sim_graph_in = open("/home/dipto/sim_graph_in", O_WRONLY | O_NONBLOCK);
    int i = 0;
    int dir = 1;
    float pwm = 0;

    bus.n_arm = 0;
    bus.n_pend = 0;
    bus.z_arm = false;
    bus.z_pend = false;

    // --- TIMING SETUP START ---
    struct timespec next_step, last_comp_time, current_time;
    const long NS_PER_STEP = 50000; // 50 microseconds
    clock_gettime(CLOCK_MONOTONIC, &next_step);
    last_comp_time = next_step; 
    // --- TIMING SETUP END ---

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

        // 1. ADVANCE ABSOLUTE TIMELINE
        next_step.tv_nsec += NS_PER_STEP;
        if (next_step.tv_nsec >= 1000000000L) {
            next_step.tv_nsec -= 1000000000L;
            next_step.tv_sec++;
        }
        
        // ==================================================
        // 1. HIGH FREQUENCY: PLANT PHYSICS & ENCODER (20kHz)
        // ==================================================
        float prev_t1 = s.theta1;
        float prev_t2 = s.theta2;

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

        int local_states[MAX_ALLOWED_STATES]; 
        int n_local_arm, n_local_pend;

        float d_t1 = (s.theta1 - prev_t1) * (180.0f / PI);
        if (d_t1 > 180.0f)  d_t1 -= 360.0f; 
        if (d_t1 < -180.0f) d_t1 += 360.0f;
        cum_t1 += d_t1; 
        
        encoder_process_angle(&e_arm, cum_t1, local_states, &n_local_arm);
        for(int j = 0; j < n_local_arm; j++) {
            if (bus.n_arm < MAX_BUS_STATES) bus.arm_states[bus.n_arm++] = local_states[j];
        }

        float d_t2 = (s.theta2 - prev_t2) * (180.0f / PI);
        if (d_t2 > 180.0f)  d_t2 -= 360.0f;
        if (d_t2 < -180.0f) d_t2 += 360.0f;
        cum_t2 += d_t2;

        float jitter = 0;
        #if NOISE_SENSOR_JITTER
        jitter = ((float)rand() / RAND_MAX - 0.5f) * JITTER_STRENGTH;
        #endif

        encoder_process_angle(&e_pend, cum_t2 + jitter, local_states, &n_local_pend);
        for(int j = 0; j < n_local_pend; j++) {
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
        float wrap_t1 = fmodf(s.theta1 * (180.0f / PI) + 180.0f, 360.0f) - 180.0f;
        float wrap_t2 = fmodf(s.theta2 * (180.0f / PI) + 180.0f, 360.0f) - 180.0f;

        if (fabs(wrap_t1) < 0.1f) {  
            bus.z_arm = true;
            bus.z_idx_arm = bus.n_arm; 
        }

        if (fabs(wrap_t2) < 0.1f) {  
            bus.z_pend = true;
            bus.z_idx_pend = bus.n_pend;
        }

        i++; 
        // ==================================================
        // 2. LOW FREQUENCY: COMPENSATOR (1kHz)
        // ==================================================
        if (i >= 20) {
            // --- TIMING CHECK START ---
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed_ms = (current_time.tv_sec - last_comp_time.tv_sec) * 1000.0 +
                                (current_time.tv_nsec - last_comp_time.tv_nsec) / 1000000.0;
            
            if (elapsed_ms > 1.05 || elapsed_ms < 0.95) {
                printf("Timing Jitter Alert: Cycle took %.4f ms\n", elapsed_ms);
            }
            last_comp_time = current_time;
            // --- TIMING CHECK END ---

            // printf("THE ARM ANGLE IN MAIN %f\n", s.theta1 * (180.0f/PI));
            // printf("THE PENDULUM ANGLE IN MAIN %f\n", s.theta2 * (180.0f/PI));
            if (runLoop == true) {
                //printf("Within Compensator loop");
                pwm = run_compensator_cycle(&bus, &p, &dir);
            }
            else if (runLoop == false)
            {
                 pwm = 0;   
            }
            
            if (sim_3d_in >= 0 && sim_graph_in >= 0) {
                char buf[192]; 
                int n = snprintf(buf, 192, "<%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f>\n", 
                                                          s.theta1, s.theta2, 
                                                          s.theta1_dot, s.theta2_dot,
                                                          s.torque, s.command_voltage);
                write(sim_3d_in, buf, n);
                write(sim_graph_in, buf, n);
            }
            bus.n_arm = 0; 
            bus.n_pend = 0;
            bus.z_arm = false;   
            bus.z_pend = false;  
            i = 0;
        }
    
        // Replacement for usleep(50)
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_step, NULL); 
    }
    return 0;
}
