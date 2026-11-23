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

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h> 
#include "encoder_decoder_logic.h"
#include "flywheel_types.h"       // Swapped for updated structural definitions
#include "compensator.h"          // Shared bus structure
#include "get_keypress.h"
#include "mdcs_config.h"

#ifdef HIL_BUILD
#include <termios.h>
int serial_fd = -1;

static int hil_serial_open(void) {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
    int retries = 20;
    while (retries-- > 0) {
        serial_fd = open(MDCS_HIL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd >= 0) {
            struct termios tty;
            if (tcgetattr(serial_fd, &tty) == 0) {
                cfmakeraw(&tty);
                cfsetispeed(&tty, B921600);
                cfsetospeed(&tty, B921600);
                tcsetattr(serial_fd, TCSANOW, &tty);
                return 0;
            }
            close(serial_fd);
            serial_fd = -1;
        }
        usleep(250000); /* 250 ms */
    }
    return -1;
}
#endif

/* All three profiles use the common 20-bit codec. HIL still sends a float
 * angle in radians to the MCU, so the wire format is unchanged. */
#if defined(SPEED_PROFILE) || defined(POSITION_PROFILE) || defined(HIL_PROFILE)
#define USE_ENCODER20 1
#endif

// --- NOISE TOGGLES (1=ON, 0=OFF) ---
#define NOISE_PWM_VIBRATION    0  // Voltage fluctuations
#define NOISE_SENSOR_JITTER    0  // Mechanical vibration (high freq)
#define NOISE_MISSING_PULSES   0  // Optical disk skips (causes drift)
#define NOISE_ENCODER_SPIKE    0  // Stray electrical pulses (causes drift)

// --- NOISE PARAMETERS ---
#define JITTER_STRENGTH        0.1f   // Degrees of vibration
#define SPIKE_PROBABILITY      2000   // 1 in 2000 chance per 50us
#define PWM_NOISE_STRENGTH     0.25f  // Volts

void normalize_angle(float *angle) {
    while (*angle >  PI) *angle -= 2.0f * PI;
    while (*angle < -PI) *angle += 2.0f * PI;
}

// Gray code equivalent for optical quadrature encoder mapping
static const uint8_t phase_to_gray[4] = {
    0b00, // 0: Both signals low
    0b01, // 1: Channel B rises
    0b11, // 2: Channel A rises (both high)
    0b10  // 3: Channel B falls
};

void compute_derivatives(const FlywheelState *x, float motor_pwm, int motor_dir, const FlywheelParams *p, FlywheelState *dx) {

    float d = x->theta_dot;
    
    // Motor Domain Model
    float V_requested = motor_pwm * p->Vmax * (float)motor_dir;
    float V_effective = 0.0f;

    // Direction-dependent deadzone compensation
    if (fabsf(V_requested) > p->deadzone) {
        V_effective = (V_requested > 0) ? (V_requested - p->deadzone) : (V_requested + p->deadzone);
    }

    // Electromechanical torque calculation including Back-EMF
    // Motor off and coasting — no electrical braking when PWM=0
    float tau_motor;
    if (fabsf(V_effective) < 0.001f && p->motor_coast_on_zero) {
        tau_motor = 0.0f;
    } else {
        tau_motor = (p->kt / p->R) * (V_effective - p->kb * d);
    }

    // Rigid Body Inertia Components calculated dynamically using passed parameters
    float I_shaft = 0.5f * p->motor_shaft_mass * p->motor_shaft_radius * p->motor_shaft_radius;
    
    // Dynamic moment of inertia for a flat solid cylinder: I = 0.5 * M * R^2
    float I_disc  = 0.5f * p->M_disc * p->R_disc * p->R_disc;
    float I_tot   = I_shaft + I_disc;

    // State Equations of Motion:
    // dx1/dt = theta_dot
    // dx2/dt = (tau_motor - b1*theta_dot) / I_tot -> NO GRAVITY (zero potential energy variation)
    dx->theta = d;
    dx->theta_dot = (tau_motor - p->b1 * d) / I_tot;
    
    dx->command_voltage = V_effective;
    dx->torque = tau_motor;
}

void rk4_step(FlywheelState *x, float pwm, int dir, float dt, const FlywheelParams *p) {
    FlywheelState k1, k2, k3, k4, xt;
    compute_derivatives(x, pwm, dir, p, &k1);
    xt.theta = x->theta + 0.5f*dt*k1.theta; 
    xt.theta_dot = x->theta_dot + 0.5f*dt*k1.theta_dot;
    compute_derivatives(&xt, pwm, dir, p, &k2);
    xt.theta = x->theta + 0.5f*dt*k2.theta; 
    xt.theta_dot = x->theta_dot + 0.5f*dt*k2.theta_dot;
    compute_derivatives(&xt, pwm, dir, p, &k3);
    xt.theta = x->theta + dt*k3.theta; 
    xt.theta_dot = x->theta_dot + dt*k3.theta_dot;
    compute_derivatives(&xt, pwm, dir, p, &k4);
    
    x->theta += (dt/6.0f)*(k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);
    x->theta_dot += (dt/6.0f)*(k1.theta_dot + 2*k2.theta_dot + 2*k3.theta_dot + k4.theta_dot);
    
    normalize_angle(&x->theta); 
    x->command_voltage = k4.command_voltage;
    x->torque = k4.torque;
}

EncoderBus bus = {0};

// Global runloop flag shared with the HIL compensator (set true via 's' key,
// false via 't' key or on HIL connection loss)
bool runloop = false;

int main() {
    
    // Setup physical constants derived from the 15cm x 1cm steel disc parameters
    FlywheelParams p = { 
        .M_disc = 1.3872f,              // Calculated mass of steel disc
        .R_disc = 0.075f,               // Radius = 15cm / 2 = 7.5cm = 0.075m
        .motor_shaft_radius = 0.005f,   // Standardized hardware values passed down
        .motor_shaft_mass = 0.06f,
        .b1 = 0.002f, 
        .g = 9.81f, 
        .kt = 0.12f, 
        .kb = 0.12f, 
        .Vmax = 12.0f, 
        .R = 2.5f, 
        .deadzone = 0.4f, 
        .motor_coast_on_zero = true
    };
    
    FlywheelState s = {0.0f, 0.0f, 0.0f, 0.0f}; 


#ifdef USE_ENCODER20
    Encoder20 e_flywheel20;
    encoder20_init(&e_flywheel20);
#else
    Encoder e_flywheel;
    encoder_init(&e_flywheel);
#endif

    double cum_theta = (double)s.theta * (180.0 / (double)PI);
#ifdef USE_ENCODER20
    e_flywheel20.prev_angle = cum_theta;
#else
    e_flywheel.prev_angle = (float)cum_theta;
#endif
    
    float dt = 0.00005f; // 20kHz Physics Step
    //Change the path values as per your directory structure, modify the socat command accordingly
    int sim_3d_in = open(MDCS_PTY_SIM_3D_IN, O_WRONLY | O_NONBLOCK);
    int sim_graph_in = open(MDCS_PTY_SIM_GRAPH_IN, O_WRONLY | O_NONBLOCK);
#ifdef HIL_BUILD
    hil_serial_open(); // non-fatal; press 'S' to retry connection
#endif
    
    int i = 0, dir = 1;
    float pwm = 0;
#ifndef SPEED_PROFILE
    bool z_prev_in_window = false;  // Z index is one-shot per revolution
#endif

    struct timespec next_step, last_comp_time, current_time;
    const long NS_PER_STEP = 50000; // 50us
    clock_gettime(CLOCK_MONOTONIC, &next_step);
    last_comp_time = next_step; 

    while(1) {
        char key = get_keypress();

        if (key == 's' || key == 'S') {
#ifdef HIL_BUILD
            if (serial_fd < 0 && hil_serial_open() < 0) {
                printf("\n>> CANNOT CONNECT — press 'S' to retry\n");
            } else {
                runloop = true;
                printf("\n>> COMPENSATOR CONNECTED\n");
            }
#else
            runloop = true;
            printf("\n>> COMPENSATOR CONNECTED\n");
#endif
        } 
        else if (key == 't' || key == 'T') {
            runloop = false;
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
        if (runloop == true) {
            active_pwm += ((float)rand() / RAND_MAX - 0.5f) * PWM_NOISE_STRENGTH;
            // SATURATION FIX: Prevent the value from going outside [0, 1]
            if (active_pwm > 1.0f){
                 active_pwm = 1.0f;
            }
            if (active_pwm < 0.0f) {
                 active_pwm = 0.0f;
            }
        } 
        else if (runloop == false) {
            active_pwm = 0;   
        }
        #endif
        
        rk4_step(&s, active_pwm, dir, dt, &p);

        // 3. Encoder Signal Corruptions
#ifdef USE_ENCODER20
        int encoder_states[ENCODER20_MAX_STATES_PER_CALL];
#else
        int encoder_states[MAX_ALLOWED_STATES];
#endif
        int n_states;

        double d_theta = ((double)s.theta - (double)prev_theta) * (180.0 / (double)PI);
        if (d_theta > 180.0)  d_theta -= 360.0;
        if (d_theta < -180.0) d_theta += 360.0;
        cum_theta += d_theta;

        float jitter = 0;
        #if NOISE_SENSOR_JITTER
        jitter = ((float)rand() / RAND_MAX - 0.5f) * JITTER_STRENGTH;
        #endif

        // Apply Jitter here
#ifdef USE_ENCODER20
        encoder20_process_angle(&e_flywheel20,
                                cum_theta + (double)jitter,
                                encoder_states, &n_states);
#else
        encoder_process_angle(&e_flywheel,
                              (float)(cum_theta + (double)jitter),
                              encoder_states, &n_states);
#endif

        for(int j = 0; j < n_states; j++) {
            #if NOISE_MISSING_PULSES
            if ((rand() % 1000) == 0) continue; 
            #endif
            if (bus.n_pend < MAX_BUS_STATES) bus.pend_states[bus.n_pend++] = encoder_states[j];
        }

        #if NOISE_ENCODER_SPIKE
        if ((rand() % SPIKE_PROBABILITY) == 0) {
            if (bus.n_pend < MAX_BUS_STATES) bus.pend_states[bus.n_pend++] = phase_to_gray[rand() % 4];
        }
        #endif

        #ifndef SPEED_PROFILE
        // Z-Pulse Index Track Sync Window. Emit a single one-shot pulse on
        // entry, like a real index gap; holding z_pend true for the whole
        // +/-0.1 deg window would re-snap (and lose) the count every 1 ms.
        // Speed control derives velocity from pulse counts only, so it does
        // not generate or consume a Z pulse at all.
        float wrap_t = fmodf(s.theta * (180.0f / PI) + 180.0f, 360.0f) - 180.0f;
        bool z_in_window = (fabsf(wrap_t) < 0.1f);
        if (z_in_window && !z_prev_in_window) {
            bus.z_pend = true;
            bus.z_idx_pend = bus.n_pend;
        }
        z_prev_in_window = z_in_window;
#endif

        // 4. Downsampled 1kHz Control Loop
        i++;
        if (i >= 20) { 
            // Timing Check
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed_ms = (current_time.tv_sec - last_comp_time.tv_sec) * 1000.0 +
                                (current_time.tv_nsec - last_comp_time.tv_nsec) / 1000000.0;

            // Print jitter alert immediately and flush so the stdio buffer
            // never fills up and blocks the real-time loop.
            if (elapsed_ms > 1.05 || elapsed_ms < 0.95) {
                printf("\rTiming Jitter Alert: Cycle took %.4f ms     \r", elapsed_ms);
                fflush(stdout);
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

            if (runloop == true) {
                pwm = run_compensator_cycle(&bus, &p, &dir);
            }
            else if (runloop == false)
            {
                 pwm = 0;   
            }
            bus.n_pend = 0;
            bus.z_pend = false;
            i = 0;
        }

        // 5. Hard Real-Time Sleep
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_step, NULL);
    }
    
    return 0;
}
