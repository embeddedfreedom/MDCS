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

Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

#ifndef COMPENSATOR_H
#define COMPENSATOR_H
#include <stdbool.h>
#include "furuta_types.h"

// The Bridge between 20kHz Plant and 1kHz Controller
#define MAX_BUS_STATES 512 // Increased to be safe for high-speed swings

typedef struct {
    // Arm Data
    int arm_states[MAX_BUS_STATES]; // Raw 0,1,2,3 quadrature sequence
    int n_arm;                      // How many states are in the buffer
    int z_idx_arm;                  // Index within arm_states where Z occurred
    bool z_arm;                     // Latched Z-crossing flag
    // Pendulum Data
    int pend_states[MAX_BUS_STATES]; // Raw 0,1,2,3 quadrature sequence
    int n_pend;                      // How many states are in the buffer
    int z_idx_pend;                  // Index within pend_states where Z occurred
    bool z_pend;                     // Latched Z-crossing flag
} EncoderBus;



typedef struct {
    float x[4];      // [theta1, theta2, d_theta1, d_theta2]
    float P[4][4];   // Uncertainty (How much we trust our estimate)
    float Q[4][4];   // Process Noise (How much we trust the physics)
    float R[2][2];   // Measurement Noise (How much we trust the encoders)
} EKF;



// The Hook: Students implement this, or you provide the binary version
// We pass FurutaParams to allow students to use physical constants in their control
float run_compensator_cycle(EncoderBus *bus, FurutaParams *p, int *motor_dir);

#endif