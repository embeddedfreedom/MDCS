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
#include "pendulum_types.h"

// The Bridge between the 20kHz Plant (Simulation) and the 1kHz Controller
// We use a buffer of states to simulate raw encoder data throughput.
#define MAX_BUS_STATES 512 

typedef struct {
    // Pendulum Data (V2 in your serial protocol)
    int pend_states[MAX_BUS_STATES]; // Raw 0,1,2,3 quadrature sequence
    int n_pend;                      // Number of states currently in buffer
    int z_idx_pend;                  // Index where Z-pulse occurred
    bool z_pend;                     // Latch for Z-pulse detection
} EncoderBus;

/**
 * run_compensator_cycle
 * ---------------------
 * Runs at 1kHz. Decodes accumulated quadrature pulses, calculates 
 * velocity, and computes the LQR control voltage.
 * * @param bus       Pointer to the encoder pulse buffer from the last 1ms.
 * @param p         Physical parameters (mass, length, kt, kb, etc.).
 * @param motor_dir Output: 1 for CW, -1 for CCW.
 * @return          Normalized PWM value [0.0 to 1.0].
 */
float run_compensator_cycle(EncoderBus *bus, PendulumParams *p, int *motor_dir);

#endif
