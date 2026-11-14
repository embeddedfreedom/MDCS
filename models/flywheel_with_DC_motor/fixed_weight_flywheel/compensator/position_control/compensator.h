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

#ifndef COMPENSATOR_H
#define COMPENSATOR_H

#include <stdbool.h>
#include "flywheel_types.h"

// The Bridge between the 20kHz Plant (Simulation) and the 1kHz Controller.
// 20-bit encoder at 15 rad/s = ~2503 states/ms; at the 12 V no-load speed
// (~40.5 rad/s) = ~6675 states/ms. 32768 covers both with margin.
#define MAX_BUS_STATES 32768 

typedef struct {
    int pend_states[MAX_BUS_STATES]; // Raw 0,1,2,3 quadrature sequence
    int n_pend;                      // Number of states currently in buffer
    int z_idx_pend;                  // Index where Z-pulse occurred
    bool z_pend;                     // Latch for Z-pulse detection
} EncoderBus;

/**
 * run_compensator_cycle
 * ---------------------\n * Runs at 1kHz. Decodes accumulated quadrature pulses, calculates 
 * filtered position/velocity deviations, and computes the position LQR voltage.
 */
float run_compensator_cycle(EncoderBus *bus, FlywheelParams *p, int *motor_dir);

#endif
