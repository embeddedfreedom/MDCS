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

#ifndef FURNACE_TYPES_H
#define FURNACE_TYPES_H

#define PI 3.14159265358979323846f

/*
 * Thermal state is double precision on purpose.
 *
 * The furnace is a very slow plant (industrial tau ~1000 s) integrated at
 * 20 kHz. A single RK4 step moves the temperature by roughly
 *
 *     dT = (ha/C) * (T - Tamb) * dt ~ 3e-7 degC
 *
 * which is smaller than one float ULP near 75 degC (~7.6e-6), so a float
 * state rounds the increment away and the slab never cools. Double keeps
 * the slow dynamics resolvable. This is host-side physics only - the MCU
 * compensator never integrates the plant and stays single precision.
 */
typedef struct { 
    double T1, T2;
    double T1_dot, T2_dot;
    double u1, u2;
    double q12;
} ThermalState;

typedef struct { 
    float C1, C2;
    float K12;
    float ha1_A1, ha2_A2;
    float Tamb;
    float Vmax;    // Max heater voltage (V)
    float R;       // Heater resistance (ohms) — P = (Vmax * pwm)^2 / R
} ThermalParams;

#endif
