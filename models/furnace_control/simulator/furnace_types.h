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

typedef struct { 
    float T1, T2;
    float T1_dot, T2_dot;
    float u1, u2;
    float q12;
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
