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

#ifndef FURUTA_TYPES_H
#define FURUTA_TYPES_H

#define PI 3.14159265358979323846f

typedef struct { 
    float theta1, theta1_dot, 
    theta2, theta2_dot, command_voltage, torque; 
} FurutaState;

typedef struct { 
    float M1, M2, M3, L1, L2, L3, r1, r2, r3, shaft_radius, shaft_mass;
    float b1, b2, g, kt, kb, Vmax, R, deadzone; 
} FurutaParams;

#endif