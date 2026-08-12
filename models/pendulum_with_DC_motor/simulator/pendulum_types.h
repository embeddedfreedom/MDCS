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

#ifndef PENDULUM_TYPES_H
#define PENDULUM_TYPES_H

#define PI 3.14159265358979323846f

typedef struct { 
    float theta, theta_dot, command_voltage, torque; 
} PendulumState;

typedef struct { 
    float M1, L1, r3, motor_shaft_radius, motor_shaft_mass;
    float b1, g, kt, kb, Vmax, R, deadzone; 
} PendulumParams;

#endif
