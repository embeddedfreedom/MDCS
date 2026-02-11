/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
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