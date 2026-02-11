/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
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
