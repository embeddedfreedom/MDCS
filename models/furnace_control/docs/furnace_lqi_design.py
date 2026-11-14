#!/usr/bin/env python3
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
furnace_lqi_design.py
=====================

Compute the LQI (Linear Quadratic Integral) gains for the coupled heat slab
furnace plant. Students can edit Q/R weights, setpoints, or physical parameters
and re-run to see how the gains change.

Output:
  - Prints A, B, A_aug, B_aug matrices
  - Solves the continuous-time Algebraic Riccati Equation (ARE)
  - Prints K_x (state feedback) and K_i (integral gain) in numeric and C format
  - Prints feedforward pwm_sp values

Run:
  python3 docs/furnace_lqi_design.py

Requires:
  numpy, scipy
"""

import numpy as np
from scipy.linalg import solve_continuous_are


# ============================================================================
# 1. PHYSICAL PARAMETERS
# ============================================================================
# Industrial parameter set (BENCH_TEST_ACCELERATED = 0 in furnace_sim.c)
C1 = 500.0          # Thermal capacitance node 1 (J/°C)
C2 = 450.0          # Thermal capacitance node 2 (J/°C)
K12 = 2.5           # Inter-nodal conductance (W/°C)
ha1_A1 = 0.5        # Ambient convection node 1 (W/°C)
ha2_A2 = 0.4        # Ambient convection node 2 (W/°C)
Tamb = 25.0         # Ambient temperature (°C)
Vmax = 12.0         # Max heater voltage (V)
R_heater = 4.0      # Heater resistance (ohms) — P = (Vmax * pwm)^2 / R

T1_setpoint = 75.0  # Target temperature zone 1 (°C)
T2_setpoint = 75.0  # Target temperature zone 2 (°C)

# Bench parameter set (BENCH_TEST_ACCELERATED = 1) — uncomment to use:
# C1 = 5.0
# C2 = 4.5
# K12 = 2.5
# ha1_A1 = 0.5
# ha2_A2 = 0.4
# Tamb = 25.0
# Vmax = 12.0
# R = 4.0
# T1_setpoint = 75.0
# T2_setpoint = 75.0

# ============================================================================
# 2. STATE-SPACE MATRICES (from energy balance derivation)
# ============================================================================
# A = d(f)/d(x) where f = [dT1/dt, dT2/dt]^T and x = [T1, T2]^T
# (A is independent of PWM — depends only on thermal parameters)
A = np.array([
    [-(K12 + ha1_A1) / C1,   K12 / C1],
    [ K12 / C2,             -(K12 + ha2_A2) / C2]
], dtype=float)

# B = d(f)/d(pwm) where pwm = [pwm1, pwm2]^T (PWM duty cycle [0, 1])
# Heater power: P = (Vmax * pwm)^2 / R
# dP/dpwm = 2 * Vmax^2 * pwm / R   (linearized at operating point)
# dT/dpwm = dP/dpwm / C

# First, compute steady-state power needed at setpoint:
# P_sp = (K12 + ha)*Tsp - K12*T_other - ha*Tamb
p_sp1 = (K12 + ha1_A1) * T1_setpoint - K12 * T2_setpoint - ha1_A1 * Tamb
p_sp2 = (K12 + ha2_A2) * T2_setpoint - K12 * T1_setpoint - ha2_A2 * Tamb

# Convert power to PWM: pwm = sqrt(P * R) / Vmax
pwm_sp1 = (p_sp1 * R_heater) ** 0.5 / Vmax if p_sp1 > 0 else 0.0
pwm_sp2 = (p_sp2 * R_heater) ** 0.5 / Vmax if p_sp2 > 0 else 0.0

# Linearize B at operating point: dT/dpwm = (2 * Vmax^2 * pwm_sp / R) / C
b11 = 2 * Vmax**2 * pwm_sp1 / (R_heater * C1)
b22 = 2 * Vmax**2 * pwm_sp2 / (R_heater * C2)

B = np.array([
    [b11, 0.0],
    [0.0, b22]
], dtype=float)

print("=" * 60)
print("PLANT MATRICES (PWM input, linearized at operating point)")
print("=" * 60)
print(f"\nOperating point PWM: pwm_sp1 = {pwm_sp1:.4f}, pwm_sp2 = {pwm_sp2:.4f}")
print(f"Steady-state power:    p_sp1 = {p_sp1:.2f} W,   p_sp2 = {p_sp2:.2f} W")
print("\nA matrix (state derivatives w.r.t. states):")
print(A)
print("\nB matrix (state derivatives w.r.t. PWM inputs):")
print(B)
print("(B entries: dT/dpwm = 2*Vmax^2*pwm_sp/(R*C), linearized)")

# ============================================================================
# 3. AUGMENTED SYSTEM (LQI: append integral error states)
# ============================================================================
# Output matrix C_out — we measure both temperatures directly
C_out = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
], dtype=float)

nx = A.shape[0]       # 2 plant states
nu = B.shape[1]       # 2 inputs
ny = C_out.shape[0]   # 2 measured outputs

zeros_sx_o = np.zeros((nx, ny))
zeros_ox_o = np.zeros((ny, ny))
zeros_ox_i = np.zeros((ny, nu))

A_aug = np.block([
    [A,          zeros_sx_o],
    [C_out,      zeros_ox_o]
])

B_aug = np.block([
    [B],
    [zeros_ox_i]
])

print("\nC_out matrix (output selection):")
print(C_out)

print("\n" + "=" * 60)
print("AUGMENTED SYSTEM (4 states: T1, T2, q1, q2)")
print("=" * 60)
print("\nA_aug:")
print(A_aug)
print("\nB_aug:")
print(B_aug)

# ============================================================================
# 4. LQI WEIGHTING MATRICES (Bryson's Rule)
# ============================================================================
# Bryson's rule: weight = 1 / (max acceptable value)^2
# Temperature error: max 5 °C  -> q_T = 1/5^2 = 0.04
# Integral error:   max 50 °C·s -> q_I = 1/50^2 = 0.0004
# Control effort:   max 100 PWM  -> r_u = 1/100^2 = 0.0001

q_temp = 1.0 / 5.0**2      # 0.04
q_int  = 1.0 / 50.0**2     # 0.0004
r_eff  = 1.0 / 100.0**2    # 0.0001

Q_aug = np.diag([q_temp, q_temp, q_int, q_int])
R = np.diag([r_eff, r_eff])

print("\n" + "=" * 60)
print("WEIGHTING MATRICES (Bryson's Rule)")
print("=" * 60)
print(f"\nQ_aug = diag([{q_temp}, {q_temp}, {q_int}, {q_int}])")
print(f"R     = diag([{r_eff}, {r_eff}])")
print("\nQ_aug:")
print(Q_aug)
print("\nR:")
print(R)

# ============================================================================
# 5. SOLVE CONTINUOUS-TIME ARE AND COMPUTE GAINS
# ============================================================================
# ARE: A_aug^T P + P A_aug - P B_aug R^-1 B_aug^T P + Q_aug = 0
P_aug = solve_continuous_are(A_aug, B_aug, Q_aug, R)

# K_aug = R^-1 B_aug^T P  (shape: 2 x 4)
K_aug = np.linalg.inv(R) @ B_aug.T @ P_aug

# Partition: K_aug = [K_x | K_i]
K_x = K_aug[:, :nx]   # first 2 columns: state feedback
K_i = K_aug[:, nx:]   # last 2 columns: integral gain

print("\n" + "=" * 60)
print("OPTIMAL GAINS (from Riccati solution)")
print("=" * 60)
print("\nP_aug (solution to ARE):")
print(np.round(P_aug, 6))
print("\nK_aug = [K_x | K_i]:")
print(np.round(K_aug, 6))
print("\nK_x (state feedback gain, 2x2):")
print(np.round(K_x, 6))
print("\nK_i (integral gain, 2x2):")
print(np.round(K_i, 6))

# Closed-loop poles for stability check
A_cl = A_aug - B_aug @ K_aug
eigs = np.linalg.eigvals(A_cl)
print("\nClosed-loop augmented poles:")
print(np.round(eigs, 6))
print(f"Minimum real part: {np.min(np.real(eigs)):.6f}  "
      f"({'STABLE' if np.min(np.real(eigs)) < 0 else 'UNSTABLE'})")

# ============================================================================
# 6. FEEDFORWARD pwm_sp (steady-state PWM to hold setpoint)
# ============================================================================

# Steady-state power needed: P = (K12+ha)*Tsp - K12*T_other - ha*Tamb
p_sp1 = float((K12 + ha1_A1) * T1_setpoint - K12 * T2_setpoint - ha1_A1 * Tamb)
p_sp2 = float((K12 + ha2_A2) * T2_setpoint - K12 * T1_setpoint - ha2_A2 * Tamb)

# Convert power to PWM: pwm = sqrt(P * R) / Vmax
pwm_sp1 = (p_sp1 * R_heater) ** 0.5 / Vmax if p_sp1 > 0 else 0.0
pwm_sp2 = (p_sp2 * R_heater) ** 0.5 / Vmax if p_sp2 > 0 else 0.0

print("\n" + "=" * 60)
print("FEEDFORWARD pwm_sp (steady-state PWM duty cycle)")
print("=" * 60)
print(f"\nT1_setpoint = {T1_setpoint} °C, T2_setpoint = {T2_setpoint} °C, Tamb = {Tamb} °C")
print(f"Steady-state power:  p_sp1 = {float(p_sp1):.4f} W,   p_sp2 = {float(p_sp2):.4f} W")
print(f"Steady-state PWM:    pwm_sp1 = {float(pwm_sp1):.4f},   pwm_sp2 = {float(pwm_sp2):.4f}")

# ============================================================================
# 7. PRINT GAINS IN C FORMAT (copy-paste into compensator.c)
# ============================================================================
print("\n" + "=" * 60)
print("C CODE — copy these #define values into compensator.c")
print("=" * 60)
print(f"\n#define IND_KX_00 {K_x[0,0]:.6f}f")
print(f"#define IND_KX_01 {K_x[0,1]:.6f}f")
print(f"#define IND_KX_10 {K_x[1,0]:.6f}f")
print(f"#define IND_KX_11 {K_x[1,1]:.6f}f")
print(f"#define IND_KI_00 {K_i[0,0]:.6f}f")
print(f"#define IND_KI_01 {K_i[0,1]:.6f}f")
print(f"#define IND_KI_10 {K_i[1,0]:.6f}f")
print(f"#define IND_KI_11 {K_i[1,1]:.6f}f")
print(f"\n#define PWM_SP1 {float(pwm_sp1):.4f}f")
print(f"#define PWM_SP2 {float(pwm_sp2):.4f}f")

print("\nDone.")
