#!/usr/bin/env python3
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
#
# pendulum_lqi_design.py
# ======================
# LQR design for Pendulum with DC Motor (models/pendulum_with_DC_motor)
# Mirrors simulator/pendulum_sim.c physics:
#
#   State: x = [theta, theta_dot]
#   Input: u = motor voltage command
#
# Run:
#   python3 docs/pendulum_lqi_design.py
#
# Requires: numpy, scipy

import numpy as np
from scipy.linalg import solve_continuous_are

# ============================================================================
# 1. PARAMETERS (match simulator/pendulum_sim.c PendulumParams)
# ============================================================================
M1 = 0.2          # kg, pendulum mass
L1 = 0.3          # m, pendulum length
I_tot = 0.00600575 # kg*m^2, total inertia (motor shaft + pendulum)
g = 9.81          # m/s^2
b1 = 0.002        # N*m*s/rad, viscous friction (UPDATED from 0.008)
kt = 0.12         # N*m/A, motor torque constant
kb = 0.12         # V*s/rad, back-EMF constant
R = 2.5           # ohm, armature resistance
deadzone = 0.4    # V, motor deadzone

# ============================================================================
# 2. PLANT MATRICES
# ============================================================================
# Linearized around upright (theta = pi):
#   A = [[0, 1], [(M1*g*L1)/(2*I_tot), -(b1 + kt*kb/R)/I_tot]]
#   B = [[0], [(kt/R)/I_tot]]
A = np.array([
    [0, 1],
    [(M1*g*L1)/(2*I_tot), -(b1 + (kt*kb/R))/I_tot]
], dtype=float)

B = np.array([[0], [(kt/R)/I_tot]], dtype=float)

print("=" * 60)
print("PENDULUM LQI DESIGN")
print("=" * 60)
print(f"M1={M1}, L1={L1}, I_tot={I_tot}")
print(f"b1={b1}, kt={kt}, kb={kb}, R={R}, g={g}")
print(f"A matrix:\n{A}")
print(f"B matrix:\n{B}")

# Open-loop poles
eigs_open = np.linalg.eigvals(A)
print(f"\nOpen-loop poles: {np.round(eigs_open, 6)}")

# ============================================================================
# 3. LQR WEIGHTS (from PDF derivation)
# ============================================================================
# Q = diag([4575, 64]) - position error heavily weighted
# R = [[0.1]] - control effort cheap
Q = np.diag([4575, 64])
R_mat = np.array([[0.1]])

# ============================================================================
# 4. SOLVE CONTINUOUS-TIME ARE AND COMPUTE GAINS
# ============================================================================
P = solve_continuous_are(A, B, Q, R_mat)
K_raw = np.linalg.inv(R_mat) @ B.T @ P

# Mapping rule:
# Raw LQR gives u = -K_raw * x
# Our code uses: u = k1 * (theta - pi) + k2 * theta_dot
# So k1 = -K_raw[0,0], k2 = -K_raw[0,1]
k1 = -K_raw[0, 0]
k2 = -K_raw[0, 1]

# Integral gain for steady-state error removal
# From PDF: ki = -75.0
ki = -75.0

print("\n" + "=" * 60)
print("LQR GAINS")
print("=" * 60)
print(f"k1 (Pendulum Pos): {k1:.6f}")
print(f"k2 (Pendulum Vel): {k2:.6f}")
print(f"ki (Integral):     {ki:.6f}")

# Closed-loop poles for stability check
A_cl = A - B @ K_raw
eigs = np.linalg.eigvals(A_cl)
print(f"\nClosed-loop poles:")
for e in eigs:
    print(f"  {e:.6f}")
print(f"Minimum real part: {np.min(np.real(eigs)):.6f}  "
      f"{'STABLE' if np.min(np.real(eigs)) < 0 else 'UNSTABLE'}")

print()
print("=" * 60)
print("C CODE — copy these #define values into compensator.c")
print("=" * 60)
print(f"#define KP  {k1:.6f}f   // Pendulum position gain")
print(f"#define KD  {k2:.6f}f   // Pendulum velocity gain")
print(f"#define KI  {ki:.6f}f   // Integral gain")

print("\nDone.")