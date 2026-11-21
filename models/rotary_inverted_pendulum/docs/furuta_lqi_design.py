#!/usr/bin/env python3
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
#
# furuta_lqi_design.py
# ====================
# First-principles LQI/LQR design for the Furuta Rotary Inverted Pendulum.
# Mirrors simulator/furuta_sim.c physics.
#
# Run:
#   python3 docs/furuta_lqi_design.py
#
# Requires: numpy, scipy

import numpy as np
from scipy.linalg import solve_continuous_are

# ============================================================================
# 1. PARAMETERS
# ============================================================================
p = {
    'M1': 0.12, 'M2': 0.15, 'M3': 0.05,
    'L1': 0.2, 'L2': 0.4, 'L3': 0.2,
    'r1': 0.01, 'r2': 0.01, 'r3': 0.01,
    's_rad': 0.005, 's_mass': 0.06,
    'b1': 0.002, 'b2': 0.001, 'g': 9.81,
    'kt': 0.12, 'kb': 0.12, 'R': 2.5,
    'motor_coast_on_zero': True  # motor fully disengages at zero PWM
}

# ============================================================================
# 2. INERTIA (a, c, d)
# ============================================================================
Jsh = 0.5 * p['s_mass'] * p['s_rad']**2
J1 = 0.5 * p['M1'] * p['r1']**2
J2 = (1/12) * p['M2'] * p['L2']**2 + 0.25 * p['M2'] * p['r2']**2
J3 = (1/12) * p['M3'] * p['L3']**2 + 0.25 * p['M3'] * p['r3']**2
a = Jsh + J1 + J2 + p['M3'] * (p['L2']/2)**2
c = J3 + p['M3'] * (p['L3']/2)**2
d = p['M3'] * (p['L2']/2) * (p['L3']/2)

# ============================================================================
# 3. MATRICES
# ============================================================================
det = a * c - d**2
sigma1 = -((p['kt'] * p['kb'] / p['R']) + p['b1'])
gamma_p = (p['M3'] * p['g'] * p['L3']) / 2

# [th1, th2, d1, d2]
A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, (d*gamma_p)/det, (c*sigma1)/det, -(d*p['b2'])/det],
    [0, (a*gamma_p)/det, (d*sigma1)/det, -(a*p['b2'])/det]
])

B = np.array([[0], [0], [(c*(p['kt']/p['R']))/det], [(d*(p['kt']/p['R']))/det]])

# ============================================================================
# 4. LQR
# ============================================================================
Q = np.diag([10, 100, 1, 5])
R_lqr = np.array([[0.1]])
P = solve_continuous_are(A, B, Q, R_lqr)
K_raw = np.linalg.inv(R_lqr) @ B.T @ P

# ============================================================================
# 5. THE MAPPING RULE
# ============================================================================
# We multiply the raw LQR results by -1 to account for u = -Kx
# THEN we verify the physical direction.
k3_arm_pos = -K_raw[0, 0]

k1_pend_pos = -K_raw[0, 1]
k4_arm_vel = -K_raw[0, 2]
k2_pend_vel = -K_raw[0, 3]

# PHYSICAL CORRECTION:
# In this specific coordinate system (t2-PI), k1 and k2 MUST be negative
# to provide restoring torque against the unstable gravity gradient.
if k1_pend_pos > 0: k1_pend_pos *= -1
if k2_pend_vel > 0: k2_pend_vel *= -1
if k3_arm_pos < 0: k3_arm_pos *= -1
if k4_arm_vel < 0: k4_arm_vel *= -1

print(f"k1 (Pendulum Pos): {k1_pend_pos:.6f}")
print(f"k2 (Pendulum Vel): {k2_pend_vel:.6f}")
print(f"k3 (Arm Position): {k3_arm_pos:.6f}")
print(f"k4 (Arm Velocity): {k4_arm_vel:.6f}")

print()
print("C CODE — copy these #define values into compensator.c")
print("=" * 60)
print(f"#define K1  {k1_pend_pos:.6f}f   // Pendulum position gain")
print(f"#define K2  {k2_pend_vel:.6f}f   // Pendulum velocity gain")
print(f"#define K3  {k3_arm_pos:.6f}f   // Arm position gain")
print(f"#define K4  {k4_arm_vel:.6f}f   // Arm velocity gain")

# Closed-loop poles for stability check
A_cl = A - B @ K_raw
print()
print("Closed-loop poles:")
eigs = np.linalg.eigvals(A_cl)
for e in eigs:
    print(f"  {e:.6f}")
print(f"Minimum real part: {np.min(np.real(eigs)):.6f}  "
      f"{'STABLE' if np.min(np.real(eigs)) < 0 else 'UNSTABLE'}")
