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
import numpy as np
from scipy.linalg import solve_continuous_are

print("🚀 FLYWHEEL SIMULATOR - STATE-SPACE → LQR GAIN GENERATOR")
print("=" * 70)

# 1️⃣ EXACT HARDWARE PARAMETERS FROM flywheel_sim.c
p = {
    'M_disc': 1.3872,              # Solid steel disc mass (kg)
    'R_disc': 0.075,               # Disc radius (15cm diameter -> 0.075m)
    'motor_shaft_radius': 0.005,   # Shaft radius (m)
    'motor_shaft_mass': 0.06,      # Shaft mass (kg)
    'b1': 0.002,                   # Viscous friction coefficient (Nms/rad)
    'kt': 0.12,                    # Motor torque constant (Nm/A)
    'kb': 0.12,                    # Back-EMF constant (Vs/rad)
    'Vmax': 12.0,                  # Maximum motor rails (V)
    'R': 2.5,                      # Armature winding resistance (ohms)
    'deadzone': 0.4                # Minimum voltage to crack static friction (V)
}

# 2️⃣ DYNAMIC MOMENT OF INERTIA COMPONENTS
I_shaft = 0.5 * p['motor_shaft_mass'] * p['motor_shaft_radius']**2
I_disc  = 0.5 * p['M_disc'] * p['R_disc']**2
I_tot   = I_shaft + I_disc

print(f"📊 SYSTEM MOI CALCULATION:")
print(f"   • I_shaft : {I_shaft:.7f} kg·m²")
print(f"   • I_disc  : {I_disc:.7f} kg·m²")
print(f"   • I_total : {I_tot:.7f} kg·m²")

# 3️⃣ STATE-SPACE COEFFICIENTS
# Combined damping (Mechanical drag + back-EMF electrical dampening)
alpha = (1.0 / I_tot) * (p['b1'] + (p['kt'] * p['kb']) / p['R'])
# Control input coupling coefficient
beta  = p['kt'] / (I_tot * p['R'])

# Form A and B Matrices: State Vector x = [theta, theta_dot]^T
A = np.array([
    [0.0, 1.0],
    [0.0, -alpha]
])

B = np.array([
    [0.0],
    [beta]
])

print(f"\n✅ 4️⃣ SYSTEM STATE-SPACE MATRICES:")
print(f"A =\n{np.round(A, 4)}")
print(f"B =\n{np.round(B, 4)}")

# 4️⃣ TUNING DESIRES (Q and R Matrix Configuration)
# Adjust the diagonal weights to prioritize tracking accuracy vs voltage cost
Q = np.diag([250.0, 2.5])  # Penalizes: [Position Error^2, Velocity Error^2]
R_lqr = np.array([[0.05]]) # Penalizes: [Voltage Effort^2]

print(f"\n⚙️  LQR OPTIMIZATION CRITERIA:")
print(f"   • Q diagonal: {np.diag(Q)}")
print(f"   • R scalar  : {R_lqr[0,0]}")

# 5️⃣ SOLVE CONTINUOUS ALGEBRAIC RICCATI EQUATION (CARE)
P = solve_continuous_are(A, B, Q, R_lqr)
K_raw = np.linalg.inv(R_lqr) @ B.T @ P

# 6️⃣ GAIN MAPPING & COORD MATRIX CORRECTION
# LQR evaluates u = -Kx. If your loop calculates error directly,
# we match the coordinate alignments cleanly.
k1_pos = -K_raw[0, 0]
k2_vel = -K_raw[0, 1]

# Physical direction verification (Restoring forces must resist error growth)
if k1_pos > 0: k1_pos *= -1
if k2_vel > 0: k2_vel *= -1

print(f"\n🎯 FINAL FLYWHEEL GAINS (u = k1*theta_err + k2*theta_dot_err):")
print(f"#define K1_FLYWHEEL_POS  {k1_pos:7.4f}f")
print(f"#define K2_FLYWHEEL_VEL  {k2_vel:7.4f}f")
print(f"\n📋 Copy #defines directly into your C code compensator header.")
print("=" * 70)
