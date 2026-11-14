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
#!/usr/bin/env python3
import numpy as np
from scipy.linalg import solve_continuous_are

# 1️⃣ PHYSICAL PARAMETERS (DC Motor + Heavy Steel Disc) — matches flywheel_sim.c FlywheelParams
p = {
    'motor_shaft_mass': 0.06,        # kg
    'motor_shaft_radius': 0.005,     # m
    'M_disc': 1.3872,                # kg (Steel flywheel)
    'R_disc': 0.075,                 # m
    'b1': 0.008,                     # Nms/rad (Viscous friction coefficient)
    'R': 2.5,                        # Ohms (Armature resistance)
    'kt': 0.12,                      # Nm/A (Torque constant)
    'kb': 0.12,                      # Vs/rad (Back-EMF constant)
}

# 2️⃣ DYNAMIC MOMENT OF INERTIA COMPUTATION
I_shaft = 0.5 * p['motor_shaft_mass'] * p['motor_shaft_radius']**2
I_disc  = 0.5 * p['M_disc'] * p['R_disc']**2
I_tot   = I_shaft + I_disc

print(f"Total Moment of Inertia (I_tot): {I_tot:.7f} kg*m^2")

# 3️⃣ LINEAR SYSTEM COEFFICIENTS
# Combined damping (Mechanical drag + back-EMF electrical dampening)
alpha = (1.0 / I_tot) * (p['b1'] + (p['kt'] * p['kb']) / p['R'])
# Control input coupling coefficient
beta  = p['kt'] / (I_tot * p['R'])

# 4️⃣ AUGMENTED STATE SPACE (3x3 Matrix for Position Control)
# States: [theta, theta_dot, integral_error]^T
A_aug = np.array([
    [0.0,  1.0,    0.0],  # d(theta)/dt = theta_dot
    [0.0, -alpha,  0.0],  # d(theta_dot)/dt = -alpha*theta_dot + beta*V
    [-1.0, 0.0,    0.0]   # d(integral_error)/dt = -theta (tracking error)
])

B_aug = np.array([
    [0.0],
    [beta],
    [0.0]
])

# 5️⃣ COST TUNING MATRICES (Q and R Balance Ratio)
# Retuned for step tracking: higher R penalizes control effort to reduce rail saturation.
Q_aug = np.diag([100.0, 2.5, 50.0])  # [Position Penalty, Velocity Jitter Brake, Integral Accumulator]
R_lqr = np.array([[1.0]])            # 20x higher control-effort penalty vs original 0.05

# 6️⃣ SOLVE ALGEBRAIC RICCATI EQUATION
P = solve_continuous_are(A_aug, B_aug, Q_aug, R_lqr)

# 7️⃣ EXTRACT GAINS (C error convention: u = K_POS*error_pos + K_VEL*error_vel + K_INT*pos_integral)
K_raw = np.linalg.inv(R_lqr) @ B_aug.T @ P
K = K_raw[0]
# LQR solves u = -(k1*theta + k2*theta_dot + k3*x3); x3 = pos_integral integrates error_pos = -theta.
# Mapping to C convention: K_POS = k1, K_VEL = k2, K_INT = -k3  (so positive error -> positive drive).
k1 =  K[0]
k2 =  K[1]
k3 = -K[2]

print("\n--- OPTIMAL LQR POSITION GAINS (C-ready) ---")
print(f"K1 (Position Error Gain) : {k1:.4f}")
print(f"K2 (Velocity Brake Gain) : {k2:.4f}")
print(f"Ki (Integral Error Gain) : {k3:.4f}")
print(f"\n#define K_POS  {k1:.4f}f")
print(f"#define K_VEL  {k2:.4f}f")
print(f"#define K_INT  {k3:.4f}f")
print("\nCopy the #define lines directly into your C compensator macros.")