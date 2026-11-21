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

# ---- SIM PARAMETERS (match flywheel_sim.c FlywheelParams) ----
p = {
    'M_disc': 1.3872,
    'R_disc': 0.075,
    'motor_shaft_radius': 0.005,
    'motor_shaft_mass': 0.06,
    'b1': 0.002,
    'kt': 0.12,
    'kb': 0.12,
    'Vmax': 12.0,
    'R': 2.5,
    'deadzone': 0.4,
}

I_shaft = 0.5 * p['motor_shaft_mass'] * p['motor_shaft_radius']**2
I_disc  = 0.5 * p['M_disc'] * p['R_disc']**2
I_tot   = I_shaft + I_disc

alpha = (1.0 / I_tot) * (p['b1'] + (p['kt'] * p['kb']) / p['R'])
beta  = p['kt'] / (I_tot * p['R'])

print(f"I_tot={I_tot:.7f} kg*m^2  alpha={alpha:.6f}  beta={beta:.6f}")

# ---- AUGMENTED STATE-SPACE (speed + integral of speed error) ----
# x = [theta_dot, int_speed_err]
A_aug = np.array([
    [-alpha, 0.0],   # d(theta_dot)/dt = -alpha*theta_dot + beta*V
    [1.0,    0.0],   # d(int_speed_err)/dt = theta_dot  (ref=0 regulator)
])
B_aug = np.array([[beta], [0.0]])

# ---- LQR COST ----
# Retuned for reference tracking at OMEGA_REF_RAD_S = 15 rad/s.
# Higher R penalizes control effort to reduce rail saturation / hunting.
Q_aug = np.diag([100.0, 50.0])  # penalize speed error, then integral
R_lqr = np.array([[1.0]])       # 20x higher control-effort penalty vs original 0.05

P = solve_continuous_are(A_aug, B_aug, Q_aug, R_lqr)
K_raw = np.linalg.inv(R_lqr) @ B_aug.T @ P  # u = -(k1*theta_dot + k2*int_err)

k1 = K_raw[0, 0]   # on theta_dot
k2 = K_raw[0, 1]   # on int_speed_err

# Map to C error convention: u = K2_SPEED*speed_err + KI_SPEED*speed_error_sum
# speed_err = theta_dot - ref; for ref=0, speed_err = theta_dot
K2_SPEED = -k1
KI_SPEED = -k2

print(f"\n--- LQR SPEED GAINS (C-ready) ---")
print(f"k1 (on speed error)  = {k1:.4f}")
print(f"k2 (on integral)     = {k2:.4f}")
print(f"\n#define K2_SPEED  {K2_SPEED:.4f}f")
print(f"#define KI_SPEED  {KI_SPEED:.4f}f")
print("\nCopy the #define lines into compensator/speed_control/compensator.c")
