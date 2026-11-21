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
flywheel_lqi_design.py
======================

First-principles LQI design for the DC-motor + steel-disc flywheel
(`models/flywheel_with_DC_motor/fixed_weight_flywheel`).

The script mirrors `LQI_DERIVATION.md`:

  1. Physical parameters and derived inertia / damping / control gain
  2. Plant matrices A, B
  3. Position LQI augmentation A_aug, B_aug
  4. Continuous-time ARE, gain K, closed-loop poles
  5. Discretization at the 1 kHz compensator rate
  6. Feedforward: actuator deadzone and speed steady-state voltage
  7. Speed LQI (textbook) and the implemented PI + feedforward comparison
  8. 20-bit encoder quantization / velocity-filter numbers
  9. C-ready #define values for both compensators

Students can edit the parameters, weights, or setpoints and re-run:

    python3 docs/flywheel_lqi_design.py

Requires: numpy, scipy
"""

import numpy as np
from scipy.linalg import solve_continuous_are, expm


# ============================================================================
# 1. PHYSICAL PARAMETERS (match simulator/flywheel_sim.c FlywheelParams)
# ============================================================================
M_disc = 1.3872          # kg, steel disc
R_disc = 0.075           # m, disc radius (15 cm diameter)
motor_shaft_mass = 0.06  # kg
motor_shaft_radius = 0.005  # m
b1 = 0.002               # N*m*s/rad, viscous friction
kt = 0.12                # N*m/A, torque constant
kb = 0.12                # V*s/rad, back-EMF constant
R = 2.5                  # ohm, armature resistance
Vmax = 12.0              # V, supply rail
deadzone = 0.4           # V, motor + driver stiction band

theta_ref_deg = 90.0     # position test target
omega_ref = 15.0         # rad/s, speed test target

print("=" * 72)
print("1. PHYSICAL PARAMETERS")
print("=" * 72)
print(f"M_disc            = {M_disc} kg")
print(f"R_disc            = {R_disc} m")
print(f"motor shaft       = {motor_shaft_mass} kg, r = {motor_shaft_radius} m")
print(f"b1                = {b1} N*m*s/rad")
print(f"kt, kb            = {kt} N*m/A, {kb} V*s/rad")
print(f"R                 = {R} ohm")
print(f"Vmax, deadzone    = {Vmax} V, {deadzone} V")

# 2. Derived inertia and first-order speed parameters
I_shaft = 0.5 * motor_shaft_mass * motor_shaft_radius**2
I_disc = 0.5 * M_disc * R_disc**2
I_tot = I_shaft + I_disc

alpha = (b1 + kt * kb / R) / I_tot     # 1/s
beta = kt / (R * I_tot)               # rad/(s^2*V)
tau_plant = 1.0 / alpha               # s

print("\n" + "=" * 72)
print("2. DERIVED PLANT CONSTANTS")
print("=" * 72)
print(f"I_shaft           = {I_shaft:.12f} kg*m^2")
print(f"I_disc            = {I_disc:.12f} kg*m^2")
print(f"I_tot             = {I_tot:.12f} kg*m^2")
print(f"alpha             = {alpha:.12f} 1/s")
print(f"beta              = {beta:.12f} rad/(s^2*V)")
print(f"plant time const  = {tau_plant:.9f} s")
print(f"speed DC gain     = {beta/alpha:.9f} rad/s/V")
print(f"no-load speed @12V= {(Vmax - deadzone)/(alpha/beta):.9f} rad/s")

# ============================================================================
# 3. PLANT MATRICES
# ============================================================================
# Position state x = [theta, omega]^T, input u = V_effective
A = np.array([
    [0.0,  1.0],
    [0.0, -alpha],
])
B = np.array([
    [0.0],
    [beta],
])
C_out = np.array([[1.0, 0.0]])   # measured position

print("\n" + "=" * 72)
print("3. PLANT MATRICES (x = [theta, omega], u = V_effective)")
print("=" * 72)
print("\nA =")
print(A)
print("\nB =")
print(B)
print("\nC_out =")
print(C_out)
print("\nPosition plant: G(s) = beta / (s*(s + alpha))")
print(f"              = {beta:.6f} / (s*(s + {alpha:.6f}))")
print("Speed plant:    G_v(s) = beta / (s + alpha)")


# ============================================================================
# 4. POSITION LQI
# ============================================================================
def lqi_design(A_aug, B_aug, Q_aug, R_lqr):
    """Solve the continuous ARE and return P, K."""
    P = solve_continuous_are(A_aug, B_aug, Q_aug, R_lqr)
    K = np.linalg.inv(R_lqr) @ B_aug.T @ P
    return P, K


def discrete_poles(A_cl, dt):
    """ZOH poles and forward-Euler poles for a continuous closed loop."""
    zoh = np.linalg.eigvals(expm(A_cl * dt))
    euler = np.linalg.eigvals(np.eye(A_cl.shape[0]) + A_cl * dt)
    return zoh, euler


# Augment with q_dot = theta - r  (furnace convention, C_out = [1, 0])
A_aug = np.block([
    [A,              np.zeros((2, 1))],
    [C_out,          np.zeros((1, 1))],
])
B_aug = np.block([
    [B],
    [np.zeros((1, 1))],
])

# Current weights in compensator_position_calculator.py
Q_pos = np.diag([100.0, 2.5, 50.0])
R_pos = np.array([[1.0]])

P_pos, K_pos = lqi_design(A_aug, B_aug, Q_pos, R_pos)
K_pos = K_pos[0]   # SISO row vector

A_cl_pos = A_aug - B_aug @ K_pos.reshape(1, -1)
poles_pos = np.linalg.eigvals(A_cl_pos)
zoh_pos, euler_pos = discrete_poles(A_cl_pos, 0.001)

print("\n" + "=" * 72)
print("4. POSITION LQI (3 states: theta, omega, integral of theta - r)")
print("=" * 72)
print("\nA_aug =")
print(A_aug)
print("\nB_aug =")
print(B_aug)
print("\nQ_aug = diag(100, 2.5, 50),  R = 1")
print("\nP (ARE solution) =")
print(np.round(P_pos, 6))
print("\nK = [K_theta, K_omega, K_q] =")
print(np.round(K_pos, 9))
print("\nClosed-loop poles (continuous, 1/s):")
print(np.round(poles_pos, 6))
print("\nClosed-loop poles (ZOH at 1 kHz, z-plane):")
print(np.round(zoh_pos, 9))
print("\nClosed-loop poles (forward Euler at 1 kHz, z-plane):")
print(np.round(euler_pos, 9))
print("\nC mapping: u = K_POS*(r - theta) + K_VEL*(-omega) + K_INT*integral(r - theta)")
print(f"#define K_POS  {K_pos[0]:.6f}f")
print(f"#define K_VEL  {K_pos[1]:.6f}f")
print(f"#define K_INT  {K_pos[2]:.6f}f")

# Position design with the 0.8/0.2 velocity filter included.
# States: [theta, omega, q, d_filt]
wf = -np.log(0.8) / 0.001          # continuous-equivalent filter pole, 1/s
Kp, Kv, Ki = K_pos
A_pos_filt = np.array([
    [0.0,             1.0,   0.0,      0.0],
    [-beta * Kp,      -alpha, beta * Ki, -beta * Kv],
    [-1.0,            0.0,   0.0,      0.0],
    [0.0,             wf,    0.0,      -wf],
])
poles_pos_filt = np.linalg.eigvals(A_pos_filt)
zoh_pos_filt, _ = discrete_poles(A_pos_filt, 0.001)

print("\nPosition design with velocity filter d_filt = 0.8*d_filt + 0.2*d_raw")
print(f"  continuous-equivalent filter pole wf = {wf:.6f} 1/s")
print("  poles (continuous, 1/s):")
print(np.round(poles_pos_filt, 6))
print("  poles (ZOH at 1 kHz, z-plane):")
print(np.round(zoh_pos_filt, 9))


# ============================================================================
# 5. POSITION WEIGHT COMPARISON
# ============================================================================
print("\n" + "=" * 72)
print("5. POSITION WEIGHT COMPARISON")
print("=" * 72)

candidates = [
    ("aggressive (original)", np.diag([250.0, 2.5, 100.0]), 0.05),
    ("current",               np.diag([100.0, 2.5, 50.0]),  1.00),
    ("Bryson tight",          np.diag([131.31, 0.04, 131.31]), 0.006944),
]

def fmt_pole(p):
    """Print a real pole as a real number, a complex one as a+bi."""
    if abs(p.imag) < 1e-9:
        return f"{p.real:.4f}"
    return f"{p.real:.4f}{p.imag:+.4f}j"


print(f"{'design':<24}{'K_POS':>10}{'K_VEL':>10}{'K_INT':>10}"
      f"  {'pole 1':>18}{'pole 2':>18}{'pole 3':>18}")
for name, Qc, Rc in candidates:
    Pc, Kc = lqi_design(A_aug, B_aug, Qc, np.array([[Rc]]))
    Kc = Kc[0]
    pc = np.linalg.eigvals(A_aug - B_aug @ Kc.reshape(1, -1))
    pc = np.sort_complex(pc)
    print(f"{name:<24}{Kc[0]:>10.4f}{Kc[1]:>10.4f}{Kc[2]:>10.4f}"
          f"  {fmt_pole(pc[0]):>18}{fmt_pole(pc[1]):>18}{fmt_pole(pc[2]):>18}")

# Verify the existing calculator convention (bottom row -1, K_INT = -K[2])
A_aug_alt = np.array([
    [0.0,  1.0,   0.0],
    [0.0, -alpha, 0.0],
    [-1.0, 0.0,   0.0],
])
B_aug_alt = np.array([[0.0], [beta], [0.0]])
P_alt, K_alt = lqi_design(A_aug_alt, B_aug_alt, Q_pos, R_pos)
K_alt = K_alt[0]
print("\nExisting calculator convention (q_dot = -theta, K_INT = -K[2]):")
print(f"  K_raw = [{K_alt[0]:.9f}, {K_alt[1]:.9f}, {K_alt[2]:.9f}]")
print(f"  mapped = [{K_alt[0]:.9f}, {K_alt[1]:.9f}, {-K_alt[2]:.9f}]")
print("  (same physical gains as the furnace convention above)")


# ============================================================================
# 6. FEEDFORWARD
# ============================================================================
V_ff_speed = (b1 * R / kt + kb) * omega_ref + deadzone
V_eff_speed = V_ff_speed - deadzone
back_emf = kb * omega_ref
current = (V_eff_speed - back_emf) / R
torque = kt * current
friction = b1 * omega_ref

print("\n" + "=" * 72)
print("6. FEEDFORWARD")
print("=" * 72)
print("\nPosition: no gravity, no spring. Holding an angle needs zero torque.")
print(f"  deadzone pre-compensation: V_cmd = u + sign(u)*{deadzone} V")
print("  plant then computes V_eff = V_cmd - sign(V_cmd)*deadzone = u")
print("\nSpeed steady state: 0 = -alpha*omega_ref + beta*V_eff")
print(f"  V_eff = alpha*omega_ref/beta = (b1*R/kt + kb)*omega_ref")
print(f"        = ({b1*R/kt:.6f} + {kb:.6f})*{omega_ref} = {V_eff_speed:.6f} V")
print(f"  V_FF  = V_eff + deadzone = {V_ff_speed:.6f} V")
print("\nPhysical check at 15 rad/s:")
print(f"  back-EMF          = {back_emf:.6f} V")
print(f"  armature current  = {current:.6f} A")
print(f"  motor torque      = {torque:.6f} N*m")
print(f"  viscous friction  = {friction:.6f} N*m")
print(f"  net torque        = {torque - friction:.9f} N*m")
print(f"  no-load speed at 12 V requested = {(Vmax - deadzone)/(alpha/beta):.6f} rad/s")


# ============================================================================
# 7. SPEED: TEXTBOOK LQI vs IMPLEMENTED PI + FEEDFORWARD
# ============================================================================
print("\n" + "=" * 72)
print("7. SPEED CONTROL")
print("=" * 72)

# Textbook speed LQI: x = [omega, integral(omega)] with ref = 0 regulator
A_s = np.array([
    [-alpha, 0.0],
    [1.0,    0.0],
])
B_s = np.array([[beta], [0.0]])
Q_s = np.diag([100.0, 50.0])
R_s = np.array([[1.0]])

P_s, K_s = lqi_design(A_s, B_s, Q_s, R_s)
K_s = K_s[0]
A_cl_s = A_s - B_s @ K_s.reshape(1, -1)
poles_s = np.linalg.eigvals(A_cl_s)
zoh_s, _ = discrete_poles(A_cl_s, 0.001)

print("\nTextbook speed LQI: Q = diag(100, 50), R = 1")
print("A_aug =")
print(A_s)
print("B_aug =")
print(B_s)
print(f"K = [{K_s[0]:.9f}, {K_s[1]:.9f}]")
print(f"C mapping: K2_SPEED = {-K_s[0]:.6f}, KI_SPEED = {-K_s[1]:.6f}")
print("poles (continuous, 1/s):", np.round(poles_s, 6))
print("poles (ZOH at 1 kHz, z-plane):", np.round(zoh_s, 9))
print(f"#define K2_SPEED  {-K_s[0]:.6f}f")
print(f"#define KI_SPEED  {-K_s[1]:.6f}f")

# Implemented PI + feedforward
K2_impl = -1.0
KI_impl = -0.05
A_pi = np.array([
    [-alpha + beta * K2_impl, beta * KI_impl],
    [1.0,                     0.0],
])
poles_pi = np.linalg.eigvals(A_pi)
zoh_pi, _ = discrete_poles(A_pi, 0.001)

print(f"\nImplemented PI + feedforward: K2 = -1.0, KI = -0.05, V_FF = {V_ff_speed:.4f} V")
print("A_pi = [[-alpha + beta*K2, beta*KI], [1, 0]] =")
print(np.round(A_pi, 6))
print("poles (continuous, 1/s):", np.round(poles_pi, 6))
print("poles (ZOH at 1 kHz, z-plane):", np.round(zoh_pi, 9))
print(f"slow pole time constant = {1/abs(min(abs(poles_pi.real))):.6f} s")

# PI with the 0.8/0.2 velocity filter. States: [omega, q, d_filt]
A_pi_filt = np.array([
    [-alpha, beta * KI_impl, beta * K2_impl],
    [0.0,    0.0,            1.0],
    [wf,     0.0,            -wf],
])
poles_pi_filt = np.linalg.eigvals(A_pi_filt)
zoh_pi_filt, _ = discrete_poles(A_pi_filt, 0.001)

print("\nImplemented PI with velocity filter (states: omega, q, d_filt):")
print("A =")
print(np.round(A_pi_filt, 6))
print("poles (continuous, 1/s):", np.round(poles_pi_filt, 6))
print("poles (ZOH at 1 kHz, z-plane):", np.round(zoh_pi_filt, 9))


# ============================================================================
# 8. 20-BIT MEASUREMENT CHAIN
# ============================================================================
CPR = 1048576                 # 2^20 counts/rev
DEG_PER_COUNT = 360.0 / CPR
RAD_PER_COUNT = 2.0 * np.pi / CPR
DT = 0.001
VEL_QUANTUM = RAD_PER_COUNT / DT
VEL_NOISE_RMS = VEL_QUANTUM / np.sqrt(12.0)
FILTER_NOISE_GAIN = 0.2 / np.sqrt(1.0 - 0.8**2)

print("\n" + "=" * 72)
print("8. 20-BIT MEASUREMENT CHAIN")
print("=" * 72)
print(f"counts/rev          = {CPR}")
print(f"degrees/count       = {DEG_PER_COUNT:.12f}")
print(f"radians/count       = {RAD_PER_COUNT:.12e}")
print(f"velocity quantum    = {VEL_QUANTUM:.9f} rad/s per count")
print(f"velocity noise RMS  = {VEL_NOISE_RMS:.9f} rad/s (uniform quantization)")
print(f"after 0.8/0.2 filter= {VEL_NOISE_RMS * FILTER_NOISE_GAIN:.9f} rad/s RMS")
print(f"counts/ms at 15     = {omega_ref * DT / RAD_PER_COUNT:.3f}")
print(f"counts/ms at 40.465 = {(Vmax-deadzone)/(alpha/beta) * DT / RAD_PER_COUNT:.3f}")

# Exact phase lag of the discrete 0.8/0.2 filter
def filter_phase_deg(w):
    z = np.exp(-1j * w * DT)
    H = 0.2 / (1.0 - 0.8 * z)
    return -np.degrees(np.angle(H))

print("\n0.8/0.2 filter phase lag (exact discrete):")
for w in [0.039, 0.712, 6.558, 15.788, 18.633, 123.054]:
    print(f"  {w:8.3f} rad/s -> {filter_phase_deg(w):7.3f} deg")


# ============================================================================
# 9. C-READY #define SUMMARY
# ============================================================================
print("\n" + "=" * 72)
print("9. C-READY #define SUMMARY")
print("=" * 72)
print("\n--- compensator/position_control/compensator.c ---")
print(f"#define K_POS  {K_pos[0]:.4f}f")
print(f"#define K_VEL  {K_pos[1]:.4f}f")
print(f"#define K_INT  {K_pos[2]:.4f}f")
print("\n--- compensator/speed_control/compensator.c ---")
print(f"#define OMEGA_REF_RAD_S  {omega_ref:.1f}f")
print(f"#define K2_SPEED  {K2_impl:.4f}f")
print(f"#define KI_SPEED  {KI_impl:.4f}f")
print(f"#define V_FF      {V_ff_speed:.4f}f")


# ============================================================================
# 10. TIME-DOMAIN CHECK (1 kHz Euler mirror of the C control logic)
# ============================================================================
# This is not the 20 kHz RK4 simulator; it is a fast check that the gains,
# signs, deadzone, and integral update reproduce the measured behavior.

def sim_position(T=60.0, target_deg=90.0):
    target = np.deg2rad(target_deg)
    th = 0.0
    om = 0.0
    q = 0.0
    d_filt = 0.0
    th_max = th
    t_max = 0.0
    rail = 0
    n = int(T / DT)
    marks = {}
    for k in range(n):
        d_filt = 0.8 * d_filt + 0.2 * om
        e = target - th
        e = (e + np.pi) % (2.0 * np.pi) - np.pi
        q += e * DT
        q = np.clip(q, -2.0, 2.0)
        u = Kp * e + Kv * (-d_filt) + Ki * q
        if abs(u) > 0.001:
            u += deadzone if u > 0.0 else -deadzone
        u = np.clip(u, -Vmax, Vmax)
        if abs(u) >= Vmax - 1e-12:
            rail += 1
        v_eff = u - deadzone if u > deadzone else (u + deadzone if u < -deadzone else 0.0)
        om += (-alpha * om + beta * v_eff) * DT
        th += om * DT
        if th > th_max:
            th_max = th
            t_max = (k + 1) * DT
        t = (k + 1) * DT
        for mark in (1, 2, 3, 4, 5, 8, 10, 12, 15, 20, 30, 40, 50, 60):
            if abs(t - mark) < 1e-12:
                marks[mark] = (th, om, q)
    return marks, th_max, t_max, rail / n


def sim_speed(T=300.0, target=15.0):
    om = 0.0
    q = 0.0
    d_filt = 0.0
    rail = 0
    n = int(T / DT)
    marks = {}
    for k in range(n):
        d_filt = 0.8 * d_filt + 0.2 * om
        e = d_filt - target
        u_unsat = K2_impl * e + KI_impl * q + V_ff_speed
        if e > 0.0 and u_unsat < Vmax:
            q += e * DT
        elif e < 0.0 and u_unsat > -Vmax:
            q += e * DT
        q = np.clip(q, -2.0, 2.0)
        u = K2_impl * e + KI_impl * q + V_ff_speed
        u = np.clip(u, -Vmax, Vmax)
        if abs(u) >= Vmax - 1e-12:
            rail += 1
        v_eff = u - deadzone if u > deadzone else (u + deadzone if u < -deadzone else 0.0)
        om += (-alpha * om + beta * v_eff) * DT
        t = (k + 1) * DT
        for mark in (1, 2, 5, 10, 20, 30, 40, 50, 60, 100, 150, 200, 250, 300):
            if abs(t - mark) < 1e-12:
                marks[mark] = (om, q)
    return marks, rail / n


print("\n" + "=" * 72)
print("10. TIME-DOMAIN CHECK (1 kHz Euler, exact C logic)")
print("=" * 72)
pos_marks, th_max, t_max, pos_rail = sim_position()
print(f"\nPosition step to {theta_ref_deg:.1f} deg:")
for t in sorted(pos_marks):
    th, om, q = pos_marks[t]
    print(f"  t={t:3d}s theta={np.degrees(th):10.6f} deg omega={om:+.6f} q={q:+.6f}")
print(f"  peak={np.degrees(th_max):.6f} deg at {t_max:.3f}s, "
      f"overshoot={np.degrees(th_max - np.deg2rad(theta_ref_deg)):.6f} deg, "
      f"rail={100*pos_rail:.3f}%")

spd_marks, spd_rail = sim_speed()
print("\nSpeed step to 15 rad/s:")
for t in sorted(spd_marks):
    om, q = spd_marks[t]
    print(f"  t={t:3d}s omega={om:.6f} rad/s q={q:+.6f}")
print(f"  rail={100*spd_rail:.3f}%")

print("\nDone.")