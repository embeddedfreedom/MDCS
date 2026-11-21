Copyright (C) 2026 Diptopal Basu (embeddedfreedom)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

# Analytical Derivation and Control of the Inverted Pendulum on a DC Motor

**Subtitle:** State-Space Linearization via Jacobian Method and LQR Implementation  
**Author:** Diptopal Basu  
**Date:** February 2026  
**Version:** 1.0  
**Tags:** [Control Theory, Robotics, Inverted Pendulum, LQR, Jacobian Linearization]

---

## Executive Summary & Configuration

This document provides a rigorous mathematical derivation of the Inverted Pendulum dynamics — specifically a pendulum mounted directly to a DC motor shaft — using Lagrangian Mechanics. The derivation accounts for both the mechanical inertia of the system and the electrical characteristics of the DC motor. It concludes with the linearization of the non-linear equations into a State-Space model ($Ax + Bu$) and provides the Python implementation for LQR gain calculation to achieve upright stability.

---

## 0. System Configuration & Coordinate Anchor

These parameters form the basis of the numerical matrices below.

| Category                    | Parameter      | Value                |
| --------------------------- | -------------- | -------------------- |
| Pendulum Mass               | $M_1$          | **0.2 kg**           |
| Pendulum Length             | $L_1$          | **0.3 m**            |
| Combined System Inertia     | $I_{tot}$      | **0.00600575 kg·m²** |
| Mechanical Friction         | $b_1$          | **0.008 Nms/rad**    |
| Torque / Back-EMF Constants | $k_t / k_b$    | **0.12**             |
| Terminal Resistance         | $R$            | **2.5 $\Omega$**     |
| Gravitational Acceleration  | $g$            | **9.81 m/s²**        |
| Minimum Motor Voltage       | $V_{deadzone}$ | **0.4 V**            |

---

## Crucial Derivation Notes for the Student

1. **The $\pi$ Offset:** Because we linearize around the upright position, the "Pendulum Position" variable in our LQR code is always the error term: $\Delta\theta_2 = (\theta_2 - \pi)$.

2. **Motor Back-EMF:** The parameter $\sigma_1$ in the $A$ matrix is not just $b_1$. It is the "Combined Damping" that includes both mechanical friction and electrical resistance: $\sigma_1 = -(b_1 + \frac{K_t K_b}{R})$.

3. **The Deadzone:** While the LQR math assumes a perfect linear motor, the code must compensate for the **0.4V deadzone** before sending the final PWM signal to the motor driver.

---

## 1. Kinematics and Lagrangian

### 1. Kinetic Energy ($T$):

$T = \frac{1}{2} I_{tot} \dot{\theta}^2$

### 2. Potential Energy ($V$):

Using the convention where $0$ is down and $\pi$ is up:

$V = M_1 g \frac{L_1}{2} \cos(\theta)$

### 3. The Lagrangian ($L = T - V$):

$L = \frac{1}{2} I_{tot} \dot{\theta}^2 - \frac{M_1 g L_1}{2} \cos(\theta)$

### 4. The Equation of Motion:

Applying $\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) - \frac{\partial L}{\partial \theta} = \tau_{motor} - \text{friction}$:

$I_{tot} \ddot{\theta} - \frac{M_1 g L_1}{2} \sin(\theta) = \frac{k_t}{R}(V_{in} - k_b \dot{\theta}) - b_1 \dot{\theta}$

---

## 2. Jacobian Linearization

### 2.1 Define the Nonlinear State Functions:

Let $x_1 = \theta$ and $x_2 = \dot{\theta}$.

$\dot{x}_1 = f_1(x, u) = x_2$

$\dot{x}_2 = f_2(x, u) = \frac{M_1 g L_1}{2 I_{tot}} \sin(x_1) - \frac{1}{I_{tot}}(b_1 + \frac{k_t k_b}{R})x_2 + \frac{k_t}{R I_{tot}} V_{in}$

### 2.2 Compute the Jacobians at $x_0 = [\pi, 0]^T$:

**For Matrix A:**

$A = \begin{bmatrix} \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} \\ \frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ \frac{M_1 g L_1}{2 I_{tot}} \cos(\pi) & -\frac{1}{I_{tot}}(b_1 + \frac{k_t k_b}{R}) \end{bmatrix}$

*Since $\cos(\pi) = -1$ and we define the error as $\Delta \theta = \theta - \pi$, the sign flips to positive for the restoring force calculation.*

**For Matrix B:**

$B = \begin{bmatrix} \frac{\partial f_1}{\partial u} \\ \frac{\partial f_2}{\partial u} \end{bmatrix} = \begin{bmatrix} 0 \\ \frac{k_t}{R I_{tot}} \end{bmatrix}$

### 2.3 Numerical Result:

$A = \begin{bmatrix} 0 & 1 \\ 49.003 & -2.291 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 7.992 \end{bmatrix}$

---

## 3. LQR and the Riccati Equation

### 3.1 LQR finds the "optimal" balance between system performance and energy usage.

It solves the Algebraic Riccati Equation (ARE) to minimize total cost:

$A^T P + P A - P B R^{-1} B^T P + Q = 0$

Solving for $P$ gives the energy surface of the system. The optimal gains are $K = R^{-1} B^T P$.

### 3.2 Penalty Selection (The Q Matrix):

**Justification:** LQR finds the "optimal" balance between system performance and energy usage. It solves the Algebraic Riccati Equation (ARE) to minimize total cost:

**3.1 Penalty Selection (The Q Matrix):**

**Justification:** LQR finds the "optimal" balance between system performance and energy usage. It solves the Algebraic Riccati Equation (ARE) to minimize total cost:

**3.2 Penalty Selection (The Q Matrix):**

$Q = \begin{bmatrix} 1010 & 0 \\ 0 & 15.7 \end{bmatrix}, \quad R = [0.1]$

| **Parameter**         | **Description** | **Ideal Value** | **Implementation** | **Logic**                       |
| --------------------- | --------------- | --------------- | ------------------ | ------------------------------- |
| **Pendulum Position** | $k_1$           | $\approx -106$  | $-220.0$           | **Negative** (Restoring Torque) |
| **Pendulum Velocity** | $k_2$           | $\approx -13$   | $-26.0$            | **Negative** (Damping Force)    |

**Ideal Selection:** $q_{11} = 1010$, $q_{22} = 15.7$  
**Implementation Selection:** $q_{11} = 4575$, $q_{22} = 64$  

**Why the difference?** The "Ideal" values come from Bryson's Rule (moderate), while the **Implementation** values are increased to dominate hardware friction and stiction. The pendulum needs higher gains because real hardware has friction, deadzone, and noise that the ideal model doesn't account for.

### 3.3 Bryson's Rule

$Q_{ii} = \frac{1}{\text{maximum acceptable value for state } i^2}$

$R = \frac{1}{\text{maximum acceptable value of } (u^2)}$

---

## 4. Integral Justification ($k_i = -75.0$)

**Why did you add $k_i$?** Because LQR alone is mathematically **"blind" to constant disturbances**.

- **The $176^\circ$ Settling Point:** Without $k_i$, gravity creates a constant torque. To counteract it, the LQR controller needs a non-zero error to produce voltage. This results in the pendulum settling slightly below the top (e.g., at $176^\circ$). This is the **steady-state error**.

- **The Integrator's Job:** $k_i$ sums the error over time. As long as the pendulum is even $0.001$ rad away from $\pi$, the sum grows. Eventually, it provides enough voltage to overcome the **0.4V deadzone** and push the pendulum to exactly $180.000^\circ$.

- **Settling without Integrator:** Without the integrator, the system settles at the point where $k_1 \times \text{Error} = \text{Gravity Torque}$. With your mass, that offset is several degrees.

---

## 5. Final Control Law

$\text{Output Voltage }(V) = \underbrace{k_1(\theta - \pi) + k_2(\dot{\theta})}_{\text{LQR Control}} + \underbrace{k_i \int (\theta - \pi) dt}_{\text{Precision Integral}} + \text{Deadzone}$

---

## 6. Numerical Implementation (Python)

The LQR controller finds the optimal gain matrix $K$ by minimizing a cost function that balances error ($Q$) and control effort ($R$). The core of this optimization is solving the **Algebraic Riccati Equation**:

$A^T P + P A - P B R^{-1} B^T P + Q = 0$

In the provided Python script, the line `solve_continuous_are(A, B, Q, R_lqr)` performs this calculation. It computes the unique positive-definite matrix **$P$**, which is then used to find the raw gains:

$K_{raw} = R^{-1} B^T P$

This script handles the mapping to the C code implementation correctly. It takes the raw output $K_{raw}$ and applies the negation logic ($k = -K$) so that the final **Pendulum gains** ($k_1$ for position and $k_2$ for velocity) result in the negative values used in the compensator. This ensures that a positive angular error produces a negative corrective voltage to restore the pendulum to the upright position.

```python
import numpy as np
from scipy.linalg import solve_continuous_are

def compute_lqr():
    # Parameters from Table
    M1, L1, I_tot = 0.2, 0.3, 0.00600575
    g, b1, kt, kb, R = 9.81, 0.008, 0.12, 0.12, 2.5

    # Jacobian Matrices
    A = np.array([[0, 1], [(M1*g*L1)/(2*I_tot), -(b1 + (kt*kb/R))/I_tot]])
    B = np.array([[0], [(kt/R)/I_tot]])

    # LQR Weights
    Q = np.diag([4575, 64])
    R_mat = np.array([[0.1]])

    # Solve Riccati
    P = solve_continuous_are(A, B, Q, R_mat)
    K = np.linalg.inv(R_mat) @ B.T @ P
    print(f"LQR Gains: k1={-K[0,0]:.4f}, k2={-K[0,1]:.4f}")


```

### 6.0 Code-to-Equation Mapping

The tables below map every line of the implementation back to the derivation, in execution order. Function names are given instead of line numbers so the mapping survives code edits.

#### 6.0.1 `simulator/pendulum_sim.c` — plant physics (Sections 1–2)

| Derivation              | Equation                                                                                       | C code                                                                                                                                                                   |
| ----------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Requested motor voltage | $V_{req} = u\,V_{max}\,\mathrm{sgn}(dir)$                                                      | `float V_requested = motor_pwm * p->Vmax * (float)motor_dir;`                                                                                                            |
| Deadzone                | $V_{eff} = V_{req} - \mathrm{sgn}(V_{req})V_{dz}$ if $\lvert V_{req}\rvert > V_{dz}$, else $0$ | `if (fabsf(V_requested) > p->deadzone) {`<br>`    V_effective = (V_requested > 0) ? (V_requested - p->deadzone) : (V_requested + p->deadzone);`<br>`}`                   |
| Motor torque            | $\tau_m = \frac{K_t}{R}\left(V_{eff} - K_b\dot{\theta}\right)$                                 | `tau_motor = (p->kt / p->R) * (V_effective - p->kb * d);`                                                                                                                |
| Coasting at zero PWM    | $\tau_m = 0$ when $V_{eff} = 0$                                                                | `if (fabsf(V_effective) < 0.001f && p->motor_coast_on_zero) {`<br>`    tau_motor = 0.0f;`<br>`}`                                                                         |
| Shaft inertia           | $I_{sh} = \frac{1}{2} m_{sh} r_{sh}^2$                                                         | `float I_shaft = 0.5f * p->motor_shaft_mass * p->motor_shaft_radius * p->motor_shaft_radius;`                                                                            |
| Pendulum inertia        | $I_{pend} = \frac{1}{3}M_1 L_1^2 + \frac{1}{4}M_1 r_3^2$                                       | `float I_pend = (1.0f/3.0f) * p->M1 * p->L1 * p->L1 + 0.25f * p->M1 * p->r3 * p->r3;`                                                                                    |
| Total inertia           | $I_{tot} = I_{sh} + I_{pend}$                                                                  | `float I_tot = I_shaft + I_pend;`                                                                                                                                        |
| Equation of motion      | $\ddot{\theta} = \frac{\tau_m - b_1\dot{\theta} - \frac{M_1 g L_1}{2}\sin\theta}{I_{tot}}$     | `dx->theta_dot = (tau_motor - p->b1 * d - (p->M1 * p->g * p->L1 / 2.0f) * sinf(t)) / I_tot;`                                                                             |
| RK4 integration         | $x_{n+1} = x_n + \frac{\Delta t}{6}\left(k_1 + 2k_2 + 2k_3 + k_4\right)$                       | `x->theta += (dt/6.0f)*(k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);`<br>`x->theta_dot += (dt/6.0f)*(k1.theta_dot + 2*k2.theta_dot + 2*k3.theta_dot + k4.theta_dot);` |
| Angle wrapping          | $\theta \in (-\pi, \pi]$                                                                       | `normalize_angle(&x->theta);`                                                                                                                                            |

#### 6.0.2 `compensator/compensator.c` — LQR + integral controller (Sections 3–5)

| Derivation                | Equation                                                                               | C code                                                                                                                              |
| ------------------------- | -------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| Encoder decoding          | accumulate 20 kHz quadrature samples into the 1 kHz state                              | `for (int i = 0; i < bus->n_pend; i++) { decoder_process(&dec_pend, bus->pend_states[i]); }`                                        |
| Z-pulse drift correction  | snap count to nearest full revolution                                                  | `if (bus->z_pend) {`<br>`    dec_pend.count = (int)(roundf((float)dec_pend.count / 4096.0f) * 4096.0f);`<br>`}`                     |
| Counts to radians         | $\theta = \text{count}\cdot\frac{2\pi}{4096}$                                          | `const float COUNT_TO_RAD = (2.0f * PI / 4096.0f);`<br>`float t = (float)dec_pend.count * COUNT_TO_RAD;`                            |
| Numerical velocity        | $\dot{\theta}_{raw} = \frac{\theta_k - \theta_{k-1}}{\Delta t}$ with $\Delta t = 1$ ms | `float d_raw = (t - t_prev) / 0.001f;`                                                                                              |
| Velocity low-pass filter  | $\dot{\theta}_f \leftarrow 0.8\dot{\theta}_f + 0.2\dot{\theta}_{raw}$                  | `d_filt = (0.80f * d_filt) + (0.20f * d_raw);`                                                                                      |
| Error relative to upright | $e = \theta - \pi$                                                                     | `float angle_err = t - PI;`                                                                                                         |
| Angle wrapping            | $e \leftarrow e \bmod 2\pi$                                                            | `while (angle_err >  PI) angle_err -= 2.0f * PI;`<br>`while (angle_err < -PI) angle_err += 2.0f * PI;`                              |
| Catch condition           | balance when $\lvert e\rvert < 0.5$ rad and $\lvert\dot{\theta}_f\rvert < 3$ rad/s     | `bool in_catch_zone = (fabsf(angle_err) < 0.50f);`<br>`bool is_slow = (fabsf(d_filt) < 3.0f);`<br>`if (in_catch_zone && is_slow) {` |
| LQR gains                 | $k_1, k_2$                                                                             | `float k1 = -220.0f;`<br>`float k2 = -26.0f;`                                                                             |
| Integral gain             | $k_i$                                                                                  | `float ki = -75.0f;`                                                                                                                |
| Integral state            | $q \leftarrow q + e\,\Delta t$                                                         | `error_sum += angle_err * 0.001f;`                                                                                                  |
| Anti-windup clamp         | $q \in [-2, 2]$                                                                        | `if (error_sum > 2.0f) error_sum = 2.0f;`<br>`if (error_sum < -2.0f) error_sum = -2.0f;`                                            |
| Control law               | $u = k_1 e + k_2\dot{\theta}_f + k_i q$                                                | `u = (k1 * angle_err) + (k2 * d_filt) + (ki * error_sum);`                                                                          |
| Deadzone compensation     | $u \leftarrow u + \mathrm{sgn}(u)V_{dz}$ when $\lvert e\rvert > 0.001$                 | `if (fabsf(angle_err) > 0.001f) {`<br>`    u += (u > 0) ? p->deadzone : -p->deadzone;`<br>`}`                                       |
| Voltage saturation        | $u \in [-V_{max}, V_{max}]$                                                            | `if (u > p->Vmax) u = p->Vmax;`<br>`if (u < -p->Vmax) u = -p->Vmax;`                                                                |
| Direction and duty cycle  | $dir = \mathrm{sgn}(u)$, $\text{PWM} = \frac{\lvert u\rvert}{V_{max}}$                 | `*motor_dir = (u >= 0) ? 1 : -1;`<br>`return fabsf(u) / p->Vmax;`                                                                   |

#### 6.0.3 Energy-Pump Swing-Up Theory

The swing-up branch is not part of the LQR derivation. It is an energy-pump controller that adds energy only in the bottom half of the swing:

**Swing-up logic:**
1. **Bottom half** ($\cos\theta > 0$): pump energy by driving full voltage in the direction of motion:
   - $u = +V_{max}$ when $\dot{\theta} \ge 0$
   - $u = -V_{max}$ when $\dot{\theta} < 0$
2. **Top half** ($\cos\theta \le 0$): coast ($u = 0$) to allow the LQR to catch the pendulum cleanly
3. **Kickstart**: if perfectly still at the bottom ($\theta \approx 0$, $\dot{\theta} \approx 0$), apply full voltage to break static stiction

This prevents propeller behavior by never adding energy in the top half, where it would cause overshoot past upright.

| Heuristic | Purpose | C code |
|-----------|---------|--------|
| Clear integrator | reset $q$ outside the catch zone | `error_sum = 0;` |
| Energy pump bottom half | add energy in bottom half by driving full voltage in direction of motion | `if (cosf(t) > 0) {`<br>`    u = (d_filt >= 0) ? p->Vmax : -p->Vmax;`<br>`} else {`<br>`    u = 0;`<br>`}` |
| Coast top half | no input in top half to allow clean LQR catch | `u = 0;` |
| Kickstart from rest | nudge when hanging still at the bottom | `if (fabsf(fmodf(t, 2*PI)) < 0.1f && fabsf(d_filt) < 0.1f) {`<br>`    u = p->Vmax;`<br>`}` |
| Voltage saturation | clamp to motor rails | `if (u >  p->Vmax) u =  p->Vmax;`<br>`if (u < -p->Vmax) u = -p->Vmax;` |

### 7.1 Final LQR Results

The table below summarizes the gains derived from the LQR optimization. The **"Ideal"** values represent standard mathematical weights, while the **"Implementation"** values reflect the high-stiffness tuning required to dominate hardware friction and stiction.

| **State Variable**    | **Symbol** | **Ideal Gain** | **Implementation** | **Logic**                       |
| --------------------- | ---------- | -------------- | ------------------ | ------------------------------- |
| **Pendulum Position** | $k_1$      | $\approx -220$ | **$-220.0$**  | **Negative** (Restoring Torque) |
| **Pendulum Velocity** | $k_2$      | $\approx -26$  | **$-26.0$**   | **Negative** (Damping Force)    |
| **Integral**          | $k_i$      | $\approx -75$  | **$-75.0$**        | **Anti-windup clamp**           |

**Ideal Selection:** $q_{11} = 4575$, $q_{22} = 64$  
**Implementation Selection:** $q_{11} = 4575$, $q_{22} = 64$  

**Why the difference?** The "Ideal" values come from Bryson's Rule (moderate), while the **Implementation** values are increased to dominate hardware friction and stiction. The pendulum needs higher gains because real hardware has friction, deadzone, and noise that the ideal model doesn't account for.

---

## 8. Limitations and Future Work

### 8.1 The Impact of Noise and Drift

It is important to observe that in a purely theoretical environment—**without sensor noise or mechanical disturbances—the LQR balance is mathematically perfect.** Under ideal conditions, the pendulum would remain vertical indefinitely with near-zero control effort once it reaches the equilibrium point.

However, the physical implementation reveals two primary challenges:

1. **Sensor Noise and Jitter:** The raw encoder data contains high-frequency noise. When we derive velocity ($\dot{\theta}$) from these readings, the noise is amplified. This forces the motor to react to **"phantom"** movements, creating the micro-vibrations heard during operation.

2. **System Drift:** Despite a high $k_1$ gain, the system may experience a slow **"drift"** over time. This is caused by thermal changes in the motor resistance, slight misalignments in the center-of-gravity, or internal encoder scaling errors.

#### The Student Challenge

Because of these real-world constraints, even this high-stiffness derivation does not fully do justice to the complexity of a perfect, silent balance. **The student is encouraged to work on this further** by focusing on:

- **Noise Rejection:** Implementing a Kalman Filter to optimally estimate the state while ignoring sensor noise.

- **Drift Compensation:** Improving the $z$-pulse synchronization logic or adding an outer-loop to track and zero out long-term drift.

- **System Identification:** Refining the $I_{tot}$ and $b_1$ parameters through empirical testing to closer match the mathematical model to the physical hardware.

**Final Note:** The implementation of $k_1 = -220.0$ and $k_i = -75.0$ provides a robust **"stiff"** balance that holds the pendulum upright despite noise, but a truly **"perfect"** balance requires the student to explore advanced filtering and non-linear modeling.

---

## 9. Recommended Learning Resources

For those interested in the deep math behind this implementation, I highly recommend:

- **[Underactuated Robotics](https://underactuated.csail.mit.edu/)** (Russ Tedrake, MIT) — Excellent for pendulum-specific dynamics.

- **[Feedback Systems](http://www.cds.caltech.edu/~murray/amwiki/index.php/Main_Page)** (Åström & Murray) — The gold standard for Jacobian and LQR theory.

- **[Steve Brunton's Control Bootcamp](https://www.youtube.com/playlist%3Flist%3DPLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085z)** — Intuitive video lectures on the Riccati equation.
