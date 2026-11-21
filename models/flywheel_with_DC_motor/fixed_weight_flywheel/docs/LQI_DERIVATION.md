/* 
Copyright (C) 2026 Diptopal Basu (embeddedfreedom)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

# Flywheel LQI Compensator — Full Derivation

**Target reader:** Electronics engineering graduate with limited mechanics background.  
**Goal:** Derive the flywheel controllers from first principles, ending at the C code in `compensator/position_control/compensator.c` and `compensator/speed_control/compensator.c`.

**Scope:** one physical plant (DC motor + steel disc), two control objectives:

1. **Position hold** — LQI (Linear Quadratic Integral) state feedback.
2. **Speed regulation** — PI plus feedforward, which is the first-order degenerate form of LQI.

---

## 0. Control scheme at a glance

| Profile | Plant order | Controller | Feedback states | Feedforward | Why this choice |
|---|---|---|---|---|---|
| Position | 2nd order: $\theta$, $\omega$ | LQI | position error, velocity, integral of position error | actuator deadzone inverse | The plant is Type 1, so PD can hold a step in the ideal model; the integral state rejects the constant residual disturbances (friction, deadzone mismatch, quantization) that would otherwise leave a standing offset. |
| Speed | 1st order: $\omega$ | PI + feedforward | speed error, integral of speed error | $V_{FF} = 2.825$ V | The plant is Type 0; the feedforward supplies the exact operating voltage and the integral trims the residual. For a first-order plant, LQI is structurally identical to PI — there is no second plant state to feed back. |

The rest of this document derives both, proves the claims, and maps every equation to the code.

---

## 1. What we are controlling

A DC motor drives a solid steel disc (flywheel) about a horizontal axis. There is:

- **No gravity torque** — the axis is horizontal, so the centre of mass does not rise or fall as the disc turns. The `g` field in `FlywheelParams` is unused in `compute_derivatives()`.
- **No spring** — the only restoring effects are viscous friction and the motor's back-EMF.
- **One input** — the motor voltage (through a PWM duty cycle and a direction bit).
- **Two measured quantities** — the disc angle $\theta$ (from a 20-bit quadrature encoder) and, by finite difference, the angular velocity $\omega = \dot{\theta}$.

The plant is therefore **SISO**: one voltage input, one angle output. Speed is not a separate sensor; it is the derivative of the same encoder count.

### 1.1 Conventions

- SI units throughout: radians, radians per second, volts, amperes, newton-metres, kilogram-metre squared.
- Positive voltage produces positive torque, positive angular acceleration, and increasing $\theta$ in the model's coordinate.
- The C code carries the sign in `motor_dir`: it computes a signed voltage $u$, sets `*motor_dir = sign(u)`, and returns $|u|/V_{max}$ as the duty cycle. The simulator reconstructs $V_{requested} = pwm \cdot V_{max} \cdot motor\_dir$. The label "clockwise" next to `motor_dir = 1` is a hardware wiring comment; it does not change the mathematics below. (The repository-root convention says clockwise is negative; the model only requires that the sign carried by `motor_dir` matches the physical wiring.)
- Angles are wrapped to $[-\pi, \pi]$ for telemetry and for the shortest-path position error.

---

## 2. Governing equations

### 2.1 Electrical subsystem

The armature circuit is a resistance $R$ in series with a back-EMF source:

$V_{eff} = R\,i + k_b\,\omega$

where:

- $V_{eff}$ is the voltage that actually reaches the motor after the driver deadzone (Section 2.5),
- $i$ is the armature current,
- $k_b$ is the back-EMF constant (V·s/rad).

The motor torque is proportional to current:

$\tau_m = k_t\,i$

where $k_t$ is the torque constant (N·m/A). Eliminating $i$:

$i = \frac{V_{eff} - k_b\,\omega}{R}$

$\tau_m = \frac{k_t}{R}\left(V_{eff} - k_b\,\omega\right)$

This is the key electromechanical coupling: **speed produces back-EMF, which reduces the torque available at a given voltage.** It acts exactly like an additional viscous damper.

### 2.2 Mechanical subsystem

Newton's second law for rotation:

$I_{tot}\,\dot{\omega} = \tau_m - b_1\,\omega$

where:

- $I_{tot}$ is the total moment of inertia of the shaft plus disc,
- $b_1$ is the mechanical viscous friction coefficient (N·m·s/rad).

The shaft and disc are modelled as solid cylinders:

$I_{shaft} = \frac{1}{2}m_{shaft}r_{shaft}^2$

$I_{disc} = \frac{1}{2}M_{disc}R_{disc}^2$

$I_{tot} = I_{shaft} + I_{disc}$

There is no gravitational term because the axis is horizontal. There is no stiffness term because there is no spring.

### 2.3 Combined first-order speed equation

Substitute $\tau_m$ into the mechanical equation:

$I_{tot}\,\dot{\omega} = \frac{k_t}{R}\left(V_{eff} - k_b\,\omega\right) - b_1\,\omega$

Expand and collect $\omega$:

$I_{tot}\,\dot{\omega} = \frac{k_t}{R}V_{eff} - \left(b_1 + \frac{k_t k_b}{R}\right)\omega$

Divide by $I_{tot}$:

$\boxed{\dot{\omega} = -\alpha\,\omega + \beta\,V_{eff}}$

with

$\alpha = \frac{1}{I_{tot}}\left(b_1 + \frac{k_t k_b}{R}\right), \qquad \beta = \frac{k_t}{R\,I_{tot}}$

The angle is the integral of speed:

$\dot{\theta} = \omega$

Equations (2.3) are **linear and time-invariant** in $V_{eff}$. There is no quadratic actuator nonlinearity like the furnace's $V^2/R$; the only nonlinearity is the deadzone, and Section 2.5 cancels it exactly.

### 2.4 Numerical values

Using the parameters in `simulator/flywheel_sim.c`:

| Symbol | Value | Unit | Meaning |
|---|---:|---|---|
| $M_{disc}$ | 1.3872 | kg | steel disc mass |
| $R_{disc}$ | 0.075 | m | disc radius (15 cm diameter) |
| $m_{shaft}$ | 0.06 | kg | shaft mass |
| $r_{shaft}$ | 0.005 | m | shaft radius |
| $b_1$ | 0.002 | N·m·s/rad | viscous friction (flywheel motor) |
| $k_t$ | 0.12 | N·m/A | torque constant |
| $k_b$ | 0.12 | V·s/rad | back-EMF constant |
| $R$ | 2.5 | Ω | armature resistance |
| $V_{max}$ | 12.0 | V | supply rail |
| $d_z$ | 0.4 | V | deadzone |

Derived:

$I_{shaft} = \frac{1}{2}(0.06)(0.005)^2 = 7.5\times10^{-7}\ \text{kg·m}^2$

$I_{disc} = \frac{1}{2}(1.3872)(0.075)^2 = 3.9015\times10^{-3}\ \text{kg·m}^2$

$I_{tot} = 3.90225\times10^{-3}\ \text{kg·m}^2$

$\alpha = \frac{0.002 + (0.12)(0.12)/2.5}{0.00390225} = 1.9885963\ \text{s}^{-1}$

$\beta = \frac{0.12}{(2.5)(0.00390225)} = 12.3005958\ \text{rad/(s}^2\text{·V)}$

The open-loop speed time constant is

$\tau = \frac{1}{\alpha} = 0.50286727\ \text{s}$

The speed DC gain is

$\frac{\beta}{\alpha} = 6.1855670\ \text{rad/s/V}$

At the 12 V rail, after the 0.4 V deadzone, the no-load speed is

$\omega_{max} = \frac{12 - 0.4}{0.002(2.5)/0.12 + 0.12} = 71.7525773\ \text{rad/s}$

These numbers are printed by `docs/flywheel_lqi_design.py`.

---

## 2.5 Actuator model: PWM → voltage → deadzone → torque

The controller does not command voltage directly. It commands a **PWM duty cycle** $pwm \in [0,1]$ and a direction bit $dir \in \{+1,-1\}$:

$V_{requested} = pwm \cdot V_{max} \cdot dir$

The motor and driver exhibit **stiction**: below about 0.4 V the shaft does not move. The simulator models this as a symmetric deadzone:

$V_{eff} = \begin{cases} 0, & |V_{requested}| \le d_z \\ V_{requested} - d_z, & V_{requested} > d_z \\ V_{requested} + d_z, & V_{requested} < -d_z \end{cases}$

This is a **nonlinear** input map. The compensator cancels it by pre-compensating:

$V_{requested} = u + \operatorname{sign}(u)\,d_z, \qquad |u| > 0$

Then, for $u > 0$:

$V_{eff} = (u + d_z) - d_z = u$

and for $u < 0$:

$V_{eff} = (u - d_z) + d_z = u$

So after the pre-compensation, **the plant sees exactly the linear control voltage $u$**. The deadzone is not linearized or approximated; it is inverted exactly, within the validity range $|u| > 0$ and $|u| + d_z \le V_{max}$.

The C code applies the pre-compensation only when $|u| > 0.001$ V:

```c
if (fabsf(u) > 0.001f) {
    u += (u > 0.0f) ? p->deadzone : -p->deadzone;
}
```

The 1 mV gate prevents the controller from commanding a permanent 0.4 V buzz when the loop is exactly at rest. When $|u| \le 1$ mV the motor is simply turned off; the residual is far below the resolution of the problem.

The linear design in Sections 3–10 therefore uses $u = V_{eff}$ as the input. The saturation limit on $u$ is $\pm V_{max}$ *before* the deadzone is added; in practice the simulator clamps the requested voltage and the effective voltage is at most $V_{max} - d_z = 11.6$ V.

---

## 3. Linear state-space form

### 3.1 State and input

Choose

$\mathbf{x} = \begin{bmatrix} \theta \\ \omega \end{bmatrix}, \qquad u = V_{eff}$

The output is the measured angle:

$y = \theta = \begin{bmatrix} 1 & 0 \end{bmatrix}\mathbf{x}$

### 3.2 A matrix via partial derivatives

From $\dot{\theta} = \omega$ and $\dot{\omega} = -\alpha\omega + \beta u$:

$f_1(\theta,\omega,u) = \dot{\theta} = \omega$

$f_2(\theta,\omega,u) = \dot{\omega} = -\alpha\omega + \beta u$

The A matrix is the Jacobian with respect to the states:

$\mathbf{A} = \begin{bmatrix} \frac{\partial f_1}{\partial \theta} & \frac{\partial f_1}{\partial \omega} \\ \frac{\partial f_2}{\partial \theta} & \frac{\partial f_2}{\partial \omega} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & -\alpha \end{bmatrix}$

### 3.3 B matrix via partial derivatives

$\mathbf{B} = \begin{bmatrix} \frac{\partial f_1}{\partial u} \\ \frac{\partial f_2}{\partial u} \end{bmatrix} = \begin{bmatrix} 0 \\ \beta \end{bmatrix}$

### 3.4 State-space and transfer functions

$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ 0 & -\alpha \end{bmatrix}\mathbf{x} + \begin{bmatrix} 0 \\ \beta \end{bmatrix}u$

$y = \begin{bmatrix} 1 & 0 \end{bmatrix}\mathbf{x}$

Numerically:

$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & -1.9885963 \end{bmatrix}, \qquad \mathbf{B} = \begin{bmatrix} 0 \\ 12.3005958 \end{bmatrix}$

The position transfer function is

$G_\theta(s) = \mathbf{C}(s\mathbf{I}-\mathbf{A})^{-1}\mathbf{B} = \frac{\beta}{s(s+\alpha)} = \frac{12.3005958}{s(s+1.9885963)}$

The speed transfer function is

$G_\omega(s) = \frac{\beta}{s+\alpha} = \frac{12.3005958}{s+1.9885963}$

**Poles:** $0$ and $-1.9885963$ for position; $-1.9885963$ for speed. There are no finite zeros.

**Interpretation:**

- The position plant is **Type 1** — it has one integrator. A constant voltage produces a ramp in angle, not a constant angle.
- The speed plant is **Type 0** — a constant voltage produces a constant speed. Proportional feedback alone therefore has a steady-state error unless a feedforward or integral term supplies the operating voltage.

---

## 4. Why plain LQR is not enough

### 4.1 Position: PD can hold a step, but not robustly

A plain LQR on the two plant states uses the law

$u = -K_\theta(\theta - r) - K_\omega\,\omega$

This is a PD controller. Because the plant is Type 1, the nominal closed-loop DC gain from $r$ to $\theta$ is unity: at equilibrium $\omega = 0$ and $e = 0$, so $u = 0$, and the disc stays at the angle it reached because there is no spring. In the ideal model, no integral state is required.

The real loop is not ideal. Three effects create a **standing error**:

1. **Constant input disturbances.** Viscous friction is modelled, but stiction, driver offset, and unmodelled load torque are not. At equilibrium, PD must produce a nonzero voltage $u = d$ to oppose the disturbance, so it must have a nonzero error: $K_\theta e = d$, hence $e = d/K_\theta$. A 0.01 V residual with $K_\theta = 11.4$ gives $e = 0.00088$ rad $= 0.05^\circ$; a 0.1 V residual gives $0.5^\circ$.
2. **Parameter mismatch.** If $\beta$ or $\alpha$ differs from the design value, the voltage needed to hold the angle differs from the model's prediction, again producing an offset.
3. **Measurement bias.** The encoder index handling is a measurement, not a plant, effect. Before the one-shot Z fix, the position loop held **90.103°** instead of 90.000° — a reproducible 0.103° standing offset caused by re-snapping the decoder count every millisecond inside the ±0.1° index window. The integral state can trim such a bias, but the correct fix is to stop generating the bias (Section 10.5). Both were needed: the Z fix removed the source, and the integral trims the residual.

**Conclusion:** plain LQR is *structurally capable* of holding a step, but it has no memory of the error. Any constant disturbance or model error leaves an offset. Adding the integral of the position error gives the controller that memory and drives the steady-state error to zero. That is LQI.

### 4.2 Speed: proportional control has a real steady-state error

For the speed plant, suppose we use proportional feedback plus a constant feedforward (here $V_{FF}$ denotes the effective voltage after the deadzone):

$u = K(\omega_{ref} - \omega) + V_{FF}$

At steady state, $\dot{\omega} = 0$:

$0 = -\alpha\,\omega_{ss} + \beta\left[K(\omega_{ref} - \omega_{ss}) + V_{FF}\right]$

Collect $\omega_{ss}$:

$(\alpha + \beta K)\,\omega_{ss} = \beta K\,\omega_{ref} + \beta V_{FF}$

If the feedforward is exact, $\beta V_{FF} = \alpha\,\omega_{ref}$, then

$(\alpha + \beta K)\,\omega_{ss} = (\alpha + \beta K)\,\omega_{ref} \implies \omega_{ss} = \omega_{ref}$

So **exact feedforward alone removes the offset**. But the feedforward is only as good as the parameters. If $V_{FF}$ is wrong by $\delta V$, the resulting speed error is

$\omega_{ss} - \omega_{ref} = \frac{\beta\,\delta V}{\alpha + \beta K}$

or, in magnitude, $|\omega_{ref}-\omega_{ss}| = \beta|\delta V|/(\alpha+\beta K)$.

The integral term removes this residual without needing the parameters to be exact. The combination is the standard two-degree-of-freedom structure:

- **Feedforward** carries the nominal operating point (fast, no waiting for the integrator).
- **Integral** trims the model error (slow, but exact).

A pure integrator without feedforward would have to build the entire 2.825 V from zero, which is slow and causes windup. Pure feedforward without integral would be exact only on paper. The implemented scheme uses both.

### 4.3 The unified view

For a first-order plant, LQI with states $[\omega, q]$ and $q = \int(\omega - \omega_{ref})$ is **exactly PI**:

$u = -k_1(\omega - \omega_{ref}) - k_2 q + V_{FF}$

Mapping $K_2 = -k_1$ and $K_I = -k_2$ gives the C law. There is no separate velocity state to feed back — speed *is* the state. So the speed controller is not an ad-hoc PI bolted onto an LQR design; it is the first-order LQI, with gains chosen for the real measurement chain and with feedforward added.

For the second-order position plant, LQI has a genuine extra state (velocity) and is therefore **not** the same as PI. It is a full state-feedback design with an integral augmentation.

This is the essential justification for the split:

- **Position:** second-order plant → full LQI (position, velocity, integral).
- **Speed:** first-order plant → LQI degenerates to PI; add feedforward for the operating point.

---

## 5. Integral error state (LQI augmentation)

### 5.1 Tracking error

Define the position tracking error

$\mathbf{e}(t) = \mathbf{C}\mathbf{x}(t) - r = \theta(t) - r$

where $r$ is the constant angle setpoint and $\mathbf{C} = [1\ 0]$.

### 5.2 Integral state

Define

$q(t) = \int_0^t e(\tau)\,d\tau = \int_0^t \left(\theta(\tau) - r\right)d\tau$

Differentiate:

$\dot{q} = \theta - r$

### 5.3 Augmented state

$\mathbf{x}_{aug} = \begin{bmatrix} \theta \\ \omega \\ q \end{bmatrix}$

### 5.4 Augmented matrices

$\dot{\mathbf{x}}_{aug} = \begin{bmatrix} \dot{\theta} \\ \dot{\omega} \\ \dot{q} \end{bmatrix} = \begin{bmatrix} \omega \\ -\alpha\omega + \beta u \\ \theta - r \end{bmatrix}$

In matrix form, dropping the constant $-r$ (it is a forcing term, not a dynamics term):

$\mathbf{A}_{aug} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ \mathbf{C} & 0 \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & -\alpha & 0 \\ 1 & 0 & 0 \end{bmatrix}$

$\mathbf{B}_{aug} = \begin{bmatrix} \mathbf{B} \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ \beta \\ 0 \end{bmatrix}$

Numerically:

$\mathbf{A}_{aug} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & -1.9885963 & 0 \\ 1 & 0 & 0 \end{bmatrix}, \qquad \mathbf{B}_{aug} = \begin{bmatrix} 0 \\ 12.3005958 \\ 0 \end{bmatrix}$

### 5.5 Why the augmentation is controllable

The controllability matrix is

$\mathcal{C} = \begin{bmatrix} \mathbf{B}_{aug} & \mathbf{A}_{aug}\mathbf{B}_{aug} & \mathbf{A}_{aug}^2\mathbf{B}_{aug} \end{bmatrix} = \begin{bmatrix} 0 & \beta & -\alpha\beta \\ \beta & -\alpha\beta & \alpha^2\beta \\ 0 & 0 & \beta \end{bmatrix}$

Its determinant is $-\beta^3 \neq 0$, so the augmented system is controllable and the Riccati equation has a stabilizing solution. The integral state is not measured directly, but it is generated by the controller from the measured error; open-loop observability of $q$ is not required.

### 5.6 The constant reference term

The full augmented equation is

$\dot{\mathbf{x}}_{aug} = \mathbf{A}_{aug}\mathbf{x}_{aug} + \mathbf{B}_{aug}u + \begin{bmatrix} 0 \\ 0 \\ -r \end{bmatrix}$

The last term is constant. It shifts the equilibrium but does not change the A or B matrices, so **the LQR gains are independent of the setpoint**. This is why the same gains work at 23°, 90°, or any other angle inside the decoder range. The integral state absorbs the constant forcing and selects the equilibrium with $e = 0$.

This also corrects a common misconception: the Python calculator is written as a $r = 0$ regulator ($\dot{q} = -\theta$ in its convention), but because $r$ is constant, the gains are valid for any $r$. The C code integrates the true error $r - \theta$, which is exactly the negative of the standard integral state, $q_e = -q$. The sign is absorbed by the gain mapping $K_q = -K_{raw}[2]$. There is no "ref = 0 mismatch" in the LQI design.

---

## 6. LQI cost function and Riccati equation

### 6.1 Cost

Minimize the infinite-horizon quadratic cost

$J = \int_0^\infty \left(\mathbf{x}_{aug}^T \mathbf{Q}_{aug}\mathbf{x}_{aug} + R\,u^2\right)dt$

where:

- $\mathbf{Q}_{aug} \succeq 0$ penalizes position error, velocity, and accumulated position error,
- $R > 0$ penalizes control voltage.

### 6.2 Continuous-time algebraic Riccati equation

Solve for the symmetric positive-definite $\mathbf{P}$:

$\mathbf{A}_{aug}^T\mathbf{P} + \mathbf{P}\mathbf{A}_{aug} - \mathbf{P}\mathbf{B}_{aug}R^{-1}\mathbf{B}_{aug}^T\mathbf{P} + \mathbf{Q}_{aug} = \mathbf{0}$

### 6.3 Optimal gain

$\mathbf{K}_{aug} = R^{-1}\mathbf{B}_{aug}^T\mathbf{P} = \begin{bmatrix} K_\theta & K_\omega & K_q \end{bmatrix}$

### 6.4 Closed-loop dynamics

$u = -\mathbf{K}_{aug}\mathbf{x}_{aug} = -K_\theta\theta - K_\omega\omega - K_q q$

$\dot{\mathbf{x}}_{aug} = \left(\mathbf{A}_{aug} - \mathbf{B}_{aug}\mathbf{K}_{aug}\right)\mathbf{x}_{aug}$

The eigenvalues of $\mathbf{A}_{aug} - \mathbf{B}_{aug}\mathbf{K}_{aug}$ are the closed-loop poles. For the current weights they are all in the open left-half plane, so the loop is stable.

### 6.5 Current numerical solution

With

$\mathbf{Q}_{aug} = \operatorname{diag}(100,\ 2.5,\ 50), \qquad R = 1$

the ARE solution is

$\mathbf{P} = \begin{bmatrix} 23.414905 & 0.926145 & 14.890379 \\ 0.926145 & 0.147891 & 0.574856 \\ 14.890379 & 0.574856 & 80.554565 \end{bmatrix}$

and the gain is

$\mathbf{K}_{aug} = \begin{bmatrix} 11.3836 & 1.9305 & 7.0711 \end{bmatrix}$

The closed-loop poles are

$s = -18.632682,\quad -6.558357,\quad -0.711772\ \text{s}^{-1}$

All three are stable. The slow pole $-0.7118$ is the integral-dominated mode; it is what gives the loop its long settling tail but also its zero steady-state error.

---

## 7. Control law

### 7.1 Standard form

$u = -K_\theta(\theta - r) - K_\omega\,\omega - K_q\,q$

with $q = \int(\theta - r)\,dt$.

### 7.2 C code form: flip the error sign

The C code defines the error the other way round:

$e = r - \theta$

and integrates it:

$q_e = \int e\,dt = \int(r - \theta)\,dt = -q$

Substitute into the standard law:

$u = -K_\theta(\theta - r) - K_\omega\omega - K_q q$

$= K_\theta(r - \theta) - K_\omega\omega + K_q q_e$

$= K_\theta e + K_\omega(-\omega) + K_q q_e$

This is exactly the C law with **all three gains positive**:

```c
float error_pos = theta_ref - t;
float error_vel = 0.0f - d_filt;
pos_integral += error_pos * 0.001f;
float u = (K_POS * error_pos) + (K_VEL * error_vel) + (K_INT * pos_integral);
```

| Math | C variable | Sign |
|---|---|---|
| $e = r - \theta$ | `error_pos` | positive when the disc is below target |
| $-\omega$ | `error_vel = -d_filt` | opposes motion |
| $q_e = \int e$ | `pos_integral` | accumulates the error |
| $K_\theta$ | `K_POS` | positive |
| $K_\omega$ | `K_VEL` | positive |
| $K_q$ | `K_INT` | positive |

The sign flip is the most common source of confusion in reading this controller. There is only one negative feedback loop; it is written with positive gains because the error is defined as *reference minus measurement*, not *measurement minus reference*.

### 7.3 Integral update and clamp

$q_e[k+1] = q_e[k] + e[k]\cdot 0.001$

with

$q_e \in [-2,\ 2]$

The clamp prevents the integral from accumulating without bound while the motor is railed. It does **not** stop integration at the rail; it only limits the stored value. Section 11 discusses the consequence.

### 7.4 Full output chain

After the control law:

1. **Deadzone inverse:** if $|u| > 0.001$, add $\operatorname{sign}(u)\,d_z$.
2. **Direction:** `*motor_dir = (u >= 0) ? 1 : -1`.
3. **Magnitude:** return $|u|/V_{max}$.
4. **Rail:** clamp $|u| \le V_{max}$ before the duty-cycle conversion.

In the simulator, the direction and duty cycle reconstruct $V_{requested} = pwm \cdot V_{max} \cdot dir$, and the deadzone model produces $V_{eff} = u$. The linear design therefore sees exactly the voltage it was designed for.

### 7.5 Position error wrap

The C code wraps the error to $[-\pi, \pi]$:

```c
while (error_pos >  PI) error_pos -= 2.0f * PI;
while (error_pos < -PI) error_pos += 2.0f * PI;
```

This makes the controller take the **shortest angular path** to the target. For a 90° step from 0, the error is +90° and the disc turns in the positive direction. For a target near 180°, the wrap chooses the shorter side instead of unwinding the long way round. The wrap is a nonlinearity, but it acts only at the ±180° boundary; inside the operating range the loop is linear.

### 7.6 Current target

The checked-in code currently has

```c
float theta_ref = 23.0f * (PI / 180.0f);
```

so a fresh build holds **23°**, not 90°. The 90° results quoted in Section 15 were taken with `theta_ref = 90°`. The derivation and gains are independent of the target; only the numerical response changes.

---

## 7.7 From math to C — line by line

### `compensator/position_control/compensator.c`

| Math | C code |
|---|---|
| Decode the 20 kHz quadrature stream | `for (i = 0; i < bus->n_pend; i++) decoder20_process(&dec_flywheel, bus->pend_states[i]);` |
| Count → radians | `float t = (float)dec_flywheel.count * (2.0f*PI/ENCODER20_COUNTS_PER_REV);` |
| Raw velocity | `float d_raw = (t - t_prev) / 0.001f;` |
| First-order filter | `d_filt = 0.80f*d_filt + 0.20f*d_raw;` |
| Position error | `float error_pos = theta_ref - t;` |
| Shortest-path wrap | `while (error_pos > PI) error_pos -= 2*PI;` etc. |
| Velocity error | `float error_vel = 0.0f - d_filt;` |
| Integral update | `pos_integral += error_pos * 0.001f;` |
| Integral clamp | `if (pos_integral > 2.0f) ...; if (pos_integral < -2.0f) ...;` |
| LQI law | `float u = K_POS*error_pos + K_VEL*error_vel + K_INT*pos_integral;` |
| Deadzone inverse | `if (fabsf(u) > 0.001f) u += (u > 0) ? p->deadzone : -p->deadzone;` |
| Direction | `*motor_dir = (u >= 0.0f) ? 1 : -1;` |
| Rail | `if (u > p->Vmax) u = p->Vmax; if (u < -p->Vmax) u = -p->Vmax;` |
| Duty cycle | `return u / p->Vmax;` |

### `simulator/flywheel_sim.c`

| Math | C code |
|---|---|
| $V_{requested} = pwm\,V_{max}\,dir$ | `float V_requested = motor_pwm * p->Vmax * (float)motor_dir;` |
| Deadzone model | `if (fabsf(V_requested) > p->deadzone) V_effective = (V_requested > 0) ? (V_requested - p->deadzone) : (V_requested + p->deadzone);` |
| Motor torque | `float tau_motor = (p->kt / p->R) * (V_effective - p->kb * d);` |
| Inertia | `I_shaft = 0.5*m*r*r; I_disc = 0.5*M*R*R; I_tot = I_shaft + I_disc;` |
| $\dot{\theta} = \omega$ | `dx->theta = d;` |
| $\dot{\omega} = (\tau_m - b_1\omega)/I_{tot}$ | `dx->theta_dot = (tau_motor - p->b1 * d) / I_tot;` |
| RK4 integration | `rk4_step(&s, active_pwm, dir, dt, &p);` |
| 1 kHz trigger | `i++; if (i >= 20) { ... i = 0; }` |
| Telemetry | `snprintf(buf, 192, "<%.4f\t%.4f\t%.4f\t%.4f>\n", s.theta, s.theta_dot, s.command_voltage, s.torque);` |

---

## 7.8 Worked numerical example

Take the 90° target, $r = \pi/2 = 1.5707963$ rad, and the current gains $K_\theta = 11.3836$, $K_\omega = 1.9305$, $K_q = 7.0711$.

### Snapshot A — standstill at zero, start of the step

$\theta = 0,\ \omega = 0,\ q_e = 0$

$e = r - \theta = 1.5707963$

$u = 11.3836(1.5707963) + 1.9305(0) + 7.0711(0) = 17.8813\ \text{V}$

Deadzone inverse: $u \to 18.2813$ V. This exceeds the 12 V rail, so the output saturates:

$pwm = 1.0,\quad dir = +1,\quad V_{eff} = 11.6\ \text{V}$

The initial acceleration is

$\dot{\omega} = \frac{(0.12/2.5)(11.6) - 0}{0.00390225} = 142.7\ \text{rad/s}^2$

The disc reaches 15 rad/s in about 0.1 s if the rail were held; in reality the position and velocity terms come down quickly, which is why the step is fast and why the velocity filter lag matters.

### Snapshot B — approaching the target, still moving

$\theta = 80^\circ = 1.3962634,\ \omega = 5\ \text{rad/s},\ q_e = 0.01$

$e = 90^\circ - 80^\circ = 10^\circ = 0.1745329$

$K_\theta e = 11.3836(0.1745329) = 1.9868$

$K_\omega(-\omega) = 1.9305(-5) = -9.6525$

$K_q q_e = 7.0710678(0.01) = 0.0707$

$u = 1.9868 - 9.6525 + 0.0707 = -7.5950\ \text{V}$

The velocity term dominates: the controller is already braking hard. Deadzone inverse gives $u \to -7.9950$ V, so $dir = -1$ and

$pwm = \frac{7.9950}{12} = 0.6663$

The effective voltage is $-7.5950$ V. The resulting torque is

$\tau_m = \frac{0.12}{2.5}\left(-7.5950 - 0.12(5)\right) = -0.3934\ \text{N·m}$

and the angular acceleration is

$\dot{\omega} = \frac{-0.3666 - 0.002(5)}{0.00390225} = -96.6\ \text{rad/s}^2$

This is the deceleration phase that produces the overshoot behaviour: the disc has inertia, the velocity filter lags, and the controller cannot remove the stored kinetic energy instantaneously.

### Snapshot C — at the target, tiny integral trim

$\theta = 90^\circ,\ \omega = 0,\ q_e = 0.0002$

$e = 0$

$u = 7.0710678(0.0002) = 0.0014142\ \text{V}$

This is just above the 1 mV gate, so the deadzone inverse adds 0.4 V:

$u \to 0.4014142\ \text{V},\quad pwm = \frac{0.4014142}{12} = 0.03345$

The simulator subtracts the 0.4 V deadzone and the motor sees $0.0014142$ V — exactly the control law's output. Without the pre-compensation, the requested 1.4 mV would be inside the deadzone and the motor would see zero; the integral would keep growing and the loop would stick. This is why the deadzone inverse is essential even though the position feedforward voltage is nominally zero.

### Snapshot D — ideal hold

$\theta = 90^\circ,\ \omega = 0,\ q_e = 0$

$u = 0$. The gate $|u| > 0.001$ is false, no deadzone is added, and the output is zero. The motor is off and the disc stays still because there is no gravity torque. This is the ideal equilibrium. The measured mean voltage at 90° was 0.0015 V, i.e. the integral had settled to a value of about $2\times10^{-4}$ rad·s — essentially zero.

---

## 8. Feedforward

Feedforward means "compute the input that the nominal model needs at the desired operating point, and add it to the feedback." It is not feedback; it does not wait for an error. It is the second degree of freedom in a two-degree-of-freedom controller.

### 8.1 Position: deadzone inverse

At a constant angle with zero speed, the required torque is zero (no gravity, no spring). The required effective voltage is therefore zero. The only feedforward needed is the **deadzone inverse**:

$V_{requested} = u + \operatorname{sign}(u)\,d_z$

This is not a setpoint feedforward; it is an exact inverse of the actuator nonlinearity. It ensures that the linear design's $u$ is what the motor actually sees. Section 2.5 gives the derivation.

There is no position-dependent feedforward term because holding a different angle does not require a different torque. This is a key difference from the furnace, where holding a temperature requires continuous power against ambient loss.

### 8.2 Speed: steady-state voltage

At the desired speed $\omega_{ref}$, set $\dot{\omega} = 0$ in the speed equation:

$0 = -\alpha\,\omega_{ref} + \beta\,V_{eff}$

Solve for the effective voltage:

$V_{eff} = \frac{\alpha}{\beta}\,\omega_{ref}$

Substitute $\alpha$ and $\beta$:

$\frac{\alpha}{\beta} = \frac{(b_1 + k_t k_b/R)/I_{tot}}{k_t/(R I_{tot})} = \frac{b_1 R}{k_t} + k_b$

So

$\boxed{V_{eff} = \left(\frac{b_1 R}{k_t} + k_b\right)\omega_{ref}}$

The bracket has units of V·s/rad. Numerically:

$\frac{b_1 R}{k_t} = \frac{0.002 \times 2.5}{0.12} = 0.0416667\ \text{V·s/rad}$

$k_b = 0.12\ \text{V·s/rad}$

$\frac{\alpha}{\beta} = 0.2866667\ \text{V·s/rad}$

At $\omega_{ref} = 15$ rad/s:

$V_{eff} = 0.1616667 \times 15 = 2.425\ \text{V}$

Add the deadzone:

$V_{FF} = 2.425 + 0.4 = 2.825\ \text{V}$

This is exactly the `#define V_FF 2.8250f` in the speed compensator.

### 8.3 Physical check at 15 rad/s

The numbers balance exactly:

- Back-EMF: $k_b\omega = 0.12 \times 15 = 1.80$ V.
- Effective voltage: 2.425 V.
- Armature current: $(2.425 - 1.80)/2.5 = 0.25$ A.
- Motor torque: $0.12 \times 0.25 = 0.030$ N·m.
- Viscous friction: $b_1\omega = 0.002 \times 15 = 0.030$ N·m.
- Net torque: $0.030 - 0.030 = 0$.

The feedforward voltage is not a tuning knob; it is the exact solution of the steady-state torque balance. That is why it removes the bulk of the speed error immediately.

### 8.4 Feedforward versus speed

| $\omega_{ref}$ (rad/s) | $V_{eff}$ (V) | $V_{FF}$ (V) | Note |
|---:|---:|---:|---|
| 5 | 0.808 | 1.208 | |
| 10 | 1.617 | 2.017 | |
| 15 | 2.425 | 2.825 | measured operating point |
| 20 | 3.233 | 3.633 | |
| 30 | 4.850 | 5.250 | |
| 71.753 | 11.588 | 11.988 | 12 V rail, no-load maximum |

The linear feedforward is valid up to the rail. Beyond 71.753 rad/s the requested voltage exceeds 12 V and the motor cannot reach the setpoint; the integral would wind up unless anti-windup stops it. The current speed setpoint of 15 rad/s has plenty of margin.

### 8.5 Why the speed loop needs feedforward more than the position loop

The position loop's equilibrium voltage is zero, so a missing feedforward only affects the transient. The speed loop's equilibrium voltage is 2.425 V effective, which is a large fraction of the 12 V rail. Without feedforward, the integral must build that 2.425 V from zero, and the loop starts with a large error and a long windup. With feedforward, the loop starts at the correct operating point and the integral only has to trim a small residual. This is the main reason the speed response in Section 15 is smooth despite the soft gains.

---

## 9. Choosing Q and R

### 9.1 Position weights

The current cost is

$\mathbf{Q}_{aug} = \operatorname{diag}(100,\ 2.5,\ 50), \qquad R = 1$

Bryson's rule interprets each weight as the inverse square of the largest acceptable value:

$Q_{ii} = \frac{1}{x_{i,max}^2}, \qquad R = \frac{1}{u_{max}^2}$

For the current numbers:

| State | Weight | Implied maximum | Interpretation |
|---|---:|---:|---|
| $\theta$ error | 100 | $1/\sqrt{100} = 0.1$ rad = 5.73° | position should stay within about 6° |
| $\omega$ | 2.5 | $1/\sqrt{2.5} = 0.632$ rad/s | velocity should stay within about 0.63 rad/s |
| $q_e$ | 50 | $1/\sqrt{50} = 0.141$ rad·s | integral should stay within about 0.14 rad·s |
| $u$ | 1 | $1/\sqrt{1} = 1$ V | control effort is cheap relative to the original design |

The ratios are what matter, not the absolute numbers. The position weight is 40× the velocity weight, so the design prioritizes getting to the angle over suppressing velocity; the integral weight is 20× the velocity weight, so it removes offset aggressively once the error is small. The control weight $R=1$ is 20× larger than the original 0.05, which is a deliberate de-tune: it buys rail margin and smoothness at the cost of a slower integral pole.

### 9.2 Position design comparison

| Design | $\mathbf{Q}_{aug}$ | $R$ | $K_\theta$ | $K_\omega$ | $K_q$ | Poles (s⁻¹) | Comment |
|---|---|---:|---:|---:|---:|---|---|
| Aggressive (original) | diag(250, 2.5, 100) | 0.05 | 75.5403 | 7.6105 | 44.7214 | −86.47, −10.04, −0.634 | large gains, more rail activity and filter excitation |
| **Current** | **diag(100, 2.5, 50)** | **1.0** | **11.3836** | **1.9305** | **7.0711** | **−18.63, −6.56, −0.712** | **0.01% rail, 7.97° overshoot, flat hold** |
| Bryson tight | diag(131.31, 0.04, 131.31) | 0.006944 | 142.8022 | 5.1042 | 137.5131 | −32.66 ± 24.997j, −1.000 | very aggressive; not used |

The current design is not the "textbook optimal" for a 5° position tolerance; it is a deliberately softer design. The aggressive set proves the point: it has 6.6× the position gain and 6.3× the integral gain, and its fast pole at −86.5 rad/s is much closer to the velocity-filter corner (223 rad/s) and the 1 kHz sample rate. The current set keeps the fastest LQI design pole at −18.6 rad/s (before the measurement filter), where the filter lag is only 4.26° (Section 10.4). That is the engineering trade-off: a little slower, much less sensitive to the measurement chain and the rail.

The Bryson-tight set is shown for reference. It places a complex pair at −32.66 ± 24.997j and an integral pole at −1.000. It would be faster but would demand much more voltage and would excite the filter more. It is not the checked-in design.

### 9.3 Speed weights

The textbook speed LQI uses

$\mathbf{Q}_{aug} = \operatorname{diag}(100,\ 50), \qquad R = 1$

with the augmented state $[\omega,\ q]$. The resulting gain is

$\mathbf{K} = \begin{bmatrix} 9.7747393 & 7.0710678 \end{bmatrix}$

which maps to

$K_2 = -9.7747393,\qquad K_I = -7.0710678$

The poles are $-123.0545$ and $-0.7068$ s⁻¹. The fast pole is the proportional-speed mode; the slow pole is the integral trim.

The implemented gains are

$K_2 = -1.0,\qquad K_I = -0.05$

which are much softer. Why?

1. **The fast LQI pole sits where the measurement filter has significant lag.** The 0.8/0.2 filter has 25.48° of phase lag at 123.05 rad/s (Section 10.4). A loop whose crossover is near that frequency loses phase margin, and the 20-bit quantization noise is amplified by the high proportional gain.
2. **The speed loop is first-order; LQI is PI.** There is no second plant state whose feedback could justify a full state-feedback design. The only design freedom is the two PI gains, and the LQI gains are just one choice of them.
3. **The feedforward carries the operating point.** With $V_{FF}=2.825$ V, the feedback only has to trim a small residual. A soft proportional gain is enough, and it keeps the rail activity low.
4. **The measured result meets the requirement.** The 5-minute run reached 14.999561 rad/s with standard deviation 0.000434 rad/s and 0.022% rail saturation. The 60-second value was 15.0042 rad/s, still converging on the slow integral pole.

The cost of the soft integral gain is the 25.7 s time constant. If faster convergence is needed, increase $|K_I|$ (for example to 0.5) and re-check ripple and rail activity; the LQI table gives the aggressive end of the trade-off. The current choice is the smooth end.

### 9.4 Summary of the gain choice

| Profile | Structure | Gains | Why |
|---|---|---|---|
| Position | LQI | 11.3836, 1.9305, 7.0711 | second-order plant, zero steady-state error, moderate bandwidth, 0.01% rail |
| Speed | PI + FF | −1.0, −0.05, 2.825 V | first-order plant, feedforward carries the operating point, soft gains avoid filter/quantization excitation |

---

## 10. Numerical solution

### 10.1 Position LQI

**Matrices:**

$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & -3.5261708 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 12.3005958 \end{bmatrix}$

$\mathbf{A}_{aug} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & -3.5261708 & 0 \\ 1 & 0 & 0 \end{bmatrix}, \quad \mathbf{B}_{aug} = \begin{bmatrix} 0 \\ 12.3005958 \\ 0 \end{bmatrix}$

**Weights:** $\mathbf{Q}_{aug} = \operatorname{diag}(100, 2.5, 50)$, $R = 1$.

**ARE solution:**

$\mathbf{P} = \begin{bmatrix} 23.414905 & 0.926145 & 14.890379 \\ 0.926145 & 0.147891 & 0.574856 \\ 14.890379 & 0.574856 & 80.554565 \end{bmatrix}$

**Gain:**

$\mathbf{K} = \begin{bmatrix} 11.3836 & 1.9305 & 7.0711 \end{bmatrix}$

**Continuous poles:**

$-18.632682,\quad -6.558357,\quad -0.711772\ \text{s}^{-1}$

**Discrete poles at 1 kHz:**

| Discretization | $z_1$ | $z_2$ | $z_3$ |
|---|---:|---:|---:|
| ZOH, $e^{\mathbf{A}_{cl}T}$ | 0.98153983 | 0.99346310 | 0.99928848 |
| Forward Euler, $\mathbf{I}+\mathbf{A}_{cl}T$ | 0.98136732 | 0.99344164 | 0.99928823 |

All inside the unit circle. The two discretizations agree to about $2\times10^{-4}$ in the fast poles and $3\times10^{-7}$ in the slow pole, which is expected because $|s|T \le 0.0186 \ll 1$.

**With the velocity filter:**

The implemented filter is

$d_{filt}[k] = 0.8\,d_{filt}[k-1] + 0.2\,d_{raw}[k]$

Its continuous-equivalent pole is

$\omega_f = -\frac{\ln(0.8)}{0.001} = 223.143551\ \text{s}^{-1}$

Augmenting the position loop with the filter state $d_{filt}$ gives the continuous poles

$-197.494751,\quad -22.264531,\quad -6.198323,\quad -0.712118\ \text{s}^{-1}$

and the ZOH-equivalent discrete poles

$0.82078445,\quad 0.97798149,\quad 0.99382085,\quad 0.99928814$

The filter moves the fast position/velocity pair from $(-18.63,-6.56)$ to $(-22.26,-6.20)$ and adds a fast pole near $-197.5$. The integral pole is essentially unchanged at $-0.7121$. The filter does **not** destabilize the loop; it adds phase lag, but at the position-loop crossover ($\approx18.6$ rad/s) the exact discrete lag is only 4.26° (Section 10.4).

### 10.2 Speed

**Textbook LQI:**

$\mathbf{A}_s = \begin{bmatrix} -1.9885963 & 0 \\ 1 & 0 \end{bmatrix}, \quad \mathbf{B}_s = \begin{bmatrix} 12.3005958 \\ 0 \end{bmatrix}$

$\mathbf{Q}_s = \operatorname{diag}(100, 50), \quad R_s = 1$

$\mathbf{K}_s = \begin{bmatrix} 9.7747393 & 7.0710678 \end{bmatrix}$

$K_2 = -9.7747393,\quad K_I = -7.0710678$

Poles: $-123.05446,\ -0.706828\ \text{s}^{-1}$.

ZOH poles at 1 kHz: $0.88421551,\ 0.99929342$.

**Implemented PI:**

$K_2 = -1.0,\quad K_I = -0.05$

The closed-loop A matrix is

$\mathbf{A}_{pi} = \begin{bmatrix} -\alpha + \beta K_2 & \beta K_I \\ 1 & 0 \end{bmatrix} = \begin{bmatrix} -15.826767 & -0.615030 \\ 1 & 0 \end{bmatrix}$

Poles:

$-15.787811,\quad -0.038956\ \text{s}^{-1}$

The slow pole has time constant

$\tau_{slow} = \frac{1}{0.038956} = 25.67\ \text{s}$

ZOH poles at 1 kHz: $0.98433616,\ 0.99996105$.

**Implemented PI with the velocity filter:**

States $[\omega,\ q,\ d_{filt}]$:

$\mathbf{A}_{pi,f} = \begin{bmatrix} -1.988596 & -0.615030 & -12.300596 \\ 0 & 0 & 1 \\ 223.143551 & 0 & -223.143551 \end{bmatrix}$

Poles:

$-209.842912,\quad -16.787853,\quad -0.038957\ \text{s}^{-1}$

ZOH poles: $0.81071159,\ 0.98335228,\ 0.99996104$.

The filter moves the fast pole from $-15.79$ to $-16.79$ and adds the $-209.8$ filter pole. The integral pole stays at $-0.0390$; that is the 25.7 s tail seen in the measurements. The filter does not change the slow dynamics in any meaningful way.

### 10.3 Discretization notes — continuous design at 1 kHz

The gains are computed from the continuous-time ARE and applied directly in the 1 kHz loop:

$u[k] = -K_\theta(\theta[k]-r) - K_\omega\omega[k] - K_q q[k]$

$q[k+1] = q[k] + (\theta[k]-r)\,T$

with $T = 0.001$ s. This is the same approach as the furnace controller.

**Why it is valid here:** the fastest pole of the LQI/PI design (excluding the measurement filter) is $-18.63$ rad/s for position and $-16.79$ rad/s for speed. The filter adds a pole near $-200$ rad/s, whose dimensionless product is about $0.21$, still well below 1. The design poles satisfy

$|s|T \le 18.63 \times 0.001 = 0.0186 \ll 1$

so the continuous gains are an excellent approximation to a discrete design. The discretization shifts the poles by about 2%, as the ZOH/Euler table shows.

**If you needed a discrete design:** discretize the augmented plant with

$\mathbf{A}_d = e^{\mathbf{A}_{aug}T}, \qquad \mathbf{B}_d = \int_0^T e^{\mathbf{A}_{aug}\tau}d\tau\,\mathbf{B}_{aug}$

then solve the discrete ARE and compute

$\mathbf{K}_d = (R + \mathbf{B}_d^T\mathbf{P}\mathbf{B}_d)^{-1}\mathbf{B}_d^T\mathbf{P}\mathbf{A}_d$

This is unnecessary at 1 kHz for this plant, but it is the correct path if the sample rate is lowered or the gains are made much more aggressive.

### 10.4 20-bit measurement chain

The encoder produces $2^{20} = 1{,}048{,}576$ counts per revolution. The resolution is

$\Delta\theta = \frac{360^\circ}{1{,}048{,}576} = 0.000343322754^\circ$

$\Delta\theta = \frac{2\pi}{1{,}048{,}576} = 5.992112\times10^{-6}\ \text{rad}$

The velocity is computed from the count difference over 1 ms:

$\omega_{raw}[k] = \frac{\Delta count \cdot \Delta\theta}{0.001}$

so one count per millisecond is

$\Delta\omega = \frac{5.992112\times10^{-6}}{0.001} = 0.005992112\ \text{rad/s}$

For uniformly distributed quantization error, the RMS velocity noise is

$\sigma_\omega = \frac{\Delta\omega}{\sqrt{12}} = 0.001729774\ \text{rad/s}$

The 0.8/0.2 filter reduces this by the factor

$\sqrt{\frac{0.2^2}{1-0.8^2}} = 0.333333$

so the filtered RMS is

$\sigma_{\omega,f} = 0.000576591\ \text{rad/s}$

This is consistent with the measured 5-minute speed standard deviation of 0.000434 rad/s.

**Position resolution:** the measured 90° hold error of 0.000210° is

$\frac{0.000210}{0.0003433} = 0.61\ \text{count}$

i.e. below one 20-bit count. The loop is at the measurement floor.

**Why 20-bit matters:** at the old 4096 counts/rev, the position resolution was $360/4096 = 0.0879^\circ$ and the velocity quantum at 1 kHz was $1.534$ rad/s (±0.767 rad/s). The 20-bit codec improves both by a factor of 256. At 15 rad/s the encoder produces

$\frac{15 \times 0.001}{5.992112\times10^{-6}} = 2503.3\ \text{counts/ms}$

and at the 71.753 rad/s no-load speed it produces 11976.8 counts/ms. The HIL bus is sized at 32768 states to cover 2.74 ms at the maximum speed, well above the 1 ms control window.

### 10.5 The Z index pulse

A real quadrature encoder has an index (Z) pulse once per revolution. The position loop uses it to remove accumulated count drift and to establish an absolute angle reference. The speed loop does not use it, because speed is derived from count differences and a Z snap would inject a spurious velocity spike.

The simulator generates a Z pulse when the angle enters a ±0.1° window around the revolution boundary. The original implementation held `z_pend` true for the *entire* window. At 1 kHz, that meant the compensator re-snapped the decoder count every millisecond while the disc was inside the window. Each snap moved the count to the nearest multiple of 1,048,576, discarding the real motion that had occurred since the previous snap. The result was a reproducible standing offset of about **0.103°**.

The fix has three parts:

1. **One-shot Z:** the simulator emits `z_pend` only on entry into the window:

```c
bool z_in_window = (fabsf(wrap_t) < 0.1f);
if (z_in_window && !z_prev_in_window) {
    bus.z_pend = true;
    bus.z_idx_pend = bus.n_pend;
}
z_prev_in_window = z_in_window;
```

2. **Tolerance-gated snap:** the compensator snaps only if the count is within 64 counts of a revolution boundary:

```c
int nearest = (int)(roundf((float)dec_flywheel.count /
                           (float)ENCODER20_COUNTS_PER_REV) *
                    (float)ENCODER20_COUNTS_PER_REV);
int delta = dec_flywheel.count - nearest;
if (delta < 0) delta = -delta;
if (delta <= Z_SNAP_TOL_COUNTS) {
    dec_flywheel.count = nearest;
    /* wrap into the valid ±180° range */
}
```

3. **No Z in the speed profile:** the simulator does not generate a Z pulse when `SPEED_PROFILE` is defined, and the speed compensator never reads `bus.z_pend`.

The 64-count tolerance is 0.022° at 20-bit, much smaller than the 0.1° window. It ensures that a coarse window event cannot erase real motion; only a genuine boundary crossing is snapped. With this fix the 5-minute position run held **90.000210°**, i.e. 0.61 count, and the last four minutes were flat to the telemetry resolution.

---

## 11. Anti-windup

### 11.1 Position

The position integrator is clamped to ±2:

```c
pos_integral += error_pos * 0.001f;
if (pos_integral >  2.0f) pos_integral =  2.0f;
if (pos_integral < -2.0f) pos_integral = -2.0f;
```

There is no conditional integration: the accumulator keeps integrating even when the output is railed. During a large step the error is 1.57 rad, so the integral reaches the +2 clamp in

$\frac{2}{1.57 \times 0.001} = 1.27\ \text{s}$

if the rail persists that long. In the measured 90° step the rail is active for only about 0.01% of the run (0.05% in the 1 kHz check), so the clamp is not the dominant effect; the 7.97° overshoot comes mostly from the inertia and the velocity-filter lag, not from integral windup. The integral then unwinds through the small negative error after the overshoot.

If you want less overshoot, the first lever is the anti-windup condition, not a gain change:

```c
bool saturated = (fabsf(u) >= p->Vmax);
if (!saturated) pos_integral += error_pos * 0.001f;
```

That freezes the integral while the motor is railed. It is not currently implemented; the present behaviour is documented here so the measured overshoot is understood.

### 11.2 Speed

The speed integrator uses a directional gate:

```c
if (speed_err > 0.0f && u_unsat < p->Vmax) {
    speed_error_sum += speed_err * 0.001f;
} else if (speed_err < 0.0f && u_unsat > -p->Vmax) {
    speed_error_sum += speed_err * 0.001f;
}
if (speed_error_sum >  2.0f) speed_error_sum =  2.0f;
if (speed_error_sum < -2.0f) speed_error_sum = -2.0f;
```

The intent is to stop integrating when the output is saturated in the direction that would make the error worse. But the condition is **one-sided**: when the error is negative and the output is at the *positive* rail (the normal start-up case), `u_unsat > -Vmax` is true, so the integrator keeps accumulating negative values. In the 1 kHz check the integral reaches −1.12 by 1 s, not the −2 clamp. That stored negative integral is what has to unwind during the long tail.

A stricter gate would be

```c
if (fabsf(u_unsat) < p->Vmax) {
    speed_error_sum += speed_err * 0.001f;
}
```

which freezes integration at either rail. This is a tuning opportunity, not a bug that prevents convergence: the 5-minute run reaches 14.9996 rad/s. It is left unchanged because the current speed tuning was accepted as-is.

### 11.3 Why the speed tail is 25 s, not 2.5 s

It is tempting to blame the integral clamp. The clamp is not the cause: the 1 kHz check shows the integral peaks at −1.12, well inside the ±2 limit. The cause is the **slow PI pole** itself. With $K_I = -0.05$, the characteristic polynomial is

$s^2 + 15.827s + 0.615 = 0$

whose roots are $-15.79$ and $-0.0390$. The slow root is the integral trim; its time constant is 25.7 s, so a 5-minute run is only about 12 time constants. The measured 60-second value of 15.0042 rad/s and the 5-minute value of 14.9996 rad/s lie exactly on that exponential.

This is the price of the soft integral gain. Increasing $|K_I|$ moves the slow pole left and shortens the tail, at the cost of more ripple and rail activity.

---

## 12. Tuning guide

| Goal | Adjustment | Watch out |
|---|---|---|
| Faster position settling | Increase $Q_\theta$ or decrease $R$ | more rail saturation, more overshoot, more filter excitation |
| Less position overshoot | Increase $Q_\omega$ or decrease $Q_\theta$; add conditional anti-windup | too much velocity weight makes the loop sluggish |
| Zero position offset | Keep $K_q > 0$; verify the one-shot Z and the 64-count tolerance | a coarse Z window can inject a standing offset |
| Faster speed convergence | Increase $|K_I|$ (e.g. −0.5) | more ripple, more rail activity, possible filter interaction |
| Less speed ripple | Decrease $|K_2|$ or $|K_I|$; increase the filter coefficient | more phase lag, slower response |
| Different speed setpoint | Change `OMEGA_REF_RAD_S`; recompute $V_{FF} = 0.1616667\,\omega_{ref} + 0.4$ | check the 12 V rail; no-load maximum is 71.753 rad/s |
| Different position setpoint | Change `theta_ref` | the decoder wraps at ±180°; the error wrap chooses the shortest path |
| Different plant | Edit the parameters in `docs/flywheel_lqi_design.py` and re-run | copy the printed `#define`s into the C files |
| Different encoder resolution | Update the codec and the velocity quantum | the 20-bit codec is shared with the other models; do not break the 4096 API |

---

## 13. Code mapping

| Math | File | Function / location |
|---|---|---|
| Electrical + mechanical model | `simulator/flywheel_sim.c` | `compute_derivatives()` |
| State-space A, B | `simulator/flywheel_sim.c` | `compute_derivatives()` |
| RK4 integration | `simulator/flywheel_sim.c` | `rk4_step()` |
| 20 kHz → 1 kHz downsample | `simulator/flywheel_sim.c` | `if (i >= 20)` block |
| Position LQI law | `compensator/position_control/compensator.c` | `run_compensator_cycle()` |
| Position gains | `compensator/position_control/compensator.c` | `#define K_POS`, `K_VEL`, `K_INT` |
| Position integral | `compensator/position_control/compensator.c` | `pos_integral` |
| Position deadzone inverse | `compensator/position_control/compensator.c` | `if (fabsf(u) > 0.001f)` |
| Speed PI + FF law | `compensator/speed_control/compensator.c` | `run_compensator_cycle()` |
| Speed gains / FF | `compensator/speed_control/compensator.c` | `K2_SPEED`, `KI_SPEED`, `V_FF` |
| Speed integral | `compensator/speed_control/compensator.c` | `speed_error_sum` |
| 20-bit encoder/decoder | `framework/transport/codecs/encoder_decoder_logic.c` | `Encoder20`, `Decoder20` |
| Z-pulse generation | `simulator/flywheel_sim.c` | one-shot block, `z_prev_in_window` |
| Z-pulse snap | `compensator/position_control/compensator.c` | `Z_SNAP_TOL_COUNTS` block |
| HIL host | `compensator/hil_control/compensator_hil.c` | `run_compensator_cycle()` |
| Telemetry | `simulator/flywheel_sim.c` | `snprintf(buf, ..., "<%.4f\t...")` |

---

## 13.5 Line-by-line code mapping

### `simulator/flywheel_sim.c` — plant physics

| Derivation | Equation | C code |
|---|---|---|
| Requested voltage | $V_{req} = pwm\,V_{max}\,dir$ | `float V_requested = motor_pwm * p->Vmax * (float)motor_dir;` |
| Deadzone | $V_{eff} = V_{req} - \operatorname{sign}(V_{req})d_z$ | `if (fabsf(V_requested) > p->deadzone) V_effective = (V_requested > 0) ? (V_requested - p->deadzone) : (V_requested + p->deadzone);` |
| Motor torque | $\tau_m = (k_t/R)(V_{eff}-k_b\omega)$ | `float tau_motor = (p->kt / p->R) * (V_effective - p->kb * d);` |
| Shaft inertia | $I_{shaft} = \frac{1}{2}m_s r_s^2$ | `float I_shaft = 0.5f * p->motor_shaft_mass * p->motor_shaft_radius * p->motor_shaft_radius;` |
| Disc inertia | $I_{disc} = \frac{1}{2}M_d R_d^2$ | `float I_disc = 0.5f * p->M_disc * p->R_disc * p->R_disc;` |
| Total inertia | $I_{tot} = I_{shaft}+I_{disc}$ | `float I_tot = I_shaft + I_disc;` |
| Angle derivative | $\dot{\theta} = \omega$ | `dx->theta = d;` |
| Speed derivative | $\dot{\omega} = (\tau_m-b_1\omega)/I_{tot}$ | `dx->theta_dot = (tau_motor - p->b1 * d) / I_tot;` |
| RK4 | $\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6}(k_1+2k_2+2k_3+k_4)$ | `x->theta += (dt/6.0f)*(k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);` |
| 1 kHz trigger | Every 20 physics steps | `i++; if (i >= 20) { ... i = 0; }` |
| Compensator call | $pwm = $ `run_compensator_cycle(...)` | `pwm = run_compensator_cycle(&bus, &p, &dir);` |
| Telemetry | `<theta theta_dot V torque>` | `snprintf(buf, 192, "<%.4f\t%.4f\t%.4f\t%.4f>\n", ...);` |

### `compensator/position_control/compensator.c` — LQI

| Derivation | Equation | C code |
|---|---|---|
| Decode 20-bit counts | — | `for (i = 0; i < bus->n_pend; i++) decoder20_process(&dec_flywheel, bus->pend_states[i]);` |
| Count → radians | $\theta = count \cdot 2\pi/N$ | `float t = (float)dec_flywheel.count * COUNT_TO_RAD;` |
| Raw velocity | $\omega_{raw} = (\theta_k-\theta_{k-1})/T$ | `float d_raw = (t - t_prev) / 0.001f;` |
| Filter | $d_f = 0.8d_f + 0.2d_{raw}$ | `d_filt = (0.80f * d_filt) + (0.20f * d_raw);` |
| Position error | $e = r-\theta$ | `float error_pos = theta_ref - t;` |
| Wrap | $e \in [-\pi,\pi]$ | `while (error_pos > PI) error_pos -= 2.0f*PI;` |
| Velocity error | $-d_f$ | `float error_vel = 0.0f - d_filt;` |
| Integral | $q_e \leftarrow q_e + eT$ | `pos_integral += error_pos * 0.001f;` |
| Clamp | $q_e \in [-2,2]$ | `if (pos_integral > 2.0f) ...;` |
| LQI law | $u = K_\theta e + K_\omega(-d_f) + K_q q_e$ | `float u = (K_POS*error_pos) + (K_VEL*error_vel) + (K_INT*pos_integral);` |
| Deadzone inverse | $u \leftarrow u + \operatorname{sign}(u)d_z$ | `if (fabsf(u) > 0.001f) u += (u > 0.0f) ? p->deadzone : -p->deadzone;` |
| Direction | $dir = \operatorname{sign}(u)$ | `*motor_dir = (u >= 0.0f) ? 1 : -1;` |
| Rail | $|u| \le V_{max}$ | `if (u > p->Vmax) u = p->Vmax; if (u < -p->Vmax) u = -p->Vmax;` |
| Duty | $pwm = |u|/V_{max}$ | `return u / p->Vmax;` |

### `compensator/speed_control/compensator.c` — PI + FF

| Derivation | Equation | C code |
|---|---|---|
| Decode 20-bit counts | — | `for (i = 0; i < bus->n_pend; i++) decoder20_process(&dec_flywheel, bus->pend_states[i]);` |
| Count delta | $\Delta count = count_k - count_{k-1}$ | `int raw_delta = dec_flywheel.count - prev_count;` |
| Unwrap | $\Delta count \in [-N/2, N/2)$ | `if (raw_delta > ENCODER20_DECODER_WRAP - 1) ...` |
| Raw velocity | $\omega_{raw} = \Delta count \cdot 2\pi/(NT)$ | `float d_raw = (float)delta_count * COUNT_TO_RAD / 0.001f;` |
| Filter | $d_f = 0.8d_f + 0.2d_{raw}$ | `d_filt = (0.80f * d_filt) + (0.20f * d_raw);` |
| Speed error | $e_\omega = d_f-\omega_{ref}$ | `float speed_err = d_filt - OMEGA_REF_RAD_S;` |
| PI + FF | $u = K_2 e_\omega + K_I q_\omega + V_{FF}$ | `float u = (K2_SPEED*speed_err) + (KI_SPEED*speed_error_sum) + V_FF;` |
| Conditional integration | $q_\omega \leftarrow q_\omega + e_\omega T$ | `if (speed_err > 0 && u_unsat < Vmax) ...` |
| Clamp | $q_\omega \in [-2,2]$ | `if (speed_error_sum > 2.0f) ...;` |
| Rail | $|u| \le V_{max}$ | `if (u > p->Vmax) u = p->Vmax;` |
| Direction | $dir = \operatorname{sign}(u)$ | `*motor_dir = (u >= 0) ? 1 : -1;` |
| Duty | $pwm = |u|/V_{max}$ | `return fabsf(u) / p->Vmax;` |

---

## 14. HIL notes

The hardware-in-the-loop profile uses the same 20-bit host path:

- `compensator/hil_control/compensator_hil.c` uses `Decoder20` and `decoder20_process`.
- `MAX_BUS_STATES` is 32768, enough for 2.74 ms at the 71.753 rad/s no-load speed.
- `Makefile_HIL` defines `-DHIL_PROFILE`, so `flywheel_sim.c` enables the 20-bit codec for HIL as well.
- Both directions use the shared **12-byte** frame (an 8-byte packet belongs to an older adapter): host→MCU `sync 0xAA 0x55 | float pos1 | float pos2 | uint8 cmd | uint8 checksum`, MCU→host `sync 0x55 0xAA | float out1 | float out2 | uint8 status | uint8 checksum`.
- `pos1` carries the angle in **radians** (not raw counts, wrapped to ±π) with `pos2 = 0`; the MCU returns the signed motor voltage in `out1` and echoes `pos2` in `out2`. 921600 baud, 3 ms poll timeout. The 20-bit change therefore needs no CPR or packet-format change on the MCU side.

**Runtime status:** the HIL control law builds and the shared USB transport is healthy — a host→MCU→host exchange measures **≈ 0.4 ms** at 921600 baud, far inside the 3 ms poll timeout. An earlier note here claiming a ~300 ms USB CDC turnaround was wrong. Both flywheel profiles have been run in HIL on the STM32F411.

---

## 15. Measured results

### 15.1 Position, 20-bit, 5 minutes, 90° target

| Metric | Value |
|---|---:|
| Final angle | 90.000210° |
| Final error | 0.000210° (0.61 of one 20-bit count) |
| Peak | 97.970053° |
| Overshoot | 7.970053° |
| Settle within 2° | 3.82 s |
| Settle within 1° | 4.80 s |
| Settle within 0.5° | 5.76 s |
| Settle within 0.1° | 8.01 s |
| Settle within 0.05° | 9.04 s |
| Settle within 0.01° | 11.64 s |
| Rail saturation | 0.01% (29 samples) |
| Last 4 minutes | mean 90.000210°, standard deviation 0 |

The last four minutes are flat to the telemetry resolution. This is the combined result of the LQI integral action, the one-shot Z fix, and the 20-bit measurement.

**Note:** the checked-in code currently sets `theta_ref = 23°`; the 90° results above were taken with `theta_ref = 90°`.

### 15.2 Speed, 20-bit

| Metric | 60 s run | 5-minute run |
|---|---:|---:|
| Final speed | 15.0042 rad/s | 14.999561 rad/s |
| Standard deviation | 0.000323 rad/s | 0.000434 rad/s |
| Maximum deviation | 0.0049 rad/s | 0.0011 rad/s |
| Rail saturation | 0.108% | 0.022% |
| Pulse-vs-true bias | — | +0.000029 rad/s |
| Pulse-vs-true RMS | — | 0.0027 rad/s |

The 60-second value is still converging on the 25.7 s integral pole; the 5-minute value is the settled result. The pulse-derived velocity is essentially unbiased; the RMS difference from the true simulator velocity is 0.0027 rad/s, consistent with the 20-bit quantization and the 0.8/0.2 filter.

### 15.3 1 kHz Euler check

`docs/flywheel_lqi_design.py` includes a 1 kHz Euler mirror of the exact C logic. It reproduces the measurements:

- Position: peak 97.961897° at 0.805 s, overshoot 7.961897°, rail 0.047%.
- Speed: 15.004388 rad/s at 60 s, 15.000000 rad/s at 300 s, rail 0.022%.

The small differences from the measured values come from the 20 kHz RK4 plant, the 20-bit quantization, and the Z snap, none of which are in the simple check.

---

## 16. Running

From the model directory:

```bash
cd models/flywheel_with_DC_motor/fixed_weight_flywheel

make clean
make position        # build/flywheel_control/flywheel_position_sim
make speed           # build/flywheel_control/flywheel_speed_sim
make -f Makefile_HIL position   # build/flywheel_control/position/ (host + firmware)
make -f Makefile_HIL speed      # build/flywheel_control/speed/ (host + firmware)
```

Start the socat bridges as described in the repository-root `AGENTS.md`:

```bash
# Edit framework/common/mdcs_config.h if your PTY links live elsewhere, then:
python3 scripts/gen_mdcs_config.py
./scripts/socat_bridges.sh
```

Run the binary from the model directory:

```bash
../../../build/flywheel_control/flywheel_position_sim
../../../build/flywheel_control/flywheel_speed_sim
../../../build/flywheel_control/position/flywheel_hil_sim   # HIL, MCU required
../../../build/flywheel_control/speed/flywheel_hil_sim      # HIL, MCU required
```

Keys while running:

- `S` — enable the compensator
- `T` — disable the compensator
- `Q` — quit

Telemetry format:

```
<theta  theta_dot  command_voltage  torque>
```

The Python visualizers read from the socat PTY links (from `framework/common/mdcs_config.h`) at 115200 baud.

---

## 17. Python design script

`docs/flywheel_lqi_design.py` is the regenerate path for every number in this document. Run it from the model directory:

```bash
python3 docs/flywheel_lqi_design.py
```

It prints:

1. Physical parameters and derived $I_{tot}$, $\alpha$, $\beta$, $\tau$.
2. Plant matrices $\mathbf{A}$, $\mathbf{B}$, $\mathbf{C}$.
3. Position $\mathbf{A}_{aug}$, $\mathbf{B}_{aug}$, the ARE solution $\mathbf{P}$, the gain $\mathbf{K}$, and the continuous/discrete poles.
4. The velocity-filter poles.
5. A comparison of the aggressive, current, and Bryson position weights.
6. The feedforward derivation and the physical torque balance.
7. The textbook speed LQI and the implemented PI, with and without the filter.
8. The 20-bit quantization and filter-noise numbers.
9. C-ready `#define` lines.
10. A 1 kHz time-domain check.

The key output for the current design is:

```
Position:
  K = [11.3836, 1.9305, 7.0711]
  poles = [-18.632682, -6.558357, -0.711772] 1/s
  #define K_POS  11.3836f
  #define K_VEL  1.9305f
  #define K_INT  7.0711f

Speed:
  K2_SPEED = -1.0000
  KI_SPEED = -0.0500
  V_FF     =  2.8250 V
  poles = [-15.787811, -0.038956] 1/s
```

To change the design, edit the parameters or weights at the top of the script, re-run it, and copy the printed `#define` lines into the matching C compensator. Then rebuild and run.

---

## 18. Summary — why this control scheme

The flywheel has two control objectives with different plant structures, so it uses two different — but mathematically related — controllers.

**Position is second-order.** The voltage drives acceleration; acceleration integrates to speed; speed integrates to angle. A full LQI feeds back all three relevant quantities: position error, velocity, and the integral of position error. The velocity term supplies damping, the position term supplies stiffness, and the integral term removes the standing error caused by friction, deadzone mismatch, quantization, and model error. The gains are chosen by the ARE with a deliberate control-effort penalty ($R=1$) that keeps the loop off the rail and below the velocity-filter corner. The result is a 0.00021° hold error, 4.8 s settling to 1°, and 0.01% rail saturation.

**Speed is first-order.** The voltage drives speed directly through a first-order lag. For a first-order plant, LQI has only two states — speed and the integral of speed error — which is exactly PI. There is no second plant state to justify a larger state-feedback structure. The feedforward computes the exact steady-state voltage (2.425 V effective, 2.825 V requested) from the torque balance, so the feedback only has to trim a small residual. The PI gains are deliberately soft because the velocity measurement is quantized and filtered; the aggressive LQI gains would put the loop bandwidth near the filter's 25° phase-lag point. The result is 14.999561 rad/s at 5 minutes, 0.000434 rad/s standard deviation, and 0.022% rail saturation.

The split is therefore not an arbitrary mixture of techniques. It is the same LQI principle applied at the order of each plant:

- **Second-order plant → full LQI** (position, velocity, integral).
- **First-order plant → LQI degenerates to PI**, with feedforward carrying the operating point.

Both loops use the same 20-bit encoder, the same 1 kHz sample rate, the same deadzone inverse, and the same actuator saturation model. The difference is the plant order and the resulting controller structure.