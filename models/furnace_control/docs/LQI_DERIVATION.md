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

# Furnace LQI Compensator — Full Derivation

**Target reader:** Electronics engineering graduate with limited mechanics background.  
**Goal:** Derive the furnace controller from first principles, ending at the C code in `compensator/compensator.c`.

---

## 1. What we are controlling

A metal slab split into two contiguous nodes (Zone 1 and Zone 2). Each zone has:

- A heater that injects power `u` (watts)
- Thermal mass (heat capacity `C`) that resists temperature change
- Thermal link `K12` conducting heat between the two zones
- Convection to ambient air (`ha·A`) losing heat to the surroundings

**MIMO system:** Two inputs (`u1`, `u2`), two outputs (`T1`, `T2`). They are coupled — Zone 1's temperature affects Zone 2 through `K12`, and vice-versa.

---

## 2. Governing equations (energy balance)

Apply conservation of energy to each node:

$\sum \dot{Q} = C \frac{dT}{dt}$

### Zone 1

$C_1 \dot{T}_1 = u_1 - K_{12}(T_1 - T_2) - h_{a1}A_{a1}(T_1 - T_{amb})$

- `u1`: heater power into Zone 1
- `K12(T1 − T2)`: heat conducted from Zone 1 to Zone 2
- `ha1A1(T1 − Tamb)`: heat lost from Zone 1 to ambient air

### Zone 2

$C_2 \dot{T}_2 = u_2 - K_{12}(T_2 - T_1) - h_{a2}A_{a2}(T_2 - T_{amb})$

- `u2`: heater power into Zone 2
- `K12(T2 − T1)`: heat conducted from Zone 2 to Zone 1 (same link, opposite direction)
- `ha2A2(T2 − Tamb)`: heat lost from Zone 2 to ambient air

---

## 2.5. Heater Actuator Model (PWM → Voltage → Power)

In a real furnace, the controller does not command power directly. It commands a **PWM duty cycle** (`pwm ∈ [0, 1]`), which drives a voltage, which produces electrical power:

**PWM → Voltage:**
$V = V_{max} \cdot pwm$

**Voltage → Power (Ohm's law + Joule heating):**
$P = \frac{V^2}{R} = \frac{(V_{max} \cdot pwm)^2}{R}$

where:

- `Vmax` = maximum heater voltage (V)
- `R` = heater resistance (ohms)
- `P` = electrical power into the slab (W)

The full energy balance with PWM input becomes:

$C_1 \dot{T}_1 = \frac{(V_{max} \cdot pwm_1)^2}{R} - K_{12}(T_1 - T_2) - h_{a1}A_{a1}(T_1 - T_{amb})$

$C_2 \dot{T}_2 = \frac{(V_{max} \cdot pwm_2)^2}{R} - K_{12}(T_2 - T_1) - h_{a2}A_{a2}(T_2 - T_{amb})$

This is **nonlinear** in `pwm` (quadratic). For LQR/LQI design, we linearize around the operating point:

**Linearized B matrix:**
$\frac{dP}{dpwm} = \frac{2 \cdot V_{max}^2 \cdot pwm_{sp}}{R}$

$\frac{dT}{dpwm} = \frac{dP}{dpwm} \cdot \frac{1}{C} = \frac{2 \cdot V_{max}^2 \cdot pwm_{sp}}{R \cdot C}$

So the linearized B matrix entries are:
$B_{11} = \frac{2 \cdot V_{max}^2 \cdot pwm_{sp1}}{R \cdot C_1}, \quad B_{22} = \frac{2 \cdot V_{max}^2 \cdot pwm_{sp2}}{R \cdot C_2}$

---

## 3. Linear state-space form (explicit partial derivatives)

We first rewrite each governing equation so the state derivatives are isolated:

### Zone 1 — solve for `dot(T1)`

$C_1 \dot{T}_1 = u_1 - K_{12}(T_1 - T_2) - h_{a1}A_{a1}(T_1 - T_{amb})$

Expand the right-hand side:

$C_1 \dot{T}_1 = u_1 - K_{12}T_1 + K_{12}T_2 - h_{a1}A_{a1}T_1 + h_{a1}A_{a1}T_{amb}$

Divide by `C1` to get the state derivative function `f1`:

$f_1(T_1,T_2,u_1,u_2) = \dot{T}_1 = -\frac{K_{12} + h_{a1}A_{a1}}{C_1}T_1 + \frac{K_{12}}{C_1}T_2 + \frac{1}{C_1}u_1 + \frac{h_{a1}A_{a1}}{C_1}T_{amb}$

### Zone 2 — solve for `dot(T2)`

$C_2 \dot{T}_2 = u_2 - K_{12}(T_2 - T_1) - h_{a2}A_{a2}(T_2 - T_{amb})$

Expand:

$C_2 \dot{T}_2 = u_2 - K_{12}T_2 + K_{12}T_1 - h_{a2}A_{a2}T_2 + h_{a2}A_{a2}T_{amb}$

Divide by `C2` to get `f2`:

$f_2(T_1,T_2,u_1,u_2) = \dot{T}_2 = \frac{K_{12}}{C_2}T_1 - \frac{K_{12} + h_{a2}A_{a2}}{C_2}T_2 + \frac{1}{C_2}u_2 + \frac{h_{a2}A_{a2}}{C_2}T_{amb}$

---

### Computing **A** via partial derivatives

The **A** matrix is the Jacobian of the state derivatives with respect to the states:

$\mathbf{A} = \frac{\partial [f_1, f_2]^T}{\partial [T_1, T_2]} = \begin{bmatrix} \frac{\partial f_1}{\partial T_1} & \frac{\partial f_1}{\partial T_2} \\ \frac{\partial f_2}{\partial T_1} & \frac{\partial f_2}{\partial T_2} \end{bmatrix}$

Compute each term:

**Row 1 — derivatives of f1:**

$\frac{\partial f_1}{\partial T_1} = -\frac{K_{12} + h_{a1}A_{a1}}{C_1}$

(T1 appears in the first term only; T2 and u terms vanish)

$\frac{\partial f_1}{\partial T_2} = \frac{K_{12}}{C_1}$

(T2 appears only in the `+K12·T2` term)

**Row 2 — derivatives of f2:**

$\frac{\partial f_2}{\partial T_1} = \frac{K_{12}}{C_2}$

(T1 appears only in the `+K12·T1` term)

$\frac{\partial f_2}{\partial T_2} = -\frac{K_{12} + h_{a2}A_{a2}}{C_2}$

(T2 appears in the first term only)

So:

$\mathbf{A} = \begin{bmatrix} -\frac{K_{12} + h_{a1}A_{a1}}{C_1} & \frac{K_{12}}{C_1} \\ \frac{K_{12}}{C_2} & -\frac{K_{12} + h_{a2}A_{a2}}{C_2} \end{bmatrix}$

---

### Computing **B** via partial derivatives

The **B** matrix is the Jacobian of the state derivatives with respect to the inputs:

$\mathbf{B} = \frac{\partial [f_1, f_2]^T}{\partial [pwm_1, pwm_2]} = \begin{bmatrix} \frac{\partial f_1}{\partial pwm_1} & \frac{\partial f_1}{\partial pwm_2} \\ \frac{\partial f_2}{\partial pwm_1} & \frac{\partial f_2}{\partial pwm_2} \end{bmatrix}$

Compute each term:

$\frac{\partial f_1}{\partial pwm_1} = \frac{2 \cdot V_{max}^2 \cdot pwm_1}{R \cdot C_1}, \frac{\partial f_1}{\partial pwm_2} = 0$

(f1 depends on pwm1 only; pwm2 does not appear in f1)

$\frac{\partial f_2}{\partial pwm_1} = 0, \frac{\partial f_2}{\partial pwm_2} = \frac{2 \cdot V_{max}^2 \cdot pwm_2}{R \cdot C_2}$

(f2 depends on pwm2 only; pwm1 does not appear in f2)

Evaluated at the operating point ($pwm_{sp1}, pwm_{sp2}$):

$\mathbf{B} = \begin{bmatrix} \frac{2 \cdot V_{max}^2 \cdot pwm_{sp1}}{R \cdot C_1} & 0 \\ 0 & \frac{2 \cdot V_{max}^2 \cdot pwm_{sp2}}{R \cdot C_2} \end{bmatrix}$

The ambient term becomes a constant disturbance vector:

$\mathbf{d} = \begin{bmatrix} \frac{h_{a1}A_{a1}}{C_1}T_{amb} \\ \frac{h_{a2}A_{a2}}{C_2}T_{amb} \end{bmatrix}$

---

### State and input vectors

$\mathbf{x} = \begin{bmatrix} T_1 \\ T_2 \end{bmatrix}, \mathbf{u} = \begin{bmatrix} pwm_1 \\ pwm_2 \end{bmatrix}$

### Expand Zone 1 equation

$C_1 \dot{T}_1 = \frac{(V_{max} \cdot pwm_1)^2}{R} - K_{12}T_1 + K_{12}T_2 - h_{a1}A_{a1}T_1 + h_{a1}A_{a1}T_{amb}$

Linearized around operating point:

$\dot{T}_1 = -\frac{K_{12} + h_{a1}A_{a1}}{C_1} T_1 + \frac{K_{12}}{C_1} T_2 + \frac{2 \cdot V_{max}^2 \cdot pwm_{sp1}}{R \cdot C_1} pwm_1 + \frac{h_{a1}A_{a1}}{C_1} T_{amb}$

### Expand Zone 2 equation

$C_2 \dot{T}_2 = u_2 - K_{12}T_2 + K_{12}T_1 - h_{a2}A_{a2}T_2 + h_{a2}A_{a2}T_{amb}$

Group `T2` terms:

$C_2 \dot{T}_2 = u_2 + K_{12}T_1 - (K_{12} + h_{a2}A_{a2})T_2 + h_{a2}A_{a2}T_{amb}$

Divide by `C2`:

$\dot{T}_2 = \frac{K_{12}}{C_2} T_1 - \frac{K_{12} + h_{a2}A_{a2}}{C_2} T_2 + \frac{1}{C_2} u_2 + \frac{h_{a2}A_{a2}}{C_2} T_{amb}$

### Matrix form

$\begin{bmatrix} \dot{T}_1 \\ \dot{T}_2 \end{bmatrix} = \begin{bmatrix} -\frac{K_{12} + h_{a1}A_{a1}}{C_1} & \frac{K_{12}}{C_1} \\ \frac{K_{12}}{C_2} & -\frac{K_{12} + h_{a2}A_{a2}}{C_2} \end{bmatrix} \begin{bmatrix} T_1 \\ T_2 \end{bmatrix} + \begin{bmatrix} \frac{2 \cdot V_{max}^2 \cdot pwm_{sp1}}{R \cdot C_1} & 0 \\ 0 & \frac{2 \cdot V_{max}^2 \cdot pwm_{sp2}}{R \cdot C_2} \end{bmatrix} \begin{bmatrix} pwm_1 \\ pwm_2 \end{bmatrix} + \begin{bmatrix} \frac{h_{a1}A_{a1}}{C_1} T_{amb} \\ \frac{h_{a2}A_{a2}}{C_2} T_{amb} \end{bmatrix}$

where the three matrices are **A**, **B**, and **d** respectively.

The vector **d** is a constant disturbance from ambient temperature. In the code we handle it via feedforward `pwm_sp` (see Section 8).

---

## 4. The problem with plain LQR

Standard LQR minimizes:

$J = \int_0^\infty (\mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u}) \, dt$

It drives **x to zero**. But our setpoint is not zero — it is `r = [75, 75]^T` °C. If we simply regulate `T1 = T2 = 0`, the furnace cools to room temperature.

Even if we regulate the error `e = x − r` to zero, a **constant disturbance** (ambient loss) creates steady-state offset: the heaters need to supply continuous power just to hold temperature, but pure state feedback `u = −Kx` produces zero output when error is zero.

**Solution:** Add integral action. This makes it **LQI** (Linear Quadratic Integral).

---

## 5. Integral error state (LQI augmentation)

Define the tracking error:

$\mathbf{e}(t) = \mathbf{C}\mathbf{x}(t) - \mathbf{r}$

where **C** selects the measured temperatures (here, identity, since we measure both T1 and T2 directly):

$\mathbf{C} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$

Define integral states **q**:

$\mathbf{q}(t) = \int_0^t \mathbf{e}(\tau) \, d\tau = \int_0^t (\mathbf{C}\mathbf{x}(\tau) - \mathbf{r}) \, d\tau$

Differentiating:

$\dot{\mathbf{q}} = \mathbf{C}\mathbf{x} - \mathbf{r}$

### Augmented state vector

Combine plant states and integral states:

$\mathbf{x}_{aug} = \begin{bmatrix} \mathbf{x} \\ \mathbf{q} \end{bmatrix} = \begin{bmatrix} T_1 \\ T_2 \\ q_1 \\ q_2 \end{bmatrix}$

### Augmented system matrices

$\dot{\mathbf{x}}_{aug} = \begin{bmatrix} \dot{\mathbf{x}} \\ \dot{\mathbf{q}} \end{bmatrix} = \begin{bmatrix} \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} \\ \mathbf{C}\mathbf{x} - \mathbf{r} \end{bmatrix}$

In block form:

$\mathbf{A}_{aug} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ \mathbf{C} & \mathbf{0} \end{bmatrix}, \mathbf{B}_{aug} = \begin{bmatrix} \mathbf{B} \\ \mathbf{0} \end{bmatrix}$

Written out (4×4):

$\mathbf{A}_{aug} = \begin{bmatrix} a_{11} & a_{12} & 0 & 0 \\ a_{21} & a_{22} & 0 & 0 \\ 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}, \mathbf{B}_{aug} = \begin{bmatrix} b_{11} & 0 \\ 0 & b_{22} \\ 0 & 0 \\ 0 & 0 \end{bmatrix}$

where:

- `a11 = −(K12 + ha1A1)/C1`, `a12 = K12/C1`
- `a21 = K12/C2`, `a22 = −(K12 + ha2A2)/C2`
- `b11 = 1/C1`, `b22 = 1/C2`

### Block-by-block construction of A_aug

The augmented matrix stacks the plant dynamics on top of the integral dynamics:

$\mathbf{A}_{aug} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ \mathbf{C} & \mathbf{0} \end{bmatrix}$

**Top-left block (2×2):** The plant matrix **A** — describes how T1 and T2 evolve from themselves:

$\begin{bmatrix} -\frac{K_{12} + h_{a1}A_{a1}}{C_1} & \frac{K_{12}}{C_1} \\ \frac{K_{12}}{C_2} & -\frac{K_{12} + h_{a2}A_{a2}}{C_2} \end{bmatrix}$

**Top-right block (2×2):** All zeros — the plant states do not directly depend on the integral states:

$\begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$

**Bottom-left block (2×2):** The output matrix **C** — the integral states q̇ depend directly on the measured temperatures:

$\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$

This means q̇₁ = T₁ and q̇₂ = T₂ (minus setpoint, handled separately).

**Bottom-right block (2×2):** All zeros — the integral states do not depend on themselves:

$\begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$

Stacking them gives the full 4×4:

$\mathbf{A}_{aug} = \begin{bmatrix} -\frac{K_{12} + h_{a1}A_{a1}}{C_1} & \frac{K_{12}}{C_1} & 0 & 0 \\ \frac{K_{12}}{C_2} & -\frac{K_{12} + h_{a2}A_{a2}}{C_2} & 0 & 0 \\ 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$

Similarly for **B_aug**:

$\mathbf{B}_{aug} = \begin{bmatrix} \mathbf{B} \\ \mathbf{0} \end{bmatrix} = \begin{bmatrix} \frac{1}{C_1} & 0 \\ 0 & \frac{1}{C_2} \\ 0 & 0 \\ 0 & 0 \end{bmatrix}$

**Top block (2×2):** **B** — heater inputs u1, u2 directly affect Ṫ₁, Ṫ₂.

**Bottom block (2×2):** All zeros — heater inputs do not directly affect q̇₁, q̇₂ (they affect q only indirectly through T).

---

## 6. LQI cost function and Riccati equation

We minimize the infinite-horizon quadratic cost:

$J = \int_0^\infty (\mathbf{x}_{aug}^T \mathbf{Q}_{aug} \mathbf{x}_{aug} + \mathbf{u}^T \mathbf{R} \mathbf{u}) dt$

### Weighting matrices

- **Q_aug** (4×4): penalizes temperature errors and integral errors
- **R** (2×2): penalizes heater effort

### Continuous-time Algebraic Riccati Equation (ARE)

Solve for positive-definite **P** (4×4):

$\mathbf{A}_{aug}^T \mathbf{P} + \mathbf{P}\mathbf{A}_{aug} - \mathbf{P}\mathbf{B}_{aug}\mathbf{R}^{-1}\mathbf{B}_{aug}^T\mathbf{P} + \mathbf{Q}_{aug} = \mathbf{0}$

### Optimal gain

$\mathbf{K}_{aug} = \mathbf{R}^{-1}\mathbf{B}_{aug}^T\mathbf{P}$

Partition the gain:

$\mathbf{K}_{aug} = \begin{bmatrix} \mathbf{K}_x & \mathbf{K}_i \end{bmatrix}$

- **Kx** (2×2): state feedback on temperature errors
- **Ki** (2×2): integral feedback on accumulated errors

---

## 7. Control law

$\mathbf{pwm}(t) = \mathbf{pwm}_{sp} - \mathbf{K}_x \mathbf{e}(t) - \mathbf{K}_i \mathbf{q}(t)$

Expanded:

$\begin{bmatrix} pwm_1 \\ pwm_2 \end{bmatrix} = \begin{bmatrix} pwm_{sp1} \\ pwm_{sp2} \end{bmatrix} - \begin{bmatrix} k_{x00} & k_{x01} \\ k_{x10} & k_{x11} \end{bmatrix} \begin{bmatrix} T_1 - T_{1sp} \\ T_2 - T_{2sp} \end{bmatrix} - \begin{bmatrix} k_{i00} & k_{i01} \\ k_{i10} & k_{i11} \end{bmatrix} \begin{bmatrix} q_1 \\ q_2 \end{bmatrix}$

Integral update (discrete, 1 kHz):

$q_1 \leftarrow q_1 + (T_1 - T_{1sp}) \cdot 0.001$

$q_2 \leftarrow q_2 + (T_2 - T_{2sp}) \cdot 0.001$

---

## 7.5. From math to C code — how `compensator.c` calculates `pwm1` and `pwm2`

Here is the actual code from `compensator/compensator.c` with each mathematical term mapped to its C equivalent.

### Tracking error: `e = x − r`

Math:
$e_1 = T_1 - T_{1sp}, e_2 = T_2 - T_{2sp}$

C code:

```c
float e1 = measured_T1 - T1_SETPOINT;
float e2 = measured_T2 - T2_SETPOINT;
```

### Integral states: `q̇ = e` (discrete integration)

Math:
$q_1 \leftarrow q_1 + e_1 \cdot 0.001, q_2 \leftarrow q_2 + e_2 \cdot 0.001$

C code (with clamping for anti-windup):

```c
float q1_next = q1 + e1 * 0.001f;
float q2_next = q2 + e2 * 0.001f;
clampf(&q1_next, -INT_CLAMP, INT_CLAMP);
clampf(&q2_next, -INT_CLAMP, INT_CLAMP);
```

### State feedback: `−Kx · e`

Math:
$-K_x \mathbf{e} = -\begin{bmatrix} k_{x00} & k_{x01} \\ k_{x10} & k_{x11} \end{bmatrix} \begin{bmatrix} e_1 \\ e_2 \end{bmatrix}$

Expanded:
$-(k_{x00}\cdot e_1 + k_{x01}\cdot e_2), -(k_{x10}\cdot e_1 + k_{x11}\cdot e_2)$

C code:

```c
-(kx00 * e1 + kx01 * e2)
-(kx10 * e1 + kx11 * e2)
```

### Integral feedback: `−Ki · q`

Math:
$-K_i \mathbf{q} = -\begin{bmatrix} k_{i00} & k_{i01} \\ k_{i10} & k_{i11} \end{bmatrix} \begin{bmatrix} q_1 \\ q_2 \end{bmatrix}$

Expanded:
$-(k_{i00}\cdot q_1 + k_{i01}\cdot q_2), -(k_{i10}\cdot q_1 + k_{i11}\cdot q_2)$

C code:

```c
-(ki00 * q1_next + ki01 * q2_next)
-(ki10 * q1_next + ki11 * q2_next)
```

### Feedforward: `+ pwm_sp`

Math: steady-state PWM duty cycle to hold setpoint against ambient losses.

Steady-state power needed: $P = (K_{12} + h_{a1}A_{a1})T_{sp} - K_{12}T_{other} - h_{a1}A_{a1}T_{amb}$

Convert to PWM: $pwm = \sqrt{P \cdot R} / V_{max}$

C code (computed once at init):

```c
float p_sp1 = (p->K12 + p->ha1_A1) * T1_SETPOINT
            - p->K12 * T2_SETPOINT
            - p->ha1_A1 * p->Tamb;
float p_sp2 = (p->K12 + p->ha2_A2) * T2_SETPOINT
            - p->K12 * T1_SETPOINT
            - p->ha2_A2 * p->Tamb;

if (p_sp1 > 0.0f)
    pwm_sp1 = sqrtf(p_sp1 * p->R) / p->Vmax;
else
    pwm_sp1 = 0.0f;

if (p_sp2 > 0.0f)
    pwm_sp2 = sqrtf(p_sp2 * p->R) / p->Vmax;
else
    pwm_sp2 = 0.0f;

clampf(&pwm_sp1, PWM_MIN, PWM_MAX);
clampf(&pwm_sp2, PWM_MIN, PWM_MAX);
```

### Full control law assembled

Math:
$\mathbf{pwm} = \mathbf{pwm}_{sp} - \mathbf{K}_x \mathbf{e} - \mathbf{K}_i \mathbf{q}$

C code:

```c
float out1 = pwm_sp1
           - (kx00 * e1 + kx01 * e2)
           - (ki00 * q1_next + ki01 * q2_next);
float out2 = pwm_sp2
           - (kx10 * e1 + kx11 * e2)
           - (ki10 * q1_next + ki11 * q2_next);
```

### Output saturation (PWM duty cycle: 0 to 1)

Math: clamp each output to `[PWM_MIN, PWM_MAX]`

C code:

```c
clampf(&out1, PWM_MIN, PWM_MAX);   /* PWM_MIN = 0.0f, PWM_MAX = 1.0f */
clampf(&out2, PWM_MIN, PWM_MAX);
```

### Anti-windup: only commit integral update if not saturated

Math: if the heater is at its limit, stop integrating the error to prevent windup.

C code:

```c
bool sat1 = (out1 <= PWM_MIN || out1 >= PWM_MAX);
bool sat2 = (out2 <= PWM_MIN || out2 >= PWM_MAX);
if (!sat1) q1 = q1_next;   /* commit integral only if not saturated */
if (!sat2) q2 = q2_next;
```

### Final output

C code:

```c
*pwm1 = out1;
*pwm2 = out2;
```

---

## 7.6. Worked numerical example — calculating `pwm1` and `pwm2` with real numbers

Let's walk through one complete 1 kHz cycle using the industrial parameters and the hardcoded gains.

### Given

**Plant parameters (industrial):**

- C1 = 500, C2 = 450, K12 = 2.5, ha1A1 = 0.5, ha2A2 = 0.4, Tamb = 25

**Setpoints:**

- T1sp = 75°C, T2sp = 75°C

**Gains (from Riccati solution, Section 10):**

```
Kx = [[46.145,  2.223],
     [ 2.470, 44.159]]
Ki = [[ 2.000, -0.005],
     [ 0.005,  2.000]]
```

**Limits:** U_MIN = 0, U_MAX = 100

**Current cycle (1st cycle, cold start):**

- measured_T1 = 50.0°C, measured_T2 = 45.0°C
- q1 = 0, q2 = 0 (no integral yet)
- dt = 0.001 s

---

### Step 1: Compute feedforward `pwm_sp` (done once at init)

$P_{sp1} = (K_{12} + h_{a1}A_{a1})T_{1sp} - K_{12}T_{2sp} - h_{a1}A_{a1}T_{amb}$

$P_{sp1} = (2.5 + 0.5) \times 75 - 2.5 \times 75 - 0.5 \times 25$

$u_{sp1} = 3.0 \times 75 - 187.5 - 12.5 = 225 - 200 = \mathbf{25.0}$

$P_{sp2} = (K_{12} + h_{a2}A_{a2})T_{2sp} - K_{12}T_{1sp} - h_{a2}A_{a2}T_{amb}$

$P_{sp2} = (2.5 + 0.4) \times 75 - 2.5 \times 75 - 0.4 \times 25$

$u_{sp2} = 2.9 \times 75 - 187.5 - 10 = 217.5 - 197.5 = \mathbf{20.0}$

---

### Step 2: Compute tracking error `e = x − r`

$e_1 = T_1 - T_{1sp} = 50.0 - 75.0 = \mathbf{-25.0}$

$e_2 = T_2 - T_{2sp} = 45.0 - 75.0 = \mathbf{-30.0}$

(Negative error means temperature is below setpoint → heaters should turn on)

---

### Step 3: Update integral states `q`

$q_{1\_next} = q_1 + e_1 \times dt = 0 + (-25.0) \times 0.001 = \mathbf{-0.025}$

$q_{2\_next} = q_2 + e_2 \times dt = 0 + (-30.0) \times 0.001 = \mathbf{-0.030}$

---

### Step 4: State feedback term `−Kx · e`

For heater 1:
$-(k_{x00} \cdot e_1 + k_{x01} \cdot e_2) = -(46.145 \times (-25.0) + 2.223 \times (-30.0))$

$= -(-1153.625 + (-66.69)) = -(-1220.315) = \mathbf{+1220.315}$

For heater 2:
$-(k_{x10} \cdot e_1 + k_{x11} \cdot e_2) = -(2.470 \times (-25.0) + 44.159 \times (-30.0))$

$= -(-61.75 + (-1324.77)) = -(-1386.52) = \mathbf{+1386.52}$

---

### Step 5: Integral feedback term `−Ki · q`

For heater 1:
$-(k_{i00} \cdot q_1 + k_{i01} \cdot q_2) = -(2.000 \times (-0.025) + (-0.005) \times (-0.030))$

$= -(-0.050 + 0.00015) = -(-0.04985) = \mathbf{+0.050}$

For heater 2:
$-(k_{i10} \cdot q_1 + k_{i11} \cdot q_2) = -(0.005 \times (-0.025) + 2.000 \times (-0.030))$

$= -(-0.000125 + (-0.060)) = -(-0.060125) = \mathbf{+0.060}$

---

### Step 6: Assemble total control `pwm = pwm_sp − Kx·e − Ki·q`

**Heater 1:**
$out1 = 25.0 + 1220.315 + 0.050 = \mathbf{1245.365}$

**Heater 2:**
$out2 = 20.0 + 1386.52 + 0.060 = \mathbf{1406.580}$

---

### Step 7: Apply saturation `[0, 100]`

Both values exceed 100, so they clamp:

$pwm_1 = clamp(1245.365, 0, 1) = \mathbf{1.0}$

$pwm_2 = clamp(1406.580, 0, 1) = \mathbf{1.0}$

**Result: both heaters at 100%** — this matches the telemetry we saw during cold start in the 5-minute test.

---

### Step 8: Anti-windup decision

Since both outputs saturated (hit U_MAX = 100), the integral states are **not committed**:

- q1 stays at 0 (not −0.025)
- q2 stays at 0 (not −0.030)

This prevents integral windup while the heaters are pegged at maximum. Once the temperature approaches setpoint and the output comes out of saturation, integration resumes normally.

---

### Bench parameter example (BENCH_TEST_ACCELERATED = 1)

For faster bench testing, the plant uses smaller thermal masses:

- C1 = 5, C2 = 4.5, K12 = 2.5, ha1A1 = 0.5, ha2A2 = 0.4, Tamb = 25

**A matrix:**
$A = \begin{bmatrix} -\frac{2.5 + 0.5}{5} & \frac{2.5}{5} \\ \frac{2.5}{4.5} & -\frac{2.5 + 0.4}{4.5} \end{bmatrix} = \begin{bmatrix} -0.6 & 0.5 \\ 0.5556 & -0.6444 \end{bmatrix}$

**B matrix** (linearized at operating point pwm_sp=[0.8333, 0.7454], dT/dpwm = 2·Vmax²·pwm_sp/(R·C)):
$B = \begin{bmatrix} \frac{1}{5} & 0 \\ 0 & \frac{1}{4.5} \end{bmatrix} = \begin{bmatrix} 0.2 & 0 \\ 0 & 0.2222 \end{bmatrix}$

Solving the ARE with the same Q/R weights gives:

$K_x = \begin{bmatrix} 17.861 & 2.033 \\ 2.259 & 17.895 \end{bmatrix}$

$K_i = \begin{bmatrix} 2.000 & -0.011 \\ 0.011 & 2.000 \end{bmatrix}$

**Feedforward:**
$u_{sp1} = (2.5 + 0.5) \times 75 - 2.5 \times 75 - 0.5 \times 25 = 25.0$
$u_{sp2} = (2.5 + 0.4) \times 75 - 2.5 \times 75 - 0.4 \times 25 = 20.0$

Note: pwm_sp is the same as industrial because it depends on temperatures, conductances, Vmax and R, not on capacitance. The gains are smaller because the bench plant responds faster (lower thermal mass = less gain needed).

**Closed-loop poles:** [-4.063, -4.534, -0.097, -0.100] — stable, but with two slow poles from the integral states.

---

## 8. Feedforward `pwm_sp` (steady-state PWM duty cycle)

To hold temperature `r` against ambient losses, the heaters must supply continuous power even when error is zero.

**Power → PWM:**
$P = \frac{(V_{max} \cdot pwm)^2}{R} \implies pwm = \frac{\sqrt{P \cdot R}}{V_{max}}$

where `Vmax` = maximum heater voltage (V) and `R` = heater resistance (ohms).

Since B is diagonal (power form):

$P_{sp1} = (K_{12} + h_{a1}A_{a1})T_{1sp} - K_{12}T_{2sp} - h_{a1}A_{a1}T_{amb}$

$P_{sp2} = (K_{12} + h_{a2}A_{a2})T_{2sp} - K_{12}T_{1sp} - h_{a2}A_{a2}T_{amb}$

### Step-by-step derivation of P_sp

At steady state, the temperatures are constant, so $\dot{T}_1 = \dot{T}_2 = 0$. The plant equations become:

$0 = P_1 - K_{12}(T_{1sp} - T_{2sp}) - h_{a1}A_{a1}(T_{1sp} - T_{amb})$

$0 = P_2 - K_{12}(T_{2sp} - T_{1sp}) - h_{a2}A_{a2}(T_{2sp} - T_{amb})$

Solving for P1:

$P_1 = K_{12}(T_{1sp} - T_{2sp}) + h_{a1}A_{a1}(T_{1sp} - T_{amb})$

$P_1 = K_{12}T_{1sp} - K_{12}T_{2sp} + h_{a1}A_{a1}T_{1sp} - h_{a1}A_{a1}T_{amb}$

$P_1 = (K_{12} + h_{a1}A_{a1})T_{1sp} - K_{12}T_{2sp} - h_{a1}A_{a1}T_{amb}$

Solving for P2:

$P_2 = K_{12}(T_{2sp} - T_{1sp}) + h_{a2}A_{a2}(T_{2sp} - T_{amb})$

$P_2 = K_{12}T_{2sp} - K_{12}T_{1sp} + h_{a2}A_{a2}T_{2sp} - h_{a2}A_{a2}T_{amb}$

$P_2 = (K_{12} + h_{a2}A_{a2})T_{2sp} - K_{12}T_{1sp} - h_{a2}A_{a2}T_{amb}$

### Numeric example (industrial parameters)

For T1sp = T2sp = 75°C, Tamb = 25°C:

**P_sp1:**
$P_{sp1} = (2.5 + 0.5) \times 75 - 2.5 \times 75 - 0.5 \times 25$
$P_{sp1} = 3.0 \times 75 - 187.5 - 12.5 = 225 - 200 = 25.0\text{ W}$

**P_sp2:**
$P_{sp2} = (2.5 + 0.4) \times 75 - 2.5 \times 75 - 0.4 \times 25$
$P_{sp2} = 2.9 \times 75 - 187.5 - 10 = 217.5 - 197.5 = 20.0\text{ W}$

So the heaters must supply 25 W and 20 W respectively, corresponding to PWM duty cycles of 0.8333 and 0.7454, just to hold 75°C against ambient losses — even with zero tracking error.

---

## 9. Choosing Q and R — Bryson's Rule

We need weights with consistent units. Bryson's rule:

$Q_{ii} = \frac{1}{(max acceptable error)^2}, R_{jj} = \frac{1}{(max acceptable effort)^2}$

### Temperature states (T1, T2)

Max acceptable error = 5 °C

$q_T = \frac{1}{5^2} = 0.04$

### Integral states (q1, q2)

Max acceptable integral error = 50 °C·s

$q_I = \frac{1}{50^2} = 0.0004$

### Control effort (pwm1, pwm2)

Max acceptable PWM duty cycle = 1.0

$r_u = \frac{1}{1.0^2} = 1.0$

### Result

$\mathbf{Q}_{aug} = diag(0.04, 0.04, 0.0004, 0.0004)$

$\mathbf{R} = diag(0.0001, 0.0001)$

---

## 10. Numerical solution (industrial parameters)

Using SciPy `solve_continuous_are` with `BENCH_TEST_ACCELERATED = 0`:

**Parameters:** C1=500, C2=450, K12=2.5, ha1A1=0.5, ha2A2=0.4, Tamb=25, **Vmax=12, R=4**

**A matrix:**

```
A = [[-0.0060,   0.0050],
     [ 0.0056,  -0.0064]]
```

**B matrix** (linearized at operating point pwm_sp=[0.8333, 0.7454], dT/dpwm = 2·Vmax²·pwm_sp/(R·C)):

```
B = [[0.0020,  0    ],
     [0   ,   0.0022]]
```

**Optimal gains:**

$\mathbf{K}_x = \begin{bmatrix} 20.77 & 0.04 \\ 0.04 & 20.77 \end{bmatrix}$

$\mathbf{K}_i = \begin{bmatrix} 2.00 & 0.00 \\ 0.00 & 2.00 \end{bmatrix}$

**Closed-loop poles:** all in left-half plane → stable.

**Feedforward:** pwm_sp1 = 0.8333, pwm_sp2 = 0.7454

---

### Discretization notes — continuous vs discrete LQR

The gains above are computed from the **continuous-time** ARE:

$A_{aug}^T P + P A_{aug} - P B_{aug} R^{-1} B_{aug}^T P + Q_{aug} = 0$

But the furnace simulator runs in **discrete time** at 1 kHz (dt = 0.001 s). There are two common approaches:

#### Option 1: Continuous-time gains applied in discrete time (current implementation)

The continuous-time Kx and Ki are used directly in the 1 kHz loop:

$pwm[k] = pwm_{sp} - K_x \cdot e[k] - K_i \cdot q[k]$
$q[k+1] = q[k] + e[k] \cdot dt$

This works well when the plant is **slow relative to the sample rate**. For the furnace:

- Plant time constant: ~1000 seconds (industrial) or ~10 seconds (bench)
- Sample period: 0.001 seconds
- Ratio: 1,000,000:1 or 10,000:1

The plant barely changes between samples, so continuous-time gains are an excellent approximation. This is what `compensator.c` implements.

#### Option 2: Discretize first, then solve discrete ARE

For faster plants or tighter performance, you would:

1. Discretize the plant: $A_d = e^{A \cdot dt}$, $B_d = \int_0^{dt} e^{A\tau} d\tau \cdot B$
2. Solve the discrete ARE: $A_d^T P A_d - P - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q = 0$
3. Compute $K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$

Python example for discretization:

```python
from scipy.linalg import expm, solve_discrete_are

dt = 0.001
A_d = expm(A_aug * dt)
# B_d approximation for small dt
B_d = B_aug * dt

P_d = solve_discrete_are(A_d, B_d, Q_aug, R)
K_d = np.linalg.inv(R + B_d.T @ P_d @ B_d) @ B_d.T @ P_d @ A_d
```

**Recommendation:** For the furnace (slow thermal plant), Option 1 (continuous gains in discrete loop) is simpler and accurate enough. Use Option 2 only if you see performance degradation at higher sample rates or faster plants.

---

## 11. Anti-windup

When heaters saturate (0% or 100%), the integral term keeps accumulating ("windup"), causing large overshoot when the setpoint is reached.

**Fix:** Only integrate when the output is *not* saturated:

```c
if (!saturated) {
    q = q + e * dt;
}
```

Implemented in `compensator.c` with clamping.

---

## 12. Tuning guide

| Goal                       | Adjustment                                            |
| -------------------------- | ----------------------------------------------------- |
| Faster warm-up             | Increase `q_T` or decrease `r_u`                      |
| Less heater activity       | Increase `r_u`                                        |
| Reduce steady-state offset | Increase `q_I`                                        |
| Less overshoot             | Increase `q_T` relative to `q_I`                      |
| Different setpoint         | Change `T1_SETPOINT`, `T2_SETPOINT`                   |
| Different plant (bench)    | Change `C1`, `C2` in `furnace_sim.c`; recompute gains |

---

## 13. Code mapping

| Math                   | File                        | Location                           |
| ---------------------- | --------------------------- | ---------------------------------- |
| Energy balance         | `simulator/furnace_sim.c`   | `compute_derivatives()`            |
| State-space A, B       | `simulator/furnace_sim.c`   | `compute_derivatives()`            |
| LQI control law        | `compensator/compensator.c` | `run_thermal_compensator_cycle()`  |
| Gains Kx, Ki           | `compensator/compensator.c` | `#define IND_KX_*` and `IND_KI_*`  |
| Feedforward pwm_sp     | `compensator/compensator.c` | `pwm_sp1`, `pwm_sp2` computation   |
| Integral states q1, q2 | `compensator/compensator.c` | `static float q1, q2`              |
| Anti-windup            | `compensator/compensator.c` | Saturation check on `out1`, `out2` |
| 1 kHz call             | `simulator/furnace_sim.c`   | `if (i >= 20)` block               |

---

## 13.5. Line-by-line code mapping

### `simulator/furnace_sim.c` — plant physics (derivation Sections 2–3)

| Derivation                | Equation                                                           | C code                                                                                                                                                             |
| ------------------------- | ------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Zone 1 energy balance     | $C_1\dot{T}_1 = P_1 - K_{12}(T_1-T_2) - h_{a1}A_{a1}(T_1-T_{amb})$ | `double q12 = p->K12 * (t1 - t2);`<br>`double p1 = (p->Vmax * pwm1) * (p->Vmax * pwm1) / p->R;`<br>`double dT1_dt = (p1 - q12 - p->ha1_A1 * (t1 - p->Tamb)) / p->C1;` |
| Zone 2 energy balance     | $C_2\dot{T}_2 = P_2 - K_{12}(T_2-T_1) - h_{a2}A_{a2}(T_2-T_{amb})$ | `double p2 = (p->Vmax * pwm2) * (p->Vmax * pwm2) / p->R;`<br>`double dT2_dt = (p2 + q12 - p->ha2_A2 * (t2 - p->Tamb)) / p->C2;`                                 |
| A matrix element a₁₁      | $-\frac{K_{12}+h_{a1}A_{a1}}{C_1}$                                 | `(p1 - q12 - p->ha1_A1 * (t1 - p->Tamb)) / p->C1`                                                                                                                  |
| A matrix element a₁₂      | $\frac{K_{12}}{C_1}$                                               | implicit via `q12 / C1` in dT1_dt                                                                                                                                  |
| A matrix element a₂₁      | $\frac{K_{12}}{C_2}$                                               | implicit via `q12 / C2` in dT2_dt                                                                                                                                  |
| A matrix element a₂₂      | $-\frac{K_{12}+h_{a2}A_{a2}}{C_2}$                                 | `(p2 + q12 - p->ha2_A2 * (t2 - p->Tamb)) / p->C2`                                                                                                                  |
| B matrix element b₁₁      | $\frac{2 \cdot V_{max}^2 \cdot pwm_{sp1}}{R \cdot C_1}$            | `(p->Vmax * pwm1) * (p->Vmax * pwm1) / p->R / p->C1` (linearized)                                                                                                  |
| B matrix element b₂₂      | $\frac{2 \cdot V_{max}^2 \cdot pwm_{sp2}}{R \cdot C_2}$            | `(p->Vmax * pwm2) * (p->Vmax * pwm2) / p->R / p->C2` (linearized)                                                                                                  |
| Disturbance vector d      | $\frac{h_{a1}A_{a1}}{C_1}T_{amb}$                                  | `p->ha1_A1 * p->Tamb / p->C1`                                                                                                                                      |
| Disturbance vector d      | $\frac{h_{a2}A_{a2}}{C_2}T_{amb}$                                  | `p->ha2_A2 * p->Tamb / p->C2`                                                                                                                                      |
| RK4 integration           | $\dot{x} = f(x,pwm)$                                               | `rk4_step(&s, active_pwm1, active_pwm2, physics_dt, &p);`                                                                                                          |
| 1 kHz compensator trigger | Every 20 physics steps                                             | `if (i >= 20)`                                                                                                                                                     |
| Compensator call          | `run_thermal_compensator_cycle(...)`                               | `run_thermal_compensator_cycle(measured_T1, measured_T2, &p, &pwm1, &pwm2);`                                                                                       |
| Telemetry output          | `<T1 T2 pwm1 pwm2 q12 Tamb>`                                       | `snprintf(buf, ..., "%.4f\t%.4f\t...")`                                                                                                                            |

### `compensator/compensator.c` — LQI controller (derivation Sections 5–8)

| Derivation         | Equation                                     | C code                                                                             |
| ------------------ | -------------------------------------------- | ---------------------------------------------------------------------------------- |
| Tracking error     | $e = x - r$                                  | `float e1 = measured_T1 - T1_SETPOINT;`<br>`float e2 = measured_T2 - T2_SETPOINT;` |
| Integral update    | $q \leftarrow q + e \cdot dt$                | `float q1_next = q1 + e1 * 0.001f;`<br>`float q2_next = q2 + e2 * 0.001f;`         |
| Anti-windup clamp  | $q \in [-INT\_CLAMP, INT\_CLAMP]$            | `clampf(&q1_next, -INT_CLAMP, INT_CLAMP);`                                         |
| State feedback     | $-K_x \cdot e$                               | `-(kx00 * e1 + kx01 * e2)`<br>`-(kx10 * e1 + kx11 * e2)`                           |
| Integral feedback  | $-K_i \cdot q$                               | `-(ki00 * q1_next + ki01 * q2_next)`<br>`-(ki10 * q1_next + ki11 * q2_next)`       |
| Feedforward        | $pwm_{sp} = \sqrt{P_{sp} \cdot R} / V_{max}$ | `pwm_sp1 = ...`<br>`pwm_sp2 = ...`                                                 |
| Full control law   | $pwm = pwm_{sp} - K_x e - K_i q$             | `float out1 = pwm_sp1 - (...) - (...);`<br>`float out2 = pwm_sp2 - (...) - (...);` |
| Output saturation  | $u \in [0, 100]$                             | `clampf(&out1, U_MIN, U_MAX);`<br>`clampf(&out2, U_MIN, U_MAX);`                   |
| Anti-windup commit | skip q update if saturated                   | `if (!sat1) q1 = q1_next;`<br>`if (!sat2) q2 = q2_next;`                           |
| Output assignment  | $pwm_1, pwm_2$                               | `*pwm1 = out1;`<br>`*pwm2 = out2;`                                                 |

---

## 14. Numerical precision — why `ThermalState` is `double`

The controller stays single precision to match the MCU, but the **plant state must be double**. The reason is the ratio between the integration step and the state itself.

At the top of the industrial range the fastest passive cooling rate is

$$\dot{T}_1 = -\frac{h_{a1}A_{a1}(T_1 - T_{amb})}{C_1}
           = -\frac{0.5 \times (75-25)}{500} = -0.05\ \text{°C/s}$$

so one 50 µs RK4 step moves the temperature by

$$\Delta T = 0.05 \times 5\times10^{-5} = 2.5\times10^{-6}\ \text{°C}.$$

A `float` has a 24-bit mantissa, so its spacing (ULP) near 75 °C is $2^{-17} \approx 7.6\times10^{-6}$ and half an ULP is $3.8\times10^{-6}$. The RK4 increment is *smaller than half an ULP*, so

```c
x->T1 += 2.5e-6;   /* rounds straight back to x->T1 — every step, forever */
```

does nothing. The slab still **heats** (36 W gives 3.6e-6 °C per step, which does clear the threshold), so the defect only shows up after pressing `T`: the PWM goes to zero and the temperature stops dead instead of decaying.

This is why bench mode appeared to work. With `C1 = 5` the same cooldown is 100× faster (2.5e-4 °C per step), far above the ULP, so a float state cools normally.

**Fix:** integrate the thermal state in `double` (`ThermalState` in `simulator/furnace_types.h`) while `ThermalParams` and `compensator.c` stay `float`. Only the host integrates the plant, so the MCU law keeps its single-precision, no-libm rules and SIL/HIL gain parity is unchanged.

With the fix the industrial plant behaves as derived:

| Phase | Behaviour |
|---|---|
| Warm-up (heaters saturated at 1.0) | 75 °C in ≈ 1100 s (18–19 min) |
| Steady state | `pwm1 = 0.8333`, `pwm2 = 0.7454` |
| `T` pressed at setpoint | ≈ 2.8 °C/min, slowing exponentially (τ ≈ 1055 s) |
| Bench parameters (`C1=5`, `C2=4.5`) | 75 °C in ≈ 12 s, back to ambient in ≈ 60 s |

---

## 15. Running

```bash
cd models/furnace_control
make                    # builds build/furnace_control/furnace_sim
# Start socat bridges (see AGENTS.md)
./build/furnace_control/furnace_sim
# Press S = enable compensator, T = disable, Q = quit
```

Watch telemetry on `/home/dipto/sim_graph` or animation on `/home/dipto/sim_3d`.

---

## 16. Python design script — `docs/furnace_lqi_design.py`

Students run this to recompute gains when changing Q/R weights, setpoints, or physical parameters. It prints C-format `#define` values ready to copy-paste into `compensator.c`.

```python
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
```

The script is the source of truth for both C copies. It prints the `IND_*` names used by `compensator/compensator.c`; the MCU law in `compensator/compensator_hil_STM32F4.c` carries the same gains under `KX_*`/`KI_*` and recomputes `pwm_sp1`/`pwm_sp2` at init. Update both files when retuning. The bench parameter set is a separate commented block, and the `BEN_*` gains in C are not generated by this script.

One comment in the script still says "max 100 PWM": the duty cycle is now normalized to `[0, 1]`, but the numeric weight (`1/100²`) is the checked-in value and is unchanged.
