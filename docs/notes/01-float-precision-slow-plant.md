# A 20 kHz integrator that heats but never cools

**Symptom.** The thermal slab heats normally. Press `T` to switch the heaters off and the temperature stops moving — not slow, frozen. It stays at whatever value it reached, indefinitely.

**Plant.** Industrial furnace model, integrated at 20 kHz with RK4:

| Parameter | Value |
|---|---|
| `C1` (thermal mass) | 500 J/K |
| `ha1_A1` (loss coefficient) | 0.5 W/K |
| `Tamb` | 25 °C |
| `Vmax`, `R` (heater) | 12 V, 4 Ω → 36 W at full duty |
| Time constant | ≈ 1055 s |

One 50 µs step moves the temperature by

```
dT = (ha1_A1 / C1) * (T - Tamb) * dt
   = 0.001 * (T - 25) * 5e-5
```

At the observed freeze point (31.868 °C) that is **3.43e-7 °C per step**.

A `float` near 32 °C has an ULP of 2⁻¹⁹ = **1.907e-6 °C**, so half an ULP is **9.54e-7 °C**. The increment is smaller than half an ULP:

```c
x->T1 += 3.43e-7f;   // rounds straight back to x->T1
```

The state is not converging slowly. It is not converging at all — every step is discarded by the rounding of the store.

## Why heating still worked

Heating delivers 36 W into 500 J/K:

```
dT = (36 / 500) * 5e-5 = 3.6e-6 °C per step
```

That is above the half-ULP at 32 °C, so energy going **in** was integrated while energy going **out** was not. The result looks like a plant with no losses, until the heaters turn off.

The same arithmetic predicts a second failure that was never reached in this run: above 64 °C the ULP doubles to 7.63e-6, half-ULP becomes 3.81e-6, and even the heating increment (3.6e-6) falls below it. A float state would asymptote below 64 °C and never reach a 75 °C setpoint. That is arithmetic, not a measurement — the observed run froze at 31.868 °C first.

## Why the bench test passed

`BENCH_TEST_ACCELERATED` uses `C1 = 5` instead of 500 — a 100× smaller thermal mass. The same cooldown becomes 3.4e-5 °C per step, far above the ULP, so a float state cools correctly and the bug is invisible. The fast test configuration masked a defect that only exists in the slow, realistic one.

## Fix

Keep the integrated state in `double`; keep everything that crosses the compensator boundary in `float`.

```c
/* simulator/furnace_types.h */
typedef struct {
    double T1, T2;
    double T1_dot, T2_dot;
    double u1, u2;
    double q12;
} ThermalState;

typedef struct {
    float C1, C2, K12, ha1_A1, ha2_A2, Tamb, Vmax, R;
} ThermalParams;
```

```c
/* simulator/furnace_sim.c — at the 1 kHz compensator call */
float measured_T1 = (float)(s.T1 + jitter1);
float measured_T2 = (float)(s.T2 + jitter2);
```

Only the host integrates the plant, so this costs nothing on the MCU. `ThermalParams` and the compensator stay `float`, which keeps the SIL control law bit-comparable to the firmware law. Changing the state to `double` does **not** change the gains.

## The general rule

For a fixed-step integrator in single precision, a state update is lost when

```
|dT/dt| * dt  <  0.5 * ULP(T)
```

with `ULP(T) ≈ 2^(floor(log2 T) - 23)`. Two ways out:

1. **Raise the precision of the integrated state** (this fix). Costs memory bandwidth, nothing else on a desktop.
2. **Integrate slow states at a slower rate.** Sub-stepping a 1055 s plant at 20 kHz is mostly wasted work anyway — a 1 kHz or 100 Hz thermal integrator with a larger dt would resolve the same dynamics.

Option 1 was chosen here because the loop is shared with the fast electromechanical models and the uniformity is worth more than the cycles.

## How to check for it

- Command a step, then remove the input and watch for *any* decay over 60 s. A perfectly flat line after the input is removed is not a slow pole; it is a rounding floor.
- Print `nextafterf(x, INFINITY) - x` at your operating point. If your per-step increment is smaller than half of that, the state cannot move.
- Run the same test with the fast parameter set. If it passes there and fails with realistic parameters, suspect precision before suspecting the physics.

## Scope

This is a host-side numerical issue only. The MCU never integrates the plant — it receives measured temperatures and returns duty cycles — so the firmware stays single precision throughout. See `04-bare-metal-float-only-port.md` for why that constraint is enforced.
