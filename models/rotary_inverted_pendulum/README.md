<!--
Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
-->

# Furuta (Rotary Inverted Pendulum)

Model directory: `models/rotary_inverted_pendulum/`.

## Build

| Command | Output |
|---|---|
| `make` | SIL simulator: `build/furuta/furuta_sim` |
| `make -f Makefile_HIL` | HIL host: `build/furuta/furuta_sim_hil` plus STM32F411 firmware (`MDCS_HIL.out`, `MDCS_HIL.bin`, `output.map`) in the same folder |

`make clean` (either Makefile) removes the whole `build/furuta/` folder, including both SIL and HIL artifacts.

## Run

Create the socat bridges first (see the repo-root `README.md` or `AGENTS.md`), then from this directory:

```bash
../../build/furuta/furuta_sim
```

Keys while running: `S` = enable compensator, `T` = disable, `Q` = quit.

Telemetry is the 6-value stream `<theta1 theta2 theta1_dot theta2_dot torque command_voltage>` (arm angle, pendulum angle, their rates, torque, commanded voltage); the 3D animation reads `/home/dipto/sim_3d` and the plotter reads `/home/dipto/sim_graph`, both at 115200 baud.

## Gain derivation (Python)

The LQR design script is embedded in `models/rotary_inverted_pendulum/docs/Complete_derivation_Rotary_Inverted_Pendulum.pdf` §5 (pages 11-12). It reproduces all four checked-in gains (`k1`, `k2`, `k3`, `k4`).

## HIL

Build the host binary and the MCU firmware with `make -f Makefile_HIL`; the MCU law is `compensator/compensator_STM32F411.c`. The 12-byte frame format, flashing steps (`0x08000000`), and runtime notes are in the repo-root `README.md` §6.

**Known firmware exception:** this is the only MCU law that still pulls in `__aeabi_d*` soft-double helpers. It includes `<math.h>`, which already defines `M_PI` as a `double`, so the local `#ifndef M_PI` float fallback never applies. The firmware builds and runs, but it does not meet the float-only rule in `README.md` §6.5 that the other three models follow.
