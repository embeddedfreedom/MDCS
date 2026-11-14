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

Telemetry is the 6-value stream `<theta theta_dot command_voltage torque>`; the 3D animation reads `/home/dipto/sim_3d` and the plotter reads `/home/dipto/sim_graph`, both at 115200 baud.

## Gain derivation (Python)

The LQR design script is embedded in `models/rotary_inverted_pendulum/docs/Complete_derivation_Rotary_Inverted_Pendulum.pdf` §5 (pages 11-12). It reproduces all four checked-in gains (`k1`, `k2`, `k3`, `k4`).

## HIL

See `docs/hil_control.md` for HIL build and flash instructions.
