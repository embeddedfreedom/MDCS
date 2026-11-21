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

Telemetry is the 6-value stream `<theta1 theta2 theta1_dot theta2_dot torque command_voltage>` (arm angle, pendulum angle, their rates, torque, commanded voltage); the 3D animation and plotter read the socat PTY links from `framework/common/mdcs_config.h` (edit there if you move machines), both at 115200 baud.

## Gain derivation (Python)

The LQR design script is embedded in `models/rotary_inverted_pendulum/docs/Complete_derivation_Furuta.pdf` §5 (pages 12-13). It reproduces all four checked-in gains (`k1`, `k2`, `k3`, `k4`).

The older MIT-licensed `Complete_derivation_Rotary_Inverted_Pendulum.pdf` is superseded by the GPLv3 `Complete_derivation_Furuta.pdf`; it is kept only as a local, untracked copy.

## HIL

Build the host binary and the MCU firmware with `make -f Makefile_HIL`; the MCU law is `compensator/compensator_STM32F411.c`. The 12-byte frame format, flashing steps (`0x08000000`), and runtime notes are in the repo-root `README.md` §6.

**Firmware:** the law is float-only. It includes `<math.h>` and calls `fabsf`, which compiles to a single-precision instruction; a forced rebuild (`make -B -f Makefile_HIL`) shows no `__aeabi_d*` or libm symbols in `MDCS_HIL.out`.
