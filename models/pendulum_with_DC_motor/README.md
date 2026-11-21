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

# Pendulum with DC Motor

Model directory: `models/pendulum_with_DC_motor/`.

## Build

| Command | Output |
|---|---|
| `make` | SIL simulator: `build/inverted_pendulum/inverted_pendulum_sim` |
| `make -f Makefile_HIL` | HIL host: `build/inverted_pendulum/inverted_pendulum_sim_hil` plus STM32F411 firmware (`MDCS_HIL.out`, `MDCS_HIL.bin`, `output.map`) in the same folder |

`make clean` (either Makefile) removes the whole `build/inverted_pendulum/` folder, including both SIL and HIL artifacts.

## Run

Create the socat bridges first (see the repo-root `README.md` or `AGENTS.md`), then from this directory:

```bash
../../build/inverted_pendulum/inverted_pendulum_sim
```

Keys while running: `S` = enable compensator, `T` = disable, `Q` = quit.

Telemetry is the 4-value stream `<theta theta_dot command_voltage torque>`; the 3D animation and plotter read the socat PTY links from `framework/common/mdcs_config.h` (edit there if you move machines), both at 115200 baud.

## Gain derivation (Python)

The LQR derivation is embedded in `docs/Complete_derivation_pendulum_DC_motor.pdf` §7 (p. 7). There is no standalone `.py` file. The script is:

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

compute_lqr()
```

It produces `k1 = -220.1116` and `k2 = -26.0793`, which round to the checked-in `-220.0` and `-26.0`. The integral gain `KI = -75.0` is hand-tuned for precise 180° tracking and is **not** produced by this script.

The PDF's §7.1 table also lists an "Ideal" pair (≈ -106 / -13). That pair is not produced by this script: the script's `Q = diag([4575, 64])` gives the high-stiffness "Implementation" values shown in the same table.

The checked-in values are duplicated in the SIL law (`compensator/compensator.c`) and the MCU law (`compensator/compensator_STM32F411.c`); retuning requires editing both.
