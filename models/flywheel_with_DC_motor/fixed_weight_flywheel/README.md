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

### How to Use the Multi-Target Makefiles

The system features a decoupled, multi-target compilation engine split across two Makefiles, following the Furuta convention. Because the position and speed compensator modules are isolated into separate functional subdirectories, `Makefile` dynamically appends target-specific include paths and binds the correct source components depending on your control objective. The HIL profiles live in a separate `Makefile_HIL`.

#### Available Compilation Profiles

**`Makefile`** (Simulation profiles):

* **`make position`**: Compiles the isolated position tracking executable. It appends the `./compensator/position_control` include directory and compiles `compensator/position_control/compensator.c`.
* **`make speed`**: Compiles the isolated velocity regulation executable. It appends the `./compensator/speed_control` include directory and compiles `compensator/speed_control/compensator.c`.
* **`make clean`**: Wipes out the generated target directories and clears compiled binary artifacts.
* **`make help`**: Displays a reference utility menu outlining usage syntax within the terminal.

**`Makefile_HIL`** (Hardware-in-the-Loop profiles):

* **`make -f Makefile_HIL position`**: Builds the position HIL host executable **and** the position MCU firmware, using `compensator/position_control/compensator_STM32F411.c` as the MCU law. Outputs land in `../../../build/flywheel_control/position/`.
* **`make -f Makefile_HIL speed`**: Builds the speed HIL host executable **and** the speed MCU firmware, using `compensator/speed_control/compensator_STM32F411.c` as the MCU law. Outputs land in `../../../build/flywheel_control/speed/`.
* Both profiles compile the host-side link `compensator/hil_control/compensator_hil.c`, which communicates with a physical MCU over `/dev/ttyACM0` at 921600 baud using fixed 12-byte packed frames.

---

#### Step-by-Step Execution Guide

##### 1. Compile and Run Position Control

To build the high-stiffness, trajectory-tracking position simulator:

```bash
make position
```

Once the compilation completes successfully, execute the position binary located in the global build directory:

```bash
../../../build/flywheel_control/flywheel_position_sim
```

##### 2. Compile and Run Speed Control

To build the continuous velocity regulation loop:

```bash
make speed
```

Once the compilation completes successfully, execute the speed binary located in the global build directory:

```bash
../../../build/flywheel_control/flywheel_speed_sim
```

##### 3. Compile and Run HIL Control

The HIL build is profile-specific because each profile has its own MCU compensator. Choose the profile that matches the law you want to run (requires a physical MCU on `/dev/ttyACM0`):

```bash
make -f Makefile_HIL position   # position LQI host + MCU firmware
# or
make -f Makefile_HIL speed      # speed PI host + MCU firmware
```

Each target produces a host binary plus `MDCS_HIL.out`, `MDCS_HIL.bin`, and `output.map` for flashing:

```bash
../../../build/flywheel_control/position/flywheel_hil_sim
# or
../../../build/flywheel_control/speed/flywheel_hil_sim
```

The firmware in `position/` and `speed/` is built from different MCU source files; do not flash one profile's firmware while running the other profile's host binary.

##### 4. Switch Profiles or Rebuild

Because the compiler utilizes distinct object mapping dependencies per profile, it is recommended to clean your workspace artifacts before switching between control profiles to prevent any caching collisions:

```bash
make clean
make position    # or make speed
```

`make clean` also removes both HIL profile folders (`build/flywheel_control/position/` and `speed/`), so rebuild the HIL target afterwards if you need the firmware.

---

#### Directory Architecture Visual Map

The compilation paths are mapped according to the following relative organizational layout:

```text
.
├── Makefile                     <-- SIL compilation engine (position + speed)
├── Makefile_HIL                 <-- HIL compilation engine (position + speed)
├── docs
│   ├── flywheel_lqi_design.py   <-- Full LQR/LQI gain generator (position + speed)
│   └── LQI_DERIVATION.md        <-- Mathematical derivation and tuning guide
├── README.md
├── compensator
│   ├── calculator.py            <-- Obsolete legacy PD calculator (no integral)
│   ├── hil_control
│   │   ├── compensator_hil.c    <-- Host-side HIL link (12-byte frames)
│   │   ├── compensator_hil.h
│   │   └── compensator.h
│   ├── position_control
│   │   ├── compensator.c        <-- SIL position law (make position)
│   │   ├── compensator.h
│   │   ├── compensator_STM32F411.c  <-- MCU position law (make -f Makefile_HIL position)
│   │   └── compensator_position_calculator.py
│   └── speed_control
│       ├── compensator.c        <-- SIL speed law (make speed)
│       ├── compensator.h
│       ├── compensator_STM32F411.c  <-- MCU speed law (make -f Makefile_HIL speed)
│       └── compensator_speed_calculator.py
└── simulator
    ├── flywheel_sim.c           <-- Core simulation engine
    └── flywheel_types.h
```

`compensator/speed_control/encoder_decoder_local.{c,h}` are dead code — not compiled; the common 20-bit codec in `framework/transport/codecs/` is used instead.

#### Python Gain Derivation

Gain calculations use SciPy with the physical parameters from `flywheel_sim.c`:

| Script | Purpose |
|---|---|
| `docs/flywheel_lqi_design.py` | Full design: position LQI, speed LQI, feedforward, encoder/velocity-filter analysis, C-ready `#define`s |
| `compensator/position_control/compensator_position_calculator.py` | Position-only quick calculator; matches the checked-in `K_POS/K_VEL/K_INT` |
| `compensator/speed_control/compensator_speed_calculator.py` | Textbook speed LQI reference; its output is **not** the checked-in speed PI (`K2_SPEED=-1.0`, `KI_SPEED=-0.05`) |
| `compensator/calculator.py` | Obsolete 2-state PD calculator; no integral state, not referenced by the build |

Run the main generator from this directory:

```bash
python3 docs/flywheel_lqi_design.py
```

Copy the printed `#define`s into the matching SIL compensator **and** its `compensator_STM32F411.c` MCU counterpart, then rebuild the SIL and HIL profiles.

All compiled executables are automatically generated outside your source directory tree under the unified `../../../build/flywheel_control/` workspace path.
