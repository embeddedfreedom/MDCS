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

# Installation Manual and Architecture Overview: Modular Dynamics Control Suite (MDCS)

**Date:** November 2026

**Version:** 2.0

**Executive Summary & Configuration:** This document serves as the installation and configuration manual and the architecture overview document for the MDCS simulation environment. It details the setup of the 20kHz simulation engine, the 1kHz compensator interface, and the virtual serial bridge configuration required for real-time visualization and control. This repository contains a collection of independent, hardcoded dynamical models. This manual also details the setup required to bridge these C-based simulations to the Python visualizers via virtual serial ports. 

> **Project status (v2.0):** MDCS is stable and feature-complete. It is maintained as needed — bug fixes, documentation corrections, and build compatibility — but no new models or features are planned. Issues and questions are welcome; responses may take time.

### 🚀 Quick Start & Live Demo

Before setting up the local environment, you can explore the system architecture and see the controllers in action via the official project resources:

- **Dashboard:** [embeddedfreedom.dev](https://www.embeddedfreedom.dev) — *A web-based overview of the MDCS models, system specifications, and ecosystem positioning.*

- **Video Walkthrough:** [MDCS Setup & Demonstration (v1.x — SIL only)](https://youtu.be/eaQ9mskr6E8) — *A 7-minute text-card walkthrough with soft background music (no narration), covering the SIL loop, telemetry plotter interaction, and 3D OpenGL visualization. HIL is covered in §6; flywheel and furnace details are in each model's README.*

## What's new in v2.0

| Addition                       | Details                                                                                                                                          |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Hardware-in-the-Loop (HIL)** | STM32F411E-DISCO (Discovery kit) over USB CDC at 921600 baud. Fixed 12-byte frames with XOR checksums. See §6 for build, flash, and pin mapping. |
| **Flywheel with DC Motor**     | Position and speed profiles, 20-bit quadrature encoder, Z-pulse drift compensation at revolution boundaries.                                     |
| **Furnace Control**            | 2×2 coupled thermal LQI, dual PWM outputs (TIM3 CH1/PA6, CH2/PA7), industrial and bench parameter sets.                                          |
| **Automation**                 | `run_sim.py --hil` and `--hil-no-flash` for build → flash → run pipeline per model.                                                              |

## Engineering notes

Six short write-ups of problems that were actually hit during development: symptom, diagnosis, fix, and scope of the fix. Written for anyone debugging the same failure.

| Note                                                                           | Problem                                                                                               |
| ------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| [Float precision in a slow plant](docs/notes/01-float-precision-slow-plant.md) | A thermal model heats but never cools, because a 20 kHz step is smaller than a `float` ULP.           |
| [Fixed-rate timing on Linux](docs/notes/02-fixed-rate-timing-and-jitter.md)    | Why relative sleeps drift, how absolute deadlines fix it, what the jitter alert measures.             |
| [The 12-byte HIL frame](docs/notes/03-hil-frame-protocol.md)                   | Sync hunting, XOR checksums, a 3 ms poll deadline, and what happens when the host stops sending.      |
| [Float-only compensator port](docs/notes/04-bare-metal-float-only-port.md)     | How a `double` reaches the firmware, and the three linker checks that catch it.                       |
| [HIL scope](docs/notes/05-what-hil-validates.md)                               | What the hardware tests prove, and what they do not.                                                  |
| [Lockstep HIL](docs/notes/06-lockstep-host-owns-the-clock.md)                  | Why the host owns the clock, and how plant and controller stay in phase when Linux preempts the host. |

Index: [`docs/notes/README.md`](docs/notes/README.md)

## 1. Architecture Overview

MDCS provides a **Real-Time (Soft) Hardware-Emulated environment—a Software-in-the-Loop (SIL) Digital Twin** that faithfully models a system using Lagrangian mechanics and the RK4 solver. Unlike theoretical scripts, this is designed to be interacted with as if it were a real plant sitting on your desk. It utilizes C, the industry standard for embedded systems, ensuring the transition from simulation to real hardware is seamless.

To enrich the student experience further, the project includes a **High-Fidelity 3D Visualizer** and **Real-Time Telemetry Graphs**. By mimicking noisy feedback and hardware signals, a bench-like environment is provided that challenges you to solve real-world problems like sensor jitter and encoder drift.

The framework is completely **configurable**; you can tweak the system specs and noise levels directly in the source code. Since the core executable is only **~20 KB** (minus the visualizers and the external shared libraries), it's incredibly lean. By writing your compensator in **native C**, you get a real feel for how industrial control actually works—managing **fixed-step timing** and **deterministic loops** rather than just dragging blocks in a GUI.

The **core physics engine** is optimized for single-thread execution to ensure deterministic timing, mirroring bare-metal microcontroller behavior. Meanwhile, the **visualization layer** utilizes asynchronous threading to handle UI rendering without blocking the 20kHz data stream. This decoupled architecture ensures high-fidelity physics even when running on standard consumer-grade x86 hardware without the overhead of heavy simulation suites.

The SIL project is built for **Linux Desktop** environments and utilizes a **high-precision timing super-loop** to emulate a **Real-Time (Soft)** bare-metal microcontroller environment. To mitigate general-purpose OS jitter, the framework employs a **lockstep architecture** where the physics solver and compensator are synchronously coupled; if the OS introduces a scheduling delay, the entire simulation state remains frozen to preserve **temporal determinism**. The system implements a **self-healing absolute timeline** that automatically corrects for OS-induced blocking to prevent cumulative timing drift against the wall clock, maintaining seamless visual realism regardless of background CPU load.

![Diagram](general_architecture.png)

_**SIL Architecture**: Software-in-the-Loop environment. The compensator and plant model run together on the host PC; no physical MCU is involved._

![Diagram](general_architecture_hil.png)

_**HIL Architecture**: Hardware-in-the-Loop architecture. The host simulator connects to a physical STM32F411E-DISCO over USB CDC at 921600 baud; the MCU runs the compensator law and the host runs the plant._

#### 1.1 Modeling Fidelity & Abstractions

Bridging the digital-to-physical gap with intentional modeling choices:

##### Actuator (PWM)

Utilizes a **Mean Value Model**. The compensator returns a normalized duty cycle. At a 20:1 physics-to-control ratio, the plant's inertia acts as a natural low-pass filter, validating this approach for rigid-body simulation.

##### Sensors (Encoders)

Explicitly modeled as **Discrete Quadrature Pulses (A/B)**. This forces the compensator to manage quantization error and velocity jitter. Absolute positioning is verified via **Z-Index** latching at the **0° (Down)** reference point.

### ⚠️ CRITICAL OPERATIONAL LIMITATION

The `socat` configuration used in this suite creates a specific set of virtual serial bridges. This setup only accommodates **one** active control suite at a time.

- **Do not run two models simultaneously.**

- Attempting to run a second model (e.g., running `inverted_pendulum_sim` while `furuta_sim` is active) will result in serial port resource conflicts and data corruption.

<div style="color: red; font-weight: bold; background: #ffe6e6; padding: 15px; border: 2px solid red; border-radius: 8px; margin: 20px 0;">

**⚠️ WARNING — Hardcoded Paths (change before building)**

Several paths are hardcoded in this repository. **You must change them to match your machine/user before building or running anything:**

| What                                     | Hardcoded value                                                                                       | Where to change                                                                                                                                                                                 |
| ---------------------------------------- | ----------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Socat PTY links (3D + graph)             | `/home/dipto/sim_3d`, `/home/dipto/sim_graph`, `/home/dipto/sim_3d_in`, `/home/dipto/sim_graph_in`    | C simulator source (`simulator/*.c`)                                                                                                                                                            |
| `run_sim.py` paths (all models)          | `/home/dipto/sim_3d`, `/home/dipto/sim_3d_in`, `/home/dipto/sim_graph`, `/home/dipto/sim_graph_in`    | `models/rotary_inverted_pendulum/run_sim.py`, `models/pendulum_with_DC_motor/run_sim.py`, `models/flywheel_with_DC_motor/fixed_weight_flywheel/run_sim.py`, `models/furnace_control/run_sim.py` |
| HIL serial port                          | `/dev/ttyACM0`                                                                                        | `compensator/compensator_hil.c` (host-side link); the Python visualizers read the socat links, not the MCU                                                                                      |
| Socat links inside Python files          | `/home/dipto/sim_3d`, `/home/dipto/sim_graph`                                                         | `visualiser/*_telemetry.py`, `visualiser/*_animation.py`                                                                                                                                        |
| HIL test defaults                        | `/dev/ttyUSB0`                                                                                        | `hil/hil_STM32F4/test/Host_test_HIL.py`, `Host_test_HIL_notimeout.py`, `host_HIL_test.c`                                                                                                        |

**If you do not update these, the simulator will write telemetry to the wrong PTY files, the Python visualizers will read from the wrong serial ports, and nothing will appear to work.**

The paths must match **exactly** between:

1. The `socat` commands you run,
2. The C source files (simulator/compensator),
3. `run_sim.py` for your chosen model, and
4. The Python visualizer files.

If you move the repo to a different user or machine, do a repo-wide search for `/home/dipto` and `/dev/ttyACM0` (and `/dev/ttyUSB0` for tests) and replace them with your own paths.

</div>

## 2. Prerequisites

### System Dependencies

- **Any modern Linux distribution** (**tested: Linux Mint 21.3 / Ubuntu 22.04+ equivalents**)

- **Python 3.x***

- **GCC** (Build Essentials)

- **socat**: Virtual serial port utility.

**bash**

```
sudo apt update && sudo apt install socat
```

### Environment & Python Packages

It is highly recommended to use **Conda** or **Python 3 venv** to prevent dependency conflicts.

Bash

```
# Setup venv
python3 -m venv venv
source venv/bin/activate

# Install required packages
pip3 install pyserial numpy pyside6 pyqtgraph pygame PyOpenGL PyOpenGL-accelerate
```

---

## 3. Project Structure

The project is organized into modular directories. For specific mathematical proofs, control theory diagrams, and stability analysis, refer to the **`docs/`** folder inside each model directory.

**📁 models/[model_name]/**

Each experimental setup (e.g., rotary_inverted_pendulum) contains the following sub-directories:

- **`compensator/`**:  Contains the actual control law and signal processing logic. This is the primary area for student modification.

- **`simulator/`**: The core physics engine. It runs at **20kHz** to ensure high-fidelity integration and numerical stability.

- **`visualiser/`**: Includes the 1kHz real-time plotter and the OpenGL 3D rendering environment.

- **`docs/`**  :This folder contains the theoretical foundation of the project:
  
  - **`Model Derivations:`** Full Lagrangian or Newtonian derivations of the equations of motion (EOM) for each system.
  
  - **`Design Diagrams:`** Includes the **System Block Diagrams**. These visualize the control architecture, showing how sensor data is filtered, passed through the compensator, and converted into actuator commands (Torque/Voltage).

- **`Makefile`**: Every model folder includes a Makefile. Run `make` to compile the compensator and simulator into a single executable binary. The flywheel is multi-profile: use `make position` or `make speed` instead of a plain `make`.

- **`Makefile_HIL`**: (All models) Builds the Hardware-in-the-Loop host executable plus STM32F411 firmware. Requires a physical MCU on `/dev/ttyACM0`.

**📁 build/  :** Contains the binary for the plant after compiling. This directory has subdirectories with the respective model names under which the compiled binaries get created after make.

**📁 framework/  :** This contains common utilities like the encoder and decoder for the Quadrature pulses (in the codecs directory) and the keypress detect (in the common directory) applications.

**📁 hil/hil_STM32F4/**: Shared HIL firmware engine for STM32F411. Provides the bare-metal USB CDC transport, TinyUSB stack integration, TIM3 PWM timer output, and the compensator adapter invoked by each model's MCU law. Model-specific MCU compensators are injected via `COMPENSATOR=<path>` from each model's `Makefile_HIL`.

**Supported Models:**

| Model                                                     | SIL (`make`)               | HIL (`make -f Makefile_HIL`)               | Notes                                                                       |
| --------------------------------------------------------- | -------------------------- | ------------------------------------------ | --------------------------------------------------------------------------- |
| Furuta (`rotary_inverted_pendulum`)                       | ✅                          | ✅                                          | Working reference — untouched. HIL outputs to `build/furuta/`.              |
| Pendulum (`pendulum_with_DC_motor`)                       | ✅                          | ✅                                          | Angle-only host→MCU link. HIL outputs to `build/inverted_pendulum/`.        |
| Flywheel (`flywheel_with_DC_motor/fixed_weight_flywheel`) | ✅ (`make position\|speed`) | ✅ (`make -f Makefile_HIL position\|speed`) | Multi-profile: separate position/speed MCU laws and build folders.          |
| Furnace (`furnace_control`)                               | ✅                          | ✅                                          | 2×2 thermal LQI, dual PWM outputs. HIL outputs to `build/furnace_control/`. |

--- 

---

### 🛠 Student Workflow

1. **Review the model's `docs/`**: Understand the mathematical model before touching the code.

2. **Modify `compensator/`**: Implement your control gains or logic.

3. **Compile**: Use the `Makefile` in the model root.

4. **Simulate & Visualise**: Launch the binary and use the Python tools to monitor performance.

### Gain Derivation (Python)

Design scripts and derivations for the LQR/LQI gains are kept with each model:

| Model    | Gain derivation                                                                                                                                                                            |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Furuta   | Python LQR script embedded in `models/rotary_inverted_pendulum/docs/Complete_derivation_Furuta.pdf` §5 (pp. 12-13); reproduces all four checked-in gains                                   |
| Pendulum | Python LQR script in `models/pendulum_with_DC_motor/docs/Complete_derivation_pendulum_DC_motor.pdf` §7 (p. 7); derives `k1`/`k2`, while `ki` is hand-tuned                                 |
| Flywheel | `models/flywheel_with_DC_motor/fixed_weight_flywheel/docs/flywheel_lqi_design.py`, plus the per-profile calculators in `compensator/` (position gains match C; the speed PI is hand-tuned) |
| Furnace  | `models/furnace_control/docs/furnace_lqi_design.py` (also embedded in `models/furnace_control/docs/LQI_DERIVATION.md` §16); matches both C copies                                          |

---

**Errata — Pendulum derivation PDF:** The prose in `Complete_derivation_pendulum_DC_motor.pdf` §3 states *"The Python solver yields k1 ≈ −106 and k2 ≈ −13"* — this refers to an older weight set. Running the embedded Python script (Q = diag([4575, 64]), R = 0.1) actually produces **k1 ≈ −220.11, k2 ≈ −26.08**, which exactly matches the checked-in C gains (`k1 = -220.0f`, `k2 = -26.0f`). The PDF prose is outdated; the embedded script and the C code are consistent.

---

## 4. Installation & Setup

#### Step 1: Initialise Serial Bridges

You must create virtual serial bridges to allow the C-based simulator to communicate with the Python visualizers.

Bash

```
# a. Clean up existing bridges in case of data transfer failure over the bridge
pkill -9 socat
rm -f /home/dipto/sim_3d /home/dipto/sim_graph /home/dipto/sim_3d_in /home/dipto/sim_graph_in

# b. Create bridges for Renderer and Grapher
socat PTY,raw,echo=0,link=/home/dipto/sim_3d PTY,raw,echo=0,link=/home/dipto/sim_3d_in &
socat PTY,raw,echo=0,link=/home/dipto/sim_graph PTY,raw,echo=0,link=/home/dipto/sim_graph_in &

# c. Verify bridge creation
ls -l /home/dipto/sim_3d /home/dipto/sim_graph
```

In ideal situations, step b above should be run only. Steps a and c should be followed during troubleshooting. This step must be started before starting the simulator, plotter or the visualiser. If not done, and the front end appears non responsive, start the applications again. No need to redo the socat commands. 

**DO NOT CLOSE THE WINDOW WHERE THE socat COMMAND IS RUN. CLOSING THE WINDOW CLOSES THE BRIDGE. SO KEEP THE WINDOW OPEN FOR THE DURATION OF THE USE**

> 🔗 **See the red "Hardcoded Paths" warning box at the top of this README** — it lists every path that must be changed before building/running (socat PTY links, `run_sim.py`, HIL serial port, HIL test defaults). Read it before proceeding.

#### Step 2: Source Code Path Configuration

The paths used in the `socat` commands above (e.g., `/home/dipto/sim_3d`) must be **exactly** matched inside your source files:

- Update the device paths in the **C source files** (simulator/compensator).

- Update the serial port strings in the **Python files** (for example `furuta_telemetry.py` and `furuta_animation.py`; the names vary by model).

#### Step 2.5: Install ARM Cross-Toolchain (for HIL Firmware)

The HIL firmware for the STM32F411 is compiled with the ARM bare-metal cross-toolchain (`arm-none-eabi-gcc`). This is **only** required for HIL builds — SIL simulation compiles with the host `gcc` and needs no cross-toolchain.

See the full walkthrough: [MDCS Cross-Toolchain Install](https://youtu.be/2fNYTPCTxbg)

Bash

```
# Download the ARM GNU toolchain (example: ARM GNU Toolchain 13.3.Rel1)
wget https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz

# Extract to /opt
sudo tar -xJf arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz -C /opt

# Add to PATH (permanent)
echo 'export PATH=/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

# Verify installation
arm-none-eabi-gcc --version
```

Expected output: `arm-none-eabi-gcc (GNU Toolchain for the Arm Architecture 13.3.rel1 ...)`

**Note:** The cross-toolchain is used by `Makefile_HIL` to compile the STM32F411 firmware (`MDCS_HIL.out/.bin`). SIL builds (`make`) use the host compiler and do not require it.

Flashing the resulting image is covered in [§6.3 Flashing Firmware](#63-flashing-firmware) — `st-flash` (the verified route), OpenOCD or STM32CubeProgrammer (both untested), or any tool of your choice, as long as the `.bin` lands at `0x08000000`.

#### Step 3: Compilation

Navigate to your chosen model directory and compile:

Bash

```
cd models/[model_name]
make
```

The resulting binary will be placed in the root `build/[model_name]` directory.

---

## 5. Execution

##### **5.1** Binary Operation Manual

Start the Binary: Run the compiled simulator/compensator from the `build` folder.

For example **./furuta_sim** in case of the Inverted_rotary_pendulum 

The simulator does not pass the "true" angle to the compensator. Instead, it converts the continuous physics angle into discrete **Gray Code states** (`phase_to_gray`).

- **Bus Logic:** The `EncoderBus` structure stores a sequence of transitions that occurred within the 1ms window.

- **Z-Pulse:** Emulates the physical index pulse of an encoder. When the pendulum passes the 0-degree mark (±0.1°), `bus.z_pend` is set to `true`, allowing the compensator to perform an absolute position reset.

#### Operational Controls

| **Key** | **Action**  | **Effect**                                                                           |
| ------- | ----------- | ------------------------------------------------------------------------------------ |
| **S**   | **Enable**  | Starts the compensator logic and applies motor power.                                |
| **T**   | **Disable** | Disconnects compensator from plant (`pwm = 0`); physics continue to run (free fall). |
| **Q**   | **Quit**    | Safely closes the simulation.                                                        |

 The model visualisers both graphical and the model will settle down like the real scenario. Pressing S again connects the compensator.  The output also displays timing jitter alerts if the control loop executes faster or slower than the expected interval of 1 millisecond. 

###### High-Precision Sync

The loop is governed by `clock_nanosleep` using the `CLOCK_MONOTONIC` timer. This ensures that the 50μs steps are consistent regardless of CPU load. A **Timing Jitter Alert** will print to the console if the 1kHz loop deviates by more than 5%.

#### 5.2 Telemetry Plotter Operational Manual

**Start the Plotter:** Run the Python script to see the system in action:

Bash

```
python3 visualiser/furuta_telemetry.py    # filenames vary by model, see the table below
```

The Telemetry plotter is a high-frequency (1kHz) data visualization and logging tool built with Python, PySide6, and PyQtGraph. Designed specifically for control systems engineering and hardware-in-the-loop implementation.

###### 5.2.1. Real-Time Visualization

- **Dual-Threaded Engine:** Data acquisition runs on a dedicated high-priority thread to prevent UI "freezing," ensuring the graph remains responsive even at 1000Hz.

- **GPU Acceleration:** Uses hardware-accelerated rendering. Instead of drawing 600,000 points, the engine "slices" the data to only render what is visible on your screen, maintaining a smooth 50 FPS.

- **Live Sync:** The graph automatically locks its X-axis to the latest incoming data packet.

###### 5.2.2. Interaction & Navigation

- **LIVE Mode:** By default, the graph auto-scrolls. You can zoom/scale the **Y-Axis** using the mouse wheel or right-click-drag to inspect signal amplitudes.

- **PAUSE Mode:** Click the "LIVE" toggle to pause. This unlocks the **X-Axis**, allowing you to scroll back through the 10-minute history buffer to find specific events.

- **Crosshair Tool:** Moving your mouse over the plot activates a high-precision crosshair. The tooltip displays the exact `Time (s)` and `Value` for the data point under your cursor.

- **Channel Toggles:** Individual channels can be turned on or off via checkboxes to reduce visual clutter during multi-variable analysis.

###### 5.2.3. Memory & Buffer Management

- **Circular History Buffer:** The plotter allocates a fixed block of memory for exactly **10 minutes** (600,000 samples at 1kHz for the furuta simulator). Use the **Clear Buffer** button between test runs to reset the timeline.

- **Zero-Latency Overwriting:** Once the 10-minute mark is reached, the oldest data is discarded to make room for new data. This allows the program to run for days without increasing RAM usage or slowing down.

- **Load Monitor:** The top-right overlay displays the real-time `Serial Rate (Hz)` and `Buffer %`. Use this to verify that your hardware is maintaining its target loop frequency.

###### 5.2.4. Data Export (CSV)

- **Snap-to-Zero Export:** When you click "Export CSV," the tool "unrolls" the circular buffer and reconstructs the last 10 minutes of data into a linear timeline.

- **Normalization:** The exported time column is automatically normalized to start at `0.0s`, making the data ready for immediate import into MATLAB, Excel, or Python for post-processing.

- **Safety Lock:** Export is disabled while in "LIVE" mode to ensure data integrity during the file-writing process.

###### 5.2.5. Unit Conversion

- **Radians/Degrees Toggle:** The plotter features a real-time conversion engine. Toggling this will instantly convert the angular data (Theta and Theta_Dot) on the fly without interrupting the data stream.

#### 5.3 3D System Simulator Operational Manual

Start the simulator: Run the Python script to see the system in action:

```bash
python3 visualiser/furuta_animation.py    # filenames vary by model, see the table below
```

##### 5.3.1. Architectural Overview

The simulator operates using a **Dual-Process Architecture** to ensure that communication bottlenecks do not interfere with smooth visual rendering:

- **The Serial Thread:** A background worker that continuously polls the serial port. It features an "auto-flush" mechanism (resetting the buffer if it exceeds 500 bytes) to prevent lag between the physical action and the 3D response.

- **The Render Loop:** Running at a locked 60 FPS, this process handles the OpenGL transformation pipeline and user input events.

###### 5.3.2. Coordinate System & Geometry

The simulator uses a standard **Right-Hand Coordinate System**:

- **X-Axis:** Horizontal across the lab table.

- **Y-Axis:** Vertical (Gravity vector).

- **Z-Axis:** Depth (Parallel to the motor shaft).

###### 5.3.3 Component Modeling:

- **The Table:** Rendered as a `GL_QUADS` composite with 3D legs to provide a spatial reference point.

- **The Pendulum:** Modeled as a hierarchical object. It inherits the rotation of the motor shaft. The rotation is applied around the **Z-axis** using `glRotatef(math.degrees(theta_pend), 0, 0, 1)`.

###### 5.3.4. User Interface & Controls

The simulator includes a 2D heads-up display (HUD) overlaid on the 3D scene.

| **Action**        | **Control**          | **Description**                                           |
| ----------------- | -------------------- | --------------------------------------------------------- |
| **Rotate View**   | `Left-Click + Drag`  | Rotates the camera around the center of the table.        |
| **Zoom In/Out**   | `Right-Click + Drag` | Adjusts the camera distance (Z-translation).              |
| **Toggle Units**  | `T` Key              | Switches the HUD between Radians and Degrees.             |
| **Resize Window** | `Drag Corner`        | The viewport automatically adjusts the perspective ratio. |

**Script names by model** (run from inside the model directory):

| Model    | Telemetry plotter                            | 3D simulator                                 |
| -------- | -------------------------------------------- | -------------------------------------------- |
| Furuta   | `visualiser/furuta_telemetry.py`             | `visualiser/furuta_animation.py`             |
| Pendulum | `visualiser/motor_and_pendulum_telemetry.py` | `visualiser/motor_and_pendulum_animation.py` |
| Flywheel | `visualiser/motor_and_flywheel_telemetry.py` | `visualiser/motor_and_flywheel_animation.py` |
| Furnace  | `visualiser/furnace_telemetry.py`            | `visualiser/furnace_animation.py`            |

**NOTE:** The simulator and plotter can be started in any order, any one can also be started if the other is not required. This will not affect the visuals at all.

## 6. Hardware-in-the-Loop (HIL)

MDCS includes a complete Hardware-in-the-Loop implementation against a physical STM32F411 MCU over USB CDC at 921600 baud on `/dev/ttyACM0`. Only sensor values cross the link; the MCU derives velocity/state and runs the compensator law.

### 6.1 HIL Frame Format

Fixed **12-byte** frames with XOR checksum:

**Host→MCU:** `sync 0xAA 0x55 | float v1 | float v2 | uint8 cmd | uint8 checksum`

**MCU→host:** `sync 0x55 0xAA | float out1 | float out2 | uint8 status | uint8 checksum`

3 ms poll timeout per cycle.

| Model    | Host→MCU                                 | MCU→host                                                    |
| -------- | ---------------------------------------- | ----------------------------------------------------------- |
| Furuta   | arm angle, pendulum angle                | signed motor voltage (`out1`), pendulum angle echo (`out2`) |
| Pendulum | pendulum angle in `v1`, `v2 = 0`         | signed motor voltage in `out1`                              |
| Flywheel | wrapped flywheel angle in `v1`, `v2 = 0` | signed motor voltage in `out1`                              |
| Furnace  | measured `T1`, `T2`                      | PWM duty 1, PWM duty 2 (both `[0, 1]`)                      |

The shared adapter calls `run_compensator_cycle_HIL(v1, v2)` by default. With `-DHIL_TWO_OUTPUTS` (furnace only) it calls `run_compensator_cycle_HIL_dual(v1, v2, &out1, &out2)`; otherwise `out2 = v2`.

### 6.2 Building HIL

From the model directory:

**Furuta:**

```bash
make -f Makefile_HIL
```

Host: `build/furuta/furuta_sim_hil` | Firmware: `build/furuta/MDCS_HIL.out/.bin`

**Pendulum:**

```bash
make -f Makefile_HIL
```

Host: `build/inverted_pendulum/inverted_pendulum_sim_hil` | Firmware: `build/inverted_pendulum/MDCS_HIL.out/.bin`

**Flywheel (position):**

```bash
make -f Makefile_HIL position
```

Host+Firmware: `build/flywheel_control/position/flywheel_hil_sim` + `MDCS_HIL.out/.bin`

**Flywheel (speed):**

```bash
make -f Makefile_HIL speed
```

Host+Firmware: `build/flywheel_control/speed/flywheel_hil_sim` + `MDCS_HIL.out/.bin`

**Furnace:**

```bash
make -f Makefile_HIL
```

Host: `build/furnace_control/furnace_sim_hil` | Firmware: `build/furnace_control/MDCS_HIL.out/.bin` (built with `-DHIL_TWO_OUTPUTS`)

### 6.2.5 Makefile Hierarchy & Build Mechanism

MDCS uses a two-level Makefile system. Each model directory contains its own `Makefile` (SIL) and `Makefile_HIL` (HIL), while the shared STM32F411 firmware engine lives in `hil/hil_STM32F4/`.

```
models/<model>/
├── Makefile                        # SIL build (host gcc)
├── Makefile_HIL                    # HIL build (host + invokes shared firmware)
├── simulator/                      # SIL physics engine (20 kHz RK4)
├── compensator/                    # Control laws
│   ├── compensator.c               # SIL law (float math)
│   ├── compensator_hil.c           # Host-side HIL link (USB CDC)
│   └── compensator_STM32F411.c     # MCU law (furnace names it compensator_hil_STM32F4.c)
├── visualiser/                     # Python 3D + telemetry
├── run_sim.py                      # Build + socat + simulator + visualizers
└── docs/                           # Derivations & diagrams
```

```
hil/hil_STM32F4/                    # Shared bare-metal firmware engine
├── Makefile                        # STM32F411 firmware (MIT licensed)
├── HIL_main.c                      # USB CDC task + compensator adapter
├── include/
│   └── tusb_config.h               # TinyUSB device configuration
├── src/
│   ├── core_initialisation_STM32F411xe.c   # Clock, GPIO, TIM3 PWM, USB
│   ├── startup_gcc.c                 # Vector table, reset handler
│   ├── UART.c                        # UART helpers
│   ├── printm.c                      # Bare-metal printf (LGPLv2.1)
│   ├── handlers_STM32F411xe.c        # IRQ handlers
│   ├── applications.c                # Main loop
│   ├── usb_descriptors.c             # USB CDC descriptors
│   └── usb_cdc_adapter.c             # 12-byte frame parser + dispatcher
└── MDCS_HIL.ld                       # Linker script (FLASH @ 0x08000000)
```

**TinyUSB stack** — The USB CDC transport is provided by the [TinyUSB](https://github.com/hathach/tinyusb) stack, vendored at `tinyusb-master/` and licensed under MIT. It is **not** part of MDCS itself — it is a third-party dependency kept under `tinyusb-master/`. The shared firmware Makefile (`hil/hil_STM32F4/Makefile`) pulls in only the subset of TinyUSB source files needed for device-mode CDC:

```make
TINYUSB_ROOT = ../../tinyusb-master

include_dirs = . include $(TINYUSB_ROOT)/src $(dir $(COMPENSATOR))

sources = HIL_main.c \
          src/core_initialisation_STM32F411xe.c \
          ...
          src/usb_descriptors.c \
          src/usb_cdc_adapter.c \
          $(COMPENSATOR) \
          $(TINYUSB_ROOT)/src/tusb.c \
          $(TINYUSB_ROOT)/src/common/tusb_fifo.c \
          $(TINYUSB_ROOT)/src/device/usbd.c \
          $(TINYUSB_ROOT)/src/class/cdc/cdc_device.c \
          $(TINYUSB_ROOT)/src/portable/synopsys/dwc2/dcd_dwc2.c \
          $(TINYUSB_ROOT)/src/portable/synopsys/dwc2/dwc2_common.c
```

The model's `Makefile_HIL` invokes this shared Makefile recursively, passing `COMPENSATOR` and `OUTPUT_DIR`. TinyUSB sources are compiled from `tinyusb-master/` directly — there is no separate build step for the stack.

**Build flow — SIL (`make`):**

The model's `Makefile` compiles the simulator (`simulator/<model>_sim.c`) + SIL compensator (`compensator/compensator.c`; flywheel uses `compensator/{position_control,speed_control}/compensator.c`) + `get_keypress.c` with host `gcc`. Output goes to `build/<model>/<model>_sim` (or `build/flywheel_control/flywheel_position_sim` / `flywheel_speed_sim`).

**Build flow — HIL (`make -f Makefile_HIL`):**

Two separate compilations happen:

1. **Host binary** — Compiled with host `gcc` from the simulator + `compensator/compensator_hil.c` (USB CDC link layer; flywheel uses `compensator/hil_control/compensator_hil.c`) + `get_keypress.c`. Outputs `build/<model>/<model>_sim_hil` (or `build/flywheel_control/{position,speed}/flywheel_hil_sim`).

2. **MCU firmware** — `Makefile_HIL` recursively invokes `hil/hil_STM32F4/Makefile`, passing:
   
   - `COMPENSATOR=<absolute path to model's MCU compensator .c file>`
   
   - `OUTPUT_DIR=<absolute path to model's build folder>`
   
   - Optional `EXTRA_CFLAGS` (e.g., `-DHIL_TWO_OUTPUTS` for furnace; the flywheel selects its profile by choosing the MCU source file)
   
   The shared firmware Makefile compiles STM32F411 bare-metal code (TinyUSB stack + clock + GPIO + TIM3 PWM + CDC adapter + model's compensator), links with `arm-none-eabi-gcc`, then `arm-none-eabi-objcopy` produces `MDCS_HIL.bin` for flashing at `0x08000000`.

**Key points:**

- **No `COMPENSATOR` hardcoded in shared firmware** — Each model's `Makefile_HIL` injects its own compensator. The shared `hil/hil_STM32F4/Makefile` requires `COMPENSATOR` to be set (errors out otherwise, except for `make clean`).
- **Single-precision only (MCU)** — `float` math, `fpv4-sp-d16` FPU, hard-float ABI. No `double`, no `M_PI`, no libm. Local `sqrt_pos()` Newton iteration replaces `sqrtf()`. All five MCU images link no soft-double or libm symbols; see §6.5.
- **Clean builds** — `make clean` removes the model's entire `build/` subtree (SIL + HIL together). For flywheel, both `position/` and `speed/` subfolders are removed.
- **Header changes** — Firmware rules list `.c` sources only; editing a `.h` does not force a rebuild. Use `make -B -f Makefile_HIL` or delete `MDCS_HIL.out` after header changes.

### 6.3 Flashing Firmware

> **Which of these has been tested?** Only **Option A (`st-flash`)**, which is also what `run_sim.py` uses. The OpenOCD and STM32CubeProgrammer commands in Options B and C follow the vendors' standard usage but have **not** been executed against this firmware — treat them as a starting point, and fall back to `st-flash` if one fails.

`Makefile_HIL` links and then converts the ELF to a raw binary:

```
build/<model>/MDCS_HIL.bin
```

Flash that file to the start of the STM32F411 flash, **`0x08000000`**. There is no bootloader offset — `startup_gcc.c` puts the vector table there via `MDCS_HIL.ld`.

Each model (and each flywheel profile) produces its own firmware because the compensator law is compiled in, so always flash the `.bin` that sits next to the host binary you are about to run:

| Model               | Binary to flash                                |
| ------------------- | ---------------------------------------------- |
| Furuta              | `build/furuta/MDCS_HIL.bin`                    |
| Pendulum            | `build/inverted_pendulum/MDCS_HIL.bin`         |
| Flywheel (position) | `build/flywheel_control/position/MDCS_HIL.bin` |
| Flywheel (speed)    | `build/flywheel_control/speed/MDCS_HIL.bin`    |
| Furnace             | `build/furnace_control/MDCS_HIL.bin`           |

#### Option A — `st-flash` (stlink-tools)

Works with the on-board ST-LINK of the Discovery kit and with a standalone ST-LINK dongle:

```bash
sudo apt install stlink-tools          # Debian/Ubuntu
st-flash --reset write build/<model>/MDCS_HIL.bin 0x08000000
```

#### Option B — OpenOCD (untested here)

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/<model>/MDCS_HIL.bin 0x08000000 verify reset exit"
```

On older OpenOCD releases the adapter config is named `interface/stlink-v2.cfg`. On most Linux systems OpenOCD needs permission to open the ST-LINK USB device — either run it under `sudo` or install the shipped udev rules (`/usr/share/openocd/contrib/60-openocd.rules`) and re-plug the board.

#### Option C — STM32CubeProgrammer (untested here)

The CLI is not on `PATH` by default on Linux; add STM32CubeProgrammer's `bin/` directory to `PATH`, use the full path, or launch it from the GUI terminal.

CLI:

```bash
STM32_Programmer_CLI -c port=SWD -w build/<model>/MDCS_HIL.bin 0x08000000 -Rst
```

GUI: connect over SWD (SWDIO, SWCLK, GND, 3V3), open the `.bin`, set the start address to `0x08000000`, then **Download**.

#### Option D — any other tool

The list above is just a starting point. `pyocd`, a J-Link, `dfu-util` (only if the ROM bootloader is enabled), or any programmer that can write a raw binary to an absolute address will all work. **Students are free to use whatever flashing tool they already know** — nothing in the firmware or in the host simulator depends on how the image gets onto the chip.

If your tool asks for an address, it is always `0x08000000`. If it asks for a format, the file is a raw `.bin`, not an ELF or a hex file (use `MDCS_HIL.out` if you want the ELF for debugging, `MDCS_HIL.bin` for flashing).

#### After flashing

- The board reboots and enumerates as a USB CDC device: `ls -l /dev/ttyACM0`.
- `dmesg | tail` should show the CDC ACM attach.
- The HIL host binary opens `/dev/ttyACM0` at 921600 baud, so the port must be free and the user must be in the `dialout` group.

If `/dev/ttyACM0` never appears, the usual cause is the clock setup: the firmware assumes an **8 MHz HSE** crystal. A board with a different crystal needs the PLL factors in `PLL_Init_84MHZ()` adjusted (see §6.6).

### 6.4 Running HIL

Start the socat bridges first (see Step 1 above), then run the HIL host binary. From a model directory two levels below the repo root (furuta, pendulum, furnace), the path is `../../build/<build_dir>/<hil_binary>`, using the build directories listed in §6.2:

```bash
../../build/furuta/furuta_sim_hil
../../build/inverted_pendulum/inverted_pendulum_sim_hil
../../build/furnace_control/furnace_sim_hil
```

The nested flywheel model is three levels deep, so its binaries are at:

```bash
../../../build/flywheel_control/{position,speed}/flywheel_hil_sim
```

Press **S** to enable the compensator, **T** to disable, **Q** to quit.

**Link behaviour (measured on hardware; all models, including both flywheel profiles, have been run in HIL on the STM32F411):** one host→MCU→host exchange costs **≈ 0.4 ms** (0.32–0.50 ms over five requests at 921600 baud), far inside the 3 ms `poll()` timeout, so USB does not limit the 1 kHz compensator cadence. Normal runs report no `CONNECTION ERROR` lines.

For the furnace model, probing the compensator code on the MCU directly, with setpoints 75/75 °C, shows the law running on the chip and not on the host (table below):

| Sent T1 / T2 | PWM1 / PWM2 returned | Why                                                       |
| ------------ | -------------------- | --------------------------------------------------------- |
| 25.0 / 25.0  | 1.0000 / 1.0000      | 50 °C below setpoint — `Kx ≈ 20.8` saturates both heaters |
| 74.0 / 74.0  | 1.0000 / 1.0000      | 1 °C below setpoint is still enough to saturate           |
| 75.0 / 75.0  | 0.8333 / 0.7454      | zero error → exactly the feedforward `pwm_sp`             |
| 76.0 / 76.0  | 0.0000 / 0.0000      | above setpoint → heaters off                              |

Every reply carried `status = 0x01` (running) and a valid XOR checksum.

**MCU integrator cadence:** the MCU law keeps no clock of its own. Each frame it receives advances the integral by a fixed `q += e * 0.001f`, i.e. it assumes exactly one host call per millisecond. The host calls the compensator once per 20 physics steps (1 kHz), so the two rates agree as long as the host loop meets its deadline. If the host falls behind, the integral simply advances more slowly in wall time — invisible at τ ≈ 1000 s, but do not change the host cadence without also changing that `0.001f`.

### 6.5 Bare-Metal Firmware Rules

MCU code must stay fully bare-metal: no `-lm`, no newlib syscalls, single-precision FPU only.

- No `double`, no `M_PI` (it is a `double` in `math.h`), no unsuffixed decimal literals, no libm calls.
- Use local float `PI` and `fabsf`.
- Firmware rules list `.c` sources only — after editing a header, force a rebuild with `make -B -f Makefile_HIL` or delete `MDCS_HIL.out`.
- `EXTRA_CFLAGS` is not a make dependency; stale firmware can persist if defines change without cleaning.

**Verified:** all five MCU images (furuta, inverted_pendulum, flywheel position/speed, furnace_control) rebuilt with `make -B -f Makefile_HIL` contain no undefined `__aeabi_d*`, `sqrt`, `fabs`, `sin`, or `cos` symbols. Furuta and pendulum include `<math.h>` and call `fabsf`, which GCC lowers to a single-precision instruction, so the float-only rule holds for every model.

### 6.6 Board & Clock Configuration

**Reference hardware: the STM32F411E-DISCO Discovery kit**, which carries an **8 MHz HSE crystal**. Every HIL measurement in this document was taken on that board.

The clock tree is configured in:

- `hil/hil_STM32F4/core_initialisation_STM32F411xe.h`
- `hil/hil_STM32F4/src/core_initialisation_STM32F411xe.c`

The 8 MHz value is baked into `PLL_Init_84MHZ()`: `PLLM = 4` gives a 2 MHz VCO input, `PLLN = 168` gives a 336 MHz VCO, `PLLP = /4` gives 84 MHz SYSCLK and `PLLQ = /7` gives the 48 MHz USB clock. A board with a different HSE crystal will not run this image as-is. The PLL input must land in 1–2 MHz and the VCO in 100–432 MHz, so most other crystals put the PLL outside its operating range; and even when they do not, SYSCLK and the 48 MHz USB clock come out at the wrong frequency. In either case the board looks dead and `/dev/ttyACM0` never appears — there is no error message.

#### PLL values

| Board              | HSE    | `PLLM` | `PLLN` | `PLLP` | `PLLQ` | SYSCLK | USB    | Status                  |
| ------------------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ----------------------- |
| STM32F411E-DISCO   | 8 MHz  | `4U`   | `168U` | `/4`   | `/7`   | 84 MHz | 48 MHz | Run on hardware         |
| Blackpill (F411CE) | 25 MHz | `25U`  | `336U` | `/4`   | `/7`   | 84 MHz | 48 MHz | Derived only — untested |

**The Blackpill row is arithmetic, not a tested build.** No Blackpill has been run with this project, and the 25 MHz figure comes from the vendor's schematic rather than from a board measured here. It follows from the RM0383 limits (`f(VCOin) = f(HSE) / PLLM` inside 1–2 MHz, `f(VCO) = f(VCOin) × PLLN` inside 100–432 MHz): `25/25 = 1 MHz` → `× 336 = 336 MHz` → `/4 = 84 MHz` → `/7 = 48 MHz`. Verify the crystal on your own board's schematic and confirm the constants before relying on them.

Button polarity, LED pins, bootloader/ST-LINK arrangement and flash/SRAM size also differ between boards, and none of that has been validated here.

`SystemCoreClock` stays `84000000U` and flash latency stays 2 wait states as long as the 84 MHz target is kept. `HSE_VALUE` is not defined or used in this project, so there is no CMSIS constant to update.

Verify the target crystal before building; an 8 MHz HSE is assumed by default.

Porting to an unrelated board also requires checking:

- Any board-specific GPIO/pin assignments in `TIM3_PWM_Init()` and `usb_hardware_init()`
- Linker script memory regions (`MDCS_HIL.ld`) if flash/SRAM sizes differ

#### TIM3 PWM Pin Assignment

The STM32F411 firmware uses TIM3 for hardware PWM output. Pin assignments:

| Model               | Timer | Channel | Pin | Alternate Function | Output               |
| ------------------- | ----- | ------- | --- | ------------------ | -------------------- |
| Furuta              | TIM3  | CH1     | PA6 | AF2                | Signed motor voltage |
| Pendulum            | TIM3  | CH1     | PA6 | AF2                | Signed motor voltage |
| Flywheel (position) | TIM3  | CH1     | PA6 | AF2                | Signed motor voltage |
| Flywheel (speed)    | TIM3  | CH1     | PA6 | AF2                | Signed motor voltage |
| Furnace             | TIM3  | CH1     | PA6 | AF2                | Heater 1 PWM duty    |
| Furnace             | TIM3  | CH2     | PA7 | AF2                | Heater 2 PWM duty    |

**Note:** All models use TIM3. The furnace uses both TIM3_CH1 (PA6) and TIM3_CH2 (PA7) for its two independent heater PWMs. TIM4 is **not** configured for PWM output in any model — it only appears in the interrupt vector table as an unused default handler.

**Scope check vs USB CDC link:** The PWM pins (PA6/PA7) output the PWM duty cycle on the STM32 and can be probed with an oscilloscope or multimeter to verify the firmware is generating the expected signal. However, in the HIL setup the actual control values are sent back to the host over USB CDC (the 12-byte frame, 921600 baud) — the PWM pins are for hardware verification only, not the primary feedback path.

## 7. System Specifications & Logic

- **Timing:** 20kHz Simulation Loop | 1kHz Compensator.

- **Coordinates:** $0$ is Down (stable), $\pi$ (or $180^\circ$) is Up (unstable).

- **Convention:** Clockwise turns are **negative**; Counter-clockwise are **positive**.

- **Arm Range (Furuta):** $0$ to $\pm 180^\circ$.

- **Data Format:** Tab-delimited fixed-precision ASCII stream. Each model has its own value set: Furuta sends a 6-value packet (Value 1 = arm angle, Value 2 = pendulum angle, then their rates, torque and commanded voltage); pendulum, flywheel and furnace streams are documented in their model READMEs.

- **UI:** The visualizer includes a toggle to switch between Radians and Degrees.

## 8. License

MDCS uses multiple licenses depending on the component:

| Component                                                                 | License  | File                    |
| ------------------------------------------------------------------------- | -------- | ----------------------- |
| Documentation (READMEs, `AGENTS.md`, `docs/*.md`, `docs/*.pdf`, diagrams) | GPLv3    | `LICENSE-GPLv3`         |
| All model source (`*.c`, `*.h`, `*.py`, `Makefile`, `Makefile_HIL`)       | GPLv3    | `LICENSE-GPLv3`         |
| Framework (`framework/`)                                                  | GPLv3    | `LICENSE-GPLv3`         |
| HIL shared firmware (`hil/hil_STM32F4/`)                                  | MIT      | `LICENSE-MIT`           |
| MCU compensators (`compensator_STM32F411.c`)                              | MIT      | `LICENSE-MIT`           |
| `tinyusb-master/` (vendored stack)                                        | MIT      | `tinyusb-master/`       |
| `printm.c` (printf implementation)                                        | LGPLv2.1 | `LICENSES/LGPL-2.1.txt` |

- **`LICENSE-GPLv3`** — GPLv3, the primary license for MDCS source code and documentation.
- **`LICENSE-MIT`** — MIT License, applies only to HIL firmware and MCU compensator files.
- **`LICENSES/LGPL-2.1.txt`** — GNU Lesser General Public License v2.1, applies to `hil/hil_STM32F4/src/printm.c`.
- **`tinyusb-master/`** — Vendored third-party stack, licensed under MIT (see `tinyusb-master/` source headers).

## 9. Quick Run with `run_sim.py`

Each model folder includes a `run_sim.py` script that automates: **build → flash → socat bridges → simulator → visualizers**. Run it from inside the model directory.

### Usage

Navigate to the model directory and run:

```bash
python3 run_sim.py           # SIL build + run
python3 run_sim.py --hil     # HIL build + flash MCU + run
python3 run_sim.py --hil-no-flash  # HIL build + run, skip flashing (flash MCU yourself first)
python3 run_sim.py --help    # Show usage help
```

For flywheel (multi-profile), specify the profile:

```bash
python3 run_sim.py position           # SIL position build + run
python3 run_sim.py speed              # SIL speed build + run
python3 run_sim.py --hil position     # HIL position build + flash + run
python3 run_sim.py --hil speed        # HIL speed build + flash + run
python3 run_sim.py --hil-no-flash position   # HIL position build + run, skip flashing
python3 run_sim.py --hil-no-flash speed      # HIL speed build + run, skip flashing
python3 run_sim.py --help             # Show usage help
```

### What the script does

1. **Builds** the model (always rebuilds — host binary for SIL, host + firmware for HIL)
2. **Flashes firmware** to STM32 via `st-flash` (HIL mode with `--hil` only; skipped with `--hil-no-flash`)
3. **Creates socat bridges** (`/home/dipto/sim_3d`, `/home/dipto/sim_graph`) if they don't already exist
4. **Launches the simulator binary** (SIL or HIL)
5. **Launches both visualizers** (3D animation + telemetry plotter)
6. **Runs in foreground** until you press `Ctrl+C`
7. **Cleans up** — kills all processes and socat bridges on exit

### Model-specific commands

| Model                   | Command (from inside model dir)                     | Output                                                         |
| ----------------------- | --------------------------------------------------- | -------------------------------------------------------------- |
| Furuta                  | `python3 run_sim.py`                                | `build/furuta/furuta_sim`                                      |
| Furuta HIL              | `python3 run_sim.py --hil`                          | `build/furuta/furuta_sim_hil` + firmware                       |
| Furuta HIL (no flash)   | `python3 run_sim.py --hil-no-flash`                 | `build/furuta/furuta_sim_hil` + firmware                       |
| Pendulum                | `python3 run_sim.py`                                | `build/inverted_pendulum/inverted_pendulum_sim`                |
| Pendulum HIL            | `python3 run_sim.py --hil`                          | `build/inverted_pendulum/inverted_pendulum_sim_hil` + firmware |
| Pendulum HIL (no flash) | `python3 run_sim.py --hil-no-flash`                 | `build/inverted_pendulum/inverted_pendulum_sim_hil` + firmware |
| Flywheel position       | `python3 run_sim.py position`                       | `build/flywheel_control/flywheel_position_sim`                 |
| Flywheel speed          | `python3 run_sim.py speed`                          | `build/flywheel_control/flywheel_speed_sim`                    |
| Flywheel HIL position   | `python3 run_sim.py --hil position`                 | `build/flywheel_control/position/` + firmware                  |
| Flywheel HIL speed      | `python3 run_sim.py --hil speed`                    | `build/flywheel_control/speed/` + firmware                     |
| Flywheel HIL (no flash) | `python3 run_sim.py --hil-no-flash position\|speed` | `build/flywheel_control/{position,speed}/` + firmware          |
| Furnace                 | `python3 run_sim.py`                                | `build/furnace_control/furnace_sim`                            |
| Furnace HIL             | `python3 run_sim.py --hil`                          | `build/furnace_control/furnace_sim_hil` + firmware             |
| Furnace HIL (no flash)  | `python3 run_sim.py --hil-no-flash`                 | `build/furnace_control/furnace_sim_hil` + firmware             |

### Controls during runtime

- **`S`** — Enable compensator (press in the simulator terminal)
- **`T`** — Disable compensator
- **`Q`** — Quit the simulator
- **`Ctrl+C`** — Stop the script and clean up all processes

### Important notes

- **HIL mode (`--hil`):** Builds host + firmware, flashes MCU via `st-flash`, then runs. Requires an STM32F411 on `/dev/ttyACM0` with `st-flash` installed. See [§6.3 Flashing Firmware](#63-flashing-firmware).
- **HIL no-flash (`--hil-no-flash`):** Builds host + firmware, skips flashing, then immediately launches the host simulator and visualizers. Flash `MDCS_HIL.bin` first with your own programmer (OpenOCD, STM32CubeProgrammer, J-Link, etc.); the script does not pause for you to flash before starting the HIL host.
- **Help (`--help`, `-h`):** Prints usage information and exits.
- **Socat bridges:** The script reuses existing bridges if they already exist; otherwise it creates new ones. You do not need to start socat manually.
- **Working directory:** Always run `run_sim.py` from *inside* the model directory.
- **Build:** Always rebuilds on every run — no need for `make clean` between changes.

---

**Support via PayPal:** [Donate via PayPal](https://paypal.me/embeddedfreedom)
