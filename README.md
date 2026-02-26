Copyright (c) 2026 Diptopal Basu (embeddedfreedom)

Licensed under the MIT License

Pendulum Controller: 1kHz Compensator / 20kHz Simulation

# Installation Manual and Architecture Overview: Modular Dynamics Control Suite (MDCS)

**Date:** February 2026

**Version:** 1.0

**Executive Summary & Configuration:** This document serves as the installation and configuration manual and the architecture overview document for the MDCS simulation environment. It details the setup of the 20kHz simulation engine, the 1kHz compensator interface, and the virtual serial bridge configuration required for real-time visualization and control. This repository contains a collection of independent, hardcoded dynamical models. This manual also details the setup required to bridge these C-based simulations to the Python visualizers via virtual serial ports. 

### 🚀 Quick Start & Live Demo

Before setting up the local environment, you can explore the system architecture and see the controllers in action via the official project resources:

- **Dashboard:** [embeddedfreedom.dev](https://www.embeddedfreedom.dev) — *A web-based overview of the MDCS models, system specifications, and ecosystem positioning.*

- **Video Walkthrough:** [MDCS Setup & Demonstration](https://youtu.be/eaQ9mskr6E8) — *A 7-minute guide covering the SIL loop, telemetry plotter interaction, and 3D OpenGL visualization.*

## 1. Architecture Overview

MDCS provides a **Hardware-Emulated** environment—a Software-in-the-Loop (SIL) Digital Twin that faithfully models a system using **Lagrangian mechanics** and the **RK4 solver**. Unlike theoretical scripts, this is designed to be interacted with as if it were a real plant sitting on your desk. It utilizes **C**, the industry standard for embedded systems, ensuring the transition from simulation to real hardware is seamless.

To enrich the student experience further, the project includes a **High-Fidelity 3D Visualizer** and **Real-Time Telemetry Graphs**. By mimicking noisy feedback and hardware signals, MDCS provides a bench-like environment that challenges you to solve real-world problems like sensor jitter and encoder drift.

The SIL project is built for **Linux Desktop** environments and utilizes a **high-precision timing super-loop** to emulate a bare-metal microcontroller environment. To mitigate general-purpose OS jitter, the framework employs a **lockstep architecture** where the physics solver and compensator are synchronously coupled; if the OS introduces a scheduling delay, the entire simulation state remains frozen to preserve **temporal determinism**. The system implements a **self-healing absolute timeline** that automatically corrects for OS-induced blocking to prevent cumulative timing drift against the wall clock maintaining seamless visual realism regardless of background CPU load.

![Diagram](general_architecture.png)

#### 1.1 Modeling Fidelity & Abstractions

Bridging the digital-to-physical gap with intentional modeling choices:

##### Actuator (PWM)

Utilizes a **Mean Value Model**. The compensator returns a normalized duty cycle. At a 20:1 physics-to-control ratio, the plant's inertia acts as a natural low-pass filter, validating this approach for rigid-body simulation.

##### Sensors (Encoders)

Explicitly modeled as **Discrete Quadrature Pulses (A/B)**. This forces the compensator to manage quantization error and velocity jitter. Absolute positioning is verified via **Z-Index** latching at the **0° (Down)** reference point.

### ⚠️ CRITICAL OPERATIONAL LIMITATION

The `socat` configuration used in this suite creates a specific set of virtual serial bridges. This setup only accommodates **one** active control suite at a time.

- **Do not run two models simultaneously.**

- Attempting to run a second model (e.g., running `inverted_pendulum` while `furuta_pendulum` is active) will result in serial port resource conflicts and data corruption.

## 2. Prerequisites

### System Dependencies

- **Any modern Linux distribution** (**tested: Linux Mint 21.3 / Ubuntu 22.04+ equivalents**)

- **Python 3.x***

- **GCC** (Build Essentials)

- **socat**: Virtual serial port utility.
  
  Bash
  
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
pip3 install pyserial numpy pyside6 pyqtgraph pygame PyOpenGL
```

---

## 3. Project Structure

The project is organized into modular directories. For specific mathematical proofs, control theory diagrams, and stability analysis, refer to the **`docs/`** folder.

**📁 models/[model_name]/**

Each experimental setup (e.g., rotary_inverted_pendulum) contains the following sub-directories:

- **`compensator/`**:  Contains the actual control law and signal processing logic. This is the primary area for student modification.

- **`simulator/`**: The core physics engine. It runs at **20kHz** to ensure high-fidelity integration and numerical stability.

- **`visualiser/`**: Includes the 1kHz real-time plotter and the OpenGL 3D rendering environment.

- **docs/**  :This folder contains the theoretical foundation of the project:
  
  - **Model Derivations:** Full Lagrangian or Newtonian derivations of the equations of motion (EOM) for each system.
  
  - **Design Diagrams:** Includes the **System Block Diagrams**. These visualize the control architecture, showing how sensor data is filtered, passed through the compensator, and converted into actuator commands (Torque/Voltage).

- **`Makefile`**: Every model folder includes a Makefile. Simply run `make` to compile the compensator and simulator into a single executable binary.

**📁 build/  :** Contains the binary for the plant after compiling. This directory has subdirectories with the respective model names under which the compiled binaries get created after make.

**📁 framework/  :** This contains common utilities like the encoder and decoder for the Quadrature pulses (in the codecs directory) and the keypress detect (in the common directory) applications. 

---

### 🛠 Student Workflow

1. **Review the `docs/`**: Understand the mathematical model before touching the code.

2. **Modify `compensator/`**: Implement your control gains or logic.

3. **Compile**: Use the `Makefile` in the model root.

4. **Simulate & Visualise**: Launch the binary and use the Python tools to monitor performance.

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

In ideal situations, step b above should be run only. Steps a and c should be followed during troubleshooting. This step must me started before starting the simulator, plotter or the visualiser. If not done, and the front end appears non responsive, start the applications again. No need to redo the socat commands. 

**DO NOT CLOSE THE WINDOW WHERE THE socat COMMAND IS RUN. CLOSING THE WINDOW CLOSES THE BRIDGE. SO KEEP THE WINDOW OPEN FOR THE DURATION OF THE USE**

#### Step 2: Source Code Path Configuration

**Crucial:** The paths used in the `socat` commands above (e.g., `/home/dipto/sim_3d`) must be **exactly** matched inside your source files:

- Update the device paths in the **C source files** (simulator/compensator).

- Update the serial port strings in the **Python files** (`plotter.py` and `visualizer_3d.py, these file names vary with models).

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
| **T**   | **Disable** | Dicconnects compensator from plant (`pwm = 0`); physics continue to run (free fall). |
| **Q**   | **Quit**    | Safely closes the simulation.                                                        |

 The model visualisers both graphical and the model will settle down like the real scenario. Pressing S again connects the compensator.  The output also displays timing jitter alerts if the control loop executes faster or slower than the expected interval of 1 millisecond. 

###### High-Precision Sync

The loop is governed by `clock_nanosleep` using the `CLOCK_MONOTONIC` timer. This ensures that the 50μs steps are consistent regardless of CPU load. A **Timing Jitter Alert** will print to the console if the 1kHz loop deviates by more than 5%.

#### 5.2 Telemetry Plotter Operational Manual

**Start the Plotter:** Run the Python script to see the system in action:

Bash

```
python3 visualiser/plotter.py (Names will vary with model)
```

The Telemetry plotter is a high-frequency (1kHz) data visualization and logging tool built with Python, PySide6, and PyQtGraph. Designed specifically for control systems engineering and hardware-in-the-loop testing.

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
python3 visualiser/visualizer_3d.py (Will differ based on model)
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

**NOTE:** The simulator and plotter can be started in any order, any one can also be started if the other is not required. This will not affect the visuals at all.

## 6. System Specifications & Logic

- **Timing:** 20kHz Simulation Loop | 1kHz Compensator.

- **Coordinates:** $0$ is Down (stable), $\pi$ (or $180^\circ$) is Up (unstable).

- **Convention:** Clockwise turns are **negative**; Counter-clockwise are **positive**.

- **Arm Range:** $0$ to $\pm 180^\circ$.

- **Data Format:** Tab-Delimited Fixed-Precision ASCII Stream (6-set packet of Radian values in case if furuta model). Value 1 = Arm, Value 2 = Pendulum.

- **UI:** The visualizer includes a toggle to switch between Radians and Degrees.

# 
