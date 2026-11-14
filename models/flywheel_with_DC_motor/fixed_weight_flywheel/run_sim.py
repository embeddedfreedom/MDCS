#!/usr/bin/env python3
"""
Run MDCS Flywheel with DC Motor simulator with visualizers.

Automates: build -> socat bridges -> simulator -> visualizers.
Press Ctrl+C to stop everything.

Usage:
    python3 run_sim.py              # SIL position build + run
    python3 run_sim.py speed        # SIL speed build + run
    python3 run_sim.py --hil        # HIL position build + run (flash MCU first!)
    python3 run_sim.py --hil speed  # HIL speed build + run (flash MCU first!)
"""

import subprocess
import signal
import sys
import os
import time

MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(MODEL_DIR)))
BUILD_DIR = os.path.join(ROOT_DIR, "build", "flywheel_control")

# Map profile to binary paths
SIM_BINARIES = {
    "position": os.path.join(BUILD_DIR, "flywheel_position_sim"),
    "speed": os.path.join(BUILD_DIR, "flywheel_speed_sim"),
}
HIL_BINARIES = {
    "position": os.path.join(BUILD_DIR, "position", "flywheel_hil_sim"),
    "speed": os.path.join(BUILD_DIR, "speed", "flywheel_hil_sim"),
}

SIM_3D = "/home/dipto/sim_3d"
SIM_3D_IN = "/home/dipto/sim_3d_in"
SIM_GRAPH = "/home/dipto/sim_graph"
SIM_GRAPH_IN = "/home/dipto/sim_graph_in"

processes = []


def cleanup(signum=None, frame=None):
    print("\n\nShutting down...")
    for p in processes:
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    try:
        subprocess.run(["pkill", "-9", "socat"], capture_output=True)
    except Exception:
        pass
    print("Done.")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    profile = args[0] if args else "position"
    if profile not in ("position", "speed"):
        print(f"Invalid profile: {profile}. Use 'position' or 'speed'.")
        sys.exit(1)

    use_hil = "--hil" in sys.argv

    binary = HIL_BINARIES[profile] if use_hil else SIM_BINARIES[profile]
    make_cmd = ["make", "-f", "Makefile_HIL", profile] if use_hil else ["make", profile]

    print(f"Profile: {profile}")
    print(f"Mode: {'HIL' if use_hil else 'SIL'}")
    print(f"Binary: {binary}")
    print(f"Binary exists: {os.path.exists(binary)}")

    # Build if binary missing
    if not os.path.exists(binary):
        print(f"Binary not found: {binary}")
        print(f"Building with: {' '.join(make_cmd)}")
        result = subprocess.run(make_cmd, cwd=MODEL_DIR)
        if result.returncode != 0:
            print("Build failed!")
            sys.exit(1)

    # Check if socat bridges already exist
    bridges_exist = all(os.path.exists(f) for f in [SIM_3D, SIM_3D_IN, SIM_GRAPH, SIM_GRAPH_IN])

    if bridges_exist:
        print("Socat bridges already exist — reusing.")
    else:
        # Clean up any stale sockets from previous runs
        subprocess.run(["pkill", "-9", "socat"], capture_output=True)
        for f in [SIM_3D, SIM_3D_IN, SIM_GRAPH, SIM_GRAPH_IN]:
            try:
                os.remove(f)
            except FileNotFoundError:
                pass
            except Exception:
                pass

        # Start socat bridges
        print("Creating socat bridges...")
        p1 = subprocess.Popen(
            ["socat", f"PTY,raw,echo=0,link={SIM_3D}", f"PTY,raw,echo=0,link={SIM_3D_IN}"]
        )
        p2 = subprocess.Popen(
            ["socat", f"PTY,raw,echo=0,link={SIM_GRAPH}", f"PTY,raw,echo=0,link={SIM_GRAPH_IN}"]
        )
        processes.extend([p1, p2])
        time.sleep(0.5)

    # Start simulator
    print(f"Starting simulator: {binary}")
    sim_proc = subprocess.Popen([binary])
    processes.append(sim_proc)

    # Start visualizers
    print("Starting visualizers...")
    anim_proc = subprocess.Popen(["python3", "visualiser/motor_and_flywheel_animation.py"], cwd=MODEL_DIR)
    tele_proc = subprocess.Popen(["python3", "visualiser/motor_and_flywheel_telemetry.py"], cwd=MODEL_DIR)
    processes.extend([anim_proc, tele_proc])

    print("\n=== Running ===")
    print(f"Profile: {profile} | Mode: {'HIL' if use_hil else 'SIL'}")
    print("Simulator + visualizers are running.")
    print("Press 'S' in the simulator terminal to enable the compensator.")
    print("Press Ctrl+C to stop everything.\n")

    try:
        while True:
            time.sleep(1)
            if sim_proc.poll() is not None:
                print("Simulator exited.")
                break
    except KeyboardInterrupt:
        pass

    cleanup()


if __name__ == "__main__":
    main()
