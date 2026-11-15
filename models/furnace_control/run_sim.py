#!/usr/bin/env python3
"""
Run MDCS Furnace Control simulator with visualizers.

Automates: build -> socat bridges -> simulator -> visualizers.
Press Ctrl+C to stop everything.

Usage:
    python3 run_sim.py              # SIL build + run
    python3 run_sim.py --hil        # HIL build + flash + run
    python3 run_sim.py --hil-no-flash  # HIL build only (flash MCU yourself)
"""

import subprocess
import signal
import sys
import os
import time
import shutil

MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(MODEL_DIR))
BUILD_DIR = os.path.join(ROOT_DIR, "build", "furnace_control")
SIM_BINARY = os.path.join(BUILD_DIR, "furnace_sim")
HIL_BINARY = os.path.join(BUILD_DIR, "furnace_sim_hil")
HIL_BIN_PATH = os.path.join(BUILD_DIR, "MDCS_HIL.bin")

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


def check_stlink_for_hil():
    """Check that st-flash is available, the STM32 MCU is reachable, and flash firmware."""
    if shutil.which("st-flash") is None:
        print("\n" + "=" * 60)
        print("ERROR: st-flash not found on this system.")
        print("HIL mode requires the STM32 ST-LINK programmer tools.")
        print("Please install stlink-tools yourself, for example:")
        print("    sudo apt install stlink-tools")
        print("Then flash the firmware manually before running --hil:")
        print(f"    st-flash --reset write {HIL_BIN_PATH} 0x08000000")
        print("=" * 60 + "\n")
        sys.exit(1)

    # Try to probe the target MCU
    probe = subprocess.run(["st-info", "--probe"], capture_output=True, text=True)
    if probe.returncode != 0 or "No target connected" in probe.stdout or "failed" in probe.stdout.lower():
        print("\n" + "=" * 60)
        print("ERROR: STM32 target not detected by ST-LINK.")
        print("Check that:")
        print("  - The STM32F411 board is connected via USB")
        print("  - BOOT0 is set to 0 (normal flash mode)")
        print("  - The ST-LINK drivers are installed")
        print("Then flash the firmware manually before running --hil:")
        print(f"    st-flash --reset write {HIL_BIN_PATH} 0x08000000")
        print("=" * 60 + "\n")
        sys.exit(1)

    # Give the ST-LINK a moment to settle after probing
    time.sleep(1.0)

    # Build HIL firmware if the .bin is missing
    if not os.path.exists(HIL_BIN_PATH):
        print(f"\nFirmware binary not found: {HIL_BIN_PATH}")
        print(f"Building with: make -f Makefile_HIL")
        result = subprocess.run(["make", "-f", "Makefile_HIL"], cwd=MODEL_DIR)
        if result.returncode != 0:
            print("\n" + "=" * 60)
            print("ERROR: HIL firmware build failed!")
            print("=" * 60 + "\n")
            sys.exit(1)
        if not os.path.exists(HIL_BIN_PATH):
            print("\n" + "=" * 60)
            print(f"ERROR: Build completed but {HIL_BIN_PATH} was not created.")
            print("=" * 60 + "\n")
            sys.exit(1)

    print(f"\nFlashing firmware to STM32: {HIL_BIN_PATH}")
    flash = subprocess.run(
        ["st-flash", "--reset", "write", HIL_BIN_PATH, "0x08000000"],
        capture_output=True, text=True
    )
    flash_ok = (flash.returncode == 0) or ("Flash written and verified" in flash.stdout) or ("jolly good" in flash.stdout)
    if not flash_ok:
        # --reset reboots the board and USB disconnects; wait for re-enumeration
        print("Waiting for target to re-enumerate after reset...")
        time.sleep(3.0)
        # Fallback: try without --reset
        print("Retrying flash without --reset...")
        flash = subprocess.run(
            ["st-flash", "write", HIL_BIN_PATH, "0x08000000"],
            capture_output=True, text=True
        )
        flash_ok = (flash.returncode == 0) or ("Flash written and verified" in flash.stdout) or ("jolly good" in flash.stdout)
    if not flash_ok:
        print("\n" + "=" * 60)
        print("ERROR: Failed to flash firmware to STM32.")
        print(flash.stderr or flash.stdout)
        print("Check your connection and retry.")
        print("=" * 60 + "\n")
        sys.exit(1)

    print("Firmware flashed successfully. (Press the board's RESET button if the new firmware does not start.)\n")


def main():
    use_hil = "--hil" in sys.argv
    hil_no_flash = "--hil-no-flash" in sys.argv

    if hil_no_flash:
        binary = HIL_BINARY
        make_cmd = ["make", "-f", "Makefile_HIL"]
        do_flash = False
    elif use_hil:
        binary = HIL_BINARY
        make_cmd = ["make", "-f", "Makefile_HIL"]
        do_flash = True
    else:
        binary = SIM_BINARY
        make_cmd = ["make"]
        do_flash = False

    # Build: always rebuild (host + firmware for HIL, host for SIL)
    print(f"Building with: {' '.join(make_cmd)}")
    result = subprocess.run(make_cmd, cwd=MODEL_DIR)
    if result.returncode != 0:
        print("Build failed!")
        sys.exit(1)

    # If HIL requested with flashing, check st-link and flash firmware
    if do_flash:
        check_stlink_for_hil()

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
    anim_proc = subprocess.Popen(["python3", "visualiser/furnace_animation.py"], cwd=MODEL_DIR)
    tele_proc = subprocess.Popen(["python3", "visualiser/furnace_telemetry.py"], cwd=MODEL_DIR)
    processes.extend([anim_proc, tele_proc])

    print("\n=== Running ===")
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
