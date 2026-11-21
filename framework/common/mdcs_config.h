/* 
 * Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file mdcs_config.h
 * @brief MDCS central configuration — single source of truth for all paths.
 *
 * Edit THIS file when moving the repo to another machine/user, then re-run
 * scripts/gen_mdcs_config.py to regenerate framework/common/mdcs_config.py
 * and framework/common/mdcs_config.sh so C, Python and shell stay in sync.
 *
 * Every model's simulator, run_sim.py, compensator_hil.c and Python
 * visualisers read from this header (C) or its generated mirror (Python).
 */

#ifndef MDCS_CONFIG_H
#define MDCS_CONFIG_H

/* ========================================================================
 * 1. Socat virtual serial bridges — shared by ALL models.
 *    These must match the socat PTY links exactly.
 * ======================================================================== */
#define MDCS_PTY_SIM_3D         "/home/dipto/sim_3d"
#define MDCS_PTY_SIM_3D_IN      "/home/dipto/sim_3d_in"
#define MDCS_PTY_SIM_GRAPH      "/home/dipto/sim_graph"
#define MDCS_PTY_SIM_GRAPH_IN   "/home/dipto/sim_graph_in"

/* Telemetry baud used by the Python visualisers (must match the socat
 * PTY link expectations — informational here, used by run_sim.py docs). */
#define MDCS_SERIAL_BAUD        115200

/* ========================================================================
 * 2. HIL serial port — STM32F411 on the host side.
 *    Used by every model's compensator/compensator_hil.c
 * ======================================================================== */
#define MDCS_HIL_PORT           "/dev/ttyACM0"
#define MDCS_HIL_BAUD           921600

/* ========================================================================
 * 3. HIL test defaults (test harness, not the simulator).
 * ======================================================================== */
#define MDCS_HIL_TEST_PORT      "/dev/ttyUSB0"

#if defined(__cplusplus)
extern "C" {
#endif

/* Placeholder so the header is never empty and can be #included safely. */
#define MDCS_CONFIG_VERSION     1

#if defined(__cplusplus)
}
#endif

/* ========================================================================
 * 4. Per-model sections (informational build-location hints, not paths
 *    opened at runtime). Keep in sync with each model's Makefile.
 * ======================================================================== */

/* rotary_inverted_pendulum (furuta) — SIL: build/furuta/furuta_sim */
#define MDCS_MODEL_FURUTA_BUILD_DIR   "build/furuta"

/* pendulum_with_DC_motor — SIL: build/inverted_pendulum/inverted_pendulum_sim */
#define MDCS_MODEL_PENDULUM_BUILD_DIR "build/inverted_pendulum"

/* flywheel_with_DC_motor/fixed_weight_flywheel — SIL: build/flywheel_control/ */
#define MDCS_MODEL_FLYWHEEL_BUILD_DIR "build/flywheel_control"

/* furnace_control — SIL: build/furnace_control/furnace_sim */
#define MDCS_MODEL_FURNACE_BUILD_DIR  "build/furnace_control"

#endif /* MDCS_CONFIG_H */
