"""
Auto-generated from framework/common/mdcs_config.h by scripts/gen_mdcs_config.py.
DO NOT EDIT by hand — edit the .h file and re-run the generator.

Single source of truth for MDCS paths (socat PTY links, HIL serial port, etc.).
Imported by run_sim.py and the Python visualisers.
"""


MDCS_PTY_SIM_3D = '/home/dipto/sim_3d'
MDCS_PTY_SIM_3D_IN = '/home/dipto/sim_3d_in'
MDCS_PTY_SIM_GRAPH = '/home/dipto/sim_graph'
MDCS_PTY_SIM_GRAPH_IN = '/home/dipto/sim_graph_in'
MDCS_SERIAL_BAUD = 115200
MDCS_HIL_PORT = '/dev/ttyACM0'
MDCS_HIL_BAUD = 921600
MDCS_HIL_TEST_PORT = '/dev/ttyUSB0'
MDCS_CONFIG_VERSION = 1
MDCS_MODEL_FURUTA_BUILD_DIR = 'build/furuta'
MDCS_MODEL_PENDULUM_BUILD_DIR = 'build/inverted_pendulum'
MDCS_MODEL_FLYWHEEL_BUILD_DIR = 'build/flywheel_control'
MDCS_MODEL_FURNACE_BUILD_DIR = 'build/furnace_control'
