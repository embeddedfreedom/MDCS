#!/usr/bin/env bash
#
# scripts/socat_bridges.sh
#
# Creates the MDCS socat virtual serial bridges using the central config in
# framework/common/mdcs_config.sh (generated from mdcs_config.h by
# scripts/gen_mdcs_config.py).
#
# Run once, in a terminal you leave open:
#
#     ./scripts/socat_bridges.sh
#
# The C simulator opens *_in (O_WRONLY | O_NONBLOCK); socat creates them.
# DO NOT close this terminal — closing it destroys the bridges.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_SH="$ROOT_DIR/framework/common/mdcs_config.sh"

if [[ ! -f "$CONFIG_SH" ]]; then
    echo "ERROR: $CONFIG_SH not found — run scripts/gen_mdcs_config.py first" >&2
    exit 1
fi

# Source the generated config (exports MDCS_PTY_SIM_3D, *_IN, etc.)
# shellcheck source=/dev/null
source "$CONFIG_SH"

# Kill any stale socat bridges first
pkill -9 socat 2>/dev/null || true

# Remove stale PTY links so socat can recreate them cleanly
rm -f "$MDCS_PTY_SIM_3D" "$MDCS_PTY_SIM_3D_IN" \
      "$MDCS_PTY_SIM_GRAPH" "$MDCS_PTY_SIM_GRAPH_IN"

echo "Creating socat bridges ..."
echo "  3D : $MDCS_PTY_SIM_3D  <->  $MDCS_PTY_SIM_3D_IN"
echo "  GR : $MDCS_PTY_SIM_GRAPH  <->  $MDCS_PTY_SIM_GRAPH_IN"

socat PTY,raw,echo=0,link="$MDCS_PTY_SIM_3D" PTY,raw,echo=0,link="$MDCS_PTY_SIM_3D_IN" &
socat PTY,raw,echo=0,link="$MDCS_PTY_SIM_GRAPH" PTY,raw,echo=0,link="$MDCS_PTY_SIM_GRAPH_IN" &

echo "Bridges created. Keep this terminal open for the duration of use."
