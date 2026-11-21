#!/usr/bin/env bash
# 
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
