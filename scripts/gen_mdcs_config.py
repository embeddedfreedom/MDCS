#!/usr/bin/env python3
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
"""
Generate framework/common/mdcs_config.py and framework/common/mdcs_config.sh
from framework/common/mdcs_config.h (the single source of truth).

Usage:
    python3 scripts/gen_mdcs_config.py

Re-run this whenever mdcs_config.h changes so C, Python and shell stay in sync.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HEADER = os.path.join(ROOT, "framework", "common", "mdcs_config.h")
PY_OUT = os.path.join(ROOT, "framework", "common", "mdcs_config.py")
SH_OUT = os.path.join(ROOT, "framework", "common", "mdcs_config.sh")

# Matches:  #define MDCS_NAME   "value"    or   #define MDCS_NAME   123
DEFINE_RE = re.compile(
    r'^\s*#\s*define\s+(MDCS_[A-Z0-9_]+)\s+(".*?"|\S+)'
)

HEADER_PY = '''\"\"\"
Auto-generated from framework/common/mdcs_config.h by scripts/gen_mdcs_config.py.
DO NOT EDIT by hand — edit the .h file and re-run the generator.

Single source of truth for MDCS paths (socat PTY links, HIL serial port, etc.).
Imported by run_sim.py and the Python visualisers.
\"\"\"

'''

HEADER_SH = '''# Auto-generated from framework/common/mdcs_config.h by scripts/gen_mdcs_config.py.
# DO NOT EDIT by hand — edit the .h file and re-run the generator.
# Sourced by scripts/socat_bridges.sh (and any shell wrapper needing the paths).

'''


def parse_header(path):
    """Return list of (name, value) tuples preserving file order."""
    entries = []
    with open(path, "r") as f:
        for line in f:
            m = DEFINE_RE.match(line)
            if not m:
                continue
            name = m.group(1)
            value = m.group(2)
            # Strip surrounding quotes for Python; keep them for shell export.
            entries.append((name, value))
    return entries


def emit_py(entries):
    lines = [HEADER_PY]
    for name, value in entries:
        if value.startswith('"') and value.endswith('"'):
            py_value = value[1:-1]  # string
            lines.append(f'{name} = {py_value!r}')
        else:
            lines.append(f'{name} = {value}')
    lines.append("")
    return "\n".join(lines)


def emit_sh(entries):
    lines = [HEADER_SH]
    for name, value in entries:
        if value.startswith('"') and value.endswith('"'):
            sh_value = value[1:-1]
        else:
            sh_value = value
        lines.append(f'export {name}="{sh_value}"')
    lines.append("")
    return "\n".join(lines)


def main():
    if not os.path.isfile(HEADER):
        print(f"ERROR: header not found: {HEADER}", file=sys.stderr)
        return 1

    entries = parse_header(HEADER)
    if not entries:
        print("ERROR: no MDCS_ defines found in header", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(PY_OUT), exist_ok=True)

    with open(PY_OUT, "w") as f:
        f.write(emit_py(entries))
    print(f"Wrote {PY_OUT} ({len(entries)} entries)")

    with open(SH_OUT, "w") as f:
        f.write(emit_sh(entries))
    print(f"Wrote {SH_OUT} ({len(entries)} entries)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
