#!/usr/bin/env python3
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
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
#!/usr/bin/env python3
import os
import sys
from pathlib import Path

# File extensions and explicit filenames to include
ALLOWED_EXTENSIONS = {'.c', '.h', '.ld'}
ALLOWED_FILENAMES = {'makefile', 'Makefile', 'GNUmakefile'}

def is_target_file(file_path: Path) -> bool:
    """Check if the file matches our C/Makefile code criteria."""
    if file_path.suffix.lower() in ALLOWED_EXTENSIONS:
        return True
    if file_path.name in ALLOWED_FILENAMES:
        return True
    return False

def build_tree_ascii(dir_path: Path, prefix: str = "") -> list[str]:
    """Recursively generate ASCII tree lines for relevant files and folders."""
    lines = []
    
    # Filter entries to only include directories or allowed target files
    try:
        entries = sorted(list(dir_path.iterdir()), key=lambda e: (e.is_file(), e.name.lower()))
    except PermissionError:
        return lines

    # Filter out empty directories or non-target files
    valid_entries = []
    for entry in entries:
        if entry.is_dir():
            # Keep directory if it contains target files anywhere inside
            if any(is_target_file(p) for p in entry.rglob('*') if p.is_file()):
                valid_entries.append(entry)
        elif is_target_file(entry):
            valid_entries.append(entry)

    count = len(valid_entries)
    for i, entry in enumerate(valid_entries):
        is_last = (i == count - 1)
        connector = "└── " if is_last else "├── "
        lines.append(f"{prefix}{connector}{entry.name}")
        
        if entry.is_dir():
            extension = "    " if is_last else "│   "
            lines.extend(build_tree_ascii(entry, prefix + extension))
            
    return lines

def flatten_project(source_dir: str, output_file: str):
    root = Path(source_dir).resolve()
    if not root.exists() or not root.is_dir():
        print(f"Error: Directory '{source_dir}' does not exist.")
        sys.exit(1)

    print(f"Scanning directory: {root}")

    # Collect target files
    target_files = sorted([p for p in root.rglob('*') if p.is_file() and is_target_file(p)])

    with open(output_file, 'w', encoding='utf-8') as out:
        # 1. Write File Header & ASCII Tree
        out.write("===============================================================================\n")
        out.write(f"PROJECT STRUCTURE: {root.name}\n")
        out.write("===============================================================================\n\n")
        out.write(f"{root.name}/\n")
        tree_lines = build_tree_ascii(root)
        out.write("\n".join(tree_lines))
        out.write("\n\n" + "=" * 79 + "\n\n")

        # 2. Append File Contents
        for file_path in target_files:
            rel_path = file_path.relative_to(root)
            print(f"Packing: {rel_path}")

            # Determine syntax highlight mode
            if file_path.name in ALLOWED_FILENAMES:
                lang = "makefile"
            elif file_path.suffix == ".ld":
                lang = "c"  # GCC linker scripts render cleanly with C highlighting
            else:
                lang = file_path.suffix.lstrip('.')

            out.write(f"### File: `{rel_path}`\n")
            out.write(f"```{lang}\n")
            
            try:
                content = file_path.read_text(encoding='utf-8', errors='replace')
                out.write(content)
                if not content.endswith('\n'):
                    out.write('\n')
            except Exception as e:
                out.write(f"/* Error reading file: {e} */\n")
                
            out.write("```\n\n" + "-" * 79 + "\n\n")

    print(f"\nSuccess! Combined {len(target_files)} files into '{output_file}'.")

if __name__ == "__main__":
    # Default to current directory if no argument is passed
    target_directory = sys.argv[1] if len(sys.argv) > 1 else "."
    output_filename = sys.argv[2] if len(sys.argv) > 2 else "bundled_project.txt"
    
    flatten_project(target_directory, output_filename)
