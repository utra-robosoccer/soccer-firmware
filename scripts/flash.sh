#!/bin/sh
# Flash an STM32 binary from a CubeIDE project in this repo.
# Usage: flash.sh [--list | <project-path>]

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# Extract <name> from a .project file (first occurrence only)
project_name() {
    grep '<name>' "$1" | head -1 | sed 's/.*<name>\(.*\)<\/name>.*/\1/'
}

list_projects() {
    printf "Available projects:\n"
    find "$REPO_ROOT" -name ".project" | sort | while IFS= read -r proj_file; do
        proj_dir="$(dirname "$proj_file")"
        name="$(project_name "$proj_file")"
        rel_dir="${proj_dir#"$REPO_ROOT/"}"
        # Skip the repo-root workspace .project (not a flashable project)
        [ "$proj_dir" = "$REPO_ROOT" ] && continue
        printf "  %-40s  (binary: %s.elf)\n" "$rel_dir" "$name"
    done
}

# ── argument handling ────────────────────────────────────────────────────────

if [ $# -eq 0 ] || [ "$1" = "--list" ] || [ "$1" = "-l" ]; then
    list_projects
    exit 0
fi

# Optional --config <slaveN>: regenerate that slave's motor_config.h and rebuild
# (via build.sh) before flashing, so one shared source tree flashes a per-slave
# binary. Without it, the already-built Debug/*.elf is flashed as-is.
CONFIG_SLAVE=""
if [ "$1" = "--config" ]; then
    shift
    CONFIG_SLAVE="$1"
    [ -n "$CONFIG_SLAVE" ] || { printf "error: --config needs a slave name (e.g. slave1)\n" >&2; exit 1; }
    shift
fi

PROJECT_ARG="$1"

# Resolve project directory (absolute or relative to repo root)
if [ -d "$PROJECT_ARG" ]; then
    PROJECT_DIR="$(cd "$PROJECT_ARG" && pwd)"
elif [ -d "$REPO_ROOT/$PROJECT_ARG" ]; then
    PROJECT_DIR="$(cd "$REPO_ROOT/$PROJECT_ARG" && pwd)"
else
    printf "error: project directory '%s' not found\n" "$PROJECT_ARG" >&2
    printf "Run '%s --list' to see available projects.\n" "$0" >&2
    exit 1
fi

PROJ_FILE="$PROJECT_DIR/.project"
if [ ! -f "$PROJ_FILE" ]; then
    printf "error: no .project file in '%s' — not a CubeIDE project\n" "$PROJECT_DIR" >&2
    printf "Run '%s --list' to see available projects.\n" "$0" >&2
    exit 1
fi

# ── optional: regenerate per-slave config + rebuild before flashing ───────────
if [ -n "$CONFIG_SLAVE" ]; then
    printf "Config: %s — regenerating + rebuilding before flash\n" "$CONFIG_SLAVE"
    "$REPO_ROOT/scripts/build.sh" --config "$CONFIG_SLAVE" "$PROJECT_DIR" || exit 1
fi

# ── locate the .elf ──────────────────────────────────────────────────────────

BIN_NAME="$(project_name "$PROJ_FILE")"
ELF="$PROJECT_DIR/Debug/${BIN_NAME}.elf"

if [ ! -f "$ELF" ]; then
    # Fallback: any .elf under Debug/ (handles edge cases)
    ELF="$(find "$PROJECT_DIR/Debug" -maxdepth 1 -name "*.elf" 2>/dev/null | head -1)"
fi

if [ -z "$ELF" ] || [ ! -f "$ELF" ]; then
    printf "error: no .elf found under '%s/Debug/'\n" "$PROJECT_DIR" >&2
    printf "Build the project in STM32CubeIDE first (Project → Build All).\n" >&2
    exit 1
fi

printf "Flashing: %s\n" "$ELF"

# ── pick a flash tool ────────────────────────────────────────────────────────

# ── pick a flash tool ────────────────────────────────────────────────────────

STM32_CLI=""
if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
    STM32_CLI="$(command -v STM32_Programmer_CLI)"
else
    # Fall back to the copy bundled inside STM32CubeIDE
    STM32_CLI="$(find /opt/st -maxdepth 7 -name "STM32_Programmer_CLI" -type f 2>/dev/null | head -1)"
fi

if [ -n "$STM32_CLI" ]; then
    printf "Tool: STM32_Programmer_CLI\n"
    "$STM32_CLI" \
        -c port=SWD \
        -w "$ELF" 0x08000000 \
        -v \
        -rst
elif command -v st-flash >/dev/null 2>&1; then
    printf "Tool: st-flash\n"
    st-flash --reset write "$ELF" 0x08000000
else
    printf "error: no flash tool found.\n" >&2
    printf "Install one of:\n" >&2
    printf "  STM32CubeProgrammer (provides STM32_Programmer_CLI)\n" >&2
    printf "    https://www.st.com/en/development-tools/stm32cubeprog.html\n" >&2
    printf "  stlink-tools (provides st-flash)\n" >&2
    printf "    sudo apt install stlink-tools\n" >&2
    exit 1
fi

printf "Done.\n"
