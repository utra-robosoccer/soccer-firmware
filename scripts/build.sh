#!/bin/sh
# Build an STM32CubeIDE project from the command line using the headless builder.
# Usage: build.sh [--list | [--clean] <project-path> [Debug|Release]]

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
        [ "$proj_dir" = "$REPO_ROOT" ] && continue
        printf "  %-40s  (project: %s)\n" "$rel_dir" "$name"
    done
}

# ── locate STM32CubeIDE ──────────────────────────────────────────────────────

# Searches common install locations; prints the path to stdout on success.
# STM32CUBEIDE_PATH env var and PATH lookup are handled inline below.
find_cubeide_auto() {
    for search_base in /opt/st /opt "$HOME/st" "$HOME"; do
        [ -d "$search_base" ] || continue
        found="$(find "$search_base" -maxdepth 2 -name "stm32cubeide" -type f 2>/dev/null | sort -V | tail -1)"
        if [ -n "$found" ] && [ -x "$found" ]; then
            printf "%s" "$found"
            return 0
        fi
    done
    return 1
}

# ── argument handling ────────────────────────────────────────────────────────

if [ $# -eq 0 ] || [ "$1" = "--list" ] || [ "$1" = "-l" ]; then
    list_projects
    exit 0
fi

CLEAN=0
CONFIG_SLAVE=""
# Parse leading options in any order: --clean / -c, --config <slaveN>
while [ $# -gt 0 ]; do
    case "$1" in
        --clean|-c) CLEAN=1; shift ;;
        --config)
            shift
            CONFIG_SLAVE="$1"
            [ -n "$CONFIG_SLAVE" ] || { printf "error: --config needs a slave name (e.g. slave1)\n" >&2; exit 1; }
            shift ;;
        *) break ;;
    esac
done

if [ $# -eq 0 ]; then
    printf "error: no project path given\n" >&2
    printf "Usage: %s [--clean] [--config <slaveN>] <project-path> [Debug|Release]\n" "$0" >&2
    exit 1
fi

PROJECT_ARG="$1"
BUILD_CONFIG="${2:-Debug}"

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

PROJ_NAME="$(project_name "$PROJ_FILE")"

# ── find CubeIDE ─────────────────────────────────────────────────────────────

if [ -n "$STM32CUBEIDE_PATH" ]; then
    if [ ! -x "$STM32CUBEIDE_PATH" ]; then
        printf "error: STM32CUBEIDE_PATH='%s' is not executable\n" "$STM32CUBEIDE_PATH" >&2
        exit 1
    fi
    CUBEIDE="$STM32CUBEIDE_PATH"
elif command -v stm32cubeide >/dev/null 2>&1; then
    CUBEIDE="$(command -v stm32cubeide)"
elif CUBEIDE="$(find_cubeide_auto)"; then
    : # found via filesystem search
else
    printf "error: STM32CubeIDE not found.\n" >&2
    printf "Install it from https://www.st.com/en/development-tools/stm32cubeide.html\n" >&2
    printf "or set STM32CUBEIDE_PATH=/path/to/stm32cubeide\n" >&2
    exit 1
fi

printf "CubeIDE: %s\n" "$CUBEIDE"
printf "Project: %s  config: %s\n" "$PROJ_NAME" "$BUILD_CONFIG"

# ── optional per-slave config regeneration ───────────────────────────────────
# --config <slaveN> regenerates the slave's motor_config.h from configs/<slaveN>.yaml
# so one shared source tree builds a per-slave binary.
if [ -n "$CONFIG_SLAVE" ]; then
    CONFIG_YAML="$REPO_ROOT/configs/${CONFIG_SLAVE}.yaml"
    if [ ! -f "$CONFIG_YAML" ]; then
        printf "error: config '%s' not found (looked for %s)\n" "$CONFIG_SLAVE" "$CONFIG_YAML" >&2
        exit 1
    fi
    printf "Config: regenerating motor_config.h from configs/%s.yaml\n" "$CONFIG_SLAVE"
    python3 "$REPO_ROOT/scripts/gen_motor_config.py" --slave "$CONFIG_YAML" || exit 1
fi

# ── run headless build ───────────────────────────────────────────────────────

if [ "$CLEAN" -eq 1 ]; then
    BUILD_FLAG="-cleanBuild"
    printf "Mode: clean + build\n"
else
    BUILD_FLAG="-build"
    printf "Mode: build\n"
fi

"$CUBEIDE" \
    --launcher.suppressErrors \
    -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "$REPO_ROOT" \
    -import "$PROJECT_DIR" \
    "$BUILD_FLAG" "${PROJ_NAME}/${BUILD_CONFIG}"

printf "Done.\n"
