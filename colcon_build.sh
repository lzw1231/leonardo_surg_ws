#!/bin/bash
# =============================================================================
# ROS2 Workspace Build Script
# Toolchain: LLVM 22 (clang / clang++ / lld)  |  Backend: Ninja
# CCJ Strategy: Full=bear | Incremental=bear capture + python merge
#
# Usage:
#   ./build.sh          - Incremental build (bear capture + merge with old CCJ)
#   ./build.sh --clean  - Full clean rebuild (bear full capture)
# =============================================================================

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
ROS_DISTRO="jazzy"
QT6_PATH="/home/lzw/Qt/6.11.1/gcc_64"
LLVM_BIN="/opt/LLVM-22/bin"
CMAKE_BIN="/opt/cmake-4/bin/cmake"

CC="${LLVM_BIN}/clang"
CXX="${LLVM_BIN}/clang++"
AR="${LLVM_BIN}/llvm-ar"
RANLIB="${LLVM_BIN}/llvm-ranlib"

# Terminal colors & symbols
GREEN='\033[0;32m'
CYAN='\033[0;36m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
BOLD='\033[1m'
NC='\033[0m'

SYM_OK="✔"
SYM_FAIL="✘"
SYM_WARN="⚠"

# Shared colcon build arguments (single source of truth)
COLCON_BUILD_ARGS=(
    --event-handlers console_direct+ status+
    --cmake-args
        -G Ninja
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        -DCMAKE_C_COMPILER="${CC}"
        -DCMAKE_CXX_COMPILER="${CXX}"
        -DCMAKE_AR="${AR}"
        -DCMAKE_RANLIB="${RANLIB}"
        -DCMAKE_C_COMPILER_AR="${AR}"
        -DCMAKE_CXX_COMPILER_AR="${AR}"
        -DCMAKE_VERBOSE_MAKEFILE=OFF
        -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld"
        -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld"
        -DCMAKE_MODULE_LINKER_FLAGS="-fuse-ld=lld"
)

# -----------------------------------------------------------------------------
# Prepare environment
# -----------------------------------------------------------------------------
export PATH="$(dirname "${CMAKE_BIN}"):$PATH"
export PATH="${LLVM_BIN}:$PATH"
export LD_LIBRARY_PATH="${LLVM_BIN}/../lib:${LD_LIBRARY_PATH}"
export CC CXX AR RANLIB

SCRIPT_PATH=$(readlink -f "$0")
WS_ROOT=$(cd "$(dirname "$SCRIPT_PATH")" && pwd)
cd "$WS_ROOT" || exit 1

# Prevent cross-contamination from previously sourced ROS environments
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

source "/opt/ros/${ROS_DISTRO}/setup.bash"
export CMAKE_PREFIX_PATH="${QT6_PATH}:${CMAKE_PREFIX_PATH}"

# =============================================================================
# Embedded Python Merger (avoids jq -s memory issues + stale entry cleanup)
# =============================================================================
run_python_merge() {
    local old_ccj="$1"
    local new_ccj="$2"
    local out_ccj="$3"

    python3 - "$old_ccj" "$new_ccj" "$out_ccj" << 'PYTHON_SCRIPT'
import json, sys, os

def merge_ccj(old_path, new_path, out_path):
    entries = {}

    # Load old entries first (lower priority)
    if old_path and os.path.isfile(old_path) and os.path.getsize(old_path) > 0:
        try:
            with open(old_path, 'r') as f:
                for entry in json.load(f):
                    if isinstance(entry, dict) and 'file' in entry:
                        entries[entry['file']] = entry
        except (json.JSONDecodeError, ValueError) as e:
            print(f"[WARN] Failed to parse old CCJ: {e}", file=sys.stderr)

    # Load new entries (higher priority, overrides old)
    if not os.path.isfile(new_path) or os.path.getsize(new_path) == 0:
        print("[WARN] New CCJ is empty or missing.", file=sys.stderr)
        return

    try:
        with open(new_path, 'r') as f:
            for entry in json.load(f):
                if isinstance(entry, dict) and 'file' in entry:
                    entries[entry['file']] = entry
    except (json.JSONDecodeError, ValueError) as e:
        print(f"[ERROR] Failed to parse new CCJ: {e}", file=sys.stderr)
        sys.exit(1)

    # Filter CMake internals AND stale entries (deleted/moved files)
    filtered = [
        e for e in entries.values()
        if os.path.isfile(e.get('file', ''))
        and 'CMakeScratch' not in e.get('directory', '')
        and 'CompilerId' not in e.get('directory', '')
        and 'gtest_vendor' not in e.get('file', '')
    ]

    with open(out_path, 'w') as f:
        json.dump(filtered, f, indent=2)

    print(f"[INFO] Merged CCJ: {len(filtered)} entries (stale removed, new overrides old).")

if __name__ == '__main__':
    old = sys.argv[1] if len(sys.argv) > 1 else ''
    new = sys.argv[2] if len(sys.argv) > 2 else ''
    out = sys.argv[3] if len(sys.argv) > 3 else 'compile_commands.json'
    merge_ccj(old, new, out)
PYTHON_SCRIPT
}

# =============================================================================
# Main entry point
# =============================================================================
main() {
    if [[ "${1:-}" == "--clean" ]]; then
        build_full
    else
        build_incremental
    fi
}

# -----------------------------------------------------------------------------
# Professional Logging Helpers
# -----------------------------------------------------------------------------
log_info() {
    echo -e "${CYAN}[Build] ${BOLD}▶${NC} ${CYAN}$1${NC}"
}

log_success() {
    local duration=$1
    echo -e "\n${GREEN}${BOLD}[Build] ${SYM_OK} Completed successfully in ${duration}s${NC}"
}

log_error() {
    local stage="$1"
    echo -e "\n${RED}${BOLD}[Build] ${SYM_FAIL} ${stage} FAILED.${NC}"
}

log_warn() {
    echo -e "${YELLOW}${BOLD}[Build] ${SYM_WARN} $1${NC}"
}

# -----------------------------------------------------------------------------
# Build functions
# -----------------------------------------------------------------------------
build_full() {
    local start_time=$(date +%s)
    log_info "[Full] Cleaning build/ install/ log/ compile_commands.json ..."
    rm -rf build/ install/ log/ compile_commands.json

    log_info "[Full] Starting bear + colcon (Clang-22, LLD, Ninja)..."
    bear --output compile_commands_new.json -- \
        colcon build "${COLCON_BUILD_ARGS[@]}"

    local status=$?
    if [ $status -ne 0 ]; then
        log_error "Full Build"
        rm -f compile_commands_new.json
        exit $status
    fi

    # Full build: no old CCJ, just filter and output
    run_python_merge "" compile_commands_new.json compile_commands.json
    rm -f compile_commands_new.json

    local end_time=$(date +%s)
    log_success "$((end_time - start_time))"
}

build_incremental() {
    local start_time=$(date +%s)
    log_info "[Incr] Starting bear + colcon (incremental capture)..."

    # Backup old CCJ for merging (do NOT delete root CCJ to keep CLion indexing alive)
    local old_ccj=""
    if [[ -f compile_commands.json ]]; then
        cp compile_commands.json compile_commands.json.bak
        old_ccj="compile_commands.json.bak"
    else
        log_warn "No existing CCJ found. Generating fresh index after build."
    fi

    bear --output compile_commands_new.json -- \
        colcon build "${COLCON_BUILD_ARGS[@]}"

    local status=$?
    if [ $status -ne 0 ]; then
        log_error "Incremental Build"
        rm -f compile_commands_new.json compile_commands.json.bak
        exit $status
    fi

    # Merge: new captures override old; unmodified files preserved; deleted files removed
    run_python_merge "$old_ccj" compile_commands_new.json compile_commands.json

    # Cleanup temp files
    rm -f compile_commands_new.json compile_commands.json.bak

    local end_time=$(date +%s)
    log_success "$((end_time - start_time))"
}

main "$@"