#!/usr/bin/env bash
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
#      platforms/stm32h7/toolchain/ (see platforms/stm32h7/setup_toolchain.sh)
#   4. A permanent PATH entry: a marked block in ~/.bashrc, ~/.profile (and
#      ~/.zshrc if present) that sources platforms/stm32h7/toolchain/env.sh
#   5. Optionally Gazebo Harmonic (--with-gazebo)
#   6. Optionally the Zephyr workspace for the hardware firmware
#      (--with-zephyr, see platforms/zephyr/README.md)
#
# Safe to re-run: every step checks what is already installed, and the
# shell-profile block is replaced, never duplicated.
#
# Usage:
#   tools/setup_ubuntu_env.sh                 # apt + python + toolchain + PATH
#   tools/setup_ubuntu_env.sh --with-gazebo   # also install Gazebo Harmonic
#   tools/setup_ubuntu_env.sh --with-zephyr   # also set up Zephyr for hardware builds
#   tools/setup_ubuntu_env.sh --no-apt        # no sudo: python + toolchain + PATH only
#   tools/setup_ubuntu_env.sh --toolchain-only
#
# Afterwards open a new terminal (or: source ~/.bashrc) and build, e.g.
#   cmake --preset sitl && cmake --build build/sitl
#   cmake --preset stm32h7 && cmake --build build/stm32h7

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLCHAIN_SETUP="${REPO_ROOT}/platforms/stm32h7/setup_toolchain.sh"
ENV_FILE="${REPO_ROOT}/platforms/stm32h7/toolchain/env.sh"
MARK_BEGIN="# >>> APX environment (tools/setup_ubuntu_env.sh) >>>"
MARK_END="# <<< APX environment <<<"

do_apt=1
do_python=1
do_gazebo=0
do_zephyr=0

usage() {
    sed -n '2,28p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

for arg in "$@"; do
    case "${arg}" in
        --with-gazebo)    do_gazebo=1 ;;
        --with-zephyr)    do_zephyr=1 ;;
        --no-apt)         do_apt=0 ;;
        --toolchain-only) do_apt=0; do_python=0 ;;
        -h|--help)        usage; exit 0 ;;
        *) echo "Unknown option: ${arg}" >&2; usage; exit 2 ;;
    esac
done

log()  { echo -e "\033[1;34m[APX setup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[APX setup] WARNING:\033[0m $*" >&2; }
die()  { echo -e "\033[1;31m[APX setup] ERROR:\033[0m $*" >&2; exit 1; }

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    if [[ "${ID:-}" != "ubuntu" ]]; then
        warn "This script is written for Ubuntu; detected '${PRETTY_NAME:-unknown}'. apt steps may fail."
    fi
fi

# ---------------------------------------------------------------- 1. apt
if [[ ${do_apt} -eq 1 ]]; then
    command -v sudo >/dev/null || die "sudo not available. Re-run with --no-apt and install the packages listed in docs/build.md manually."
    log "Installing build dependencies with apt (sudo password may be requested)..."
    sudo apt-get update
    sudo apt-get install -y \
        build-essential cmake ninja-build git curl xz-utils ca-certificates \
        lsb-release gnupg python3 python3-pip python3-venv \
        libprotobuf-dev protobuf-compiler

    if [[ ${do_gazebo} -eq 1 ]]; then
        if dpkg -s gz-harmonic >/dev/null 2>&1; then
            log "Gazebo Harmonic already installed."
        else
            log "Installing Gazebo Harmonic from the OSRF apt repository..."
            sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
                -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
                | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
            sudo apt-get update
            sudo apt-get install -y gz-harmonic
        fi
    fi
else
    log "Skipping apt packages (--no-apt / --toolchain-only)."
    for tool in cmake ninja curl tar xz; do
        command -v "${tool}" >/dev/null || warn "'${tool}' not found on PATH - install it before building."
    done
fi

# ------------------------------------------------------------- 2. python
if [[ ${do_python} -eq 1 ]]; then
    if [[ -s "${REPO_ROOT}/requirements.txt" ]]; then
        log "Installing Python packages from requirements.txt (user site)..."
        # --break-system-packages is needed on Ubuntu 24.04 (PEP 668) and is
        # not understood by the older pip on 22.04, so try both forms.
        python3 -m pip install --user -r "${REPO_ROOT}/requirements.txt" 2>/dev/null \
            || python3 -m pip install --user --break-system-packages -r "${REPO_ROOT}/requirements.txt"
    fi
fi

# ---------------------------------------------------------- 3. toolchain
[[ -x "${TOOLCHAIN_SETUP}" ]] || die "Missing ${TOOLCHAIN_SETUP}"
log "Installing the pinned Arm GNU Toolchain (downloads ~150 MB the first time)..."
"${TOOLCHAIN_SETUP}"
[[ -f "${ENV_FILE}" ]] || die "Toolchain setup did not create ${ENV_FILE}"

# ------------------------------------------------------- 4. PATH (permanent)
install_profile_block() {
    local profile="$1"
    local tmp
    [[ -f "${profile}" ]] || touch "${profile}"
    tmp="$(mktemp)"
    # drop any previous APX block, then append the current one
    awk -v b="${MARK_BEGIN}" -v e="${MARK_END}" '
        $0 == b {skip = 1; next}
        $0 == e {skip = 0; next}
        !skip {print}' "${profile}" > "${tmp}"
    {
        echo "${MARK_BEGIN}"
        echo "if [ -f \"${ENV_FILE}\" ]; then . \"${ENV_FILE}\"; fi"
        echo "${MARK_END}"
    } >> "${tmp}"
    cat "${tmp}" > "${profile}"
    rm -f "${tmp}"
    log "PATH entry written to ${profile}"
}

install_profile_block "${HOME}/.bashrc"
install_profile_block "${HOME}/.profile"
[[ -f "${HOME}/.zshrc" ]] && install_profile_block "${HOME}/.zshrc"

# ------------------------------------------------------------- 5. zephyr
if [[ ${do_zephyr} -eq 1 ]]; then
    log "Setting up the Zephyr workspace for the hardware firmware (large download)..."
    "${REPO_ROOT}/platforms/zephyr/setup_zephyr.sh"
fi

# ------------------------------------------------------------- 6. verify
# shellcheck disable=SC1090
. "${ENV_FILE}"
log "Installed versions:"
arm-none-eabi-gcc --version | head -n1
command -v cmake >/dev/null && cmake --version | head -n1
command -v ninja >/dev/null && echo "ninja $(ninja --version)"
python3 -c "import pymavlink; print('pymavlink', getattr(pymavlink, '__version__', 'installed'))" 2>/dev/null \
    || warn "pymavlink not importable (SITL tests need it)."
command -v gz >/dev/null && echo "gazebo $(gz sim --versions 2>/dev/null | head -n1)" \
    || log "Gazebo not installed (optional; re-run with --with-gazebo)."

log "Done. Open a new terminal (or run: source ~/.bashrc) so PATH includes the ARM toolchain."
