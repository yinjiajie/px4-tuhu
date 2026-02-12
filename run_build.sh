#!/bin/bash
set -e
cd "$(dirname "$0")"
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
echo "=== Installing deps ==="
bash Tools/setup/ubuntu.sh --no-sim-tools
echo "=== Syncing and initializing submodules ==="
git submodule sync --recursive
git submodule update --init --recursive
echo "=== Building px4_fmu-v5_default ==="
make px4_fmu-v5_default
echo "=== Done ==="
