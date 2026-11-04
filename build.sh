#!/usr/bin/env bash
# Builds IvPico in BUILD_DIR (default build) and installs it into BUILD_DIR/usr (e.g. add
# build/usr/bin to PATH). Arguments are passed on to IvPico's cmake.
set -euo pipefail

cd "$(dirname "$0")"
BUILD_DIR=$(realpath -m "${BUILD_DIR:-build}")

cmake -S . -B "${BUILD_DIR}" -G Ninja -DCMAKE_INSTALL_PREFIX="${BUILD_DIR}/usr" "$@"
ninja -C "${BUILD_DIR}"
cmake --install "${BUILD_DIR}" > /dev/null
