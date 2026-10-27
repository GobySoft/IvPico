#!/usr/bin/env bash
# Builds openocean-messages from its submodule, then IvPico against it, all in BUILD_DIR (default
# build). Arguments are passed on to IvPico's cmake.
set -euo pipefail

cd "$(dirname "$0")"
if [ ! -e external/openocean-messages/CMakeLists.txt ]; then
    echo "external/openocean-messages is empty: run ./init.sh first" >&2
    exit 1
fi
BUILD_DIR=$(realpath -m "${BUILD_DIR:-build}")
OPENOCEAN_BUILD_DIR=${BUILD_DIR}/external/openocean-messages

BUILD_DIR=${OPENOCEAN_BUILD_DIR} external/openocean-messages/build.sh -DBUILD_TESTING=OFF

cmake -S . -B "${BUILD_DIR}" -G Ninja -Dopenocean_messages_DIR="${OPENOCEAN_BUILD_DIR}" "$@"
ninja -C "${BUILD_DIR}"
