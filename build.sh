#!/usr/bin/env bash
# Builds the dependencies from their submodules (openocean-messages, and MOOS-IvP's helm libraries),
# then IvPico against them, all in BUILD_DIR (default build). Arguments are passed on to IvPico's
# cmake.
set -euo pipefail

cd "$(dirname "$0")"
for submodule in openocean-messages moos-ivp; do
    if [ -z "$(ls -A "external/${submodule}")" ]; then
        echo "external/${submodule} is empty: run ./init.sh first" >&2
        exit 1
    fi
done
BUILD_DIR=$(realpath -m "${BUILD_DIR:-build}")
OPENOCEAN_BUILD_DIR=${BUILD_DIR}/external/openocean-messages
IVP_BUILD_DIR=${BUILD_DIR}/external/moos-ivp
# find_package(IVP) only works from an installation
IVP_INSTALL_DIR=${BUILD_DIR}/external/install

BUILD_DIR=${OPENOCEAN_BUILD_DIR} external/openocean-messages/build.sh -DBUILD_TESTING=OFF

cmake -S external/moos-ivp/ivp/src -B "${IVP_BUILD_DIR}" -G Ninja \
    -DIVP_BUILD_HELM_LIBS_ONLY=ON -DBUILD_SHARED_LIBS=ON \
    -DIVP_LIB_DIRECTORY="${IVP_BUILD_DIR}/lib" \
    -DIVP_INC_DIRECTORY="${IVP_BUILD_DIR}/include" \
    -DIVP_BIN_DIRECTORY="${IVP_BUILD_DIR}/bin"
ninja -C "${IVP_BUILD_DIR}"
cmake --install "${IVP_BUILD_DIR}" --prefix "${IVP_INSTALL_DIR}" > /dev/null

cmake -S . -B "${BUILD_DIR}" -G Ninja \
    -Dopenocean_messages_DIR="${OPENOCEAN_BUILD_DIR}" \
    -DIVP_DIR="${IVP_INSTALL_DIR}/lib/cmake/IVP" \
    "$@"
ninja -C "${BUILD_DIR}"
