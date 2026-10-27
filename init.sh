#!/usr/bin/env bash
# Checks out the submodules and installs the Ubuntu packages needed to build them and IvPico.
#
# Usage: init.sh
set -euo pipefail

cd "$(dirname "$0")"

case "${1:-}" in
    "") ;;
    -h | --help)
        sed -n '4p' "$0" | sed 's/^# //'
        exit 0
        ;;
    *)
        sed -n '4p' "$0" | sed 's/^# //' >&2
        exit 1
        ;;
esac

git submodule update --init --recommend-shallow

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    SUDO=sudo
fi

# moos-ivp's libraries and non-GUI apps (build-ivp.sh --nogui), so not FLTK and the X libraries
packages=(cmake make g++ git ca-certificates protobuf-compiler libprotobuf-dev)
$SUDO apt-get update
$SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${packages[@]}"

external/openocean-messages/init.sh --cxx
