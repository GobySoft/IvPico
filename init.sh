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

apt_install() {
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
}

# moos-ivp's helm libraries (IVP_BUILD_HELM_LIBS_ONLY) need only CMake and a compiler
# xz-utils: the replay tests' recorded missions are compressed
packages=(cmake ninja-build g++ git ca-certificates protobuf-compiler libprotobuf-dev curl gpg xz-utils)
$SUDO apt-get update
apt_install "${packages[@]}"

# Goby3, for src/bin/goby, from packages.gobysoft.org
keyring=/usr/share/keyrings/gobysoft-archive-keyring.gpg
if [ ! -f "${keyring}" ]; then
    curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x19478082E2F8D3FE" |
        gpg --dearmor | $SUDO tee "${keyring}" > /dev/null
fi
. /etc/os-release
echo "deb [signed-by=${keyring}] http://packages.gobysoft.org/${ID}/release/ ${VERSION_CODENAME}/" |
    $SUDO tee /etc/apt/sources.list.d/gobysoft_release.list > /dev/null
$SUDO apt-get update
# goby's CMake package links libais.so, which libgoby3-dev only recommends
apt_install libgoby3-dev goby3-apps libais-dev

external/openocean-messages/init.sh --cxx
