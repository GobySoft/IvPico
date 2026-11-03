#!/usr/bin/env bash
# Checks out the submodules and installs the Ubuntu packages needed to build them and IvPico.
#
# Usage: init.sh [--no-goby]
#   --no-goby  don't install Goby3 (build.sh then skips the Goby app)
set -euo pipefail

cd "$(dirname "$0")"

usage() { sed -n '4,5p' "$0" | sed 's/^# //'; }

goby=true
for arg in "$@"; do
    case "${arg}" in
        --no-goby) goby=false ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            exit 1
            ;;
    esac
done

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
packages=(cmake ninja-build g++ git ca-certificates protobuf-compiler libprotobuf-dev xz-utils)
$SUDO apt-get update
apt_install "${packages[@]}"

if ${goby}; then
    # Goby3, for src/bin/goby. If apt doesn't have it yet, add packages.gobysoft.org's
    # continuous builds, which have 'goby test' (newer than 3.6.1)
    goby_policy=$(apt-cache policy libgoby3-dev)
    if ! grep -q "Candidate: [^(]" <<< "${goby_policy}"; then
        apt_install curl gpg
        keyring=/usr/share/keyrings/gobysoft-archive-keyring.gpg
        if [ ! -f "${keyring}" ]; then
            curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x19478082E2F8D3FE" |
                gpg --dearmor | $SUDO tee "${keyring}" > /dev/null
        fi
        . /etc/os-release
        echo "deb [signed-by=${keyring}] http://packages.gobysoft.org/${ID}/continuous/ ${VERSION_CODENAME}/" |
            $SUDO tee /etc/apt/sources.list.d/gobysoft_continuous.list > /dev/null
        $SUDO apt-get update
    fi
    # libgoby3-dev only recommends these, but goby's CMake package links libais.so,
    # and goby_add_app() runs protoc-gen-dccl
    apt_install libgoby3-dev goby3-apps libais-dev dccl5-compiler
fi

external/openocean-messages/init.sh --cxx
