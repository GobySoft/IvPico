#!/usr/bin/env bash
# Installs the Ubuntu packages needed to build IvPico.
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

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    SUDO=sudo
fi

apt_install() {
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
}

# xz-utils: the replay tests' recorded missions are compressed
packages=(cmake ninja-build g++ git ca-certificates protobuf-compiler libprotobuf-dev xz-utils)
# optional: YAML behavior files (.bhv.yml), and the test that validates them against the schema
packages+=(libyaml-cpp-dev python3-jsonschema python3-yaml)
$SUDO apt-get update
apt_install "${packages[@]}"

. /etc/os-release

packages=(libopenocean-messages-dev libivp-helm-dev)
if ${goby}; then
    # libgoby3-dev only recommends libais-dev and dccl5-compiler, but goby's CMake package
    # links libais.so, and goby_add_app() runs protoc-gen-dccl. goby3-gui has goby_liaison,
    # for the Goby launches' map
    packages+=(libgoby3-dev goby3-apps goby3-gui libais-dev dccl5-compiler libgoby3-moos-dev)
fi

# Adds an apt.oceansoft.org channel to openocean_sources, checking its key's fingerprint
openocean_sources=""
add_openocean() {
    local channel=$1 fingerprint=$2 key
    local keyring=/usr/share/keyrings/openocean-${channel}.asc
    key=$(curl -fsSL "https://raw.githubusercontent.com/openocean-software/openocean-build/main/keys/${channel}.asc")
    if ! gpg --show-keys --with-colons <<< "${key}" | grep -q "^fpr:*${fingerprint}:"; then
        echo "The Open Ocean Software ${channel} key's fingerprint isn't ${fingerprint}" >&2
        exit 1
    fi
    $SUDO tee "${keyring}" <<< "${key}" > /dev/null
    openocean_sources+="Types: deb
URIs: https://apt.oceansoft.org/${channel}/${ID}-${VERSION_CODENAME}
Suites: ${VERSION_CODENAME}
Components: main
Signed-By: ${keyring}

"
}

# If apt can't install them yet, add Open Ocean Software's continuous builds (for
# openocean-messages, MOOS-IvP's helm libraries built without MOOS, and Goby3 with
# 'goby test') and its releases, which have what those depend on (DCCL, MOOS, ...)
if ! apt-get -s install "${packages[@]}" > /dev/null 2>&1; then
    apt_install curl gpg
    add_openocean continuous DAF08DD7788E6C84C6F8E8D3E59E1A81FECDB228
    add_openocean release EAD278F6A25644C2C559DBB6D28A2EAB14D2A9E5
    $SUDO tee /etc/apt/sources.list.d/openocean.sources <<< "${openocean_sources}" > /dev/null
    $SUDO apt-get update
fi
apt_install "${packages[@]}"
