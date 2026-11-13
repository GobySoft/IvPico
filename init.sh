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
$SUDO apt-get update
apt_install "${packages[@]}"

. /etc/os-release

# openocean-messages and MOOS-IvP's helm libraries (built without MOOS), from Open Ocean
# Software's continuous builds (apt.oceansoft.org) if apt doesn't have them yet
oo_policy=$(apt-cache policy libivp-helm-dev)
if ! grep -q "Candidate: [^(]" <<< "${oo_policy}"; then
    apt_install curl gpg
    keyring=/usr/share/keyrings/openocean-continuous.asc
    fingerprint=DAF08DD7788E6C84C6F8E8D3E59E1A81FECDB228
    key=$(curl -fsSL https://raw.githubusercontent.com/openocean-software/openocean-build/main/keys/continuous.asc)
    if ! gpg --show-keys --with-colons <<< "${key}" | grep -q "^fpr:*${fingerprint}:"; then
        echo "The Open Ocean Software continuous key's fingerprint isn't ${fingerprint}" >&2
        exit 1
    fi
    $SUDO tee "${keyring}" <<< "${key}" > /dev/null
    printf "Types: deb\nURIs: https://apt.oceansoft.org/continuous/%s-%s\nSuites: %s\nComponents: main\nSigned-By: %s\n" \
        "${ID}" "${VERSION_CODENAME}" "${VERSION_CODENAME}" "${keyring}" |
        $SUDO tee /etc/apt/sources.list.d/openocean.sources > /dev/null
    $SUDO apt-get update
fi
apt_install libopenocean-messages-dev libivp-helm-dev

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
        echo "deb [signed-by=${keyring}] http://packages.gobysoft.org/${ID}/continuous/ ${VERSION_CODENAME}/" |
            $SUDO tee /etc/apt/sources.list.d/gobysoft_continuous.list > /dev/null
        $SUDO apt-get update
    fi
    # libgoby3-dev only recommends these, but goby's CMake package links libais.so,
    # and goby_add_app() runs protoc-gen-dccl
    apt_install libgoby3-dev goby3-apps libais-dev dccl5-compiler
    # pMarineViewer for the Goby launches (viewer.launch), through goby_moos_gateway;
    # MOOS-IvP's apps need libmoos-ivp but don't depend on it
    apt_install libgoby3-moos-dev goby3-moos moosdb10 moos-ivp-apps moos-ivp-gui libmoos-ivp
fi
