#!/usr/bin/env bash
# Sets the alpha mission's datum (also the simulator's start position and where Liaison's
# map opens) and time warp in its configuration.
#
# Usage: set_datum_warp.sh [--datum <lat> <lon>] [--warp <factor>]
#   With no options, prints the current settings.
set -euo pipefail

cd "$(dirname "$0")"

usage() { sed -n '5,6p' "$0" | sed 's/^# //'; }
fail() {
    echo "$1" >&2
    usage >&2
    exit 1
}
is_number() { [[ $1 =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; }

lat="" lon="" warp=""
while [ $# -gt 0 ]; do
    case "$1" in
        --datum)
            [ $# -ge 3 ] && is_number "$2" && is_number "$3" || fail "--datum needs a latitude and longitude"
            lat=$2 lon=$3
            shift 3
            ;;
        --warp)
            [ $# -ge 2 ] && is_number "$2" || fail "--warp needs a number"
            warp=$2
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *) fail "unknown option: $1" ;;
    esac
done

goby_cfgs=(config/*.pb.cfg)

if [ -n "${lat}" ]; then
    sed -i -E "s/(lat_origin: ).*/\1${lat}/; s/(lon_origin: ).*/\1${lon}/" "${goby_cfgs[@]}"
    sed -i -E "s/^(\s*lat: )[-0-9.]+/\1${lat}/; s/^(\s*lon: )[-0-9.]+/\1${lon}/" config/frontseat.pb.cfg
    sed -i -E "s/(initial_lat: ).*/\1${lat}/; s/(initial_lon: ).*/\1${lon}/" config/liaison.pb.cfg
fi

if [ -n "${warp}" ]; then
    sed -i -E "s/(warp_factor: ).*/\1${warp}/" "${goby_cfgs[@]}"
fi

grep -HE "lat_origin|lon_origin|initial_lat|initial_lon|warp_factor|^\s*(lat|lon): " "${goby_cfgs[@]}" | sed -E 's/\s+/ /g'
