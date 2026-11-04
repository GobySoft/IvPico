#!/usr/bin/env bash
# Sets the alpha mission's datum (also the simulator's start position) and time warp in its
# Goby configuration and alpha.moos, choosing the MOOS-IvP image that covers the datum.
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
image_dir=${IVP_IMAGE_DIRS:-../../../external/moos-ivp/ivp/data}

# the smallest image whose .info bounds contain the datum
image_for() {
    for info in "${image_dir}"/*.info; do
        awk -v lat="$1" -v lon="$2" -v tif="$(basename "${info%.info}").tif" -F= '
            { gsub(/[ \t]/, ""); b[$1] = $2 }
            END {
                if (!("lat_north" in b) || !("lon_east" in b)) exit
                if (lat <= b["lat_north"] && lat >= b["lat_south"] && lon <= b["lon_east"] && lon >= b["lon_west"])
                    print (b["lat_north"] - b["lat_south"]) * (b["lon_east"] - b["lon_west"]), tif
            }' "${info}"
    done | sort -g | head -1 | cut -d' ' -f2
}

if [ -n "${lat}" ]; then
    sed -i -E "s/(lat_origin: ).*/\1${lat}/; s/(lon_origin: ).*/\1${lon}/" "${goby_cfgs[@]}"
    sed -i -E "s/^(\s*lat: )[-0-9.]+/\1${lat}/; s/^(\s*lon: )[-0-9.]+/\1${lon}/" config/frontseat.pb.cfg
    sed -i -E "s/^(LatOrigin\s*=\s*).*/\1${lat}/; s/^(LongOrigin\s*=\s*).*/\1${lon}/" alpha.moos
    image=$(image_for "${lat}" "${lon}")
    if [ -n "${image}" ]; then
        sed -i -E "s/^(\s*tiff_file\s*=\s*).*/\1${image}/" alpha.moos
    else
        echo "No image in ${image_dir} covers ${lat}, ${lon}: set tiff_file in alpha.moos" >&2
    fi
fi

if [ -n "${warp}" ]; then
    sed -i -E "s/(warp_factor: ).*/\1${warp}/" "${goby_cfgs[@]}"
    sed -i -E "s/^(MOOSTimeWarp\s*=\s*).*/\1${warp}/" alpha.moos
fi

grep -HE "lat_origin|lon_origin|warp_factor|^\s*(lat|lon): " "${goby_cfgs[@]}" | sed -E 's/\s+/ /g'
grep -HE "^(LatOrigin|LongOrigin|MOOSTimeWarp)|^\s*tiff_file" alpha.moos | sed -E 's/\s+/ /g'
