#!/usr/bin/env bash
# Downloads the MOOS-IvP background images that set_datum_warp.sh can choose between (those with
# lat/lon bounds) into images/, for pMarineViewer (viewer.launch).
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p images
url=https://raw.githubusercontent.com/GobySoft/moos-ivp/ivp-without-moos/ivp/data
for image in forrest19 MIT_SP; do
    for ext in tif info; do
        [ -s "images/${image}.${ext}" ] || curl -fsSL -o "images/${image}.${ext}" "${url}/${image}.${ext}"
    done
done
ls images
