#!/usr/bin/env bash
# Records MOOS-IvP missions under pHelmIvP for the replay tests: builds all of
# MOOS-IvP from external/moos-ivp, runs each mission in missions.txt headless,
# and keeps each vehicle's helm inputs and outputs in fixtures/<mission>.
#
# Usage: record.sh [mission...]   (default: every mission in missions.txt)
#
# WORK_DIR (default build/missions) holds the MOOS-IvP build and the runs.
# MOOS_BUILD_ARGS are passed to build-moos.sh, e.g. --with-proj=off.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "${here}/../../.." && pwd)
work=$(realpath -m "${WORK_DIR:-${root}/build/missions}")
moos_ivp=${work}/moos-ivp

# A copy, so that building leaves the submodule clean
if [ ! -x "${moos_ivp}/ivp/bin/pHelmIvP" ]; then
    rm -rf "${moos_ivp}"
    mkdir -p "${moos_ivp}"
    git -C "${root}/external/moos-ivp" archive HEAD | tar -x -C "${moos_ivp}"
    (cd "${moos_ivp}" && ./build-moos.sh --minrobot ${MOOS_BUILD_ARGS:-} && ./build-ivp.sh --nogui)
fi
export PATH="${moos_ivp}/ivp/bin:${moos_ivp}/build/MOOS/MOOSCore/bin:${moos_ivp}/build/MOOS/MOOSEssentials/bin:${PATH}"

# Keeps the log header, pHelmIvP's posts, and every post to a variable pHelmIvP
# registered, without the duplicate lines pLogger sometimes writes
trim() {
    awk 'NR == FNR { if ($2 == "IVPHELM_REGISTER") registered[$4] = 1; next }
         /^%%/ { print; next }
         ($3 ~ /^pHelmIvP/ || ($2 in registered)) && !seen[$0]++' "$1" "$1"
}

# Adds a uTimerScript that makes the pokes from inside the community: its posts
# are timestamped and delivered like the other apps', where uPokeDB's reach
# pHelmIvP seconds after their timestamps under time warp.
add_pokes() {
    local moos_file=$1
    shift
    awk 'tolower($0) ~ /processconfig *= *antler/ { antler = 1 }
         { print }
         antler && /^[ \t]*{/ { print "  Run = uTimerScript @ NewConsole = false ~ uTimerScript_record"; antler = 0 }' \
        "${moos_file}" > "${moos_file}.poked"
    {
        echo "ProcessConfig = uTimerScript_record"
        echo "{"
        local poke
        for poke in "$@"; do
            echo "  event = var=${poke%%=*}, val=${poke#*=}, time=10"
        done
        echo "}"
    } >> "${moos_file}.poked"
    mv "${moos_file}.poked" "${moos_file}"
}

# pHelmIvP's posts and the variables it registered, from a first run's logs
helm_variables() {
    awk '$1 !~ /^-?[0-9.]+$/ { next }
         $3 ~ /^pHelmIvP/ { print $2 }
         $2 == "IVPHELM_REGISTER" { print $4 }' "$@" | grep -E '^[A-Za-z_][A-Za-z0-9_]*$' | sort -u
}

# Logs each variable from its first post on: pLogger's wildcard logging only
# finds a variable some time after its first post
add_logging() {
    local moos_file=$1 variables=$2
    awk -v variables="${variables}" '
        tolower($0) ~ /processconfig *= *plogger/ { logger = 1 }
        { print }
        logger && /^[ \t]*{/ {
            n = split(variables, vars, " ")
            for (i = 1; i <= n; i++) print "  Log = " vars[i] " @ 0 NOSYNC"
            logger = 0
        }' "${moos_file}" > "${moos_file}.logged"
    mv "${moos_file}.logged" "${moos_file}"
}

# run <directory> <moos file> <warp> <seconds> [log variables]
run() {
    local run=$1 moos_file=$2 warp=$3 seconds=$4 variables=${5:-}
    rm -rf "${run}"
    mkdir -p "$(dirname "${run}")"
    cp -r "${moos_ivp}/ivp/missions/${mission}" "${run}"
    (
        cd "${run}"
        add_pokes "${moos_file}" "${pokes[@]}"
        [ -n "${variables}" ] && add_logging "${moos_file}" "${variables}"
        pAntler "${moos_file}" --MOOSTimeWarp="${warp}" > antler.log 2>&1 &
        antler=$!
        sleep $((seconds / warp))
        kill -INT "${antler}"
        wait "${antler}" || true
    )
}

record() {
    local moos_file=$2 warp=$3 seconds=$4
    mission=$1
    shift 4
    pokes=("$@")
    echo "Recording ${mission} (${seconds} s at warp ${warp}, twice)"

    run "${work}/runs/${mission}.discover" "${moos_file}" "${warp}" "${seconds}"
    local variables
    variables=$(helm_variables "${work}/runs/${mission}.discover"/MOOSLog_*/*.[ax]log | tr '\n' ' ')
    run "${work}/runs/${mission}" "${moos_file}" "${warp}" "${seconds}" "${variables}"

    local fixture=${here}/fixtures/${mission}
    rm -rf "${fixture}"
    mkdir -p "${fixture}"
    local log
    for log in "${work}/runs/${mission}"/MOOSLog_*/; do
        local name
        name=$(basename "${log}")
        trim "${log}/${name}.alog" | xz -9 > "${fixture}/${mission}.alog.xz"
        cp "${log}/${name}._moos" "${fixture}/${mission}.moos"
        for bhv in "${log}"/*._bhv; do
            cp "${bhv}" "${fixture}/$(basename "${bhv}" ._bhv).bhv"
        done
    done
}

missions=("$@")
while read -r mission moos_file warp seconds pokes; do
    [[ -z "${mission}" || "${mission}" == \#* ]] && continue
    if [ ${#missions[@]} -eq 0 ] || [[ " ${missions[*]} " == *" ${mission} "* ]]; then
        # shellcheck disable=SC2086
        record "${mission}" "${moos_file}" "${warp}" "${seconds}" ${pokes} < /dev/null
    fi
done < "${here}/missions.txt"
