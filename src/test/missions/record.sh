#!/usr/bin/env bash
# Records MOOS-IvP missions under pHelmIvP for the replay tests: builds all of
# MOOS-IvP from GobySoft's ivp-without-moos branch (libivp-helm-dev's source),
# runs each mission in missions.txt headless, and keeps each vehicle's helm
# inputs and outputs in fixtures/<mission> (one vehicle) or
# fixtures/<mission>.<vehicle> (missions started by launch.sh).
#
# Usage: record.sh [mission...]   (default: every mission in missions.txt)
#
# WORK_DIR (default build/missions) holds the MOOS-IvP build and the runs.
# MOOS_IVP_BRANCH overrides the branch.
# MOOS_BUILD_ARGS are passed to build-moos.sh, e.g. --with-proj=off.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "${here}/../../.." && pwd)
work=$(realpath -m "${WORK_DIR:-${root}/build/missions}")
moos_ivp=${work}/moos-ivp

if [ ! -x "${moos_ivp}/ivp/bin/pHelmIvP" ]; then
    rm -rf "${moos_ivp}"
    git clone --depth 1 --branch "${MOOS_IVP_BRANCH:-ivp-without-moos}" \
        https://github.com/GobySoft/moos-ivp.git "${moos_ivp}"
    (cd "${moos_ivp}" && ./build-moos.sh --minrobot ${MOOS_BUILD_ARGS:-} && ./build-ivp.sh --nogui)
fi
export PATH="${moos_ivp}/ivp/bin:${moos_ivp}/bin:${moos_ivp}/build/MOOS/MOOSCore/bin:${moos_ivp}/build/MOOS/MOOSEssentials/bin:${PATH}"

# Keeps the log header, pHelmIvP's posts, and every post to a variable pHelmIvP
# registered, without the duplicate lines pLogger sometimes writes, its
# app logging, the objective functions (which replay doesn't compare), or the
# lines a multi-line value spills onto
trim() {
    awk 'NR == FNR { if ($2 == "IVPHELM_REGISTER") registered[$4] = 1; next }
         /^%%/ { print; next }
         $1 !~ /^-?[0-9.]+$/ || $2 ~ /^(APP_LOG|APPCAST|PHELMIVP_STATUS|BHV_IPF)$/ { next }
         ($3 ~ /^pHelmIvP/ || ($2 in registered)) && !seen[$0]++' "$1" "$1"
}

# Launches pLogger right after MOOSDB, so it logs pHelmIvP from the start,
# without omitting anything (e.g. BHV_STATUS), and adds a uTimerScript that pokes once pHelmIvP is up. uTimerScript's posts are
# timestamped and delivered like the other apps', where uPokeDB's reach
# pHelmIvP seconds after their timestamps under time warp.
prepare() {
    local moos_file=$1
    shift
    awk 'tolower($0) ~ /processconfig *= *antler/ { antler = 1 }
         antler && /^[ \t]*Run *= *pLogger[ \t@]/ { next }
         tolower($0) ~ /^[ \t]*wildcardomitpattern/ { next }
         { print }
         antler && /^[ \t]*{/ { print "  Run = uTimerScript @ NewConsole = false ~ uTimerScript_record" }
         antler && /^[ \t]*Run *= *MOOSDB[ \t@]/ { print "  Run = pLogger @ NewConsole = false" }
         antler && /^[ \t]*}/ { antler = 0 }' \
        "${moos_file}" > "${moos_file}.prepared"
    {
        echo "ProcessConfig = uTimerScript_record"
        echo "{"
        echo "  condition = (IVPHELM_STATE = PARK) or (IVPHELM_STATE = DRIVE)"
        local poke
        for poke in "$@"; do
            echo "  event = var=${poke%%=*}, val=${poke#*=}, time=5"
        done
        echo "}"
    } >> "${moos_file}.prepared"
    mv "${moos_file}.prepared" "${moos_file}"
}

# Drops a shoreside's GUIs, which a headless run can't start
strip_gui() {
    awk '!/^[ \t]*Run *= *(pMarineViewer|uMAC)[ \t@]/' "$1" > "$1.headless"
    mv "$1.headless" "$1"
}

# The log directories of a run, wherever its pLogger writes them
log_dirs() {
    find "$1" -name '*.alog' -exec dirname {} \; | sort -u
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

# run <directory> <moos file, or launch.sh> <warp> <seconds> [log variables]
# With launch.sh, the mission's own launch.sh writes a .moos file per community
# (targ_*.moos), and each one runs.
run() {
    local run=$1 moos_file=$2 warp=$3 seconds=$4 variables=${5:-}
    rm -rf "${run}"
    mkdir -p "$(dirname "${run}")"
    cp -r "${moos_ivp}/ivp/missions/${mission}" "${run}"
    (
        cd "${run}"
        local moos_files=("${moos_file}")
        if [ "${moos_file}" = launch.sh ]; then
            ./launch.sh --just_make "${warp}" > launch.log 2>&1
            moos_files=(targ_*.moos)
        fi
        local f
        for f in "${moos_files[@]}"; do
            if grep -qiE '^[[:space:]]*processconfig[[:space:]]*=[[:space:]]*phelmivp' "${f}"; then
                prepare "${f}" "${pokes[@]}"
                [ -n "${variables}" ] && add_logging "${f}" "${variables}"
            else
                strip_gui "${f}"
            fi
        done
        # job control puts each pAntler in its own process group, to stop whatever it leaves behind
        set -m
        local antlers=()
        for f in "${moos_files[@]}"; do
            pAntler "${f}" --MOOSTimeWarp="${warp}" > "antler_${f%.moos}.log" 2>&1 &
            antlers+=($!)
        done
        sleep $((seconds / warp))
        local antler
        for antler in "${antlers[@]}"; do kill -INT "${antler}" 2> /dev/null || true; done
        for _ in $(seq 20); do
            local running=0
            for antler in "${antlers[@]}"; do kill -0 "${antler}" 2> /dev/null && running=1; done
            [ "${running}" = 0 ] && break
            sleep 1
        done
        for antler in "${antlers[@]}"; do
            kill -KILL -- "-${antler}" 2> /dev/null || true
            wait "${antler}" || true
        done
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
    local logs=()
    mapfile -t logs < <(find "${work}/runs/${mission}.discover" -name '*.alog' -o -name '*.xlog')
    variables=$(helm_variables "${logs[@]}" | tr '\n' ' ')
    run "${work}/runs/${mission}" "${moos_file}" "${warp}" "${seconds}" "${variables}"

    rm -rf "${here}/fixtures/${mission}" "${here}/fixtures/${mission}".*
    local log
    for log in $(log_dirs "${work}/runs/${mission}"); do
        local name community fixture
        name=$(basename "${log}")
        awk '$2 == "IVPHELM_STATE" && $3 ~ /^pHelmIvP/ { found = 1; exit } END { exit !found }' \
            "${log}/${name}.alog" || continue
        community=$(awk -F= 'tolower($1) ~ /^[ \t]*community[ \t]*$/ { gsub(/[ \t]/, "", $2); print $2; exit }' \
            "${log}/${name}._moos")
        fixture=${here}/fixtures/${mission}
        [ "${moos_file}" = launch.sh ] && fixture+=".${community}"
        mkdir -p "${fixture}"
        local base
        base=$(basename "${fixture}")
        trim "${log}/${name}.alog" | xz -9 > "${fixture}/${base}.alog.xz"
        cp "${log}/${name}._moos" "${fixture}/${base}.moos"
        if ! xz -dc "${fixture}/${base}.alog.xz" | grep "IVPHELM_REGISTER" > /dev/null; then
            echo "warning: ${base}'s log misses pHelmIvP's startup" >&2
        fi
        for bhv in "${log}"/*._bhv; do
            [ -e "${bhv}" ] || continue
            cp "${bhv}" "${fixture}/$(basename "${bhv}" ._bhv).bhv"
        done
        # pHelmIvP only asks pLogger to copy its behavior files once it has started
        if ! ls "${fixture}"/*.bhv > /dev/null 2>&1; then
            if [ "${moos_file}" = launch.sh ]; then
                cp "${work}/runs/${mission}/targ_${community}.bhv" "${fixture}/"
            else
                cp "${work}/runs/${mission}"/*.bhv "${fixture}/"
            fi
        fi
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
