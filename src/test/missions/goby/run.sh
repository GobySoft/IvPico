#!/usr/bin/env bash
# Runs one MOOS-IvP mission closed loop under Goby: gobyd, goby's basic
# frontseat simulator and goby_frontseat_interface, goby_helm_ivp on the
# mission's recorded .bhv (../fixtures/<mission>), and ivpico_mission_check,
# which pokes the mission and checks <mission>/check.pb.cfg. Time warp 10.
#
# Usage: run.sh <mission> <goby_helm_ivp> <ivpico_mission_check> <simulator port>
set -euo pipefail
# own process group per background job, so cleanup also reaches their children
set -m

here=$(cd "$(dirname "$0")" && pwd)
mission=$1 helm=$2 check=$3 port=$4
work=$(mktemp -d)
platform="ivpico_${mission}_$$"
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -- "-${pid}" 2> /dev/null || true; done
    wait 2> /dev/null || true
    [[ -n "${IVPICO_KEEP_RUN:-}" ]] && echo "kept ${work}" >&2 || rm -rf "${work}"
}
trap cleanup EXIT

common="app {
  geodesy { lat_origin: 43.8253 lon_origin: -70.3304 }
  simulation { time { use_sim_time: true warp_factor: 10 } }
}
interprocess { platform: \"${platform}\" }"

echo "${common}" > "${work}/gobyd.pb.cfg"
cat > "${work}/frontseat.pb.cfg" << EOF2
${common}
frontseat_cfg {
  require_helm: true
  helm_running_timeout: 10
  frontseat_connected_timeout: 10
  status_period: 5
  exit_on_error: false
  [goby.middleware.frontseat.protobuf.basic_simulator_config] {
    tcp_address: "127.0.0.1"
    tcp_port: ${port}
    start {
      lat: 43.8253
      lon: -70.3304
      duration: 0
      control_freq: 10
      vehicle { accel: 1.0 hdg_rate: 50 z_rate: 2 }
    }
  }
}
EOF2
{
    echo "${common}"
    sed "s|@FIXTURE@|${here}/../fixtures/${mission}|" "${here}/${mission}/helm.pb.cfg"
    echo "frontseat { enable: true }"
} > "${work}/helm.pb.cfg"
{
    echo "${common}"
    cat "${here}/${mission}/check.pb.cfg"
} > "${work}/check.pb.cfg"

goby_basic_frontseat_simulator "${port}" > "${work}/simulator.log" 2>&1 &
pids+=($!)
gobyd "${work}/gobyd.pb.cfg" > "${work}/gobyd.log" 2>&1 &
pids+=($!)
sleep 1
goby_frontseat_interface_basic_simulator "${work}/frontseat.pb.cfg" > "${work}/frontseat.log" 2>&1 &
pids+=($!)
"${helm}" "${work}/helm.pb.cfg" > "${work}/helm.log" 2>&1 &
pids+=($!)

if ! "${check}" "${work}/check.pb.cfg" -v > "${work}/check.log" 2>&1; then
    grep -v "^\s*$" "${work}/check.log" | tail -15 >&2
    echo "goby_helm_ivp log:" >&2
    tail -15 "${work}/helm.log" >&2
    exit 1
fi
grep -E "met |PASSED" "${work}/check.log"
