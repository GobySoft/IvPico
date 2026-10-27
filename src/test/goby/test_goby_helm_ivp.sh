#!/usr/bin/env bash
# Runs goby_helm_ivp with gobyd and checks its frontseat bridge end to end:
#
#  1. a constant-heading behavior (course 90) with the helm starting in DRIVE;
#  2. one frontseat NodeStatus, published with `goby zeromq publish`, gives the
#     helm its navigation, converted to x/y with the app's datum;
#  3. the helm should then publish a DesiredCourse with heading 90 and a
#     HelmStateReport of HELM_DRIVE, and its own Status in DRIVE.
#
# Usage: test_goby_helm_ivp.sh <goby_helm_ivp> <libivpico_interface.so>
set -euo pipefail

helm=$1
export GOBY_TOOL_LOAD_SHARED_LIBRARY=$2
work=$(mktemp -d)
export GOBY_INTERPROCESS="platform: \"ivpico_test_$$\""
pids=()
cleanup() {
    kill "${pids[@]}" 2> /dev/null || true
    wait 2> /dev/null || true
    rm -rf "${work}"
}
trap cleanup EXIT

cat > "${work}/test.bhv" << 'BHV'
Behavior = BHV_ConstantHeading
{
  name     = east
  heading  = 90
  duration = no-time-limit
}
BHV
cat > "${work}/helm_ivp.pb.cfg" << CFG
app { geodesy { lat_origin: 43.8 lon_origin: -70.3 } }
helm {
  behavior_file: "${work}/test.bhv"
  domain { name: "course" low: 0 high: 359 points: 360 }
  domain { name: "speed" low: 0 high: 4 points: 21 optional: true }
  start_in_drive: true
}
frontseat { enable: true }
CFG

gobyd > "${work}/gobyd.log" 2>&1 &
pids+=($!)
sleep 1
"${helm}" "${work}/helm_ivp.pb.cfg" > "${work}/helm.log" 2>&1 &
pids+=($!)
sleep 2

goby zeromq publish goby::middleware::frontseat::node_status \
    PROTOBUF/goby.middleware.frontseat.protobuf.NodeStatus \
    "time: $(date +%s) name: \"test\" global_fix { lat: 43.8 lon: -70.3 } pose { heading: 0 } speed { over_ground: 0 }"

subscribe() {
    timeout "$1" goby zeromq subscribe "$2" > "${work}/$3" 2>&1 || true
}
subscribe 4 goby::middleware::frontseat::desired_course desired.txt
subscribe 2 goby::middleware::frontseat::helm_state helm_state.txt
subscribe 2 ivpico::helm::status status.txt

failed=0
check() {
    if ! grep -q "$2" "${work}/$1"; then
        echo "failed: no \"$2\" in $1:" >&2
        tail -5 "${work}/$1" >&2
        failed=1
    fi
}
check desired.txt "heading: 90"
check helm_state.txt "HELM_DRIVE"
check status.txt "state: DRIVE"
if [ ${failed} -ne 0 ]; then
    echo "goby_helm_ivp log:" >&2
    tail -20 "${work}/helm.log" >&2
fi
exit ${failed}
