#!/usr/bin/env bash
# Deploys the vehicle: DEPLOY=true for the behaviors, and lifts the manual override.
set -euo pipefail

export GOBY_INTERPROCESS='platform: "frontseat_sim_example"'
export GOBY_TOOL_LOAD_SHARED_LIBRARY=${GOBY_TOOL_LOAD_SHARED_LIBRARY:-libivpico_interface.so}

goby zeromq publish ivpico::helm::variable PROTOBUF/ivp.interface.Variable 'key: "DEPLOY" sval: "true"'
goby zeromq publish ivpico::helm::command PROTOBUF/ivp.interface.Command 'manual_override: false'
