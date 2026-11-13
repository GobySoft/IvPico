#!/usr/bin/env bash
# Deploys the vehicle: DEPLOY=true for the behaviors, and lifts the manual override.
set -euo pipefail

export GOBY_INTERPROCESS='platform: "alpha"'
# IvPico's build (build.sh), or else an installed libivpico_interface.so. Given with -l, as
# GOBY_TOOL_LOAD_SHARED_LIBRARY may be set for another project
lib=$(dirname "$0")/../../../build/usr/lib/libivpico_interface.so
[ -f "${lib}" ] || lib=libivpico_interface.so

goby zeromq publish -l "${lib}" ivpico::helm::variable PROTOBUF/ivp.interface.Variable 'key: "DEPLOY" sval: "true"'
goby zeromq publish -l "${lib}" ivpico::helm::command PROTOBUF/ivp.interface.Command 'manual_override: false'
