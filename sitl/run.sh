#!/bin/sh
# Start foamie-sitl and ArduPlane SITL, fly fly.py, stop both.  ARDUPILOT=path to an ardupilot tree built with
# `./waf configure --board sitl && ./waf plane`.   Usage: ./run.sh [fly.py args]   (env WIND=m/s, SPEEDUP=n)
set -e
here=$(cd "$(dirname "$0")" && pwd)
AP=${ARDUPILOT:-$HOME/ardupilot}
work=${WORK:-$here/run}
mkdir -p "$work"; cd "$work"; rm -f eeprom.bin
cargo build --release --manifest-path "$here/Cargo.toml" -q
"$here/target/release/foamie-sitl" --wind "${WIND:-0}" --log "$work/truth.csv" > bridge.log 2>&1 &
bridge=$!
"$AP/build/sitl/bin/arduplane" --model JSON:127.0.0.1 --speedup "${SPEEDUP:-4}" -I0 \
  --home 35.8989,14.4897,10,0 --defaults "$here/foamie.parm" > ardupilot.log 2>&1 &
apid=$!
trap 'kill $apid $bridge 2>/dev/null' EXIT
sleep 3
python3 "$here/fly.py" "$@"
