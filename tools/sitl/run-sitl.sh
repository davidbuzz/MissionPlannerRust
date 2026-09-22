#!/usr/bin/env bash
# Starts ArduPilot SITL for integration testing.
#
# usage: tools/sitl/run-sitl.sh [copter|plane] [extra args...]
#
# Override the binary with SITL_BINARY to use your own ArduPilot build.
set -euo pipefail

VEHICLE="${1:-copter}"
shift || true

HERE="$(cd "$(dirname "$0")" && pwd)"
case "$VEHICLE" in
    copter) DEFAULT_BIN="$HERE/arducopter"; MODEL="quad";  PARAMS="$HERE/params/copter.parm" ;;
    plane)  DEFAULT_BIN="$HERE/arduplane";  MODEL="plane"; PARAMS="$HERE/params/plane.parm" ;;
    *) echo "unknown vehicle: $VEHICLE (expected copter or plane)" >&2; exit 2 ;;
esac

BIN="${SITL_BINARY:-$DEFAULT_BIN}"
[ -x "$BIN" ] || { echo "SITL binary not found or not executable: $BIN" >&2; exit 1; }

# SITL writes eeprom.bin and logs into the working directory.
WORK="${SITL_WORKDIR:-$(mktemp -d -t mp-sitl-XXXXXX)}"
mkdir -p "$WORK"
cd "$WORK"

# SITL serves exactly one TCP client on SERIAL0 (tcp:5760). SITL_EXTRA_OUT adds a second link on
# SERIAL1 so a GUI can watch while a script flies the vehicle.
#
# Note this is the *binary's* flag, not sim_vehicle.py's: `--out` belongs to MAVProxy and makes
# the binary print its usage and exit. Device strings look like `udpclient:127.0.0.1:14550`.
EXTRA=()
[ -n "${SITL_EXTRA_OUT:-}" ] && EXTRA=(--serial1 "$SITL_EXTRA_OUT")

echo "SITL $VEHICLE in $WORK, listening on tcp:127.0.0.1:5760 (speedup ${SITL_SPEEDUP:-1})"
exec "$BIN" --model "$MODEL" --speedup "${SITL_SPEEDUP:-1}" --defaults "$PARAMS" "${EXTRA[@]}" "$@"
