#!/usr/bin/env bash
# Starts SITL and proves it is streaming before handing it over.
#
# usage: tools/sitl/start-sitl.sh [copter|plane]      prints "sitl ready pid N" or fails
#
# On 2026-09-25 one start in about a dozen of the bundled binary stopped after "Smoothing reset
# at 0.001": its first client was accepted, SERIAL1 was never bound, and not one MAVLink frame
# was ever sent, so a GUI suite run against it failed every script with "params.held is '0'".
# This wrapper connects a throwaway client to tcp:127.0.0.1:5760, requires a MAVLink frame within
# ten seconds, and kills and starts the simulator again when none comes, up to three times.
# SITL keeps running after the client leaves and accepts the next one at once, so the probe costs
# nothing (measured: re-accepted after 0.0 s, three times).
#
# Any earlier SITL of ours is stopped first - matched by its binary's path, never by a pattern
# that a calling shell's command line would also match.

set -u
VEHICLE="${1:-copter}"
HERE="$(cd "$(dirname "$0")" && pwd)"
LOG="${SITL_LOG:-$(mktemp -t mp-sitl-XXXXXX.log)}"

stop_ours() {
    for pid in $(pgrep -f "$HERE/arducopte[r]" ; pgrep -f "$HERE/arduplan[e]"); do
        kill "$pid" 2>/dev/null
    done
    sleep 1
}

streams() {
    python3 - <<'EOF'
import socket, sys, time
try:
    s = socket.create_connection(("127.0.0.1", 5760), timeout=5)
except OSError:
    sys.exit(2)
s.settimeout(0.5)
deadline = time.time() + 10
while time.time() < deadline:
    try:
        data = s.recv(65536)
    except socket.timeout:
        continue
    if not data:
        sys.exit(3)
    if b"\xfd" in data or b"\xfe" in data:
        s.close()
        sys.exit(0)
s.close()
sys.exit(1)
EOF
}

for attempt in 1 2 3; do
    stop_ours
    SITL_WORKDIR="${SITL_WORKDIR:-$(mktemp -d -t mp-sitl-XXXXXX)}" nohup "$HERE/run-sitl.sh" "$VEHICLE" > "$LOG" 2>&1 &
    sleep 2
    if streams; then
        PID=$(pgrep -f "$HERE/arducopte[r]" | head -1)
        [ -z "$PID" ] && PID=$(pgrep -f "$HERE/arduplan[e]" | head -1)
        echo "sitl ready pid $PID, log $LOG"
        exit 0
    fi
    echo "sitl start $attempt did not stream; last lines:" >&2
    tail -3 "$LOG" >&2
done
stop_ours
echo "sitl would not stream after three starts" >&2
exit 1
