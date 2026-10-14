#!/usr/bin/env bash
# The whole crawl: four workers on virtual displays - two disconnected, two connected to a copter
# SITL each - then the verification pass and the report. Everything is stopped by PID at the end.
#
#   tools/gui-crawl/run.sh [--binary PATH]      results, report.md, recipe.txt, dead/ in out/
#
# Uses displays :91 to :94 and SITL instances 7 and 8 (tcp:5830, tcp:5840), its own. Nothing touches
# the desktop's display, a serial device or another simulator.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/out"
BINARY=()
[ "${1:-}" = "--binary" ] && BINARY=(--binary "$2")
mkdir -p "$OUT"
rm -f "$OUT"/results-*.jsonl "$OUT"/verify-*.jsonl "$OUT"/opened-*.log "$OUT"/*.log.txt
rm -rf "$OUT/dead"

# The crawl's own simulators, on instances no one else uses (tcp 5830 and 5840): the connected
# workers write parameters, arm and fly, and must never reach a SITL that is not theirs - on
# 2026-10-14 an audit whose SITL could not bind 5760 drove the owner's own SITL instead. run-sitl.sh
# refuses a port already in use; here, before any worker starts, each port must be listened on by
# the simulator this script started.
P0=5830; P1=5840
"$ROOT/tools/sitl/run-sitl.sh" copter -I7 > "$OUT/sitl0.log.txt" 2>&1 & SITL0=$!
"$ROOT/tools/sitl/run-sitl.sh" copter -I8 > "$OUT/sitl1.log.txt" 2>&1 & SITL1=$!
trap 'kill $SITL0 $SITL1 2>/dev/null' EXIT
sleep 8
for pair in "$SITL0:$P0" "$SITL1:$P1"; do
    pid=${pair%%:*}; port=${pair##*:}
    holder=$(ss -ltnpH "sport = :$port" | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
    if ! kill -0 "$pid" 2>/dev/null || [ "$holder" != "$pid" ]; then
        echo "tcp:$port is not this crawl's SITL (started $pid, listener ${holder:-none}): stopping" >&2
        exit 3
    fi
done
echo "crawl from $(date +%T)"
OTHER=plan,params,logs,simulation,help,experimental,plugins
python3 "$HERE/crawl.py" disc-a --display :94 --screens fly "${BINARY[@]}" > "$OUT/run-disc-a.log.txt" 2>&1 & W1=$!
python3 "$HERE/crawl.py" disc-b --display :93 --screens "$OTHER,setup,config" "${BINARY[@]}" > "$OUT/run-disc-b.log.txt" 2>&1 & W2=$!
python3 "$HERE/crawl.py" conn-a --display :92 --screens "fly,$OTHER" "${BINARY[@]}" -- tcp:127.0.0.1:$P0 > "$OUT/run-conn-a.log.txt" 2>&1 & W3=$!
python3 "$HERE/crawl.py" conn-b --display :91 --screens setup,config "${BINARY[@]}" -- tcp:127.0.0.1:$P1 > "$OUT/run-conn-b.log.txt" 2>&1 & W4=$!
wait $W1 $W2 $W3 $W4
echo "verify from $(date +%T)"
python3 "$HERE/verify.py" disc-a --display :94 "${BINARY[@]}" > "$OUT/verify-disc-a.log.txt" 2>&1 & V1=$!
python3 "$HERE/verify.py" disc-b --display :93 "${BINARY[@]}" > "$OUT/verify-disc-b.log.txt" 2>&1 & V2=$!
python3 "$HERE/verify.py" conn-a --display :92 "${BINARY[@]}" -- tcp:127.0.0.1:$P0 > "$OUT/verify-conn-a.log.txt" 2>&1 & V3=$!
python3 "$HERE/verify.py" conn-b --display :91 "${BINARY[@]}" -- tcp:127.0.0.1:$P1 > "$OUT/verify-conn-b.log.txt" 2>&1 & V4=$!
wait $V1 $V2 $V3 $V4
echo "report from $(date +%T)"
python3 "$HERE/report.py" --out "$OUT"
