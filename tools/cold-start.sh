#!/usr/bin/env bash
# Measures the planner's start: how long until its first frames are on screen.
#
# `MP_SMOKE=1 planner` paints, and once three frames have gone through the swap chain it prints
# "smoke: 3 frames painted in <time>" and exits (crates/mp-gui/src/smoke.rs). This runs that N
# times and reports the smoke's figure and the process's whole wall time - start of the process to
# its exit - as min, median and max. PLAN.md section 8 asks for a 20-run median; D20 for a cold
# start under 500 ms.
#
# On the headless display (Xvfb, Mesa's lavapipe), as tools/gui-headless.sh runs the GUI scripts:
# it touches nothing on the desktop, and the software rasteriser is the slower of the two, so a
# figure met here is met on a GPU. A data directory of its own, so the first start's import and
# the recording do not count, and offline, so no fetch is in the way.
#
#   tools/cold-start.sh [runs] [binary]     default 20 runs of dist/planner (else target/release)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNS="${1:-20}"
BIN="${2:-}"
if [ -z "$BIN" ]; then
    for candidate in "$ROOT/dist/planner" "$ROOT/target/release/planner"; do
        [ -x "$candidate" ] && BIN="$candidate" && break
    done
fi
[ -n "$BIN" ] && [ -x "$BIN" ] || { echo "no release planner: tools/package.sh builds one" >&2; exit 2; }

DISPLAY_NO="${HEADLESS_DISPLAY:-:99}"
SOCKET="/tmp/.X11-unix/X${DISPLAY_NO#:}"
if [ ! -S "$SOCKET" ]; then
    nohup setsid Xvfb "$DISPLAY_NO" -screen 0 2560x1440x24 +extension GLX +extension RANDR +render -noreset \
        > "${TMPDIR:-/tmp}/xvfb-${DISPLAY_NO#:}.log" 2>&1 < /dev/null 3>&- 4>&- 5>&- 6>&- 7>&- 8>&- 9>&- &
    for _ in $(seq 1 50); do [ -S "$SOCKET" ] && break; sleep 0.1; done
    [ -S "$SOCKET" ] || { echo "Xvfb $DISPLAY_NO did not start" >&2; exit 2; }
fi
ICD=/usr/share/vulkan/icd.d/lvp_icd.json
[ -f "$ICD" ] || { echo "no lavapipe ($ICD): install mesa-vulkan-drivers" >&2; exit 2; }
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
export DISPLAY="$DISPLAY_NO" VK_ICD_FILENAMES="$ICD" XDG_DATA_HOME="$WORK" MP_SMOKE=1 MP_OFFLINE=1 MP_NO_RECORD=1

frames=()
walls=()
for i in $(seq 1 "$RUNS"); do
    start=$(date +%s%N)
    out=$("$BIN" 2>/dev/null || true)
    end=$(date +%s%N)
    wall=$(( (end - start) / 1000000 ))
    # "smoke: 3 frames painted in 412.3ms" or "... in 1.2s"
    painted=$(echo "$out" | sed -n 's/^smoke: [0-9]* frames painted in \([0-9.]*\)\(ms\|s\).*/\1 \2/p')
    if [ -z "$painted" ]; then echo "run $i: the window did not paint: $out" >&2; exit 1; fi
    ms=$(echo "$painted" | awk '{ if ($2 == "s") printf "%d", $1 * 1000; else printf "%d", $1 }')
    frames+=("$ms"); walls+=("$wall")
    printf 'run %2d: first frames %5d ms, process %5d ms\n' "$i" "$ms" "$wall"
done
stats() { printf '%s\n' "$@" | sort -n | awk '{ a[NR]=$1 } END { printf "min %d ms, median %d ms, max %d ms", a[1], a[int((NR+1)/2)], a[NR] }'; }
echo
echo "first frames (the smoke's clock, from main): $(stats "${frames[@]}")"
echo "process (start to exit):                    $(stats "${walls[@]}")"
echo "binary $BIN, $RUNS runs, lavapipe on $DISPLAY_NO"
