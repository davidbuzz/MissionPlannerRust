#!/usr/bin/env bash
# Launches a GUI binary, waits for its window, screenshots it, and leaves it up for a while.
#
# Screenshots are the progress report for GUI work (DELIVERABLES.md D6/D7): each deliverable that
# changes what the user sees gets one, committed under docs/progress/.
#
# usage: tools/screenshot.sh <output-name> [seconds-visible] [-- <binary args>]
set -uo pipefail

NAME="${1:?usage: screenshot.sh <output-name> [seconds] [-- args]}"
SECONDS_VISIBLE="${2:-12}"
shift 2 2>/dev/null || shift 1
[ "${1:-}" = "--" ] && shift

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/docs/progress/$NAME.png"
BIN="$ROOT/target/debug/mpr-gui"
WINDOW_TITLE="Mission Planner"

[ -x "$BIN" ] || { echo "binary not built: $BIN" >&2; exit 1; }
: "${DISPLAY:=:0}"
export DISPLAY

echo "launching $BIN $*"
"$BIN" "$@" &
APP_PID=$!
trap 'kill $APP_PID 2>/dev/null' EXIT

# Wait for the window to map. A GPU-backed window can take a moment to appear.
WIN_ID=""
for _ in $(seq 1 60); do
    WIN_ID=$(xdotool search --name "$WINDOW_TITLE" 2>/dev/null | head -1)
    [ -n "$WIN_ID" ] && break
    kill -0 $APP_PID 2>/dev/null || { echo "app exited before showing a window" >&2; wait $APP_PID; exit 1; }
    sleep 0.5
done
[ -n "$WIN_ID" ] || { echo "no window titled '$WINDOW_TITLE' appeared" >&2; exit 1; }

# Let the first frames render before capturing.
sleep 2
xdotool windowactivate "$WIN_ID" 2>/dev/null
sleep 1

GEO=$(xwininfo -id "$WIN_ID" | awk '
    /Absolute upper-left X/ {x=$4}
    /Absolute upper-left Y/ {y=$4}
    /Width:/ {w=$2}
    /Height:/ {h=$2}
    END {printf "%dx%d+%d,%d", w, h, x, y}')
SIZE="${GEO%%+*}"
POS="${GEO##*+}"
echo "window $WIN_ID at $SIZE offset $POS"

ffmpeg -hide_banner -loglevel error -y \
    -f x11grab -video_size "$SIZE" -i "$DISPLAY+$POS" \
    -frames:v 1 "$OUT"

if [ -s "$OUT" ]; then
    echo "saved $OUT ($(du -h "$OUT" | cut -f1))"
else
    echo "screenshot failed" >&2
    exit 1
fi

# Keep the window visible so a human watching sees it too.
echo "leaving the window up for ${SECONDS_VISIBLE}s"
sleep "$SECONDS_VISIBLE"
