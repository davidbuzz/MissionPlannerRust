#!/usr/bin/env bash
# Launches a GUI binary, waits for its window, screenshots it, and leaves it up for a while.
#
# Screenshots are the progress report for GUI work (DELIVERABLES.md D6/D7): each deliverable that
# changes what the user sees gets one, committed under docs/progress/.
#
# usage: tools/screenshot.sh <output-name> [seconds-visible] [-- <binary args>]
#
# CLICK names controls to click before capturing, comma separated, each optionally suffixed with
# :right for a right-click. Controls are addressed by name rather than coordinate; see
# tools/gui-click.sh. Example:
#
#     CLICK=tab-plan,plan-row-0 tools/screenshot.sh plan-editor 5 -- tcp:127.0.0.1:5760
#
# This is how anything that only exists after an interaction gets into a screenshot: a tab that is
# not the default, a panel that appears once an item is selected, a context menu.
#
# Windows are left up for 5 seconds by default: long enough for a human to see what the screenshot
# claims, short enough not to sit on someone's desktop. These run on a real desktop, not a
# headless CI box.
set -uo pipefail

NAME="${1:?usage: screenshot.sh <output-name> [seconds] [-- args]}"
SECONDS_VISIBLE="${2:-5}"
shift 2 2>/dev/null || shift 1
[ "${1:-}" = "--" ] && shift

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/docs/progress/$NAME.png"
BIN="$ROOT/target/debug/mpr-gui"
WINDOW_TITLE="Mission Planner"

[ -x "$BIN" ] || { echo "binary not built: $BIN" >&2; exit 1; }
: "${DISPLAY:=:0}"
export DISPLAY

# Never leave a window on someone's desktop. The trap covers a normal exit and the signals
# `timeout` and Ctrl-C send; the pre-kill covers a previous run that was killed with -9 and so
# never ran its own trap.
pkill -f "$(basename "$BIN")" 2>/dev/null && sleep 1

# Clicking needs the application to report where its controls are, so the probe is switched on
# whenever a click is asked for. It writes nothing otherwise.
PROBE_FILE=""
if [ -n "${CLICK:-}" ]; then
    PROBE_FILE="$(mktemp -t mpr-probe-XXXXXX.json)"
    export MP_PROBE="$PROBE_FILE"
fi

echo "launching $BIN $*"
"$BIN" "$@" &
APP_PID=$!
cleanup() {
    kill "$APP_PID" 2>/dev/null
    wait "$APP_PID" 2>/dev/null
    [ -n "$PROBE_FILE" ] && rm -f "$PROBE_FILE" "${PROBE_FILE%.json}.tmp"
}
trap cleanup EXIT INT TERM HUP

# Wait for the window to map. A GPU-backed window can take a moment to appear.
#
# Search by pid, not by name: a window manager reparents the client window into a decoration
# frame, and the frame inherits the title. Matching on the title therefore returns two ids - the
# frame and the client - and picking either at random makes captures inconsistent in size and
# position. Only the client window belongs to our process.
WIN_ID=""
for _ in $(seq 1 60); do
    WIN_ID=$(xdotool search --pid "$APP_PID" 2>/dev/null | head -1)
    [ -z "$WIN_ID" ] && WIN_ID=$(xdotool search --name "$WINDOW_TITLE" 2>/dev/null | tail -1)
    [ -n "$WIN_ID" ] && break
    kill -0 $APP_PID 2>/dev/null || { echo "app exited before showing a window" >&2; wait $APP_PID; exit 1; }
    sleep 0.5
done
[ -n "$WIN_ID" ] || { echo "no window titled '$WINDOW_TITLE' appeared" >&2; exit 1; }

# Let the window settle before capturing. Telemetry screenshots need longer than a static
# window: a simulated GPS takes tens of seconds to acquire, and a shot taken too early shows
# "no fix" and looks like a bug in the port rather than a vehicle that has not warmed up.
sleep "${SETTLE:-2}"
xdotool windowactivate "$WIN_ID" 2>/dev/null
sleep 1

# Clicks happen after the window has settled and been activated: a click delivered to a window
# that does not have focus goes to whatever does.
if [ -n "${CLICK:-}" ]; then
    IFS=',' read -ra TARGETS <<< "$CLICK"
    for TARGET in "${TARGETS[@]}"; do
        BUTTON=1
        case "$TARGET" in
            *:right) BUTTON=3; TARGET="${TARGET%:right}" ;;
            *:middle) BUTTON=2; TARGET="${TARGET%:middle}" ;;
        esac
        "$ROOT/tools/gui-click.sh" "$PROBE_FILE" "$WIN_ID" "$TARGET" "$BUTTON" || exit 1
        # Let the click take effect and the next frame paint before the following one: a second
        # click sent into the old layout lands on whatever used to be there.
        sleep 0.6
    done
fi

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

# Keep the window visible so a human watching sees it too, then close it.
#
# Note MP_BENCH: it repaints as fast as the executor will schedule, which is how the renderer is
# measured and also what makes the window appear to flicker. It is never set for an ordinary
# screenshot, only when a number is being taken.
echo "leaving the window up for ${SECONDS_VISIBLE}s"
sleep "$SECONDS_VISIBLE"
echo "closing"
