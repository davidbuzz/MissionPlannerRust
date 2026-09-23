#!/usr/bin/env bash
# Launches a GUI binary, waits for its window, screenshots it, and leaves it up for a while.
#
# Screenshots are the progress report for GUI work (DELIVERABLES.md D6/D7): each deliverable that
# changes what the user sees gets one, committed under docs/progress/.
#
# usage: tools/screenshot.sh <output-name> [seconds-visible] [-- <binary args>]
#
# A target may carry a ~N suffix to wait N seconds after clicking it, for when a click starts
# something slow and the next control does not exist until it finishes.
#
# TYPE sends keystrokes after the clicks, for text fields. Given as "text" or as
# "control:target=text" pairs separated by commas is not supported - keep it simple: click the
# field with CLICK, then TYPE the text.
#
# SHOT_DELAY waits that many seconds after the last interaction before capturing, for state that
# arrives on the next telemetry message rather than immediately.
#
# To capture a screen taller than the window, make the window taller: MP_WINDOW=1100x2200. A wheel
# scroll was tried and does not work - see the note further down - and a tall window is both
# simpler and shows the whole thing in one image.
#
# CLICK names controls to click before capturing, comma separated, each optionally suffixed with
# :right for a right-click. Controls are addressed by name rather than coordinate, and a name may
# carry a position within the control - "map@0.25x0.75:right" right-clicks a quarter of the way
# across the map and three quarters down it. See tools/gui-click.sh. Example:
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
if [ -n "${CLICK:-}" ] || [ -n "${DRAG:-}" ]; then
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

# Put the window on one nominated monitor and keep it there.
#
# This desktop has three monitors in a single 5120x3040 X screen. Left to itself the window
# appeared on a different one almost every run - observed at 0,37 then 2998,206 then 3134,240 -
# which puts somebody's tool window on top of whatever they were doing, somewhere new each time.
# That is not a cosmetic problem: these run on a real desktop while a person is using it.
#
# SHOT_AT overrides the corner. The default is DP-1-3 at the X screen origin, chosen by the owner
# of this desktop; on another machine set SHOT_AT to a corner that is out of the way.
SHOT_AT="${SHOT_AT:-0,0}"
WANT_X="${SHOT_AT%%,*}"
WANT_Y="${SHOT_AT##*,}"

# Activated first, then moved. A window manager will often pull a window to the active monitor
# when it is activated, so activating after the move undoes it - which is exactly what was
# happening. Moving last, and checking, is what makes the placement stick.
xdotool windowactivate "$WIN_ID" 2>/dev/null
sleep 0.5

# Placed, then verified, then placed again. windowmove is a request and a window manager is free
# to answer it with something else.
place_window() {
    for _ in $(seq 1 5); do
        xdotool windowmove "$WIN_ID" "$WANT_X" "$WANT_Y" 2>/dev/null
        sleep 0.25
        AT=$(xwininfo -id "$WIN_ID" | awk '
            /Absolute upper-left X/ {x=$4}
            /Absolute upper-left Y/ {y=$4}
            END {printf "%d,%d", x, y}')
        # A title bar can offset the y by its own height, which is the window manager doing its
        # job rather than ignoring the request. Anything within 64px counts as placed.
        DX=$(( ${AT%%,*} - WANT_X )); DX=${DX#-}
        DY=$(( ${AT##*,} - WANT_Y )); DY=${DY#-}
        [ "$DX" -le 64 ] && [ "$DY" -le 64 ] && return 0
    done
    echo "warning: asked for the window at $WANT_X,$WANT_Y and it is at $AT" >&2
    return 1
}
place_window

xdotool windowraise "$WIN_ID" 2>/dev/null
sleep 0.3

# Does the window fit on the monitor it was put on?
#
# The monitor, not the X screen. The capture is an x11grab of a rectangle of the X screen, and on
# a multi-monitor desktop that rectangle can cover areas no monitor is showing - this layout has
# three monitors at different offsets inside a 5120x3040 screen, so x=0..1600 y=1440..2200 belongs
# to nothing at all. Grabbing there produces a band of whatever the server happens to hold, in an
# image that otherwise looks correct, which is the silent-wrong-screenshot failure again.
WIN_W=$(xwininfo -id "$WIN_ID" | awk '/Width:/ {print $2}')
WIN_H=$(xwininfo -id "$WIN_ID" | awk '/Height:/ {print $2}')
# xrandr prints each monitor as "WIDTH/mmxHEIGHT/mm+X+Y"; this pulls out the four numbers and
# keeps the one whose rectangle contains the window's corner.
MONITOR=$(xrandr --listmonitors 2>/dev/null | awk -v wx="$WANT_X" -v wy="$WANT_Y" '
    NR > 1 {
        geom = $3
        gsub(/\/[0-9]+/, "", geom)
        split(geom, parts, /[x+]/)
        mw = parts[1]; mh = parts[2]; mx = parts[3]; my = parts[4]
        if (wx >= mx && wx < mx + mw && wy >= my && wy < my + mh) {
            printf "%d %d %d %d", mw, mh, mx, my
            exit
        }
    }')
if [ -n "$MONITOR" ]; then
    set -- $MONITOR
    MON_W=$1; MON_H=$2; MON_X=$3; MON_Y=$4
    ROOM_W=$(( MON_X + MON_W - WANT_X ))
    ROOM_H=$(( MON_Y + MON_H - WANT_Y ))
    if [ "$WIN_W" -gt "$ROOM_W" ] || [ "$WIN_H" -gt "$ROOM_H" ]; then
        echo "warning: the window is ${WIN_W}x${WIN_H} but only ${ROOM_W}x${ROOM_H} of this" \
             "monitor is below and right of $WANT_X,$WANT_Y; the capture will include screen" \
             "area the window does not cover" >&2
    fi
fi

# Clicks happen after the window has settled and been activated: a click delivered to a window
# that does not have focus goes to whatever does.

if [ -n "${CLICK:-}" ]; then
    IFS=',' read -ra TARGETS <<< "$CLICK"
    for TARGET in "${TARGETS[@]}"; do
        BUTTON=1
        # A ~N suffix waits N seconds after this click before the next. Needed when a click starts
        # something slow - a parameter download takes twenty seconds - and the control the next
        # click wants does not exist until it finishes.
        WAIT=0.6
        case "$TARGET" in
            *~*) WAIT="${TARGET##*~}"; TARGET="${TARGET%~*}" ;;
        esac
        case "$TARGET" in
            *:right) BUTTON=3; TARGET="${TARGET%:right}" ;;
            *:middle) BUTTON=2; TARGET="${TARGET%:middle}" ;;
        esac
        "$ROOT/tools/gui-click.sh" "$PROBE_FILE" "$WIN_ID" "$TARGET" "$BUTTON" || exit 1
        # Let the click take effect and the next frame paint before the following one: a second
        # click sent into the old layout lands on whatever used to be there.
        sleep "$WAIT"
    done
fi

# Drags happen after clicks: a click puts the application into the state a drag then acts on,
# and a drag that ran first would be dragging whatever was there before.
if [ -n "${TYPE:-}" ]; then
    # --clearmodifiers so a held modifier from an earlier click does not turn letters into
    # shortcuts, and a small delay so the application sees discrete key events rather than a burst
    # the event loop coalesces.
    echo "typing '$TYPE'"
    xdotool type --window "$WIN_ID" --clearmodifiers --delay 60 "$TYPE"
    sleep 0.6
fi

if [ -n "${DRAG:-}" ]; then
    # DRAG="from>to,from2>to2" - each pair separated by >, pairs separated by commas.
    IFS=',' read -ra DRAGS <<< "$DRAG"
    for PAIR in "${DRAGS[@]}"; do
        "$ROOT/tools/gui-drag.sh" "$PROBE_FILE" "$WIN_ID" "${PAIR%%>*}" "${PAIR##*>}" || exit 1
        sleep 0.6
    done
fi

# There is no scroll step here, and that is deliberate.
#
# One was written: it resolved a named control, moved the real pointer over it with XTEST and sent
# wheel notches. The pointer landed in the right place and nothing scrolled - two captures of the
# setup screen, one with ten notches and one without, differed only in the frame counter. Whether
# that is gpui's X11 wheel handling, this window manager, or the application's own scroll state is
# not established, and it is not established because finding out means opening windows on somebody
# else's desktop.
#
# Rather than ship a step that reports success and does nothing - the failure this file has now
# been bitten by three times - there is no step. To capture something below the fold, make the
# window taller with MP_WINDOW; the X screen here is 5120x3040, so most screens fit whole.
#
# A pause between the last interaction and the capture. Some things take a moment to come back -
# an armed flag arrives on the next heartbeat, a second away - and a screenshot taken immediately
# after a click shows the state before the answer.
[ -n "${SHOT_DELAY:-}" ] && sleep "$SHOT_DELAY"

# Once more before capturing: a click can raise a window that was underneath, and a tooltip or an
# input method can take the top for itself.
#
# And then check it worked, because it does not always. The capture is an x11grab of the screen at
# the window's coordinates, so a window that is not on top produces a screenshot of whatever is -
# and it produces one silently. A shot of the joystick panel came out as a picture of an editor,
# and the only reason anybody noticed was that a human looked at it. Raising is a request a window
# manager is free to ignore, so the request is repeated and then verified.
for _ in $(seq 1 10); do
    xdotool windowraise "$WIN_ID" 2>/dev/null
    xdotool windowactivate --sync "$WIN_ID" 2>/dev/null && break
    sleep 0.3
done
# Activating may have pulled it to another monitor again, so put it back before capturing.
place_window
sleep 0.4
ACTIVE=$(xdotool getactivewindow 2>/dev/null)
if [ "$ACTIVE" != "$WIN_ID" ]; then
    echo "the window could not be raised (active window is ${ACTIVE:-none}, wanted $WIN_ID);" \
         "the capture would be of whatever is on top instead" >&2
    exit 1
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
