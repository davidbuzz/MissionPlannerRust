#!/usr/bin/env bash
# Clicks a named control in a running planner window.
#
# Screenshots prove what the application looks like when it starts. They cannot show anything that
# only exists after an interaction - a tab that is not the default, a panel that appears once an
# item is selected, a context menu. This drives those.
#
# Controls are addressed by name, not by coordinate. The application publishes where each named
# control actually ended up (see crates/mp-gui/src/probe.rs, enabled with MP_PROBE), and this looks
# the name up. Hard-coded coordinates would be the obvious approach and the wrong one: every layout
# change silently moves the target, and a click that lands on the wrong control still produces a
# screenshot, so the test goes green while testing nothing.
#
# A name may carry a position within the control: "map@0.25x0.75" clicks a quarter of the way
# across and three quarters down it. That is how a point on the map is addressed - the map has no
# named sub-controls, but a fraction of it stays correct when the window is resized or the layout
# around it changes, which a raw pixel coordinate would not.
#
# The two fractions are separated by "x" rather than a comma because the caller's list of targets
# is already comma separated, and a comma inside a target would split it in half.
#
# usage: gui-click.sh <probe-file> <window-id> <control-name>[@fxXfy] [button]
#        button: 1 left (default), 2 middle, 3 right
set -uo pipefail

# --resolve prints the window-relative coordinates and does not click, so gui-drag.sh can reuse
# this lookup rather than reimplementing it and drifting from it.
RESOLVE_ONLY=""
if [ "${1:-}" = "--resolve" ]; then
    RESOLVE_ONLY=1
    shift
fi

PROBE="${1:?usage: gui-click.sh [--resolve] <probe-file> <window-id> <control-name>[@fxXfy] [button]}"
WIN_ID="${2:?window id}"
TARGET="${3:?control name}"
BUTTON="${4:-1}"

# The "@" is a fraction's only when a fraction follows it: a name may carry one itself (the
# MAVFtp page's "@SYS/" node, mavftp-node-@SYS/).
NAME="$TARGET"
FRACTION=""
if [[ "$TARGET" =~ ^(.*)@([0-9.]+[x,][0-9.]+)$ ]]; then
    NAME="${BASH_REMATCH[1]}"
    FRACTION="${BASH_REMATCH[2]}"
fi

# The probe file is written when a control moves, so it may not exist the instant the window maps.
# The name is text, not a pattern (-F): a log field called MAV[0].flags has brackets in it.
# Looked for every 50 ms, up to ten seconds: a control that appears 60 ms after the click that
# made it costs 60 ms, not a quarter second.
for _ in $(seq 1 200); do
    [ -s "$PROBE" ] && grep -qF "\"$NAME\"" "$PROBE" && break
    sleep 0.05
done

if ! grep -qF "\"$NAME\"" "$PROBE" 2>/dev/null; then
    echo "control '$NAME' not found in $PROBE" >&2
    echo "known controls:" >&2
    grep -o '"[a-zA-Z0-9_.-]*":' "$PROBE" 2>/dev/null | tr -d '":' | sed 's/^/  /' >&2
    exit 1
fi

# One line per control, so a line-oriented read is enough and jq is not a dependency.
# The application rewrites the probe file whenever a control moves, and a click can move things:
# a chip whose label grows reflows the row after it. Resolving the target from a file written
# before that reflow clicks where the control was. So wait until the file has been still for
# 100 ms (at most 1 s) before reading it - a reflow follows a click within a frame or two.
# Nanosecond modification times: whole seconds plus the size, as this read before, could not
# tell two writes 100 ms apart from one, so the wait was no wait at all.
# Still for 300 ms, not 100: a page chosen from the SETUP list moved the list's rows once more
# a few frames after the click that chose it, and a click 100 ms into the calm landed on the
# neighbour four rows down (setup-list.gui, 2026-09-26).
STAMP=""
SAME=0
for _ in $(seq 1 12); do
    NOW=$(stat -c '%.9Y' "$PROBE" 2>/dev/null || echo "")
    if [ -n "$STAMP" ] && [ "$NOW" = "$STAMP" ]; then
        SAME=$((SAME + 1))
        [ "$SAME" -ge 3 ] && break
    else
        SAME=0
    fi
    STAMP="$NOW"
    sleep 0.1
done
LINE=$(grep -F "\"$NAME\"" "$PROBE")
if [ -z "$FRACTION" ]; then
    COORDS=$(echo "$LINE" | sed -n 's/.*"centre_x": \([0-9.-]*\), "centre_y": \([0-9.-]*\).*/\1 \2/p')
else
    BOX=$(echo "$LINE" | sed -n 's/.*"x": \([0-9.-]*\), "y": \([0-9.-]*\), "width": \([0-9.-]*\), "height": \([0-9.-]*\).*/\1 \2 \3 \4/p')
    FX="${FRACTION%%[x,]*}"
    FY="${FRACTION#*[x,]}"
    COORDS=$(echo "$BOX $FX $FY" | awk '{printf "%.1f %.1f", $1 + $3 * $5, $2 + $4 * $6}')
fi
X=$(echo "$COORDS" | cut -d' ' -f1 | cut -d. -f1)
Y=$(echo "$COORDS" | cut -d' ' -f2 | cut -d. -f1)

if [ -z "$X" ] || [ -z "$Y" ]; then
    echo "could not read coordinates for '$NAME' from $PROBE" >&2
    exit 1
fi

# Coordinates from the probe are relative to the window, which is what --window takes. Using
# absolute screen coordinates would break the moment the window manager moved the window.
if [ -n "$RESOLVE_ONLY" ]; then
    echo "$X $Y"
    exit 0
fi

echo "clicking '$TARGET' (button $BUTTON) at window-relative $X,$Y"
# Move first and press after a beat: a press in the same batch as the move can reach the
# application before it has hit-tested the new pointer position, and a click whose press is not
# over the control is not a click on it.
xdotool mousemove --window "$WIN_ID" "$X" "$Y"
sleep 0.05
xdotool click "$BUTTON"
