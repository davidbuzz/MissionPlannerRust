#!/usr/bin/env bash
# Drags between two named positions in a running planner window.
#
# The companion to gui-click.sh, and addressed the same way: by control name, optionally with a
# position inside it. Dragging is how a map is panned and how a waypoint is moved, so a test suite
# that can only click cannot reach either.
#
# The pointer is moved in steps rather than jumped, because a press followed by a single jump and a
# release is not what an application sees from a human, and a handler that only acts on movement
# would see nothing at all.
#
# usage: gui-drag.sh <probe-file> <window-id> <from>[@fxXfy] <to>[@fxXfy] [button]
set -uo pipefail

PROBE="${1:?usage: gui-drag.sh <probe-file> <window-id> <from> <to> [button]}"
WIN_ID="${2:?window id}"
FROM="${3:?from target}"
TO="${4:?to target}"
BUTTON="${5:-1}"

HERE="$(cd "$(dirname "$0")" && pwd)"

# Resolve a target to window-relative pixels, reusing the click tool's lookup so the two cannot
# disagree about where a control is.
resolve() {
    "$HERE/gui-click.sh" --resolve "$PROBE" "$WIN_ID" "$1"
}

FROM_XY=$(resolve "$FROM") || { echo "could not resolve $FROM" >&2; exit 1; }
TO_XY=$(resolve "$TO") || { echo "could not resolve $TO" >&2; exit 1; }

FROM_X="${FROM_XY%% *}"; FROM_Y="${FROM_XY##* }"
TO_X="${TO_XY%% *}"; TO_Y="${TO_XY##* }"

echo "dragging '$FROM' ($FROM_X,$FROM_Y) to '$TO' ($TO_X,$TO_Y)"

xdotool mousemove --window "$WIN_ID" "$FROM_X" "$FROM_Y"
sleep 0.2
xdotool mousedown "$BUTTON"
sleep 0.2
STEPS=12
for i in $(seq 1 $STEPS); do
    X=$(awk "BEGIN{printf \"%d\", $FROM_X + ($TO_X - $FROM_X) * $i / $STEPS}")
    Y=$(awk "BEGIN{printf \"%d\", $FROM_Y + ($TO_Y - $FROM_Y) * $i / $STEPS}")
    xdotool mousemove --window "$WIN_ID" "$X" "$Y"
    sleep 0.04
done
sleep 0.2
xdotool mouseup "$BUTTON"
