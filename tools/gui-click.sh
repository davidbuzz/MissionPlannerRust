#!/usr/bin/env bash
# Clicks a named control in a running mpr-gui window.
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
# usage: gui-click.sh <probe-file> <window-id> <control-name> [button]
#        button: 1 left (default), 2 middle, 3 right
set -uo pipefail

PROBE="${1:?usage: gui-click.sh <probe-file> <window-id> <control-name> [button]}"
WIN_ID="${2:?window id}"
NAME="${3:?control name}"
BUTTON="${4:-1}"

# The probe file is written when a control moves, so it may not exist the instant the window maps.
for _ in $(seq 1 40); do
    [ -s "$PROBE" ] && grep -q "\"$NAME\"" "$PROBE" && break
    sleep 0.25
done

if ! grep -q "\"$NAME\"" "$PROBE" 2>/dev/null; then
    echo "control '$NAME' not found in $PROBE" >&2
    echo "known controls:" >&2
    grep -o '"[a-zA-Z0-9_.-]*":' "$PROBE" 2>/dev/null | tr -d '":' | sed 's/^/  /' >&2
    exit 1
fi

# One line per control, so a line-oriented read is enough and jq is not a dependency.
COORDS=$(grep "\"$NAME\"" "$PROBE" | sed -n 's/.*"centre_x": \([0-9.-]*\), "centre_y": \([0-9.-]*\).*/\1 \2/p')
X=$(echo "$COORDS" | cut -d' ' -f1 | cut -d. -f1)
Y=$(echo "$COORDS" | cut -d' ' -f2 | cut -d. -f1)

if [ -z "$X" ] || [ -z "$Y" ]; then
    echo "could not read coordinates for '$NAME' from $PROBE" >&2
    exit 1
fi

# Coordinates from the probe are relative to the window, which is what --window takes. Using
# absolute screen coordinates would break the moment the window manager moved the window.
echo "clicking '$NAME' (button $BUTTON) at window-relative $X,$Y"
xdotool mousemove --window "$WIN_ID" "$X" "$Y" click "$BUTTON"
