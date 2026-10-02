#!/usr/bin/env bash
# Runs GUI scripts on a virtual X display, so they need neither the desktop nor its pointer.
#
# The application is drawn by gpui through Vulkan; on an Xvfb screen Mesa's lavapipe (a software
# Vulkan device, `lvp_icd.json`) draws it, and xdotool's XTEST clicks and keys reach it as they do
# on the real display. The desktop's windows, the mouse and the keyboard are never touched. Found
# on 2026-09-26, when the desktop's session-failed screen sat over every monitor and took every
# click: on `:99` the same scripts passed.
#
# Slower than the GPU: lavapipe draws a 1600x1200 frame in tens of milliseconds, so a budget
# recorded on the GPU can be missed here by a script that paints a great deal. A budget is
# recorded from the slower of the two.
#
#   tools/gui-headless.sh name [name ...]        the scripts, as tools/gui-suite.sh names them
#   tools/gui-headless.sh --all                   every script
#   tools/gui-headless.sh -- tests/gui/x.gui -- tcp:127.0.0.1:5760   one script through gui-test.sh
#
# MP_GUI_BIN, MP_GUI_SHOT_DIR and -o work as for the runners. HEADLESS_DISPLAY (default :99) is the
# display; it is started if nothing listens on it, 2560x1440, and left running for the next call.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DISPLAY_NO="${HEADLESS_DISPLAY:-:99}"
SOCKET="/tmp/.X11-unix/X${DISPLAY_NO#:}"
if [ ! -S "$SOCKET" ]; then
    # With every inherited descriptor closed: run under `flock`, the display server inherited the
    # lock's descriptor and held the machine's build lock for as long as it lived (2026-10-02).
    nohup setsid Xvfb "$DISPLAY_NO" -screen 0 2560x1440x24 +extension GLX +extension RANDR +render -noreset \
        > "${TMPDIR:-/tmp}/xvfb-${DISPLAY_NO#:}.log" 2>&1 < /dev/null 3>&- 4>&- 5>&- 6>&- 7>&- 8>&- 9>&- &
    for _ in $(seq 1 50); do [ -S "$SOCKET" ] && break; sleep 0.1; done
    [ -S "$SOCKET" ] || { echo "Xvfb $DISPLAY_NO did not start" >&2; exit 2; }
fi
ICD=/usr/share/vulkan/icd.d/lvp_icd.json
[ -f "$ICD" ] || { echo "no lavapipe ($ICD): install mesa-vulkan-drivers" >&2; exit 2; }
export DISPLAY="$DISPLAY_NO" VK_ICD_FILENAMES="$ICD" SHOT_AT="${SHOT_AT:-0,0}"
if [ "${1:-}" = "--" ]; then
    shift
    exec "$ROOT/tools/gui-test.sh" "$@"
fi
exec "$ROOT/tools/gui-suite.sh" "$@"
