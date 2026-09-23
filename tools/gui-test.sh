#!/usr/bin/env bash
# Drives the application through a sequence of interactions and checks what happened.
#
# usage: tools/gui-test.sh <script.gui> [-- <binary args>]
#
# The thing `screenshot.sh` cannot do. That one clicks and produces a PNG, and a human has to look
# at the PNG - which `DELIVERABLES.md`'s test policy rules out in as many words: "if it is not a
# program that fails, it is not a test". A click-to-add-waypoint that quietly stopped adding
# waypoints would still produce a perfectly good screenshot and still exit zero.
#
# This runs a script of steps and exits non-zero on the first expectation that does not hold,
# naming what it wanted and what it found.
#
#   # comments and blank lines are ignored
#   screen plan                 open on a screen (MP_SCREEN)
#   window 1600x1200            window size (MP_WINDOW)
#   settle 6                    wait, for telemetry to arrive or a view to settle
#   click map@0.45x0.40         click a named control, as tools/gui-click.sh addresses them
#   click tab-plan:right        a right-click
#   type flight.bin             type into whatever has focus
#   key Return                  press a named key
#   expect mission.items 3      assert a published fact equals a value
#   expect status ~ saved       assert a fact contains a substring
#   expect log.open true
#
# The facts come from the application itself: set MP_FACTS and it writes what it believes after
# every frame. `crates/mp-gui/src/facts.rs` lists them, and an unknown key is an error rather than
# a silent pass - a test asserting on a fact that no longer exists must fail, not succeed.
set -uo pipefail

SCRIPT="${1:?usage: gui-test.sh <script.gui> [-- <binary args>]}"
shift
[ "${1:-}" = "--" ] && shift

[ -r "$SCRIPT" ] || { echo "cannot read $SCRIPT" >&2; exit 2; }

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/target/debug/mpr-gui"
WINDOW_TITLE="Mission Planner"
[ -x "$BIN" ] || { echo "binary not built: $BIN" >&2; exit 1; }
: "${DISPLAY:=:0}"
export DISPLAY

# The binary's arguments, saved before anything else touches the positional parameters.
#
# The pre-scan below uses `set --` to split each line into words, which overwrites $@. Without
# this the application was launched with the last line of the test script as its arguments, saw
# "expect", printed its usage and exited - and the failure read as "app exited before showing a
# window", which points at everything except the cause.
APP_ARGS=("$@")

# Read the directives that must be set before the application starts.
while IFS= read -r LINE; do
    LINE="${LINE%%#*}"
    # shellcheck disable=SC2086 # deliberate word splitting
    set -- $LINE
    case "${1:-}" in
        screen) export MP_SCREEN="${2:-fly}" ;;
        window) export MP_WINDOW="${2:-1600x1200}" ;;
    esac
done < "$SCRIPT"

# A test never records a flight, never fetches a tile and never opens a real port unless the
# caller asked for one: a test run that leaves .tlog files behind or hits a tile server is a test
# nobody can run twice.
export MP_NO_RECORD=1
: "${MP_NO_TILES:=1}"; export MP_NO_TILES

PROBE_FILE="$(mktemp -t mpr-probe-XXXXXX.json)"
FACTS_FILE="$(mktemp -t mpr-facts-XXXXXX.conf)"
export MP_PROBE="$PROBE_FILE"
export MP_FACTS="$FACTS_FILE"

# A settings file of its own, per run.
#
# The application remembers things on purpose - the last link, the window size, the altitude frame
# new waypoints get. A test that inherits them is a test whose result depends on what the last
# test did: the altitude-frame test passed, wrote "terrain" to the real settings file, and the
# next run of the same test started in terrain and failed its first expectation. Tests that must
# be run in a particular order, once, are not tests.
SETTINGS_FILE="$(mktemp -t mpr-settings-XXXXXX.conf)"
export MP_SETTINGS="$SETTINGS_FILE"

POINTER_HOME=$(xdotool getmouselocation --shell 2>/dev/null | awk -F= '/^X=/{x=$2} /^Y=/{y=$2} END{print x" "y}')
SHOT_AT="${SHOT_AT:-2560,0}"
xdotool mousemove "${SHOT_AT%%,*}" "${SHOT_AT##*,}" 2>/dev/null

"$BIN" "${APP_ARGS[@]}" &
APP_PID=$!
cleanup() {
    kill "$APP_PID" 2>/dev/null
    wait "$APP_PID" 2>/dev/null
    rm -f "$PROBE_FILE" "$FACTS_FILE" "${FACTS_FILE%.conf}.facts.tmp" \
          "$SETTINGS_FILE" "${SETTINGS_FILE%.conf}.tmp"
    # shellcheck disable=SC2086 # two words on purpose
    [ -n "$POINTER_HOME" ] && xdotool mousemove $POINTER_HOME 2>/dev/null
}
trap cleanup EXIT INT TERM HUP

# The window must belong to the process this script started. The real Mission Planner shares our
# title, and a test that drives it instead of us is worse than a test that does not run.
WIN_ID=""
for _ in $(seq 1 60); do
    for CANDIDATE in $(xdotool search --pid "$APP_PID" --onlyvisible --name "$WINDOW_TITLE" 2>/dev/null); do
        OWNER=$(xdotool getwindowpid "$CANDIDATE" 2>/dev/null)
        [ "$OWNER" = "$APP_PID" ] && xwininfo -id "$CANDIDATE" >/dev/null 2>&1 && WIN_ID="$CANDIDATE"
    done
    [ -n "$WIN_ID" ] && break
    kill -0 $APP_PID 2>/dev/null || { echo "app exited before showing a window" >&2; exit 1; }
    sleep 0.5
done
[ -n "$WIN_ID" ] || { echo "no window owned by pid $APP_PID appeared" >&2; exit 1; }

sleep 1
xdotool windowactivate --sync "$WIN_ID" 2>/dev/null
xdotool windowmove "$WIN_ID" "${SHOT_AT%%,*}" "${SHOT_AT##*,}" 2>/dev/null
sleep 0.5

# Reads one fact. Empty if the key is absent, which `expect` reports as a failure rather than
# comparing against nothing.
fact() {
    [ -s "$FACTS_FILE" ] || return 1
    sed -n "s/^$1 = \(.*\)$/\1/p" "$FACTS_FILE" | tail -1
}

FAILURES=0
LINE_NO=0
while IFS= read -r RAW; do
    LINE_NO=$((LINE_NO + 1))
    LINE="${RAW%%#*}"
    # shellcheck disable=SC2086 # deliberate word splitting into positional parameters
    set -- $LINE
    [ $# -eq 0 ] && continue

    case "$1" in
        screen|window) ;;  # already applied before launch
        settle)
            sleep "${2:-1}"
            ;;
        click)
            TARGET="${2:?click needs a target}"
            BUTTON=1
            case "$TARGET" in
                *:right) BUTTON=3; TARGET="${TARGET%:right}" ;;
                *:middle) BUTTON=2; TARGET="${TARGET%:middle}" ;;
            esac
            if ! "$ROOT/tools/gui-click.sh" "$PROBE_FILE" "$WIN_ID" "$TARGET" "$BUTTON"; then
                echo "line $LINE_NO: could not click '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.6
            ;;
        type)
            shift
            xdotool type --window "$WIN_ID" --clearmodifiers --delay 60 "$*"
            sleep 0.6
            ;;
        key)
            xdotool key --window "$WIN_ID" --clearmodifiers "${2:?key needs a name}"
            sleep 0.6
            ;;
        expect)
            KEY="${2:?expect needs a key}"
            OP="${3:?expect needs a value}"
            # `expect key value` is equality; `expect key ~ value` is containment.
            if [ "$OP" = "~" ]; then
                shift 3
                WANT="$*"
                GOT=$(fact "$KEY")
                case "$GOT" in
                    *"$WANT"*) echo "  ok   $KEY contains '$WANT'" ;;
                    *)
                        echo "FAIL line $LINE_NO: $KEY is '$GOT', expected to contain '$WANT'" >&2
                        FAILURES=$((FAILURES + 1))
                        ;;
                esac
            else
                shift 2
                WANT="$*"
                GOT=$(fact "$KEY")
                if [ -z "$GOT" ] && ! grep -q "^$KEY = " "$FACTS_FILE" 2>/dev/null; then
                    echo "FAIL line $LINE_NO: no such fact '$KEY'" >&2
                    echo "       known facts:" >&2
                    sed 's/^/         /' "$FACTS_FILE" >&2 2>/dev/null
                    FAILURES=$((FAILURES + 1))
                elif [ "$GOT" = "$WANT" ]; then
                    echo "  ok   $KEY = $WANT"
                else
                    echo "FAIL line $LINE_NO: $KEY is '$GOT', expected '$WANT'" >&2
                    FAILURES=$((FAILURES + 1))
                fi
            fi
            ;;
        *)
            echo "line $LINE_NO: unknown directive '$1'" >&2
            FAILURES=$((FAILURES + 1))
            ;;
    esac
done < "$SCRIPT"

if [ "$FAILURES" -gt 0 ]; then
    echo "$(basename "$SCRIPT"): $FAILURES failure(s)" >&2
    exit 1
fi
echo "$(basename "$SCRIPT"): passed"
