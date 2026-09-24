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
#   tiles offline               configure a tile source but fetch nothing (default: no tiles)
#   env MP_TILE_CACHE $WORK/c   export a variable before launch; the rest of the line is the value
#   setup tools/seed $WORK/c    run a command, from the repo root, before launch
#   settle 6                    wait, for telemetry to arrive or a view to settle
#   click map@0.45x0.40         click a named control, as tools/gui-click.sh addresses them
#   click tab-plan:right        a right-click
#   doubleclick log-chart@0.5x0.5  a double click: two left presses at one point, 80 ms apart
#   scroll servo-SERVO9_FUNCTION-list down 3 [ms]  the wheel over a control: up or down, N notches, a gap between them
#   hover map@0.40x0.40          move the pointer onto a control and press nothing
#   type flight.bin             type into whatever has focus
#   key Return                  press a named key
#   expect mission.items 3      assert a published fact equals a value
#   expect status ~ saved       assert a fact contains a substring
#   expect map.tiles.disk > 0   assert a fact is an integer greater than a value
#   expect map.tiles.drawn >= 4 ... or at least a value
#   expect frame.p99 < 8        ... or less than a value
#   expect log.open true
#   restart                     close the window by its close box and start the application again
#
# `restart` is for a test of what survives a restart. It sends the window WM_DELETE_WINDOW, as a
# window manager's close box does - so the application does what it does on closing, which is to
# save Mission Planner's config.xml - waits for the process to end (a failure if it has not within
# ten seconds), and starts it again with the same environment, arguments and scratch directory.
# The facts and control positions the old process published are cleared first, so nothing is read
# from it; `settle` after a `restart` as after the first start. Closing takes python3 with the Xlib
# module (python3-xlib); without it `restart` fails rather than killing, since a kill is not a
# close and saves nothing.
#
# `env` and `setup` are for a test that needs the world arranged before the application starts -
# a tile cache laid out the way another program writes it, say. Every `env` is exported first
# (after the harness's own variables, so a test's choice wins), then every `setup` runs in file
# order through `bash -c`; one that exits non-zero aborts the run with status 2, because a test
# whose preconditions were not met has not failed, it has not run. In both, the literal text
# `$WORK` becomes a scratch directory made for this run and removed after it, so nothing a test
# lays out survives it or collides with a concurrent run. A `#` starts a comment anywhere on a
# line, commands included.
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
# The debug binary, as every script runs against; MP_GUI_BIN names another, such as a release
# build for a measurement whose number the debug build cannot stand for (tests/gui/storm.gui).
BIN="${MP_GUI_BIN:-$ROOT/target/debug/mpr-gui}"
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

# Everything cleanup touches, empty until made, so a run aborted before launch - a failed `setup` -
# removes what it did make and nothing else. `set -u` would otherwise turn the first unset name
# into a second error on the way out, and an empty FACTS_FILE into a relative `.facts.tmp`.
APP_PID=""
PROBE_FILE=""
FACTS_FILE=""
SETTINGS_FILE=""
POINTER_HOME=""
WORK="$(mktemp -d -t mpr-work-XXXXXX)" || { echo "cannot make a scratch directory" >&2; exit 2; }
cleanup() {
    if [ -n "$APP_PID" ]; then
        kill "$APP_PID" 2>/dev/null
        wait "$APP_PID" 2>/dev/null
    fi
    [ -n "$PROBE_FILE" ] && rm -f "$PROBE_FILE"
    [ -n "$FACTS_FILE" ] && rm -f "$FACTS_FILE" "${FACTS_FILE%.conf}.facts.tmp"
    [ -n "$SETTINGS_FILE" ] && rm -f "$SETTINGS_FILE" "${SETTINGS_FILE%.conf}.tmp"
    rm -rf "$WORK"
    # shellcheck disable=SC2086 # two words on purpose
    [ -n "$POINTER_HOME" ] && xdotool mousemove $POINTER_HOME 2>/dev/null
}
trap cleanup EXIT INT TERM HUP

# The text of a line after its first word, with the surrounding whitespace trimmed and nothing
# else touched. `env` values and `setup` commands are taken from this rather than rejoined from
# the split words, which would collapse spacing and expand any `*` against the current directory.
rest_of() {
    local rest="${2#"${2%%[![:space:]]*}"}"
    rest="${rest#"$1"}"
    rest="${rest#"${rest%%[![:space:]]*}"}"
    printf '%s' "${rest%"${rest##*[![:space:]]}"}"
}

ENV_NAMES=()
ENV_VALUES=()
SETUP_COMMANDS=()
SETUP_LINES=()
SCAN_NO=0

# Read the directives that must be set before the application starts.
while IFS= read -r LINE; do
    SCAN_NO=$((SCAN_NO + 1))
    LINE="${LINE%%#*}"
    # shellcheck disable=SC2086 # deliberate word splitting
    set -- $LINE
    case "${1:-}" in
        screen) export MP_SCREEN="${2:-fly}" ;;
        window) export MP_WINDOW="${2:-1600x1200}" ;;
        tiles) TILES="${2:-off}" ;;
        env)
            if ! [[ "${2:-}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
                echo "line $SCAN_NO: env needs a variable name, not '${2:-}'" >&2
                exit 2
            fi
            VALUE=$(rest_of "$2" "$(rest_of env "$LINE")")
            ENV_NAMES+=("$2")
            ENV_VALUES+=("${VALUE//\$WORK/$WORK}")
            ;;
        setup)
            COMMAND=$(rest_of setup "$LINE")
            [ -n "$COMMAND" ] || { echo "line $SCAN_NO: setup needs a command" >&2; exit 2; }
            SETUP_COMMANDS+=("${COMMAND//\$WORK/$WORK}")
            SETUP_LINES+=("$SCAN_NO")
            ;;
    esac
done < "$SCRIPT"

# A test never records a flight, never fetches a tile and never opens a real port unless the
# caller asked for one: a test run that leaves .tlog files behind or hits a tile server is a test
# nobody can run twice.
export MP_NO_RECORD=1

# Map tiles, which a test does not want by default.
#
#   (nothing)      no tile source at all - the fastest, and right for anything not about maps
#   tiles offline  the source is configured but nothing is fetched; use this to assert on which
#                  provider is selected without asking a tile server for anything
#   tiles on       fetches for real. Only for a test that is about fetching.
#
# The default is off because a test suite that pulls tiles is one that fails when the network
# does, and one that a provider is entitled to be annoyed about.
case "${TILES:-off}" in
    on)      unset MP_NO_TILES; unset MP_OFFLINE ;;
    offline) unset MP_NO_TILES; export MP_OFFLINE=1 ;;
    *)       export MP_NO_TILES=1 ;;
esac

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

for I in "${!ENV_NAMES[@]}"; do
    export "${ENV_NAMES[$I]}=${ENV_VALUES[$I]}"
done

for I in "${!SETUP_COMMANDS[@]}"; do
    (cd "$ROOT" && bash -c "${SETUP_COMMANDS[$I]}")
    STATUS=$?
    if [ "$STATUS" -ne 0 ]; then
        echo "line ${SETUP_LINES[$I]}: setup exited $STATUS, not running the test: ${SETUP_COMMANDS[$I]}" >&2
        exit 2
    fi
done

# Mission Planner's config.xml, which the application writes as Mission Planner does: on starting,
# on the FLIGHT DATA and FLIGHT PLAN buttons, and on closing. A test that gave it neither a data
# directory of its own (`env XDG_DATA_HOME $WORK`) nor a file (`env MP_CONFIG_XML ...`) gets a copy
# of the file it would have read, in $WORK: it reads what it always read, and the settings of the
# Mission Planner installed on the machine running it are never rewritten by a test. The directory
# is found as the application finds it - ~/Mission Planner if that exists, else the XDG data one.
if [ -z "${MP_CONFIG_XML:-}" ]; then
    if [ -d "$HOME/Mission Planner" ]; then
        DATA_DIR="$HOME/Mission Planner"
    else
        DATA_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/Mission Planner"
    fi
    case "$DATA_DIR" in
        "$WORK"/*) ;;
        *)
            export MP_CONFIG_XML="$WORK/config.xml"
            if [ -f "$DATA_DIR/config.xml" ]; then
                cp "$DATA_DIR/config.xml" "$MP_CONFIG_XML"
            fi
            ;;
    esac
fi

POINTER_HOME=$(xdotool getmouselocation --shell 2>/dev/null | awk -F= '/^X=/{x=$2} /^Y=/{y=$2} END{print x" "y}')
SHOT_AT="${SHOT_AT:-2560,0}"
xdotool mousemove "${SHOT_AT%%,*}" "${SHOT_AT##*,}" 2>/dev/null

# Starts the application and waits for its window, then activates it and moves it where the
# pointer waits. Once before the steps, and again for each `restart`.
start_app() {
    "$BIN" "${APP_ARGS[@]}" </dev/null &
    APP_PID=$!

    # The window must belong to the process this script started. The real Mission Planner shares
    # our title, and a test that drives it instead of us is worse than a test that does not run.
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
}

# Sends a window WM_DELETE_WINDOW, as a window manager's close box does. xdotool's windowclose
# destroys the window instead, which the application never hears about.
close_window() {
    python3 - "$1" <<'PY'
import sys
from Xlib import X, display, protocol
d = display.Display()
window = d.create_resource_object("window", int(sys.argv[1]))
event = protocol.event.ClientMessage(
    window=window,
    client_type=d.intern_atom("WM_PROTOCOLS"),
    data=(32, [d.intern_atom("WM_DELETE_WINDOW"), X.CurrentTime, 0, 0, 0]),
)
window.send_event(event, event_mask=X.NoEventMask)
d.flush()
PY
}

start_app

# Reads one fact. Empty if the key is absent, which `expect` reports as a failure rather than
# comparing against nothing.
fact() {
    [ -s "$FACTS_FILE" ] || return 1
    sed -n "s/^$1 = \(.*\)$/\1/p" "$FACTS_FILE" | tail -1
}

# Reports a fact the application never published. Returns 0 - "yes, it is missing" - after
# printing what was published instead, so a typo in a key reads as one rather than as a mismatch.
no_such_fact() {
    if [ -z "$2" ] && ! grep -q "^$1 = " "$FACTS_FILE" 2>/dev/null; then
        echo "FAIL line $LINE_NO: no such fact '$1'" >&2
        echo "       known facts:" >&2
        sed 's/^/         /' "$FACTS_FILE" >&2 2>/dev/null
        return 0
    fi
    return 1
}

# Whether `$1 $2 $3` holds for the integer forms. A value that is not an integer never does: a
# fact that reads "none" is not greater than zero, and must not pass because `[` gave up on it.
holds() {
    [[ "$1" =~ ^-?[0-9]+$ ]] || return 1
    case "$2" in
        ">") [ "$1" -gt "$3" ] ;;
        ">=") [ "$1" -ge "$3" ] ;;
        "<") [ "$1" -lt "$3" ] ;;
        *) return 1 ;;
    esac
}

FAILURES=0
LINE_NO=0
while IFS= read -r RAW; do
    LINE_NO=$((LINE_NO + 1))
    # A comment is a line starting with # or a # after whitespace; a # inside a value stays, as
    # in the compass page's SENSOR_ID#1 device text, which is the C#'s own.
    case "$RAW" in \#*) continue ;; esac
    LINE="${RAW%%[[:space:]]#*}"
    # shellcheck disable=SC2086 # deliberate word splitting into positional parameters
    set -- $LINE
    [ $# -eq 0 ] && continue

    case "$1" in
        screen|window|tiles|env|setup) ;;  # already applied before launch
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
        doubleclick)
            # Two `click`s cannot make one: each waits for the probe file to settle and then
            # sleeps, far past the 400 ms a double click must happen in. So the target is resolved
            # once, the pointer moved there, and both presses sent together.
            TARGET="${2:?doubleclick needs a target}"
            if COORDS=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$TARGET"); then
                echo "double-clicking '$TARGET' at window-relative ${COORDS/ /,}"
                # shellcheck disable=SC2086 # "x y", two words on purpose
                xdotool mousemove --window "$WIN_ID" $COORDS
                sleep 0.05
                xdotool click --repeat 2 --delay 80 1
            else
                echo "line $LINE_NO: could not double-click '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.6
            ;;
        hover)
            # The pointer moved onto a control, no button: what a marker's hover shows.
            TARGET="${2:?hover needs a target}"
            if COORDS=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$TARGET"); then
                echo "hovering '$TARGET' at window-relative ${COORDS/ /,}"
                # shellcheck disable=SC2086 # "x y", two words on purpose
                xdotool mousemove --window "$WIN_ID" $COORDS
            else
                echo "line $LINE_NO: could not hover '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.6
            ;;
        scroll)
            # The wheel, N notches, over a control: what brings a drop-down list's later rows into
            # view. Resolved once and the notches sent together, like the double click.
            TARGET="${2:?scroll needs a target}"
            case "${3:-down}" in
                up) WHEEL=4 ;;
                down) WHEEL=5 ;;
                *) echo "line $LINE_NO: scroll wants up or down, not '${3}'" >&2; FAILURES=$((FAILURES + 1)); continue ;;
            esac
            NOTCHES="${4:-1}"
            # Milliseconds between notches: a fifth word, for measuring what the application
            # keeps up with.
            GAP="${5:-60}"
            if COORDS=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$TARGET"); then
                echo "scrolling '$TARGET' ${3:-down} $NOTCHES at window-relative ${COORDS/ /,}"
                # shellcheck disable=SC2086 # "x y", two words on purpose
                xdotool mousemove --window "$WIN_ID" $COORDS
                sleep 0.05
                xdotool click --repeat "$NOTCHES" --delay "$GAP" "$WHEEL"
            else
                echo "line $LINE_NO: could not scroll '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.6
            ;;
        type)
            shift
            xdotool type --window "$WIN_ID" --clearmodifiers --delay 60 "$*"
            # Keystrokes reach the application through the input method when one is running
            # (ibus over XIM here) and come back after a round trip; a click or a key sent next
            # does not wait for them, and has overtaken typed text more than once. Give the
            # text time to land before the next line runs.
            sleep 1
            sleep 0.6
            ;;
        key)
            xdotool key --window "$WIN_ID" --clearmodifiers "${2:?key needs a name}"
            sleep 0.6
            ;;
        restart)
            if ! close_window "$WIN_ID"; then
                echo "FAIL line $LINE_NO: restart could not close the window (it needs python3 with Xlib)" >&2
                FAILURES=$((FAILURES + 1))
            fi
            for _ in $(seq 1 40); do
                kill -0 "$APP_PID" 2>/dev/null || break
                sleep 0.25
            done
            if kill -0 "$APP_PID" 2>/dev/null; then
                echo "FAIL line $LINE_NO: the application did not exit when its window was closed" >&2
                FAILURES=$((FAILURES + 1))
                kill "$APP_PID" 2>/dev/null
            fi
            wait "$APP_PID" 2>/dev/null
            APP_PID=""
            # What the old process published is not what the new one believes.
            : > "$FACTS_FILE"
            : > "$PROBE_FILE"
            echo "restarted after line $((LINE_NO - 1))"
            start_app
            ;;
        expect)
            KEY="${2:?expect needs a key}"
            OP="${3:?expect needs a value}"
            # `expect key value` is equality; `expect key ~ value` is containment; `expect key > n`,
            # `expect key >= n` and `expect key < n` compare integers.
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
            elif [ "$OP" = ">" ] || [ "$OP" = ">=" ] || [ "$OP" = "<" ]; then
                WANT="${4:-}"
                GOT=$(fact "$KEY")
                if ! [[ "$WANT" =~ ^-?[0-9]+$ ]]; then
                    echo "FAIL line $LINE_NO: '$OP' needs an integer to compare with, not '$WANT'" >&2
                    FAILURES=$((FAILURES + 1))
                elif no_such_fact "$KEY" "$GOT"; then
                    FAILURES=$((FAILURES + 1))
                elif holds "$GOT" "$OP" "$WANT"; then
                    echo "  ok   $KEY = $GOT $OP $WANT"
                else
                    echo "FAIL line $LINE_NO: $KEY is '$GOT', expected $OP $WANT" >&2
                    FAILURES=$((FAILURES + 1))
                fi
            else
                shift 2
                WANT="$*"
                GOT=$(fact "$KEY")
                if no_such_fact "$KEY" "$GOT"; then
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
