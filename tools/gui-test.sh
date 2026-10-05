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
#   tiles on                    configure a tile source and fetch what the cache lacks
#   env MP_TILE_CACHE $WORK/c   export a variable before launch; the rest of the line is the value
#   setup tools/seed $WORK/c    run a command, from the repo root, before launch
#   budget 5                    the test's expected run time in seconds, from its window to its
#                               last line; 5 unless the script says otherwise (MP_GUI_BUDGET
#                               overrides the default). Over it is a failure, and the hard stop
#                               is three seconds past it: the owner's rules of 2026-09-25,
#                               "each individual GUI test is fully completed within 5 seconds"
#                               and "keep a record of the expected run time of each UI test, and
#                               terminate after expected time + 3 seconds". tools/gui-budgets.py
#                               `record` writes the times a suite run measured, `bump` adds a
#                               second to every script a run found over its budget ("increase
#                               budget by 1 sec for all the ones that missed"), `retime` sets the
#                               budget from a run that failed on its time alone. Every line is stamped
#                               `t=+1.234s` on the way, so the log says where the time went.
#   (hard stop)                 a test still running its budget plus MP_GUI_HARD_STOP_MARGIN
#                               seconds (3) after its window has a screenshot
#                               of its window taken - <name>-hardstop.png in MP_GUI_SHOT_DIR,
#                               /tmp by default; the suite puts it beside the logs - and is
#                               killed, a failure (the owner's request of 2026-09-25)
#   allow-hidden                the run may end with a control cut off (every run otherwise
#                               fails when the application's layout.hidden fact is not 0 at its
#                               end: crates/mp-gui/src/layout_guard.rs, the owner's self-test of
#                               2026-10-03, mandatory for every control since 2026-10-04 - one
#                               clipped by the window or a box above it, but a scrolling list's
#                               row, fails the script that left it so)
#   within 45                   the next expect may wait this many seconds for its fact, for
#                               the few things slower than MP_GUI_EXPECT_WAIT (a page's partial
#                               refresh reads its parameters back one by one)
#   settle 6                    wait, for telemetry to arrive or a view to settle - rarely
#                               needed now: `expect` waits for its fact (up to
#                               MP_GUI_EXPECT_WAIT seconds, 10 by default) and a click waits
#                               for its control, so a test takes the time the application
#                               takes and no more (the owner's rule of 2026-09-25: the
#                               smallest waits that pass)
#   click map@0.45x0.40         click a named control, as tools/gui-click.sh addresses them
#   click tab-plan:right        a right-click
#   doubleclick log-chart@0.5x0.5  a double click: two left presses at one point, 80 ms apart
#   scroll servo-SERVO9_FUNCTION-list down 3 [ms]  the wheel over a control: up or down, N notches, a gap between them
#   hover map@0.40x0.40          move the pointer onto a control and press nothing
#   drag fft-INS_LOG_BAT_CNT-thumb fft-INS_LOG_BAT_CNT-track@0.5x0.5  press on the first control,
#                               move to the second in steps, let go: a track bar's thumb dragged
#   reveal servo-SERVO9_FUNCTION-list servo-SERVO9_FUNCTION-1   wheel a list until an entry is inside its box
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
# a silent pass - a test asserting on a fact that no longer exists must fail, not succeed. With
# MP_FACTS_KEEP=<file> the run's last facts are copied there at the end, for numbers a script
# measures rather than asserts (tests/gui/perf-*.gui, crates/mp-gui/src/frametimes.rs).
set -uo pipefail
# Script words are text: `set -- $LINE` must not turn `MAV[0]` into a file glob.
set -f

SCRIPT="${1:?usage: gui-test.sh <script.gui> [-- <binary args>]}"
shift
[ "${1:-}" = "--" ] && shift

[ -r "$SCRIPT" ] || { echo "cannot read $SCRIPT" >&2; exit 2; }

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# The debug binary, as every script runs against; MP_GUI_BIN names another, such as a release
# build for a measurement whose number the debug build cannot stand for (tests/gui/storm.gui).
BIN="${MP_GUI_BIN:-$ROOT/target/debug/planner}"
WINDOW_TITLE="MissionPlannerRust"
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
WATCHDOG=""
PROBE_FILE=""
FACTS_FILE=""
SETTINGS_FILE=""
POINTER_HOME=""
WORK="$(mktemp -d -t planner-work-XXXXXX)" || { echo "cannot make a scratch directory" >&2; exit 2; }
cleanup() {
    [ -n "${WATCHDOG:-}" ] && kill "$WATCHDOG" 2>/dev/null
    if [ -n "$APP_PID" ]; then
        kill "$APP_PID" 2>/dev/null
        wait "$APP_PID" 2>/dev/null
    fi
    [ -n "$PROBE_FILE" ] && rm -f "$PROBE_FILE"
    [ -n "$FACTS_FILE" ] && [ -n "${MP_FACTS_KEEP:-}" ] && cp "$FACTS_FILE" "$MP_FACTS_KEEP"
    [ -n "$FACTS_FILE" ] && rm -f "$FACTS_FILE" "${FACTS_FILE%.conf}.facts.tmp"
    [ -n "$SETTINGS_FILE" ] && rm -f "$SETTINGS_FILE" "${SETTINGS_FILE%.conf}.tmp"
    rm -rf "$WORK"
    # shellcheck disable=SC2086 # two words on purpose
    [ -n "$POINTER_HOME" ] && xdotool mousemove $POINTER_HOME 2>/dev/null
}
# A signal ends the run: a trap that only cleaned up would hand control back to the script,
# which then went on past the hard stop, expecting against a closed application (2026-09-25).
# An expect in progress - the usual thing a hard stop lands on, as one waits up to ten seconds
# and the stop comes three past the budget - says what it found first, or the log would only
# say "hard stop".
EXPECT_WORDS=()
# What the last expect read, so a fact that held is reported without a second read.
LAST_VALUE=""
on_signal() {
    if [ "${#EXPECT_WORDS[@]}" -gt 0 ]; then
        QUIET=""
        expect_once "${EXPECT_WORDS[@]}" || true
    fi
    exit 124
}
trap cleanup EXIT
trap on_signal INT TERM HUP

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
# The time a test may take, from its window to its last line: 5 s, the owner's rule.
TEST_BUDGET_SECONDS="${MP_GUI_BUDGET:-5}"
# How long an `expect` waits for its fact, and a click for its control, before it is a failure.
EXPECT_WAIT_MS=$(awk -v s="${MP_GUI_EXPECT_WAIT:-10}" 'BEGIN { printf "%d", s * 1000 }')
# Set while an `expect` is asked quietly, in its wait loop.
QUIET=""
# A `within N` line: the wait for the next expect alone, in ms; empty otherwise.
NEXT_WAIT_MS=""
now_ms() { date +%s%3N; }
# Milliseconds as "1.234".
seconds() { printf '%d.%03d' $(($1 / 1000)) $(($1 % 1000)); }
# Waits until the application has drawn $1 frames since the input sent last, up to a second.
# What a key or typed text needs before it is sent: the box a click opened is drawn on the
# next frame and takes the focus on the one after, and keys sent before that go nowhere
# (config-battery2.gui, 2026-09-25, once the pauses after clicks were short). The frames are
# counted by the `ui.frame` fact, one a frame, so frames drawn already are not waited for again:
# an application that draws only when something changes - repaint on new data, the planner's
# default since 2026-10-05 - draws no more till it does, and waiting for new writes of the file
# waited out its one-a-second floor before every click and key (2026-10-06). Before any input,
# or from an application without the count, $1 new writes of the facts file.
INPUT_FRAME=""
wait_publishes() {
    local seen=0 last now frame
    if [[ "$INPUT_FRAME" =~ ^[0-9]+$ ]]; then
        for _ in $(seq 1 20); do
            frame=$(fact ui.frame)
            if [[ "$frame" =~ ^[0-9]+$ ]] && [ "$frame" -ge $((INPUT_FRAME + $1)) ]; then
                return 0
            fi
            sleep 0.05
        done
        return 0
    fi
    last=$(stat -c '%.9Y' "$FACTS_FILE" 2>/dev/null || echo "")
    for _ in $(seq 1 20); do
        sleep 0.05
        now=$(stat -c '%.9Y' "$FACTS_FILE" 2>/dev/null || echo "")
        if [ "$now" != "$last" ]; then
            last="$now"
            seen=$((seen + 1))
            [ "$seen" -ge "$1" ] && return 0
        fi
    done
}
# Notes the frame an input goes in, for `wait_publishes`: read just before the input is sent (a
# click's after the pointer has moved and paused), since the frame that shows it can be drawn
# before a read after it - and then the next step waited for a frame that was not coming.
sent_input() {
    INPUT_FRAME=$(fact ui.frame || true)
}
SCAN_NO=0

# A line may be for one platform: `linux: expect ...` runs here without its prefix, and
# `windows:` or `macos:` lines are for the other runners - where the C# itself differs by
# platform (the SITL page's launcher, the line ends a file is written with). Prints the line to
# run, or fails for a line that is not this platform's.
for_this_platform() {
    case "$1" in
        linux:*) printf '%s\n' "${1#linux:}" ;;
        windows:*|macos:*) return 1 ;;
        *) printf '%s\n' "$1" ;;
    esac
}

# Read the directives that must be set before the application starts.
while IFS= read -r LINE; do
    SCAN_NO=$((SCAN_NO + 1))
    LINE="${LINE%%#*}"
    LINE=$(for_this_platform "$LINE") || continue
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
        budget)
            [[ "${2:-}" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "line $SCAN_NO: budget wants seconds, not '${2:-}'" >&2; exit 2; }
            TEST_BUDGET_SECONDS="$2"
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

PROBE_FILE="$(mktemp -t planner-probe-XXXXXX.json)"
FACTS_FILE="$(mktemp -t planner-facts-XXXXXX.conf)"
export MP_PROBE="$PROBE_FILE"
export MP_FACTS="$FACTS_FILE"
# The application's built-in plugins stay out of a script that does not ask for them with
# `env MP_BUILTIN_PLUGINS 1`: the Drone ID plugin's first-start question would otherwise sit
# over every script's first clicks (2026-10-05, fifteen scripts failing behind it).
export MP_BUILTIN_PLUGINS=0

# A settings file of its own, per run.
#
# The application remembers things on purpose - the last link, the window size, the altitude frame
# new waypoints get. A test that inherits them is a test whose result depends on what the last
# test did: the altitude-frame test passed, wrote "terrain" to the real settings file, and the
# next run of the same test started in terrain and failed its first expectation. Tests that must
# be run in a particular order, once, are not tests.
SETTINGS_FILE="$(mktemp -t planner-settings-XXXXXX.conf)"
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
# application on the machine running it are never rewritten by a test. The directory is found as
# the application finds it: the XDG data one, never ~/MissionPlannerRust (PLAN.md section 12, D11).
if [ -z "${MP_CONFIG_XML:-}" ]; then
    DATA_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/MissionPlannerRust"
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
# Finds the application's window: the one owned by the process this script started, with our
# title (MissionPlannerRust since 2026-10-03; until then it was Mission Planner's own, and a test
# that drives a real Mission Planner instead of us is worse than a test that does not run, so the
# owner check stays). Tries for up to `$1` half-seconds.
find_window() {
    WIN_ID=""
    for _ in $(seq 1 "${1:-60}"); do
        for CANDIDATE in $(xdotool search --pid "$APP_PID" --onlyvisible --name "$WINDOW_TITLE" 2>/dev/null); do
            OWNER=$(xdotool getwindowpid "$CANDIDATE" 2>/dev/null)
            [ "$OWNER" = "$APP_PID" ] && xwininfo -id "$CANDIDATE" >/dev/null 2>&1 && WIN_ID="$CANDIDATE"
        done
        [ -n "$WIN_ID" ] && break
        kill -0 $APP_PID 2>/dev/null || { echo "app exited before showing a window" >&2; exit 1; }
        sleep 0.5
    done
}

# A control resolved with patience: the probe is written a frame after the layout that put the
# control there, so a click straight after the action that made it would miss. Tries again
# every 50 ms up to EXPECT_WAIT_MS, and prints what it waited when it waited at all.
resolve_control() {
    local started coords waited
    started=$(now_ms)
    coords=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$1") || return 1
    waited=$(($(now_ms) - started))
    [ "$waited" -gt 150 ] && echo "       ($1 resolved after $(seconds $waited) s)"
    echo "$coords"
}

# Before a step drives the window: the id it holds must still be a window. Once in sixty runs an
# id went stale between steps (xdotool answered BadWindow and the click landed nowhere, so a
# script clicked into the wrong screen); the window is found again by the application's pid.
ensure_window() {
    if ! xwininfo -id "$WIN_ID" >/dev/null 2>&1; then
        echo "window $WIN_ID is gone; finding the application's window again" >&2
        find_window 20
        [ -n "$WIN_ID" ] || { echo "line $LINE_NO: the application has no window" >&2; exit 1; }
    fi
}

start_app() {
    "$BIN" "${APP_ARGS[@]}" </dev/null &
    APP_PID=$!
    find_window 60
    [ -n "$WIN_ID" ] || { echo "no window owned by pid $APP_PID appeared" >&2; exit 1; }

    sleep 0.3
    xdotool windowactivate --sync "$WIN_ID" 2>/dev/null
    xdotool windowmove "$WIN_ID" "${SHOT_AT%%,*}" "${SHOT_AT##*,}" 2>/dev/null
    sleep 0.2
    # The application's first tick, after its first frame, loads the SETUP and CONFIG lists and
    # the settings' view; a click before it selects a page that the load then forgets
    # (config-advanced.gui, 2026-09-25, once its 3-second settle was gone). So wait for the
    # facts to be published twice - a second write after the first - up to three seconds.
    local first now
    for _ in $(seq 1 60); do
        [ -s "$FACTS_FILE" ] && break
        sleep 0.05
    done
    first=$(stat -c '%.9Y' "$FACTS_FILE" 2>/dev/null || echo "")
    for _ in $(seq 1 60); do
        sleep 0.05
        now=$(stat -c '%.9Y' "$FACTS_FILE" 2>/dev/null || echo "")
        [ -n "$first" ] && [ "$now" != "$first" ] && break
    done
    # And the layout: the probe is rewritten as the screen's controls appear over the first
    # frames, and a map clicked before its view is there records nothing (fly-poi.gui,
    # 2026-09-25). Wait until the probe has been still for 300 ms, up to three seconds.
    local stamp="" same=0
    for _ in $(seq 1 30); do
        now=$(stat -c '%.9Y' "$PROBE_FILE" 2>/dev/null || echo "")
        if [ -n "$stamp" ] && [ "$now" = "$stamp" ]; then
            same=$((same + 1))
            [ "$same" -ge 3 ] && break
        else
            same=0
        fi
        stamp="$now"
        sleep 0.1
    done
    # The window this run drives, for the watchdog's screenshot: a `restart` makes a new one
    # after the watchdog has forked with the old id.
    printf '%s\n' "$WIN_ID" > "$WORK/window"
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

T_LAUNCH=$(now_ms)
start_app
T0=$(now_ms)
echo "window after $(seconds $((T0 - T_LAUNCH))) s; budget $TEST_BUDGET_SECONDS s from here"

# The application's window on top before every click, key and typed text. Another window
# raised over it in the meantime - the owner's editor on the same display took a run's clicks
# for twenty seconds (setup-list.gui, 2026-09-26, seen in the hard stop's screenshot) - would
# take them instead; xdotool sends the pointer to the window's coordinates, not to it. Raised,
# not activated: activating it before each keystroke lost the typed text of a box that had
# just been clicked (config-battery.gui, 2026-09-26), where the run's one activation at its
# start had not. Keys and text go to the window by id whichever window has the focus.
raise_window() {
    xdotool windowraise "$WIN_ID" 2>/dev/null || true
}

# The window as it is, for the hard stop: what the test was looking at when it ran out of time.
window_shot() {
    local out X=0 Y=0 WIDTH=0 HEIGHT=0 win
    win=$(cat "$WORK/window" 2>/dev/null || echo "$WIN_ID")
    eval "$(xdotool getwindowgeometry --shell "$win" 2>/dev/null | grep -E '^(X|Y|WIDTH|HEIGHT)=')"
    [ "$X" -lt 0 ] && X=0
    [ "$Y" -lt 0 ] && Y=0
    [ "$WIDTH" -gt 0 ] || return 1
    out="${MP_GUI_SHOT_DIR:-/tmp}/$(basename "$SCRIPT" .gui)-hardstop.png"
    if timeout 5 ffmpeg -loglevel error -y -f x11grab -video_size "${WIDTH}x${HEIGHT}" \
        -i "${DISPLAY:-:0}+${X},${Y}" -frames:v 1 "$out" 2>/dev/null; then
        echo "screenshot: $out"
    fi
}

# The hard stop: a watchdog that, this many seconds after the window, takes the screenshot,
# says so and kills this run (cleanup then closes the application). The budget marks a test
# that took too long; this one stops a test that would not end.
HARD_STOP_S=$(awk -v b="$TEST_BUDGET_SECONDS" -v m="${MP_GUI_HARD_STOP_MARGIN:-3}" 'BEGIN { printf "%d", b + m }')
(
    # The sleep holds none of this run's pipes: a caller reading them (`| tail`, `$(...)`)
    # waited the whole budget after a pass while the sleep still had them open.
    sleep "$HARD_STOP_S" </dev/null >/dev/null 2>&1
    echo "FAIL: hard stop - $(basename "$SCRIPT") was still running $HARD_STOP_S s after its window" >&2
    window_shot >&2
    # The trap runs when the script's current command ends, so a click's poll for its control
    # (up to ten seconds) is ended too - by PID, each child of the script but this subshell.
    for CHILD in $(pgrep -P $$); do
        [ "$CHILD" = "$BASHPID" ] || kill -TERM "$CHILD" 2>/dev/null
    done
    kill -TERM $$ 2>/dev/null
) &
WATCHDOG=$!

# Reads one fact. Empty if the key is absent, which `expect` reports as a failure rather than
# comparing against nothing.
# A key is text, not a pattern: its dots are escaped, or `video.device` would also match
# `video_device` (which sorts after it and won the `tail`) - a video script failed on that on
# 2026-09-25 while the page was right.
fact_key_pattern() {
    printf '%s' "$1" | sed 's/[][\.*^$]/\\&/g'
}

fact() {
    [ -s "$FACTS_FILE" ] || return 1
    sed -n "s/^$(fact_key_pattern "$1") = \(.*\)$/\1/p" "$FACTS_FILE" | tail -1
}

# Reports a fact the application never published. Returns 0 - "yes, it is missing" - after
# printing what was published instead, so a typo in a key reads as one rather than as a mismatch.
no_such_fact() {
    if [ -z "$2" ] && ! grep -q "^$(fact_key_pattern "$1") = " "$FACTS_FILE" 2>/dev/null; then
        echo "FAIL line $LINE_NO: no such fact '$1'" >&2
        echo "       known facts:" >&2
        sed 's/^/         /' "$FACTS_FILE" >&2 2>/dev/null
        return 0
    fi
    return 1
}

# Whether `$1 $2 $3` holds for the numbers. A value that is not a number never does: a fact that
# reads "none" is not greater than zero, and must not pass because the comparison gave up on it.
# Decimals compare as numbers (a Quick view altitude reads "584.00"), so awk does the comparing.
holds() {
    [[ "$1" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || return 1
    [[ "$3" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || return 1
    case "$2" in
        ">") awk -v a="$1" -v b="$3" 'BEGIN { exit !(a + 0 > b + 0) }' ;;
        ">=") awk -v a="$1" -v b="$3" 'BEGIN { exit !(a + 0 >= b + 0) }' ;;
        "<") awk -v a="$1" -v b="$3" 'BEGIN { exit !(a + 0 < b + 0) }' ;;
        *) return 1 ;;
    esac
}

# One check of an `expect` line, its words as the arguments. Prints its verdict and returns 0
# when the expectation holds, 1 when it does not; under QUIET it prints nothing, so the wait
# loop can ask again. `no_such_fact`'s listing of the known facts is for the final verdict only.
expect_once() {
    local KEY OP WANT GOT OTHER VERDICT
    KEY="${2:?expect needs a key}"
    OP="${3:?expect needs a value}"
    # `expect key value` is equality; `expect key ~ value` is containment; `expect key > n`,
    # `expect key >= n` and `expect key < n` compare numbers, decimals included.
    if [ "$OP" = "~" ]; then
        shift 3
        WANT="$*"
        GOT=$(fact "$KEY")
        LAST_VALUE="$GOT"
        case "$GOT" in
            *"$WANT"*) [ -z "$QUIET" ] && echo "  ok   $KEY contains '$WANT'"; return 0 ;;
        esac
        [ -z "$QUIET" ] && echo "FAIL line $LINE_NO: $KEY is '$GOT', expected to contain '$WANT'" >&2
        return 1
    elif [ "$OP" = ">" ] || [ "$OP" = ">=" ] || [ "$OP" = "<" ]; then
        WANT="${4:-}"
        GOT=$(fact "$KEY")
        LAST_VALUE="$GOT"
        if ! [[ "$WANT" =~ ^-?[0-9]+$ ]]; then
            [ -z "$QUIET" ] && echo "FAIL line $LINE_NO: '$OP' needs an integer to compare with, not '$WANT'" >&2
            return 1
        fi
        if [ -z "$GOT" ] && ! grep -q "^$(fact_key_pattern "$KEY") = " "$FACTS_FILE" 2>/dev/null; then
            [ -z "$QUIET" ] && no_such_fact "$KEY" ""
            return 1
        fi
        if holds "$GOT" "$OP" "$WANT"; then
            [ -z "$QUIET" ] && echo "  ok   $KEY = $GOT $OP $WANT"
            return 0
        fi
        [ -z "$QUIET" ] && echo "FAIL line $LINE_NO: $KEY is '$GOT', expected $OP $WANT" >&2
        return 1
    elif [ "$OP" = "below" ] || [ "$OP" = "above" ] || [ "$OP" = "left-of" ] || [ "$OP" = "right-of" ]; then
        # `expect a below b` (above, left-of, right-of): where two measured controls sit
        # relative to each other, from the positions the application reports.
        OTHER="${4:?expect $OP needs another control}"
        VERDICT=$(python3 - "$PROBE_FILE" "$KEY" "$OTHER" "$OP" <<'PY'
import json, sys
probe = json.load(open(sys.argv[1]))
a, b, op = probe.get(sys.argv[2]), probe.get(sys.argv[3]), sys.argv[4]
if a is None or b is None:
    print("missing " + (sys.argv[2] if a is None else sys.argv[3])); sys.exit(0)
holds = {
    "below": a["y"] >= b["y"] + b["height"],
    "above": a["y"] + a["height"] <= b["y"],
    "left-of": a["x"] + a["width"] <= b["x"],
    "right-of": a["x"] >= b["x"] + b["width"],
}[op]
print("ok" if holds else "no (%s at %d,%d %dx%d; %s at %d,%d %dx%d)" % (
    sys.argv[2], a["x"], a["y"], a["width"], a["height"],
    sys.argv[3], b["x"], b["y"], b["width"], b["height"]))
PY
)
        if [ "$VERDICT" = "ok" ]; then
            [ -z "$QUIET" ] && echo "  ok   $KEY $OP $OTHER"
            return 0
        fi
        [ -z "$QUIET" ] && echo "FAIL line $LINE_NO: $KEY is not $OP $OTHER: $VERDICT" >&2
        return 1
    else
        shift 2
        WANT="$*"
        GOT=$(fact "$KEY")
        LAST_VALUE="$GOT"
        if [ -z "$GOT" ] && ! grep -q "^$(fact_key_pattern "$KEY") = " "$FACTS_FILE" 2>/dev/null; then
            [ -z "$QUIET" ] && no_such_fact "$KEY" ""
            return 1
        fi
        if [ "$GOT" = "$WANT" ]; then
            [ -z "$QUIET" ] && echo "  ok   $KEY = $WANT"
            return 0
        fi
        [ -z "$QUIET" ] && echo "FAIL line $LINE_NO: $KEY is '$GOT', expected '$WANT'" >&2
        return 1
    fi
}

FAILURES=0
LINE_NO=0
while IFS= read -r RAW; do
    LINE_NO=$((LINE_NO + 1))
    # A comment is a line starting with # or a # after whitespace; a # inside a value stays, as
    # in the compass page's SENSOR_ID#1 device text, which is the C#'s own.
    case "$RAW" in \#*) continue ;; esac
    LINE="${RAW%%[[:space:]]#*}"
    # `$WORK` is the scratch directory on every line, as in `env` and `setup`: a path typed into
    # a box or expected back is the run's own, on every platform - not a fixed /tmp, which the
    # Windows planner reads as C:\tmp where the setup wrote to the temporary folder.
    LINE="${LINE//\$WORK/$WORK}"
    LINE=$(for_this_platform "$LINE") || continue
    # shellcheck disable=SC2086 # deliberate word splitting into positional parameters
    set -- $LINE
    [ $# -eq 0 ] && continue
    # Every line stamped with the time since the window appeared, so the log says where a
    # test's seconds went.
    echo "t=+$(seconds $(($(now_ms) - T0)))s line $LINE_NO: $LINE"

    case "$1" in
        click|doubleclick|hover|scroll|reveal|type|key) ensure_window ;;
    esac
    case "$1" in
        screen|window|tiles|env|setup|budget) ;;  # already applied before launch
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
            # A frame first: a box that has just closed is drawn once more, and a click sent in
            # that frame lands on it (config-compass.gui's arrow after its dialog, 2026-09-25).
            wait_publishes 1
            raise_window
            if COORDS=$(resolve_control "$TARGET"); then
                COORDS=$(printf '%s\n' "$COORDS" | tail -1)
                echo "clicking '$TARGET' (button $BUTTON) at window-relative ${COORDS/ /,}"
                # shellcheck disable=SC2086 # "x y", two words on purpose
                xdotool mousemove --window "$WIN_ID" $COORDS
                sleep 0.03
                sent_input
                xdotool click "$BUTTON"
            else
                echo "line $LINE_NO: could not click '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            # A frame for the click to land; what follows waits for its own fact or control.
            sleep 0.1
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
                sent_input
                xdotool click --repeat 2 --delay 80 1
            else
                echo "line $LINE_NO: could not double-click '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.1
            ;;
        reveal)
            # Scrolls a drop-down list until one of its entries lies inside the list's box, a
            # notch at a time from the probe's positions, so a script need not know which row a
            # documentation order puts a value on. Fails when forty notches do not bring it in.
            LIST="${2:?reveal needs a list}"
            ENTRY="${3:?reveal needs an entry}"
            REVEALED=""
            # A list draws only the rows in its box, so an entry above or below it is not in the
            # probe at all: start from the top - the wheel stops there - and walk down. Not when
            # the entry is already in the probe: then the walk starts from where the list is,
            # which spares the 45 notches up (a tree revealed twice took 8 s each, 2026-09-25).
            if ! grep -qF "\"$ENTRY\"" "$PROBE_FILE" 2>/dev/null \
                && COORDS=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$LIST"); then
                # shellcheck disable=SC2086 # "x y", two words on purpose
                xdotool mousemove --window "$WIN_ID" $COORDS
                sleep 0.05
                sent_input
                xdotool click --repeat 45 --delay 60 4
                sleep 0.1
            fi
            for _ in $(seq 1 45); do
                DIRECTION=$(python3 - "$PROBE_FILE" "$LIST" "$ENTRY" <<'PY'
import json, sys
probe = json.load(open(sys.argv[1]))
lst, entry = probe.get(sys.argv[2]), probe.get(sys.argv[3])
if lst is None or entry is None:
    print("missing"); sys.exit(0)
top, bottom = lst["y"], lst["y"] + lst["height"]
# Inside means the centre, where the click lands, is in the box with 3 px to spare: an entry
# whose centre sat on the box's edge was clicked on the border (2026-09-25). Not the whole
# entry - a list's first row starts on its box's edge and would never count.
cy = entry["centre_y"]
if top + 3 <= cy <= bottom - 3:
    print("inside")
else:
    print("down" if cy > bottom - 3 else "up")
PY
)
                case "$DIRECTION" in
                    inside) REVEALED=yes; break ;;
                    # Not drawn yet: it is further down.
                    missing) DIRECTION=down ;;
                esac
                WHEEL=5; [ "$DIRECTION" = up ] && WHEEL=4
                if COORDS=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$LIST"); then
                    # shellcheck disable=SC2086 # "x y", two words on purpose
                    xdotool mousemove --window "$WIN_ID" $COORDS
                    sleep 0.05
                    sent_input
                    xdotool click "$WHEEL"
                    sleep 0.12
                else
                    break
                fi
            done
            if [ -n "$REVEALED" ]; then
                echo "revealed '$ENTRY' in '$LIST'"
            else
                echo "line $LINE_NO: could not reveal '$ENTRY' in '$LIST' ($DIRECTION)" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.1
            ;;
        drag)
            # The left button pressed on one control and let go on another, the pointer moved
            # between them in eight steps, as a hand moves it, so the application sees it travel:
            # a track bar's thumb dragged along its channel.
            FROM="${2:?drag needs a start}"
            TO="${3:?drag needs an end}"
            wait_publishes 1
            raise_window
            if FROM_AT=$(resolve_control "$FROM") && TO_AT=$("$ROOT/tools/gui-click.sh" --resolve "$PROBE_FILE" "$WIN_ID" "$TO"); then
                FROM_AT=$(printf '%s\n' "$FROM_AT" | tail -1)
                echo "dragging '$FROM' at ${FROM_AT/ /,} to '$TO' at ${TO_AT/ /,}"
                read -r X0 Y0 <<<"$FROM_AT"
                read -r X1 Y1 <<<"$TO_AT"
                xdotool mousemove --window "$WIN_ID" "$X0" "$Y0"
                sleep 0.05
                xdotool mousedown 1
                for STEP in 1 2 3 4 5 6 7 8; do
                    sleep 0.03
                    xdotool mousemove --window "$WIN_ID" $(( X0 + (X1 - X0) * STEP / 8 )) $(( Y0 + (Y1 - Y0) * STEP / 8 ))
                done
                sleep 0.05
                sent_input
                xdotool mouseup 1
            else
                echo "line $LINE_NO: could not drag '$FROM' to '$TO'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.1
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
            sleep 0.1
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
                sent_input
                xdotool click --repeat "$NOTCHES" --delay "$GAP" "$WHEEL"
            else
                echo "line $LINE_NO: could not scroll '$TARGET'" >&2
                FAILURES=$((FAILURES + 1))
            fi
            sleep 0.1
            ;;
        type)
            shift
            wait_publishes 2
            raise_window
            sent_input
            xdotool type --window "$WIN_ID" --clearmodifiers --delay 60 "$*"
            # Keystrokes reach the application through the input method when one is running
            # (ibus over XIM here) and come back after a round trip; a click or a key sent next
            # does not wait for them, and has overtaken typed text more than once. Give the
            # text time to land before the next line runs: a second, measured - 0.3 s let a
            # Return overtake "LOW {batv}" on 2026-09-25 - and an `expect` on the field is the
            # sure way when the script has one.
            sleep 1
            sleep 0.1
            ;;
        key)
            wait_publishes 2
            raise_window
            sent_input
            xdotool key --window "$WIN_ID" --clearmodifiers "${2:?key needs a name}"
            sleep 0.1
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
                allow-hidden)
            ALLOW_HIDDEN=1
            ;;
        within)
            [[ "${2:-}" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "line $LINE_NO: within wants seconds, not '${2:-}'" >&2; FAILURES=$((FAILURES + 1)); continue; }
            NEXT_WAIT_MS=$(awk -v s="$2" 'BEGIN { printf "%d", s * 1000 }')
            ;;
        expect)
            # Waits for the fact, up to EXPECT_WAIT_MS (or the `within` before it), checking
            # every 50 ms: the test takes the time the application takes. A fact that never
            # comes fails with what was found.
            T_EXPECT=$(now_ms)
            WAIT_MS="${NEXT_WAIT_MS:-$EXPECT_WAIT_MS}"
            NEXT_WAIT_MS=""
            EXPECT_WORDS=("$@")
            QUIET=1
            HELD=""
            until expect_once "$@"; do
                if [ $(($(now_ms) - T_EXPECT)) -ge "$WAIT_MS" ]; then HELD=1; break; fi
                sleep 0.05
            done
            QUIET=""
            # A fact that held is reported from the read that saw it, not read again: a value
            # that lasts a frame (a box open, a fetch running) could be gone by a second read.
            if [ -n "$HELD" ]; then
                expect_once "$@" || FAILURES=$((FAILURES + 1))
            else
                echo "  ok   $2 = $LAST_VALUE"
            fi
            EXPECT_WORDS=()
            WAITED=$(($(now_ms) - T_EXPECT))
            [ "$WAITED" -gt 150 ] && echo "       (held after $(seconds $WAITED) s)"
            ;;
        *)
            echo "line $LINE_NO: unknown directive '$1'" >&2
            FAILURES=$((FAILURES + 1))
            ;;
    esac
done < "$SCRIPT"

# Nothing cut off at the end: the application judges every named control's clipping from gpui's
# own content mask and counts the ones clipped on the screen showing, but a scrolling list's rows
# (crates/mp-gui/src/layout_guard.rs); a script that leaves one cut off fails, unless it said
# `allow-hidden`. "n/a" is a run without the probe, which has nothing to judge.
if [ -z "${ALLOW_HIDDEN:-}" ]; then
    HIDDEN=$(fact layout.hidden || true)
    if [ -n "$HIDDEN" ] && [ "$HIDDEN" != "0" ] && [ "$HIDDEN" != "n/a" ]; then
        echo "FAIL: $HIDDEN control(s) cut off at the end: $(fact layout.hidden.names)" >&2
        FAILURES=$((FAILURES + 1))
    fi
fi

# Done in time: the watchdog is not needed.
kill "$WATCHDOG" 2>/dev/null
# The budget: the whole test, from its window to its last line, restarts included.
TOTAL_MS=$(($(now_ms) - T0))
BUDGET_MS=$(awk -v s="$TEST_BUDGET_SECONDS" 'BEGIN { printf "%d", s * 1000 }')
echo "took $(seconds $TOTAL_MS) s from its window (launch $(seconds $((T0 - T_LAUNCH))) s)"
if [ "$TOTAL_MS" -gt "$BUDGET_MS" ]; then
    echo "FAIL: $(basename "$SCRIPT") took $(seconds $TOTAL_MS) s; the budget is $TEST_BUDGET_SECONDS s" >&2
    FAILURES=$((FAILURES + 1))
fi

if [ "$FAILURES" -gt 0 ]; then
    echo "$(basename "$SCRIPT"): $FAILURES failure(s)" >&2
    exit 1
fi
echo "$(basename "$SCRIPT"): passed in $(seconds $TOTAL_MS) s"
