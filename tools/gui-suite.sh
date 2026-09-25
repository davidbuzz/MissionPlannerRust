#!/usr/bin/env bash
# Runs GUI scripts one after another, each with the argument its own header names.
#
# A script that needs a link says so in its header, as the command line to run it:
#   tools/gui-test.sh tests/gui/x.gui -- tcp:127.0.0.1:5760
# The runner passes nothing by default, and a SITL script run without its argument starts the
# application idle: params.metadata.documented reads 0, mission.items reads 0, setup.pages is the
# disconnected list - failures that look like a broken link and are only a missing argument. This
# reads the argument out of the header so it is never missed.
#
#   tools/gui-suite.sh [-o logdir] name [name ...]     names without tests/gui/ and .gui
#   tools/gui-suite.sh -o logdir --all                 every script
#
# One line per script: "name: PASS" or "name: FAIL <first failures>", then "suite done <time>".
# Waits for the load average to fall below MAX_LOAD (default 20) before each script; lost
# keystrokes and facts read before a render are what a loaded machine does to these tests.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="${TMPDIR:-/tmp}"
MAX_LOAD="${MAX_LOAD:-20}"
ALL=0
while [ $# -gt 0 ]; do
    case "$1" in
        -o) LOGDIR="$2"; shift 2 ;;
        --all) ALL=1; shift ;;
        *) break ;;
    esac
done
mkdir -p "$LOGDIR"
if [ "$ALL" = 1 ]; then
    set -- $(ls "$ROOT"/tests/gui/*.gui | xargs -n1 basename | sed 's/\.gui$//')
fi
FAILED=0
for NAME in "$@"; do
    SCRIPT="$ROOT/tests/gui/$NAME.gui"
    if [ ! -f "$SCRIPT" ]; then echo "$NAME: no script"; FAILED=$((FAILED+1)); continue; fi
    # The header's command line names the argument; a header that only says which SITL port or
    # which file it needs names it that way.
    ARG=$(grep -m1 -oE "gui-test\.sh tests/gui/$NAME\.gui -- [^ ]+" "$SCRIPT" | awk '{print $NF}')
    # A "Run as" line with no argument is a script that starts without a link on purpose
    # (main-connect needs SITL running and connects to it itself), so the port named elsewhere in
    # its header is not passed to it.
    if [ -z "$ARG" ] && ! grep -qE "gui-test\.sh tests/gui/$NAME\.gui[[:space:]]*$" "$SCRIPT"; then
        ARG=$(grep -m1 -oE "(tcp:127\.0\.0\.1:5760|file:[^ ,]+\.tlog)" "$SCRIPT" | head -1)
    fi
    for _ in $(seq 1 60); do
        LOAD=$(cut -d' ' -f1 /proc/loadavg | cut -d. -f1)
        [ "$LOAD" -lt "$MAX_LOAD" ] && break
        sleep 10
    done
    LOG="$LOGDIR/gui-$NAME.log"
    if [ -n "$ARG" ]; then
        SHOT_AT="${SHOT_AT:-2560,0}" timeout 300 "$ROOT/tools/gui-test.sh" "$SCRIPT" -- "$ARG" > "$LOG" 2>&1
    else
        SHOT_AT="${SHOT_AT:-2560,0}" timeout 300 "$ROOT/tools/gui-test.sh" "$SCRIPT" > "$LOG" 2>&1
    fi
    STATUS=$?
    if [ "$STATUS" = 0 ]; then
        echo "$NAME: PASS"
    else
        FAILED=$((FAILED+1))
        echo "$NAME: FAIL $(grep -m3 -E "^FAIL|not found|exited" "$LOG" | cut -c1-150 | tr '\n' '|')"
    fi
done
echo "suite done $(date -u +%T), $FAILED failed"
[ "$FAILED" = 0 ]
