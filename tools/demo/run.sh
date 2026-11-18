#!/usr/bin/env bash
# The user demo (Buzz, 2026-10-17), as the list it is since 2026-11-18: dist/planner --demo opens
# disconnected and plays tools/demo/owner.gui, a step a line, with its drawn pointer - SIMULATION >
# Multirotor, PLAN, Zoom To Vehicle, Set Home Here, four waypoints, a survey, Write, FLY > Actions,
# force arm, TakeOff, Auto, and then whatever the list goes on to. The list is the whole demo and
# the planner plays it by itself: this script only makes the way clear for it (an earlier demo's
# planner, a SITL left on tcp:5760), starts it, and says how it ended. The planner stays open at
# the end, the copter flying.
#
#   tools/demo/run.sh
#
# No arguments: this display (DISPLAY) with dist/planner, the owner's word of 2026-10-17. The
# pointer is drawn by the planner; the real mouse is not taken. A rehearsal runs the same command
# on an Xvfb display (`tools/gui-headless.sh` makes :99) with DISPLAY=:99 and, so it leaves the
# machine's settings, tiles and plugin choices alone, XDG_DATA_HOME, MP_CONFIG_XML and MP_SETTINGS
# set to a scratch directory of its own in the environment; the demo proper uses the owner's. The
# demo's SITL is the SIMULATION tab's, tcp 5760, as Mission Planner's is.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
BINARY="$ROOT/dist/planner"; SHOW="${DISPLAY:-:0}"
[ $# -eq 0 ] || { echo "usage: $0 - no arguments: the demo on this display with dist/planner" >&2; exit 2; }
[ -x "$BINARY" ] || { echo "no planner at $BINARY" >&2; exit 1; }
# An earlier demo's planner still open - this binary with --demo - is stopped first, by its pid.
for pid in $(pgrep -f "^$BINARY --demo"); do
    echo "stopping the earlier demo planner (pid $pid)"
    kill "$pid" 2>/dev/null
done
# A simulator left on tcp:5760 - the demo's own from a run that ended badly, or any ArduPilot SITL
# - is stopped, by its pid (the owner's word of 2026-10-17: "needs to kill any sitl process it
# finds"); anything else holding the port, not a SITL, stops the demo instead.
for pid in $(ss -ltnpH "sport = :5760" | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u); do
    comm=$(cat "/proc/$pid/comm" 2>/dev/null || echo unknown)
    case "$comm" in
        [Aa]rdu*|AntennaTracker*|[Bb]limp*|SITL*|sitl*)
            echo "tcp:5760: stopping the SITL left there ($comm, pid $pid)"
            kill "$pid" 2>/dev/null ;;
        *)
            echo "tcp:5760 is held by $comm (pid $pid), not a SITL: the demo's SITL needs it; stop that first" >&2
            exit 3 ;;
    esac
done
for _ in $(seq 1 20); do ss -ltnH "sport = :5760" | grep -q . || break; sleep 0.5; done
if ss -ltnH "sport = :5760" | grep -q .; then
    echo "tcp:5760 is still held after stopping its SITL: stop that first" >&2
    exit 3
fi
WORK="$(mktemp -d -t planner-demo-XXXXXX)"
export DISPLAY="$SHOW"
export MP_PROBE="$WORK/probe.json" MP_FACTS="$WORK/facts.txt"
# A rehearsal names a scratch config in the environment; one that does not exist yet is made with
# the simulator's stable release chosen, as the SITL scripts choose it.
if [ -n "${MP_CONFIG_XML:-}" ] && [ ! -f "$MP_CONFIG_XML" ]; then
    mkdir -p "$(dirname "$MP_CONFIG_XML")"
    printf '<?xml version="1.0" encoding="utf-8"?><Config><sitl_download_version>2</sitl_download_version></Config>' > "$MP_CONFIG_XML"
fi
START=$(date +%s)
echo "demo: $BINARY --demo on $DISPLAY, facts in $WORK"
# The list is read from where the planner is started: the repository, so an edit to
# tools/demo/owner.gui is what plays.
cd "$ROOT"
"$BINARY" --demo --window 1600x1200 > "$WORK/planner.log" 2>&1 &
PID=$!
fact() { grep "^$1 = " "$MP_FACTS" 2>/dev/null | head -1 | cut -d= -f2- | sed 's/^ //'; }
# The list plays: its pointer shows within a minute, and demo.script says the line under way,
# then "done" - or "stopped: ..." with the line that did not come and why.
for _ in $(seq 1 120); do
    [ "$(fact demo.pointer)" = shown ] && break
    kill -0 $PID 2>/dev/null || { echo "the planner ended: $(tail -3 "$WORK/planner.log")" >&2; exit 1; }
    sleep 0.5
done
echo "demo: pointer shown at +$(( $(date +%s) - START )) s"
while :; do
    case "$(fact demo.script)" in
        done) echo "demo: the list played to its end at +$(( $(date +%s) - START )) s, $(fact demo.clicks) clicks"; break ;;
        stopped*) echo "demo: $(fact demo.script)" >&2; break ;;
    esac
    kill -0 $PID 2>/dev/null || { echo "the planner ended: $(tail -3 "$WORK/planner.log")" >&2; exit 1; }
    sleep 1
done
echo "the planner (pid $PID) stays open; its log and facts are in $WORK"
