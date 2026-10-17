#!/usr/bin/env bash
# The user demo (Buzz, 2026-10-17): a 5-10 minute autonomous, fully visible run of the Linux
# planner on this display. dist/planner --demo opens disconnected and its Welcome-Demo-Sitl plugin
# takes over with its drawn pointer - SIMULATION > Multirotor, PLAN, Zoom To Vehicle, Set Home
# Here, four waypoints, a survey, Write, FLY > Actions, force arm, TakeOff, Auto, then it unticks
# itself - and when its pointer goes, tools/demo/tour.py replays the click-everything crawl's
# recipe with the real pointer on the same window for the minutes left. The planner stays open at
# the end, the copter flying.
#
#   tools/demo/run.sh
#
# No arguments: it runs five minutes on this display (DISPLAY) with dist/planner, the owner's word
# of 2026-10-17. Takes the mouse: on the desktop only when its owner says so. A rehearsal runs the
# same command on an Xvfb display (`tools/gui-headless.sh` makes :99) with DISPLAY=:99 and, so it
# leaves the machine's settings, tiles and plugin choices alone, XDG_DATA_HOME, MP_CONFIG_XML and
# MP_SETTINGS set to a scratch directory of its own in the environment; the demo proper uses the
# owner's. The demo's SITL is the SIMULATION tab's, tcp 5760, as Mission Planner's is.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MINUTES=5; BINARY="$ROOT/dist/planner"; SHOW="${DISPLAY:-:0}"; DWELL=0.8
[ $# -eq 0 ] || { echo "usage: $0 - no arguments: five minutes on this display with dist/planner" >&2; exit 2; }
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
echo "demo: $BINARY --demo on $DISPLAY, $MINUTES min, facts in $WORK"
"$BINARY" --demo --window 1600x1200 > "$WORK/planner.log" 2>&1 &
PID=$!
fact() { grep "^$1 = " "$MP_FACTS" 2>/dev/null | head -1 | cut -d= -f2- | sed 's/^ //'; }
# The welcome part: its pointer shows within a minute, and goes once it has unticked itself and
# saved - or it stops early, saying why in plugins.status.
for _ in $(seq 1 120); do [ "$(fact demo.pointer)" = shown ] && break; kill -0 $PID 2>/dev/null || { echo "the planner ended: $(tail -3 "$WORK/planner.log")" >&2; exit 1; }; sleep 0.5; done
echo "welcome demo: pointer shown at +$(( $(date +%s) - START )) s"
for _ in $(seq 1 600); do
    if [ "$(fact demo.last)" = plugin-manager-save ] || [ "$(fact demo.pointer)" = hidden ] && [ "$(fact demo.clicks)" != 0 ]; then
        [ "$(fact demo.pointer)" = hidden ] && break
    fi
    kill -0 $PID 2>/dev/null || { echo "the planner ended: $(tail -3 "$WORK/planner.log")" >&2; exit 1; }
    sleep 0.5
done
echo "welcome demo: done at +$(( $(date +%s) - START )) s, $(fact demo.clicks) clicks, last $(fact demo.last); plugins.status: $(fact plugins.status)"
LEFT=$(python3 -c "import sys; print(max(1.0, $MINUTES - ($(date +%s) - $START) / 60))")
python3 "$HERE/tour.py" --display "$DISPLAY" --pid $PID --probe "$MP_PROBE" --facts "$MP_FACTS" --minutes "$LEFT" --dwell "$DWELL" --log "$WORK/tour.log"
echo "demo over at +$(( $(date +%s) - START )) s; the planner (pid $PID) stays open; log in $WORK"
