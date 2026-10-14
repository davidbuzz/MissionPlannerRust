#!/usr/bin/env bash
# The crawler's own test. First its guards, which keep a crawl inside its own planners: every
# button that opens a port of its own - a serial device, a network host, a camera - is on the deny
# list, and run-sitl.sh will not start a simulator on a port something already holds (on
# 2026-10-14 an audit's SITL failed to bind 5760 and its planners drove the owner's own SITL for
# an hour). Then HELP crawled alone, and the verdicts it already knows checked - Show Console
# Window changes itself, Change Log hands a link to the desktop, the welcome text is no control.
# Then the Full Parameter List reached as verify.py reaches a control the screen shows at once:
# its opening warning answered, its search box typed into; and a dimmed control told from a live
# one by its colours. Display :90.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
python3 - "$HERE" <<'PY'
import socket, subprocess, sys
sys.path.insert(0, sys.argv[1])
import crawl
denied = ["nmea-BUT_connect", "cot-BUT_connect", "rtk-BUT_connect", "proxy-BUT_connect",
          "dronecan-connect", "mirror-cell-0-4", "mirror-cell-1-5", "sikradio-BUT_upload",
          "experimental-but_mavserialport", "planner-BUT_videostart", "fw-bootloaderupdate",
          "experimental-but_reboot"]
allowed = ["cot-CMB_serialport", "mirror-cell-0-1", "experimental-BUT_CoT", "help-updatecheck"]
bad = [n for n in denied if not crawl.DENY.search(n)] + [n for n in allowed if crawl.DENY.search(n)]
held = socket.socket()
held.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
held.bind(("127.0.0.1", 5850))
held.listen(1)
sitl = subprocess.run([sys.argv[1] + "/../sitl/run-sitl.sh", "copter", "-I9"],
                      capture_output=True, text=True, timeout=30)
if sitl.returncode != 3:
    bad.append(f"run-sitl.sh started on a port in use (exit {sitl.returncode})")
print("guards: " + ("passed" if not bad else "FAILED: " + ", ".join(bad)))
sys.exit(1 if bad else 0)
PY
python3 "$HERE/crawl.py" selftest --display :90 --screens help --out "$OUT" "$@"
python3 "$HERE/report.py" --out "$OUT" > /dev/null
python3 - "$OUT" <<'PY'
import json, sys
rows = {json.loads(l)["id"]: json.loads(l) for l in open(sys.argv[1] + "/results-selftest.jsonl")}
want = {"help-showconsole": ("changed-itself", "visible"), "help-changelog": ("visible",)}
bad = [f"{k}: {rows.get(k, {}).get('verdict')}, wanted {v}" for k, v in want.items()
       if rows.get(k, {}).get("verdict") not in v]
if rows.get("help-changelog", {}).get("opened", 0) < 1:
    bad.append("help-changelog handed nothing to the desktop")
report = open(sys.argv[1] + "/report.md").read()
if "help-text" in report:
    bad.append("help-text, the welcome text, reported as a dead control")
print("selftest: " + ("passed" if not bad else "FAILED: " + "; ".join(bad)))
sys.exit(1 if bad else 0)
PY
# A box a page opens with is answered on arrival as well as after each step: verify.py replays a
# control the screen shows at once with no step before it, and the Full Parameter List's warning
# left over the screen took every such click (2026-10-15). Its search box, reached so, takes typing.
python3 - "$HERE" "$OUT" "$@" <<'PY'
import sys, time
sys.path.insert(0, sys.argv[1])
import crawl
args = sys.argv[3:]
binary = args[args.index("--binary") + 1] if "--binary" in args else None
crawler = crawl.Crawler("selftest-boxes", ":90", [], sys.argv[2], binary)
app = crawler.launch("params", [])
try:
    left = "raw-param-warning-ok" in app.probe()
    app.click("param-search")
    time.sleep(0.4)
    app.key(crawl.TYPED)
    time.sleep(0.8)
    typed = app.facts().get("params.search")
finally:
    app.close()
bad = []
if left:
    bad.append("the Full Parameter List's warning still over the screen on arrival")
if typed != crawl.TYPED:
    bad.append(f"param-search took no typing: {typed!r}")
print("boxes: " + ("passed" if not bad else "FAILED: " + "; ".join(bad)))
sys.exit(1 if bad else 0)
PY
# A dimmed control is told from a live one by its colours (observe.look): on FLIGHT DATA with no
# vehicle, Force Arm is dimmed and Raw Sensor View is live. The screenshot read BGR as RGB until
# 2026-10-15, and every dimmed control came out a dead one.
python3 - "$HERE" "$OUT" "$@" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
import crawl, observe
args = sys.argv[3:]
binary = args[args.index("--binary") + 1] if "--binary" in args else None
crawler = crawl.Crawler("selftest-look", ":90", [], sys.argv[2], binary)
app = crawler.launch("fly", [("click", "fly-tab-actions")])
try:
    probe = app.probe()
    shot = app.shot()
    looks = {name: observe.look(shot, probe[name]) if name in probe else "missing"
             for name in ("force-arm", "fly-rawsensor")}
finally:
    app.close()
bad = []
if looks["force-arm"] != "disabled":
    bad.append(f"force-arm looks {looks['force-arm']}, not disabled")
if looks["fly-rawsensor"] != "live":
    bad.append(f"fly-rawsensor looks {looks['fly-rawsensor']}, not live")
print("look: " + ("passed" if not bad else "FAILED: " + "; ".join(bad)))
sys.exit(1 if bad else 0)
PY
