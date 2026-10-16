---
name: test-sitls-own-their-ports
description: Buzz runs his own SITL (SIMULATION tab, tcp 5760) and planner (UDP 14550) while I test; my harness drove his SITL for an hour on 2026-10-14 - a test SITL must own its port, crawls deny port-opening buttons; the planner reopens config.xml's remembered link at start, so the runner strips comport from its config copy (2026-10-16)
metadata:
  type: feedback
---

**What happened (2026-10-14, 21:09-22:11 local):** my first click audit started its SITL with
`tools/sitl/run-sitl.sh copter` (instance 0, tcp 5760). Buzz's own SITL - started from his
planner's SIMULATION tab, printing to his terminal - already held 5760, so mine failed to bind
and SITL only logged it. The audit's "connected" planners then connected to *his* SITL 317 times
(his terminal's "New connection on SERIAL0 / Closed connection on SERIAL0", which he reported as
a bug) and clicked through FLY and SETUP on it: 373 parameter writes - FRAME_CLASS 7,
COMPASS_USE/2/3 0, ESC_CALIBRATION 3, SERVO1-16_REVERSED 1, CHUTE_ENABLED 1, ARSPD_ENABLE 1. His
next SITL starts hung in ESC calibration, and his Arm and Takeoff test failed (no compass, so
Guided "requires position"). He wiped it ("done ok. no problem"). Found by the SERIAL0 agent from
his SITL's dataflash logs and tlogs.

**Why:** a test that connects to "its" port without proving the listener is its own process
drives whatever holds the port. Separately, a crawler clicking every button presses Connect/Go
in output windows: the MAVLink Mirror's default would forward a SITL's stream to UDP 14550, where
his planner listens; NMEA/CoT/RTK/SiK/DroneCAN open serial devices or network hosts.

**How to apply:**
- Never use instances 0/1 (5760/5770) for my SITLs; the crawl uses 7 and 8, my scripts 3.
  `run-sitl.sh` now refuses a port in use (exit 3); `start-sitl.sh` and `tools/gui-crawl/run.sh`
  check the listener's pid is the SITL they started; scratch scripts source an `own_sitl` helper
  that does the same. Check `ss -ltnp` before any SITL work.
- The crawler's DENY list covers every button that opens a port of its own, and its planners run
  with `MP_OFFLINE=1` and `MP_FIRMWARE_DEVICE` set; `tools/gui-crawl/selftest.sh` checks both
  guards first. Extend DENY when a new output window appears.
- His `./dist/planner` and SITL may be running at any time: never kill, connect to or reconfigure
  them. See [[kill-by-pid-not-pattern]], [[gui-tests-take-the-mouse]], [[bench-cubeorange]].

**Found 2026-10-16:** the planner reopens the last link at start (main.rs's `target` falls back to
`Settings::load()`'s `link`, which `with_mission_planner_defaults` fills from config.xml's
`comport` and `<comport>_BAUD`). `tools/gui-test.sh` hands a script without `MP_CONFIG_XML` or
`XDG_DATA_HOME` a copy of Buzz's config.xml, whose comport is his bench CubeOrange's by-id path:
eleven scripts with no link of their own (logs-dialogs among them) opened his board over USB at
every run, and `vehicle.connected` was true with a serial target no script had asked for.
logs-dialogs failed on it (a loaded log is name-only while a port is open, FlightData.cs:3553-3567)
and tiles-startup failed on his remembered zoom 20. The runner now strips `<comport>` from the
copy; a test's link is the one after `--`, or none. A script that needs a known zoom or port
sets `env MP_CONFIG_XML $WORK/config.xml`. If his comport were TCP, the fallback would have been
his SITL on 5760 - the same hazard by another road.

