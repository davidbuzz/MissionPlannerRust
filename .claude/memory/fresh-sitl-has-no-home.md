---
name: fresh-sitl-has-no-home
description: A vehicle's home reaches the planner only when the vehicle sets it (a fresh SITL, a minute after start) or when the planner asks - in a mission read or write, as the C# does, never at connect: a script that wants home reads the mission first; a settled SITL gives none unasked
metadata:
  type: project
---

**Settled 2026-10-17.** The planner hears HOME_POSITION in two ways only: the vehicle sends one
unasked when its home is set - a fresh SITL does that a minute or so after it starts, once its
EKF has an origin and a fix - and it answers MAV_CMD_GET_HOME_POSITION, which the C# sends only in
a mission read (FlightPlanner.cs:6277) or after a write (:6567) and Set Home Here, never at
connect; mp-link's `get_home_position` is asked from the same places. So:
- a script on a *fresh* SITL sees home arrive by itself at 60-90 s (the passes of 2026-10-16);
- a script on a *settled* SITL - one that set home before the planner connected - never sees it
  unless the script reads or writes the mission (three batch runs on a 5-25 minute old SITL,
  2026-10-17: `vehicle.home` none throughout); why the suite's own order passed it is not
  established and was not re-checked - each script is a fresh planner, so an earlier script's
  arming does not help it;
- the fix is in the script, not the port: plan-tracker-home.gui clicks plan-read first.
**How to apply:** a script wanting `vehicle.home` or `plan.home` reads the mission (or writes
one) and waits on `plan.home`; never wait on home after a bare connect, and never read a missing
home as the day's code. Related: [[test-sitls-own-their-ports]].

