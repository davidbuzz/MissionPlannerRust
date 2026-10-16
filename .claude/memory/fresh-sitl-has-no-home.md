---
name: fresh-sitl-has-no-home
description: A freshly started SITL has no home for 60-90 s (its EKF origin and fix come first), so scripts that expect the vehicle's home (plan-home-marker-vehicle, plan-tracker-home) fail on a fresh own SITL and pass on a settled one - wait, or run them late in a batch
metadata:
  type: project
---

On 2026-10-16 `plan-home-marker-vehicle.gui` and `plan-tracker-home.gui` failed every batch run on
my own fresh SITL (instance 3, tcp:5790) with `vehicle.home` none, and a probe showed home still
unknown 70 s after the SITL started; the same script passed unchanged at about 90 s. ArduPilot's
SITL sets home only once its EKF has an origin and a 3D fix, which on this loaded machine takes
60-90 s at speedup 1; the suite's long-running SITL never shows it. Two afternoons were spent
suspecting the day's code before the probe settled it.

**How to apply:** a script that reads the vehicle's home is run on a SITL at least 90 s old - late
in a batch, or after a wait; a failing `vehicle.home`/`plan.home` on a fresh SITL is the simulator,
not the port, until a settled run says otherwise. The planner's default home in those runs is the
config's -27.5097083, 153.0153894. Related: [[test-sitls-own-their-ports]],
[[signing-script-keys-the-sitl]].

**Revised 2026-10-17:** the age of the SITL is not the whole story. In three batch runs on my own
SITL (instance 3) plan-tracker-home.gui found `vehicle.home` none at 5, 20 and 25 minutes of SITL
life, while the suite's own order passes it. The difference is what ran before: the suite runs
experimental-takeoff.gui (which arms and flies the copter) and fly-messages.gui before plan-*, my
batches left both out as fresh-SITL-dependent. Likely home reaches the planner only once the
vehicle has armed (ArduPilot sets home on arming) or when the mission is read (the C# asks
GET_HOME_POSITION in getWPs; mp-link's requests.rs:683 has the command) - to be checked, not
assumed. Until then: run plan-tracker-home after a script that arms the SITL, or read the mission
first; and never read a failing `vehicle.home` as the day's code without that check.

