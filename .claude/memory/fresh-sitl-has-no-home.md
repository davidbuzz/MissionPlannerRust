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
