---
name: no-foreground-waiting
description: Never block in the foreground waiting for a condition; background it and keep working
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-22T14:54:55.355Z
---

Do not run foreground wait loops (`until <cond>; do sleep 1; done`, polling for a port to open, a
build to finish, a process to appear). Buzz interrupted one of these mid-run.

**Why:** a foreground wait freezes the session on something that is not work. It also hides the
thing being waited on — when the SITL launch failed, the wait loop would have spun until timeout
instead of surfacing the error.

**How to apply:** launch the thing with `run_in_background: true` and carry on with unrelated work;
poll with a single cheap non-blocking check (`ss -tln | grep -q 5760`, `ls target/debug/x`) rather
than a loop; or use Monitor when repeated notifications are genuinely needed. Read the background
task's output file to diagnose failures instead of waiting for success.
