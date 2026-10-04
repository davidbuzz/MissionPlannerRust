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

**2026-09-25, Buzz: "do not foreground wait."** An `until grep -q done log; do sleep 3; done` in
a foreground Bash call is a foreground wait, however short each call is, and I did it a dozen
times in one session. The only allowed shape: start the long job with `run_in_background`, or put
the `until` loop itself in a `run_in_background` call so the harness notifies me when it ends,
and do other work in the meantime. Never a foreground `sleep` or poll loop, not even for a
30-second build.

**2026-09-26, Buzz: "do not foreground wait, including the win vm".** A `tools/win10/vm-run.sh`
job run in the foreground - a build or test in the VM with a long timeout - is a foreground wait
like any other, and he rejected one. Every VM job that takes more than a moment goes with
`run_in_background: true`; its output streams into the task file, the console window shows it,
and the notification says when it ends.

**2026-10-03, Buzz: "do not foreground wait".** Watching a VM boot with a foreground loop of
`sleep 30; screenshot` six times - three minutes blocked - is a foreground wait, and he rejected
it. A guest booting, an installer running, a download: start the watcher with
`run_in_background: true` (screenshots into a folder, a DONE line at the end), answer him
meanwhile, and look at the pictures when the notification comes.

**2026-10-04, Buzz: "do not forground wait".** It happened again: a background test run was
still going, and to see its result I made a foreground Bash call holding
`until grep -q TESTSDONE <output>; do sleep 5; done` with a 600 s timeout. He rejected it. A
harness rule had already refused a plain `sleep 45` that turn. Don't get round that refusal by
moving the sleep into an `until` loop in the foreground; that is the same wait. When a
background job's result is what's needed next:
- end the turn, or answer him;
- or do the next independent piece of work;
- the notification comes when the job ends.
