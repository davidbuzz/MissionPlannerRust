# What was changed in the shipped scripts, and why

The nineteen scripts here are Mission Planner's `Scripts/*.py` at the reference commit, which its
IronPython 2.7 runs. This port's engine is RustPython, Python 3 (PLAN.md §12 D20, the owner's
ruling of 2026-09-25: "use RustPython, and make mods to the example/included scripts to work with
it"). Each change is mechanical and recorded here; nothing else in a script was touched, and the
originals stay in `references/missionplanner/Scripts/`. Every file parses under CPython 3 as well
as under the engine, which `crates/mp-script/tests/stock_scripts.rs` checks by running each one.

## The Python 2 forms changed, 2026-09-25

- `print x` statements became `print(x)` calls, in all their spellings: `print x`, `print'x'`
  with no space (six lines in the two mission scripts), and `print x;` with Python 2's optional
  semicolon (one line each in example2 and example10). A trailing comma, Python 2's "no
  newline", became `end=' '`. A trailing comment on the line stays outside the call.
- `except socket.error, msg:` became `except socket.error as msg:` (example6).
- Two Windows paths in example9, `'C:\Users\michael\...'`, became raw strings: in Python 3 `\U`
  opens a unicode escape, and the file would not parse.

| Script | `print` statements, all now calls | Other changes |
|---|---:|---|
| PARACHUTE LANDING APPROACH.py | 15 | |
| TAKEOFF.py | 13 | |
| cubeorange.py | 2 | |
| datetime.py | 1 | |
| debugenv.py | 9 | |
| example1.py | 6 | |
| example10.py | 7 | |
| example2.py | 3 | |
| example3.py | 10 | |
| example4 wp.py | 8 | |
| example5 inject data.py | 1 | |
| example6.py | 11 | one `except ..., msg:` |
| example7.py | 1 | |
| example8 - speech.py | 2 | |
| example9 - sitl.py | 0 | two raw strings |
| rc - heli.py | 6 | |
| rc.py | 1 | |
| ui.py | 4 | |
| wipe.py | 4 | |

`example9 - sitl.py` also opens with a byte-order mark and has CRLF line ends; both are left as
they are, and the engine drops the mark as CPython does when it reads a file.

## What is not changed, and so does not run

- Fourteen scripts `import clr` and stop there (`No module named 'clr'`): everything they do
  after that reaches .NET (`from System import ...`, `MissionPlanner.Utilities`) or the link
  (`MAV.setParam`, `MAV.sendPacket`, `MAV.getWP` ...). `wipe.py` reaches `MAV` without `clr`.
  The engine hands `MAV`, `MainV2`, the screens, `Ports` and `Joystick` out as objects that raise
  with their name on first use, so a script stops at that line with a message saying what it
  reached for; the corpus test records the verdict per script. Giving scripts the link is D16's
  next step, not a change to the scripts.
- `rc.py` runs to its end and `example1.py` loops until stopped: the two that only need `Script`
  and `cs`.
- `datetime.py` is not a Python script: its first line is `c#`, and it mixes C# statements with
  Python ones. Mission Planner runs it through the same IronPython `runScript` as the others
  (`GCSViews/FlightData.cs:4731`), where it fails at that first line just as it does here.
- `debugenv.py` imports `distutils`, which left Python at 3.12 and is not in RustPython's
  stdlib; it stops there.
