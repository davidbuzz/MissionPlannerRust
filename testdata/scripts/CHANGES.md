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
| example6.py | 11 | one `except ..., msg:`; `recv(...).decode()` and `msg.args[1]` (below) |
| example7.py | 1 | |
| example8 - speech.py | 2 | |
| example9 - sitl.py | 0 | two raw strings |
| rc - heli.py | 6 | |
| rc.py | 1 | |
| ui.py | 4 | |
| wipe.py | 4 | |

`example9 - sitl.py` also opens with a byte-order mark and has CRLF line ends; both are left as
they are, and the engine drops the mark as CPython does when it reads a file.

## `example6.py`'s bytes, 2026-09-26

Two more Python 2 forms, found once the shim let example6 run past `import clr`:

- `msg = rsock.recv(1024)` became `msg = rsock.recv(1024).decode()`. Python 2's `recv` returned
  a `str`, which `re.compile("[ ]").split(msg)` splits; Python 3's returns `bytes`, which a text
  pattern refuses (`TypeError: cannot use a string pattern on a bytes-like object`).
- `msg[1]` in the bind failure's `except socket.error as msg:` became `msg.args[1]`: Python 2's
  exceptions were sequences, Python 3's are not.

## How each script ends now, 2026-09-26

The engine answers to the names IronPython gives a script - `clr`, `System`, `MissionPlanner`,
`MAVLink`, and `MAV`, `MainV2`, `FlightPlanner`, `FlightData`, `Ports`, `Joystick` in the scope -
from `crates/mp-script/src/clr/shim.py`, each member with `MAVLinkInterface`'s semantics and the
link's side through the Scripts tab's host. A name it does not give raises "<name> is not
available to scripts in this version". `crates/mp-script/tests/stock_scripts.rs` runs every
script against a simulated vehicle and asserts each verdict, and what each asked of the vehicle.

| Script | Before (2026-09-25) | Now |
|---|---|---|
| PARACHUTE LANDING APPROACH.py | `import clr` | uploads its five items, then `MAV.setWPCurrent(1)`: TypeError |
| TAKEOFF.py | `import clr` | uploads its six items, then `MAV.setWPCurrent(1)`: TypeError |
| cubeorange.py | `import clr` | runs to the end: six `MAV.setParam(..., True)` |
| datetime.py | NameError `c` | unchanged: not Python |
| debugenv.py | `distutils` | unchanged: `distutils` |
| example1.py | loops until stopped | unchanged |
| example10.py | `import clr` | `MAV.SubscribeToPacketType(id, func)`: TypeError |
| example2.py | `import clr` | `MAV.SubscribeToPacketType(id, func)`: TypeError |
| example3.py | `import clr` | `setGuidedModeWP`, then loops until stopped |
| example4 wp.py | `import clr` | runs to the end: its five items written, `setWPACK` |
| example5 inject data.py | `import clr` | runs to the end: six bytes on the port |
| example6.py | `import clr` | follows each UDP datagram with `setGuidedModeWP`, waits for the next |
| example7.py | `import clr` | NameError `null` |
| example8 - speech.py | `import clr` | loops until stopped, `SpeakAsync` each second |
| example9 - sitl.py | `import clr` | needs `System.Diagnostics` |
| rc - heli.py | `import clr` | runs to the end: arm, Guided, take-off |
| rc.py | runs to the end | unchanged |
| ui.py | `import clr` | needs `System.Windows.Forms` |
| wipe.py | needs `MAV` | runs to the end: `MAV.doReboot()` |

Four of these end in an error that is the script's own, and Mission Planner's IronPython ends
them the same way:

- `TAKEOFF.py` and `PARACHUTE LANDING APPROACH.py` call `MAV.setWPCurrent(1)`. The C# has one
  `setWPCurrent`, of three parameters (`byte sysid, byte compid, ushort index`,
  `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2452`), so IronPython refuses one argument -
  after the missions are written, before the scripts' `setWPACK` and closing line.
- `example2.py` and `example10.py` call `MAV.SubscribeToPacketType(msgid, func)`. The C#'s takes
  `msgid, func, sysid, compid[, exclusive]` (`MAVLinkInterface.cs:5567`); the scripts were
  written for an older signature. Everything example2 does after that line (`doCommand`,
  `setDigicamControl`, `doARM`, `GetParam`, `getWPCount`, `getParamList`, `sendPacket` with a
  `mavlink_command_long_t`) is therefore never reached, under Mission Planner or here, and those
  members are not given to scripts: no shipped script reaches them.
- `example7.py` passes `null`, which neither IronPython nor Python has; `MainV2.instance.
  FlightPlanner.BUT_read_Click` resolves first (it is public), and would refuse if called: reading
  the vehicle's mission into the Plan screen from a script is not given in this version.

## What is out of scope, member by member

- `example9 - sitl.py`: `System.Diagnostics.Process` (it starts twenty Windows
  `ArduCopter.exe` SITLs), then `MissionPlanner.Comms.TcpSerial`,
  `System.Net.Sockets.TcpClient`, `MissionPlanner.MAVLinkInterface()`, `getHeartBeat` and
  `MainV2.Comports.Add` - a second link made by a script. It stops at the first.
- `ui.py`: `System.Windows.Forms` (`Application`, `Form`, `Label`), `System.Drawing`,
  `System.Activator.CreateInstance` and `clr.References` - a WinForms form. It stops at the
  first; `clr.ClearProfilerData()` before it is IronPython's profiler, off, and does nothing.
- `example8 - speech.py`: speech itself is DELIVERABLES D15. `MainV2.speechEnable` and
  `MainV2.speechEngine.SpeakAsync` are given, with `SpeakAsync`'s checks and rewording; the text
  it would say is handed to the host, which the Scripts tab publishes as the fact
  `fly.script.speech`.
- `example10.py`'s `MAVLink.MAVLINK_MSG_ID.STATUSTEXT_LONG` is not in the C#'s enum either; the
  script never reaches it.
- `MAV.SubscribeToPacketType` with its four arguments and `MAV.OnPacketReceived`: a feed of the
  link's packets to a script. No shipped script reaches either (see example2 and example10).
- `Joystick` is `None`: `MainV2.joystick` is null until a joystick is started, and no shipped
  script reads it. `Ports` is the list holding `MAV`, as `MainV2.Comports` holds the link.

## What is not changed, and so does not run

- `datetime.py` is not a Python script: its first line is `c#`, and it mixes C# statements with
  Python ones. Mission Planner runs it through the same IronPython `runScript` as the others
  (`GCSViews/FlightData.cs:4731`), where it fails at that first line just as it does here.
- `debugenv.py` imports `distutils`, which left Python at 3.12 and is not in RustPython's
  stdlib; it stops there.
