# Moving from Mission Planner: what changed

For a pilot who has flown with Mission Planner (the C# application) and is starting this one. It
is the same ground station - the screens, the words on them, the files it reads and writes - with
the differences below. Where a thing is not here yet, `NOT_DONE_YET_MATRIX.md` says so and in what
order it is coming; `ledger/ledger.csv` has every one of Mission Planner's source files and what
became of it.

## Installing

Linux: the Debian package `tools/package.sh deb` writes installs `planner` (the graphical
application) and `headless-planner` (the same over the command line) into `/usr/bin`, with a
desktop entry under *MissionPlannerRust*. It needs a Vulkan driver (`mesa-vulkan-drivers`
answers for any GPU, in software when it must). Windows and macOS: build from source for now
(`cargo build --release`); the installers are not made yet.

## Your settings, missions and files

Mission Planner keeps its files under *Mission Planner* (`Documents\Mission Planner` on Windows,
`~/.local/share/Mission Planner` on Linux). This application keeps its own under
*MissionPlannerRust* next to it, so running both never writes over the other's files.

On the first start that finds its directory missing or empty, it copies these from Mission
Planner's, once, and leaves the originals as they were: `config.xml` (every setting), `poi.txt`,
`cameras.xml`, `checklist.xml`, `warnings.xml`, `UserAlerts.json`, `authkeys.xml`, `logo.png`,
`logo.txt` and the `History` folder. A file named `imported-from-Mission-Planner.txt` records what
was copied. Not copied, because each is large and comes back on its own: the map tile cache
(`gmapcache`), the terrain tiles (`srtm`), the flight logs (`logs`) and the parameter metadata -
the tiles and terrain are fetched again as you look at them, and your old logs stay where Mission
Planner wrote them, readable from there with Load Log.

`config.xml` is read and written as Mission Planner reads and writes it - the same keys, the same
events (start-up, the FLIGHT DATA and FLIGHT PLAN buttons, Connect, the close box) - so a setting
made here is understood there and the other way round. Mission files (`.waypoints`), parameter
files (`.param`), fence and rally files, `.tlog` recordings and `.BIN` dataflash logs are the
same formats, byte for byte where a file is written.

The map tile cache is Mission Planner's own layout: a cache either application filled is read by
both.

## The screens

The menu across the top is `MainV2`'s: FLIGHT DATA (`fly`), FLIGHT PLAN (`plan`), SETUP, CONFIG,
SIMULATION and HELP, and two more Mission Planner opens as windows - the vehicle's parameters and
the log browser - which are tabs here. The flight screen's lower-left tab control, the planner's
right-click menu, SETUP and CONFIG's lists and their pages are Mission Planner's own, in its order;
each page's controls sit where the `.resx` puts them. The flight screen keeps its own panel of mode
buttons beside Mission Planner's mode drop-down (the owner's choice).

An error the window can show as state - a port that would not open, a download that failed, a
conversion that did not run - goes on the status line at the bottom rather than into a message
box. Questions and warnings keep their boxes.

## Logs and recordings

Every connection is recorded to a `.tlog`, both directions, into the `logs` folder of this
application's directory, named in local time as Mission Planner names them. `MP_NO_RECORD=1` turns
that off. The DataFlash Logs page's conversions (Bin to Log, KML + GPX, Matlab) write what Mission
Planner writes; Auto Analysis runs ArduPilot's LogAnalyzer checks here, on every platform, with no
download.

## Updates

The HELP screen's *Check for Updates* and *Check for BETA Updates* are Mission Planner's, and the
once-a-day check at start-up. Mission Planner's channels carry its own .NET files, so here the
channel is empty until one is published for this program: the settings `UpdateLocationVersion`,
`UpdateLocation` and `UpdateLocationMD5` (and the `Beta*` and `Master*` names from
Mission Planner's `app.config`) in `config.xml` name it. With none set, the checks do nothing.

## When it crashes

A crash writes a report under the data directory's `crash-reports/`, with the stack. The next
start asks Mission Planner's question - *An error has occurred ... Report this Error???* - and
Yes posts the report, with a message you can add, where the setting `CrashReportUrl` says; with
none set, the report is kept and the status line says where.

## The command line

`headless-planner` does, without a window, what the screens do: watch and record a link, read and
write parameters and missions, generate a survey, convert and analyse logs, download logs over
MAVFTP, install firmware, query terrain. `headless-planner --help` lists it.

## Not here

Mission Planner's C# plugins: this application loads WebAssembly plugins from `plugins/` beside
the program instead, and the shipped ones are ported. Python scripts run on Python 3 (RustPython);
scripts that `import clr` stop there. Pages and windows not yet ported are listed in
`NOT_DONE_YET_MATRIX.md`; the matrix is the order they come in.
