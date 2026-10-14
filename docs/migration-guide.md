# Moving from Mission Planner: what changed

For a pilot who has flown with Mission Planner (the C# application) and is starting this one. It
is the same ground station - the screens, the words on them, the files it reads and writes - with
the differences below. Where a thing is not here yet, `NOT_DONE_YET_MATRIX.md` says so and in what
order it is coming; `ledger/ledger.csv` has every one of Mission Planner's source files and what
became of it.

## Installing

Linux: the Debian package `tools/package.sh deb` writes installs `planner`, the application, into
`/usr/bin`, with a desktop entry under *MissionPlannerRust*. It needs a Vulkan driver
(`mesa-vulkan-drivers` answers for any GPU, in software when it must). A release
(`.github/workflows/release.yml`, run for a tag of "v" and a number) carries that package and the
planner for Linux (x86-64, glibc 2.35 or newer), Windows (a zip) and macOS (one universal binary for
Apple Silicon and Intel), each unsigned; from source, `cargo build --release -p mp-gui --bin
planner` builds it. The installers - an MSI, a `.dmg` - are not made yet.

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

A setting that points into Mission Planner's folder - the log folder above all, which the copied
`config.xml` carries - is pointed at the same place in this application's folder each time the
settings are loaded, and the status line says once what changed. Nothing in Mission Planner's
folder is moved or copied by that.

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
the log browser - which are tabs here, as are EXPERIMENTAL (Mission Planner's temp form, which
Ctrl+F opens) and PLUGINS (the plugin manager, Ctrl+P's), the owner's additions. The flight
screen's lower-left tab control, the planner's right-click menu, SETUP and CONFIG's lists and their
pages are Mission Planner's own, in its order; each page's controls sit where the `.resx` puts them. The flight screen keeps its own panel of mode
buttons, one click a mode, in place of Mission Planner's mode drop-down and Set Mode button (the
owner's choice).

An error the window can show as state - a port that would not open, a download that failed, a
conversion that did not run - goes on the status line rather than into a message box: the
header's last line, under the tabs, on every screen, an error in the alert colour. Questions and
warnings keep their boxes.

## Logs and recordings

Every connection is recorded to a `.tlog`, both directions, into the `logs` folder of this
application's directory, named in local time as Mission Planner names them. `MP_NO_RECORD=1` turns
that off. The DataFlash Logs page's conversions (Bin to Log, KML + GPX, Matlab) write what Mission
Planner writes; Auto Analysis runs ArduPilot's LogAnalyzer checks here, on every platform, with no
download.

## Updates

The HELP screen's *Check for Updates* and *Check for BETA Updates* say only "Sorry, Updates
aren't implemented yet" on the status line (the owner's word, 2026-10-14), and nothing else is
done: the once-a-day check at start-up, the download and the `/update` and `/updatebeta`
command-line switches are refused too. Mission Planner's own update code is ported
(`crates/mp-update`) and held back, off.

## When it crashes

A crash writes a report under the data directory's `crash-reports/`, with the stack. The next
start asks Mission Planner's question - *An error has occurred ... Report this Error???* - and
Yes posts the report, with a message you can add, where the setting `CrashReportUrl` says; with
none set, the report is kept and the status line says where.

## The command line

There is none for pilots. `headless-planner`, built from the source, does some of what the screens
do without a window - watch and record a link, read and write parameters and missions, generate a
survey, convert and analyse logs, download logs over MAVFTP, describe firmware and the board on a
port, query terrain (`headless-planner --help` lists it) - but it is an internal testing tool, not
part of the application, and no release or package carries it (the owner, 2026-10-04). It flashes
no firmware: that is done from SETUP's Install Firmware page, where the board is on screen.

## Not here

Mission Planner's C# plugins: this application loads WebAssembly plugins from `plugins/` beside
the program instead, and the shipped ones are ported and built in. Python scripts run on Python 3
(RustPython); a script that does `import clr` is given the objects Mission Planner's scripts use -
`MAV`, `MainV2`, the screens, `Ports`, `Joystick` - and stops, with the name, at a .NET member that
is not there. Pages and windows not yet ported are listed in
`NOT_DONE_YET_MATRIX.md`; the matrix is the order they come in.
