# Survey grid goldens

What Mission Planner's own `Grid.CreateGrid` (`ExtLibs/Utilities/Grid.cs`) returns for the cases
in `cases.txt`, one `golden/<case>.csv` per case. `crates/mp-mission/tests/grid_vectors.rs` runs
the Rust port (`mp_mission::grid::create_grid`) over every case and compares. PLAN.md §13.3 item 3,
DELIVERABLES.md D11.

## How they are made

`tools/csharp-reference/regen-grid.sh`, which needs mono 6.12 (`mono`, `msbuild`, `mcs`) and
`rsync`:

1. copies `ExtLibs` out of `referneces/missionplanner` into a cache directory keyed on the Mission
   Planner commit (`~/.cache/mp-csharp-reference/<sha>`), so the reference tree is never written to;
2. builds `ExtLibs/Utilities/MissionPlanner.Utilities.csproj` there with mono's `msbuild` - the
   project PLAN.md §7.1 showed builds on Linux. `Grid.cs`, `utmpos.cs`, `PointLatLngAlt.cs`, `Rect.cs`
   and the ProjNet they use all build in it, so nothing is stubbed;
3. compiles `tools/csharp-reference/MpGrid.cs` against the result with `mcs`;
4. runs it over `cases.txt` and swaps the output in as `golden/`.

The first run restores NuGet packages (from the network, or `~/.nuget/packages`) and builds: 15 s
from a cold cache with the packages already local. Later runs reuse the build and take about a
second. Running the script again reproduces every file byte for byte.

**Oracle version:** Mission Planner `efb080190de0bf091f9aab982c8848a124de7588`, the upstream pin at
the top of PLAN.md, built and run under Mono 6.12.0.200 on Linux x86-64 (glibc libm).

## The files

`cases.txt` defines named polygons and the cases that use them. A case names its parameters as
`CreateGrid` does (`Grid.cs:354`); one left out takes the value `GridUI` gives it on a fresh
install, including the angle, which `GridUI` sets to the bearing of the polygon's longest side. The
defaults, with their sources, are listed at the top of `MpGrid.cs`.

Each golden records, as `key,value` lines, the case name, every vertex and every argument exactly as
`CreateGrid` received them, then `points,<n>` and one `wp,lat,lng,alt,tag` line per point returned.
The test reads the arguments from the golden rather than re-deriving them, so a default resolved by
the harness is used as the C# used it. Doubles are written `G17` and floats `G9` in the invariant
culture: both always round-trip.

180 cases over 40 polygons. 37 of them - rectangles (one drawn clockwise, one closed on its first
vertex), an L, a T, a U, a comb, concave fields, a thin strip, a sliver, triangles, a pentagon, a
hexagon, a trapezoid; around the SITL home in Canberra and at other sites south and north of the
equator up to 69.6 N; the same area straddling the equator and crossing the zone 55/56 edge with its
first vertex on either side, so projected in either frame; a zone 32/33 crossing and one across the
antimeridian - are each run at angles 0, 45, 90 and 137 with lane spacings scaled to the polygon and
no, symmetric or asymmetric overshoot. Around those: trigger spacing, lead-in, negative lead-in and
overshoot, every start position, a home in another zone, lane separation with and without the
extended end point, altitude, the dialog's untouched defaults, the 0.1 m clamps on a 2 m square,
and two polygons that enclose no area (collapsed to a point, and flat).

## Tolerance

PLAN.md §7.2 class C: the same number of points, in the same order with the same tags, each within
1e-7 degrees, at the same altitude. A case that can match only up to a tie-break is listed by name
and reason in `TIE_BREAKS` in the test and held to the class C invariants instead. None is: all 180
match, and on this machine they match bit for bit.

The invariants are checked against every golden as well, so they are known to be true of Mission
Planner's grids: lanes one spacing apart to 1%; every part of a lane's line inside the area flown,
with the area reaching less than one spacing past the outermost lanes; nothing further outside the
area than the longest lead-in or overshoot. §7.2's "every interior point within spacing/2 of a lane"
is **not** true of Mission Planner's grids - lanes are laid from the bounding rectangle, so an edge
strip can be almost a whole spacing wide, and a sliver's tip can fall between lanes - and is held
only between the outermost lanes; the test's module documentation has the measurements.

Mono 6.12 is not .NET Framework 4.7.2 (PLAN.md R5). Every value here comes from arithmetic and
libm, not from culture or formatting, but a Windows run would use the Microsoft C runtime's
`sin`/`cos`/`pow`, which can differ from glibc's in the last bit: within class C, and able to flip a
tie-break only where two lanes are equidistant to the last bit.
