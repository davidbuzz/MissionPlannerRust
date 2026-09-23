# Survey grid goldens

What Mission Planner's own survey generators (`ExtLibs/Utilities/Grid.cs`) return for the cases in
`cases.txt`, one file per case:

| directive  | C#                                   | goldens                        | compared by                                   |
|------------|--------------------------------------|--------------------------------|-----------------------------------------------|
| `case`     | `Grid.CreateGrid`                    | `golden/<case>.csv`            | `crates/mp-mission/tests/grid_vectors.rs`     |
| `corridor` | `Grid.CreateCorridor`                | `golden/corridor/<case>.csv`   | `crates/mp-mission/tests/corridor_vectors.rs` |
| `rotary`   | `Grid.CreateRotary`                  | `golden/rotary/<case>.csv`     | `crates/mp-mission/tests/rotary_vectors.rs`   |
| `offset`   | `ClipperLib.ClipperOffset` (below)   | `golden/offset/<case>.csv`     | `crates/mp-mission/src/clipper.rs`, its tests |

Each test runs the Rust port (`mp_mission::grid::create_grid`, `mp_mission::corridor::create_corridor`,
`mp_mission::rotary::create_rotary`, and the crate's own ClipperLib port) over every case of its
kind and compares. PLAN.md §13.3 item 3 and §13.4 item 8, DELIVERABLES.md D11.

## How they are made

`tools/csharp-reference/regen-grid.sh`, which needs mono 6.12 (`mono`, `msbuild`, `mcs`) and
`rsync`:

1. copies `ExtLibs` out of `referneces/missionplanner` into a cache directory keyed on the Mission
   Planner commit (`~/.cache/mp-csharp-reference/<sha>`), so the reference tree is never written to;
2. builds `ExtLibs/Utilities/MissionPlanner.Utilities.csproj` there with mono's `msbuild` - the
   project PLAN.md §7.1 showed builds on Linux. `Grid.cs`, `clipper.cs`, `utmpos.cs`,
   `PointLatLngAlt.cs`, `Rect.cs` and the ProjNet they use all build in it, so nothing is stubbed;
3. compiles `tools/csharp-reference/MpGrid.cs` against the result with `mcs`;
4. runs its `grid`, `corridor`, `rotary` and `offset` verbs over `cases.txt`, each picking out its
   own directive, and swaps the output in as `golden/`.

The first run restores NuGet packages (from the network, or `~/.nuget/packages`) and builds: 15 s
from a cold cache with the packages already local. Later runs reuse the build and take about a
second. Running the script again reproduces every file byte for byte.

**Oracle version:** Mission Planner `efb080190de0bf091f9aab982c8848a124de7588`, the upstream pin at
the top of PLAN.md, built and run under Mono 6.12.0.200 on Linux x86-64 (glibc libm).

## The files

`cases.txt` defines named polygons and the cases that use them. A case names its parameters as its
generator does (`CreateGrid` `Grid.cs:354`, `CreateCorridor` `:55`, `CreateRotary` `:196`); one left
out takes the value `GridUI` gives it on a fresh install, including the angle, which `GridUI` sets
to the bearing of the polygon's longest side. The defaults, with their sources, are listed at the
top of `MpGrid.cs`. A corridor's polygon is its centre line, which `Grid.cs` calls `polygon` too.

Each golden records, as `key,value` lines, the case name, every vertex and every argument exactly as
the generator received them - including the ones `CreateCorridor` and `CreateRotary` take and never
read - then `points,<n>` and one `wp,lat,lng,alt,tag` line per point returned. The tests read the
arguments from the golden rather than re-deriving them, so a default resolved by the harness is used
as the C# used it. Doubles are written `G17` and floats `G9` in the invariant culture: both always
round-trip.

### Grids

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

### Corridors

41 cases over 18 centre lines of 2 to 8 points: the dialog's defaults; lane counts from one (a width
under the lane distance) to seven, and a width that is not a whole float; trigger spacing at 4 m,
at 2 m (clamped to 4), at 10.5 m (laid at whole metres), and longer than the legs; every start
position, which only splits Home from the rest (the far end of the line first); a hairpin, a line
barely bent, a right angle; the arguments `CreateCorridor` never reads, set and unset, with the
same result; altitude; the 0.1 m distance clamp; a repeated vertex and three vertices exactly
collinear on zone 55's central meridian, whose parallel legs meet at `utmpos.Zero` (0, 172.5 E),
which the C# flies to; and the same sites and frames as the grids - Zurich, Tromsø, Brisbane, the
equator and the zone 55/56 edge with the first vertex either side, and the antimeridian. A
two-point line gives nothing: `GenerateOffsetPath` lays lanes only around corners.

### Rotary

63 cases over 39 polygons: the dialog's defaults (laps until the area runs out, 200 at most); 0, 1
and 3 laps, and 10 m laps; clockwise laps 1, 2, all (-1) and more than there are laps; the matched
spiral perimeter with 0, 1 and 3 clockwise laps; every start position, a start point, a home in
another zone; a polygon drawn clockwise and one closed on its first vertex; convex and concave
shapes that split into separate outlines as they shrink (L, T, U, comb, fields, a strip, a sliver);
a notch sharp enough to be squared off rather than mitred, a vertex exactly on its neighbours'
line, polygons drawn crossing and touching themselves (bow tie, pentagram, figure 8, a doubled
edge, a dumbbell), and a comb whose teeth end on the equator, where northings are exactly zero; the
arguments `CreateRotary` never reads; altitude; the 0.1 m distance clamp; a polygon collapsed to a
point and a flat one; and the same sites and frames as the grids.

### ClipperLib's offset

25 cases of `ClipperOffset` itself, driven as `CreateRotary` drives it (`Grid.cs:248-257`: mitred,
closed, one object executed at each delta in turn), over integer paths: squares that touch along
an edge, at a corner, stacked and shifted, overlapping; a square with a hole, a hole touching the
outer edge, a hole with an island; triangles sharing an edge; a comb, an L, stairs, battlements, a
keyhole, a figure 8, a bow tie, a thin strip, a 25-pointed star; rings and twins drawn as one path
through a zero-width slit. Exactly collinear, touching edges are what the union's joins, splits and
hole fixups exist for, and projected latitudes and longitudes almost never produce them, so these
reach the parts of the ported ClipperLib that the rotary cases do not; positive deltas also take
the offset's other branch. Each golden records the paths, then per delta the top-level count and
every node of the `PolyTree`, depth first.

## Tolerance

PLAN.md §7.2 class C: the same number of points, in the same order with the same tags, each within
1e-7 degrees, at the same altitude. A grid case that can match only up to a tie-break is listed by
name and reason in `TIE_BREAKS` in the test and held to the class C invariants instead. None is:
all 180 match, and on this machine they match bit for bit.

Corridors and rotary patterns are held tighter: bit for bit, every latitude, longitude and altitude
the same double. A case that could match only to within rounding would be listed with its reason in
the test's `CLASS_C` and held to class C instead; none is. The offset cases match node for node, and
coordinate for coordinate.

The rotary pattern's one dependency on the runtime beyond IEEE arithmetic and libm is
`List<T>.Sort`, which ClipperLib uses to order the intersections on one scanline and which is not
stable: `clipper.rs` ports mono's introsort, which matched every one of 276 tie-heavy sorts a mono
probe printed (2 to 300 keys), and its tests hold it to a sample of them. The introsort's heapsort
fallback, which only a partition deeper than twice the list's log2 capacity reaches, is ported but
reached by none of them.

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
