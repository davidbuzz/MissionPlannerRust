# Projection goldens

What Mission Planner's own projection and geodesy code returns for the coordinates in `points.txt`:
GMap.NET's `MercatorProjection` (`ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjection.cs`),
which every map in the application draws with, and `PointLatLngAlt`'s `GetDistance`, `GetBearing`,
`newpos` and UTM (`ExtLibs/Utilities/PointLatLngAlt.cs`, `utmpos.cs`, and ProjNet behind them).
`crates/mp-units/tests/projection.rs` re-runs every line through `mp_units` and compares. PLAN.md
§13.3 item 8, DELIVERABLES.md D8.

## How they are made

`tools/csharp-reference/regen-projection.sh`, which needs mono 6.12 (`mono`, `msbuild`, `mcs`) and
`rsync`. It builds exactly as `regen-grid.sh` does, into the same cache, so whichever runs first
builds for both:

1. copies `ExtLibs` out of `referneces/missionplanner` into a cache directory keyed on the Mission
   Planner commit (`~/.cache/mp-csharp-reference/<sha>`, or `$MP_ORACLE_CACHE/<sha>`), so the
   reference tree is never written to;
2. builds `ExtLibs/Utilities/MissionPlanner.Utilities.csproj` there with mono's `msbuild`. It
   references `GMap.NET.Core`, so `MercatorProjection` is built beside `PointLatLngAlt` and ProjNet
   and nothing is stubbed;
3. compiles `tools/csharp-reference/MpProjection.cs` against the result with `mcs`;
4. runs it over `points.txt` and swaps the output in as `golden/`.

17 s from a cold cache with the NuGet packages already in `~/.nuget/packages`; about a second after
that. Running the script again reproduces every file byte for byte.

**Oracle version:** Mission Planner `efb080190de0bf091f9aab982c8848a124de7588`, the upstream pin at
the top of PLAN.md, built and run under Mono 6.12.0.200 on Linux x86-64 (glibc libm).

## The files

`points.txt` lists the coordinates: three lattices (15 degrees over the whole globe, which lands on
the poles, the equator and both sides of the antimeridian; 17.9 degrees from 89.5 south, which
lands on nothing round; 0.01 degrees around ArduPilot's SITL home), then single points at GMap's
latitude clip and the projection's true edge and either side of both, the poles' neighbourhoods,
the antimeridian, Null Island, SITL home and points a millimetre from it, and UTM zone edges. 676
points in all. `pair` and `newpos` lines add geodesy cases beyond the consecutive points. The
directives are described at the top of `MpProjection.cs`; a lattice is stepped in `decimal`, so its
ends are hit exactly.

`golden/` holds three files of `key,value...` lines, every input written back exactly as the C#
received it, so the test uses the C#'s doubles rather than re-parsing `points.txt`:

- `mercator.csv` - `size,<zoom>,<w>,<h>` from `GetTileMatrixSizePixel` for zooms 1, 10, 16, 20 and
  30, then per point `point,<lat>,<lng>` and, for each zoom, `FromLatLngToPixel`'s whole pixel and
  `FromPixelToLatLng` of that pixel. Zoom 30 is there because GMap only ever returns whole pixels:
  it is the deepest zoom at which GMap's `1 << zoom` is still exact, so its pixel - 2^-38 of the
  world, 0.15 mm at the equator - is the finest reading of the continuous projection GMap allows.
- `geodesy.csv` - `pair,<lat1>,<lng1>,<lat2>,<lng2>,<GetDistance>,<GetBearing>,<lat>,<lng>`, the
  last two being `newpos` from the first point with that bearing and distance, for every `pair`
  line and every two consecutive points (690 pairs); and
  `newpos,<lat>,<lng>,<bearing>,<distance>,<lat>,<lng>` for every `newpos` line.
- `utm.csv` - per point, `utm,<lat>,<lng>,<zone>,<x>,<y>,<lat>,<lng>`: `new utmpos(point)`, whose
  zone is `GetUTMZone` (negative in the south, PLAN.md §1.3), and that `utmpos`'s `ToLLA()`.
  ProjNet refused nothing, so there are no `error` lines. `mp_units` has no UTM and the port that
  does, `mp_mission`'s `utm` module, is private to its crate, so this file is generated and checked
  for shape but not yet compared.

Doubles are written `G17` in the invariant culture, as the grid goldens are: it always round-trips.
