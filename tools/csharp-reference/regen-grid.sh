#!/usr/bin/env bash
# Regenerates testdata/grid/golden from Mission Planner's own Grid.CreateGrid, Grid.CreateCorridor
# and Grid.CreateRotary: the `grid`, `corridor` and `rotary` verbs of PLAN.md §7.1, for §13.3 item 3,
# §13.4 item 8 and crates/mp-mission/tests/{grid,corridor,rotary}_vectors.rs.
#
# Unlike regen.sh this builds from the pinned source tree, not a downloaded binary distribution:
#   1. copy referneces/missionplanner/ExtLibs out of tree, so the reference tree stays read-only;
#   2. msbuild ExtLibs/Utilities/MissionPlanner.Utilities.csproj under mono - the project §7.1
#      proved builds on Linux, and the one Grid.cs, clipper.cs, utmpos.cs and PointLatLngAlt.cs live
#      in, so no WinForms stub is needed;
#   3. mcs MpGrid.cs against the result;
#   4. run every directive of testdata/grid/cases.txt through its verb: `case` lines as
#      golden/<case>.csv, `corridor` lines as golden/corridor/<case>.csv, `rotary` lines as
#      golden/rotary/<case>.csv and `offset` lines - ClipperLib's offset on its own, for
#      crates/mp-mission/src/clipper.rs - as golden/offset/<case>.csv; `accept` lines - the
#      Survey (Grid) dialog, MpGridUi.cs, for crates/mp-mission/tests/gridui_vectors.rs - as
#      golden/accept/<case>.csv, with the camera list as golden/accept/cameras.csv.
#
# The build is cached per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires
# mono 6.12 (mono, msbuild, mcs), rsync, and the NuGet packages the csproj restores - from the
# network, or already in ~/.nuget/packages.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/referneces/missionplanner}"
DATA="$ROOT/testdata/grid"

[ -f "$MP/ExtLibs/Utilities/Grid.cs" ] || {
    echo "Mission Planner source not found at $MP (set MP_SRC)" >&2
    exit 1
}
SHA="$(git -C "$MP" rev-parse HEAD)"
BUILD="${MP_ORACLE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/mp-csharp-reference}/$SHA"
OUT="$BUILD/src/ExtLibs/Utilities/bin/Release/netstandard2.0"
NS="$(find /usr/lib/mono -path '*/4.5/Facades/netstandard.dll' | head -1)"
[ -n "$NS" ] || { echo "mono's netstandard facade not found under /usr/lib/mono" >&2; exit 1; }

if [ ! -f "$OUT/MissionPlanner.Utilities.dll" ]; then
    echo "building MissionPlanner.Utilities at $SHA in $BUILD" >&2
    mkdir -p "$BUILD/src"
    rsync -a --delete --exclude bin --exclude obj "$MP/ExtLibs" "$BUILD/src/"
    cp "$MP/nuget.config" "$BUILD/src/"
    # CopyLocalLockFileAssemblies puts the package assemblies (GeoAPI, which ProjNet needs at run
    # time, and the rest) beside the library, so the harness runs from that directory.
    msbuild -nologo -v:minimal -t:Restore,Build -p:Configuration=Release \
        -p:CopyLocalLockFileAssemblies=true \
        "$BUILD/src/ExtLibs/Utilities/MissionPlanner.Utilities.csproj" >"$BUILD/msbuild.log" 2>&1 || {
        tail -30 "$BUILD/msbuild.log" >&2
        exit 1
    }
fi

# GMap.NET.Core and System.Memory are referenced only so mcs can resolve PointLatLngAlt's
# overloads; CS1685 is mono's own Span clashing with System.Memory's, which nothing here uses.
# ProjNET and GeoAPI are for MpGridUi.cs's calcpolygonarea, which projects with them directly.
mcs -nologo -nowarn:1685 -out:"$OUT/MpGrid.exe" \
    -r:"$OUT/MissionPlanner.Utilities.dll" -r:"$OUT/GMap.NET.Core.dll" -r:"$OUT/System.Memory.dll" \
    -r:"$OUT/ProjNET.dll" -r:"$OUT/GeoAPI.dll" -r:"$OUT/GeoAPI.CoordinateSystems.dll" \
    -r:System.Xml.dll -r:"$NS" "$HERE/MpGrid.cs" "$HERE/MpGridUi.cs"

# Write to a scratch directory and swap it in, so a failed run leaves the old goldens untouched.
TMP="$(mktemp -d "$DATA/.golden.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/corridor" "$TMP/rotary" "$TMP/offset" "$TMP/accept"
mono "$OUT/MpGrid.exe" grid "$DATA/cases.txt" "$TMP"
mono "$OUT/MpGrid.exe" corridor "$DATA/cases.txt" "$TMP/corridor"
mono "$OUT/MpGrid.exe" rotary "$DATA/cases.txt" "$TMP/rotary"
mono "$OUT/MpGrid.exe" offset "$DATA/cases.txt" "$TMP/offset"
# The Survey (Grid) dialog, over the camera list Mission Planner ships beside its executable.
mono "$OUT/MpGrid.exe" accept "$DATA/cases.txt" "$TMP/accept" "$MP/camerasBuiltin.xml"
rm -rf "$DATA/golden"
mv "$TMP" "$DATA/golden"
chmod 755 "$DATA/golden" "$DATA/golden/corridor" "$DATA/golden/rotary" "$DATA/golden/offset" \
    "$DATA/golden/accept"
trap - EXIT
echo "regenerated $(find "$DATA/golden" -name '*.csv' | wc -l) goldens in $DATA/golden from Mission Planner $SHA" >&2
