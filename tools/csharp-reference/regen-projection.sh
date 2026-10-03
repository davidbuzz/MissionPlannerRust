#!/usr/bin/env bash
# Regenerates testdata/projection/golden from GMap.NET's MercatorProjection and PointLatLngAlt's
# geodesy and UTM: the `projection` verb of PLAN.md §7.1, for §13.3 item 8 and
# crates/mp-units/tests/projection.rs.
#
# The build is regen-grid.sh's, step for step and in the same cache, so whichever script runs first
# builds ExtLibs/Utilities once for both:
#   1. copy https://github.com/ArduPilot/MissionPlanner/tree/efb0801/ExtLibs out of tree, so the reference tree stays read-only;
#   2. msbuild ExtLibs/Utilities/MissionPlanner.Utilities.csproj under mono - it references
#      GMap.NET.Core, so MercatorProjection comes out beside PointLatLngAlt and ProjNet;
#   3. mcs MpProjection.cs against the result;
#   4. run it over testdata/projection/points.txt and swap the output in as golden/.
#
# The build is cached per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires
# mono 6.12 (mono, msbuild, mcs), rsync, and the NuGet packages the csproj restores - from the
# network, or already in ~/.nuget/packages.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:?MP_SRC must name a clone of https://github.com/ArduPilot/MissionPlanner (commit efb0801)}"
DATA="$ROOT/testdata/projection"

[ -f "$MP/ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjection.cs" ] || {
    echo "Mission Planner source not found at $MP (set MP_SRC)" >&2
    exit 1
}
SHA="$(git -C "$MP" rev-parse HEAD)"
BUILD="${MP_ORACLE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/mp-csharp-reference}/$SHA"
OUT="$BUILD/src/ExtLibs/Utilities/bin/Release/netstandard2.0"
NS="$(find /usr/lib/mono -path '*/4.5/Facades/netstandard.dll' | head -1)"
[ -n "$NS" ] || { echo "mono's netstandard facade not found under /usr/lib/mono" >&2; exit 1; }

# Kept identical to regen-grid.sh's build so the two share one cache entry.
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

# System.Memory is referenced only so mcs can resolve PointLatLngAlt's overloads; CS1685 is mono's
# own Span clashing with System.Memory's, which nothing here uses.
mcs -nologo -nowarn:1685 -out:"$OUT/MpProjection.exe" \
    -r:"$OUT/MissionPlanner.Utilities.dll" -r:"$OUT/GMap.NET.Core.dll" -r:"$OUT/System.Memory.dll" \
    -r:"$NS" "$HERE/MpProjection.cs"

# Write to a scratch directory and swap it in, so a failed run leaves the old goldens untouched.
TMP="$(mktemp -d "$DATA/.golden.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
mono "$OUT/MpProjection.exe" projection "$DATA/points.txt" "$TMP"
rm -rf "$DATA/golden"
mv "$TMP" "$DATA/golden"
chmod 755 "$DATA/golden"
trap - EXIT
echo "regenerated $(ls "$DATA/golden" | wc -l) goldens in $DATA/golden from Mission Planner $SHA" >&2
