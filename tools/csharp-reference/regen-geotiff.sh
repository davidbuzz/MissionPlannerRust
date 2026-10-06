#!/usr/bin/env bash
# Regenerates testdata/geotiff/oracle.txt from Mission Planner's own GeoTIFF terrain lookup,
# GeoTiff (ExtLibs/Utilities/GeoTiff.cs) and the srtm.getAltitude that asks it first
# (ExtLibs/Utilities/srtm.cs:116-150), for crates/mp-terrain/tests/geotiff.rs.
#
# The build is regen-srtm.sh's, step for step and in the same cache, so whichever script runs first
# builds ExtLibs/Utilities once for all of them:
#   1. copy https://github.com/ArduPilot/MissionPlanner/tree/efb0801/ExtLibs out of tree, so the reference tree stays read-only;
#   2. msbuild ExtLibs/Utilities/MissionPlanner.Utilities.csproj under mono - GeoTiff.cs and srtm.cs
#      are in it, with LibTiff.Net, DotSpatial.Projections, Microsoft.Extensions.Caching.Memory and
#      GMap.NET.Core beside it;
#   3. mcs GeoTiffOracle.cs against the result;
#   4. make the test GeoTIFFs if any is missing (make-geotiffs.py; they are committed, and the
#      script writes the same bytes every time);
#   5. run the oracle over a scratch copy of them and swap its output in as oracle.txt.
#
# Nothing leaves the machine: srtm's servers are pointed at a loopback port and GMap.NET is put in
# CacheOnly mode, so a lookup that falls through to SRTM queues no download.
#
# The build is cached per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires
# mono 6.12 (mono, msbuild, mcs), rsync, python3 with numpy, and the NuGet packages the csproj
# restores - from the network, or already in ~/.nuget/packages.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:?MP_SRC must name a clone of https://github.com/ArduPilot/MissionPlanner (commit efb0801)}"
DATA="$ROOT/testdata/geotiff"

[ -f "$MP/ExtLibs/Utilities/GeoTiff.cs" ] || {
    echo "Mission Planner source not found at $MP (set MP_SRC)" >&2
    exit 1
}
SHA="$(git -C "$MP" rev-parse HEAD)"
BUILD="${MP_ORACLE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/mp-csharp-reference}/$SHA"
OUT="$BUILD/src/ExtLibs/Utilities/bin/Release/netstandard2.0"
NS="$(find /usr/lib/mono -path '*/4.5/Facades/netstandard.dll' | head -1)"
[ -n "$NS" ] || { echo "mono's netstandard facade not found under /usr/lib/mono" >&2; exit 1; }

# Kept identical to regen-grid.sh's build so the scripts share one cache entry.
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

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY

if [ ! -f "$DATA/geo_west_area.tif" ] || [ ! -f "$DATA/polar_stereo.tif" ]; then
    python3 "$HERE/make-geotiffs.py" "$DATA"
fi

# Write to a scratch directory and swap it in, so a failed run leaves the old oracle untouched.
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# The harness runs from a copy of the build, as regen-georef.sh's does and for the same reason:
# LibTiff.Net targets netstandard1.3 and opens every file with File.Open(string, FileMode,
# FileAccess, FileShare), and the System.IO.FileSystem.Primitives the restore put beside the
# library is the portable one, whose FileMode is not mscorlib's - so under mono every Tiff.Open
# throws MissingMethodException and LoadFile leaves every file out of the index. Mono's own facade
# forwards the types to mscorlib, as .NET Framework's binding does for the application. The shared
# cache is left as it is.
BIN="$TMP/bin"
mkdir -p "$BIN"
cp "$OUT"/*.dll "$BIN/"
cp "$(dirname "$NS")/System.IO.FileSystem.Primitives.dll" "$BIN/"
# CS1685 is mono's own Span clashing with System.Memory's, which nothing here uses.
mcs -nologo -nowarn:1685 -out:"$BIN/GeoTiffOracle.exe" \
    -r:"$BIN/MissionPlanner.Utilities.dll" -r:"$BIN/GMap.NET.Core.dll" \
    -r:"$BIN/DotSpatial.Projections.dll" -r:"$BIN/System.Memory.dll" \
    -r:"$NS" "$HERE/GeoTiffOracle.cs"

mono "$BIN/GeoTiffOracle.exe" "$DATA" "$TMP/srtm" "$TMP/oracle.txt" >"$TMP/stdout.txt"
mv "$TMP/oracle.txt" "$DATA/oracle.txt"
echo "regenerated $(grep -c '^alt ' "$DATA/oracle.txt") lookups in $DATA/oracle.txt from Mission Planner $SHA" >&2
