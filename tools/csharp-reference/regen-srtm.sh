#!/usr/bin/env bash
# Regenerates testdata/srtm/oracle.txt from Mission Planner's own terrain lookup,
# srtm.getAltitude (ExtLibs/Utilities/srtm.cs), for crates/mp-terrain/tests/oracle.rs.
#
# The build is regen-grid.sh's, step for step and in the same cache, so whichever script runs first
# builds ExtLibs/Utilities once for all of them:
#   1. copy https://github.com/ArduPilot/MissionPlanner/tree/efb0801/ExtLibs out of tree, so the reference tree stays read-only;
#   2. msbuild ExtLibs/Utilities/MissionPlanner.Utilities.csproj under mono - srtm.cs, GeoTiff.cs
#      and DTED.cs are all in it, with GMap.NET.Core and SharpZipLib beside it;
#   3. mcs SrtmOracle.cs against the result;
#   4. make testdata/srtm/N00W001.hgt.zip if it is missing (SrtmOracle mksynthetic: the zip carries
#      a timestamp, so remaking it every run would change a committed file for nothing);
#   5. run the oracle over a scratch cache directory and swap its output in as oracle.txt.
#
# The oracle runs its own web server on 127.0.0.1 and points srtm's two servers at it; nothing
# leaves the machine. Proxy variables are cleared so mono does not send loopback traffic to one.
#
# The build is cached per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires
# mono 6.12 (mono, msbuild, mcs), rsync, and the NuGet packages the csproj restores - from the
# network, or already in ~/.nuget/packages.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:?MP_SRC must name a clone of https://github.com/ArduPilot/MissionPlanner (commit efb0801)}"
DATA="$ROOT/testdata/srtm"

[ -f "$MP/ExtLibs/Utilities/srtm.cs" ] || {
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

# CS1685 is mono's own Span clashing with System.Memory's, which nothing here uses.
mcs -nologo -nowarn:1685 -out:"$OUT/SrtmOracle.exe" \
    -r:"$OUT/MissionPlanner.Utilities.dll" -r:"$OUT/GMap.NET.Core.dll" \
    -r:"$OUT/ICSharpCode.SharpZipLib.dll" -r:"$OUT/System.Memory.dll" -r:System.Net.Http \
    -r:"$NS" "$HERE/SrtmOracle.cs"

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY

if [ ! -f "$DATA/N00W001.hgt.zip" ]; then
    mono "$OUT/SrtmOracle.exe" mksynthetic "$DATA/N00W001.hgt.zip"
fi

# Write to a scratch directory and swap it in, so a failed run leaves the old oracle untouched.
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mono "$OUT/SrtmOracle.exe" oracle "$DATA" "$TMP/srtm" "$TMP/oracle.txt" >"$TMP/stdout.txt"
mv "$TMP/oracle.txt" "$DATA/oracle.txt"
echo "regenerated $(grep -c '^alt ' "$DATA/oracle.txt") answers in $DATA/oracle.txt from Mission Planner $SHA" >&2
