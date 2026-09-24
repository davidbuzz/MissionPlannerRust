#!/usr/bin/env bash
# Regenerates testdata/georef/golden from Mission Planner's own Geo Reference Images code,
# GeoRefImageBase (ExtLibs/Utilities/GeoRefImageBase.cs), for crates/mp-georef/tests/oracle.rs.
#
# The build is regen-grid.sh's, step for step and in the same cache, so whichever script runs first
# builds ExtLibs/Utilities once for all of them:
#   1. copy referneces/missionplanner/ExtLibs out of tree, so the reference tree stays read-only;
#   2. msbuild ExtLibs/Utilities/MissionPlanner.Utilities.csproj under mono - GeoRefImageBase.cs,
#      ImageProjection.cs, srtm.cs, DFLogBuffer.cs are all in it, with MetaDataExtractor, SharpKml,
#      ExifLibNet and MAVLink beside it;
#   3. mcs GeorefOracle.cs against the result;
#   4. unzip testdata/srtm/S28E153.hgt.zip into a scratch terrain directory (the recorded flight is
#      over Brisbane, so every footprint's ground comes from that tile);
#   5. relabel camera.bin's CAM messages as TRIG for the TRIG case;
#   6. run every case over testdata/georef/camera.bin, camera.tlog and the photos, and swap the
#      output in as golden/.
#
# The inputs: camera.bin and camera.tlog are one ArduCopter SITL flight over Brisbane with a
# camera (CAM1_TYPE 1) told to take a picture every two seconds, one pair 0.3 s apart - flown by
# testdata/georef/fly.py with georef.parm as extra defaults; photos/ are the 25 synthetic JPEGs
# crates/mp-georef/tests/common/photos.rs makes, one per picture.
#
# Everything runs under TZ=UTC and the invariant culture (see GeorefOracle.cs). The build is cached
# per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires mono 6.12 (mono,
# msbuild, mcs), rsync, and the NuGet packages the csproj restores.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/referneces/missionplanner}"
DATA="$ROOT/testdata/georef"

[ -f "$MP/ExtLibs/Utilities/GeoRefImageBase.cs" ] || {
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
    msbuild -nologo -v:minimal -t:Restore,Build -p:Configuration=Release \
        -p:CopyLocalLockFileAssemblies=true \
        "$BUILD/src/ExtLibs/Utilities/MissionPlanner.Utilities.csproj" >"$BUILD/msbuild.log" 2>&1 || {
        tail -30 "$BUILD/msbuild.log" >&2
        exit 1
    }
fi

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY
export TZ=UTC

# Write to a scratch directory and swap it in, so a failed run leaves the old goldens untouched.
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# The harness runs from a copy of the build: ExifLibNet targets netstandard1.3 and saves through
# new FileStream(string, FileMode, FileAccess, FileShare), and the System.IO.FileSystem.Primitives
# the restore put beside the library is the portable one, whose FileMode is not mscorlib's - so
# under mono every save throws MissingMethodException (and WriteCoordinatesToImage's catch turns
# it into "There was a problem with image"). Mono's own facade forwards the types to mscorlib, as
# .NET Framework's binding does for the application. The shared cache is left as it is.
BIN="$TMP/bin"
mkdir -p "$BIN"
cp "$OUT"/*.dll "$BIN/"
cp "$(dirname "$NS")/System.IO.FileSystem.Primitives.dll" "$BIN/"
# CS1685 is mono's own Span clashing with System.Memory's, which nothing here uses.
mcs -nologo -nowarn:1685 -out:"$BIN/GeorefOracle.exe" \
    -r:"$BIN/MissionPlanner.Utilities.dll" -r:"$BIN/ICSharpCode.SharpZipLib.dll" \
    -r:"$BIN/MAVLink.dll" -r:"$BIN/System.Memory.dll" -r:"$NS" "$HERE/GeorefOracle.cs"
OUT="$BIN"

mono "$OUT/GeorefOracle.exe" srtm "$ROOT/testdata/srtm/S28E153.hgt.zip" "$TMP/srtm"
mono "$OUT/GeorefOracle.exe" mktrig "$DATA/camera.bin" "$TMP/trig.bin"
# The same flight as a text log, which DFLogBuffer reads line by line (DFLogBuffer.cs:144-201).
mono "$OUT/GeorefOracle.exe" bintolog "$DATA/camera.bin" "$TMP/camera.log"
GOLD="$TMP/golden"
mkdir -p "$GOLD"
case_() {
    local name="$1"
    shift
    mono "$OUT/GeorefOracle.exe" case "$GOLD/$name" "$@" "srtm=$TMP/srtm" >/dev/null
}
# The form's defaults: AMSL altitude, no shutter lag, 0.5 s minimum shutter interval.
case_ cam-amsl cam "$DATA/camera.bin" "$DATA/photos"
case_ cam-relalt-gpsalt cam "$DATA/camera.bin" "$DATA/photos" amsl=0 camgpsalt=1
case_ cam-lag cam "$DATA/camera.bin" "$DATA/photos" lag=150
case_ cam-drop cam "$DATA/camera.bin" "$DATA/photos" dropstart=2 dropend=3 minshutter=0
case_ cam-tlog cam "$DATA/camera.tlog" "$DATA/photos"
case_ time-bin time "$DATA/camera.bin" "$DATA/photos" offset=36003.3
case_ time-bin-cam time "$DATA/camera.bin" "$DATA/photos" offset=36004.3 usecam=1 amsl=0
case_ time-tlog time "$DATA/camera.tlog" "$DATA/photos" offset=36003.3 amsl=0
case_ trig trig "$TMP/trig.bin" "$DATA/photos" triggpsalt=1
case_ cam-textlog cam "$TMP/camera.log" "$DATA/photos" lag=150
# Runs that end early. testdata/dataflash.bin is a recorded flight with a CAM format and no CAM
# message, and no GPS position the reader keeps: with AMSL wanted there are no positions; without,
# there are no CAM messages, and PrevNowNext over none throws. Then TRIG messages one short of
# the photos.
case_ cam-nogps cam "$ROOT/testdata/dataflash.bin" "$DATA/photos"
case_ cam-nocam cam "$ROOT/testdata/dataflash.bin" "$DATA/photos" amsl=0
case_ trig-mismatch trig "$TMP/trig.bin" "$DATA/photos" dropend=1
# GPS2 asked for where the log has only GPS: no positions.
case_ time-gps2 time "$DATA/camera.bin" "$DATA/photos" offset=36003.3 gps=GPS2
mono "$OUT/GeorefOracle.exe" estimate "$GOLD/estimate.txt" "$DATA/camera.bin" "$DATA/photos" >/dev/null
mono "$OUT/GeorefOracle.exe" roundtrip "$GOLD/roundtrip.txt"
# The edge photos: their dates, and their geotagged copies south-east with an altitude, north-west
# at zero, and below zero (which ExifLibrary's UFraction32 refuses).
mono "$OUT/GeorefOracle.exe" phototimes "$DATA/edge" "$GOLD/edge-phototimes.txt"
mono "$OUT/GeorefOracle.exe" geotag "$DATA/edge" "$GOLD/edge-se" -27.4697998 153.0251002 65.18 >/dev/null
mono "$OUT/GeorefOracle.exe" geotag "$DATA/edge" "$GOLD/edge-nw" 51.4778 -0.0015 0 >/dev/null
mono "$OUT/GeorefOracle.exe" geotag "$DATA/edge" "$GOLD/edge-below" -27.4697998 153.0251002 -3.5 >/dev/null
rm -rf "$DATA/golden"
mv "$GOLD" "$DATA/golden"
find "$DATA/golden" -type d -exec chmod 755 {} +
echo "regenerated $(find "$DATA/golden" -type f | wc -l) goldens in $DATA/golden from Mission Planner $SHA" >&2
