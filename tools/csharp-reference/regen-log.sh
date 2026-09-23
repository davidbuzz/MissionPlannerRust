#!/usr/bin/env bash
# Regenerates testdata/dataflash/golden from Mission Planner's own dataflash converters: the four
# buttons of the flight screen's DataFlash Logs page (GCSViews/FlightData.cs), for D14 and
# crates/mp-log/tests/{convert,matlab,analysis}.rs and crates/mp-kml/tests/dflog.rs.
#
# The build is regen-grid.sh's, step for step and in the same cache, so whichever script runs first
# builds ExtLibs/Utilities once for all of them:
#   1. copy referneces/missionplanner/ExtLibs out of tree, so the reference tree stays read-only;
#   2. msbuild ExtLibs/Utilities/MissionPlanner.Utilities.csproj under mono - BinaryLog.cs,
#      DFLogBuffer.cs, LogOutput.cs and MatLab.cs are all in it, with KMLib, csmatio and SharpZipLib
#      beside it;
#   3. mcs MpLog.cs against the result;
#   4. run every verb over testdata/dataflash.bin and testdata/dataflash_damaged.bin, and the
#      loganalysis verb over the analyzer's own example output, and swap the output in as golden/:
#        <log>.log               bintolog: BinaryLog.ConvertBin
#        kml/<log>.bin.*          dflogtokml: every file LogOutput.writeKML leaves beside the log -
#                                 .gpx, .param, <n>wp.txt, <n>rally.txt, .obs when there is raw GPS -
#                                 the .kml out of the .kmz, and the .kmz's entry list
#        kml/resync/, kml/text/   the same for the damaged log from its first header, and for the
#                                 converted dataflash.log read as text
#        matlab/<log>.bin-<n>.mat matlab: MatLab.ProcessLog, and matlab/resync/ likewise
#        edge.log, kml/edge/, matlab/edge/   all three for edge.bin, which the harness writes
#        kml/synthetic/, matlab/synthetic/   both for the hand-written synthetic.log
#        loganalysis/example_output.txt   LogAnalyzer.Results and the report text
#
# block_plane_0.dae goes beside MpLog.exe, which is Settings.GetRunningDirectory() for the harness,
# because that is where writeKML looks for the model it puts in the .kmz (LogOutput.cs:1130-1152) and
# the application ships it there. Everything runs under TZ=UTC (the GPX times are local) and in a
# private TMPDIR (MatLab.DoubleList spills to temporary files).
#
# The build is cached per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires
# mono 6.12 (mono, msbuild, mcs), rsync, and the NuGet packages the csproj restores - from the
# network, or already in ~/.nuget/packages.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/referneces/missionplanner}"
DATA="$ROOT/testdata/dataflash"
LOGS=("$ROOT/testdata/dataflash.bin" "$ROOT/testdata/dataflash_damaged.bin")

[ -f "$MP/ExtLibs/Utilities/BinaryLog.cs" ] || {
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

# SharpZipLib reads the .kmz back; CS1685 is mono's own Span clashing with System.Memory's, which
# nothing here uses.
mcs -nologo -nowarn:1685 -out:"$OUT/MpLog.exe" \
    -r:"$OUT/MissionPlanner.Utilities.dll" -r:"$OUT/ICSharpCode.SharpZipLib.dll" \
    -r:"$OUT/System.Memory.dll" -r:System.Xml.Linq -r:"$NS" "$HERE/MpLog.cs"
cp -f "$MP/block_plane_0.dae" "$OUT/"

# Write to a scratch directory and swap it in, so a failed run leaves the old goldens untouched.
TMP="$(mktemp -d "$DATA/.golden.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/kml" "$TMP/matlab" "$TMP/loganalysis" "$TMP/tmp"
export TZ=UTC TMPDIR="$TMP/tmp"
META="$MP/ParameterMetaDataBackup.xml"
for log in "${LOGS[@]}"; do
    name="$(basename "$log" .bin)"
    mono "$OUT/MpLog.exe" bintolog "$log" "$TMP/$name.log" "$META" >/dev/null
    # Both of these write beside the log they are given, as the buttons do.
    cp "$log" "$TMP/kml/"
    mono "$OUT/MpLog.exe" dflogtokml "$TMP/kml/$name.bin" "$META" >/dev/null
    rm "$TMP/kml/$name.bin"
    cp "$log" "$TMP/matlab/"
    mono "$OUT/MpLog.exe" matlab "$TMP/matlab/$name.bin" "$META" >/dev/null
    rm "$TMP/matlab/$name.bin"
done
# dataflash_damaged.bin opens with 18 bytes of garbage, so DFLogBuffer reads it as a text log
# (DFLogBuffer.cs:77-83) and neither writeKML nor ProcessLog sees one message of it. The same log from
# its first header is the one fixture with a GPS fix, POS and attitude that goes the binary way, with
# every unknown type and torn record the damage left; crates/mp-kml/tests/dflog.rs and
# crates/mp-log/tests/matlab.rs cut it the same way.
mkdir "$TMP/kml/resync" "$TMP/matlab/resync"
tail -c +19 "$ROOT/testdata/dataflash_damaged.bin" >"$TMP/kml/resync/resync.bin"
cp "$TMP/kml/resync/resync.bin" "$TMP/matlab/resync/"
mono "$OUT/MpLog.exe" dflogtokml "$TMP/kml/resync/resync.bin" "$META" >/dev/null
mono "$OUT/MpLog.exe" matlab "$TMP/matlab/resync/resync.bin" "$META" >/dev/null
rm "$TMP/kml/resync/resync.bin" "$TMP/matlab/resync/resync.bin"
# A text log goes line by line through processLine (FlightData.cs:1178-1186): the converted one.
mkdir "$TMP/kml/text"
cp "$TMP/dataflash.log" "$TMP/kml/text/"
mono "$OUT/MpLog.exe" dflogtokml "$TMP/kml/text/dataflash.log" "$META" >/dev/null
rm "$TMP/kml/text/dataflash.log"
# synthetic.log is written by hand to reach what no recorded fixture does: raw GNSS for the RINEX
# file, rally points, missions re-uploaded, a mode change between fixes, aircraft that do and do not
# move, POS thinning, instance columns and array fields for the .mat.
for verb in kml matlab; do
    mkdir "$TMP/$verb/synthetic"
    cp "$DATA/synthetic.log" "$TMP/$verb/synthetic/"
done
mono "$OUT/MpLog.exe" dflogtokml "$TMP/kml/synthetic/synthetic.log" "$META" >/dev/null
mono "$OUT/MpLog.exe" matlab "$TMP/matlab/synthetic/synthetic.log" "$META" >/dev/null
rm "$TMP/kml/synthetic/synthetic.log" "$TMP/matlab/synthetic/synthetic.log"
# edge.bin is the harness's own (MpLog.cs MkEdge): every field type at its extremes, arbitrary float
# and double bit patterns, and each way BinaryLog skips, repeats or cuts a message.
mono "$OUT/MpLog.exe" mkedge "$DATA/edge.bin"
mono "$OUT/MpLog.exe" bintolog "$DATA/edge.bin" "$TMP/edge.log" "$META" >/dev/null
for verb in kml matlab; do
    mkdir "$TMP/$verb/edge"
    cp "$DATA/edge.bin" "$TMP/$verb/edge/"
done
mono "$OUT/MpLog.exe" dflogtokml "$TMP/kml/edge/edge.bin" "$META" >/dev/null
mono "$OUT/MpLog.exe" matlab "$TMP/matlab/edge/edge.bin" "$META" >/dev/null
rm "$TMP/kml/edge/edge.bin" "$TMP/matlab/edge/edge.bin"
cp "$MP/LogAnalyzer/py2exe/example_output.xml" "$DATA/example_output.xml"
mono "$OUT/MpLog.exe" loganalysis "$DATA/example_output.xml" "$TMP/loganalysis/example_output.txt"
rmdir "$TMP/tmp" 2>/dev/null || rm -rf "$TMP/tmp"
rm -rf "$DATA/golden"
mv "$TMP" "$DATA/golden"
find "$DATA/golden" -type d -exec chmod 755 {} +
trap - EXIT
echo "regenerated $(find "$DATA/golden" -type f | wc -l) goldens in $DATA/golden from Mission Planner $SHA" >&2
