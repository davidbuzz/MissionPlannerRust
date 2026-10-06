#!/usr/bin/env bash
# Regenerates testdata/dashware/golden from Mission Planner's own DashWare.Create
# (ExtLibs/Utilities/DashWare.cs), run by MpLog.cs's dashware verb under mono with the flight-mode
# names wired as MainV2 wires them: the CSV crates/mp-log/src/dashware.rs's tests hold the port to.
# Each line of testdata/dashware/cases.txt is name|log|types, log relative to testdata, types the
# "DashWare Types" answer or "-" for the whole log.
#
# The MissionPlanner.Utilities.dll it runs is the cached build of efb0801, which regen-grid.sh,
# regen-srtm.sh and regen-log.sh make; run one of them first if it is missing. DashWare.cs, DFLog.cs,
# BinaryLog.cs and ParameterMetaDataBackup.xml are the same at 5dbb2b0, where the reference tree now
# is; DFLogBuffer.cs's one change since (a12cdcaa8) is the index cache it keeps for a log of 300 MB
# or more, which no case here comes near.
#
# Each CSV is kept as mono writes it, its lines ended with mono's Environment.NewLine, "\n", where
# the C# on Windows writes "\r\n": a field can hold "\n" bytes of its own (a FILE message's data),
# so the line ends cannot be told from them afterwards. The tests write the same lines with "\n" to
# compare, and check Windows' "\r\n" on a case with no such field. A CSV over 1 MB is not kept:
# golden/<name>.sha256 holds its SHA-256 and its count of "\n" bytes instead. Requires mono (mcs,
# mono), sha256sum.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/references/missionplanner}"
SHA="${MP_ORACLE_SHA:-efb080190de0bf091f9aab982c8848a124de7588}"
OUT="${MP_ORACLE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/mp-csharp-reference}/$SHA/src/ExtLibs/Utilities/bin/Release/netstandard2.0"
[ -f "$OUT/MissionPlanner.Utilities.dll" ] || {
    echo "no MissionPlanner.Utilities.dll at $OUT: run regen-srtm.sh or regen-grid.sh first" >&2
    exit 1
}
NS="$(find /usr/lib/mono -path '*/4.5/Facades/netstandard.dll' | head -1)"
META="$MP/ParameterMetaDataBackup.xml"
# As regen-log.sh compiles it.
mcs -nologo -nowarn:1685 -out:"$OUT/MpLog.exe" \
    -r:"$OUT/MissionPlanner.Utilities.dll" -r:"$OUT/ICSharpCode.SharpZipLib.dll" \
    -r:"$OUT/System.Memory.dll" -r:System.Xml.Linq -r:"$NS" "$HERE/MpLog.cs"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export TZ=UTC
grep -v '^#' "$ROOT/testdata/dashware/cases.txt" | while IFS='|' read -r name log types; do
    [ -n "$name" ] || continue
    mono "$OUT/MpLog.exe" dashware "$ROOT/testdata/$log" "$work/$name.csv" "$types" "$META" >/dev/null
done
golden="$ROOT/testdata/dashware/golden"
rm -rf "$golden"
mkdir -p "$golden"
for csv in "$work"/*.csv; do
    name="$(basename "$csv" .csv)"
    if [ "$(stat -c %s "$csv")" -gt 1048576 ]; then
        printf '%s %s\n' "$(sha256sum "$csv" | cut -d' ' -f1)" "$(wc -l < "$csv")" > "$golden/$name.sha256"
    else
        cp "$csv" "$golden/$name.csv"
    fi
done
ls -la "$golden"
