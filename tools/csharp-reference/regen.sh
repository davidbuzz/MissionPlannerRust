#!/usr/bin/env bash
# Regenerates the C# reference corpora used by the differential tests (Deliverable 19).
# Requires mono and a Mission Planner binary distribution.
set -euo pipefail
MP="${MP_DIST:-$HOME/Downloads/MissionPlanner-latest}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../../testdata/mavlink"
NS="$(find /usr/lib/mono -name netstandard.dll 2>/dev/null | head -1)"

[ -f "$MP/MAVLink.dll" ] || { echo "MAVLink.dll not found in $MP (set MP_DIST)" >&2; exit 1; }

mcs -out:"$HERE/MpRefDump.exe" -r:"$MP/MAVLink.dll" -r:"$NS" "$HERE/MpRefDump.cs"
cp -f "$MP/MAVLink.dll" "$HERE/"

mono "$HERE/MpRefDump.exe" infos > "$OUT/binary_message_infos.csv"
for tlog in "$OUT"/*.tlog; do
    [ -e "$tlog" ] || continue
    echo "dumping $(basename "$tlog")" >&2
    mono "$HERE/MpRefDump.exe" tlog "$tlog" > "$tlog.csharp.csv"
done
echo "regenerated corpora in $OUT" >&2
