#!/usr/bin/env bash
# Regenerates the C# reference corpora used by the differential tests (Deliverable 19).
# Requires mono, and MAVLink.dll: from a Mission Planner binary distribution (MP_DIST), or - with
# MP_DIST unset and MP_SRC naming the pinned clone (xtask/src/upstream.rs) - built here from the
# clone's ExtLibs/Mavlink with mcs, so the reference is the pinned commit's (done so on 2026-10-06,
# when the pin moved to 5dbb2b0). Building needs Newtonsoft.Json 13.0.3 and
# System.Runtime.CompilerServices.Unsafe in ~/.nuget/packages, as MAVLink.csproj restores them.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../../testdata/mavlink"
NS="$(find /usr/lib/mono -name netstandard.dll 2>/dev/null | head -1)"

if [ -z "${MP_DIST:-}" ] && [ -n "${MP_SRC:-}" ]; then
    MP="$(mktemp -d)"
    trap 'rm -rf "$MP"' EXIT
    json="$HOME/.nuget/packages/newtonsoft.json/13.0.3/lib/net45/Newtonsoft.Json.dll"
    unsafe="$(ls "$HOME"/.nuget/packages/system.runtime.compilerservices.unsafe/*/lib/net46*/System.Runtime.CompilerServices.Unsafe.dll | tail -1)"
    # MAVLink.csproj: every .cs beside it (not mavlink/ or pymavlink/), unsafe, TRACE;UNSAFE.
    mcs -target:library -unsafe -define:'TRACE;UNSAFE' -nowarn:612,618,414,169,649,219,168,162 \
        -out:"$MP/MAVLink.dll" -r:"$json" -r:"$unsafe" -r:System.Numerics.dll \
        -r:System.Runtime.Serialization.dll "$MP_SRC"/ExtLibs/Mavlink/*.cs
    cp "$json" "$unsafe" "$MP/"
    echo "built MAVLink.dll from $MP_SRC at $(git -C "$MP_SRC" rev-parse --short=7 HEAD)" >&2
else
    MP="${MP_DIST:-$HOME/Downloads/MissionPlanner-latest}"
fi

[ -f "$MP/MAVLink.dll" ] || { echo "MAVLink.dll not found in $MP (set MP_DIST, or MP_SRC to build it)" >&2; exit 1; }

mcs -out:"$HERE/MpRefDump.exe" -r:"$MP/MAVLink.dll" -r:"$NS" "$HERE/MpRefDump.cs"
cp -f "$MP/MAVLink.dll" "$HERE/"

mono "$HERE/MpRefDump.exe" infos > "$OUT/binary_message_infos.csv"
for tlog in "$OUT"/*.tlog; do
    [ -e "$tlog" ] || continue
    echo "dumping $(basename "$tlog")" >&2
    mono "$HERE/MpRefDump.exe" tlog "$tlog" > "$tlog.csharp.csv"
done
echo "regenerated corpora in $OUT" >&2
