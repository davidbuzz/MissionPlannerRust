#!/usr/bin/env bash
# Regenerates testdata/currentstate from Mission Planner's own CurrentState (ExtLibs/ArduPilot/
# CurrentState.cs), for crates/mp-vehicle/tests/current_state_oracle.rs:
#   <log>.csv     the fields of vehicle 1:1 after each packet of testdata/mavlink/<log>.tlog, played
#                 through MAVLinkInterface and UpdateCurrentSettings (MpState.cs `tlog`)
#   synthetic.tlog, synthetic.csv   a flight the harness writes (MpState.cs `mktlog`) to reach what
#                 the recorded ones do not, and the same for it
#   getters.csv   the derived getters on a grid of inputs (MpState.cs `getters`)
#   fence.csv     GeoFenceDist for a set of fences and positions (MpState.cs `fence`)
#
# The build is regen-grid.sh's with one project more: ExtLibs is copied out of tree, and msbuild
# builds ExtLibs/ArduPilot/MissionPlanner.ArduPilot.csproj - CurrentState, MAVLinkInterface and
# MAVState - which builds MissionPlanner.Utilities, MAVLink, Comms, Strings and GMap.NET.Core as its
# references. Its Resources.resx embeds ../../../Resources/quad2.png, so that one file is copied too.
# Then mcs MpState.cs against the result.
#
# The build is cached per Mission Planner commit ($MP_ORACLE_CACHE, default ~/.cache). Requires
# mono 6.12 (mono, msbuild, mcs), rsync, and the NuGet packages the csproj restores - from the
# network, or already in ~/.nuget/packages.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/references/missionplanner}"
DATA="$ROOT/testdata/currentstate"
LOGS=("$ROOT/testdata/mavlink/autotest.tlog" "$ROOT/testdata/mavlink/multisystem.tlog")

[ -f "$MP/ExtLibs/ArduPilot/CurrentState.cs" ] || {
    echo "Mission Planner source not found at $MP (set MP_SRC)" >&2
    exit 1
}
SHA="${MP_SHA:-$(git -C "$MP" rev-parse HEAD)}"
BUILD="${MP_ORACLE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/mp-csharp-reference}/$SHA"
OUT="$BUILD/src/ExtLibs/ArduPilot/bin/Release/netstandard2.0"
NS="$(find /usr/lib/mono -path '*/4.5/Facades/netstandard.dll' | head -1)"
[ -n "$NS" ] || { echo "mono's netstandard facade not found under /usr/lib/mono" >&2; exit 1; }

if [ ! -f "$OUT/MissionPlanner.ArduPilot.dll" ]; then
    echo "building MissionPlanner.ArduPilot at $SHA in $BUILD" >&2
    mkdir -p "$BUILD/src/Resources"
    if [ ! -d "$BUILD/src/ExtLibs" ]; then
        rsync -a --delete --exclude bin --exclude obj "$MP/ExtLibs" "$BUILD/src/"
        cp "$MP/nuget.config" "$BUILD/src/"
    fi
    cp "$MP/Resources/quad2.png" "$BUILD/src/Resources/"
    msbuild -nologo -v:minimal -t:Restore,Build -p:Configuration=Release \
        -p:CopyLocalLockFileAssemblies=true \
        "$BUILD/src/ExtLibs/ArduPilot/MissionPlanner.ArduPilot.csproj" >"$BUILD/msbuild-ardupilot.log" 2>&1 || {
        tail -30 "$BUILD/msbuild-ardupilot.log" >&2
        exit 1
    }
fi

# CS1685 is mono's own Span clashing with System.Memory's, which nothing here uses.
mcs -nologo -nowarn:1685 -out:"$OUT/MpState.exe" \
    -r:"$OUT/MissionPlanner.ArduPilot.dll" -r:"$OUT/MissionPlanner.Utilities.dll" \
    -r:"$OUT/MAVLink.dll" -r:"$OUT/GMap.NET.Core.dll" -r:"$OUT/System.Memory.dll" \
    -r:"$NS" "$HERE/MpState.cs"

# Write to a scratch directory and swap it in, so a failed run leaves the old data untouched.
mkdir -p "$DATA"
TMP="$(mktemp -d "$DATA/.new.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/home"
# lastlogread is local time (MAVLinkInterface.cs:6557), and Settings reads config.xml from the
# user's data directory: UTC, and an empty home.
export TZ=UTC HOME="$TMP/home"
# The synthetic flight is written by the harness, committed as the input, and played like the rest.
mono "$OUT/MpState.exe" mktlog "$TMP/synthetic.tlog"
for log in "${LOGS[@]}" "$TMP/synthetic.tlog"; do
    mono "$OUT/MpState.exe" tlog "$log" "$TMP/$(basename "$log" .tlog).csv"
done
mono "$OUT/MpState.exe" getters "$TMP/getters.csv"
mono "$OUT/MpState.exe" fence "$TMP/fence.csv"
rm -rf "$TMP/home"

for f in "$TMP"/*.csv "$TMP/synthetic.tlog"; do
    mv -f "$f" "$DATA/"
done
echo "regenerated $DATA" >&2
