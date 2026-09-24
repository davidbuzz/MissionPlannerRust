#!/usr/bin/env bash
# Regenerates testdata/comms/golden from Mission Planner's own CommsNTRIP, WebSocket and
# UdpSerialConnect, for crates/mp-transport/tests/csharp_goldens.rs.
#
#   1. csc the three transports (ExtLibs/Comms/CommsNTRIP.cs, CommsWebSocket.cs,
#      CommsUDPSerialConnect.cs), what they stand on (CommsBase.cs, CommsStream.cs,
#      ExtLibs/Interfaces/ICommsSerial.cs) and MpComms.cs into one executable, against log4net -
#      straight from the reference tree, which is only read;
#   2. run each verb of MpComms.exe, whose peers are all on 127.0.0.1;
#   3. swap the output in as golden/.
#
# Requires mono 6.12 (mono, csc) and log4net 2.0.13 in ~/.nuget/packages (what the csproj restores;
# regen-grid.sh's msbuild restore puts it there). Mono's System.Net.WebSockets is corefx's
# ManagedWebSocket, so ws.txt is what .NET's managed client does; the Windows build of Mission
# Planner, on .NET Framework, has its own client, which this cannot run.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/referneces/missionplanner}"
DATA="$ROOT/testdata/comms"
COMMS="$MP/ExtLibs/Comms"

[ -f "$COMMS/CommsNTRIP.cs" ] || {
    echo "Mission Planner source not found at $MP (set MP_SRC)" >&2
    exit 1
}
LOG4NET="${LOG4NET:-$HOME/.nuget/packages/log4net/2.0.13/lib/net45/log4net.dll}"
[ -f "$LOG4NET" ] || { echo "log4net not found at $LOG4NET (set LOG4NET)" >&2; exit 1; }

BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
cp "$LOG4NET" "$BUILD/"
csc -nologo -nowarn:1998,4014,0168,0219,0414,0649,0169 -out:"$BUILD/MpComms.exe" \
    -r:"$BUILD/log4net.dll" -r:System.Net.Http.dll \
    "$COMMS/CommsBase.cs" "$COMMS/CommsStream.cs" "$MP/ExtLibs/Interfaces/ICommsSerial.cs" \
    "$COMMS/CommsNTRIP.cs" "$COMMS/CommsWebSocket.cs" "$COMMS/CommsUDPSerialConnect.cs" \
    "$HERE/MpComms.cs" >"$BUILD/csc.log" 2>&1 || { cat "$BUILD/csc.log" >&2; exit 1; }

# Write to a scratch directory and swap it in, so a failed run leaves the old goldens untouched.
TMP="$(mktemp -d "$DATA/.golden.XXXXXX")"
trap 'rm -rf "$BUILD" "$TMP"' EXIT
mono "$BUILD/MpComms.exe" ntrip "$DATA/ntrip-cases.txt" "$TMP"
mono "$BUILD/MpComms.exe" gga "$DATA/gga-positions.txt" "$TMP"
mono "$BUILD/MpComms.exe" reconnect - "$TMP"
mono "$BUILD/MpComms.exe" ws - "$TMP"
mono "$BUILD/MpComms.exe" udp - "$TMP"
rm -rf "$DATA/golden"
mv "$TMP" "$DATA/golden"
chmod 755 "$DATA/golden"
SHA="$(git -C "$MP" rev-parse HEAD 2>/dev/null || echo unknown)"
echo "regenerated $(ls "$DATA/golden" | wc -l) goldens in $DATA/golden from Mission Planner $SHA" >&2
