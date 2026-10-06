#!/usr/bin/env bash
# Regenerates testdata/cot/golden from Controls/SerialOutputCoT.cs's getXmlString, re-hosted in
# CotOracle.cs over the CoT classes of the pinned tree (ExtLibs/Utilities/CoT): the Cursor-on-Target
# text crates/mp-gui/src/config/cot_output.rs's tests hold the port to. Requires mono (mcs, mono).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MP="${MP_SRC:-$ROOT/references/missionplanner}"
[ -f "$MP/ExtLibs/Utilities/CoT/event.cs" ] || { echo "no CoT classes under $MP (set MP_SRC)" >&2; exit 1; }
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mcs -nologo -r:System.Xml.dll -out:"$work/CotOracle.exe" "$HERE/CotOracle.cs" "$MP"/ExtLibs/Utilities/CoT/*.cs
rm -rf "$ROOT/testdata/cot/golden"
mono "$work/CotOracle.exe" "$ROOT/testdata/cot/cases.txt" "$ROOT/testdata/cot/golden"
ls "$ROOT/testdata/cot/golden"
