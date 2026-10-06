#!/usr/bin/env bash
# Copyright (C) 2026 David "Buzz" Bussenschutt
# This file is part of MissionPlannerRust; see LICENSE (GPL-3.0-only).
# SPDX-License-Identifier: GPL-3.0-only
#
# This repository's MAVLink binding against tridge's generated Rust binding (pymavlink pull
# request 1303), every message of Mission Planner's all.xml, both ways, byte for byte -
# harness/src/main.rs says what is compared. Exit 1 on any over-the-wire difference.
#
# The reference is the optional submodule third_party/pymavlink, not checked out by a clone:
#     git submodule update --init --checkout third_party/pymavlink
# Without it this says so and exits 0. The XML is the one our dialect is generated from,
# $MP_SRC (default references/missionplanner)/ExtLibs/Mavlink/message_definitions/all.xml.
#
# Usage: tools/wire-agreement/run.sh [random patterns per message, default 4]
# Builds a debug binary in target/wire-agreement/target, CARGO_BUILD_JOBS (default 4) at a time,
# with $CARGO (default cargo).
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
here=$root/tools/wire-agreement
pymavlink=$root/third_party/pymavlink
xml=${MP_SRC:-$root/references/missionplanner}/ExtLibs/Mavlink/message_definitions/all.xml
out=$root/target/wire-agreement
randoms=${1:-4}

if [ ! -f "$pymavlink/generator/mavgen_rust.py" ]; then
    echo "wire agreement: skipped - the reference binding's submodule is not checked out" >&2
    echo "    git submodule update --init --checkout third_party/pymavlink" >&2
    exit 0
fi
if [ ! -f "$xml" ]; then
    echo "wire agreement: no $xml (set MP_SRC to a Mission Planner checkout)" >&2
    exit 1
fi
mkdir -p "$out"

# The reference crate, replaced only when what mavgen writes has changed, so cargo does not
# rebuild it every run.
rm -rf "$out/generated.new"
PYTHONPATH=$root/third_party python3 -m pymavlink.tools.mavgen --lang=Rust --wire-protocol=2.0 \
    --output="$out/generated.new" "$xml" > "$out/mavgen.log" 2>&1 \
    || { cat "$out/mavgen.log" >&2; exit 1; }
if [ -d "$out/mavlink-generated" ] && diff -rq "$out/generated.new" "$out/mavlink-generated" > /dev/null; then
    rm -rf "$out/generated.new"
else
    rm -rf "$out/mavlink-generated"
    mv "$out/generated.new" "$out/mavlink-generated"
fi

PYTHONPATH=$root/third_party python3 "$here/cases.py" "$xml" "$randoms" > "$out/cases.txt" 2> "$out/cases.log" \
    || { cat "$out/cases.log" >&2; exit 1; }

# The product's lock file, so the harness builds our crates with the versions the product does.
cp "$root/Cargo.lock" "$here/harness/Cargo.lock"
CARGO_TARGET_DIR=$out/target "${CARGO:-cargo}" run --quiet --manifest-path "$here/harness/Cargo.toml" \
    -j "${CARGO_BUILD_JOBS:-4}" -- "$out/cases.txt"
