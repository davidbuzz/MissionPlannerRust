#!/usr/bin/env bash
# Measures Deliverable 1's build budgets: a cold `cargo check --workspace` and the incremental check after one
# leaf file changes (DELIVERABLES.md Deliverable 1 asks for under 90 s cold and under 5 s incremental on the
# dev box).
#
# The cold check goes into a target directory of its own, so the figure is the whole workspace
# from nothing with the dependencies already fetched. Then two incremental checks: a comment
# appended to a leaf of mp-gui (the application crate, the top of the graph: only it is checked
# again) and a comment appended to mp-units' lib.rs (an L0 crate: everything above it is checked
# again), each reverted afterwards. The directory is removed at the end. Nothing else may build
# while this runs - the figures are of this machine idle - and a release build must not run
# beside the Windows VM.
#
#   tools/build-budget.sh [target-dir]      default a directory under the scratch area
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export CARGO_TARGET_DIR="${1:-${TMPDIR:-/tmp}/mp-build-budget-$$}"
rm -rf "$CARGO_TARGET_DIR"
trap 'rm -rf "$CARGO_TARGET_DIR"' EXIT

time_check() {
    # The checks' own lines are noise here; cargo's summary and the timing stay.
    /usr/bin/time -f "$1: %e s wall, %M KB max rss" cargo check --workspace 2>&1 \
        | grep -v '^\s*Checking\|^\s*Compiling' | tail -3
}

probe() {
    # A comment appended to the file, the check timed, the comment removed again.
    local file="$1" label="$2"
    echo "// build budget probe" >> "$file"
    time_check "$label"
    sed -i '$ d' "$file"
    if ! git diff --quiet -- "$file"; then
        echo "warning: $file is not as it was" >&2
    fi
}

echo "== cold check --workspace, $(nproc) jobs, into $CARGO_TARGET_DIR"
time_check "cold check"
echo "== incremental check, mp-gui leaf touched (crates/mp-gui/src/raw_sensor.rs)"
probe crates/mp-gui/src/raw_sensor.rs "incremental check (mp-gui touched)"
echo "== incremental check, L0 crate touched (crates/mp-units/src/lib.rs)"
probe crates/mp-units/src/lib.rs "incremental check (mp-units touched)"
echo "== target directory: $(du -sh "$CARGO_TARGET_DIR" | cut -f1)"
