#!/usr/bin/env bash
# Runs one fuzz target against the committed seeds.
#
# usage: tools/fuzz.sh <target> [libfuzzer args...]
#        tools/fuzz.sh message_decode -max_total_time=180
#
# Two directories are passed to libFuzzer: fuzz/corpus/<target>, where it writes what it finds,
# and fuzz/seeds/<target>, the committed starting point. The corpus directory is gitignored and
# has to be created here - libFuzzer refuses to start if a corpus directory is missing, and on a
# fresh checkout it always is.
#
# Needs a nightly toolchain and cargo-fuzz; see fuzz/README.md.
set -euo pipefail

TARGET="${1:?usage: fuzz.sh <target> [libfuzzer args...]}"
shift

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SEEDS="$ROOT/fuzz/seeds/$TARGET"
CORPUS="$ROOT/fuzz/corpus/$TARGET"

if [ ! -d "$SEEDS" ]; then
    echo "no seeds for '$TARGET' in $SEEDS" >&2
    echo "known targets:" >&2
    ls "$ROOT/fuzz/seeds" 2>/dev/null | sed 's/^/  /' >&2
    exit 1
fi

mkdir -p "$CORPUS"
cd "$ROOT"
exec cargo +nightly fuzz run "$TARGET" "$CORPUS" "$SEEDS" -- -print_final_stats=1 "$@"
