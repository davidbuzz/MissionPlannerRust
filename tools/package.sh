#!/usr/bin/env bash
# Builds a stripped release binary and reports what a machine needs to run it.
#
# usage: tools/package.sh
#
# Not a package. D20 - installers, `.deb`, AppImage, signing, auto-update - is not started, and
# this does not pretend otherwise. What it does is the thing that is useful today: produce the one
# file somebody can be sent, and say out loud what will stop it running, because both of those are
# properties of the machine it was built on rather than of the code.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/dist"
mkdir -p "$OUT"

echo "building..."
cargo build --release --manifest-path "$ROOT/Cargo.toml"

for BIN in planner headless-planner; do
    SRC="$ROOT/target/release/$BIN"
    [ -x "$SRC" ] || { echo "not built: $SRC" >&2; exit 1; }
    cp "$SRC" "$OUT/$BIN"
    # The release profile carries `debug = 1` on purpose, for symbolicated crash reports. That is
    # right for a build being debugged and wrong for one being sent to somebody: it is most of the
    # file size and none of the function.
    strip "$OUT/$BIN"
done

echo
printf '%-12s %10s  %10s\n' "binary" "built" "stripped"
for BIN in planner headless-planner; do
    BUILT=$(stat -c%s "$ROOT/target/release/$BIN")
    SHIPPED=$(stat -c%s "$OUT/$BIN")
    printf '%-12s %9.1fM  %9.1fM\n' "$BIN" \
        "$(echo "$BUILT" | awk '{print $1/1048576}')" \
        "$(echo "$SHIPPED" | awk '{print $1/1048576}')"
done

# The two things that stop it running somewhere else, both decided by this machine rather than by
# the code. Printed every time, because a binary handed over without them is a binary that fails
# on somebody else's desktop with a loader error nobody can read.
echo
GLIBC=$(objdump -T "$OUT/planner" 2>/dev/null | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sort -V | tail -1)
echo "needs $GLIBC or newer - built on $(. /etc/os-release && echo "$PRETTY_NAME")"
case "$GLIBC" in
    GLIBC_2.39) echo "  that is Ubuntu 24.04. It will NOT start on 22.04 or older." ;;
    GLIBC_2.35) echo "  that is Ubuntu 22.04." ;;
esac

echo
echo "needs these shared libraries:"
ldd "$OUT/planner" | awk '{print $1}' | grep -E '^lib' | sort | sed 's/^/  /'
echo
echo "on a bare Ubuntu that usually means:"
echo "  sudo apt install libxkbcommon-x11-0 libxcb-xkb1 libbsd0"
echo
echo "written to $OUT"
