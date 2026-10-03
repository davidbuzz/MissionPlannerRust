#!/usr/bin/env bash
# Builds the stripped release binaries and reports what a machine needs to run them; with `deb`,
# packages them.
#
# usage: tools/package.sh                 the two binaries into dist/, and what they need
#        tools/package.sh deb [--no-build] dist/missionplanner-rust_<version>_<arch>.deb as well:
#                                         the binaries in /usr/bin, the desktop entry and icon
#                                         (tools/packaging/), the licence, Depends from what the
#                                         planner links (Deliverable 20); --no-build packages the release
#                                         binaries already in target/release
#
# Deliverable 20 asks for signed artefacts on three systems, auto-update and crash reports: the update and the
# crash reports are the program's (crates/mp-update, crates/mp-gui/src/crash.rs); of the artefacts
# this is the Debian package, unsigned - signing, the AppImage, the MSI and the .dmg are not here.
# The package is held to a clean container by crates/mp-cli/tests/package_smoke.rs.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/dist"
mkdir -p "$OUT"

MODE="${1:-}"
BUILD=1
for arg in "$@"; do
    [ "$arg" = "--no-build" ] && BUILD=0
done

if [ "$BUILD" = 1 ]; then
    echo "building..."
    cargo build --release --manifest-path "$ROOT/Cargo.toml" -p mp-gui --bin planner -p mp-cli --bin headless-planner
fi

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

[ "$MODE" = "deb" ] || exit 0

# ---- The Debian package ----------------------------------------------------------------------
VERSION=$(grep -m1 '^version = ' "$ROOT/Cargo.toml" | sed 's/.*"\(.*\)"/\1/')
ARCH=$(dpkg --print-architecture)
NAME="missionplanner-rust"
STAGE="$OUT/deb/${NAME}_${VERSION}_${ARCH}"
DEB="$OUT/${NAME}_${VERSION}_${ARCH}.deb"
rm -rf "$STAGE"
mkdir -p "$STAGE/DEBIAN" "$STAGE/usr/bin" "$STAGE/usr/share/applications" \
    "$STAGE/usr/share/icons/hicolor/128x128/apps" "$STAGE/usr/share/doc/$NAME"
install -m 0755 "$OUT/planner" "$OUT/headless-planner" "$STAGE/usr/bin/"
install -m 0644 "$ROOT/tools/packaging/planner.desktop" "$STAGE/usr/share/applications/"
install -m 0644 "$ROOT/tools/packaging/planner.png" "$STAGE/usr/share/icons/hicolor/128x128/apps/"
install -m 0644 "$ROOT/LICENSE" "$STAGE/usr/share/doc/$NAME/copyright"

# What the planner links, as the packages that hold them here; the Vulkan loader and a driver are
# loaded at run time by the renderer, and a font by the text system.
DEPENDS=$(ldd "$OUT/planner" | awk '/=>/ {print $3}' | while read -r lib; do
    dpkg -S "$(realpath "$lib")" 2>/dev/null | cut -d: -f1 | head -1
done | sort -u | grep -v '^$' | paste -sd, - | sed 's/,/, /g')
GLIBC_MIN=${GLIBC#GLIBC_}
DEPENDS=$(echo "$DEPENDS" | sed "s/libc6/libc6 (>= $GLIBC_MIN)/")
SIZE=$(du -sk "$STAGE/usr" | cut -f1)
cat > "$STAGE/DEBIAN/control" <<CONTROL
Package: $NAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Installed-Size: $SIZE
Maintainer: David Buzz <davidbuzz@gmail.com>
Homepage: https://github.com/davidbuzz/MissionPlannerRust
Depends: $DEPENDS, libvulkan1
Recommends: mesa-vulkan-drivers, fonts-dejavu-core
Description: Mission Planner, the ArduPilot ground station, in Rust
 The planner: flight data, flight planning, setup, configuration and tuning of
 ArduPilot vehicles over MAVLink, with the simulator screen and the log tools;
 and headless-planner, the same over the command line.
CONTROL
(cd "$STAGE" && find usr -type f -exec md5sum {} \; > DEBIAN/md5sums)
fakeroot dpkg-deb --build --root-owner-group "$STAGE" "$DEB" >/dev/null
rm -rf "$OUT/deb"
echo
dpkg-deb --info "$DEB"
echo
dpkg-deb --contents "$DEB" | awk '{print $1, $6}'
echo
echo "package: $DEB ($(du -h "$DEB" | cut -f1))"
