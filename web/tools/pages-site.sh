#!/usr/bin/env bash
# The planner's web page as a static site, for GitHub Pages (.github/workflows/pages.yml) or any
# server that cannot send headers:   tools/pages-site.sh <out-dir>
# From www/ as tools/planner-wasm.sh build and wasm-bindgen leave it (pkg-planner/ built), with the
# SITL's files copied in place of www/sitl's link, and
# coi-serviceworker: gpui_web's threads need SharedArrayBuffer, which a page has only when it is
# cross-origin isolated, by the COOP and COEP headers www/serve.py sends - and GitHub Pages sends
# no headers of a site's choosing. coi-serviceworker (MIT, github.com/gzuidhof/coi-serviceworker)
# is a service worker that adds them to every response from the site, and reloads the page once
# it is in charge; where the headers are there already it does nothing.
set -euo pipefail
out=${1:?usage: pages-site.sh <out-dir>}
www=$(cd "$(dirname "$0")/../www" && pwd)
COI_VERSION=0.1.7
[ -f "$www/pkg-planner/planner_bg.wasm" ] || { echo "no $www/pkg-planner: build the planner first" >&2; exit 1; }
rm -rf "$out"
mkdir -p "$out"
cp -rL "$www/index.html" "$www/link.js" "$www/storage.js" "$www/files.js" "$www/serial.js" "$www/sitl-worker.js" "$www/tailscale.js" \
    "$www/tailscale" "$www/sitl" "$www/pkg-planner" "$out/"
# Only what a page loads: the SITL's README and Node bridge stay behind.
rm -f "$out/sitl/README.md" "$out/sitl/bridge.mjs"
# The service worker from npm's registry, at a pinned version.
tmp=$(mktemp -d)
(cd "$tmp" && npm pack --silent "coi-serviceworker@$COI_VERSION" > /dev/null && tar xzf "coi-serviceworker-$COI_VERSION.tgz")
cp "$tmp/package/coi-serviceworker.min.js" "$out/"
if [ -f "$tmp/package/LICENSE" ]; then cp "$tmp/package/LICENSE" "$out/coi-serviceworker.LICENSE"; fi
rm -rf "$tmp"
# First in the head, before the planner's module: it must be in charge before anything loads.
sed -i 's|<head>|<head>\n<script src="coi-serviceworker.min.js"></script>|' "$out/index.html"
grep -q 'coi-serviceworker.min.js' "$out/index.html"
# The planner's size as built, for the loading screen's percentage: GitHub Pages sends it gzipped,
# with only the compressed length, and the page counts it as it unpacks.
size=$(wc -c < "$out/pkg-planner/planner_bg.wasm")
sed -i "s|^const WASM_BYTES = 0;|const WASM_BYTES = $size;|" "$out/index.html"
grep -q "^const WASM_BYTES = $size;" "$out/index.html"
# GitHub Pages runs Jekyll over a site unless told not to; nothing here is Jekyll's.
touch "$out/.nojekyll"
du -sh "$out"
