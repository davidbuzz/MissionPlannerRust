#!/usr/bin/env bash
# Builds the page's Tailscale node (main.go: Tailscale's own Go client for the browser) into
# www/tailscale/: tailscale.wasm.gz, which the page decompresses itself, and Go's wasm_exec.js.
#
# Go is a development dependency of this one artifact only: it is needed to change the browser's
# Tailscale networking (main.go, or the Tailscale version in go.mod), not to build or run the
# planner. The built files are kept in the tree, as tools/sitl/wasm keeps ArduPilot's.
#
#   tailscale/build.sh            # with `go` on PATH, at least the version go.mod names
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=$here/../www/tailscale
mkdir -p "$out"
cd "$here"
GOOS=js GOARCH=wasm go build -trimpath -ldflags "-s -w" -o "$out/tailscale.wasm" .
gzip -9 -n -f "$out/tailscale.wasm"
cp "$(go env GOROOT)/lib/wasm/wasm_exec.js" "$out/wasm_exec.js"
# What went in, and under which licences, beside it (www/tailscale/MODULES.txt).
GOOS=js GOARCH=wasm go list -deps -f '{{if .Module}}{{.Module.Path}} {{.Module.Version}}{{end}}' . \
    | sort -u | grep -v '^missionplannerrust' > "$out/modules.list"
echo "modules compiled in: $(wc -l < "$out/modules.list") (licences: www/tailscale/MODULES.txt)"
echo "built $(go list -m -f '{{.Path}} {{.Version}}' tailscale.com) into $out:"
ls -l "$out"
