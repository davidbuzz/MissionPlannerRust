#!/usr/bin/env bash
# The browser build's Tailscale networking, end to end on one machine: a tailnet of its own -
# Headscale (coordination, with its embedded DERP on plain HTTP) behind check/cors_proxy.py, a
# userspace tailscaled named sitl-box, and ArduCopter's WebAssembly SITL on that machine's 5760 -
# then check/tailscale_check.js: the planner in a page joins the tailnet with an auth key and
# connects to sitl-box:5760 over it; then, without a key, check/tailscale_login_check.js: the page
# shows the coordination server's sign-in link. Everything it starts is stopped when it ends.
#
# Needs Go (a development dependency of the browser's Tailscale networking only: it builds Headscale
# and tailscaled here), Node with Playwright (NODE_PATH), and the page served on 127.0.0.1:8080 with
# the planner built into www/pkg-planner (README.md).
#
#   check/tailnet_e2e.sh [work-dir]
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
work=${1:-$(mktemp -d)}
mkdir -p "$work/bin" "$work/tsd" "$work/sitl"
cd "$work"
pids=()
cleanup() { for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT

# The same Tailscale as the page's node (tailscale/go.mod), and Headscale.
tsver=$(cd "$here/../tailscale" && go list -m -f '{{.Version}}' tailscale.com)
GOBIN=$work/bin go install "tailscale.com/cmd/tailscaled@$tsver" "tailscale.com/cmd/tailscale@$tsver"
GOBIN=$work/bin go install github.com/juanfont/headscale/cmd/headscale@v0.29.4

cat > config.yaml <<YAML
server_url: http://127.0.0.1:8090
listen_addr: 127.0.0.1:8090
metrics_listen_addr: 127.0.0.1:9092
grpc_listen_addr: 127.0.0.1:50444
grpc_allow_insecure: false
noise: { private_key_path: ./noise_private.key }
prefixes: { v4: 100.64.0.0/10, v6: "fd7a:115c:a1e0::/48", allocation: sequential }
derp:
  server:
    enabled: true
    region_id: 999
    region_code: headscale
    region_name: Headscale Embedded DERP
    verify_clients: true
    stun_listen_addr: 127.0.0.1:3479
    private_key_path: ./derp_server_private.key
    automatically_add_embedded_derp_region: true
    ipv4: 127.0.0.1
  urls: []
  paths: []
  auto_update_enabled: false
disable_check_updates: true
node: { expiry: 0, ephemeral: { inactivity_timeout: 30m } }
database: { type: sqlite, sqlite: { path: ./db.sqlite, write_ahead_log: true } }
log: { level: warn, format: text }
policy: { mode: file, path: "" }
dns: { magic_dns: true, base_domain: tailnet.test, override_local_dns: false, nameservers: { global: [], split: {} }, search_domains: [], extra_records: [] }
unix_socket: ./hs.sock
unix_socket_permission: "0770"
logtail: { enabled: false }
YAML

"$work/bin/headscale" serve -c config.yaml > headscale.log 2>&1 & pids+=($!)
for _ in $(seq 60); do [ -S hs.sock ] && break; sleep 0.5; done
"$work/bin/headscale" -c config.yaml users create mpr > /dev/null
key=$("$work/bin/headscale" -c config.yaml preauthkeys create --user 1 --reusable --expiration 1h | tail -1)

python3 "$here/cors_proxy.py" 8091 8090 > proxy.log 2>&1 & pids+=($!)
TS_DEBUG_USE_DERP_HTTP=1 "$work/bin/tailscaled" --tun=userspace-networking --statedir=./tsd --socket=./tsd.sock --port=0 > tailscaled.log 2>&1 & pids+=($!)
for _ in $(seq 60); do [ -S tsd.sock ] && break; sleep 0.5; done
"$work/bin/tailscale" --socket=./tsd.sock up --login-server http://127.0.0.1:8090 --authkey "$key" --hostname sitl-box
address=$("$work/bin/tailscale" --socket=./tsd.sock ip -4)

(cd sitl && exec node "$repo/tools/sitl/wasm/bridge.mjs" "$repo/tools/sitl/wasm/arducopter.js" 5760 \
    -Mquad -O-35.36,149.16,584,353 -s1 --serial0 wasm --serial1 none --serial2 none) > sitl.log 2>&1 & pids+=($!)
for _ in $(seq 60); do ss -ltn | grep -q "127.0.0.1:5760 " && break; sleep 0.5; done

echo "tailnet up: sitl-box at $address; the page joins through http://127.0.0.1:8091"
node "$here/tailscale_check.js" "$work" http://127.0.0.1:8091 "$key" "$address" 5760
# And without a key: the sign-in a pilot meets.
node "$here/tailscale_login_check.js" "$work" http://127.0.0.1:8091 "$address"
