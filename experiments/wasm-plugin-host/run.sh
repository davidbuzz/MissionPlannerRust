#!/usr/bin/env bash
# Builds the plugin for WebAssembly, runs the host's tests against it, then the benchmark.
#
#   experiments/wasm-plugin-host/run.sh
#
# Needs the wasm32-unknown-unknown target: rustup target add wasm32-unknown-unknown
# Runs from the repository root: both crates are members of its workspace.
set -eu
cd "$(dirname "$0")/../.."
rustup target list --installed | grep -q wasm32-unknown-unknown || {
    echo "rustup target add wasm32-unknown-unknown" >&2
    exit 2
}
cargo build --release -p fencedist --target wasm32-unknown-unknown
ls -l target/wasm32-unknown-unknown/release/fencedist.wasm
cargo test -p wasm-plugin-host
cargo run --release -p wasm-plugin-host --bin bench
