#!/usr/bin/env bash
# Builds (or checks) the whole planner, crates/mp-gui, for a web page:
#   tools/planner-wasm.sh check|build [cargo args...]
# - nightly, with std rebuilt with atomics and shared memory: gpui_web's threads (Zed's recipe);
# - emsdk's clang for ring's C under ureq's HTTPS (Ubuntu's clang has no wasm32 target here),
#   unless CC_wasm32_unknown_unknown and AR_wasm32_unknown_unknown name others (a CI runner's
#   clang has the target: .github/workflows/pages.yml);
# - getrandom 0.3's browser backend for RustPython's random numbers.
# The target directory is the caller's CARGO_TARGET_DIR, else target/web.
set -euo pipefail
mode=${1:-check}; shift || true
repo=$(cd "$(dirname "$0")/../.." && pwd)
emsdk=${EMSDK:-$HOME/emsdk}
export CC_wasm32_unknown_unknown=${CC_wasm32_unknown_unknown:-$emsdk/upstream/bin/clang}
export AR_wasm32_unknown_unknown=${AR_wasm32_unknown_unknown:-$emsdk/upstream/bin/llvm-ar}
flags="-C target-feature=+atomics,+bulk-memory,+mutable-globals --cfg getrandom_backend=\"wasm_js\""
if [ "$mode" = build ]; then
    flags="$flags -C link-arg=--shared-memory -C link-arg=--max-memory=4294967296 -C link-arg=--import-memory -C link-arg=--export=__wasm_init_tls -C link-arg=--export=__tls_size -C link-arg=--export=__tls_align -C link-arg=--export=__tls_base"
fi
export CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUSTFLAGS="$flags"
export CARGO_TARGET_DIR=${CARGO_TARGET_DIR:-$repo/target/web}
# The workspace's release profile unwinds (wasm32 cannot) and links with fat LTO in one unit (an
# hour for the whole planner); a web build aborts on a panic and links in parallel.
export CARGO_PROFILE_RELEASE_PANIC=abort
export CARGO_PROFILE_RELEASE_LTO=thin
export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=16
export CARGO_PROFILE_RELEASE_DEBUG=0
cd "$repo"
exec cargo +nightly "$mode" -Zbuild-std=std,panic_abort --target wasm32-unknown-unknown -p mp-gui "$@"
