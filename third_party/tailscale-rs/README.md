# tailscale-rs 0.6.1, vendored, with MagicDNS

Two crates of [tailscale-rs](https://github.com/tailscale/tailscale-rs), Tailscale's Rust library,
as published on crates.io at version 0.6.1 (from upstream commit `d34658bb`, their
`.cargo_vcs_info.json`):

- `tailscale/` - the `tailscale` crate, the library's API (`Device`);
- `ts_runtime/` - the `ts_runtime` crate, the actors under a `Device`.

Copyright (c) 2026 Tailscale Inc & contributors, under the BSD 3-Clause License (`LICENSE` in each
directory) with Tailscale's patent grant (`tailscale/PATENTS`). tailscale-rs is an unaudited
preview by its authors' own account (`tailscale/src/lib.rs`'s warning).

Only these two crates are here. Every other crate of tailscale-rs comes from crates.io at 0.6.1, as
`Cargo.lock` pins it; the root `Cargo.toml`'s `[patch.crates-io]` points the names `tailscale` and
`ts_runtime` at these directories, and its `[workspace] exclude` keeps `third_party` out of the
workspace, so the workspace's lints, clippy and tests are not run over this code.

## Why

MissionPlannerRust's built-in Tailscale node (`crates/mp-tailnet`, behind mp-gui's `tailnet-node`
feature and the tick "Use the planner's built-in node (experimental)" under the PLUGINS tab's Tailscale row) is the owner's feature of
2026-10-14, not Mission Planner's: "we WILL take the existing rust code, and add MagicDNS to it
enough so that 'white' or 'buzz55' as a hostname works instead of a [100].x ip address."

tailscale-rs 0.6.1 dials addresses only. Its `Device::peer_by_name` is not MagicDNS: it waits for a
first peer update that a tailnet with no other node never sends, matches a bare host name from any
tailnet a peer comes from, does not know this node itself, and compares names without folding
case. Tailscale's Go client resolves a name it dials (tsnet's `Dial`) without a DNS server,
from a table of the network map's node names held in memory; these changes give tailscale-rs that
table and that lookup, ported from the Go client (Tailscale v1.104.0) and cited line by line.

Upstream does not take outside code contributions (`tailscale/CONTRIBUTING.md`: "Currently, we are
not requesting external code or docs contributions"), so the change is carried here, written as
upstream's own code is written, so that it could go there.

## What is changed

Every change in the code is marked with a `// MissionPlannerRust:` comment naming the Go lines it
follows; search for that to find them all.

- `ts_runtime/src/peer_tracker/magic_dns.rs` - new: the rules, as plain functions of names and
  addresses - the lookup key (lowercase, one trailing dot removed), the tailnet's suffix, the table
  (every node's full name; the short name for a node in this node's own tailnet; this node itself
  included) and the address a dial uses (IPv4 first).
- `ts_runtime/src/peer_tracker/mod.rs` - the peer tracker keeps this node from the network map and
  rebuilds the table from it and the peers at each update, and answers a new message,
  `ResolveMagicDns`, once the network map has named this node (lookups made before then wait for
  it, at most 64 of them).
- `ts_runtime/src/lib.rs` - the runtime forwards `ResolveMagicDns` to the peer tracker.
- `tailscale/src/lib.rs` - `Device::resolve(host)` (an IP address is itself; a name is looked up,
  no longer than `MAGIC_DNS_TIMEOUT`), `Device::tcp_connect_host(host, port)` and
  `Device::udp_bind_for_host(host, port)`.
- `tailscale/src/error.rs` - `Error::Unresolved`, Go's `errUnresolved`.
- `ts_runtime/src/netmon.rs` - `State::routes`' `#[expect(dead_code)]` made `#[allow(dead_code)]`:
  this workspace's rustc (1.95) finds the expectation unfulfilled where upstream's (1.98) does
  not, a warning in every build of a path dependency and an error under CI's `-D warnings`.
- `ts_runtime/LICENSE` - added: the published `ts_runtime` crate carries no licence file, and this
  is tailscale-rs's own, the same terms and holder as `tailscale/LICENSE`.
- Left out of the published `tailscale` crate: `.cargo-ok` (Cargo's unpacking marker), and
  `AGENTS.md` and `CLAUDE.md`, upstream's instructions to coding agents working on upstream, which
  would be read as instructions by agents working in this repository.

Nothing else differs from the published crates.

## Tests

`magic_dns.rs` carries the rules' tests. They run in MissionPlannerRust's workspace: `cargo test -p
mp-tailnet` compiles that file into mp-tailnet's own tests. The vendored crates are outside the
workspace, and `ts_runtime`'s published manifest lists none of the dev-dependencies its other tests
need (`chrono`, `proptest`, ...), so `cargo test` does not run here.

## Licence

BSD 3-Clause, which this GPL-3.0-only work can carry. `cargo xtask licences` lists both crates in
`THIRD_PARTY_LICENSES`'s table (licence `BSD-3-Clause`, source `path`); xtask's licence-header check
skips `third_party/`, whose files keep their authors' own headers or none.

## Taking a newer release

1. Copy the new release's two crates from the Cargo registry (`~/.cargo/registry/src/*/` after a
   build that fetched them) into fresh `tailscale/` and `ts_runtime/` directories, leaving out
   `.cargo-ok`, `AGENTS.md` and `CLAUDE.md`, and copying `LICENSE` into `ts_runtime/` if it is
   still missing there.
2. Apply the changes again: every `// MissionPlannerRust:` site in the old copy, and
   `magic_dns.rs` as it is. Upstream's own changes come first - after 0.6.1, upstream's `main`
   already reworks `ControlRunner::auth_url` (`AuthResponse`, for a rejected auth key).
3. Raise the version in `crates/mp-tailnet/Cargo.toml` (`tailscale = "=..."`).
4. Build with the node and run its tests: `cargo build -p mp-gui --features tailnet-node`,
   `cargo test -p mp-tailnet --features node`; then `cargo xtask licences`.

Should upstream gain a name lookup of its own, the vendoring goes: the `[patch.crates-io]` entries
removed, these directories deleted, and mp-tailnet moved to upstream's call.
