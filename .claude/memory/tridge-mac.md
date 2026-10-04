---
name: tridge-mac
description: The borrowed Apple Silicon Mac the macOS build runs on (ssh tridge-mac) - where the clone, the build/test/release scripts and their logs live, what its toolchain is, and the three things that bit on 2026-10-03 (no window over SSH, Homebrew's liblzma, TMPDIR)
metadata:
  type: reference
---

`ssh tridge-mac` (tridgell.net:7932, user buzz; ~/.ssh/config has it). Apple Silicon, 10 cores,
16 GB, macOS 26.3, Xcode 26.2, Homebrew with pkg-config and xz; rustup with the pinned 1.95.0.
tridge is logged in at its console - it is his desktop - so a `planner` launched over SSH opens
no window; only `headless-planner` and the test suite run from here. Disk was 11 GB free on
2026-10-03 and 41 GB after he cleared some; the scripts carry a disk watchdog.

- Clone: `~/MissionPlannerRust`, pulled from GitHub (`git pull`), never pushed from there.
- Scripts in `~`: `mpr-build.sh` (debug planner, 6m42s), `mpr-test.sh` (`cargo test --workspace
  --no-fail-fast`, 177 binaries), `mpr-release.sh` (release, timed with `/usr/bin/time -l`:
  9m45s, 8.05 GB peak), `mpr-release-static.sh` (`LZMA_API_STATIC=1`), `mpr-verify.sh` (tests
  of named crates against `~/mac-fixes.patch`, applied with `git apply`), `mpr-release-universal.sh`
  (2026-10-04: both architectures with `--target`, joined by `lipo`, packaged in `~/mpr-universal/`
  as the release workflow does; the x86_64-apple-darwin target is installed for 1.95.0;
  `mpr-release-intel.sh`, Intel alone, was stopped for it at Buzz's word), each logging to
  `~/mpr-*.log`; run them with `nohup zsh ~/x.sh > ~/x.log 2>&1 < /dev/null & disown`.
- Always `CARGO_INCREMENTAL=0 CARGO_PROFILE_DEV_DEBUG=0` for debug work there (disk).
- The release planner linked `/opt/homebrew/opt/xz/lib/liblzma.5.dylib` until built with
  `LZMA_API_STATIC=1` (liblzma-sys, through RustPython's lzma module); DEV_MACOS.md says so.
- A test's `std::env::temp_dir()` is under `$(getconf DARWIN_USER_TEMP_DIR)`, not /tmp.
- macOS sleeps 16.7 ms as about 22 ms; pty slaves refuse a second open by path (ENOTTY);
  Apple's libm differs from glibc's in the last bits (`mp_units::golden_match`).

**How to apply:** verify a macOS-touching change there before committing: `git diff -- crates/ >
patch`, `scp`, `git checkout -- crates/ && git apply`, run `mpr-verify.sh`, and after the push
`git checkout -- crates/ && git pull`. Related: [[one-build-at-a-time]] (its 16 GB is not ours
to fill), [[scratch-target-dirs-fill-the-disk]].
