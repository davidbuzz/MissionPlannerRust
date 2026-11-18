# How to do a beta or a release

Written 2026-11-18. The design and its reasons are in `updates.md`; this file is the steps: who
does each, where it runs, and the command.

## Two rules first

**1. Nothing releases by itself.** The owner's word, 2026-11-18: "DO NOT DO ANY FORM OF AUTOMATED
RELEASE, EVER. this is DENIED. user must always action it." A release starts when the owner pushes
a tag: `v0_0_2` for a release, `beta` for a beta. There is no schedule, no nightly build, and
nothing an assistant or a script may start on his behalf. What runs *after* his action is
automated; the action is always his.

**2. No release exists yet with the updater in it, and that is deliberate.** The owner, the same
day: the absence of a `v0_0_2` tag "is NOT an oversight by the user, its an active and deliberate
choice by the user, as inadequate testing has been performed on the new 'updater' at this stage."
Nothing in this file is a prompt to tag. When to release is his decision, made when he judges
the testing adequate (see "Testing before a release").

This file describes a workflow that **has not yet run on GitHub in this form**. Its scripts are
tested; its steps are not. The first real run may show something to fix; correct this file then.

## Where things happen

| Place | What it is | What happens there |
|---|---|---|
| **The Linux box** | the owner's Ubuntu machine, the repository at `~/MissionPlannerRust` | every command typed by hand: commit, push, tag, the checks before and after |
| **GitHub, CI** | workflow `CI`, on GitHub's runners | starts on every push to `main`: the tests. Makes no release. |
| **GitHub, pages** | workflow `pages` | starts on every push to `main`: builds and publishes the browser planner's website. Makes no release of the desktop planner. |
| **GitHub, release builds** | workflow `release builds` (`.github/workflows/release.yml`), on GitHub's Linux, Windows and macOS runners | starts only on the owner's tag or his Run workflow: builds, packages and publishes |
| **The Windows VM** | `tiny10` on the Linux box, driven by `tools/win10/vm-run.sh` | optional testing of a Windows planner before a release. No part of a release is built or published there. |

## What is by hand and what is automated

| | By hand, by the owner, on the Linux box | Automated, on GitHub, after his action |
|---|---|---|
| A release | pushes `main`; waits for CI; **pushes the tag** | the three builds, their checks, the packaging, the release notes, the release, `update.json`, the check of what was published, the commit of `RELEASE_NOTES.md` |
| A beta | pushes `main`; **moves the tag `beta` and pushes it** | the same, published under the tag `beta` as a pre-release; no commit to the repository |
| Tests | the checks before and after, below | CI on each push |

## Prerequisites

On the Linux box:

- the repository with the remote `buzz` (GitHub) and push rights;
- `git`, and `gh` signed in (for the checks; the web pages do as well);
- for the optional tests: the Rust toolchain of `rust-toolchain.toml`, Python 3, and for the
  Windows one the `x86_64-pc-windows-gnu` target with MinGW and the VM running and logged in.

On GitHub: nothing to set up. The workflow uses the repository's own token.

Before either a beta or a release:

1. `main` is pushed.
2. CI is green on that commit.
3. For a beta: at least one release tag (`v...`) exists before the commit.
4. For a release: the owner has decided the updater is tested enough.

## The order

1. Commit and push `main`.
2. CI green.
3. Test (below), as much as the owner wants.
4. A release first: tag, wait, check.
5. Then at least one more commit on `main`, pushed, CI green.
6. Then a beta: run the workflow, wait, check.

Why 4 before 6, with 5 between: a release planner is offered a beta only when the beta is another
commit, built later. A beta built from the release's own commit exists and works, and a release
planner pressing "Check for BETA Updates" is told "No update available."

## Before either: push, and CI green

By hand, on the Linux box:

```
git push buzz main
gh run list --workflow CI --limit 1
```

Automated, on GitHub (workflow `CI`, started by the push): tests on Linux, Windows and macOS, the
Windows cross-compile, the browser build, the simulator tests, fuzzing, generated code, the wire
check, and licences and advisories. About an hour.

If **licences and advisories** is red, a new security advisory has been published against a
dependency. By hand, on the Linux box:

```
cargo update -p <crate>          # to the version the advisory names as fixed
cargo xtask licences             # rewrites THIRD_PARTY_LICENSES
```

then test, commit and push. Where there is no fixed version, the crate is replaced or the
exception is written in `deny.toml` with its reason; that is the owner's decision each time.

## Testing before a release

All optional, all by hand, all on the Linux box unless marked. None of it releases anything.

**The updater's own tests** (Linux box):

```
cargo test -p mp-update
cargo test -p mp-gui --bin planner -- help::tests
tools/gui-headless.sh help help-devbuild
```

**A real planner updating itself** (Linux box). A planner is built told it is a release, served a
release of itself from this machine, and made to update over its own running file:

```
mkdir -p target/claude/tmp
MP_RELEASE_VERSION=0.0.2 MP_RELEASE_CHANNEL=stable MP_RELEASE_COMMIT=$(git rev-parse HEAD) \
  MP_RELEASE_BUILT=$(date -u +%Y-%m-%dT%H:%M:%SZ) cargo build -p mp-gui --bin planner
strip -o target/claude/planner-as-release target/debug/planner
TMPDIR=$PWD/target/claude/tmp python3 tools/release/self-update-test.py target/claude/planner-as-release
cargo build -p mp-gui --bin planner     # the development build back in target/debug
```

**The same on Windows** (built on the Linux box, run in the Windows VM):

```
# Linux box: a Windows planner told it is a release, and the crate's tests, cross-compiled
MP_RELEASE_VERSION=0.0.2 MP_RELEASE_CHANNEL=stable MP_RELEASE_COMMIT=$(git rev-parse HEAD) \
  MP_RELEASE_BUILT=$(date -u +%Y-%m-%dT%H:%M:%SZ) CARGO_PROFILE_DEV_DEBUG=0 \
  cargo build --target x86_64-pc-windows-gnu -p mp-gui --bin planner
CARGO_PROFILE_DEV_DEBUG=0 cargo test -p mp-update --target x86_64-pc-windows-gnu --no-run
```

Then three files go into the folder the VM sees as `S:\updater` - the stripped `planner.exe`
(`x86_64-w64-mingw32-strip`), the test binary as `mp_update-tests.exe`, and
`tools/release/self-update-test.py`. That folder is outside the repository: the owner puts them
there, or says to. Then, from the Linux box, into the VM's console window:

```
tools/win10/vm-run.sh updater tools/win10/updater-test.ps1
```

which copies them to `C:\mp-updater-test` in the VM, runs the tests and the self-update there, and
removes the folder.

**What these do not test**, and only a real release or beta can: the workflow's own steps on
GitHub, the release builds (these are debug builds), the planner finding a release at GitHub's
address, the restart from the window, and a second release to update to.

## A release, step by step

### By hand, on the Linux box

1. The next number after the last:

   ```
   git tag --list 'v*'
   ```

2. On the pushed head of `main`, CI green on it:

   ```
   git fetch buzz && git status -sb
   gh run list --workflow CI --limit 1
   ```

3. The tag, and its push. **This is the action that releases.**

   ```
   git tag v0_0_2
   git push buzz v0_0_2
   ```

4. Watch, if wanted:

   ```
   gh run watch $(gh run list --workflow "release builds" --limit 1 --json databaseId --jq '.[0].databaseId')
   ```

### Automated, on GitHub, started by that push (about an hour)

| Job, and where | What it runs |
|---|---|
| **What this run builds** - GitHub, Linux | `tools/release/what.sh`: from the tag, release 0.0.2 of the stable channel |
| **Linux** - GitHub, Ubuntu 22.04 | `tools/package.sh deb` (the release build, stripped, and the `.deb`); `dist/planner --version` held to the version, channel and commit; `xvfb-run dist/planner` once, to open and paint; `tar -czf` the archive; `sha256sum` |
| **Windows** - GitHub, Windows | `cargo build --release -p mp-gui --bin planner`; `planner.exe --version` held the same way; `planner.exe` once, to open and paint; `Compress-Archive`; `Get-FileHash` |
| **macOS** - GitHub, macOS | `cargo build --release` for `aarch64-apple-darwin` and for `x86_64-apple-darwin`; `lipo -create` and `strip` into one universal planner; `--version` held the same way; `otool -L` (it links macOS alone); the planner once, to open and paint; `tar -czf`; `shasum -a 256` |
| **GitHub release** - GitHub, Linux, after all three | `tools/release/notes.py section` (the advisories fixed and the dependency changes since the last release tag); `tools/release/manifest.py write` (`update.json`); `gh release create` with the downloads, marked Latest; `gh release upload update.json`, last; `tools/release/manifest.py verify` on the published address; `tools/release/notes.py prepend`, then `git commit` and `git push origin main` of `RELEASE_NOTES.md` as `github-actions[bot]` |

### By hand afterwards, on the Linux box

```
gh release view v0_0_2 --json assets --jq '.assets[].name'     # seven files, update.json among them
git pull buzz main                                              # the RELEASE_NOTES.md commit
./target/debug/planner                                          # HELP > Check for Updates
```

Expect "Update Found: a development build to 0.0.2"; a release planner says "0.0.1 to 0.0.2".

## A beta, step by step

### By hand, on the Linux box

1. `main` pushed, CI green on it, and it is a later commit than the last release.
2. On that commit:

   ```
   git fetch buzz && git status -sb
   ```

3. The tag `beta`, moved to it, and its push. **This is the action that makes the beta.**

   ```
   git tag -f beta
   git push -f buzz beta
   ```

   `-f` because the tag is one rolling tag: it is taken from the last beta's commit and put on
   this one. Only the owner moves it.

### Automated, on GitHub, started by that push

The same five jobs with the same commands, with these differences: `what.sh` answers "a beta
after release 0.0.2"; the builds are told they are the beta channel's; the downloads' names carry
the commit; `gh release create beta --prerelease --verify-tag` attaches them to the tag as a
**pre-release**, or `gh release upload beta` puts them beside the last beta's, whose files are
removed once the new `update.json` is up; the notes are on the release's page only - **nothing is
committed to the repository**. The workflow neither makes nor moves the tag, and names no commit
to GitHub: it publishes to the tag that started it.

### By hand afterwards, on the Linux box

```
gh release view beta --json assets,isPrerelease --jq '.isPrerelease, (.assets[].name)'
./target/debug/planner                                          # HELP > Check for BETA Updates
```

## A build to look at, and no release

By hand: Actions > release builds > Run workflow. Automated: the
three builds, kept as the run's artifacts. Nothing is published, and each planner calls itself a
development build.

## When it goes wrong

| What happened | What to do, by hand |
|---|---|
| A build job failed for no reason of ours (a runner, a download) | Actions > the run > Re-run failed jobs. Only those rebuild. |
| A build job failed and the code needs a fix, and **nothing was published** | Fix on `main`, push, CI green. Then `git tag -d v0_0_2`, `git push buzz :refs/tags/v0_0_2`, and tag and push again. |
| The release **was published** and is wrong | Never move or reuse its tag: planners may already have it. Fix on `main` and release the next number. |
| The job *GitHub release* failed (the notes could not be computed, an upload failed, the check of the published release failed) | Re-run that job alone. The builds are kept, nothing rebuilds, and a release half made is completed. |
| `RELEASE_NOTES.md` could not be pushed to `main` | Re-run the job. Writing the same release twice changes nothing. |
| *GitHub release* fails with "HTTP 403: Resource not accessible by integration" | The workflow's own token was asked for something it may not do. It happened on 2026-11-18, when the workflow asked for a release at a named commit (`--target`), and again with the tag already pushed; since then it only attaches to the tag that started the run. If it returns, read what the failing `gh` command asked for. Re-running the job does not help: a re-run uses the workflow file of the run's own commit. |
| A planner says the check failed with a 404 | The newest release has no `update.json`: it was made before this workflow, or its release job did not finish. |

## Never

- Start a release or a beta any way but the owner doing it himself.
- Add a schedule, or any trigger that releases without him, to the workflow.
- Let anything but the owner move the tag `beta`: the workflow does not, and may not.
- Start a beta from Run workflow: that only makes a build to look at.
- Move or reuse a release tag once its release is published.
- Edit `RELEASE_NOTES.md` or `update.json` by hand: the workflow writes both.

## Where it is

| | |
|---|---|
| The workflow | `.github/workflows/release.yml` |
| What a run is, from how it was started | `tools/release/what.sh` |
| `update.json`, written and checked | `tools/release/manifest.py` |
| The release notes | `tools/release/notes.py` |
| A real planner updating itself, as a test | `tools/release/self-update-test.py` |
| The same in the Windows VM | `tools/win10/updater-test.ps1` |
| The design, and what is proven where | `updates.md` |
