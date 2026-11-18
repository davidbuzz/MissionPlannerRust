# Updates: how the planner finds and installs a newer version of itself

Status, 2026-11-18: **the plan is agreed and its questions answered (section 7); being built,
phase by phase (section 6).** Until phase 2 lands, HELP's "Check for Updates" and "Check for BETA
Updates" still answer "Sorry, Updates aren't implemented yet" (the owner's word of 2026-10-14). The owner's word of 2026-11-18 asks for a working version that need not
copy the C#: "i dont think i want it to work exactly like the old C# app ... write a technical
implementation plan that works on at least linux/macos/windows (we might still want the update/s
turned off in wasm?)". This file is that plan, and becomes the description of the process once
it is built. Section 7 holds what the owner decided.

## 1. What Mission Planner does

Read from `Utilities/Update.cs`, `Updater/Program.cs`, `GCSViews/Help.cs`, `MainV2.cs:3664-3677,
4039-4051`, `Program.cs:194-204` and `app.config:10-18`; the addresses were fetched on 2026-11-18.

| Channel | What it reads | Today |
|---|---|---|
| Stable | `https://firmware.oborne.me/MissionPlanner/upgrade/version.txt`, then `https://firmware.ardupilot.org/MissionPlanner/checksums.txt` and the files under `https://firmware.ardupilot.org/MissionPlanner/upgrade/` | version `1.3.9384.38258`; 1,317 checksum lines |
| Beta | the GitHub release tagged `betarelease` on ArduPilot/MissionPlanner: `version.txt`, `checksums.txt`, `MissionPlannerBeta.zip` (entries read by HTTP range) | version `1.3.9812.1271`; 1,373 checksum lines |
| Master | AppVeyor's artifacts for branch master (Ctrl held on the beta button) | 404, "Artifact not found or access denied" |

- **When it checks.** Once a day at start (the `update_check` setting holds the date), at every
  start when `beta_updates` is set, and from HELP's button. Stable compares the first line of
  `version.txt` beside the program with the server's and asks "Update Found [ChangeLog]", Yes or
  No. The beta button asks nothing and goes straight to the update.
- **How it updates.** A write test in the install directory; `checksums.txt` fetched; every local
  file hashed with MD5; each file that differs downloaded as `<file>.new`, three at a time; any
  `.dll` or `.exe` the list does not name deleted; `Updater.exe` started and the planner exits.
  The updater sleeps five seconds, moves each file to `.old` and each `.new` into place, and
  starts the planner again.
- **Where it does not.** A Windows Store install: both buttons hidden (`Help.cs:27-31`) and
  `DoUpdate` says "Not available when used as a windows store app".

Why not copy it as it is:

- It is built for about 1,300 loose .NET files, where a few change per release. This planner is
  one executable with its licences and the `sitl-wasm` folder, so per-file differencing saves
  nothing.
- MD5 over a list fetched from the same place as the files proves little, and nothing is signed.
- It needs a second program. The port names `headless-planner` as that program
  (`crates/mp-gui/src/help.rs`), which by the owner's word of 2026-10-04 is not shipped: turned
  on today, the ported update could download and never finish.
- Its addresses hold Mission Planner's .NET files, not ours.

## 2. What we have

- **`crates/mp-update`**: a tested port of the above (`check.rs`, `apply.rs`, `md5.rs`,
  `version.rs`), with HELP's conversation around it in `help.rs` - the question, the progress
  dialog with Cancel, "No update available.", the daily check, `/update` and `/updatebeta`. All of
  it is off behind `UPDATES_ALLOWED = false`, and the channel settings are empty.
- **CI** (`.github/workflows`):
  - `ci.yml`, every push to main and every pull request: tests on Linux, Windows and macOS, the
    cross-compile, the wasm check, SITL, fuzzing, codegen, wire agreement, licences and
    advisories. It keeps no binaries.
  - `pages.yml`, every push to main: builds the browser planner, runs the browser checks, deploys
    GitHub Pages.
  - `release.yml`, by hand or on a tag `v[0-9]*`: three native release builds (Linux on
    ubuntu-22.04 for glibc 2.35, Windows x86-64, macOS universal), each opened and painted once,
    then packaged. On a tag its `publish` job makes the GitHub release. A run takes about an hour.
- **Releases so far**: `v0_0_0` (2026-10-05) and `v0_0_1` (2026-10-06, marked Latest), each with
  a Linux `.tar.gz`, a `.deb`, a Windows `.zip`, a macOS `.tar.gz` and `SHA256SUMS.txt`.

What is missing for updating:

1. **The planner does not know which release it is.** The workspace version is `0.0.0`, so
   `planner --version` prints 0.0.0 from a `v0_0_1` download and the `.deb` is named 0.0.0.
2. **No fixed address.** Asset names carry the version.
3. **Nothing a program can read** that says what the newest version is and what each download's
   hash is, short of GitHub's API with its 60 requests an hour for each address.
4. **Nothing is signed.**

## 3. Where the binaries come from

| Source | Verdict |
|---|---|
| **GitHub Releases** on this repository | **Yes.** Public, no sign-in, kept for good, 2 GiB a file. `https://github.com/<owner>/<repo>/releases/latest/download/<name>` redirects to that file in the newest full release without the API, and `releases/download/<tag>/<name>` does the same for one tag. Mission Planner's own beta channel works this way. `release.yml` already publishes here. |
| Actions artifacts | No: downloading needs a signed-in token, and they expire. |
| GitHub Pages | No: the site is the browser planner; a site is limited to about 1 GB and our archives are 55-92 MB each. |
| firmware.ardupilot.org | No: Mission Planner's own server. |

## 4. The design

### 4.1 Channels

- **Stable** is the newest full GitHub release:
  `releases/latest/download/update.json`.
- **Beta** is one rolling pre-release under the fixed tag `beta`, replaced by CI:
  `releases/download/beta/update.json`. GitHub's "latest" never points at a pre-release, so the
  two cannot be confused.
- **Master** is dropped. The C#'s is dead, and beta already is main's newest build.

### 4.2 The manifest, `update.json`

One small file per release, written by the `publish` job from the files it has just uploaded:

```json
{
  "schema": 1,
  "channel": "stable",
  "version": "0.0.2",
  "commit": "42992d086a789143682f2997adc737545e3d8c2e",
  "built": "2026-11-20T03:10:00Z",
  "notes_url": "https://github.com/davidbuzz/MissionPlannerRust/releases/tag/v0_0_2",
  "assets": {
    "linux-x86_64":    { "name": "missionplanner-rust_0_0_2_linux-x86_64.tar.gz",   "size": 58623651, "sha256": "..." },
    "linux-x86_64-deb":{ "name": "missionplanner-rust_0.0.2_amd64.deb",              "size": 44743122, "sha256": "..." },
    "windows-x86_64":  { "name": "missionplanner-rust_0_0_2_windows-x86_64.zip",     "size": 54487793, "sha256": "..." },
    "macos-universal": { "name": "missionplanner-rust_0_0_2_macos-universal.tar.gz", "size": 91778607, "sha256": "..." }
  }
}
```

The planner downloads the same archives a person downloads by hand, so there is one set of files
and the release job's open-and-paint check covers what the updater installs. Each asset is
fetched from the manifest's own release (`releases/download/<tag>/<name>`).

### 4.3 The planner knows what it is

`release.yml` passes three values into each build, compiled in: the version (from the tag), the
commit and the channel. `planner --version` prints all three. A build without them - `cargo
build` on a developer's machine - is a **development build**: the buttons say so on the status
line and nothing is fetched.

Tags keep their form, `v0_0_2` (the owner, 2026-11-18): the planner reads the tag's numbers,
underscores or dots alike, as the version `0.0.2`, and orders versions number by number. The
`.deb` is named and versioned by the same numbers with dots, as Debian allows no underscore. A
beta build carries the version of the last release tag before its commit and is told apart by its
commit and build time, so a beta made after `v0_0_2` is offered `v0_0_3` when that is released
without any special case.

### 4.4 Checking

| Trigger | Stable | Beta |
|---|---|---|
| HELP's "Check for Updates" | offered when the manifest's version is above this build's; else "No update available." | - |
| HELP's "Check for BETA Updates" | - | offered when the beta's commit is not this build's and it was built later |
| Once a day at start (`update_check`), unless CONFIG > Planner's new box turns it off | as the button, silent when there is nothing | also, when `beta_updates` is set (CONFIG > Planner's box) |

The question is the C#'s, with the versions added: "Update Found: 0.0.1 to 0.0.2 [ChangeLog]",
Yes or No, the link opening `notes_url`. A check sends one HTTPS GET to github.com and nothing
about the user or the vehicle. A failure goes on the status line, never in a box.

### 4.5 Installing, by how the planner was installed

| Install | What "Yes" does |
|---|---|
| An unpacked archive in a folder the user can write (Linux `.tar.gz`, Windows `.zip`, macOS `.tar.gz`) | Updates in place, section 4.6. |
| The `.deb` (the program is under `/usr/bin`) | No in-place update: the package manager owns the file. The new `.deb` is downloaded to the user's Downloads folder, checked against the manifest, and the command to run is shown: `sudo apt install ./<file>.deb` (the owner, 2026-11-18). |
| A folder the user cannot write | The C#'s write test fails: "Unable to write to the install directory", and the release page's link. No elevation is asked for. |
| A development build | Says it is one. |
| **The browser page (wasm)** | **Off.** Both buttons hidden, as `Help.cs:27-31` hides them for the Store app, no check at start: the page is whatever the site serves. Left out at compile time. |

### 4.6 The update in place

No second program: the planner replaces itself.

1. **Download** the platform's archive to `<install>/.update/`, with the ported progress dialog
   and its Cancel.
2. **Verify** its size and SHA-256 against the manifest. A mismatch stops here with nothing
   changed.
3. **Unpack** into `<install>/.update/staged/`, refusing any entry that would land outside it.
4. **Try it**: run `staged/planner --version` and require the manifest's version and commit. This
   catches a build this machine cannot start (an older glibc, the wrong architecture) before
   anything is replaced.
5. **Swap**: each installed file is renamed to `<file>.old` and its staged file renamed into
   place; any failure renames everything back. On Windows a running `.exe` cannot be overwritten
   or deleted, which is why the C# has `Updater.exe`; it can be renamed, and that is all this
   needs. Tested on Windows and on a Mac, section 9.
6. **Restart**: the new planner is started and this one exits. **With a link open, it does not
   restart**: the status line says the update is installed and takes effect at the next start. A
   ground station does not end itself while it is connected to a vehicle.
7. **Next start**: the `.old` files and `.update/` are removed. Until then the previous planner is
   on disk as `planner.old`.

### 4.7 Trust

- HTTPS to GitHub and SHA-256 from the manifest. That is integrity against a bad download, and
  already more than the C#'s MD5; authenticity rests on the GitHub account.
- The manifest is not signed, and will not be (the owner, 2026-11-18).
- Never a downgrade without being asked, and nothing downloaded is run before its hash matches.

### 4.8 Crates that do this already

`self_update` 1.3.3 (MIT, updated 2026-10-27; the owner's question of 2026-11-18) is useful, in
part:

- **What fits.** A `manifest` backend for a static manifest file, `checksums` (SHA-256 before
  install), `signatures` (zipsign), `.tar.gz` and `.zip` reading, `MoveAll` (several files moved
  with a roll-back, which is 4.6's swap), a macOS `.app` bundle swap, a `restart` module, and
  `ureq` 3 as its HTTP client, which this tree already uses. It swaps the executable with the
  `self-replace` crate, whose Windows method is the rename tested in section 9.1.
- **What does not.** Its `update()` is a terminal's: it prints, asks yes or no on standard input
  and draws its own progress bar. Those turn off, but HELP's progress dialog and its Cancel need
  the download to report and to stop, which its documentation does not show - to be read in its
  source before it is chosen. Its GitHub backend calls the API (60 requests an hour for each
  address); the manifest backend avoids that. Its defaults bring in `reqwest`, so it would be
  taken without them. It wants `sha2` 0.11 beside this tree's 0.10, and adds `self-replace`,
  `tar`, `zip` and `urlencoding` to the licence table. It is left out of the browser build.
- **Recommendation.** Phase 3 starts by reading whether its pieces (`manifest`, `checksums`,
  `MoveAll`, `restart`) can be driven without its terminal flow. If they can, use them and write
  only the conversation. If the download cannot report or be cancelled, keep our own download
  (`mp_firmware`'s `Fetch`, `sha2` and `flate2` are already here) and take `self-replace` alone
  (Apache-2.0; `tempfile`, `fastrand` and `windows-sys`, all three already in the tree) for the
  swap.

## 5. CI changes

- **`release.yml`**: the three build values (4.3); `publish` writes and uploads `update.json`,
  last, so no manifest is ever visible before its files; a closing step fetches the manifest by
  its public address and checks every asset's hash.
- **Beta**: the same workflow run for the `beta` channel, publishing to the `beta` pre-release:
  nightly when main has moved since the last beta and its CI is green, and by hand (section 7).
  Each run is about an hour on three platforms.
- **Tests**: `mp-update` unit tests for the manifest, the ordering, a hash mismatch, an archive
  entry that escapes, a cancelled download, and the swap with its roll-back in a scratch folder;
  the swap of a running program on CI's Windows runner; `help.gui` against
  `tests/gui/update-server.py` serving a manifest and a small archive, through the
  `MP_UPDATE_DIR` door the harness already has.

## 6. Phases

1. **Identity and manifest.** The build values, the version read from the tag, `update.json` in
   the release job. Nothing changes for a user but `--version`.
2. **Check only.** Both buttons and the daily check read the manifest and answer; Yes opens the
   release page in the browser. Useful by itself, and it cannot break an install.
3. **Install in place** on Linux, Windows and macOS archives; the `.deb` downloaded with its
   command shown; an unwritable folder gets its message. The ported C# updater removed.
4. **The beta channel**: CI's nightly rolling pre-release, the beta button, `beta_updates`.
5. Each installer as it arrives (MSI, `.dmg`, AppImage) with its own way of being replaced.

The planners already released, `v0_0_0` and `v0_0_1`, have no updater: their users move to the
first updating release by hand, once.

## 7. Decided by the owner, 2026-11-18

| Question | His answer |
|---|---|
| May it be built now, under the demo freeze? | The demo is done and the freeze lifted. |
| When may main be pushed? | Not by me: everything stays local until he pushes. The release job's manifest step is unproven on GitHub until then. |
| What is the first updating release called? | `v0_0_2`, underscores kept; the planner reads the numbers. |
| How is the work committed? | Locally, an item a commit, nothing pushed. |
| When are beta builds made? | Nightly when main has moved and its CI is green, and by hand. |
| The `.deb`? | Download the new `.deb`, check it, show the `apt` command. |
| Signing? | Never. |
| The ported C# updater? | Removed once the new path passes; the ledger's two rows (`Utilities/Update.cs`, `Updater/Program.cs`) marked redesigned by his word. |
| Ctrl on the beta button? | Nothing: no master channel. |
| The check at start? | Once a day as the C#, with a tick box in CONFIG > Planner to turn it off - beyond the C#, his own. |
| "Yes" while a link is open? | Install, and restart later: the planner never ends itself while connected. |
| Crates? | Whichever fits best, `self_update` or `self-replace` alone, said with its reason before it is added. |

## 8. To be proven on the machines

- macOS: the move, the quarantine mark and the signature are proven from a shell (9.2); still to
  do, the updated planner started from Finder, and an Intel Mac.
- Windows: the rename itself is proven with stand-ins (9.1); still to do, the real `planner.exe`
  on the CI runner and in the VM, and what Defender makes of a program that writes an executable.
- Linux: the staged `--version` run on a machine below glibc 2.35 refusing cleanly.

## 9. What the tests taught: a running program's file on Windows and macOS

Both tests were run on 2026-11-18, before any of the updater was written, on the owner's questions
("can we test it?", "do we need to test macos ?"). The rules they leave for whoever writes the
swap, with the evidence under them:

1. **Replace by renaming, on every platform.** Move the installed file aside, move a fresh file
   into its name. Never open the installed file and write the new program into it.
2. **Windows and Linux refuse the wrong way loudly; macOS allows it and breaks the planner.**
   Windows will not overwrite or delete a running `.exe` at all, and Linux refuses the write
   ("Text file busy", checked here on kernel 7.0), so on those two a mistake fails at once. macOS
   lets a running program's file be written into and then kills that name at its next start - the
   update would report success and leave a planner that cannot start. Only a test on a Mac shows
   it.
3. **The old file cannot go in the same run on Windows.** It can be renamed while it runs but not
   deleted until it has ended, so it is removed at the next start. No helper process is needed.
4. **What the updater writes is not quarantined on macOS**, so an update needs no second approval.
5. **The universal planner's Intel half is unsigned as shipped.** `codesign -v` on the whole file
   says "not signed at all"; check the Apple Silicon half by itself (`--arch arm64`).
6. **Keep both scripts as the proof**: `tools/win10/self-replace-test.ps1` and
   `tools/macos/self-replace-test.sh` rerun in a minute each and exit with the number of lines
   that went other than expected.

### 9.1 Windows: tested 2026-11-18

The owner's question: his reading was that a running Windows executable cannot overwrite itself,
so a helper process must wait for it to exit and swap the file. Both halves are true and the test
shows which: `tools/win10/self-replace-test.ps1`, run in the tiny10 VM (Windows 10 Enterprise LTSC
2021, 21H2 build 19044, NTFS) with a copy of `cmd.exe` as the running program and a copy of
`hostname.exe` as the new one. Fourteen lines, every one as expected:

| Done to the file of a running program | Windows |
|---|---|
| Overwrite it with the new file | **refused**: "being used by another process" |
| Delete it | **refused**: "Access to the path ... is denied" |
| Rename it to `app.exe.old` | **allowed** |
| Copy the new file to the old name | allowed, and starting that name runs the new program |
| Delete `app.exe.old` while the old program still runs | refused |
| Delete `app.exe.old` once it has ended | allowed |

And with the running program doing it to itself - renaming the file it runs from, copying the new
one to that name and starting it as its last act - the new program ran, the old file sat beside
it as `app.exe.old`, and it could be deleted afterwards. So no helper is needed for the swap; what
cannot be done in the same run is the delete, which is why the old file goes at the next start
(4.6, step 7), as the C#'s `Program.CleanupFiles` clears its `.old` files.

Not yet covered by it: the real `planner.exe` (the test's stand-ins are small console programs),
a folder Defender is watching, and a file system other than NTFS.

### 9.2 macOS: tested 2026-11-18

The owner's question, "do we need to test macos ?": yes, and for another reason than Windows.
`tools/macos/self-replace-test.sh`, run over SSH on an Apple Silicon Mac (macOS 26.3, APFS), with
two small C programs signed as rustc's output is (ad hoc, linker-signed) and then the real
universal planner from a release archive. It opens no window and runs nothing quarantined.

| Done | macOS |
|---|---|
| A fresh file moved over the program's file, the program running or not | allowed; the name runs the new program, and a running old one carries on |
| The new program **written into** the file of a program that is running | allowed - and the name is then **killed at its next start** (signal 9); the old one carries on |
| The same once the old program has ended | runs |
| The running program moving itself aside, moving the new one in and starting it | the new program ran; the old sat beside it as `app.old` and was deleted after |
| A file this session downloaded; the planner unpacked from the archive | no quarantine mark on either |
| The real planner unpacked, moved over an installed copy, started | `planner --version` answers as before, natively and as Intel under Rosetta |

What it settles:

- **The swap must be a move of a fresh file, never a write into the installed one.** macOS lets
  the write happen and then kills the result, because the planner is running while it updates.
  4.6's step 5 is a rename, which is the safe one; a copy over the installed file, which looks
  the same on Linux, would leave a Mac with a planner that cannot start.
- **Nothing the updater writes is quarantined**, so the new planner needs no second approval in
  Privacy & Security. That holds for a plain program as shipped today; a sandboxed `.app` is
  marked whatever it writes, and is for the day there is one.
- **The universal binary survives the unpack**: the Apple Silicon half's signature is valid after
  it. The Intel half carries none, in the release as built and before any unpack - Intel asks for
  none and its linker adds none - so `codesign -v` on the whole file says "not signed at all".
  That is how `v0_0_1` shipped, not something the unpack did; it matters only when the planner is
  one day signed with a developer identity.

Not covered: Gatekeeper's assessment of the planner is "rejected", as for any program without a
developer identity, and it was started here from a shell, not from Finder; an Intel Mac.
