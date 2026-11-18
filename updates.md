# Updates: how the planner finds and installs a newer version of itself

Status, 2026-11-18: **built, all of it but signing, which the owner ruled out, and not yet
released: no release is to be made until he judges it tested enough, and none is ever made but
by his own action** - the check, the
install in place, the Debian package's download, the beta channel and the release workflow that
feeds them. Sections 1 to 9 are the plan as it was agreed and what was tested before it was
written; **section 10 is what was built, how to release with it, and what is proven where.** The
one part not yet run is the workflow on GitHub itself, which waits for the owner's push.

It began as the owner's bug of 2026-11-18: HELP's "Check for Updates" and "Check for BETA
Updates" worked in the C# and here said only "Sorry, Updates aren't implemented yet" (his own word
of 2026-10-14). He asked for a working version that need not copy the C#: "i dont think i want it
to work exactly like the old C# app ... write a technical implementation plan that works on at
least linux/macos/windows (we might still want the update/s turned off in wasm?)", and then "pls
do all phases of the 'update' together, excluding signing".

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

- **`crates/mp-update`**, as it was before this: a tested port of the above (`check.rs`,
  `apply.rs`, `md5.rs`, `version.rs`), with HELP's conversation around it in `help.rs` - the
  question, the progress dialog with Cancel, "No update available.", the daily check, `/update`
  and `/updatebeta`. All of it was off behind `UPDATES_ALLOWED = false`, and the channel settings
  were empty. The conversation is kept; the port behind it is replaced (section 10).
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
  "tag": "v0_0_2",
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
and the release job's open-and-paint check covers what the updater installs. Each asset also
carries its `url`, under the manifest's own release (`releases/download/<tag>/<name>`), left out
above for width. An address that is not https is refused, the manifest's and each download's.

### 4.3 The planner knows what it is

`release.yml` passes three values into each build, compiled in: the version (from the tag), the
commit and the channel. `planner --version` prints all three. A build without them - `cargo
build` on a developer's machine, `target/debug/planner` or `target/release/planner` - is a
**development build**. It was first to be refused any update; the owner's word the same day
changed that ("we have a process for that called ~/MissionPlannerRust$ ./target/debug/planner
... or the release binary at the same place"): its buttons update it as any planner's do. It
cannot say which release it is, so it is offered whatever the channel holds, as "a development
build to 0.0.2". Only the check at start is not for it: it would be asked every day, and every
test run would ask GitHub.

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
| A development build (`target/debug`, `target/release`) | Updates in place when a button asks, the release's files written beside the planner in that folder; the next `cargo build` puts the development build back. |
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
- **What was done.** Neither `self_update` nor `self-replace` is used. What this updater needs
  beyond a download and a swap is its own - the manifest of section 4.2, progress into HELP's
  dialog and its Cancel between reads, the trial run of the new planner, the Debian package's
  path, no restart with a link open - and the swap itself, once section 9 had shown that a rename
  is all it takes on all three systems, is forty lines with their roll-back (`install.rs`). One
  crate was added: `tar` 0.4 (MIT or Apache-2.0, with `filetime` under it), to read the Linux and
  macOS archives; `sha2`, `flate2`, `ureq` and `serde_json` were here already.

## 5. CI changes

- **`release.yml`**: the three build values (4.3); `publish` writes and uploads `update.json`,
  last, so no manifest is ever visible before its files; a closing step fetches the manifest by
  its public address and checks every asset's hash.
- **Beta**: the same workflow run for the `beta` channel, publishing to the `beta` pre-release,
  when the owner runs it by hand. It was first to run nightly as well; he ruled that out the same
  day (section 7). Each run is about an hour on three platforms.
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
4. **The beta channel**: the rolling pre-release the owner makes by hand, the beta button,
   `beta_updates`.
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
| When are beta builds made? | First: nightly when main has moved and its CI is green, and by hand. **Then, the same day, overruled by him: "DO NOT DO ANY FORM OF AUTOMATED RELEASE, EVER. this is DENIED. user must always action it."** By hand only; the schedule is removed. |
| When is the first updating release made? | When he decides. That there is no `v0_0_2` tag "is NOT an oversight by the user, its an active and deliberate choice by the user, as inadequate testing has been performed on the new 'updater' at this stage." |
| The `.deb`? | Download the new `.deb`, check it, show the `apt` command. |
| Signing? | Never. |
| The ported C# updater? | Removed once the new path passes; the ledger's two rows (`Utilities/Update.cs`, `Updater/Program.cs`) marked redesigned by his word. |
| Ctrl on the beta button? | Nothing: no master channel. |
| The check at start? | Once a day as the C#, with a tick box in CONFIG > Planner to turn it off - beyond the C#, his own. |
| "Yes" while a link is open? | Install, and restart later: the planner never ends itself while connected. |
| Crates? | Whichever fits best, `self_update` or `self-replace` alone, said with its reason before it is added. |

## 8. To be proven on the machines

- macOS: the move, the quarantine mark and the signature are proven from a shell (9.2), and a
  real planner updating itself on Apple Silicon (10.4); still to do, the updated planner started
  from Finder, the workflow's universal build, and an Intel Mac.
- Windows: the rename is proven with stand-ins (9.1) and with a real `planner.exe` updating
  itself in the VM (10.4); still to do, the workflow's MSVC build doing the same, and what
  Defender makes of a program that writes an executable in a folder it watches.
- Linux: the staged `--version` run on a machine below glibc 2.35 refusing cleanly.
- GitHub: the release workflow itself, at the owner's first push and tag (10.4).

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

## 10. As built

### 10.1 Where it is

| What | Where |
|---|---|
| What this build is: version, channel, commit, compiled in by the release workflow; `--version` | `crates/mp-update/src/identity.rs` |
| A version read from `v0_0_2`, `0_0_2` or `0.0.2` alike, ordered by its numbers | `crates/mp-update/src/release.rs` |
| The manifest read and held to its form; which release is offered to which build | `crates/mp-update/src/manifest.rs`, `check.rs` |
| A download to a file with its SHA-256 taken on the way, progress and Cancel | `crates/mp-update/src/download.rs` |
| The `.tar.gz` and `.zip` unpacked, nothing outside the staging folder, files and folders only | `crates/mp-update/src/unpack.rs` |
| The trial run, the swap with its roll-back, the next start's clean-up, the Debian package | `crates/mp-update/src/install.rs` |
| HELP's two buttons, the question, the progress dialog, the daily check, `/update`, the restart | `crates/mp-gui/src/help.rs` |
| "Check for updates at start", the owner's box | `crates/mp-gui/src/config/planner.rs` (`chk_updateatstart`) |
| What a workflow run builds; the manifest written and the published release checked | `tools/release/what.sh`, `tools/release/manifest.py`, `.github/workflows/release.yml` |
| The release notes, computed: advisories fixed and dependency changes since the last release | `tools/release/notes.py`, and `RELEASE_NOTES.md`, which the workflow commits after a tagged release |

The first port of the C#'s updater - the per-file MD5 update and `headless-planner update-apply` -
is removed, and the ledger's rows for `Utilities/Update.cs` and `Updater/Program.cs` are marked
redesigned by the owner's word.

### 10.2 What a user sees

- **A release** (the workflow's build): "Check for Updates" asks "Update Found: 0.0.1 to 0.0.2",
  with the release's page as its ChangeLog link; "Check for BETA Updates" asks about the beta's
  build by its commit and day. Yes shows the C#'s progress dialog - getting, unpacking, checking
  the new planner, installing - and then the new planner starts and the old one ends.
- **With a link open** it installs and does not restart: "Update installed: it takes effect when
  the planner is next started". Until then both buttons say "The update is installed: restart
  the planner to use it".
- **From the `.deb`**: Yes fetches the new package into Downloads, checks it, and a box says
  where it is and the command, `sudo apt install <path>`.
- **A development build** (`cargo build`, the owner's local working copy): both buttons ask the
  channel and offer what it holds, "Update Found: a development build to 0.0.2", and Yes installs
  it over `target/debug/planner`. Nothing is asked at start.
- **In the browser page**: neither button is there.
- **At start**: once a day, silently unless there is a release to offer; on later starts the same
  day, the beta too when CONFIG > Planner's Beta Updates is ticked; never when "Check for updates
  at start" is unticked.
- **Settings**: `update_check` (the day last checked), `beta_updates`, `update_check_at_start`;
  `UpdateManifest` and `BetaUpdateManifest` name another manifest than this repository's, for a
  fork or a test.

### 10.3 Making a release

The steps, the checks afterwards and what to do when a run fails are in
`how_to_do_a_beta_or_release.md`; what follows is the outline.

1. **A stable release**: push a tag, `v0_0_2`. The workflow builds the three planners told they
   are release 0.0.2 of the stable channel at that commit, checks each says so (`planner
   --version`), opens and paints each, packages them, publishes them under the tag as the latest
   release, uploads `update.json` last, and then fetches the manifest by its public address and
   every download it names, holding each to its size and hash. The release's notes are the
   workflow's own (the owner, 2026-11-18: "i want the process that writes any RELEASE_NOTES.md to
   be 100% automated"): the RustSec advisories against what the last release was built from and
   not against this one, asked of the OSV database for every crate whose version changed, and
   the dependency changes between the two `Cargo.lock` files. They head the release's page, are
   attached as `RELEASE_NOTES.md`, and are committed to `main` in that file.
2. **The beta**: the owner runs the workflow by hand with the channel `beta` (Actions > release
   builds > Run workflow). The same three builds are published under the one tag `beta`, a
   pre-release, their names carrying the commit, the manifest replaced after they are up and the
   previous beta's files removed after that. Nothing makes a beta by itself.
3. **A build to look at**: Run workflow with no channel, as before. It is kept as the run's
   artifacts and calls itself a development build.

The planners released before this, `v0_0_0` and `v0_0_1`, have no updater: their users fetch the
first updating release by hand, once.

### 10.4 What is proven, and where

| | How | Result |
|---|---|---|
| The updater's parts | `cargo test -p mp-update`: 23 unit tests - the manifest, the ordering, a wrong hash, a short and a long download, Cancel, an archive that would write outside itself, a link in an archive, the trial against a planner that is another release or does not start, the swap undone part way, the clean-up touching only what its list names | pass, Linux |
| The workflow's tools against the planner's reader | `crates/mp-update/tests/manifest_tool.rs`: `manifest.py write` read back by `Manifest::parse`; `manifest.py verify` on a release laid out on disk, and on one tampered with; `what.sh` for a tag and for a run by hand | pass, Linux |
| HELP's logic | `help.rs`'s tests: a development build, the question and its link, Yes installing in place, a failed trial installing nothing, the beta, the Debian package, Cancel, no restart with a link open, the daily check and its off box | pass, Linux |
| The screen | `tests/gui/help.gui` (a release asks, installs into a scratch folder, then says to restart), `help-devbuild.gui`, `config-planner.gui` (the new box), headless | pass, Linux |
| **A real planner updating itself** | `tools/release/self-update-test.py <planner built as a release>`: a stamped planner, served a release of itself from this machine, refuses a wrong download, then fetches, checks, unpacks, tries and installs over its own running file, starts as the release, and deletes the old files at its next start | see below |

The self-update test, by platform:

| Platform | The planner | Result, 2026-11-18 |
|---|---|---|
| Linux (this machine, x86-64) | `cargo build` with the release values set, stripped; 147 MB | **16 of 16** |
| macOS 26.3, Apple Silicon | built on the Mac the same way, from this tree; 134 MB. Also `cargo test -p mp-update` there: 26 of 26 | **16 of 16** |
| Windows 10 21H2 (the tiny10 VM) | cross-compiled here for `x86_64-pc-windows-gnu`, stripped; 135 MB. Also the crate's unit tests there: 22 of 22 (the trial's own test is Unix's) | **16 of 16** |

On each, the step section 9 is about is the real one: the running planner moved its own file
aside, moved the new one into its name, and the new one started. The test opens no window - it
drives `/update`, HELP's update without its boxes - so what it does not cover is the restart from
the window and the dialogs themselves, which `help.gui` and `help.rs`'s tests cover on Linux.
The planners here were debug builds stamped by hand; the workflow's are release builds stamped
by `what.sh`, and on macOS universal ones.

Not yet run: the release workflow on GitHub - the stamped builds, the manifest's upload, the
check of the published release, the beta. Its scripts are tested here; the workflow's
own steps are first exercised by the first tag or beta run after the owner pushes. Nor, still,
the updated planner started from Finder on a Mac, an Intel Mac, or a Windows folder Defender
watches (section 8).
