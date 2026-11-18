# How to do a beta or a release

Written 2026-11-18. **The release workflow has not yet run on GitHub in this form**: its scripts
are tested here, its steps are not. The first release and the first beta may show something to
fix; when they have run, correct this file to what happened.

The design and its reasons are in `updates.md`. This file is only the steps.

## What the two are

| | A release | The beta |
|---|---|---|
| What it is | a numbered version, kept for good | one rolling build of `main`, replaced by the next |
| Its tag | `v0_0_2`: `v`, then numbers joined by underscores. You push it. | `beta`. The workflow makes and moves it. **Never make it by hand.** |
| Who gets it | HELP > Check for Updates, and the once-a-day check | HELP > Check for BETA Updates |
| How it starts | you push the tag | by itself each night, or by hand |

Both are built by one workflow, **release builds** (`.github/workflows/release.yml`). Nothing is
built or uploaded by hand.

Pushing to `main` starts two other workflows, **CI** (the tests) and **pages** (the website).
Neither makes a release or a beta.

## Before either: main is pushed and CI is green

1. Push `main`.
2. Wait for the **CI** run on that commit to finish green: the Actions tab, or

   ```
   gh run list --workflow CI --limit 1
   ```

3. If it is red, fix that first. The job **licences and advisories** goes red by itself when new
   security advisories are published against a dependency; the fix is to update that crate
   (`cargo update -p <crate>`), or, where there is no fixed version, to record the exception in
   `deny.toml` with its reason.

## A release, step by step

1. **Choose the number**: the next after the last one.

   ```
   git tag --list 'v*'
   ```

2. **Be on the commit to release**, the pushed head of `main`, with CI green on it.

   ```
   git fetch buzz && git status -sb
   ```

3. **Tag it and push the tag.** This is the whole of your part.

   ```
   git tag v0_0_2
   git push buzz v0_0_2
   ```

4. **Wait about an hour.** Watch it in Actions > release builds, or

   ```
   gh run watch $(gh run list --workflow "release builds" --limit 1 --json databaseId --jq '.[0].databaseId')
   ```

   What it does without you:

   - decides what the run is (job *What this run builds*): release 0.0.2 of the stable channel;
   - builds the planner on Linux, Windows and macOS, each told its version, channel and commit;
   - checks each says so (`planner --version`) and opens and paints once;
   - packages the four downloads and their checksums;
   - writes the release notes: the security advisories fixed since the last release and the
     dependency changes, computed from the two `Cargo.lock` files;
   - publishes the release, marked Latest, and uploads `update.json` last;
   - fetches the published manifest and every download it names, and checks each size and hash;
   - commits `RELEASE_NOTES.md` to `main`.

5. **Check it.**

   - The release holds seven files: the Linux `.tar.gz`, the `.deb`, the Windows `.zip`, the macOS
     `.tar.gz`, `SHA256SUMS.txt`, `RELEASE_NOTES.md` and `update.json`.

     ```
     gh release view v0_0_2 --json assets --jq '.assets[].name'
     ```

   - The step *The published release is what the manifest says* is green.
   - `main` has one new commit by `github-actions[bot]`, "RELEASE_NOTES.md: v0_0_2 ...". Pull it.

     ```
     git pull buzz main
     ```

   - A planner finds it: `./target/debug/planner`, HELP, Check for Updates, and expect "Update
     Found: a development build to 0.0.2". A release planner says "0.0.1 to 0.0.2".

## The beta

**Each night, by itself.** At 16:17 UTC the workflow looks at `main` and builds a beta only when
both hold:

- the head of `main` is not already the beta;
- CI passed on the last commit that changed anything but `RELEASE_NOTES.md`.

**By hand, at once.** Actions > release builds > Run workflow, branch `main`, channel `beta`, or

```
gh workflow run "release builds" -f channel=beta
```

By hand it builds whatever `main` is, without those two checks.

Either way it builds the same three planners told they are the beta channel's, publishes them as
the pre-release under the tag `beta` in place of the last beta's, with their own `update.json`
and the notes of what changed since the last release. A beta is not written into
`RELEASE_NOTES.md` in the repository.

Check it:

```
gh release view beta --json assets,isPrerelease --jq '.isPrerelease, (.assets[].name)'
```

and from a planner, HELP > Check for BETA Updates.

Two rules:

- **A beta follows a release.** With no `v...` tag before its commit the run stops and says so.
- **Release first, then a later commit, then the beta.** A release planner is offered a beta only
  when the beta is another commit, built later. Built from the same commit as the release, the
  beta is there and a release planner is told "No update available."

## A build to look at, and no release

Actions > release builds > Run workflow with the channel left at `none`. The three planners are
kept as the run's artifacts, nothing is published, and each calls itself a development build.

## When it goes wrong

| What happened | What to do |
|---|---|
| A build job failed for no reason of ours (a runner, a download) | Actions > the run > Re-run failed jobs. Only those rebuild. |
| A build job failed and the code needs a fix, and **nothing was published** | Fix on `main`, push, wait for CI. Remove the tag and tag the fixed commit: `git tag -d v0_0_2`, `git push buzz :refs/tags/v0_0_2`, then tag and push as before. |
| The release **was published** and is wrong | Never move or reuse its tag: planners may already have it. Fix on `main` and release the next number. |
| The job *GitHub release* failed (the notes could not be computed, an upload failed, the check of the published release failed) | Re-run that job alone. The builds are kept, so nothing rebuilds, and a release half made is completed. |
| `RELEASE_NOTES.md` could not be pushed to `main` | Re-run the job. Writing the same release twice changes nothing. |
| No beta appeared overnight | Read the job *What this run builds*: it says which of the two checks stopped it. |
| A planner says the check failed with a 404 | The newest release has no `update.json`: it was made before this workflow, or the release job did not finish. |

## Never

- Make, move or delete the tag `beta` by hand.
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
| The design, and what is proven where | `updates.md` |
