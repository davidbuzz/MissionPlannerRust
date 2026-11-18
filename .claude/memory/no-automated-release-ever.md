---
name: no-automated-release-ever
description: No release or beta is ever started by automation or by me - no schedule, no tag pushed, no workflow run on his behalf; and never prompt him to tag: v0_0_2 is absent on purpose (Buzz, 2026-11-18)
metadata:
  type: feedback
---

**The rule, in Buzz's words (2026-11-18):** "DO NOT DO ANY FORM OF AUTOMATED RELEASE, EVER. this
is DENIED. user must always action it."

And of the missing tag: "the absense of a 'Tag `v0_0_2`' in the git repo is NOT an oversight by
teh user, its an active and deliberate choice by the user, as inadequate testing has been
performed on the new 'updater' at this stage."

**What happened:** asked that morning when beta builds should be made, he chose "Nightly, if main
moved", and I built a scheduled trigger into `.github/workflows/release.yml`. Later the same day
he overruled it outright. I had also ended several reports with "your next step: tag v0_0_2" and
offered to push and tag for him.

**Why:** a release puts software on other people's machines, and now updates what they already
run. That is his to decide and to do, every time, and the updater is not yet tested enough.

**How to apply:**

- I never start a release or a beta: no `git tag v...` pushed, no `gh workflow run "release
  builds"`, no `gh release create/upload/edit`, no moving the `beta` tag. Not when asked to "get
  it working", not to test. If a task seems to need a release, say so and stop.
- No trigger that releases without him goes into any workflow: no `schedule`, no release on push
  to main, no release from another workflow's completion. What runs after his tag or his Run
  workflow may be automated; the start is his.
- Do not prompt him to tag or release, and do not list it as a "next step". Report what is tested
  and what is not, and leave the decision with him.
- My interpretation: this covers the desktop planner's releases and betas. The `pages` workflow
  publishes the browser planner on every push to main - he pushes, so he actions it - and it
  predates this; raised with him 2026-11-18, not yet ruled.

Related: [[explicit-consent-outside-the-repo]]. The steps are in
`how_to_do_a_beta_or_release.md`.
