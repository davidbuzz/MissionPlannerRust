#!/usr/bin/env python3
"""A release's notes, written by the release workflow and by nobody else (the owner, 2026-11-18:
"i want the process that writes any RELEASE_NOTES.md to be 100% automated").

    tools/release/notes.py section --tag v0_0_2 --previous v0_0_1 --commit SHA --date 2026-11-18
                                   --old-lock old.lock --new-lock Cargo.lock
                                   [--advisories FILE] [--save-advisories FILE] [--out FILE]
    tools/release/notes.py prepend --file RELEASE_NOTES.md --section FILE --tag v0_0_2

`section` compares two Cargo.lock files - the previous release's and this one's - and writes
this release's section:

  * Security advisories fixed: every RustSec advisory that names a version of a crate the
    previous release was built from and no version of that crate this release is built from.
    Nobody lists them: each crate whose version changed is asked of the OSV database
    (https://osv.dev, which carries RustSec's advisories for crates.io) at its old versions and at
    its new ones, and what the old had and the new has not is what was fixed.
  * Dependency changes: the crates updated, added and removed, with their versions.

`--advisories FILE` answers from a file in place of the network - `{"crate version": [{"id",
"summary"}]}`, which `--save-advisories` writes - for the test and for looking at what was asked.

`prepend` puts a section at the head of RELEASE_NOTES.md, newest first, in place of any section
the file already has for that tag, so a run done twice writes once.

The workflow puts the section at the top of the release's page, attaches the file to the release
and, for a tagged release, commits it to main (.github/workflows/release.yml).
crates/mp-update/tests/manifest_tool.rs holds this to a pair of lock files and their answers.
"""

import argparse
import json
import pathlib
import re
import sys
import tomllib
import urllib.request

OSV = "https://api.osv.dev/v1"
HEADER = """# Release notes

Written by the release workflow at each release (`tools/release/notes.py`), newest first, from
the two releases' `Cargo.lock` files and the RustSec advisories against them. Not edited by hand.
"""


def crates(lock):
    """A lock file's crates from crates.io: name -> its versions, sorted."""
    found = {}
    for package in tomllib.loads(pathlib.Path(lock).read_text()).get("package", []):
        if str(package.get("source", "")).startswith("registry+"):
            found.setdefault(package["name"], set()).add(package["version"])
    return {name: sorted(versions, key=order) for name, versions in found.items()}


def order(version):
    """A version's numbers, for sorting: 0.103.9 before 0.103.17."""
    return [int(part) if part.isdigit() else -1 for part in re.split(r"[.+-]", version)]


def post(url, body):
    request = urllib.request.Request(
        url, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "User-Agent": "missionplanner-rust-release"})
    with urllib.request.urlopen(request, timeout=60) as answer:
        return json.load(answer)


def get(url):
    request = urllib.request.Request(url, headers={"User-Agent": "missionplanner-rust-release"})
    with urllib.request.urlopen(request, timeout=60) as answer:
        return json.load(answer)


def ask_osv(wanted):
    """The RustSec advisories against each (crate, version): {"crate version": [{id, summary}]}."""
    wanted = sorted(wanted)
    answers, summaries = {}, {}
    for start in range(0, len(wanted), 500):
        batch = wanted[start:start + 500]
        results = post(f"{OSV}/querybatch", {"queries": [
            {"package": {"name": name, "ecosystem": "crates.io"}, "version": version}
            for name, version in batch]})["results"]
        if len(results) != len(batch):
            sys.exit(f"notes.py: OSV answered {len(results)} of {len(batch)} questions")
        for (name, version), result in zip(batch, results):
            if result.get("next_page_token"):
                sys.exit(f"notes.py: OSV has more advisories for {name} {version} than one answer holds")
            ids = sorted(v["id"] for v in result.get("vulns", []) if v["id"].startswith("RUSTSEC-"))
            for advisory in ids:
                if advisory not in summaries:
                    summaries[advisory] = get(f"{OSV}/vulns/{advisory}").get("summary", "").strip()
            answers[f"{name} {version}"] = [{"id": a, "summary": summaries[a]} for a in ids]
    return answers


def section(args):
    old, new = crates(args.old_lock), crates(args.new_lock)
    updated = sorted(name for name in old if name in new and old[name] != new[name])
    added = sorted(name for name in new if name not in old)
    removed = sorted(name for name in old if name not in new)

    # Only a crate whose versions changed can have had an advisory closed.
    wanted = {(name, version) for name in updated + removed for version in old[name]}
    wanted |= {(name, version) for name in updated for version in new[name]}
    if args.advisories:
        answers = json.loads(pathlib.Path(args.advisories).read_text())
    else:
        answers = ask_osv(wanted)
    if args.save_advisories:
        pathlib.Path(args.save_advisories).write_text(json.dumps(answers, indent=2, sort_keys=True) + "\n")

    def against(name, versions):
        return {a["id"]: a["summary"] for v in versions for a in answers.get(f"{name} {v}", [])}

    fixed = []
    for name in updated + removed:
        still = against(name, new.get(name, []))
        for advisory, summary in sorted(against(name, old[name]).items()):
            if advisory not in still:
                fixed.append((advisory, name, ", ".join(old[name]), ", ".join(new.get(name, [])) or "removed", summary))
    fixed.sort()

    lines = [f"## {args.tag} - {args.date}", ""]
    lines += [f"Commit `{args.commit[:12]}`, compared with {args.previous}.", ""]
    lines += ["### Security advisories fixed", ""]
    if fixed:
        lines += ["| Advisory | Crate | Was | Now | What |", "|---|---|---|---|---|"]
        lines += [f"| [{a}](https://rustsec.org/advisories/{a}) | {name} | {was} | {now} | {text(summary)} |"
                  for a, name, was, now, summary in fixed]
    else:
        lines += [f"None: this release closes no advisory against what {args.previous} was built from."]
    lines += ["", "### Dependency changes", ""]
    lines += [f"{len(updated)} updated, {len(added)} added, {len(removed)} removed, of {len(new)} crates from crates.io."]
    if updated or added or removed:
        lines += ["", "<details><summary>Every one</summary>", "", "| Crate | Was | Now |", "|---|---|---|"]
        lines += [f"| {name} | {', '.join(old[name])} | {', '.join(new[name])} |" for name in updated]
        lines += [f"| {name} | - | {', '.join(new[name])} |" for name in added]
        lines += [f"| {name} | {', '.join(old[name])} | removed |" for name in removed]
        lines += ["", "</details>"]
    written = "\n".join(lines) + "\n"
    if args.out:
        pathlib.Path(args.out).write_text(written)
    print(written, end="")


def text(summary):
    """An advisory's summary for a table's cell."""
    return " ".join(summary.replace("|", "/").split()) or "-"


def prepend(args):
    path = pathlib.Path(args.file)
    new = pathlib.Path(args.section).read_text().strip() + "\n"
    if not new.startswith(f"## {args.tag} "):
        sys.exit(f"notes.py: {args.section} is not the section of {args.tag}")
    body = path.read_text() if path.exists() else ""
    # The sections the file holds, each from its "## " line; what is before the first is the header.
    parts = re.split(r"(?m)^(?=## )", body)
    kept = [part.strip() + "\n" for part in parts[1:] if not part.startswith(f"## {args.tag} ")]
    path.write_text("\n".join([HEADER, new] + kept))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    commands = parser.add_subparsers(dest="command", required=True)
    s = commands.add_parser("section")
    for name in ["--tag", "--previous", "--commit", "--date", "--old-lock", "--new-lock"]:
        s.add_argument(name, required=True)
    s.add_argument("--advisories")
    s.add_argument("--save-advisories")
    s.add_argument("--out")
    s.set_defaults(run=section)
    p = commands.add_parser("prepend")
    for name in ["--file", "--section", "--tag"]:
        p.add_argument(name, required=True)
    p.set_defaults(run=prepend)
    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
