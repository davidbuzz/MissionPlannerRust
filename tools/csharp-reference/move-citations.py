#!/usr/bin/env python3
# Moves the port's line citations into the C# tree from one Mission Planner commit to another, for
# a move of the reference clone (MP_SRC) forward:
#
#   tools/csharp-reference/move-citations.py OLD NEW [--write] [--report FILE]
#
# A citation is `<name>.cs:<lines>` in a Rust file under crates/ or xtask/, a GUI script under
# tests/gui/, or NOT_DONE_YET_MATRIX.md or DELIVERABLES.md - `LogIndex.cs:372-410`,
# `Log/LogIndex.cs:300-302`, `SITL.cs:198-217, 604-738` - its lines a list of numbers and ranges.
# For a file that changed between the two commits, each cited line is carried through the diff to
# where it is now; a line upstream changed or removed has nowhere to go, and is left as it was and
# reported, as is a range with changed lines inside it: the port there may need the change. A name
# more than one file in the tree has is taken as the one the citation's path names, else the one
# file of that name the ledger has not dropped, else the one the port has worked on (the shallowest
# of several: the planner's own Program.cs), else reported. A line whose citation moved and that
# names the old commit (`@ efb0801`) is made to name the new one, whose numbers it now holds.
# Without --write nothing is changed; the report says what would be.
import argparse
import bisect
import csv
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TREE = os.environ.get("MP_SRC") or os.path.join(ROOT, "references", "missionplanner")
# A file's name, the path before it if any, and the lines after the colon.
CITATION = re.compile(
    r"(?P<path>(?:[A-Za-z0-9_.-]+/)*)(?P<name>[A-Za-z0-9_.]+\.cs):"
    r"(?P<lines>\d+(?:-\d+)?(?:, ?\d+(?:-\d+)?)*)"
)
HUNK = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")


def git(*args):
    return subprocess.run(
        ["git", "-C", TREE, *args], check=True, capture_output=True, text=True
    ).stdout


class Shift:
    """Old line numbers to new ones through one file's diff."""

    def __init__(self, diff):
        # Each hunk: (old start, old count, new start, new count).
        self.hunks = []
        for line in diff.splitlines():
            match = HUNK.match(line)
            if match:
                a, b, c, d = match.groups()
                self.hunks.append((int(a), int(b or 1), int(c), int(d or 1)))

    def line(self, old):
        """The line's number now, or None when upstream changed or removed it."""
        delta = 0
        for a, b, c, d in self.hunks:
            if b == 0:
                # An insertion after old line a.
                if a < old:
                    delta += d
                continue
            if old < a:
                break
            if old < a + b:
                return None
            delta += d - b
        return old + delta

    def changed_within(self, first, last):
        """Whether upstream changed any line from first to last, or inserted between them."""
        for a, b, _, _ in self.hunks:
            if b == 0:
                if first <= a < last:
                    return True
            elif a <= last and a + b - 1 >= first:
                return True
        return False


# The ledger's states of a file the port has worked on (xtask/src/ledger/mod.rs, STARTED).
WORKED = {"claimed", "ported", "tested", "verified", "reviewed", "done"}


def ledger_states():
    path = os.path.join(ROOT, "ledger", "ledger.csv")
    with open(path, newline="") as f:
        return {row["path"]: row["state"] for row in csv.DictReader(f)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("old")
    parser.add_argument("new")
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--report")
    args = parser.parse_args()

    old_short = git("rev-parse", "--short=7", args.old).strip()
    new_short = git("rev-parse", "--short=7", args.new).strip()
    changed = set(git("diff", "--name-only", args.old, args.new, "--", "*.cs").split())
    changed_names = {os.path.basename(c) for c in changed}
    by_name = {}
    for path in git("ls-tree", "-r", "--name-only", args.new).split():
        if path.endswith(".cs"):
            by_name.setdefault(os.path.basename(path), []).append(path)
    states = ledger_states()
    shifts = {}

    def shift(path):
        if path not in shifts:
            shifts[path] = Shift(git("diff", "-U0", args.old, args.new, "--", path))
        return shifts[path]

    def which(prefix, name):
        candidates = by_name.get(name, [])
        if prefix:
            named = [c for c in candidates if c.endswith(prefix + name)]
            if len(named) == 1:
                return named[0], None
        if len(candidates) == 1:
            return candidates[0], None
        live = [c for c in candidates if states.get(c) != "dropped"]
        if len(live) == 1:
            return live[0], None
        worked = sorted((c.count("/"), c) for c in live if states.get(c) in WORKED)
        if len(worked) == 1 or (len(worked) > 1 and worked[0][0] < worked[1][0]):
            return worked[0][1], None
        return None, f"{name}: {len(candidates)} files of that name"

    report, moved, kept = [], 0, 0
    sources = [
        os.path.join(root, file)
        for top, kind in (("crates", ".rs"), ("xtask", ".rs"), ("tests/gui", ".gui"))
        for root, _, files in os.walk(os.path.join(ROOT, top))
        for file in sorted(files)
        if file.endswith(kind)
    ]
    sources += [os.path.join(ROOT, doc) for doc in ("NOT_DONE_YET_MATRIX.md", "DELIVERABLES.md")]
    for source in sources:
        with open(source, encoding="utf-8") as f:
            text = f.read()
        if ".cs:" not in text:
            continue
        at = os.path.relpath(source, ROOT)
        starts = [m.start() for m in re.finditer("\n", text)]

        def replace(match):
            nonlocal moved, kept
            name = match.group("name")
            if name not in changed_names:
                return match.group(0)
            path, why = which(match.group("path"), name)
            lineno = bisect.bisect_left(starts, match.start()) + 1
            where = f"{at}:{lineno}: {match.group(0)}"
            if path is None:
                report.append(f"{where} - {why}")
                kept += 1
                return match.group(0)
            if path not in changed:
                return match.group(0)
            s = shift(path)
            parts = []
            for part in re.split(r"(, ?)", match.group("lines")):
                if not part or part[0] in ", ":
                    parts.append(part)
                    continue
                ends = [int(n) for n in part.split("-")]
                now = [s.line(n) for n in ends]
                if None in now:
                    report.append(f"{where} - {path}: line {part} changed upstream")
                    kept += 1
                    parts.append(part)
                    continue
                if len(ends) == 2 and s.changed_within(*ends):
                    report.append(f"{where} - {path}: lines {part} have changes inside")
                moved += now != ends
                parts.append("-".join(str(n) for n in now))
            lines = "".join(parts)
            return match.group(0)[: -len(match.group("lines"))] + lines

        new = CITATION.sub(replace, text)
        if new != text:
            # The lines whose citations moved name the new commit.
            new = "\n".join(
                after.replace(f"@ {old_short}", f"@ {new_short}") if after != before else after
                for before, after in zip(text.split("\n"), new.split("\n"))
            )
        if args.write and new != text:
            with open(source, "w", encoding="utf-8") as f:
                f.write(new)

    out = open(args.report, "w") if args.report else sys.stdout
    for line in report:
        print(line, file=out)
    print(
        f"{moved} cited lines moved, {kept} left as they were; {len(report)} to look at",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
