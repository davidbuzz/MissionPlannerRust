#!/usr/bin/env python3
# Closes the entries of a reference move's worklist (ledger/upstream-review-<new>.txt, written by
# move-citations.py) whose upstream change is only a type - Mission Planner's 32-bit system ids
# (e6454ccdd) turned many a `byte sysid` into `uint sysid` and dropped the `(byte)` casts on it,
# which the port, its ids a u32 throughout, owes nothing:
#
#   tools/csharp-reference/review-worklist.py OLD NEW [--write]
#
# A hunk is type-only when its old and new lines are the same once `byte`, `int` and `uint` are
# read as one word, casts to them are dropped, parentheses around a lone name are dropped, and
# blank lines and spacing are ignored. An entry closes when every hunk that touches its cited
# range is type-only:
#
# * "line A-B changed upstream" - the citation was left on its old numbers. Each end is carried
#   through the diff, an end inside a hunk to the same place in it when the hunk replaces its
#   lines one for one; the citation is rewritten to the new numbers (--write) where the Rust file
#   holds it as often as the worklist lists it.
# * "lines A-B have changes inside" - the citation was moved; nothing to rewrite.
#
# Anything else - a hunk with a real change, an end that cannot be carried, a name more than one
# file has - stays on the worklist for a person. Without --write nothing is changed; the counts
# say what would be.
import argparse
import os
import re
import subprocess
import sys
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TREE = os.environ.get("MP_SRC") or os.path.join(ROOT, "references", "missionplanner")
HUNK = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")
# `<rust file>:<line>: <citation> - <C# path>: line(s) <part> <why>`
ENTRY = re.compile(
    r"^(?P<rust>[^:]+):(?P<line>\d+): (?P<citation>.*) - (?P<path>[^:]+\.cs): "
    r"lines? (?P<part>\d+(?:-\d+)?) (?P<why>changed upstream|have changes inside)$"
)


def git(*args):
    return subprocess.run(
        ["git", "-C", TREE, *args], check=True, capture_output=True, text=True,
        encoding="utf-8", errors="replace",
    ).stdout


def normal(line):
    """A line as a type-only change leaves it."""
    line = line.replace("\r", "")
    line = re.sub(r"\((?:byte|int|uint)\)\s*", "", line)
    line = re.sub(r"\b(?:byte|int|uint)\b", "T", line)
    # `(sysid)` and `(MAV.sysid)` once their cast has gone.
    line = re.sub(r"\(([A-Za-z_][A-Za-z0-9_.]*)\)", r"\1", line)
    return re.sub(r"\s+", "", line)


class Hunks:
    """One file's -U0 diff: each hunk's place on both sides and whether it is type-only."""

    def __init__(self, diff):
        self.hunks = []
        old, new = [], []
        for line in diff.split("\n"):
            match = HUNK.match(line)
            if match:
                self._close(old, new)
                a, b, c, d = match.groups()
                self.hunks.append([int(a), int(b or 1), int(c), int(d or 1), None])
                old, new = [], []
            elif self.hunks and line.startswith("-") and not line.startswith("---"):
                old.append(line[1:])
            elif self.hunks and line.startswith("+") and not line.startswith("+++"):
                new.append(line[1:])
        self._close(old, new)

    def _close(self, old, new):
        if self.hunks and self.hunks[-1][4] is None:
            kept = lambda lines: [n for n in map(normal, lines) if n]
            self.hunks[-1][4] = kept(old) == kept(new)

    def old_touching(self, first, last):
        """The hunks that change or insert within old lines first..last."""
        out = []
        for a, b, c, d, typed in self.hunks:
            if b == 0:
                if first <= a < last:
                    out.append((a, b, c, d, typed))
            elif a <= last and a + b - 1 >= first:
                out.append((a, b, c, d, typed))
        return out

    def new_touching(self, first, last):
        """The hunks that put new lines within new lines first..last, or remove lines there."""
        out = []
        for a, b, c, d, typed in self.hunks:
            if d == 0:
                if first <= c < last:
                    out.append((a, b, c, d, typed))
            elif c <= last and c + d - 1 >= first:
                out.append((a, b, c, d, typed))
        return out

    def carry(self, old):
        """The old line's number now: through a hunk that replaces its lines one for one too."""
        delta = 0
        for a, b, c, d, _ in self.hunks:
            if b == 0:
                if a < old:
                    delta += d
                continue
            if old < a:
                break
            if old < a + b:
                return c + (old - a) if b == d else None
            delta += d - b
        return old + delta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("old")
    parser.add_argument("new")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    new_short = git("rev-parse", "--short=7", args.new).strip()
    worklist = os.path.join(ROOT, "ledger", f"upstream-review-{new_short}.txt")
    with open(worklist, encoding="utf-8") as f:
        lines = f.read().split("\n")

    diffs = {}

    def hunks(path):
        if path not in diffs:
            diffs[path] = Hunks(git("diff", "-U0", args.old, args.new, "--", path))
        return diffs[path]

    closed, rewrites, kept = set(), [], Counter()
    for index, line in enumerate(lines):
        match = ENTRY.match(line)
        if not match:
            continue
        h = hunks(match["path"])
        ends = [int(n) for n in match["part"].split("-")]
        first, last = ends[0], ends[-1]
        if match["why"] == "have changes inside":
            touching = h.new_touching(first, last)
            if touching and all(t[4] for t in touching):
                closed.add(index)
            else:
                kept["a real change inside a moved range"] += 1
            continue
        touching = h.old_touching(first, last)
        now = [h.carry(n) for n in ends]
        if not touching or not all(t[4] for t in touching):
            kept["a real change at a range left on its old lines"] += 1
        elif None in now:
            kept["an end that cannot be carried"] += 1
        else:
            closed.add(index)
            rewrites.append((index, match["rust"], match["path"], match["part"],
                             "-".join(str(n) for n in now)))

    # A rewrite is made where the Rust file holds the old part, cited into that C# file, exactly as
    # often as the worklist lists it; otherwise the entry stays for a person.
    wanted = Counter((rust, path, part) for _, rust, path, part, _ in rewrites)
    texts = {}
    for _, rust, path, part, now in rewrites:
        if rust not in texts:
            with open(os.path.join(ROOT, rust), encoding="utf-8") as f:
                texts[rust] = f.read()
    plan = {}
    for index, rust, path, part, now in rewrites:
        name = re.escape(os.path.basename(path))
        cite = re.compile(
            rf"(?<![A-Za-z0-9_.]){name}:(?:\d+(?:-\d+)?, ?)*{re.escape(part)}(?![0-9-])"
        )
        found = len(cite.findall(texts[rust]))
        if found != wanted[(rust, path, part)]:
            closed.discard(index)
            kept["a citation not found as listed"] += 1
            continue
        plan[(rust, path, part)] = (cite, now)
    for (rust, path, part), (cite, now) in plan.items():
        texts[rust] = cite.sub(lambda m: m.group(0)[: -len(part)] + now, texts[rust])

    if args.write:
        for rust in {rust for rust, _, _ in plan}:
            with open(os.path.join(ROOT, rust), "w", encoding="utf-8") as f:
                f.write(texts[rust])
        with open(worklist, "w", encoding="utf-8") as f:
            f.write("\n".join(l for i, l in enumerate(lines) if i not in closed))
    left = sum(1 for l in lines if ENTRY.match(l) or (l and not l.startswith("#"))) - len(closed)
    print(f"{len(closed)} closed ({len(plan)} citations rewritten), {left} left", file=sys.stderr)
    for why, n in kept.most_common():
        print(f"  kept: {n} - {why}", file=sys.stderr)


if __name__ == "__main__":
    main()
