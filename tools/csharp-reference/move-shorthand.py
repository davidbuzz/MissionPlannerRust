#!/usr/bin/env python3
# Carries the port's shorthand citations through a move of the reference clone - the `:N` after a
# file named earlier in the same comment (`ConfigPlanner.cs:92 ... (:1058-1062)`), which
# move-citations.py cannot read, so the move left them on the old commit's numbers:
#
#   tools/csharp-reference/move-shorthand.py OLD NEW MOVE [--write]
#
# MOVE is the commit that moved the reference (b177fbf for efb0801 to 5dbb2b0): a shorthand on a
# line last changed before it holds OLD's numbers and is carried through OLD..NEW's diff as
# move-citations.py carries a full citation; one changed since holds NEW's already and is left. Its
# file is the last `<name>.cs` named before it in the same comment - the same line or the comment
# lines just above it. A number upstream changed is carried through the hunk as
# review-worklist.py --reviewed carries it (difflib, type-only changes read as the same line), and
# each such is printed to be looked at. Without --write nothing is changed.
#
# The file is a guess where a comment names several: a shorthand after an inline citation of
# another file (`(:1136-1151, MAVLinkInterface.cs:3887-3940) ... (:434-445)`) is taken for that
# file's. SHOW_ALL=1 prints every move with the C# line before and after; check each whose comment
# names more than one file before `--write` (2026-10-06: six of 38 such were wrong, set by hand).
import argparse
import importlib.util
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
TREE = os.environ.get("MP_SRC") or os.path.join(ROOT, "references", "missionplanner")


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


movecite = load("movecite", "move-citations.py")
worklist = load("worklist", "review-worklist.py")

SHORT = re.compile(
    r"(?<=[(`\s]):(?P<lines>\d{1,5}(?:-\d{1,5})?(?:, ?\d{1,5}(?:-\d{1,5})?)*)(?=[`),;.\s]|$)"
)
COMMENT = re.compile(r"^\s*(//|#)")


def git(cwd, *args):
    return subprocess.run(
        ["git", "-C", cwd, *args], check=True, capture_output=True, text=True,
        encoding="utf-8", errors="replace",
    ).stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("old")
    parser.add_argument("new")
    parser.add_argument("move")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    changed = set(git(TREE, "diff", "--name-only", args.old, args.new, "--", "*.cs").split())
    by_name = {}
    for path in git(TREE, "ls-tree", "-r", "--name-only", args.new).split():
        if path.endswith(".cs"):
            by_name.setdefault(os.path.basename(path), []).append(path)
    shifts, aligned = {}, {}

    def shift(path):
        if path not in shifts:
            shifts[path] = movecite.Shift(git(TREE, "diff", "-U0", args.old, args.new, "--", path))
        return shifts[path]

    def align(path):
        if path not in aligned:
            old = worklist.lines_of(args.old, path)
            new = worklist.lines_of(args.new, path)
            aligned[path] = (worklist.Aligned(old, new), old, new)
        return aligned[path]

    states = movecite.ledger_states()

    def which(prefix, name):
        # move-citations.py's rule: the path named, the one file, the one the ledger has not
        # dropped, else the shallowest the port has worked on.
        candidates = by_name.get(name, [])
        named = [c for c in candidates if prefix and c.endswith(prefix + name)]
        if len(named) == 1:
            return named[0]
        if len(candidates) == 1:
            return candidates[0]
        live = [c for c in candidates if states.get(c) != "dropped"]
        if len(live) == 1:
            return live[0]
        worked = sorted((c.count("/"), c) for c in live if states.get(c) in movecite.WORKED)
        if len(worked) == 1 or (len(worked) > 1 and worked[0][0] < worked[1][0]):
            return worked[0][1]
        return None

    files = [
        f for f in git(ROOT, "ls-files", "crates", "xtask", "tests/gui").split()
        if f.endswith((".rs", ".gui"))
    ]
    moved, looked, unknown = 0, [], []
    for rel in files:
        with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
            lines = f.read().split("\n")
        if not any(SHORT.search(l) for l in lines if COMMENT.match(l)):
            continue
        blame = None
        out = list(lines)
        for i, line in enumerate(lines):
            if not COMMENT.match(line) or not SHORT.search(line):
                continue
            # The file: the last full citation before the shorthand, on this line or the comment
            # lines just above it.
            source = None
            for j in range(i, max(i - 30, -1), -1):
                if not COMMENT.match(lines[j]):
                    break
                text = lines[j]
                if j == i:
                    first = SHORT.search(text)
                    text = text[: first.start()] if first else text
                found = list(movecite.CITATION.finditer(text))
                names = list(re.finditer(r"(?:[A-Za-z0-9_.-]+/)*[A-Za-z0-9_.]+\.cs\b", text))
                if found or names:
                    last = found[-1] if found else None
                    m = names[-1]
                    if last and last.end() >= m.end():
                        source = (last.group("path"), last.group("name"))
                    else:
                        whole = m.group(0)
                        source = (whole[: -len(os.path.basename(whole))], os.path.basename(whole))
                    break
            if source is None:
                continue
            path = which(*source)
            if path is None:
                if source[1] in {os.path.basename(c) for c in changed}:
                    unknown.append(f"{rel}:{i + 1}: {source[1]} - more than one file of that name")
                continue
            if path not in changed:
                continue
            if blame is None:
                blame = git(ROOT, "blame", "--line-porcelain", "--", rel)
                commits = [l.split()[0] for l in blame.split("\n") if re.match(r"^[0-9a-f]{40} ", l)]
                before = {}
                for c in set(commits):
                    before[c] = subprocess.run(
                        ["git", "-C", ROOT, "merge-base", "--is-ancestor", c, args.move],
                        capture_output=True,
                    ).returncode == 0 and c != git(ROOT, "rev-parse", args.move).strip()
                blame = [before[c] for c in commits]
            if not blame[i]:
                continue
            s = shift(path)

            def carry(match):
                nonlocal moved
                parts = []
                for part in re.split(r"(, ?)", match.group("lines")):
                    if not part or part[0] in ", ":
                        parts.append(part)
                        continue
                    ends = [int(n) for n in part.split("-")]
                    now = [s.line(n) for n in ends]
                    if None in now:
                        a, old, new = align(path)
                        now = [a.carry(n) for n in ends]
                        for o, n in zip(ends, now):
                            before_text = old[o - 1].strip() if 0 < o <= len(old) else "?"
                            after_text = new[n - 1].strip() if n and 0 < n <= len(new) else "?"
                            if worklist.normal(before_text) != worklist.normal(after_text):
                                looked.append(f"{rel}:{i + 1}: {path}:{o} -> {n}\n"
                                              f"    old {before_text[:100]}\n    new {after_text[:100]}")
                    if None in now:
                        parts.append(part)
                        continue
                    moved += now != ends
                    if os.environ.get("SHOW_ALL") and now != ends:
                        a, old, new = align(path)
                        print(f"{rel}:{i + 1}: {path}:{part} -> {'-'.join(map(str, now))}"
                              f" | {old[ends[0] - 1].strip()[:60]} | {new[now[0] - 1].strip()[:60]}")
                    parts.append("-".join(str(n) for n in now))
                return ":" + "".join(parts)

            # Only the shorthands after the file's citation on this line.
            out[i] = SHORT.sub(carry, line)
        if args.write and out != lines:
            with open(os.path.join(ROOT, rel), "w", encoding="utf-8") as f:
                f.write("\n".join(out))
    for item in looked:
        print(item)
    for item in unknown:
        print(item)
    print(f"{moved} shorthand ranges moved; {len(looked)} ends to look at; {len(unknown)} unknown",
          file=sys.stderr)


if __name__ == "__main__":
    main()
