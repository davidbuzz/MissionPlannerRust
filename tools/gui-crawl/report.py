#!/usr/bin/env python3
# Copyright (C) 2026 David "Buzz" Bussenschutt
#
# This file is part of MissionPlannerRust, a Rust implementation derived from
# Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
# https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
#
# MissionPlannerRust is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by the
# Free Software Foundation, version 3 of the License.
#
# MissionPlannerRust is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-3.0-only
"""The crawl's results as a report, a replayable recipe and a script per fault.

    report.py [--out DIR]

Reads <out>/results-*.jsonl (crawl.py) and <out>/verify-*.jsonl (verify.py), and writes:

* report.md - the faults first, grouped by screen and page, each with the steps that reach it;
  then every verdict counted;
* recipe.txt - every click of the crawl in order, as `.gui` lines, one block a control;
* dead/<state>-<screen>-<control>.gui - for each fault, a script that walks to the control and
  clicks it, for tools/gui-headless.sh, to be given its expectation when the fault is fixed.

A fault is a control whose click gave no visible answer and was confirmed by the verification
pass: `nothing` (no change at all), `internal-only` (the planner changed something no user can
see), `status-only` (a status message the screen does not draw), `untypable` (a text field that
showed nothing typed into it). A control reached by several ways - the same dialog opened from
two buttons - is one fault, listed once with the number of ways.
"""
import argparse
import collections
import glob
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
FAULTS = ("nothing", "internal-only", "status-only", "untypable")


def gui_line(step):
    kind, target = step[0], step[1]
    if kind == "click":
        return f"click {target}"
    if kind == "rclick":
        return f"click {target}:right"
    if kind == "key":
        return f"key {target}"
    if kind == "type":
        return f"click {target}\ntype 7"
    return f"# {kind} {target}"


def connected(state):
    return state.startswith("conn")


def script(row):
    lines = [
        f"# Found by tools/gui-crawl ({row['state']}): {row['screen']}/{row['page']}, "
        f"'{row['id']}' gave no visible answer ({row['final']}).",
        "# Give it the expectation its fix meets; until then it reproduces the fault.",
    ]
    if connected(row["state"]):
        lines.append("# Run as: tools/gui-headless.sh -- <this> -- tcp:127.0.0.1:5760")
    lines += [f"screen {row['screen']}", "window 1600x1200", "budget 60",
              "env XDG_DATA_HOME $WORK", "env MP_CONFIG_XML $WORK/config.xml", ""]
    if connected(row["state"]):
        lines += ["within 45", "expect params.held > 1000"]
    for step in row["recipe"]:
        lines.append(gui_line(step))
    return "\n".join(lines) + "\n"


def load(out):
    rows = []
    for path in sorted(glob.glob(os.path.join(out, "results-*.jsonl"))):
        with open(path) as f:
            rows.extend(json.loads(line) for line in f if line.strip())
    verified = {}
    for path in sorted(glob.glob(os.path.join(out, "verify-*.jsonl"))):
        with open(path) as f:
            for line in f:
                if line.strip():
                    v = json.loads(line)
                    verified[(v["state"], v["screen"], v["page"], v["id"], v["kind"])] = v
    for row in rows:
        key = (row["state"], row["screen"], row["page"], row["id"], row["kind"])
        v = verified.get(key)
        row["final"] = v["verify"] if v else row["verdict"]
        row["verified"] = bool(v)
    return rows


SOURCE = os.path.join(os.path.dirname(os.path.dirname(HERE)), "crates", "mp-gui", "src")
CLICK = re.compile(r"on_click|on_mouse_down|on_mouse_up|\baction\(|action_sized\(|listener\(|check_box\(|"
                   r"checkbox\(|button\(|text_field\(|combo\(")
_SOURCES = None


def sources():
    global _SOURCES
    if _SOURCES is None:
        _SOURCES = {}
        for directory, _, files in os.walk(SOURCE):
            for name in files:
                if name.endswith(".rs"):
                    path = os.path.join(directory, name)
                    with open(path, errors="replace") as f:
                        _SOURCES[path] = f.read().split("\n")
    return _SOURCES


def clickable(name):
    """Whether the planner's source gives this control a click: its id - the literal, or the part
    of it before a `format!`'s `{}` - named within a few lines of a click handler. A text or a
    panel that is measured but takes no click is no fault."""
    parts = re.split(r"([-_:@])", name)
    literals = [name] + ["".join(parts[:cut]) for cut in range(len(parts) - 1, 0, -1)]
    for literal in literals:
        if len(literal) < 4:
            break
        exact = literal == name
        pattern = re.compile('"' + re.escape(literal) + ('"' if exact else r'[-_:@]?\{'))
        hits = [(p, i) for p, lines in sources().items() for i, line in enumerate(lines)
                if pattern.search(line)]
        if hits:
            return any(CLICK.search("\n".join(sources()[p][max(0, i - 3): i + 25]))
                       for p, i in hits)
    return True  # not found by name: kept, to be looked at


def safe(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)[:80]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=os.path.join(HERE, "out"))
    options = parser.parse_args()
    rows = load(options.out)
    # A text field answers a click with no more than its caret: when typing into it showed what
    # was typed, its click is no fault (2026-10-15).
    typed = {(r["state"], r["screen"], r["page"], r["id"]) for r in rows
             if r["kind"] == "type" and r["final"] == "typed"}
    for r in rows:
        if r["final"] in FAULTS and r["kind"] == "click" and not clickable(r["id"]):
            r["final"] = "not-a-control"
        elif (r["final"] in FAULTS and r["kind"] == "click"
              and (r["state"], r["screen"], r["page"], r["id"]) in typed):
            r["final"] = "typed"
    faults = [r for r in rows if r["final"] in FAULTS]
    # One fault a control: the same screen, page, control and kind, reached by several ways.
    unique = {}
    for r in faults:
        key = (r["state"], r["screen"], r["page"], r["id"], r["kind"])
        if key in unique:
            unique[key]["ways"] += 1
        else:
            unique[key] = dict(r, ways=1)
    faults = list(unique.values())
    dead = os.path.join(options.out, "dead")
    os.makedirs(dead, exist_ok=True)
    for r in faults:
        name = f"{r['state']}-{r['screen']}-{safe(r['id'])}.gui"
        with open(os.path.join(dead, name), "w") as f:
            f.write(script(r))
    with open(os.path.join(options.out, "recipe.txt"), "w") as f:
        for r in rows:
            f.write(f"# {r['state']} {r['screen']}/{r['page']} {r['id']}: {r['final']}\n")
            f.write(f"screen {r['screen']}\n")
            for step in r["recipe"]:
                f.write(gui_line(step) + "\n")
            f.write("\n")
    counts = collections.Counter(r["final"] for r in rows)
    by_page = collections.defaultdict(list)
    for r in faults:
        by_page[(r["screen"], r["page"])].append(r)
    out = ["# Click everything: the crawl's report", "",
           f"{len(rows)} clicks; {len(faults)} gave no visible answer.", "",
           "| verdict | count |", "|---|---|"]
    out += [f"| {k} | {v} |" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])]
    repeats = sorted({(r["screen"], r["page"], r["id"]) for r in rows if r["final"] == "repeated-status"})
    if repeats:
        out += ["", "## Answered with the message already showing", "",
                "Each said again what the status line already said: nothing changes on screen.", ""]
        out += [f"- {screen} / {page}: `{name}`" for screen, page, name in repeats]
    out += ["", "## The faults", ""]
    for (screen, page), items in sorted(by_page.items()):
        out.append(f"### {screen} / {page}")
        out.append("")
        out.append("| control | state | verdict | what changed | the way there | ways |")
        out.append("|---|---|---|---|---|---|")
        for r in sorted(items, key=lambda r: (r["id"], r["state"])):
            what = r.get("status") or ", ".join(r.get("facts") or []) or "-"
            way = " → ".join(gui_line(s).replace("\n", "; ").split(" ", 1)[1] for s in r["recipe"])
            out.append(f"| `{r['id']}` | {r['state']} | {r['final']} | {what[:80]} | {way} "
                       f"| {r['ways']} |")
        out.append("")
    with open(os.path.join(options.out, "report.md"), "w") as f:
        f.write("\n".join(out) + "\n")
    print(f"{len(rows)} clicks, {len(faults)} faults; report.md, recipe.txt, dead/ in {options.out}")


if __name__ == "__main__":
    main()
