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
"""The crawl's faults clicked again, each alone, before they are reported.

    verify.py <label> [--display :97] [--out DIR] [--binary PATH] [-- planner arguments]

Every row of <out>/results-<label>.jsonl whose verdict is a fault (`nothing`, `internal-only`,
`status-only`) is replayed in a window of its own - the screen, then its recipe up to the click -
and the click judged again after 2.5 s, where the crawl waited 0.7. A busy machine or a slow
answer is not reported as a dead control. Rows go to <out>/verify-<label>.jsonl.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crawl  # noqa: E402
import driver  # noqa: E402
import observe  # noqa: E402

FAULTS = ("nothing", "internal-only", "status-only", "untypable")


def main():
    argv = sys.argv[1:]
    planner_args = []
    if "--" in argv:
        cut = argv.index("--")
        argv, planner_args = argv[:cut], argv[cut + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("label")
    parser.add_argument("--display", default=":97")
    parser.add_argument("--out", default=os.path.join(crawl.HERE, "out"))
    parser.add_argument("--binary", default=None)
    options = parser.parse_args(argv)
    crawler = crawl.Crawler("verify-" + options.label, options.display, planner_args,
                            options.out, options.binary)
    with open(os.path.join(options.out, f"results-{options.label}.jsonl")) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    todo = [r for r in rows if r.get("verdict") in FAULTS]
    print(f"[verify {options.label}] {len(todo)} to click again", flush=True)
    out = open(os.path.join(options.out, f"verify-{options.label}.jsonl"), "a")
    for row in todo:
        recipe = [tuple(step) for step in row["recipe"]]
        *way, last = recipe
        result = dict(state=row["state"], screen=row["screen"], page=row["page"], id=row["id"],
                      kind=row["kind"])
        app = None
        try:
            app = crawler.launch(row["screen"], way)
            baseline = observe.Baseline(app)
            rect = {"x": 0, "y": 0, "width": 0, "height": 0}
            if last[0] != "key":
                if not crawler.reveal(app, last[1]):
                    result["verify"] = "unreachable"
                    out.write(json.dumps(result) + "\n"); out.flush()
                    continue
                rect = app.probe()[last[1]]
            before = observe.Before(app, crawler.opened_count)
            crawler.perform(app, last)
            outcome = observe.judge(app, before, baseline, rect, crawler.opened_count, wait=2.5)
            if last[0] == "type":
                shown = (outcome["inside_px"] >= crawl.TYPED_PIXELS
                         or outcome["verdict"] == "visible")
                outcome["verdict"] = "typed" if shown else "untypable"
            result.update(verify=outcome["verdict"], evidence=outcome)
        except Exception as error:
            result.update(verify="error", error=str(error)[:300])
        finally:
            if app:
                app.close()
        out.write(json.dumps(result) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
