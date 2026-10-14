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
"""Click everything: every named control on every page, and whatever its click reveals.

    crawl.py <label> [--display :97] [--screens fly,plan] [--pages REGEX] [--out DIR]
             [--binary PATH] [-- planner arguments, e.g. tcp:127.0.0.1:5760]

From each screen, and each page of a screen (SETUP's groups opened), every named control the
planner's probe reports is clicked once and judged (observe.py). A click that makes new controls
appear - a drop-down, a dialog, a menu, a sub-form - has those explored too, up to MAX_DEPTH
levels; the maps are right-clicked for their menus; the HELP text's shortcuts are pressed; a row
scrolled out of sight is scrolled into it first. Every row records the recipe that reached its
control - the steps from the screen's start - so the crawl replays as one list of actions, and a
fault can be reproduced alone (report.py makes a .gui script of each).

A text field (the planner's `ui.textfields` fact names them) is typed into as well as clicked: a
"7" typed after the click must show in it, and is taken out again with BackSpace - a field that
shows nothing typed is `untypable`. A control whose box holds other controls - a form, a panel -
is a container, not a control, and is not clicked; its controls are.

Between clicks the state is restored by Escape, or by a relaunch and the recipe replayed when
Escape does not bring the control back. Nothing on the deny list is clicked.

Rows go to <out>/results-<label>.jsonl, one a click.
"""
import argparse
import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import driver  # noqa: E402
import observe  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_DEPTH = 3
# A family of like controls appearing together (a list's rows): this many of them are clicked.
FAMILY_SAMPLE = 3
FAMILY_MIN = 7
# The window's frame, the page containers and the selectors: navigation, not controls.
CHROME = re.compile(
    r"^(tab-.*|root|header|header-byline|body|.*-body|.*-backdrop|panel:.*|status-bar|"
    r"main-(port|baud|connect)|map-canvas)$")
# Never clicked: what ends the session or the simulator, or rewrites firmware or parameters
# wholesale; and what opens a port of its own - a serial device, a network host or a camera -
# which reaches outside the crawl's planner: the output windows' Connect (NMEA, CoT, RTK inject,
# Support Proxy), the MAVLink Mirror's Write and Go (its default sends to UDP 14550, where the
# owner's own planner listens), DroneCAN's connect, every SiK Radio button, the serial passthrough,
# the video. (The update buttons are clicked: they only say updates are not implemented.)
DENY = re.compile(
    r"(reboot|bootloader|BUT_FlashBL|flashdl|dfu|upload-firmware|install|erase|format|"
    r"exit|quit|plugin-manager-save|BUT_resetparams|reset-to-default|disconnect|"
    r"BUT_connect$|^dronecan-connect$|^mirror-cell-\d+-[45]$|^sikradio-|^rtk-but_|"
    r"mavserialport|geinjection|videostart)", re.I)
SCREENS = {
    "fly": r"^fly-tab-", "plan": None, "params": None, "logs": r"^logs-tab-",
    "setup": r"^setup-page-", "config": r"^config-page-", "simulation": None, "help": None,
    "experimental": None, "plugins": None,
}
# A box a page opens with: answered before the page is crawled.
BOX_OKS = ("raw-param-warning-ok",)
# Right-clicked, for their menus.
RIGHT_CLICK = {"fly": ["map", "hud"], "plan": ["map"]}
# The shortcuts the HELP text lists, on the screen each belongs to.
SHORTCUTS = {
    "fly": ["F2", "F3", "F4", "F5", "ctrl+f", "ctrl+t"] + [f"ctrl+{d}" for d in "1234567890"],
    "plan": ["ctrl+z", "ctrl+o", "ctrl+s"],
}


# Lists whose rows are one control each, named for the row: clicked as a sample.
ROW_FAMILIES = re.compile(r"^(param-(value|desc|fav|group)|param|fly-action|mavftp-node|log-row)-")


def family(name):
    """The family a control's id belongs to when it is a list's row: a known list's prefix
    (`param-group-ACRO`), or the id without a trailing number (`frame-picture-3`,
    `fft-mask-bit2`). Any other control is a family of its own."""
    known = ROW_FAMILIES.match(name)
    if known:
        return known.group(0)
    return re.sub(r"[-_:](bit)?\d+$", "", name)


def sample(names):
    """A big family of like controls cut to FAMILY_SAMPLE of them: the first, the middle, the
    last."""
    groups = {}
    for name in names:
        groups.setdefault(family(name), []).append(name)
    kept, dropped = [], []
    for members in groups.values():
        if len(members) >= FAMILY_MIN:
            picks = {members[0], members[len(members) // 2], members[-1]}
            kept.extend(m for m in members if m in picks)
            dropped.extend(m for m in members if m not in picks)
        else:
            kept.extend(members)
    return sorted(kept), sorted(dropped)


class Crawler:
    def __init__(self, label, display, args, out, binary):
        self.label = label
        self.display = display
        self.args = list(args)
        self.connected = any(a.startswith(("tcp:", "udp:", "serial:")) for a in self.args)
        self.out = out
        self.binary = binary
        os.makedirs(out, exist_ok=True)
        self.opened_log = os.path.join(out, f"opened-{label}.log")
        self.openers = os.path.join(out, f"openers-{label}")
        driver.fake_openers(self.openers, self.opened_log)
        self.rows = open(os.path.join(out, f"results-{label}.jsonl"), "a")
        driver.ensure_display(display)

    def opened_count(self):
        try:
            with open(self.opened_log) as f:
                return sum(1 for _ in f)
        except OSError:
            return 0

    def record(self, **row):
        row["state"] = self.label
        self.rows.write(json.dumps(row) + "\n")
        self.rows.flush()

    # --- getting to a state --------------------------------------------------------------------

    def launch(self, screen, nav):
        app = driver.App(self.display, screen, self.args, self.binary, self.openers)
        if self.connected:
            deadline = time.time() + 45
            while time.time() < deadline:
                try:
                    if int(app.facts().get("params.held", "0") or 0) > 1000:
                        break
                except ValueError:
                    pass
                time.sleep(0.5)
        self.replay(app, nav)
        return app

    def perform(self, app, step):
        kind, target = step[0], step[1]
        if kind == "click":
            return app.click(target)
        if kind == "type":
            # Clicked for the keyboard, then a character typed: TYPED.
            if not app.click(target):
                return False
            time.sleep(0.4)
            app.key(TYPED)
            return True
        if kind == "rclick":
            return app.click(target, button="3")
        if kind == "key":
            app.key(target)
            return True
        return False

    def replay(self, app, steps):
        for step in steps:
            if step[0] != "key" and step[1] not in app.probe():
                self.reveal(app, step[1])
            self.perform(app, step)
            time.sleep(0.9)
            for ok in BOX_OKS:
                if ok in app.probe():
                    app.click(ok)
                    time.sleep(0.8)
        app.park()
        time.sleep(0.5)

    def reveal(self, app, name, rounds=15):
        """A control laid out out of sight, scrolled into it with the wheel over its list."""
        for _ in range(rounds):
            rect = app.probe().get(name)
            if not rect:
                return False
            if driver.App.seen(rect):
                return True
            vx, vy = rect.get("visible_x", 0.0), rect.get("visible_y", 0.0)
            vw, vh = rect.get("visible_width", 0.0), rect.get("visible_height", 0.0)
            if vw < 4 or vh < 4:
                return False
            down = rect["y"] > vy + vh / 2
            app.wheel(vx + vw / 2, vy + vh / 2, down)
            time.sleep(0.3)
        return bool(driver.App.seen(app.probe().get(name) or {"x": 0, "y": 0, "width": 0, "height": 0}))

    # --- the pages -----------------------------------------------------------------------------

    def pages(self, screen, wanted):
        selector = SCREENS.get(screen)
        if not selector:
            return [("main", [])]
        app = self.launch(screen, [])
        found = []
        try:
            for name in sorted(n for n in app.probe() if re.match(selector, n)):
                before = set(app.probe())
                app.click(name)
                time.sleep(1.0)
                children = sorted(c for c in set(app.probe()) - before if re.match(selector, c))
                found.append((name, [("click", name)]))
                found.extend((c, [("click", name), ("click", c)]) for c in children)
                if children:
                    app.click(name)
                    time.sleep(0.6)
        finally:
            app.close()
        if wanted:
            found = [f for f in found if re.search(wanted, f[0])]
        return found or [("main", [])]

    def controls(self, names, screen, probe=None):
        selector = SCREENS.get(screen)
        names = [n for n in names if not CHROME.match(n) and not (selector and re.match(selector, n))]
        if probe:
            names = [n for n in names if not container(n, probe)]
        return names

    @staticmethod
    def text_fields(app):
        return set(filter(None, app.facts().get("ui.textfields", "").split(",")))

    # --- one page ------------------------------------------------------------------------------

    def crawl_page(self, screen, page, nav):
        app = self.launch(screen, nav)
        try:
            baseline = observe.Baseline(app)
            kept, dropped = sample(self.controls(sorted(app.probe()), screen, app.probe()))
            for name in dropped:
                self.record(screen=screen, page=page, id=name, kind="click", depth=0,
                            recipe=nav + [("click", name)], verdict="sampled-out")
            todo = [([], ("click", name)) for name in kept]
            # A control the page shows when nothing is open over it: the soft reset's landmark.
            anchor = kept[0] if kept else "header"
            if page == "main" or page == SCREENS_FIRST.get(screen):
                todo += [([], ("rclick", name)) for name in RIGHT_CLICK.get(screen, [])]
                todo += [([], ("key", key)) for key in SHORTCUTS.get(screen, [])]
            print(f"[{self.label}] {screen}/{page}: {len(todo)} to click", flush=True)
            current = []
            # The controls each opened state brought with it (its dialog, its list): the state is
            # left only when they are gone, not merely when the page's own controls are listed -
            # a form left open over the page keeps them listed under it.
            marks = {}
            i = 0
            while i < len(todo):
                prefix, step = todo[i]
                i += 1
                kind, target = step
                if kind == "click" and DENY.search(target):
                    self.record(screen=screen, page=page, id=target, kind=kind, depth=len(prefix),
                                recipe=nav + prefix + [step], verdict="denied")
                    continue
                # The state the step needs: its prefix applied, and its control on screen.
                if not app.alive():
                    app.close(); app = self.launch(screen, nav); baseline = observe.Baseline(app)
                    current = []
                if current is None or current != prefix:
                    reached = False
                    if current is not None and current[:len(prefix)] == prefix:
                        # Deeper than needed: Escape back out, and every deeper state's own
                        # controls gone.
                        for _ in range(len(current) - len(prefix)):
                            app.key("Escape")
                            time.sleep(0.4)
                        probe = app.probe()
                        deeper = set()
                        for depth in range(len(prefix) + 1, len(current) + 1):
                            deeper |= marks.get(tuple(current[:depth]), set())
                        reached = (kind == "key" or target in probe) and not (deeper & set(probe))
                    if not reached:
                        # A soft reset - Escape twice, every opened state's controls gone - and
                        # the prefix replayed; a relaunch when that does not bring the control.
                        app.key("Escape"); time.sleep(0.3); app.key("Escape"); time.sleep(0.4)
                        probe = set(app.probe())
                        opened = set().union(*marks.values()) if marks else set()
                        if (anchor in probe and not (opened & probe)
                                and app.facts().get("screen") == screen):
                            self.replay(app, prefix)
                            reached = kind == "key" or target in app.probe()
                    if not reached:
                        app.close(); app = self.launch(screen, nav)
                        self.replay(app, prefix)
                        baseline = observe.Baseline(app)
                    current = prefix
                rect = None
                if kind != "key":
                    if target not in app.probe() or not driver.App.seen(app.probe()[target]):
                        if not self.reveal(app, target):
                            self.record(screen=screen, page=page, id=target, kind=kind,
                                        depth=len(prefix), recipe=nav + prefix + [step],
                                        verdict="unreachable")
                            continue
                    rect = app.probe()[target]
                before = observe.Before(app, self.opened_count)
                if not self.perform(app, step):
                    self.record(screen=screen, page=page, id=target, kind=kind, depth=len(prefix),
                                recipe=nav + prefix + [step], verdict="unreachable")
                    continue
                outcome = observe.judge(app, before, baseline,
                                        rect or {"x": 0, "y": 0, "width": 0, "height": 0},
                                        self.opened_count)
                if kind != "key" and outcome["verdict"] == "nothing":
                    outcome["look"] = observe.look(before.shot, rect)
                self.record(screen=screen, page=page, id=target, kind=kind, depth=len(prefix),
                            recipe=nav + prefix + [step], **outcome)
                if (kind == "click" and target in self.text_fields(app) and outcome["alive"]
                        and not outcome["appeared"] and outcome["screen_after"] == screen):
                    # The field has the keyboard now, or should: a character typed must show.
                    typing = observe.Before(app, self.opened_count)
                    app.key(TYPED)
                    typed = observe.judge(app, typing, baseline, rect, self.opened_count)
                    shown = typed["inside_px"] >= TYPED_PIXELS or typed["verdict"] == "visible"
                    typed["verdict"] = "typed" if shown else "untypable"
                    self.record(screen=screen, page=page, id=target, kind="type",
                                depth=len(prefix), recipe=nav + prefix + [("type", target)],
                                **typed)
                    app.key("BackSpace")
                    time.sleep(0.3)
                appeared = self.controls(outcome["appeared"], screen, app.probe())
                if (appeared and len(prefix) + 1 < MAX_DEPTH and outcome["screen_after"] == screen
                        and kind != "key"):
                    kept_children, dropped_children = sample(appeared)
                    for name in dropped_children:
                        self.record(screen=screen, page=page, id=name, kind="click",
                                    depth=len(prefix) + 1,
                                    recipe=nav + prefix + [step, ("click", name)],
                                    verdict="sampled-out")
                    children = [(prefix + [step], ("click", name)) for name in kept_children]
                    todo[i:i] = children
                    current = prefix + [step]
                    marks[tuple(current)] = set(outcome["appeared"])
                elif outcome["vanished"] or outcome["screen_after"] != screen or not outcome["alive"]:
                    current = None
                else:
                    current = prefix
        finally:
            app.close()

    def crawl(self, screens, wanted_pages):
        for screen in screens:
            for page, nav in self.pages(screen, wanted_pages):
                try:
                    self.crawl_page(screen, page, nav)
                except Exception as error:  # a page that failed is a row, and the crawl goes on
                    self.record(screen=screen, page=page, id="*", kind="page", depth=0,
                                recipe=nav, verdict="error", error=str(error)[:400])
                    print(f"[{self.label}] {screen}/{page}: error {error}", flush=True)


# The page of a paged screen on which the screen-wide right-clicks and keys are tried.
SCREENS_FIRST = {"fly": "fly-tab-quick"}
# What a text field is given, and how many changed pixels inside it say it showed: one digit at
# the planner's small text size is more than this.
TYPED = "7"
TYPED_PIXELS = 8


def container(name, probe):
    """Whether a control's box holds another control's box whole - a form, a panel, a row of
    cells - and is more than twice its size: its controls are clicked, not it."""
    outer = probe.get(name)
    if not outer:
        return False
    area = outer["width"] * outer["height"]
    for other, inner in probe.items():
        inner_area = inner["width"] * inner["height"]
        if other == name or inner_area < 4 or inner_area * 2 > area:
            continue
        if (inner["x"] >= outer["x"] and inner["y"] >= outer["y"]
                and inner["x"] + inner["width"] <= outer["x"] + outer["width"]
                and inner["y"] + inner["height"] <= outer["y"] + outer["height"]):
            return True
    return False


def main():
    argv = sys.argv[1:]
    planner_args = []
    if "--" in argv:
        cut = argv.index("--")
        argv, planner_args = argv[:cut], argv[cut + 1:]
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("label")
    parser.add_argument("--display", default=":97")
    parser.add_argument("--screens", default=",".join(SCREENS))
    parser.add_argument("--pages", default=None, help="a regular expression over page names")
    parser.add_argument("--out", default=os.path.join(HERE, "out"))
    parser.add_argument("--binary", default=None)
    options = parser.parse_args(argv)
    crawler = Crawler(options.label, options.display, planner_args, options.out, options.binary)
    crawler.crawl(options.screens.split(","), options.pages)


if __name__ == "__main__":
    main()
