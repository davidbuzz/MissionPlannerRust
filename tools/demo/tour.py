#!/usr/bin/env python3
"""The click-everything tour of the user demo (Buzz, 2026-10-17): the crawl's recipe replayed to a
running planner with the real pointer, slowly enough to watch, for a given number of minutes.

    tools/demo/tour.py --display :0 --pid PID --probe FILE --facts FILE [--minutes 6]
                       [--dwell 0.8] [--recipe tools/gui-crawl/out/recipe.txt] [--screens fly,plan,...]

The planner was started by someone else (tools/demo/run.sh) with MP_PROBE and MP_FACTS, so every
named control's place is known; the pointer glides to each and clicks it, as tools/gui-crawl's
crawl does on its virtual displays. Only blocks the crawl judged `visible` or `changed-itself` are
played, screen by screen in turn, so a watcher sees something happen at every click. The crawl's
deny list keeps the tour off every port and device; the demo's own keeps it off whatever would
change the copter's flight, the connection or the plugins - the Welcome-Demo-Sitl has it flying
Auto by then - and off typing. A box a click opens is answered as the crawl answers it.
"""
import argparse
import os
import random
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools", "gui-crawl"))
import driver  # noqa: E402  (tools/gui-crawl/driver.py: the planner's probe, facts and xdotool)
from crawl import BOX_OKS, DENY  # noqa: E402  (the crawl's own guards)

# What the tour must not click while the copter flies and the watcher watches: the simulation and
# plugins tabs (the demo's own), the flight modes and actions, arming and flight commands, writes
# and reads of the mission and parameters, the connection controls, home, and anything that
# writes to the vehicle or a device beyond the crawl's deny list.
DEMO_DENY = re.compile(
    r"(^tab-(simulation|plugins)$|^sim-|^fly-action-|^fly-prompt-ok$|^plan-prompt-ok$|"
    r"^plan-(write|read|upload|download)|^param(s)?-(write|refresh|load|save|compare|reset)|"
    r"^main-(connect|port|baud|stats)|plugin-manager|set-?home|takeoff|land\b|\brtl\b|^(dis)?arm$|"
    r"\barm(ing|ed)?\b|reboot|calib|erase|format|flash|dfu|upload|install|guided|^ctrl\+t$|"
    r"^fly-(doaction|flyto|restartmission|resumemis|setwp|setloiterrad|mountmode|sendmsg)|"
    r"fence-(write|clear)|clearMission|clearPolygon|clearRallyPoints|saveRallyPoints|setRallyPoint|"
    r"prefetch|^fly-.*-(write|send|set)\b|"
    # Nothing that opens a browser or an editor (the owner's demo bugs of 2026-10-17: the Tailscale
    # guide and the ChangeLog came up in the browser), nor HELP's Beta Updates, a button he keeps
    # deliberately non-functional, nor Check for Updates: the links and wiki buttons by their names
    # here, and every click the crawl saw open something by `OPENERS` below.
    r"^help-(changelog|tailscale|betaupdate|updatecheck)$|-link$|-wiki$|-help$|^plan-kml$|"
    r"^georef-BUT_networklink|^fwl-motors$|^tailscale-|^scripts-.*edit)",
    re.I)
# The controls the crawl saw open a browser, an editor or a file (`opened` in its result rows,
# tools/gui-crawl/out/results-*.jsonl), read when the tour starts so the list never goes stale.
OPENERS = set()
# A block's last click inside a dialog another click of it opened: a file chooser's row or type,
# a prompt's button. Blocks that reopen the same dialog on a page are one sequence to a watcher.
DIALOG_STEP = re.compile(r"^(chooser-|.*-prompt-)")
# On the setup, config and other pages the copter's parameters are what most buttons write: only
# what the crawl's disconnected workers saw happen (their labels start with "disc") is played
# there - pages, drop-downs, dialogs - while on FLY and PLAN the connected crawl's clicks are, less
# the deny above. The recipe's block header names the worker.
CONNECTED_SCREENS = {"fly", "plan"}
# A mode button on the flight screen's Actions page is a bare mode name, no dash.
MODE_NAME = re.compile(r"^[A-Z][A-Za-z_ ]*$")
# The verdicts worth watching.
PLAYED = {"visible", "changed-itself"}
# Pages played after everything else, if time is left - the owner's word of 2026-10-17 on SETUP's
# Advanced page, whose buttons open the output windows and tools: "move all the setup tab ->
# advanced to the end of the demo, or remove them".
LAST_PAGES = ("setup-page-ConfigAdvanced",)
# The order screens are visited in, round and round; the welcome demo leaves the planner on FLY.
# EXPERIMENTAL is left out: its tools start outputs and windows a watcher is not here for.
SCREENS = ["fly", "plan", "setup", "config", "params", "logs", "help"]
# Boxes a click may leave: the crawl's, and the planner's prompts' cancels.
CLOSERS = BOX_OKS + ("fly-prompt-cancel", "plan-prompt-cancel", "param-box-ok", "servo-message-ok")


def dialogs_to_close(ground, probe):
    """What a block left open, by the planner's own naming: every control measured now and not at
    the tour's start whose name ends in -cancel (a prompt's, answered No) and then every -close (a
    window's). The owner's run of 2026-10-17 stuck on Serial Output NMEA: a port row asked for a
    host in a prompt the tour had no name for, and every click after landed under it."""
    opened = [name for name in probe if name not in ground]
    cancels = sorted(name for name in opened if name.endswith("-cancel"))
    closes = sorted(name for name in opened if name.endswith("-close"))
    return [name for name in cancels + closes if not DENY.search(name) and not DEMO_DENY.search(name)]


def read_recipe(path):
    """The recipe's blocks: (screen, page, control, verdict, [clicks])."""
    blocks, block = [], None
    for raw in open(path, errors="replace"):
        line = raw.rstrip("\n")
        if line.startswith("# "):
            # "# conn-a fly/fly-tab-actions Acro: visible"
            head, _, verdict = line[2:].rpartition(": ")
            parts = head.split(" ", 2)
            where = parts[1] if len(parts) > 1 else ""
            screen, _, page = where.partition("/")
            control = parts[2] if len(parts) > 2 else ""
            block = {"label": parts[0], "screen": screen, "page": page, "control": control,
                     "verdict": verdict.strip(), "clicks": []}
            blocks.append(block)
        elif line.startswith("click ") and block is not None:
            block["clicks"].append(line[6:])
        elif line.startswith("key ") and block is not None:
            # The HELP text's shortcuts the crawl pressed: played as keys, judged by the same deny.
            block["clicks"].append("key:" + line[4:])
    return blocks


def load_openers(out_dir):
    """The crawl's result rows: every control whose click opened something."""
    import glob
    import json
    found = set()
    for path in glob.glob(os.path.join(out_dir, "results-*.jsonl")):
        with open(path, errors="replace") as rows:
            for line in rows:
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if row.get("opened", 0):
                    found.add(row.get("id", ""))
    return found


def collapse_dialog_sequences(blocks):
    """Blocks on one page that reopen the same dialog - the review page's log chooser was opened
    three times over (the owner, 2026-10-17: "remove the first two sequences and leave only the last
    one") - keep the last; and a block whose clicks are only the opening of a kept block's dialog
    goes, as that block opens it anyway."""
    def sequence_key(block):
        """The clicks up to the first one inside a dialog: what opened it. None for a block that
        opens none."""
        for at, action in enumerate(block["clicks"]):
            if DIALOG_STEP.match(action.removeprefix("key:")):
                return (block["screen"], block["page"], tuple(block["clicks"][:at]))
        return None
    last_of = {}
    for index, block in enumerate(blocks):
        key = sequence_key(block)
        if key is not None:
            last_of[key] = index
    kept = []
    for index, block in enumerate(blocks):
        key = sequence_key(block)
        if key is not None and last_of[key] != index:
            continue
        kept.append(block)
    sequences = {(b["screen"], b["page"], tuple(b["clicks"])) for b in kept}
    def opens_a_kept_dialog(block):
        mine = tuple(block["clicks"])
        return any(s[0] == block["screen"] and s[1] == block["page"] and len(s[2]) > len(mine)
                   and s[2][:len(mine)] == mine for s in sequences)
    return [b for b in kept if not opens_a_kept_dialog(b)]


# A page's tab, the first click of a block on a paged screen.
PAGE_TAB = re.compile(r"-(page|tab)-")
# Families played once, by their first block alone: PLAN's draw area (the owner, 2026-10-17: "it
# goes to plan tab -> draw block and clicks a bunch of stuff, pls limit that block to just the
# first one in the 'draw' area").
ONCE_FAMILIES = [("plan", re.compile(r"^draw-"))]
# Played first, as the welcome demo ends, before the screens' rounds - the owner's word of
# 2026-10-17: "click config tab -> Planner -> Video Device Start as the first thing after sitl".
# The crawl denies Video Device Start (it opens a device); here it is his.
OPENING = [{"label": "owner", "screen": "config", "page": "config-page-ConfigPlanner",
            "control": "planner-BUT_videostart", "verdict": "visible",
            "clicks": ["config-page-ConfigPlanner", "planner-BUT_videostart"]}]


def collapse_families(blocks):
    """Of the blocks whose first click is one of a family named in `ONCE_FAMILIES`, the first."""
    seen, kept = set(), []
    for block in blocks:
        family = None
        for screen, pattern in ONCE_FAMILIES:
            if block["screen"] == screen and block["clicks"] and pattern.match(block["clicks"][0]):
                family = (screen, pattern.pattern)
        if family is not None:
            if family in seen:
                continue
            seen.add(family)
        kept.append(block)
    return kept


def collapse_window_sequences(blocks):
    """Blocks on one page that go through the same button after the page's tab - a window or
    sub-form opened from the page, the joystick setup from CONFIG > Planner, a drop-down's rows -
    keep the first alone (the owner, 2026-10-17: "it keeps going into config tab -> Planner ->
    Joystick setup, pls remove all but the first")."""
    seen, kept = set(), []
    for block in blocks:
        clicks = block["clicks"]
        if len(clicks) >= 3 and PAGE_TAB.search(clicks[0]):
            key = (block["screen"], block["page"], clicks[1])
            if key in seen:
                continue
            seen.add(key)
        kept.append(block)
    return kept


def allowed(block):
    if block["verdict"] not in PLAYED:
        return False
    if block["screen"] not in CONNECTED_SCREENS and not block["label"].startswith("disc"):
        return False
    for action in block["clicks"]:
        name = action.removeprefix("key:")
        if DENY.search(name) or DEMO_DENY.search(name) or name in OPENERS:
            return False
        if block["screen"] == "fly" and MODE_NAME.match(name):
            return False
    return bool(block["clicks"])


def clicks_to_play(at_nav, clicks):
    """A block's clicks less the leading ones already in effect: a page's tab clicked by the block
    before is not clicked again (the owner, 2026-10-17: "repeatedly clicking the 'Actions' tab when
    you are already on the Actions tab is unneeded clicks")."""
    at_nav = list(at_nav)
    if at_nav and len(clicks) > len(at_nav) and clicks[:len(at_nav)] == at_nav:
        return clicks[len(at_nav):]
    return clicks


class Tour:
    def __init__(self, app, dwell, log):
        self.app, self.dwell, self.log = app, dwell, log
        self.clicked = 0
        # The page-reaching clicks in effect: the last block's clicks but its last, on its screen.
        self.at_nav = ()
        # Pages a block missed on: their other blocks are skipped - a page the disconnected crawl
        # saw can look different connected, and every block of it would miss, a second each, with
        # the tour "sitting there doing nothing" (the owner, 2026-10-17).
        self.dead_pages = set()
        # The controls there when the tour begins: anything beyond them that can cancel or close is
        # a block's leftover, closed after the block (`dialogs_to_close`).
        self.ground = set(app.probe())

    def say(self, text):
        print(text, flush=True)
        self.log.write(text + "\n")
        self.log.flush()

    def glide(self, x, y, seconds=0.35, steps=10):
        """The pointer moved over `seconds` so a watcher can follow it, then at (x, y)."""
        where = self.app.sh("xdotool", "getmouselocation", "--shell").stdout
        try:
            at = dict(line.split("=") for line in where.split())
            x0, y0 = float(at["X"]), float(at["Y"])
            win = self.app.sh("xdotool", "getwindowgeometry", "--shell", self.app.win).stdout
            geo = dict(line.split("=") for line in win.split())
            x0 -= float(geo.get("X", 0)); y0 -= float(geo.get("Y", 0))
        except (ValueError, KeyError):
            x0, y0 = x, y
        for i in range(1, steps + 1):
            self.app.point(x0 + (x - x0) * i / steps, y0 + (y - y0) * i / steps)
            time.sleep(seconds / steps)

    def answer_boxes(self):
        probe = self.app.probe()
        for name in CLOSERS:
            if name in probe:
                self.click(name, note="answering")
                time.sleep(0.4)
                probe = self.app.probe()

    def close_dialogs(self, rounds=3):
        """Whatever the last block opened, closed: prompts cancelled, windows closed, a round at a
        time until nothing new is left."""
        for _ in range(rounds):
            leftovers = dialogs_to_close(self.ground, self.app.probe())
            if not leftovers:
                return
            self.click(leftovers[0], note="closing")
            time.sleep(0.5)

    def click(self, name, note=None, tries=4):
        """A visible click on `name`: waits for it to be measured, glides, clicks. A FLY page tab
        off the end of the tab strip is brought in with the strip's right arrow first, as a hand
        would (the rehearsal of 2026-10-17 found fly-tab-actions off screen after the demo)."""
        rect = None
        for attempt in range(tries):
            rect = self.app.probe().get(name)
            if rect:
                break
            if attempt == 3 and name.startswith("fly-tab-") and "fly-tabs-right" in self.app.probe():
                self.click("fly-tabs-right", note="revealing")
            time.sleep(0.25)
        if not rect:
            self.say(f"  {name}: not on screen")
            return False
        seen = self.app.seen(rect)
        if not seen:
            self.say(f"  {name}: off screen")
            return False
        x0, y0, x1, y1 = seen
        self.glide((x0 + x1) / 2, (y0 + y1) / 2)
        self.app.sh("xdotool", "click", "1")
        self.clicked += 1
        self.say(f"  {note + ' ' if note else ''}click {name}")
        return True

    def play(self, block):
        # A page the screen does not list now - the legacy firmware page, which SETUP shows only
        # disconnected (the owner's log of 2026-10-17: four misses in a row on it) - is skipped at
        # once, without the wait a missing control costs.
        first = block["clicks"][0].removeprefix("key:") if block["clicks"] else ""
        if first and not block["clicks"][0].startswith("key:") and first not in self.app.probe():
            # Nor any block whose first control is not there: a demo does not try what it cannot
            # see (the owner, 2026-10-17: "'not on screen' is a poor choice for a demo").
            if PAGE_TAB.search(first):
                self.dead_pages.add((block["screen"], block["page"]))
            self.say(f"{block['screen']}/{block['page']} {block['control']}: skipped, {first} is not there")
            return
        self.answer_boxes()
        todo = clicks_to_play(self.at_nav, block["clicks"])
        if todo is block["clicks"]:
            self.say(f"{block['screen']}/{block['page']} {block['control']}")
        done = True
        for action in todo:
            if action.startswith("key:"):
                self.app.key(action[4:])
                self.say(f"  key {action[4:]}")
            elif not self.click(action):
                done = False
                break
            time.sleep(self.dwell)
        # What is in effect now: the block's way to its control, if every click of it landed.
        self.at_nav = tuple(block["clicks"][:-1]) if done else ()
        if not done:
            self.dead_pages.add((block["screen"], block["page"]))
        self.answer_boxes()
        self.close_dialogs()
        self.app.key("Escape")
        time.sleep(0.3)

    def go_to(self, screen):
        """The screen's tab clicked, and the screen's own fact waited for before its blocks play;
        a tab click that a left-over menu or box swallowed is tried once more after an Escape (the
        second rehearsal of 2026-10-17 lost PLAN and PARAMS once each that way)."""
        for attempt in range(2):
            if self.app.facts().get("screen") == screen:
                return True
            self.at_nav = ()
            if attempt:
                self.answer_boxes()
                self.app.key("Escape")
                time.sleep(0.4)
            if not self.click(f"tab-{screen}"):
                continue
            for _ in range(12):
                if self.app.facts().get("screen") == screen:
                    return True
                time.sleep(0.25)
        self.say(f"  {screen}: the screen did not show")
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--probe", required=True)
    parser.add_argument("--facts", required=True)
    parser.add_argument("--recipe", default=os.path.join(ROOT, "tools", "gui-crawl", "out", "recipe.txt"))
    parser.add_argument("--minutes", type=float, default=6.0)
    parser.add_argument("--dwell", type=float, default=0.8)
    parser.add_argument("--screens", default=",".join(SCREENS))
    parser.add_argument("--per-screen", type=int, default=6, help="blocks played on a screen before the next")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--log", default=None)
    options = parser.parse_args()

    app = driver.App.attach(options.display, options.pid, options.probe, options.facts)
    log = open(options.log or os.path.join(os.path.dirname(options.facts), "tour.log"), "a")
    tour = Tour(app, options.dwell, log)
    OPENERS.update(load_openers(os.path.dirname(options.recipe)))
    blocks = collapse_families(collapse_window_sequences(
        collapse_dialog_sequences([b for b in read_recipe(options.recipe) if allowed(b)])))
    screens = [s for s in options.screens.split(",") if s]
    tail = [b for b in blocks if b["page"] in LAST_PAGES]
    by_screen = {s: [b for b in blocks if b["screen"] == s and b["page"] not in LAST_PAGES] for s in screens}
    random.Random(options.seed).shuffle  # the order is the recipe's; a seed is kept for a future shuffle
    tour.say(f"tour: {sum(len(v) for v in by_screen.values())} blocks on {len(screens)} screens, "
             f"{len(tail)} kept for the end, {options.minutes:g} min, dwell {options.dwell:g} s")
    deadline = time.time() + options.minutes * 60
    cursors = {s: 0 for s in screens}
    played = 0
    # The owner's opening, first.
    for block in OPENING:
        if tour.go_to(block["screen"]):
            tour.play(block)
            played += 1
    while time.time() < deadline and app.alive():
        progressed = False
        for screen in screens:
            queue = by_screen.get(screen, [])
            if cursors[screen] >= len(queue):
                continue
            if not tour.go_to(screen):
                cursors[screen] = len(queue)
                continue
            time.sleep(0.6)
            for _ in range(options.per_screen):
                if time.time() >= deadline or cursors[screen] >= len(queue):
                    break
                block = queue[cursors[screen]]
                cursors[screen] += 1
                if (block["screen"], block["page"]) in tour.dead_pages:
                    continue
                tour.play(block)
                played += 1
                progressed = True
        if not progressed:
            break
    # The Advanced page's blocks last, with whatever time is left.
    for block in tail:
        if time.time() >= deadline or not app.alive():
            break
        if block["screen"] in screens and not tour.go_to(block["screen"]):
            break
        tour.play(block)
        played += 1
    tour.say(f"tour done: {played} blocks, {tour.clicked} clicks")


if __name__ == "__main__":
    main()
