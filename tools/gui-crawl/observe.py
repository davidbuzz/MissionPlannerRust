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
"""What a click did: the window before and after, and the planner's facts.

A click is judged on what a user could see: the pixels that changed outside the clicked control
(a focus border or a pressed look inside it is not an answer), a control appearing or vanishing,
the screen changing, a link or folder handed to the desktop. A blinking caret and the screen's own
animation are left out: the caret by a second capture after the first, the animation by a mask
taken while nothing was clicked. Behind the pixels, the facts: what changed there, less the
telemetry that drifts on its own.
"""
import re
import time

import numpy as np

# Facts that move with the vehicle or the clock, whatever is clicked.
TELEMETRY = re.compile(
    r"^(status\.serial$|fly\.quick|fly\.ekf|fly\.messages|fly\.xpdr|fly\.gauge|hud\.|raw\.|vehicle\.|link\.|"
    r"sticks\.|map\.(centre|ready|tiles|paint)|config\.failsafe\.out|config\.servo\.[0-9]+\.(pwm|bar)|"
    r"params\.table\.built|frames|recording|status\.drawn$)")
# The theme's colours (crates/mp-gui/src/ui.rs): ui::action draws a disabled button as dim text
# on the panel's colour, an enabled one on the action colour.
PANEL = np.array([0x24, 0x29, 0x2E])
ACTION = np.array([0x2D, 0x33, 0x3B])
DIM = np.array([0x8B, 0x94, 0x9E])
SEEN_PIXELS = 40
SMALL_PIXELS = 8


def share(crop, colour, tolerance):
    return float((np.abs(crop - colour).max(axis=1) <= tolerance).mean()) if crop.size else 0.0


def look(shot, rect):
    """How a control is drawn: `disabled` (dim on the panel), `live` (on the action colour) or
    `other`."""
    x0, y0 = max(int(rect["x"]) + 2, 0), max(int(rect["y"]) + 2, 0)
    x1, y1 = int(rect["x"] + rect["width"]) - 2, int(rect["y"] + rect["height"]) - 2
    crop = shot[y0:y1, x0:x1].reshape(-1, 3).astype(int)
    if share(crop, ACTION, 6) > 0.4:
        return "live"
    if share(crop, PANEL, 6) > 0.4 and share(crop, DIM, 12) > 0.01:
        return "disabled"
    return "other"


class Baseline:
    """The screen left alone for a second: the pixels and facts that change by themselves."""

    def __init__(self, app):
        first, facts1 = app.shot(), app.facts()
        time.sleep(1.0)
        second, facts2 = app.shot(), app.facts()
        self.mask = np.any(first != second, axis=2)
        self.facts = {k for k in set(facts1) | set(facts2) if facts1.get(k) != facts2.get(k)}


class Before:
    """The state a click is judged against."""

    def __init__(self, app, opened_count):
        app.park()
        time.sleep(0.1)
        self.shot = app.shot()
        self.facts = app.facts()
        self.probe = app.probe()
        self.opened = opened_count()


def judge(app, before, baseline, rect, opened_count, wait=0.7):
    """The click's outcome, after `wait` seconds, as a dict: its verdict and the evidence."""
    app.park()
    time.sleep(wait)
    after = app.shot()
    time.sleep(0.25)
    later = app.shot()
    facts = app.facts()
    probe = app.probe()
    opened = opened_count() - before.opened
    blink = np.any(after != later, axis=2)
    changed = np.any(before.shot != after, axis=2) & np.any(before.shot != later, axis=2)
    changed &= ~baseline.mask & ~blink
    inside = np.zeros_like(changed)
    y0, x0 = max(int(rect["y"]) - 3, 0), max(int(rect["x"]) - 3, 0)
    inside[y0:int(rect["y"] + rect["height"]) + 3, x0:int(rect["x"] + rect["width"]) + 3] = True
    outside_px = int((changed & ~inside).sum())
    inside_px = int((changed & inside).sum())
    moved = {k for k in set(before.facts) | set(facts) if before.facts.get(k) != facts.get(k)}
    own = sorted(k for k in moved if k not in baseline.facts and not TELEMETRY.match(k))
    appeared = sorted(set(probe) - set(before.probe))
    vanished = sorted(set(before.probe) - set(probe))
    status = facts.get("status") if facts.get("status") != before.facts.get("status") else None
    # The same words set again: nothing changes on screen, but the planner counts every message
    # (`status.serial`), so a repeated answer is told from none.
    repeated = (status is None and facts.get("status.serial") is not None
                and facts.get("status.serial") != before.facts.get("status.serial"))
    screen_changed = facts.get("screen") != before.facts.get("screen")
    if outside_px > SEEN_PIXELS or appeared or vanished or opened or screen_changed:
        verdict = "visible"
    elif inside_px > SEEN_PIXELS or (own and inside_px > SMALL_PIXELS):
        # A small change inside with the planner's own state changed is an answer too: a tick
        # in a box twelve pixels wide.
        verdict = "changed-itself"
    elif status:
        verdict = "status-only"
    elif repeated:
        verdict = "repeated-status"
    elif own:
        verdict = "internal-only"
    else:
        verdict = "nothing"
    if verdict == "nothing" and look(before.shot, rect) == "disabled":
        verdict = "disabled"
    return dict(verdict=verdict, outside_px=outside_px, inside_px=inside_px, facts=own[:12],
                status=status, opened=opened, appeared=appeared[:40], vanished=vanished[:40],
                screen_after=facts.get("screen"), alive=app.alive())
