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
"""The planner on a virtual display, driven by its own probe.

The planner publishes every named control's rectangle (`MP_PROBE`) and what it believes
(`MP_FACTS`); this starts it on an Xvfb display with lavapipe (tools/gui-headless.sh's setup),
reads both, captures the window with `xwd`, and sends the pointer and keys with `xdotool`. Nothing
here touches the desktop's display.
"""
import json
import os
import shutil
import struct
import subprocess
import tempfile
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ICD = "/usr/share/vulkan/icd.d/lvp_icd.json"
# Desktop openers replaced by ones that log their arguments: a link or a folder handed to the
# desktop is a response the user would see, and must not open anything here.
OPENERS = ["xdg-open", "gio", "open", "firefox", "sensible-browser", "x-www-browser", "nautilus"]


def ensure_display(display):
    """An Xvfb on `display` (":99"), started when nothing listens there."""
    socket = f"/tmp/.X11-unix/X{display.lstrip(':')}"
    if os.path.exists(socket):
        return
    subprocess.Popen(
        ["setsid", "Xvfb", display, "-screen", "0", "2560x1440x24", "+extension", "GLX",
         "+extension", "RANDR", "+render", "-noreset"],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(50):
        if os.path.exists(socket):
            return
        time.sleep(0.1)
    raise RuntimeError(f"Xvfb {display} did not start")


def fake_openers(directory, log):
    """The desktop openers as scripts appending their arguments to `log`."""
    os.makedirs(directory, exist_ok=True)
    for name in OPENERS:
        path = os.path.join(directory, name)
        with open(path, "w") as f:
            f.write(f'#!/bin/sh\necho "{name} $*" >> "{log}"\n')
        os.chmod(path, 0o755)


class App:
    """One planner window: started on a screen, with arguments (a link URL), alone in a scratch
    data directory so no setting carries from one launch to the next."""

    def __init__(self, display, screen, args=(), binary=None, openers=None, window="1600x1200"):
        self.display = display
        self.env0 = dict(os.environ, DISPLAY=display, VK_ICD_FILENAMES=ICD)
        self.work = tempfile.mkdtemp(prefix="gui-crawl-")
        self.probe_file = os.path.join(self.work, "probe.json")
        self.facts_file = os.path.join(self.work, "facts.txt")
        env = dict(self.env0, MP_SCREEN=screen, MP_WINDOW=window, MP_PROBE=self.probe_file,
                   MP_FACTS=self.facts_file, XDG_DATA_HOME=self.work, HOME=self.work,
                   MP_CONFIG_XML=os.path.join(self.work, "config.xml"),
                   # Offline - a map source configured and nothing fetched, as a script's
                   # `tiles offline` has it, so the map type buttons act (with MP_NO_TILES they
                   # did nothing, 2026-10-15) - and the firmware pages shown a made-up board in
                   # place of the machine's ports, so Force Bootloader and Bootloader Update
                   # refuse any real serial device (config/firmware.rs's DEVICE_ENV).
                   MP_OFFLINE="1", MP_FIRMWARE_DEVICE="gui-crawl, no device",
                   # A fake Tailscale on a tailnet, as tools/gui-test.sh gives every script: the
                   # crawl never reads this computer's own, and no first-start offer covers it.
                   MP_TAILSCALE_CLI=os.path.join(
                       os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "tests", "gui", "fake-tailscale.sh"),
                   MP_LOG_DIR=os.path.join(self.work, "logs"),
                   MP_PLAN_DIR=os.path.join(self.work, "plans"))
        if openers:
            env["PATH"] = openers + ":" + env["PATH"]
        os.makedirs(os.path.join(self.work, "logs"), exist_ok=True)
        binary = binary or os.path.join(ROOT, "target", "debug", "planner")
        self.stderr = open(os.path.join(self.work, "stderr.txt"), "w")
        self.proc = subprocess.Popen([binary, *args], env=env, cwd=self.work,
                                     stdout=subprocess.DEVNULL, stderr=self.stderr)
        self.win = None
        deadline = time.time() + 60
        while time.time() < deadline:
            found = self.sh("xdotool", "search", "--pid", str(self.proc.pid), "--onlyvisible",
                            "--name", ".").stdout.split()
            if found and os.path.exists(self.probe_file):
                self.win = found[-1]
                break
            if self.proc.poll() is not None:
                break
            time.sleep(0.25)
        if not self.win:
            tail = open(os.path.join(self.work, "stderr.txt")).read()[-1500:]
            self.close()
            raise RuntimeError("the planner did not open its window: " + tail)
        self.sh("xdotool", "windowmove", self.win, "0", "0")
        time.sleep(1.5)

    def sh(self, *argv):
        return subprocess.run(argv, capture_output=True, text=True, env=self.env0)

    def alive(self):
        return self.proc.poll() is None

    def probe(self):
        for _ in range(20):
            try:
                with open(self.probe_file) as f:
                    return json.load(f)
            except (OSError, ValueError):
                time.sleep(0.05)
        return {}

    def facts(self):
        out = {}
        try:
            with open(self.facts_file, errors="replace") as f:
                for line in f:
                    if " = " in line:
                        key, value = line.rstrip("\n").split(" = ", 1)
                        out[key] = value
        except OSError:
            pass
        return out

    def shot(self):
        """The window's pixels, height x width x RGB.

        Each pixel a 32-bit word in the header's byte order, its channels where the header's
        masks put them: on an X server that is LSBFirst, as Xvfb is, the bytes run B, G, R, so
        reading them as they lie gave BGR, and observe.look() - whose colours are RGB - never
        found a dimmed control (2026-10-15)."""
        data = subprocess.run(["xwd", "-silent", "-id", self.win], capture_output=True,
                              env=self.env0).stdout
        header, = struct.unpack(">I", data[0:4])
        width, height = struct.unpack(">II", data[16:24])
        byte_order, = struct.unpack(">I", data[28:32])
        per_line, = struct.unpack(">I", data[48:52])
        masks = struct.unpack(">III", data[56:68])
        colours, = struct.unpack(">I", data[76:80])
        start = header + colours * 12
        words = np.frombuffer(data[start:start + per_line * height],
                              dtype="<u4" if byte_order == 0 else ">u4")
        words = words.reshape(height, per_line // 4)[:, :width]
        channels = []
        for mask in masks:
            shift = (mask & -mask).bit_length() - 1
            channels.append(((words & mask) >> shift).astype(np.uint8))
        return np.stack(channels, axis=2)

    @staticmethod
    def seen(rect):
        """The part of a control that could be seen where it was laid out, or None."""
        vx, vy = rect.get("visible_x", 0.0), rect.get("visible_y", 0.0)
        vw, vh = rect.get("visible_width", 1e9), rect.get("visible_height", 1e9)
        x0, x1 = max(rect["x"], vx), min(rect["x"] + rect["width"], vx + vw)
        y0, y1 = max(rect["y"], vy), min(rect["y"] + rect["height"], vy + vh)
        if x1 - x0 < 2 or y1 - y0 < 2:
            return None
        return x0, y0, x1, y1

    def point(self, x, y):
        self.sh("xdotool", "mousemove", "--window", self.win, str(int(x)), str(int(y)))

    def click(self, name, rect=None, button="1", at=None):
        """A click on a control's visible centre - or at fractions `at` of it - with `button`
        (1 left, 3 right). False when the control is not on screen."""
        rect = rect or self.probe().get(name)
        if not rect:
            return False
        seen = self.seen(rect)
        if not seen:
            return False
        x0, y0, x1, y1 = seen
        fx, fy = at or (0.5, 0.5)
        self.point(x0 + (x1 - x0) * fx, y0 + (y1 - y0) * fy)
        self.sh("xdotool", "click", button)
        return True

    def park(self):
        """The pointer off every control, at the window's bottom-right corner."""
        self.point(1595, 1195)

    def key(self, name):
        """A key to the window, the window given the keyboard first without a click."""
        self.sh("xdotool", "windowactivate", "--sync", self.win)
        self.sh("xdotool", "windowfocus", "--sync", self.win)
        self.sh("xdotool", "key", "--window", self.win, "--clearmodifiers", name)

    def wheel(self, x, y, down, notches=3):
        self.point(x, y)
        self.sh("xdotool", "click", "--repeat", str(notches), "--delay", "40", "5" if down else "4")

    def close(self):
        try:
            self.proc.terminate()
            self.proc.wait(5)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        try:
            self.stderr.close()
        except Exception:
            pass
        shutil.rmtree(self.work, ignore_errors=True)
