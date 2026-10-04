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
# SPDX-License-Identifier: GPL-3.0-only
#
# Makes testdata/dataflash_damaged.bin: the damaged DataFlash log the parser, the converters and the
# log browser are tested against. It is what a log read off a card or a flash chip that was not
# erased between flights looks like: the file opens part way through a record of an older flight,
# carries on through that flight's records - whose FMT messages are gone, so every type is unknown -
# and then two newer boots start over it, each with its own FMTs: a short one that gets its GPS fix
# and is cut off part way through a record, and a flight, with a takeoff and mode changes, cut off
# the same way.
#
# Both flights are flown here on the bundled SITL copter (tools/sitl/arducopter, at Canberra Model
# Aircraft Club, SITL's default home), so the fixture's every byte is ArduPilot's own output:
#   older flight  GUIDED takeoff, a square of waypoints, LOITER, LAND, then sitting disarmed
#   short boot    the GPS fix and the position estimate, then a reboot
#   new flight    an arm asked for before the position estimate (refused, and nothing logged of it),
#                 the GPS fix, GUIDED takeoff and two legs, LOITER, LAND
# all logged from boot (LOG_DISARMED), at most 10 Hz a message (LOG_FILE_RATEMAX, as a small board
# logs) and without the simulated ESC telemetry, which would be most of the bytes. The file is the older flight's last 288 4 KiB blocks, from
# just before a record header past its last FMT (so its first header is at byte 18, as the tests and
# regen-log.sh expect) - then the short boot from its start to 48 blocks past its first GPS fix,
# and the new flight from its start to the end of the block after its LOITER, each cut wherever
# that lands in a record. So the readable part holds two boots, and its time goes back once.
#
# A new run flies new flights, so the fixture's numbers change with it: the tests that pin them, and
# the goldens (tools/csharp-reference/regen-log.sh), go with it.
#
# usage: tools/sitl/make-damaged-log.py [out [older.BIN short.BIN newer.BIN]]
#   needs pymavlink; out defaults to testdata/dataflash_damaged.bin. Given three logs, it splices those
#   rather than flying - a run keeps its logs and says where.
import os
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "testdata", "dataflash_damaged.bin")
INSTANCE = 4
PORT = 5760 + 10 * INSTANCE
BLOCK = 4096
HEADER = b"\xa3\x95"
OPENING = 18


def connect():
    deadline = time.time() + 60
    while True:
        try:
            link = mavutil.mavlink_connection(f"tcp:127.0.0.1:{PORT}", source_system=255)
            if link.wait_heartbeat(timeout=10):
                link.mav.request_data_stream_send(link.target_system, link.target_component,
                                                  mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                return link
        except OSError:
            pass
        if time.time() > deadline:
            sys.exit("SITL never answered")
        time.sleep(1)


def wait_for(link, test, seconds, what):
    deadline = time.time() + seconds
    while time.time() < deadline:
        message = link.recv_match(blocking=True, timeout=1)
        if message is not None and test(message):
            return message
    sys.exit(f"timed out waiting for {what}")


def command(link, cmd, *params):
    params = list(params) + [0] * (7 - len(params))
    link.mav.command_long_send(link.target_system, link.target_component, cmd, 0, *params)


def mode(link, name):
    link.set_mode(link.mode_mapping()[name])
    wait_for(link, lambda m: m.get_type() == "HEARTBEAT" and m.custom_mode == link.mode_mapping()[name],
             20, f"mode {name}")


def position_ready(link):
    # EKF_STATUS_REPORT flags: horizontal absolute position (0x10) and no constant position mode.
    wait_for(link, lambda m: m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3, 120, "a GPS fix")
    wait_for(link, lambda m: m.get_type() == "EKF_STATUS_REPORT" and m.flags & 0x10 and not m.flags & 0x80,
             120, "the position estimate")


def arm(link):
    for _ in range(30):
        command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        try:
            wait_for(link, lambda m: m.get_type() == "HEARTBEAT"
                     and m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 3, "arming")
            return
        except SystemExit:
            time.sleep(1)
    sys.exit("would not arm")


def altitude(link, metres):
    wait_for(link, lambda m: m.get_type() == "GLOBAL_POSITION_INT" and m.relative_alt >= metres * 900,
             120, f"{metres} m")


def goto(link, north, east, alt, seconds=8):
    link.mav.set_position_target_local_ned_send(0, link.target_system, link.target_component,
                                                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
                                                north, east, -alt, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(seconds)


def landed(link):
    wait_for(link, lambda m: m.get_type() == "HEARTBEAT"
             and not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 180, "landing")


def older_flight(link):
    position_ready(link)
    mode(link, "GUIDED")
    arm(link)
    command(link, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 20)
    altitude(link, 20)
    for north, east in [(60, 0), (60, 60), (0, 60), (0, 0)]:
        goto(link, north, east, 20)
    mode(link, "LOITER")
    time.sleep(10)
    mode(link, "LAND")
    landed(link)
    time.sleep(40)


def short_boot(link):
    position_ready(link)
    time.sleep(3)


def new_flight(link):
    # Asked for before the estimate is there; the vehicle refuses.
    command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    position_ready(link)
    mode(link, "GUIDED")
    arm(link)
    command(link, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
    altitude(link, 10)
    # Moving, so the KML has aircraft along the path and the GPX a track.
    for north, east in [(40, 0), (40, 40)]:
        goto(link, north, east, 10, seconds=3)
    mode(link, "LOITER")
    time.sleep(10)
    mode(link, "LAND")
    landed(link)


def formats(data):
    """Each type's record length, from every FMT in the log."""
    lengths = {128: 89}
    for at, kind, _ in records(data, 0, lengths):
        if kind == 128:
            lengths[data[at + 3]] = data[at + 4]
    return lengths


def records(data, start, lengths=None):
    """Where each record a header walk finds from `start` begins, with its type: the types in
    `lengths`, or those of the FMTs it has passed."""
    lengths = dict(lengths) if lengths else {128: 89}
    at = start
    while at + 3 <= len(data):
        if data[at:at + 2] == HEADER and data[at + 2] in lengths and at + lengths[data[at + 2]] <= len(data):
            kind = data[at + 2]
            if kind == 128:
                lengths[data[at + 3]] = data[at + 4]
            yield at, kind, data
            at += lengths[kind]
        else:
            at += 1


def splice(older, short, newer):
    # The older flight's last 288 blocks, from the first record past its last FMT with a clean
    # 18-byte run before it.
    last_fmt = max(at for at, kind, _ in records(older, 0) if kind == 128)
    lower = max(last_fmt + 1024, len(older) - 288 * BLOCK)
    start = next(at for at, _, _ in records(older, lower, formats(older))
                 if HEADER not in older[at - OPENING:at])
    body = older[start - OPENING:]
    body = body[:len(body) // BLOCK * BLOCK]
    # The short boot to 48 blocks past its first GPS record with a 3D fix (Status, after TimeUS
    # and the instance, at 3 + 8 + 1).
    gps = type_named(short, b"GPS\0")
    fix = next(at for at, kind, data in records(short, 0) if kind == gps and data[at + 12] >= 3)
    short_cut = (fix // BLOCK + 48) * BLOCK
    # The new flight up to the block after its switch to LOITER (MODE record, mode 5).
    loiter = next(at for at, kind, data in records(newer, 0)
                  if kind == type_named(newer, b"MODE") and data[at + 11] == 5)
    cut = (loiter // BLOCK + 2) * BLOCK
    return body + short[:short_cut] + newer[:cut]


def type_named(data, name):
    for at, kind, _ in records(data, 0):
        if kind == 128 and data[at + 5:at + 9] == name:
            return data[at + 3]
    sys.exit(f"no {name} format")


def fly():
    work = tempfile.mkdtemp(prefix="mp-damaged-log-")
    # LOG_DISARMED in the defaults, which -w loads at every boot, the reboot's too.
    defaults = os.path.join(work, "copter.parm")
    with open(defaults, "w") as out:
        out.write(open(os.path.join(HERE, "params", "copter.parm")).read()
                  + "\nLOG_DISARMED 1\nLOG_FILE_RATEMAX 10\nSIM_ESC_TELEM 0\n")
    sitl = subprocess.Popen([os.path.join(HERE, "arducopter"), "--model", "quad", "--speedup", "5", "-w",
                             "-I", str(INSTANCE), "--defaults", defaults],
                            cwd=work, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        link = connect()
        for flight in (older_flight, short_boot):
            flight(link)
            command(link, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
            time.sleep(5)
            link.close()
            link = connect()
        new_flight(link)
        time.sleep(3)
    finally:
        sitl.terminate()
        sitl.wait()
    logs = sorted(os.path.join(work, "logs", name) for name in os.listdir(os.path.join(work, "logs"))
                  if name.endswith(".BIN"))
    return logs[-3:]


def main():
    logs = sys.argv[2:5] if len(sys.argv) > 4 else fly()
    older, short, newer = (open(path, "rb").read() for path in logs)
    data = splice(older, short, newer)
    with open(OUT, "wb") as out:
        out.write(data)
    print(f"{OUT}: {len(data)} bytes from {', '.join(logs)}")


if __name__ == "__main__":
    main()
