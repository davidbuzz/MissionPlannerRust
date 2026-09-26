#!/usr/bin/env python3
"""A copter that speaks the legacy fence and rally protocols, for the GUI scripts of the planner's
Geo-Fence and Rally Points menus.

usage: legacy-vehicle.py --work DIR [--port 5790]

Mission Planner hides those two menus over a vehicle that reports
MAV_PROTOCOL_CAPABILITY_MISSION_FENCE (FlightPlanner.cs:2680-2691) - every ArduPilot 4.x, the SITL
among them - so they are driven against this instead: an ArduCopter as firmware before 4.0 was,
whose AUTOPILOT_VERSION carries MISSION_FLOAT, PARAM_FLOAT, MISSION_INT, COMMAND_INT and MAVLINK2
and neither MISSION_FENCE nor MISSION_RALLY nor FTP. It answers:

  HEARTBEAT at 1 Hz (a quad in Stabilize, disarmed), AUTOPILOT_VERSION when asked and once at
  connect; the parameters (PARAM_REQUEST_LIST, _READ, PARAM_SET echoed); FENCE_POINT kept and
  FENCE_FETCH_POINT answered, RALLY_POINT kept and RALLY_FETCH_POINT answered; the mission
  protocol for every list type (MISSION_COUNT and the items it asks for, MISSION_REQUEST_LIST and
  the items asked of it, MISSION_CLEAR_ALL); SET_MODE; any other COMMAND_LONG with UNSUPPORTED.

Listens on 127.0.0.1:PORT, one client at a time, and returns once listening, leaving itself
serving in the background. It goes when DIR does, which the harness removes after the run, or
after five minutes.
"""
import argparse
import os
import socket
import sys
import threading
import time

import gui_background

from pymavlink.dialects.v20 import ardupilotmega as mavlink

LIFETIME = 300.0
GCS = (255, 190)
# MISSION_FLOAT | PARAM_FLOAT | MISSION_INT | COMMAND_INT | MAVLINK2.
CAPABILITIES = 0x1 | 0x2 | 0x4 | 0x8 | 0x2000
# ArduCopter 3.6.0, official.
FLIGHT_SW_VERSION = (3 << 24) | (6 << 16) | (0 << 8) | 0xFF
REAL32 = mavlink.MAV_PARAM_TYPE_REAL32
INT8 = mavlink.MAV_PARAM_TYPE_INT8
INT16 = mavlink.MAV_PARAM_TYPE_INT16

PARAMS = [
    ("FENCE_ENABLE", 0.0, INT8),
    ("FENCE_ACTION", 1.0, INT8),
    ("FENCE_TOTAL", 0.0, INT8),
    ("FENCE_RADIUS", 150.0, REAL32),
    ("FENCE_ALT_MAX", 100.0, REAL32),
    ("RALLY_TOTAL", 0.0, INT8),
    ("RALLY_LIMIT_KM", 0.3, REAL32),
    ("SYSID_THISMAV", 1.0, INT16),
    ("FRAME_CLASS", 1.0, INT8),
]


class Writer:
    """The socket as pymavlink's file."""

    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()

    def write(self, data):
        with self.lock:
            self.sock.sendall(data)


class Vehicle:
    def __init__(self):
        self.params = [list(p) for p in PARAMS]
        self.fence = {}
        self.rally = {}
        self.lists = {}
        self.upload = None  # (mission_type, count, items so far)
        self.custom_mode = 0

    def param_value(self, mav, index):
        name, value, kind = self.params[index]
        mav.param_value_send(name.encode(), value, kind, len(self.params), index)

    def handle(self, mav, msg):
        kind = msg.get_type()
        if kind == "PARAM_REQUEST_LIST":
            for index in range(len(self.params)):
                self.param_value(mav, index)
        elif kind == "PARAM_REQUEST_READ":
            for index, (name, _, _) in enumerate(self.params):
                if index == msg.param_index or name == msg.param_id:
                    self.param_value(mav, index)
                    break
        elif kind == "PARAM_SET":
            for index, entry in enumerate(self.params):
                if entry[0] == msg.param_id:
                    entry[1] = float(msg.param_value)
                    if entry[0] == "RALLY_TOTAL":
                        # ArduPilot's AP_Rally is one store for both protocols and RALLY_TOTAL its
                        # length: Clear Rally Points' RALLY_TOTAL=0 empties the mission-protocol
                        # list a Download reads as well as the legacy points.
                        total = max(int(entry[1]), 0)
                        self.lists[mavlink.MAV_MISSION_TYPE_RALLY] = self.lists.get(
                            mavlink.MAV_MISSION_TYPE_RALLY, []
                        )[:total]
                        self.rally = {i: p for i, p in self.rally.items() if i < total}
                    self.param_value(mav, index)
                    break
        elif kind == "COMMAND_LONG":
            wanted = int(msg.param1)
            if msg.command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES or (
                msg.command == mavlink.MAV_CMD_REQUEST_MESSAGE
                and wanted == mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
            ):
                mav.command_ack_send(msg.command, mavlink.MAV_RESULT_ACCEPTED)
                self.autopilot_version(mav)
            else:
                mav.command_ack_send(msg.command, mavlink.MAV_RESULT_UNSUPPORTED)
        elif kind == "SET_MODE":
            self.custom_mode = msg.custom_mode
        elif kind == "FENCE_POINT":
            self.fence[msg.idx] = (msg.count, msg.lat, msg.lng)
        elif kind == "FENCE_FETCH_POINT":
            if msg.idx in self.fence:
                count, lat, lng = self.fence[msg.idx]
                mav.fence_point_send(*GCS, msg.idx, count, lat, lng)
        elif kind == "RALLY_POINT":
            self.rally[msg.idx] = msg
        elif kind == "RALLY_FETCH_POINT":
            if msg.idx in self.rally:
                p = self.rally[msg.idx]
                mav.rally_point_send(
                    *GCS, p.idx, p.count, p.lat, p.lng, p.alt, p.break_alt, p.land_dir, p.flags
                )
        elif kind == "MISSION_REQUEST_LIST":
            items = self.lists.get(msg.mission_type, [])
            mav.mission_count_send(*GCS, len(items), msg.mission_type)
        elif kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            if self.upload is None:
                items = self.lists.get(msg.mission_type, [])
                if msg.seq < len(items):
                    item = items[msg.seq]
                    mav.mission_item_int_send(
                        *GCS, msg.seq, item.frame, item.command, 0, item.autocontinue,
                        item.param1, item.param2, item.param3, item.param4,
                        item.x, item.y, item.z, msg.mission_type,
                    )
        elif kind == "MISSION_COUNT":
            self.upload = (msg.mission_type, msg.count, [])
            if msg.count == 0:
                self.lists[msg.mission_type] = []
                self.upload = None
                mav.mission_ack_send(*GCS, mavlink.MAV_MISSION_ACCEPTED, msg.mission_type)
            else:
                mav.mission_request_int_send(*GCS, 0, msg.mission_type)
        elif kind == "MISSION_ITEM_INT":
            if self.upload is not None and self.upload[0] == msg.mission_type:
                mission_type, count, items = self.upload
                if msg.seq == len(items):
                    items.append(msg)
                if len(items) < count:
                    mav.mission_request_int_send(*GCS, len(items), mission_type)
                else:
                    self.lists[mission_type] = items
                    self.upload = None
                    mav.mission_ack_send(*GCS, mavlink.MAV_MISSION_ACCEPTED, mission_type)
        elif kind == "MISSION_CLEAR_ALL":
            self.lists[msg.mission_type] = []
            mav.mission_ack_send(*GCS, mavlink.MAV_MISSION_ACCEPTED, msg.mission_type)

    def autopilot_version(self, mav):
        mav.autopilot_version_send(
            CAPABILITIES, FLIGHT_SW_VERSION, 0, 0, 0,
            [0] * 8, [0] * 8, [0] * 8, 0, 0, 0,
        )


def serve_client(client, vehicle, work, started):
    mav = mavlink.MAVLink(Writer(client), srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    client.settimeout(0.2)
    last_beat = 0.0
    greeted = False
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        now = time.monotonic()
        try:
            if now - last_beat >= 1.0:
                mav.heartbeat_send(
                    mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 80, vehicle.custom_mode,
                    mavlink.MAV_STATE_STANDBY,
                )
                last_beat = now
                if not greeted:
                    vehicle.autopilot_version(mav)
                    greeted = True
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            if not data:
                return
            for msg in mav.parse_buffer(data) or []:
                vehicle.handle(mav, msg)
        except OSError:
            return


def run(listener, work):
    """The vehicle, answering one client at a time until its directory goes or LIFETIME ends."""
    started = time.monotonic()
    vehicle = Vehicle()
    listener.settimeout(0.5)
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            serve_client(client, vehicle, work, started)


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5790)
    args = parser.parse_args()
    if child is not None:
        run(child, args.work)
        return 0

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # The last script's vehicle may still be leaving: it looks for its directory twice a second.
    deadline = time.monotonic() + 5.0
    while True:
        try:
            listener.bind(("127.0.0.1", args.port))
            break
        except OSError:
            if time.monotonic() > deadline:
                print(f"port {args.port} is busy", file=sys.stderr)
                return 3
            time.sleep(0.2)
    listener.listen(1)

    # Into the background: the harness waits for this command, and the vehicle must outlive it.
    gui_background.serve_in_background(
        listener, f"legacy vehicle on {args.port}", lambda held: run(held, args.work)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
