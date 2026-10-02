#!/usr/bin/env python3
"""A copter with an onboard OSD, for the GUI script of the Onboard OSD page.

usage: osd-vehicle.py --work DIR [--port 5791]

The SITL copter lists one OSD parameter, OSD_TYPE, and no screens, so the page's editor - its
screens of items, the OSD5 and OSD6 parameter slots - is driven against this instead: an
ArduCopter 4.x with AP_OSD's whole table as the firmware lists it (OSD_* globals, OSD1 to OSD4
with their items, OSD5 and OSD6 with their nine parameter slots each) and the slot protocol.
It answers:

  HEARTBEAT at 1 Hz (a quad in Stabilize, disarmed), AUTOPILOT_VERSION when asked and once at
  connect; the parameters (PARAM_REQUEST_LIST, _READ, PARAM_SET echoed and kept);
  OSD_PARAM_SHOW_CONFIG with the slot's parameter, type and range (success with no name for a
  slot not set, an error for a screen or index it has not got); OSD_PARAM_CONFIG kept and
  acknowledged; any other COMMAND_LONG with UNSUPPORTED.

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
# ArduCopter 4.5.0, official.
FLIGHT_SW_VERSION = (4 << 24) | (5 << 16) | (0 << 8) | 0xFF
REAL32 = mavlink.MAV_PARAM_TYPE_REAL32
INT8 = mavlink.MAV_PARAM_TYPE_INT8
INT16 = mavlink.MAV_PARAM_TYPE_INT16
INT32 = mavlink.MAV_PARAM_TYPE_INT32

# AP_OSD_Screen's items with their default places on screen 1, and whether screen 1 shows them.
ITEMS = [
    ("ALTITUDE", 1, 23, 8),
    ("BAT_VOLT", 1, 24, 1),
    ("RSSI", 0, 1, 1),
    ("CURRENT", 1, 25, 2),
    ("BATUSED", 1, 23, 3),
    ("SATS", 1, 1, 3),
    ("FLTMODE", 1, 2, 8),
    ("MESSAGE", 1, 2, 14),
    ("GPSLAT", 0, 9, 14),
    ("GPSLONG", 0, 9, 15),
    ("HORIZON", 1, 14, 8),
    ("HOME", 1, 14, 1),
    ("HEADING", 0, 13, 2),
    ("THROTTLE", 0, 24, 11),
    ("COMPASS", 1, 15, 3),
    ("WIND", 0, 2, 12),
    ("ASPEED", 0, 2, 13),
    ("VSPEED", 1, 24, 9),
    ("GSPEED", 0, 2, 11),
    ("PITCH", 0, 2, 9),
    ("ROLL", 0, 2, 10),
    ("CLK", 0, 1, 12),
    ("SIDEBARS", 0, 4, 5),
    ("HDOP", 0, 1, 4),
]

GLOBALS = [
    ("OSD_TYPE", 1.0, INT8),
    ("OSD_CHAN", 0.0, INT8),
    ("OSD_SW_METHOD", 0.0, INT8),
    ("OSD_OPTIONS", 1.0, INT32),
    ("OSD_FONT", 0.0, INT8),
    ("OSD_V_OFFSET", 16.0, INT8),
    ("OSD_H_OFFSET", 18.0, INT8),
    ("OSD_UNITS", 0.0, INT8),
    ("OSD_MSG_TIME", 10.0, INT8),
    ("OSD_ARM_SCR", 0.0, INT8),
    ("OSD_DSARM_SCR", 0.0, INT8),
    ("OSD_FS_SCR", 0.0, INT8),
    ("OSD_BTN_DELAY", 300.0, INT16),
    ("OSD_W_RSSI", 30.0, INT8),
    ("OSD_W_NSAT", 9.0, INT8),
    ("OSD_W_BATVOLT", 10.0, REAL32),
    ("OSD_W_TERR", 0.0, INT8),
    ("OSD_W_AVGCELLV", 3.6, REAL32),
    ("OSD_CELL_COUNT", -1.0, INT8),
    ("OSD_W_RESTVOLT", 10.0, REAL32),
    ("OSD_TYPE2", 0.0, INT8),
]

OTHERS = [
    ("SYSID_THISMAV", 1.0, INT16),
    ("FRAME_CLASS", 1.0, INT8),
    ("RTL_ALT", 1500.0, INT32),
    ("WPNAV_SPEED", 1000.0, REAL32),
    ("ACRO_RP_P", 4.5, REAL32),
    ("ARMING_CHECK", 1.0, INT32),
    ("ANGLE_MAX", 3000.0, INT16),
    ("LAND_SPEED", 50.0, INT16),
    ("PILOT_SPEED_UP", 250.0, INT16),
    ("FS_THR_ENABLE", 1.0, INT8),
    ("SERIAL1_PROTOCOL", 2.0, INT8),
    ("SERVO9_FUNCTION", 0.0, INT16),
    ("RC7_OPTION", 0.0, INT16),
    ("FLTMODE1", 0.0, INT8),
    ("BATT_FS_LOW_ACT", 0.0, INT8),
    ("BATT_FS_CRT_ACT", 0.0, INT8),
]

# The slots as the vehicle starts: (screen, index) -> (name, type, min, max, incr).
SLOTS = {
    (5, 1): ("RTL_ALT", 0, 0.0, 10000.0, 100.0),
    (5, 2): ("WPNAV_SPEED", 0, 20.0, 2000.0, 50.0),
    (6, 1): ("FLTMODE1", 4, 0.0, 0.0, 0.0),
}


def build_params():
    params = [list(p) for p in OTHERS]
    params.extend(list(p) for p in GLOBALS)
    for screen in range(1, 5):
        params.append([f"OSD{screen}_ENABLE", 1.0 if screen == 1 else 0.0, INT8])
        params.append([f"OSD{screen}_CHAN_MIN", 900.0 + screen, INT16])
        params.append([f"OSD{screen}_CHAN_MAX", 2100.0, INT16])
        for name, enabled, x, y in ITEMS:
            params.append([f"OSD{screen}_{name}_EN", float(enabled if screen == 1 else 0), INT8])
            params.append([f"OSD{screen}_{name}_X", float(x), INT8])
            params.append([f"OSD{screen}_{name}_Y", float(y), INT8])
    for screen in (5, 6):
        params.append([f"OSD{screen}_ENABLE", 0.0, INT8])
        params.append([f"OSD{screen}_CHAN_MIN", 900.0, INT16])
        params.append([f"OSD{screen}_CHAN_MAX", 2100.0, INT16])
        params.append([f"OSD{screen}_SAVE_X", 23.0, INT8])
        params.append([f"OSD{screen}_SAVE_Y", 11.0, INT8])
        for index in range(1, 10):
            slot = SLOTS.get((screen, index))
            kind, lo, hi, inc = (slot[1], slot[2], slot[3], slot[4]) if slot else (0, 0.0, 1.0, 0.001)
            base = f"OSD{screen}_PARAM{index}"
            params.append([f"{base}_EN", 1.0 if index <= 3 else 0.0, INT8])
            params.append([f"{base}_X", 2.0, INT8])
            params.append([f"{base}_Y", float(index), INT8])
            params.append([f"{base}_KEY", 0.0, INT32])
            params.append([f"{base}_IDX", 0.0, INT16])
            params.append([f"{base}_GRP", 0.0, INT16])
            params.append([f"{base}_MIN", lo, REAL32])
            params.append([f"{base}_MAX", hi, REAL32])
            params.append([f"{base}_INCR", inc, REAL32])
            params.append([f"{base}_TYPE", float(kind), INT8])
    return params


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
        self.params = build_params()
        self.slots = dict(SLOTS)
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
        elif kind == "OSD_PARAM_SHOW_CONFIG":
            key = (msg.osd_screen, msg.osd_index)
            if msg.osd_screen not in (5, 6):
                mav.osd_param_show_config_reply_send(
                    msg.request_id, mavlink.OSD_PARAM_INVALID_SCREEN, b"", 0, 0.0, 0.0, 0.0
                )
            elif not 1 <= msg.osd_index <= 9:
                mav.osd_param_show_config_reply_send(
                    msg.request_id, mavlink.OSD_PARAM_INVALID_PARAMETER_INDEX, b"", 0, 0.0, 0.0, 0.0
                )
            elif key in self.slots:
                name, ptype, lo, hi, inc = self.slots[key]
                mav.osd_param_show_config_reply_send(
                    msg.request_id, mavlink.OSD_PARAM_SUCCESS, name.encode(), ptype, lo, hi, inc
                )
            else:
                mav.osd_param_show_config_reply_send(
                    msg.request_id, mavlink.OSD_PARAM_SUCCESS, b"", 0, 0.0, 1.0, 0.001
                )
        elif kind == "OSD_PARAM_CONFIG":
            key = (msg.osd_screen, msg.osd_index)
            if msg.osd_screen not in (5, 6) or not 1 <= msg.osd_index <= 9:
                mav.osd_param_config_reply_send(msg.request_id, mavlink.OSD_PARAM_INVALID_SCREEN)
            else:
                name = msg.param_id
                if isinstance(name, bytes):
                    name = name.split(b"\0", 1)[0].decode(errors="replace")
                else:
                    name = str(name).split("\0", 1)[0]
                self.slots[key] = (name, msg.config_type, msg.min_value, msg.max_value, msg.increment)
                base = f"OSD{msg.osd_screen}_PARAM{msg.osd_index}"
                for entry in self.params:
                    if entry[0] == f"{base}_TYPE":
                        entry[1] = float(msg.config_type)
                    elif entry[0] == f"{base}_MIN":
                        entry[1] = float(msg.min_value)
                    elif entry[0] == f"{base}_MAX":
                        entry[1] = float(msg.max_value)
                    elif entry[0] == f"{base}_INCR":
                        entry[1] = float(msg.increment)
                mav.osd_param_config_reply_send(msg.request_id, mavlink.OSD_PARAM_SUCCESS)

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
    parser.add_argument("--port", type=int, default=5791)
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
        listener, f"osd vehicle on {args.port}", lambda held: run(held, args.work)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
