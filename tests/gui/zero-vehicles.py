#!/usr/bin/env python3
"""Two vehicles on one link, one of them without a fix, for the GUI script of the map's
zero-position rule (map-zero-position.gui).

usage: zero-vehicles.py --work DIR [--port 5796]

Listens on 127.0.0.1:PORT and serves one client at a time. System 1 is a quad at Brisbane:
HEARTBEAT at 1 Hz and GLOBAL_POSITION_INT at 2 Hz, creeping north so the flown route has a few
points. System 2 is a quad whose GPS has no fix: HEARTBEAT at 1 Hz and GLOBAL_POSITION_INT at
latitude 0, longitude 153 - the position a vehicle reports with a longitude and no latitude yet,
and the one that zoomed the owner's map out to half the world (2026-10-03). System 1 also
reports one ADSB_VEHICLE at 0,0 every second. Everything else the planner asks for is ignored.
Returns once listening, leaving itself serving in the background (gui_background); goes when DIR
does, or after five minutes.
"""
import argparse
import os
import socket
import sys
import time

import gui_background

from pymavlink.dialects.v20 import ardupilotmega as mavlink

LIFETIME = 300.0
BRISBANE = (-274698000, 1530251000)


class Writer:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        self.sock.sendall(data)


def serve_client(client, work, started):
    writer = Writer(client)
    fixed = mavlink.MAVLink(writer, srcSystem=1, srcComponent=1)
    unfixed = mavlink.MAVLink(writer, srcSystem=2, srcComponent=1)
    for mav in (fixed, unfixed):
        mav.robust_parsing = True
    client.settimeout(0.1)
    opened = time.monotonic()
    last_beat = 0.0
    last_position = 0.0
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        now = time.monotonic()
        try:
            if now - last_beat >= 1.0:
                for mav in (fixed, unfixed):
                    mav.heartbeat_send(
                        mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, mavlink.MAV_STATE_STANDBY,
                    )
                fixed.adsb_vehicle_send(
                    0xABCDEF, 0, 0, mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC, 0, 0, 0, 0,
                    b"NOFIX   ", mavlink.ADSB_EMITTER_TYPE_LIGHT, 1,
                    mavlink.ADSB_FLAGS_VALID_CALLSIGN, 0,
                )
                last_beat = now
            if now - last_position >= 0.5:
                boot_ms = int((now - opened) * 1000)
                # North a metre a report: a route with points, not a dot.
                lat = BRISBANE[0] + int((now - opened) * 20)
                fixed.global_position_int_send(
                    boot_ms, lat, BRISBANE[1], 40000, 10000, 0, 0, 0, 9000,
                )
                unfixed.global_position_int_send(
                    boot_ms, 0, 1530000000, 0, 0, 0, 0, 0, 0,
                )
                last_position = now
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            if not data:
                return
            fixed.parse_buffer(data)
        except OSError:
            return


def run(listener, work):
    started = time.monotonic()
    listener.settimeout(0.5)
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            serve_client(client, work, started)


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5796)
    args = parser.parse_args()
    if child is not None:
        run(child, args.work)
        return 0

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    deadline = time.monotonic() + 5.0
    while True:
        try:
            listener.bind(("127.0.0.1", args.port))
            break
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)
    listener.listen(1)
    gui_background.serve_in_background(
        listener,
        f"two vehicles on 127.0.0.1:{args.port}",
        lambda sock: run(sock, args.work),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
