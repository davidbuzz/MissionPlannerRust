#!/usr/bin/env python3
"""A copter holding two dataflash logs, for the GUI script of the Log Downloader's Clear Logs
(logs-downloader-clear.gui).

usage: log-vehicle.py --work DIR [--port 5792]

Listens on 127.0.0.1:PORT and serves one client at a time. System 1 component 1 is a quad:
HEARTBEAT at 1 Hz. LOG_REQUEST_LIST addressed to it is answered with a LOG_ENTRY for each log it
holds - or, holding none, the one LOG_ENTRY of no logs ArduPilot sends - then a STATUSTEXT,
"listed N", so a script knows the whole list has arrived. A LOG_ERASE addressed to it as Mission
Planner's EraseLog addresses it, system 1 component 1 (MAVLinkInterface.cs:6269-6279), erases
every log, and each one is counted aloud: STATUSTEXT "LOG_ERASE N". Everything else the planner
asks for is ignored. Returns once listening, leaving itself serving in the background
(gui_background); goes when DIR does, or after five minutes.
"""
import argparse
import os
import socket
import sys
import time

import gui_background

from pymavlink.dialects.v20 import ardupilotmega as mavlink

LIFETIME = 300.0
# The logs held until erased: id, size in bytes, and when each began, Unix seconds.
LOGS = [(1, 2048, 1700000000), (2, 4096, 1700003600)]


class Writer:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        self.sock.sendall(data)


class Copter:
    """What survives a client: the logs held, and the LOG_ERASEs counted."""

    def __init__(self):
        self.logs = list(LOGS)
        self.erases = 0


def serve_client(client, work, started, copter):
    mav = mavlink.MAVLink(Writer(client), srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    client.settimeout(0.1)
    last_beat = 0.0
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        now = time.monotonic()
        try:
            if now - last_beat >= 1.0:
                mav.heartbeat_send(
                    mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, mavlink.MAV_STATE_STANDBY,
                )
                last_beat = now
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            if not data:
                return
            for message in mav.parse_buffer(data) or []:
                kind = message.get_type()
                if kind == "LOG_REQUEST_LIST" and message.target_system == 1:
                    if copter.logs:
                        last = copter.logs[-1][0]
                        for log_id, size, began in copter.logs:
                            mav.log_entry_send(log_id, len(copter.logs), last, began, size)
                    else:
                        mav.log_entry_send(0, 0, 0, 0, 0)
                    mav.statustext_send(
                        mavlink.MAV_SEVERITY_INFO, f"listed {len(copter.logs)}".encode()
                    )
                elif kind == "LOG_ERASE" and (
                    message.target_system, message.target_component
                ) == (1, 1):
                    copter.logs = []
                    copter.erases += 1
                    mav.statustext_send(
                        mavlink.MAV_SEVERITY_INFO, f"LOG_ERASE {copter.erases}".encode()
                    )
        except OSError:
            return


def run(listener, work):
    started = time.monotonic()
    copter = Copter()
    listener.settimeout(0.5)
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            serve_client(client, work, started, copter)


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5792)
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
        f"a copter holding two logs on 127.0.0.1:{args.port}",
        lambda sock: run(sock, args.work),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
