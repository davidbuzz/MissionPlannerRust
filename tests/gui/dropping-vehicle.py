#!/usr/bin/env python3
"""A vehicle that hangs up, for the GUI script of the link's reconnection (link-reconnect.gui).

usage: dropping-vehicle.py --work DIR [--port 5795] [--drop-after 3] [--down 1.5]

Listens on 127.0.0.1:PORT and serves one client at a time: HEARTBEAT twice a second (a quad in
Stabilize, disarmed). DROP_AFTER seconds into the first connection it closes the socket - a SITL
rebooted, a radio gone - waits DOWN seconds before it accepts again, and then serves the next
client for as long as it lives. Returns once listening, leaving itself serving in the background
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


class Writer:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        self.sock.sendall(data)


def serve_client(client, work, started, hang_up_after):
    """Heartbeats until the client goes, the directory goes, or `hang_up_after` (seconds from now,
    or None) runs out - when the socket is shut as a vanished peer shuts it."""
    mav = mavlink.MAVLink(Writer(client), srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    client.settimeout(0.2)
    opened = time.monotonic()
    last_beat = 0.0
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        now = time.monotonic()
        if hang_up_after is not None and now - opened >= hang_up_after:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            return
        try:
            if now - last_beat >= 0.5:
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
            mav.parse_buffer(data)
        except OSError:
            return


def listen(port):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    deadline = time.monotonic() + 5.0
    while True:
        try:
            listener.bind(("127.0.0.1", port))
            break
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)
    listener.listen(1)
    return listener


def run(listener, work, drop_after, down):
    started = time.monotonic()
    port = listener.getsockname()[1]
    listener.settimeout(0.5)
    first = True
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            serve_client(client, work, started, drop_after if first else None)
        if first:
            first = False
            # Down: the listener closed too, so the planner's tries in this window are refused
            # as a rebooting SITL refuses them - a listener left bound would accept them into its
            # backlog at once and the reconnection would be over before anyone saw it.
            listener.close()
            time.sleep(down)
            listener = listen(port)
            listener.settimeout(0.5)


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5795)
    parser.add_argument("--drop-after", type=float, default=3.0)
    parser.add_argument("--down", type=float, default=1.5)
    args = parser.parse_args()
    if child is not None:
        run(child, args.work, args.drop_after, args.down)
        return 0

    listener = listen(args.port)
    gui_background.serve_in_background(
        listener,
        f"dropping vehicle on 127.0.0.1:{args.port}",
        lambda sock: run(sock, args.work, args.drop_after, args.down),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
