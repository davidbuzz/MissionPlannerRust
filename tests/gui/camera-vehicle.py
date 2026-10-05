#!/usr/bin/env python3
"""A copter with a MAVLink camera on board, for the GUI script of Ctrl+Z's camera test
(keys-camera-test.gui).

usage: camera-vehicle.py --work DIR [--port 5797]

Listens on 127.0.0.1:PORT and serves one client at a time. System 1 component 1 is a quad,
component 100 (MAV_COMP_ID_CAMERA) a camera: each sends HEARTBEAT at 1 Hz. The camera acknowledges
every COMMAND_LONG addressed to it - MAV_RESULT_DENIED for REQUEST_VIDEO_STREAM_INFORMATION, which
the test must move on from as from an acceptance, MAV_RESULT_ACCEPTED for the rest - and writes
each command's number, one a line, to DIR/camera-commands.txt. Everything else the planner asks
for is ignored. Returns once listening, leaving itself serving in the background
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
CAMERA = 100


class Writer:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        self.sock.sendall(data)


def serve_client(client, work, started):
    writer = Writer(client)
    copter = mavlink.MAVLink(writer, srcSystem=1, srcComponent=1)
    camera = mavlink.MAVLink(writer, srcSystem=1, srcComponent=CAMERA)
    for mav in (copter, camera):
        mav.robust_parsing = True
    client.settimeout(0.1)
    last_beat = 0.0
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        now = time.monotonic()
        try:
            if now - last_beat >= 1.0:
                copter.heartbeat_send(
                    mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, mavlink.MAV_STATE_STANDBY,
                )
                camera.heartbeat_send(
                    mavlink.MAV_TYPE_CAMERA, mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                    mavlink.MAV_STATE_ACTIVE,
                )
                last_beat = now
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            if not data:
                return
            for message in copter.parse_buffer(data) or []:
                if message.get_type() != "COMMAND_LONG":
                    continue
                if (message.target_system, message.target_component) != (1, CAMERA):
                    continue
                with open(os.path.join(work, "camera-commands.txt"), "a") as log:
                    log.write(f"{message.command}\n")
                result = (
                    mavlink.MAV_RESULT_DENIED
                    if message.command == mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
                    else mavlink.MAV_RESULT_ACCEPTED
                )
                camera.command_ack_send(
                    message.command, result, 0, 0,
                    message.get_srcSystem(), message.get_srcComponent(),
                )
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
    parser.add_argument("--port", type=int, default=5797)
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
        f"a copter and its camera on 127.0.0.1:{args.port}",
        lambda sock: run(sock, args.work),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
