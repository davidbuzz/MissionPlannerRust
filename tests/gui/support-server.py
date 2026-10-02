#!/usr/bin/env python3
"""A support server, for the GUI script of the Support Proxy (Controls/SerialSupportProxy.cs):
what support.ardupilot.org is to the proxy - a UDP port the proxy sends the vehicle's packets to,
and from which a support engineer's ground station's packets come back.

usage: support-server.py --work DIR [--port 5799]

Listens on UDP 127.0.0.1:PORT. For each HEARTBEAT it hears from the vehicle (system 1), relayed
by the proxy, it answers the address it came from with a ground station's HEARTBEAT from system
253, component 190, and a PARAM_REQUEST_READ of SYSID_THISMAV for the vehicle: the support
engineer asking, through the proxy, which writes both to the vehicle. Only an answer to what was
relayed is sent, so what the proxy relays to the vehicle shows that it relayed from it.

Returns once listening, leaving itself serving in the background. It goes when DIR does, which
the harness removes after the run, or after five minutes.
"""
import argparse
import os
import socket
import sys
import time

import gui_background

from pymavlink.dialects.v20 import ardupilotmega as mavlink

LIFETIME = 300.0
# The support engineer's ground station.
ENGINEER = (253, 190)
VEHICLE = 1


def run(sock, work):
    """Answers each relayed vehicle heartbeat until the directory goes or LIFETIME ends."""
    started = time.monotonic()
    sock.settimeout(0.5)
    engineer = mavlink.MAVLink(None, srcSystem=ENGINEER[0], srcComponent=ENGINEER[1])
    reader = mavlink.MAVLink(None)
    reader.robust_parsing = True
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            data, peer = sock.recvfrom(65536)
        except socket.timeout:
            continue
        except OSError:
            return
        try:
            messages = reader.parse_buffer(data) or []
        except mavlink.MAVError:
            continue
        for message in messages:
            if message.get_type() != "HEARTBEAT" or message.get_srcSystem() != VEHICLE:
                continue
            heartbeat = engineer.heartbeat_encode(
                mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
            )
            request = engineer.param_request_read_encode(VEHICLE, 1, b"SYSID_THISMAV", -1)
            try:
                sock.sendto(heartbeat.pack(engineer), peer)
                sock.sendto(request.pack(engineer), peer)
            except OSError:
                continue


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5799)
    args = parser.parse_args()
    if child is not None:
        run(child, args.work)
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # The last script's server may still be leaving: it looks for its directory twice a second.
    deadline = time.monotonic() + 5.0
    while True:
        try:
            sock.bind(("127.0.0.1", args.port))
            break
        except OSError:
            if time.monotonic() > deadline:
                print(f"port {args.port} is busy", file=sys.stderr)
                return 3
            time.sleep(0.2)

    # Into the background: the harness waits for this command, and the server must outlive it.
    gui_background.serve_in_background(
        sock, f"support server on udp {args.port}", lambda held: run(held, args.work)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
