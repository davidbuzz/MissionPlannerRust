#!/usr/bin/env python3
"""A small NTRIP caster for tests/gui/config-rtk.gui: one mount point serving a recorded RTCM stream.

usage: rtk-caster.py --work DIR --stream FILE

Listens on 127.0.0.1 on a port the system picks, writes DIR/config.xml - Mission Planner's
settings, as the application reads them - so the RTK/GPS Inject page starts on NTRIP with this
caster's URL saved as NTRIP_url, and its .gpsbase log goes in DIR/logs; then returns, leaving
itself serving in the background.

It answers `GET /RTCM3` with `ICY 200 OK`, as NTRIP 1 casters and ntripcaster answer, waits half
a second so the answer reaches the client in a read of its own, then sends the stream a kilobyte
every tenth of a second - about ninety messages a second - from the start again when it ends.
Anything else gets the caster's SOURCETABLE refusal. It goes when DIR does, which the harness
removes after the run, or after five minutes.
"""

import argparse
import os
import socket
import sys
import time

import gui_background

MOUNT = "RTCM3"
CHUNK = 1000
PERIOD = 0.1
LIFETIME = 300.0


def config_xml(url, logs):
    keys = {
        "NTRIP_url": url,
        "SerialInjectGPS_port": "NTRIP",
        "logdirectory": logs,
    }
    body = "".join(f"\n  <{key}>{value}</{key}>" for key, value in sorted(keys.items()))
    return '﻿<?xml version="1.0" encoding="utf-8"?>\n<Config>' + body + "\n</Config>"


def serve(listener, stream, work, started):
    listener.settimeout(0.5)
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            client.settimeout(5.0)
            request = b""
            try:
                while b"\r\n\r\n" not in request and len(request) < 8192:
                    got = client.recv(1024)
                    if not got:
                        break
                    request += got
            except OSError:
                continue
            first = request.split(b"\r\n", 1)[0].decode("ascii", "replace")
            if not first.startswith(f"GET /{MOUNT} "):
                try:
                    client.sendall(b"SOURCETABLE 200 OK\r\n\r\nENDSOURCETABLE\r\n")
                except OSError:
                    pass
                continue
            try:
                client.sendall(b"ICY 200 OK\r\n\r\n")
                time.sleep(0.5)
                at = 0
                while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
                    piece = stream[at : at + CHUNK]
                    at += len(piece)
                    if at >= len(stream):
                        at = 0
                    client.sendall(piece)
                    time.sleep(PERIOD)
            except OSError:
                # The client hung up: wait for the next.
                continue


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--stream", required=True)
    args = parser.parse_args()

    with open(args.stream, "rb") as f:
        stream = f.read()
    if child is not None:
        serve(child, stream, args.work, time.monotonic())
        return 0
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(4)
    port = listener.getsockname()[1]

    logs = os.path.join(args.work, "logs")
    os.makedirs(logs, exist_ok=True)
    url = f"ntrip://headless-planner:rtk@127.0.0.1:{port}/{MOUNT}"
    with open(os.path.join(args.work, "config.xml"), "w", encoding="utf-8") as f:
        f.write(config_xml(url, logs))

    # Into the background: the harness waits for this command, and the caster must outlive it.
    gui_background.serve_in_background(
        listener,
        f"caster on {port}",
        lambda held: serve(held, stream, args.work, time.monotonic()),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
