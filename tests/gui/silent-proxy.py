#!/usr/bin/env python3
"""An HTTP proxy that accepts every connection and answers none, for tests/gui/tiles-startup.gui.

usage: silent-proxy.py --port N --work DIR

What a tile server behind a slow or absent network looks like to the map's fetcher: the connection
opens, the request goes out, and nothing ever comes back, until the fetcher's own timeout gives up
on it. Named in the environment as HTTPS_PROXY and HTTP_PROXY before the application starts, so
every tile the cache does not hold hangs here and nothing leaves the machine.

Listens on 127.0.0.1 at the port given, holds every connection open, and goes when DIR does, which
the harness removes after the run, or after five minutes.
"""
import argparse
import os
import socket
import sys
import time

LIFETIME = 300.0


def serve(listener, work, started):
    listener.settimeout(0.5)
    held = []
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        held.append(client)
    for client in held:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--work", required=True)
    args = parser.parse_args()

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.port))
    listener.listen(64)

    # Into the background: the harness waits for this command, and the proxy must outlive it.
    if os.fork() > 0:
        print(f"silent proxy on {args.port}")
        return 0
    os.setsid()
    devnull = os.open(os.devnull, os.O_RDWR)
    for fd in (0, 1, 2):
        os.dup2(devnull, fd)
    serve(listener, args.work, time.monotonic())
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
