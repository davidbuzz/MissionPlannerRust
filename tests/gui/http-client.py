"""A client of the planner's built-in HTTP server, for the GUI script http-server.gui.

usage: http-client.py --port PORT --work DIR

Waits up to twenty seconds for the port to answer, then asks, as a browser or Google Earth would:
GET /, /network.kml, /location.kml, /wps.kml, /mavlink/, /command_long (a 404), and finally
GET /guided?lat=-35.363&lng=149.165&alt=30 - a loopback client, so the planner sends the guided
waypoint. What each answered is written to DIR/http-client.txt, a line a request, for the record.

Returns at once, leaving itself asking in the background (gui_background), as the runner waits
for a setup command to finish before the planner is launched.
"""
import argparse
import http.client
import os
import socket
import sys
import time

import gui_background

PATHS = [
    "/",
    "/network.kml",
    "/location.kml",
    "/wps.kml",
    "/mavlink/",
    "/command_long",
    "/guided?lat=-35.363&lng=149.165&alt=30",
]


def wait_for(port, deadline):
    """Until something listens on the port, or the deadline."""
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.25)
    return False


def ask(port, work):
    deadline = time.time() + 20.0
    lines = []
    if not wait_for(port, deadline):
        lines.append("no server")
    else:
        # The planner publishes its first snapshot a frame after the listener is up.
        time.sleep(1.0)
        for path in PATHS:
            try:
                connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5.0)
                connection.request("GET", path, headers={"Host": "127.0.0.1"})
                answer = connection.getresponse()
                body = answer.read(200_000)
                lines.append(f"{path} {answer.status} {len(body)}")
                connection.close()
            except Exception as err:  # noqa: BLE001 - the record says what went wrong
                lines.append(f"{path} error {err}")
            time.sleep(0.2)
    with open(os.path.join(work, "http-client.txt"), "w", encoding="utf-8") as out:
        out.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--work", required=True)
    args = parser.parse_args()
    if gui_background.child_listener() is not None:
        ask(args.port, args.work)
        return
    # No socket of its own to hand over: the forked child asks and goes.
    if hasattr(os, "fork"):
        if os.fork() > 0:
            print(f"http-client asking 127.0.0.1:{args.port} in the background")
            return
        os.setsid()
        devnull = os.open(os.devnull, os.O_RDWR)
        for fd in (0, 1, 2):
            os.dup2(devnull, fd)
        ask(args.port, args.work)
        os._exit(0)
    ask(args.port, args.work)


if __name__ == "__main__":
    main()
