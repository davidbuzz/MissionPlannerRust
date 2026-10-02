"""An update channel, for the GUI script of the HELP screen.

usage: update-server.py --work DIR [--port 5793]

Serves what `Utilities/Update.cs` asks a channel for, over HTTP on 127.0.0.1:PORT:

  /version.txt            1.0.0.0 - the stable channel, at the program's own version
  /checksums.txt          one file, ./update-probe.txt, and its MD5
  /files//update-probe.txt  that file (the C# asks for a root file with two slashes)
  /beta/version.txt       2.0.0.0 - the beta channel, ahead
  /beta/checksums.txt     the same one file
  /beta/Beta.zip          a zip holding update-probe.txt, as the beta channel is read
  /ChangeLog.txt, /beta/ChangeLog.txt  a line each
  POST anything        a crash report's sink: the body kept in DIR as post-<ms>.txt, "ok" back

Returns once listening, leaving itself serving in the background; goes when DIR does, which the
harness removes after the run, or after five minutes.
"""
import argparse
import hashlib
import http.server
import io
import os
import socket
import sys
import threading
import time
import zipfile

import gui_background

LIFETIME = 300.0
PROBE = b"probe 2.0.0.0\n"
# Where a POST's body is kept: the work directory, set by main.
POSTED_DIR = "."


def pages():
    """Every path the channel answers, with its body."""
    checksums = f"{hashlib.md5(PROBE).hexdigest()}  MissionPlanner/update-probe.txt\n".encode()
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("update-probe.txt", PROBE)
    return {
        "/version.txt": b"1.0.0.0\n",
        "/checksums.txt": checksums,
        "/files//update-probe.txt": PROBE,
        "/files/update-probe.txt": PROBE,
        "/beta/version.txt": b"2.0.0.0\n",
        "/beta/checksums.txt": checksums,
        "/beta/Beta.zip": buffer.getvalue(),
        "/ChangeLog.txt": b"1.0.0.0 the first\n",
        "/beta/ChangeLog.txt": b"2.0.0.0 the beta\n",
    }


class Handler(http.server.BaseHTTPRequestHandler):
    PAGES = pages()

    def do_GET(self):  # noqa: N802 - http.server's name
        path = self.path.split("?", 1)[0]
        body = self.PAGES.get(path)
        if body is None:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):  # noqa: N802
        """A crash report's sink, as the C#'s mail.php: the body read, 200 and "ok"."""
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length)
        with open(os.path.join(POSTED_DIR, f"post-{int(time.time() * 1000)}.txt"), "wb") as f:
            f.write(body)
        self.send_response(200)
        self.send_header("Content-Length", "2")
        self.end_headers()
        self.wfile.write(b"ok")

    def do_HEAD(self):  # noqa: N802
        path = self.path.split("?", 1)[0]
        self.send_response(200 if path in self.PAGES else 404)
        self.end_headers()

    def log_message(self, *_args):
        pass


def run(listener, work):
    """Serve on the socket already bound until the work directory goes or the lifetime ends."""
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler, bind_and_activate=False)
    server.socket = listener
    server.server_address = listener.getsockname()

    def watch():
        end = time.monotonic() + LIFETIME
        while time.monotonic() < end and os.path.isdir(work):
            time.sleep(0.5)
        server.shutdown()

    threading.Thread(target=watch, daemon=True).start()
    server.serve_forever(poll_interval=0.2)


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5793)
    args = parser.parse_args()
    global POSTED_DIR
    POSTED_DIR = args.work
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
                print(f"port {args.port} is busy", file=sys.stderr)
                return 3
            time.sleep(0.2)
    listener.listen(5)
    gui_background.serve_in_background(
        listener, f"update channel on {args.port}", lambda held: run(held, args.work)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
