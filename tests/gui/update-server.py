"""An update channel, for the GUI scripts of the HELP screen.

usage: update-server.py --work DIR [--port 5793]

Serves what the planner's own update asks a channel for (updates.md; crates/mp-update), over HTTP
on 127.0.0.1:PORT, as GitHub's releases do for the real one:

  /update.json            the stable channel's manifest: release 0.0.2
  /beta/update.json       the beta channel's: a later build after 0.0.2
  /files/<name>           the downloads the manifests name - a .tar.gz holding a planner that
                          answers --version as release 0.0.2 (a shell script, so Linux and macOS;
                          the manifests name none for Windows) and its licence, and a file standing
                          for the Debian package
  POST anything           a crash report's sink: the body kept in DIR as post-<ms>.txt, "ok" back

Returns once listening, leaving itself serving in the background; goes when DIR does, which the
harness removes after the run, or after five minutes.
"""
import argparse
import hashlib
import http.server
import io
import json
import os
import socket
import sys
import tarfile
import threading
import time

import gui_background

LIFETIME = 300.0
# The release on the stable channel, and the build on the beta channel after it.
STABLE = "42992d086a789143682f2997adc737545e3d8c2e"
BETA = "9f8e7d6c5b4a39281706f5e4d3c2b1a098765432"
PLANNER = b"#!/bin/sh\necho 'planner 0.0.2 (stable, commit 42992d086a78, built 2026-11-20T03:10:00Z)'\n"
BETA_PLANNER = b"#!/bin/sh\necho 'planner 0.0.2 (beta, commit 9f8e7d6c5b4a, built 2026-11-25T02:00:00Z)'\n"
DEB = b"!<arch>\ndebian-binary, standing for the package\n"
# Where a POST's body is kept: the work directory, set by main.
POSTED_DIR = "."


def archive(folder, planner):
    """A release's .tar.gz: the planner and its licence in the release's own folder."""
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
        for name, body, mode in [("planner", planner, 0o755), ("LICENSE", b"the new licence\n", 0o644)]:
            entry = tarfile.TarInfo(f"{folder}/{name}")
            entry.size = len(body)
            entry.mode = mode
            tar.addfile(entry, io.BytesIO(body))
    return buffer.getvalue()


def manifest(port, channel, tag, commit, built, files):
    """A manifest as tools/release/manifest.py writes one, its downloads on this server."""
    base = f"http://127.0.0.1:{port}"
    return json.dumps({
        "schema": 1,
        "channel": channel,
        "version": "0.0.2",
        "tag": tag,
        "commit": commit,
        "built": built,
        "notes_url": f"{base}/releases/tag/{tag}",
        "assets": {
            platform: {
                "name": name,
                "url": f"{base}/files/{name}",
                "size": len(body),
                "sha256": hashlib.sha256(body).hexdigest(),
            }
            for platform, name, body in files
        },
    }).encode()


def pages(port):
    """Every path the channel answers, with its body."""
    stable = archive("missionplanner-rust_0_0_2_linux-x86_64", PLANNER)
    beta = archive("missionplanner-rust_beta-9f8e7d6_linux-x86_64", BETA_PLANNER)
    stable_files = [
        ("linux-x86_64", "missionplanner-rust_0_0_2_linux-x86_64.tar.gz", stable),
        ("macos-universal", "missionplanner-rust_0_0_2_macos-universal.tar.gz", stable),
        ("linux-x86_64-deb", "missionplanner-rust_0.0.2_amd64.deb", DEB),
    ]
    beta_files = [
        ("linux-x86_64", "missionplanner-rust_beta-9f8e7d6_linux-x86_64.tar.gz", beta),
        ("macos-universal", "missionplanner-rust_beta-9f8e7d6_macos-universal.tar.gz", beta),
        ("linux-x86_64-deb", "missionplanner-rust_0.0.2+beta.9f8e7d6_amd64.deb", DEB),
    ]
    answers = {
        "/update.json": manifest(port, "stable", "v0_0_2", STABLE, "2026-11-20T03:10:00Z", stable_files),
        "/beta/update.json": manifest(port, "beta", "beta", BETA, "2026-11-25T02:00:00Z", beta_files),
    }
    for _, name, body in stable_files + beta_files:
        answers[f"/files/{name}"] = body
    return answers


class Handler(http.server.BaseHTTPRequestHandler):
    PAGES = {}

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
    Handler.PAGES = pages(args.port)
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
