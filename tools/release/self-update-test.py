#!/usr/bin/env python3
"""A real planner updating itself, end to end, on the machine it runs on: Linux, macOS, Windows.

    tools/release/self-update-test.py <a planner built as a release>

The planner must be a release build - one the workflow made, or one built by hand with
MP_RELEASE_VERSION, MP_RELEASE_CHANNEL and MP_RELEASE_COMMIT set, so that `--version` names a
release (crates/mp-update/src/identity.rs). No window is opened: the update is the planner's own
`/update` command line, which is HELP's "Check for Updates" and "Update Now" without the boxes
(crates/mp-gui/src/help.rs, update_from_command_line), over the same crates/mp-update.

What it does, in a scratch folder of its own:

  * installs a copy of the planner, with a licence file, as an unpacked archive would be;
  * serves a release of that same planner from this machine - update.json and the platform's
    archive, a .tar.gz, or a .zip on Windows - as GitHub serves the real ones;
  * runs the installed planner told it is an older release (MP_UPDATE_IDENTITY, the harness's
    door), so that it finds the release, fetches it, checks it, unpacks it, runs the new
    planner's --version, and moves the new files over its own - the running program's file among
    them, which is the step updates.md's section 9 is about;
  * holds the result to what should be there: the new files in place, the old ones beside them
    as .old and listed, nothing left of the work folder, and the planner still starting;
  * runs it again as itself: nothing to update, and the .old files of the last update deleted;
  * and before all that, serves a download that is not the manifest's: refused, nothing changed.

Each line says what was looked at; the exit code is the number that were not as expected.
"""

import hashlib
import http.server
import io
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import threading
import zipfile

WINDOWS = os.name == "nt"
EXE = "planner.exe" if WINDOWS else "planner"
PLATFORM = "windows-x86_64" if WINDOWS else "macos-universal" if sys.platform == "darwin" else "linux-x86_64"
failures = 0


def check(what, holds, detail=""):
    global failures
    if not holds:
        failures += 1
    print(f"  {'ok  ' if holds else 'FAIL'} {what}" + (f": {detail}" if detail and not holds else ""), flush=True)


def archive(folder, files):
    """The platform's archive of a release: `files` in the release's own folder."""
    buffer = io.BytesIO()
    if WINDOWS:
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as z:
            for name, body in files:
                z.writestr(f"{folder}/{name}", body)
        return buffer.getvalue(), ".zip"
    with tarfile.open(fileobj=buffer, mode="w:gz", compresslevel=1) as tar:
        for name, body in files:
            entry = tarfile.TarInfo(f"{folder}/{name}")
            entry.size = len(body)
            entry.mode = 0o755 if name == EXE else 0o644
            tar.addfile(entry, io.BytesIO(body))
    return buffer.getvalue(), ".tar.gz"


class Channel(http.server.BaseHTTPRequestHandler):
    pages = {}

    def do_GET(self):  # noqa: N802 - http.server's name
        body = self.pages.get(self.path)
        if body is None:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        pass


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    source = pathlib.Path(sys.argv[1]).resolve()
    said = subprocess.run([str(source), "--version"], capture_output=True, text=True, timeout=120).stdout.strip()
    built_as = re.fullmatch(r"planner (\S+) \((stable|beta), commit ([0-9a-f]+)(?:, built (\S+))?\)", said)
    if not built_as:
        sys.exit(f"self-update-test: {source} is not a release build: it says {said!r}.\n"
                 "Build one with MP_RELEASE_VERSION, MP_RELEASE_CHANNEL and MP_RELEASE_COMMIT set.")
    version, channel, commit, built = built_as.groups()
    built = built or "2026-01-01T00:00:00Z"
    print(f"{said}\non {sys.platform} ({PLATFORM}), {source.stat().st_size >> 20} MB", flush=True)

    work = pathlib.Path(tempfile.mkdtemp(prefix="mp-self-update-"))
    try:
        run(work, source, version, channel, commit, built, said)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"\nself-update-test: {failures} line(s) other than expected")
    sys.exit(failures)


def run(work, source, version, channel, commit, built, said):
    install = work / "install"
    install.mkdir()
    planner = install / EXE
    shutil.copy2(source, planner)
    (install / "LICENSE").write_bytes(b"the old licence\n")
    planner_bytes = planner.read_bytes()

    body, ending = archive(f"missionplanner-rust_test_{PLATFORM}", [
        (EXE, planner_bytes),
        ("LICENSE", b"the new licence\n"),
        ("sitl-wasm/new-in-this-release.txt", b"a file the old install had not got\n"),
    ])
    name = f"missionplanner-rust_test_{PLATFORM}{ending}"

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Channel)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def manifest(sha256):
        return json.dumps({
            "schema": 1, "channel": channel, "version": version, "tag": "test", "commit": commit,
            "built": built, "notes_url": f"{base}/releases/tag/test",
            "assets": {PLATFORM: {"name": name, "url": f"{base}/files/{name}", "size": len(body), "sha256": sha256}},
        }).encode()

    Channel.pages = {f"/files/{name}": body}
    config = work / "config.xml"
    key = "UpdateManifest" if channel == "stable" else "BetaUpdateManifest"
    config.write_text(f'<?xml version="1.0" encoding="utf-8"?><Config><{key}>{base}/update.json</{key}></Config>')
    environment = {**os.environ, "MP_CONFIG_XML": str(config), "MP_UPDATE_DIR": str(install)}
    # An older release than any, of the same channel, built before this one.
    older = {**environment, "MP_UPDATE_IDENTITY": f"0,{channel},{'0' * 40},2020-01-01T00:00:00Z"}
    switch = "/update" if channel == "stable" else "/updatebeta"

    def update(env):
        done = subprocess.run([str(planner), switch], capture_output=True, text=True, env=env, timeout=600)
        return done.returncode, done.stdout, done.stderr

    def there(*names):
        return sorted(str(p.relative_to(install)).replace("\\", "/") for p in install.rglob("*") if p.is_file())

    print("\nA download that is not the manifest's")
    Channel.pages["/update.json"] = manifest(hashlib.sha256(b"something else").hexdigest())
    code, out, err = update(older)
    check("the update is refused", code == 1, f"exit {code}\n{out}{err}")
    check("and says the download does not match", "does not match hash" in err, err)
    check("the install is as it was", there() == ["LICENSE", EXE] and (install / "LICENSE").read_bytes() == b"the old licence\n", str(there()))

    print("\nThe planner updates itself")
    Channel.pages["/update.json"] = manifest(hashlib.sha256(body).hexdigest())
    code, out, err = update(older)
    check("the update succeeds", code == 0, f"exit {code}\n{out}{err}")
    check("it found the release", "Update Found: " in out, out)
    check("it installed three files", "Update installed: 3 files" in out, out)
    print("    " + out.strip().replace("\n", "\n    "))
    check("the new licence is in place", (install / "LICENSE").read_bytes() == b"the new licence\n")
    check("the old licence is beside it", (install / "LICENSE.old").is_file() and (install / "LICENSE.old").read_bytes() == b"the old licence\n")
    check("the planner that ran is beside the new one as .old", (install / (EXE + ".old")).is_file())
    check("the new planner is in place, whole", planner.is_file() and planner.read_bytes() == planner_bytes)
    check("the file new in this release is there", (install / "sitl-wasm/new-in-this-release.txt").is_file())
    check("the work folder is gone", not (install / ".update").exists())
    listed = sorted((install / ".update-old.txt").read_text().split()) if (install / ".update-old.txt").is_file() else []
    check("the old files are listed for the next start", listed == sorted(["LICENSE.old", EXE + ".old"]), str(listed))
    again = subprocess.run([str(planner), "--version"], capture_output=True, text=True, timeout=120).stdout.strip()
    check("the installed planner starts and is the release", again == said, again)

    print("\nThe next start")
    code, out, err = update(environment)
    check("there is nothing to update", code == 0 and "No update available." in out, f"exit {code}\n{out}{err}")
    check("the old files are deleted", there() == ["LICENSE", EXE, "sitl-wasm/new-in-this-release.txt"], str(there()))
    server.shutdown()


if __name__ == "__main__":
    main()
