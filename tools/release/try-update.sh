#!/usr/bin/env bash
# Releases of this tree and a channel serving them on this machine - for trying the update by
# hand before any release with a manifest is on GitHub. The owner's local working copy is
# target/debug/planner, whose own buttons update it (updates.md, section 4.3); what this adds is
# something for a planner to update *to* and a planner that is a release, which the check at
# start needs. tools/release/self-update-test.py is the same thing as a test.
#
#   tools/release/try-update.sh <folder>      a folder of your choosing; it is emptied first
#
# The folder is asked for, never assumed: the first run of this put ~/mp-update-demo in the
# owner's home unasked (2026-11-18).
#
# A planner from `cargo build` is a development build and is never updated, so this builds three
# that are told they are releases, as the release workflow tells its own (MP_RELEASE_*): 0.0.1 of
# the stable channel, 0.0.2 of the stable channel, and a beta built after 0.0.2. They are debug
# builds, stripped. It then lays out, in the folder:
#
#   install/      release 0.0.1, unpacked as an archive would be: the planner to run
#   channel/      what GitHub would serve: update.json (0.0.2), beta/update.json, their archives
#   config.xml    settings naming that channel in place of this repository's (UpdateManifest,
#                 BetaUpdateManifest) - a file of the demo's own, so the user's settings are not
#                 touched
#   run.sh        serves channel/ on 127.0.0.1:5797 and starts install/planner with config.xml
#   reset.sh      puts install/ back to release 0.0.1
#   stop.sh       stops serving channel/
#
# Builds with cargo, three times and once more to leave target/debug/planner the development
# build it was: run it when no other build is going.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DEMO=${1:?usage: tools/release/try-update.sh <folder> - a folder to build the demo in, emptied first}
case "$DEMO" in
    /|"$HOME"|"$HOME/") echo "not $DEMO" >&2; exit 2 ;;
esac
PORT=5797
JOBS=${JOBS:-4}
cd "$ROOT"

rm -rf "$DEMO"
mkdir -p "$DEMO/builds" "$DEMO/channel/files" "$DEMO/channel/beta"

# The commits the three say they are: this tree's last three, so each is another's.
old=$(git rev-parse HEAD~2) stable=$(git rev-parse HEAD~1) beta=$(git rev-parse HEAD)
build_as() { # name version channel commit built
    echo "== building $1: $2, $3"
    MP_RELEASE_VERSION=$2 MP_RELEASE_CHANNEL=$3 MP_RELEASE_COMMIT=$4 MP_RELEASE_BUILT=$5 \
        cargo build -p mp-gui --bin planner -j"$JOBS" 2>&1 | grep -v '^ *Compiling' | tail -2
    mkdir -p "$DEMO/builds/$1"
    strip -o "$DEMO/builds/$1/planner" target/debug/planner
    "$DEMO/builds/$1/planner" --version
}
build_as 0.0.1 0.0.1 stable "$old" 2026-11-01T00:00:00Z
build_as 0.0.2 0.0.2 stable "$stable" 2026-11-10T00:00:00Z
build_as beta 0.0.2 beta "$beta" 2026-11-17T00:00:00Z
echo "== the development build back in target/debug"
cargo build -p mp-gui --bin planner -j"$JOBS" 2>&1 | grep -v '^ *Compiling' | tail -1
target/debug/planner --version

# The channel: each release's archive as release.yml packs one, and its manifest.
python3 - "$DEMO" "$PORT" "$stable" "$beta" <<'PY'
import hashlib, io, json, pathlib, sys, tarfile
demo, port, stable, beta = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
root = pathlib.Path.cwd()
base = f"http://127.0.0.1:{port}"

def release(build, label, channel, tag, commit, built, manifest):
    name = f"missionplanner-rust_{label}_linux-x86_64.tar.gz"
    folder = name[: -len(".tar.gz")]
    target = demo / "channel/files" / name
    with tarfile.open(target, "w:gz", compresslevel=1) as tar:
        tar.add(demo / "builds" / build / "planner", f"{folder}/planner")
        for licence in ["LICENSE", "NOTICE", "THIRD_PARTY_LICENSES", "README.md"]:
            tar.add(root / licence, f"{folder}/{licence}")
        note = f"MissionPlannerRust {label}, the {channel} channel's, from tools/release/try-update.sh\n".encode()
        info = tarfile.TarInfo(f"{folder}/BUILD-INFO.txt")
        info.size, info.mode = len(note), 0o644
        tar.addfile(info, io.BytesIO(note))
    body = target.read_bytes()
    page = demo / "channel/releases/tag" / tag
    page.mkdir(parents=True)
    (page / "index.html").write_text(f"<h1>MissionPlannerRust {tag}</h1><p>The demo channel's notes for {label}.</p>\n")
    (demo / "channel" / manifest).write_text(json.dumps({
        "schema": 1, "channel": channel, "version": "0.0.2", "tag": tag, "commit": commit,
        "built": built, "notes_url": f"{base}/releases/tag/{tag}/",
        "assets": {"linux-x86_64": {
            "name": name, "url": f"{base}/files/{name}", "size": len(body),
            "sha256": hashlib.sha256(body).hexdigest()}},
    }, indent=2) + "\n")
    print(f"{manifest}: {name}, {len(body) >> 20} MB")

release("0.0.2", "0_0_2", "stable", "v0_0_2", stable, "2026-11-10T00:00:00Z", "update.json")
release("beta", f"beta-{beta[:7]}", "beta", "beta", beta, "2026-11-17T00:00:00Z", "beta/update.json")
PY

cat > "$DEMO/config.xml" <<XML
<?xml version="1.0" encoding="utf-8"?><Config><UpdateManifest>http://127.0.0.1:$PORT/update.json</UpdateManifest><BetaUpdateManifest>http://127.0.0.1:$PORT/beta/update.json</BetaUpdateManifest></Config>
XML

cat > "$DEMO/reset.sh" <<'SH'
#!/usr/bin/env bash
# install/ back to release 0.0.1, as an archive unpacked: nothing of an update left.
here=$(cd "$(dirname "$0")" && pwd)
rm -rf "$here/install"
mkdir -p "$here/install"
cp "$here/builds/0.0.1/planner" "$here/install/planner"
printf 'the licence of release 0.0.1\n' > "$here/install/LICENSE"
"$here/install/planner" --version
SH

cat > "$DEMO/run.sh" <<SH
#!/usr/bin/env bash
# The channel served on 127.0.0.1:$PORT, and install/planner started with the demo's settings.
# The planner an update starts in its place inherits the settings, and the channel stays up:
# stop.sh ends it.
here=\$(cd "\$(dirname "\$0")" && pwd)
if ! (exec 3<>/dev/tcp/127.0.0.1/$PORT) 2>/dev/null; then
    nohup python3 -m http.server $PORT --bind 127.0.0.1 --directory "\$here/channel" \\
        > "\$here/channel.log" 2>&1 < /dev/null &
    echo \$! > "\$here/channel.pid"
    sleep 0.5
fi
"\$here/install/planner" --version
MP_CONFIG_XML="\$here/config.xml" exec "\$here/install/planner" "\$@"
SH

cat > "$DEMO/stop.sh" <<'SH'
#!/usr/bin/env bash
here=$(cd "$(dirname "$0")" && pwd)
[ -f "$here/channel.pid" ] && kill "$(cat "$here/channel.pid")" 2>/dev/null
rm -f "$here/channel.pid"
SH
chmod +x "$DEMO/run.sh" "$DEMO/reset.sh" "$DEMO/stop.sh"
"$DEMO/reset.sh"
echo "== ready: $DEMO/run.sh"
