#!/usr/bin/env python3
"""A release's update manifest, update.json: written from the files the release workflow has just
packaged, and checked against the files as the world fetches them (updates.md, sections 4.2 and 5).

    tools/release/manifest.py write --channel stable --tag v0_0_2 --commit SHA --built ISO
                                    --repository https://github.com/OWNER/REPO --dir assets
                                    [--version 0.0.2] [--out assets/update.json]
    tools/release/manifest.py verify <update.json or its address>

`write` takes the four downloads of a release from --dir, one of each or it stops - the Linux
archive, the Debian package, the Windows zip and the macOS archive, by the names
.github/workflows/release.yml gives them - and writes their names, addresses, sizes and SHA-256s
under the platform each is for. The version is --version, or the tag's own numbers (v0_0_2 is
0.0.2). The planner reads the result with crates/mp-update/src/manifest.rs, and
crates/mp-update/tests/manifest_tool.rs holds the two together.

`verify` fetches the manifest and then every download it names, and exits 1 unless each is the
size and the hash the manifest says: the release workflow's last step, on the published release.
"""

import argparse
import hashlib
import json
import pathlib
import re
import sys
import urllib.request

SCHEMA = 1
# The platform each download is for, by the end of its name.
PLATFORMS = [
    ("linux-x86_64", "_linux-x86_64.tar.gz"),
    ("linux-x86_64-deb", "_amd64.deb"),
    ("windows-x86_64", "_windows-x86_64.zip"),
    ("macos-universal", "_macos-universal.tar.gz"),
]


def version_of(tag):
    """The numbers of a tag, joined by points: v0_0_2 and v0.0.2 are both 0.0.2."""
    numbers = re.fullmatch(r"[vV]?(\d+(?:[._]\d+)*)", tag)
    if not numbers:
        sys.exit(f"manifest.py: {tag!r} is not a release tag, and no --version was given")
    return numbers.group(1).replace("_", ".")


def sha256_of(stream):
    digest, size = hashlib.sha256(), 0
    while chunk := stream.read(1 << 20):
        digest.update(chunk)
        size += len(chunk)
    return digest.hexdigest(), size


def write(args):
    directory = pathlib.Path(args.dir)
    repository = args.repository.rstrip("/")
    assets = {}
    for platform, ending in PLATFORMS:
        found = sorted(p for p in directory.iterdir() if p.is_file() and p.name.endswith(ending))
        if len(found) != 1:
            names = ", ".join(p.name for p in found) or "none"
            sys.exit(f"manifest.py: {directory} holds {len(found)} downloads for {platform} ({names}); one is wanted")
        with found[0].open("rb") as stream:
            digest, size = sha256_of(stream)
        assets[platform] = {
            "name": found[0].name,
            "url": f"{repository}/releases/download/{args.tag}/{found[0].name}",
            "size": size,
            "sha256": digest,
        }
    manifest = {
        "schema": SCHEMA,
        "channel": args.channel,
        "version": args.version or version_of(args.tag),
        "tag": args.tag,
        "commit": args.commit,
        "built": args.built,
        "notes_url": f"{repository}/releases/tag/{args.tag}",
        "assets": assets,
    }
    text = json.dumps(manifest, indent=2) + "\n"
    out = pathlib.Path(args.out) if args.out else directory / "update.json"
    out.write_text(text)
    print(text, end="")


def open_address(address):
    """A file by its path, or anything urllib fetches - GitHub's redirects followed."""
    if re.match(r"^[a-z][a-z0-9+.-]*://", address):
        request = urllib.request.Request(address, headers={"User-Agent": "missionplanner-rust-release"})
        return urllib.request.urlopen(request, timeout=120)
    return open(address, "rb")


def verify(args):
    with open_address(args.manifest) as stream:
        manifest = json.load(stream)
    if manifest.get("schema") != SCHEMA:
        sys.exit(f"manifest.py: schema {manifest.get('schema')!r}, not {SCHEMA}")
    print(f"{manifest['channel']} {manifest['version']} ({manifest['tag']}, {manifest['commit'][:12]}, built {manifest['built']})")
    wrong = 0
    for platform, asset in sorted(manifest["assets"].items()):
        try:
            with open_address(asset["url"]) as stream:
                digest, size = sha256_of(stream)
        except OSError as why:
            print(f"  FAIL {platform}: {asset['url']}: {why}")
            wrong += 1
            continue
        if (size, digest) == (asset["size"], asset["sha256"]):
            print(f"  ok   {platform}: {asset['name']}, {size} bytes")
        else:
            print(f"  FAIL {platform}: {asset['name']} is {size} bytes, sha256 {digest}; "
                  f"the manifest says {asset['size']} bytes, {asset['sha256']}")
            wrong += 1
    if wrong or len(manifest["assets"]) != len(PLATFORMS):
        sys.exit(f"manifest.py: {wrong} download(s) are not the manifest's, of {len(manifest['assets'])} named")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    commands = parser.add_subparsers(dest="command", required=True)
    w = commands.add_parser("write")
    w.add_argument("--channel", required=True, choices=["stable", "beta"])
    w.add_argument("--tag", required=True)
    w.add_argument("--version")
    w.add_argument("--commit", required=True)
    w.add_argument("--built", required=True)
    w.add_argument("--repository", required=True)
    w.add_argument("--dir", required=True)
    w.add_argument("--out")
    w.set_defaults(run=write)
    v = commands.add_parser("verify")
    v.add_argument("manifest")
    v.set_defaults(run=verify)
    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
