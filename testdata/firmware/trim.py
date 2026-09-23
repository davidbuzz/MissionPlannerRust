#!/usr/bin/env python3
"""Cuts testdata/firmware/manifest.json.gz out of the real ArduPilot firmware manifest.

usage: trim.py <full manifest.json.gz> <out manifest.json.gz>

The full manifest is what Mission Planner downloads (APFirmware.cs:103, ConfigFirmwareManifest.cs:67):
about 97,000 records and 77 MB unpacked. The excerpt keeps a few hundred, chosen so the selection rules
have something to choose between, and keeps them in the manifest's own order - GetRelease groups by
first appearance, so the order is part of the data. Records are copied as they are; nothing is edited.

Kept:
  - CubeOrange (board 140), CubeBlack (board 9), MatekH743 (board 1013): every OFFICIAL, BETA and DEV
    record, every vehicle - so every release has every vehicle the page labels;
  - the other platforms sharing a board id with those, Copter OFFICIAL only: CubeOrange-bdshot and
    CubeOrange-SimOnHardWare (140), fmuv3 and Pixhawk1 (9) - so a board id selects more than one;
  - CubeRedPrimary and CubeRedSecondary, Copter OFFICIAL: the primary/secondary platform names;
  - navio2, Copter OFFICIAL: a Linux board with no board id;
  - f103-GPS, OFFICIAL: an AP_Periph (CAN_PERIPHERAL) board;
  - CubeOrange Copter STABLE-4.7.1: a release type no RELEASE_TYPES value names;
  - apm2-quad Copter, apm2 AntennaTracker and apm2 Rover, OFFICIAL: old versions under OFFICIAL, which
    the newest-version rule must pass over.
"""
import gzip
import json
import sys

ALL_RELEASES = {"CubeOrange", "CubeBlack", "MatekH743"}
COPTER_OFFICIAL = {
    "CubeOrange-bdshot",
    "CubeOrange-SimOnHardWare",
    "fmuv3",
    "Pixhawk1",
    "CubeRedPrimary",
    "CubeRedSecondary",
    "navio2",
}
OLD_OFFICIAL = {("apm2-quad", "Copter"), ("apm2", "ANTENNA_TRACKER"), ("apm2", "GROUND_ROVER")}


def keep(record):
    platform = record.get("platform")
    release = record.get("mav-firmware-version-type")
    mav_type = record.get("mav-type")
    if platform in ALL_RELEASES and release in ("OFFICIAL", "BETA", "DEV"):
        return True
    if platform in COPTER_OFFICIAL and mav_type == "Copter" and release == "OFFICIAL":
        return True
    if platform == "f103-GPS" and release == "OFFICIAL":
        return True
    if platform == "CubeOrange" and mav_type == "Copter" and release == "STABLE-4.7.1":
        return True
    return (platform, mav_type) in OLD_OFFICIAL and release == "OFFICIAL"


def main():
    source, destination = sys.argv[1], sys.argv[2]
    with gzip.open(source, "rt", encoding="utf-8") as f:
        manifest = json.load(f)
    manifest["firmware"] = [r for r in manifest["firmware"] if keep(r)]
    text = json.dumps(manifest, indent=1)
    # mtime=0 so the same input makes the same bytes.
    with open(destination, "wb") as out:
        with gzip.GzipFile(fileobj=out, mode="wb", mtime=0) as gz:
            gz.write(text.encode("utf-8"))
    print(f"{len(manifest['firmware'])} records", file=sys.stderr)


if __name__ == "__main__":
    main()
