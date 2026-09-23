"""Seeds the frame_parse fuzz corpus from a recorded telemetry log.

usage: tools/fuzz-seed.py <flight.tlog> fuzz/corpus/frame_parse

Why this exists: libFuzzer mutating random bytes almost never produces a frame whose CRC passes,
so without seeds it only ever exercises the framing layer. A 46-second run reached 90 edges and
stopped finding anything - the per-message decoders, which are where a length field gets trusted,
were never reached at all. Real frames give the mutator somewhere to start, and a mutated real
frame still has the right shape.

Two frames per message id: enough to give every decoder a starting point without committing a
corpus nobody will ever read.

Only frames whose checksum validates. The scan resynchronises rather than walking a strict record
structure, which means a 0xFD byte inside a payload looks exactly like the start of a frame - a
1.5 MB log yielded 267 "message ids" that way, most of them noise. The checksum is what tells the
difference, and CRC_EXTRA comes from the same generated source the decoders use, so a seed for a
message id is a seed the decoder for that id will recognise.
"""
import sys, os, hashlib

MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD

GENERATED = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'crates/mp-mavlink-dialects/src/generated/all.rs',
)


def crc_extras():
    """message id -> CRC_EXTRA, read out of the generated dialect."""
    import re

    text = open(GENERATED).read()
    pairs = re.findall(
        r'const ID: u32 = (\d+);\s*\n\s*const NAME[^\n]*\n\s*const CRC_EXTRA: u8 = (\d+);',
        text,
    )
    return {int(msgid): int(extra) for msgid, extra in pairs}


def crc16_mcrf4xx(data, seed=0xFFFF):
    """CRC-16/MCRF4XX, the one MAVLink uses."""
    crc = seed
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def checksum_valid(frame, magic, msgid, extras):
    """Whether a candidate frame's checksum agrees with its contents."""
    extra = extras.get(msgid)
    if extra is None:
        return False
    header = 6 if magic == MAGIC_V1 else 10
    length = frame[1]
    end = header + length
    if end + 2 > len(frame):
        return False
    crc = crc16_mcrf4xx(frame[1:end])
    crc = crc16_mcrf4xx(bytes([extra]), crc)
    return crc == (frame[end] | (frame[end + 1] << 8))


def frames(data):
    """Yields every MAVLink frame in the buffer, resynchronising as it goes.

    Resynchronising rather than walking a strict record structure. A tlog is a timestamp followed
    by a frame, repeated - except where it is not: ArduPilot prints boot text before MAVLink
    starts, a recording can begin mid-frame, and a log copied off a failing card has holes. The
    committed fixtures begin with "\n\nInit ArduCopter V4.8.0", and a strict walker gives up on
    the first byte and reports a file with no frames in it.

    No checksum validation: a seed does not have to be valid, only shaped right. The fuzzer is
    going to mutate it anyway.
    """
    i = 0
    while i < len(data):
        magic = data[i]
        if magic == MAGIC_V2 and i + 10 <= len(data):
            length = data[i + 1]
            incompat = data[i + 2]
            msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
            total = 12 + length + (13 if incompat & 0x01 else 0)
        elif magic == MAGIC_V1 and i + 6 <= len(data):
            length = data[i + 1]
            msgid = data[i + 5]
            total = 8 + length
        else:
            i += 1
            continue
        if i + total <= len(data):
            yield magic, msgid, data[i:i + total]
            i += total
        else:
            i += 1


src, dest = sys.argv[1], sys.argv[2]
os.makedirs(dest, exist_ok=True)
data = open(src, 'rb').read()
extras = crc_extras()
seen = {}
rejected = 0
for magic, msgid, frame in frames(data):
    if not checksum_valid(frame, magic, msgid, extras):
        # A 0xFD inside a payload looks exactly like a frame start. The checksum is what says
        # otherwise, and a seed that is not a frame teaches the mutator nothing.
        rejected += 1
        continue
    # Two per message id: enough to give every decoder a starting point without committing a
    # corpus nobody will ever read.
    key = (magic, msgid)
    if seen.get(key, 0) < 2:
        seen[key] = seen.get(key, 0) + 1
        name = hashlib.sha1(frame).hexdigest()
        open(os.path.join(dest, name), 'wb').write(frame)
print(
    f"{len(seen)} message ids, {sum(seen.values())} frames -> {dest}"
    f"  ({rejected} candidates rejected by checksum)"
)
