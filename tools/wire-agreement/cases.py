#!/usr/bin/env python3
# Copyright (C) 2026 David "Buzz" Bussenschutt
#
# This file is part of MissionPlannerRust; see LICENSE (GPL-3.0-only).
#
# SPDX-License-Identifier: GPL-3.0-only
"""The wire-agreement cases: a payload per message and pattern, from the XML through pymavlink's
own parser, so neither Rust binding writes the bytes both are judged by.

One line per case: `id name pattern min_len len crc_extra target_offset payload_hex`, the
payload its full length (`len`) and untrimmed; `target_offset` is -1 for a message that names no
target system. Patterns 0 and 1 are the values tests/test_mavgen_rust.py gives every field in
tridge's pull request (zero or an enum's first entry; a per-offset spread with an enum's last
entry); patterns 2 and up are seeded random bytes, an enum field still one of its entries (or,
for a bitmask, a subset of its flags) since a typed binding may refuse any other value.

Usage: PYTHONPATH=<dir holding pymavlink> cases.py <all.xml> <random patterns>
"""
import contextlib
import random
import struct
import sys
from pathlib import Path

from pymavlink.generator import mavparse

PACK = {
    'char': 'B', 'int8_t': 'b', 'uint8_t': 'B', 'int16_t': 'h', 'uint16_t': 'H',
    'int32_t': 'i', 'uint32_t': 'I', 'int64_t': 'q', 'uint64_t': 'Q',
    'float': 'f', 'double': 'd',
}


def dialects(path):
    """The XML and everything it includes, as the pull request's test loads them."""
    result, seen, pending = [], set(), [Path(path)]
    while pending:
        path = pending.pop(0).resolve()
        if path in seen:
            continue
        seen.add(path)
        xml = mavparse.MAVXML(str(path), '2.0')
        result.append(xml)
        pending.extend(path.parent / include for include in xml.include)
    if mavparse.check_duplicates(result):
        sys.exit('duplicate message ids in ' + str(path))
    return result


def entries_of(field, enums):
    """The field's enum's entries that fit its width - the pull request's filter."""
    entries = [e for e in enums[field.enum].entry if not e.end_marker]
    return [e for e in entries if e.value < 2 ** (field.type_length * 8)]


def values(field, enums, pattern):
    """The pull request's `values()` (tests/test_mavgen_rust.py) for patterns 0 and 1."""
    if field.enum:
        entries = entries_of(field, enums)
        value = entries[0 if pattern == 0 else -1].value
    elif field.const_value is not None:
        value = field.const_value
    elif pattern == 0:
        value = 0
    elif field.type in ('float', 'double'):
        value = -123.25 - field.wire_offset
    elif field.type == 'char':
        value = 200
    elif field.type.startswith('int'):
        value = -(2 ** (field.type_length * 8 - 2)) + field.wire_offset % (2 ** (field.type_length * 8 - 2))
    else:
        value = (2 ** (field.type_length * 8) - 1) - field.wire_offset
    result = [value] * (field.array_length or 1)
    if pattern and field.array_length and field.enum:
        result = [entries[i % len(entries)].value for i in range(field.array_length)]
    if pattern and field.array_length and not field.enum:
        for i in range(field.array_length):
            if field.type in ('float', 'double'):
                result[i] = value - i * 0.25
            elif field.type.startswith('int'):
                bits = field.type_length * 8
                result[i] = ((value + i + 2 ** (bits - 1)) % 2 ** bits) - 2 ** (bits - 1)
            else:
                result[i] = (value - i) % 2 ** (field.type_length * 8)
    return result


def random_bytes(field, enums, rng):
    """A field's bytes, random but for an enum (one of its entries) or a constant."""
    count = field.array_length or 1
    fmt = '<' + PACK[field.type]
    if field.enum:
        entries = entries_of(field, enums)
        if enums[field.enum].bitmask:
            picked = [sum(e.value for e in entries if rng.random() < 0.5) for _ in range(count)]
        else:
            picked = [rng.choice(entries).value for _ in range(count)]
        return b''.join(struct.pack(fmt, v) for v in picked)
    if field.const_value is not None:
        return b''.join(struct.pack(fmt, int(field.const_value)) for _ in range(count))
    return bytes(rng.randrange(256) for _ in range(field.type_length * count))


def payload(message, enums, pattern):
    out = bytearray(message.wire_length)
    rng = random.Random(pattern * 1_000_003 + message.id)
    for field in message.ordered_fields:
        if pattern < 2:
            fmt = '<' + PACK[field.type]
            raw = b''.join(struct.pack(fmt, v) for v in values(field, enums, pattern))
        else:
            raw = random_bytes(field, enums, rng)
        out[field.wire_offset:field.wire_offset + len(raw)] = raw
    return bytes(out)


def main():
    # mavparse writes its warnings to stdout, which is the cases' own.
    with contextlib.redirect_stdout(sys.stderr):
        xml = dialects(sys.argv[1])
    randoms = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    enums = {e.name: e for x in xml for e in x.enum}
    messages = sorted((m for x in xml for m in x.message), key=lambda m: m.id)
    for message in messages:
        target = message.target_system_ofs if message.target_system_fieldname else -1
        for pattern in range(2 + randoms):
            print(message.id, message.name, pattern, message.wire_min_length, message.wire_length,
                  message.crc_extra, target, payload(message, enums, pattern).hex())


if __name__ == '__main__':
    main()
