#!/usr/bin/env python3
"""Writes base-station.rtcm3: a minute of an RTK base station's RTCM 3 stream, made from known values.

No recorded RTCM stream was on the machine this was written on (none under /home, no RTKLIB, no
pyrtcm), so this builds one bit by bit from the RTCM 10403.x layouts, and the values it puts in
are the test's expectations: `crates/mp-gui/src/config/rtk_inject.rs` decodes the file and checks
the message counts, the base position and the signal strengths against the numbers below.

The frames were checked with an independent decoder, the rtcm-rs 0.11.0 crate (its
`MsgFrameIter` accepts every frame's CRC-24Q and decodes the fields back to these values), and
the CRC here is the polynomial worked bit by bit, not the table the Rust port uses.

Run from the repository root: python3 testdata/rtcm/generate.py
"""

import os

# SITL's default home, Canberra: -35.363261, 149.165230, 584 m - its ECEF in 0.1 mm.
BASE_ECEF = (-44715714384, 26692707600, -36711445368)
ANTENNA_HEIGHT = 1234  # 0.1 mm: 0.1234 m
STATION = 7

# (prn, L1 cnr dBHz) per constellation; every satellite has two signals, L1 and L2.
GPS = [(2, 41), (5, 44), (10, 47), (12, 38), (15, 50), (18, 43), (24, 46), (29, 39)]
GLONASS = [(1, 40), (7, 45), (8, 42), (14, 37), (22, 48), (23, 44)]
GALILEO = [(3, 43), (11, 46), (19, 39), (27, 49), (30, 41)]
BEIDOU = [(6, 36), (9, 42), (13, 45), (21, 40), (28, 47), (33, 44)]

# MSM signal ids: GPS 1C and 2W; GLONASS 1C and 2C; Galileo 1C and 5Q; BeiDou 2I and 7I.
SIGNALS = {"G": (2, 16), "R": (2, 8), "E": (2, 23), "B": (2, 14)}


class Bits:
    def __init__(self):
        self.bits = []

    def u(self, value, width):
        assert 0 <= value < (1 << width), (value, width)
        self.bits.extend((value >> (width - 1 - i)) & 1 for i in range(width))

    def s(self, value, width):
        assert -(1 << (width - 1)) <= value < (1 << (width - 1)), (value, width)
        self.u(value & ((1 << width) - 1), width)

    def text(self, value):
        self.u(len(value), 8)
        for c in value.encode("ascii"):
            self.u(c, 8)

    def bytes(self):
        bits = self.bits + [0] * (-len(self.bits) % 8)
        return bytes(
            sum(bit << (7 - i) for i, bit in enumerate(bits[n : n + 8]))
            for n in range(0, len(bits), 8)
        )


def crc24q(data):
    """CRC-24Q, the polynomial 0x1864CFB worked bit by bit."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def frame(payload):
    assert len(payload) < 1024
    head = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF]) + payload
    crc = crc24q(head)
    return head + bytes([crc >> 16, (crc >> 8) & 0xFF, crc & 0xFF])


def msg1005(anth=None):
    b = Bits()
    b.u(1005 if anth is None else 1006, 12)
    b.u(STATION, 12)
    b.u(0, 6)  # ITRF year
    b.u(1, 1)  # GPS
    b.u(1, 1)  # GLONASS
    b.u(1, 1)  # Galileo
    b.u(0, 1)  # reference station
    b.s(BASE_ECEF[0], 38)
    b.u(1, 1)  # single receiver oscillator
    b.u(0, 1)  # reserved
    b.s(BASE_ECEF[1], 38)
    b.u(0, 2)  # quarter cycle
    b.s(BASE_ECEF[2], 38)
    if anth is not None:
        b.u(anth, 16)
    return frame(b.bytes())


def msm(number, sats, sys, tow_ms, seven):
    """An MSM4 (seven False) or MSM7 message: every satellite with both of its signals."""
    b = Bits()
    b.u(number, 12)
    b.u(STATION, 12)
    b.u(tow_ms, 30)
    b.u(0, 1)  # multiple message
    b.u(0, 3)  # IODS
    b.u(0, 7)  # reserved
    b.u(0, 2)  # clock steering
    b.u(0, 2)  # external clock
    b.u(0, 1)  # smoothing
    b.u(0, 3)  # smoothing interval
    prns = [prn for prn, _ in sats]
    for n in range(1, 65):
        b.u(1 if n in prns else 0, 1)
    sigs = SIGNALS[sys]
    for n in range(1, 33):
        b.u(1 if n in sigs else 0, 1)
    for _ in range(len(sats) * len(sigs)):
        b.u(1, 1)
    for i, _ in enumerate(sats):
        b.u(70 + i, 8)  # whole milliseconds of range
    if seven:
        for _ in sats:
            b.u(0, 4)  # extended info
    for i, _ in enumerate(sats):
        b.u(100 + 13 * i, 10)  # the range's fraction of a millisecond
    if seven:
        for i, _ in enumerate(sats):
            b.s(-300 + 40 * i, 14)  # rough phase range rate
    cells = [(cnr, sig) for _, cnr in sats for sig in range(len(sigs))]
    for i, _ in enumerate(cells):
        b.s(1000 * i - 3000, 20 if seven else 15)  # fine pseudorange
    for i, _ in enumerate(cells):
        b.s(2000 * i - 5000, 24 if seven else 22)  # fine phase range
    for _ in cells:
        b.u(9, 10 if seven else 4)  # lock time
    for _ in cells:
        b.u(0, 1)  # half cycle
    for cnr, sig in cells:
        # L2 a little weaker than L1. MSM7 is in 1/16 dBHz, MSM4 in whole dBHz.
        value = cnr - 3 * sig
        b.u(value * 16 + 5 if seven else value, 10 if seven else 6)
    if seven:
        for i, _ in enumerate(cells):
            b.s(100 * i - 200, 15)  # fine phase range rate
    return frame(b.bytes())


def msg1004(sats, tow_ms):
    b = Bits()
    b.u(1004, 12)
    b.u(STATION, 12)
    b.u(tow_ms, 30)
    b.u(0, 1)  # synchronous
    b.u(len(sats), 5)
    b.u(0, 1)  # smoothing
    b.u(0, 3)  # smoothing interval
    for i, (prn, cnr) in enumerate(sats):
        b.u(prn, 6)
        b.u(0, 1)  # L1 code
        b.u(1_000_000 + 7919 * i, 24)  # L1 pseudorange, 0.02 m
        b.s(1200 - 300 * i, 20)  # L1 phase - pseudorange, 0.5 mm
        b.u(100, 7)  # L1 lock time
        b.u(70, 8)  # ambiguity, light milliseconds
        b.u(cnr * 4, 8)  # L1 CNR, 0.25 dBHz
        b.u(0, 2)  # L2 code
        b.s(-50 + 11 * i, 14)  # L2 - L1 pseudorange
        b.s(900 - 200 * i, 20)  # L2 phase - L1 pseudorange
        b.u(100, 7)  # L2 lock time
        b.u((cnr - 6) * 4, 8)  # L2 CNR
    return frame(b.bytes())


def msg1012(sats, tod_ms):
    b = Bits()
    b.u(1012, 12)
    b.u(STATION, 12)
    b.u(tod_ms, 27)
    b.u(0, 1)  # synchronous
    b.u(len(sats), 5)
    b.u(0, 1)  # smoothing
    b.u(0, 3)  # smoothing interval
    for i, (prn, cnr) in enumerate(sats):
        b.u(prn, 6)
        b.u(0, 1)  # L1 code
        b.u(7 + i, 5)  # frequency channel
        b.u(2_000_000 + 6007 * i, 25)  # L1 pseudorange, 0.02 m
        b.s(-800 + 250 * i, 20)  # L1 phase - pseudorange
        b.u(100, 7)  # L1 lock time
        b.u(60, 7)  # ambiguity, two light milliseconds
        b.u(cnr * 4, 8)  # L1 CNR, 0.25 dBHz
        b.u(0, 2)  # L2 code
        b.s(30 - 7 * i, 14)  # L2 - L1 pseudorange
        b.s(-400 + 90 * i, 20)  # L2 phase - L1 pseudorange
        b.u(100, 7)  # L2 lock time
        b.u((cnr - 5) * 4, 8)  # L2 CNR
    return frame(b.bytes())


def msg1033():
    b = Bits()
    b.u(1033, 12)
    b.u(STATION, 12)
    b.text("ADVNULLANTENNA")
    b.u(0, 8)  # antenna setup
    b.text("")  # antenna serial
    b.text("MPR TEST BASE")  # receiver
    b.text("1.0")  # firmware
    b.text("0001")  # receiver serial
    return frame(b.bytes())


def msg1230():
    b = Bits()
    b.u(1230, 12)
    b.u(STATION, 12)
    b.u(1, 1)  # code-phase bias indicator
    b.u(0, 3)  # reserved
    b.u(0, 4)  # no biases
    return frame(b.bytes())


def stream():
    """Sixty one-second epochs from GPS time of week 302400 s: each has the four MSM4 messages;
    every tenth starts with 1005 and 1230; the first with 1006 and 1033 as well; epoch 30 carries
    the MSM7 and legacy messages instead of MSM4 for GPS and GLONASS."""
    out = b""
    for epoch in range(60):
        tow = 302_400_000 + epoch * 1000
        if epoch % 10 == 0:
            out += msg1005()
            out += msg1230()
        if epoch == 0:
            out += msg1005(ANTENNA_HEIGHT)
            out += msg1033()
        if epoch == 30:
            out += msm(1077, GPS, "G", tow, True)
            out += msm(1087, GLONASS, "R", tow, True)
            out += msg1004(GPS[:5], tow)
            out += msg1012(GLONASS[:4], (tow + 10_800_000) % 86_400_000)
        else:
            out += msm(1074, GPS, "G", tow, False)
            out += msm(1084, GLONASS, "R", tow, False)
        out += msm(1094, GALILEO, "E", tow, False)
        out += msm(1124, BEIDOU, "B", tow, False)
    return out


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    data = stream()
    with open(os.path.join(here, "base-station.rtcm3"), "wb") as f:
        f.write(data)
    print(f"{len(data)} bytes")
