#!/usr/bin/env python3
# A telemetry log for experimental-gpsinject.gui: a heartbeat, a GPS_INJECT_DATA of 5 bytes, a
# GPS_RTCM_DATA of 7, another heartbeat and a GPS_RTCM_DATA of 3 - 15 bytes of corrections in all,
# each record an 8-byte big-endian microsecond timestamp and the frame, as Mission Planner writes
# a .tlog.
#
#   tests/gui/gps-inject-tlog.py <out.tlog>
import struct
import sys

from pymavlink.dialects.v20 import ardupilotmega as mavlink

mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
stamp = 1759700000000000
out = b""


def record(message):
    global stamp, out
    stamp += 100000
    out += struct.pack(">Q", stamp) + message.pack(mav)


def padded(data, size):
    return list(data) + [0] * (size - len(data))


heartbeat = mavlink.MAVLink_heartbeat_message(2, 3, 81, 0, 4, 3)
record(heartbeat)
record(mavlink.MAVLink_gps_inject_data_message(1, 1, 5, padded(b"\xd3\x00\x13\x3e\xd0", 110)))
record(mavlink.MAVLink_gps_rtcm_data_message(0, 7, padded(b"\xd3\x00\x04\x4c\xe0\x00\x80", 180)))
record(heartbeat)
record(mavlink.MAVLink_gps_rtcm_data_message(0, 3, padded(b"\xed\xb8\x1f", 180)))
with open(sys.argv[1], "wb") as f:
    f.write(out)
