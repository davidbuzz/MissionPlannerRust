"""Fly ArduCopter SITL with a camera and record a tlog beside the dataflash log.

The vehicle takes off in GUIDED, flies two legs, and is told to take a picture
(MAV_CMD_DO_DIGICAM_CONTROL) every two seconds, with one pair 0.3 s apart so the
C#'s minimum-shutter filter has something to remove. Every message received is
written to flight.tlog as Mission Planner records one: an 8-byte big-endian
microsecond wall-clock timestamp, then the raw frame.
"""
import struct, sys, time
from pymavlink import mavutil

PORT = int(sys.argv[1])
TLOG = sys.argv[2]

tlog = open(TLOG, 'wb')
m = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % PORT, source_system=255, source_component=190)


def record(msg):
    if msg is None or msg.get_type() == 'BAD_DATA':
        return
    if msg.get_type() == 'STATUSTEXT':
        print('STATUSTEXT', msg.text, flush=True)
    tlog.write(struct.pack('>Q', int(time.time() * 1e6)))
    tlog.write(msg.get_msgbuf())


def pump(seconds):
    end = time.time() + seconds
    last = None
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is not None:
            record(msg)
            last = msg
    return last


def wait_for(pred, timeout):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        record(msg)
        if pred(msg):
            return msg
    raise SystemExit('timed out')


wait_for(lambda msg: msg.get_type() == 'HEARTBEAT', 60)
print('heartbeat', flush=True)


def setparam(name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    try:
        wait_for(lambda msg: msg.get_type() == 'PARAM_VALUE' and msg.param_id == name, 5)
    except SystemExit:
        print('param', name, 'not acknowledged', flush=True)


for stream, rate in ((mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),
                     (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
                     (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 5),
                     (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2)):
    m.mav.request_data_stream_send(m.target_system, m.target_component, stream, rate, 1)



pump(2)

# EKF and GPS
wait_for(lambda msg: msg.get_type() == 'GPS_RAW_INT' and msg.fix_type >= 3, 120)
print('gps fix', flush=True)
pump(25)

m.set_mode('GUIDED')
pump(1)
armed = False
for attempt in range(60):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    end = time.time() + 2
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=0.1)
        record(msg)
        if msg is not None and msg.get_type() == 'HEARTBEAT' and msg.get_srcComponent() == 1 and (msg.base_mode & 128):
            armed = True
    if armed:
        break
print('armed', armed, flush=True)
if not armed:
    raise SystemExit('could not arm')
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 40)
wait_for(lambda msg: msg.get_type() == 'GLOBAL_POSITION_INT' and msg.relative_alt > 38000, 90)
print('at altitude', flush=True)

pos = wait_for(lambda msg: msg.get_type() == 'GLOBAL_POSITION_INT', 10)
lat0, lon0 = pos.lat, pos.lon


def goto(dlat, dlon, alt):
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
        lat0 + dlat, lon0 + dlon, alt, 0, 0, 0, 0, 0, 0, 0, 0)


def shoot():
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL, 0, 0, 0, 0, 0, 1, 0, 0)


shots = 0
for leg, (dlat, dlon) in enumerate(((20000, 0), (20000, 25000), (0, 25000))):
    goto(dlat, dlon, 40)
    for i in range(8):
        shoot()
        shots += 1
        if leg == 1 and i == 3:
            pump(0.3)
            shoot()
            shots += 1
            pump(1.7)
        else:
            pump(2)
print('shots', shots, flush=True)

m.set_mode('LAND')
wait_for(lambda msg: msg.get_type() == 'HEARTBEAT' and not (msg.base_mode & 128), 180)
print('disarmed', flush=True)
pump(3)
tlog.close()
