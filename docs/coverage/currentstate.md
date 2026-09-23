# CurrentState field coverage

Generated from `crates/mp-vehicle/src/coverage.rs` by `cargo test -p mp-vehicle coverage -- --ignored update_report`; a test fails when this file is stale. One row per public property or field of `ExtLibs/ArduPilot/CurrentState.cs`, in the file's order. The display text and group are the C#'s `[DisplayText]` and `[GroupText]`, which the quick view's field chooser shows. *Done* means a field or method of `mp_vehicle::VehicleState` holds the same value in the same units (SI: the C# applies the user's display units in each getter); *derived* means it can be computed from what is held.

| total | done | derived | missing | plumbing | dropped |
|---:|---:|---:|---:|---:|---:|
| 550 | 471 | 48 | 0 | 30 | 1 |

## Missing, by group

None: every field is held, derivable, plumbing or deliberately dropped.

## Every field

| C# | type | display text | group | ours |
|---|---|---|---|---|
| `Speech` | `static ISpeech` |  |  | plumbing: the speech engine the C# hangs on this class, a service rather than vehicle state |
| `multiplierdist` | `static float` |  |  | done: `DisplayUnits::dist` |
| `DistanceUnit` | `static string` |  |  | done: `DisplayUnits::dist_unit` |
| `multiplierspeed` | `static float` |  |  | done: `DisplayUnits::speed` |
| `SpeedUnit` | `static string` |  |  | done: `DisplayUnits::speed_unit` |
| `multiplieralt` | `static float` |  |  | done: `DisplayUnits::alt` |
| `AltUnit` | `static string` |  |  | done: `DisplayUnits::alt_unit` |
| `rateattitudebackup` | `static int` |  |  | plumbing: the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from |
| `ratepositionbackup` | `static int` |  |  | plumbing: the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from |
| `ratestatusbackup` | `static int` |  |  | plumbing: the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from |
| `ratesensorsbackup` | `static int` |  |  | plumbing: the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from |
| `ratercbackup` | `static int` |  |  | plumbing: the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from |
| `KIndexstatic` | `static int` |  |  | done: `VehicleState::kindex()` (process-wide, -1 until `VehicleState::set_kindex`, which start-up must call with the `kindex` setting and the K-index download, as MainV2.cs:3940-3981 does) |
| `firmware` | `Firmwares` |  |  | derived: from `autopilot` and `vehicle_type` as MAVLinkInterface.cs:6700-6815 does, `VehicleFamily::from_mav_type` for ArduPilot; the version-string lookup it tries first reads the firmware's `STATUSTEXT` banner (MAVLinkInterface.cs:1827-1830), which is not held here |
| `hilch1` | `int` |  |  | done: `hil_channels[0]` (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' roll times 10000) |
| `hilch2` | `int` |  |  | done: `hil_channels[1]` (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' pitch times 10000) |
| `hilch3` | `int` |  |  | done: `hil_channels[2]` (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' throttle times 10000) |
| `hilch4` | `int` |  |  | done: `hil_channels[3]` (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' yaw times 10000) |
| `hilch5` | `int` |  |  | done: `hil_channels[4]` (`RC_CHANNELS_SCALED`) |
| `hilch6` | `int` |  |  | done: `hil_channels[5]` (`RC_CHANNELS_SCALED`) |
| `hilch7` | `int` |  |  | done: `hil_channels[6]` (`RC_CHANNELS_SCALED`) |
| `hilch8` | `int` |  |  | done: `hil_channels[7]` (`RC_CHANNELS_SCALED`) |
| `lastautowp` | `int` |  |  | done: `last_auto_wp` (`None` is the C#'s -1) |
| `parent` | `MAVState` |  |  | plumbing: the back-reference to the owning `MAVState`; `VehicleRegistry` holds that relationship |
| `rcoverridech1` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech10` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech11` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech12` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech13` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech14` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech15` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech16` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech17` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech18` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech2` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech3` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech4` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech5` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech6` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech7` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech8` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `rcoverridech9` | `short` |  |  | plumbing: the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`) |
| `sensors_enabled` | `Mavlink_Sensors` |  |  | done: `sensors.enabled` |
| `sensors_health` | `Mavlink_Sensors` |  |  | done: `sensors.health` |
| `sensors_present` | `Mavlink_Sensors` |  |  | done: `sensors.present` |
| `prearmstatus` | `bool` |  |  | derived: connected and (`sensors.health` or not `sensors.enabled`) at the pre-arm bit, 0x10000000 |
| `custom_field_names` | `static Dictionary<string, string>` |  |  | plumbing: the static name-to-slot map behind `customfield0` to `customfield19` |
| `customfield0` | `float` |  |  | done: `custom_fields[0]` (a `NAMED_VALUE_FLOAT`, in the field its name claimed; the names, the C#'s static `custom_field_names`, are `VehicleState::custom_field_name`, and the `customfield<n>` settings start-up adds are `VehicleState::add_custom_field_name`) |
| `customfield1` | `float` |  |  | done: `custom_fields[1]` |
| `customfield2` | `float` |  |  | done: `custom_fields[2]` |
| `customfield3` | `float` |  |  | done: `custom_fields[3]` |
| `customfield4` | `float` |  |  | done: `custom_fields[4]` |
| `customfield5` | `float` |  |  | done: `custom_fields[5]` |
| `customfield6` | `float` |  |  | done: `custom_fields[6]` |
| `customfield7` | `float` |  |  | done: `custom_fields[7]` |
| `customfield8` | `float` |  |  | done: `custom_fields[8]` |
| `customfield9` | `float` |  |  | done: `custom_fields[9]` |
| `customfield10` | `float` |  |  | done: `custom_fields[10]` |
| `customfield11` | `float` |  |  | done: `custom_fields[11]` |
| `customfield12` | `float` |  |  | done: `custom_fields[12]` |
| `customfield13` | `float` |  |  | done: `custom_fields[13]` |
| `customfield14` | `float` |  |  | done: `custom_fields[14]` |
| `customfield15` | `float` |  |  | done: `custom_fields[15]` |
| `customfield16` | `float` |  |  | done: `custom_fields[16]` |
| `customfield17` | `float` |  |  | done: `custom_fields[17]` |
| `customfield18` | `float` |  |  | done: `custom_fields[18]` |
| `customfield19` | `float` |  |  | done: `custom_fields[19]` |
| `roll` | `float` | Roll (deg) | Attitude | derived: `attitude.roll`, in radians where the C# holds degrees |
| `pitch` | `float` | Pitch (deg) | Attitude | derived: `attitude.pitch`, in radians where the C# holds degrees |
| `yaw` | `float` | Yaw (deg) | Attitude | derived: `attitude.yaw`, in radians where the C# holds degrees from 0 to 360 |
| `SSA` | `float` | SSA (deg) | Attitude | done: `ssa` |
| `AOA` | `float` | AOA (deg) | Attitude | done: `aoa` |
| `groundcourse` | `float` | GroundCourse (deg) | Position | done: `gps.course` |
| `lat` | `double` | Latitude (dd) | Position | done: `position` (`None` is the C#'s 0, 0) |
| `lng` | `double` | Longitude (dd) | Position | done: `position` (`None` is the C#'s 0, 0) |
| `alt` | `float` | Altitude (alt) | Position | done: `altitude_relative` (the C# subtracts the user's `altoffsethome`, 0 unless set: `alt()` is that difference, in the C#'s single precision) |
| `altasl` | `float` | Altitude (alt) | Position | done: `altitude_msl` |
| `horizondist` | `float` | Horizon Dist (dist) | Position | derived: 3570 * sqrt(`altitude_relative`), metres |
| `vx` | `double` | Velocity X (ms) | Position | done: `velocity_north` |
| `vy` | `double` | Velocity Y (ms) | Position | done: `velocity_east` |
| `vz` | `double` | Velocity Z (ms) | Position | done: `velocity_down` |
| `vlen` | `double` |  | Position | derived: the length of (`velocity_north`, `velocity_east`, `velocity_down`) |
| `altoffsethome` | `float` | Alt Home Offset (dist) | Position | done: `alt_offset_home` (0 until the flight screen's Home Alt button writes it, FlightData.cs:1236-1247) |
| `gpsstatus` | `float` | Gps Status | Position | done: `gps.fix_type` |
| `gpshdop` | `float` | Gps HDOP | Position | done: `gps.hdop` |
| `satcount` | `float` | Sat Count | Position | done: `gps.satellites_visible` |
| `gpsh_acc` | `float` | H Acc (m) | Position | done: `gps.h_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpsv_acc` | `float` | V Acc (m) | Position | done: `gps.v_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpsvel_acc` | `float` | Velocity Accuracy | Position | done: `gps.vel_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpshdg_acc` | `float` | Heading Accuracy | Position | done: `gps.hdg_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpsyaw` | `float` | GPS Yaw (deg) | Position | done: `gps.yaw` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `lat2` | `double` | Latitude2 (dd) | Position | done: `gps2.position` (`None` is the C#'s 0, 0) |
| `lng2` | `double` | Longitude2 (dd) | Position | done: `gps2.position` (`None` is the C#'s 0, 0) |
| `altasl2` | `float` | Altitude2 (dist) | Position | done: `gps2.altitude_msl` |
| `gpsstatus2` | `float` | Gps Status2 | Position | done: `gps2.fix_type` |
| `gpshdop2` | `float` | Gps HDOP2 | Position | done: `gps2.hdop` |
| `satcount2` | `float` | Sat Count2 | Position | done: `gps2.satellites_visible` |
| `groundspeed2` | `float` |  | Position | done: `gps2.ground_speed` |
| `groundcourse2` | `float` | GroundCourse2 (deg) | Position | done: `gps2.course` |
| `gpsh_acc2` | `float` | H Acc2 (m) | Position | done: `gps2.h_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpsv_acc2` | `float` | V Acc2 (m) | Position | done: `gps2.v_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpsvel_acc2` | `float` | Velocity Accuracy | Position | done: `gps2.vel_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpshdg_acc2` | `float` | Heading Accuracy | Position | done: `gps2.hdg_acc` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `gpsyaw2` | `float` | GPS Yaw (deg) | Position | done: `gps2.yaw` (a MAVLink 1 frame reads 0 where the C# says -1) |
| `satcountB` | `float` | Sat Count Blend | Position | derived: `gps.satellites_visible` + `gps2.satellites_visible` |
| `gpstime` | `DateTime` |  | Position | done: `gps_time_unix_ms` (milliseconds since 1970, where the C# holds a `DateTime`) |
| `altd1000` | `float` |  | Other | derived: `altitude_relative` / 1000 % 10, the thousands digit |
| `altd100` | `float` |  | Other | derived: `altitude_relative` / 100 % 10, the hundreds digit |
| `airspeed` | `float` | AirSpeed (speed) | Sensor | done: `air_speed` |
| `targetairspeed` | `float` | Airspeed Target (speed) | NAV | derived: `target_airspeed()`, without the C#'s low-pass and its division of the error by 100 |
| `lowairspeed` | `bool` |  |  | done: `low_airspeed` (from `VFR_HUD`; the `AIRSPEED_MIN` or `ARSPD_FBW_MIN` parameter the C# reads itself comes from `set_airspeed_min_params()`, which the parameter table's owner must call) |
| `asratio` | `float` | Airspeed Ratio | Calibration | done: `airspeed_ratio` |
| `airspeed1_temp` | `float` | Airspeed1 Temperature | Sensor | done: `airspeed1_temp` |
| `airspeed2_temp` | `float` | Airspeed2 Temperature | Sensor | done: `airspeed2_temp` |
| `groundspeed` | `float` | GroundSpeed (speed) | Position | done: `ground_speed` |
| `ax` | `float` | Accel X | Sensor | done: `imu[0].accel[0]` |
| `ay` | `float` | Accel Y | Sensor | done: `imu[0].accel[1]` |
| `az` | `float` | Accel Z | Sensor | done: `imu[0].accel[2]` |
| `accelsq` | `float` | Accel Strength | Sensor | derived: the length of `imu[0].accel`, / 1000 |
| `gx` | `float` | Gyro X | Sensor | done: `imu[0].gyro[0]` |
| `gy` | `float` | Gyro Y | Sensor | done: `imu[0].gyro[1]` |
| `gz` | `float` | Gyro Z | Sensor | done: `imu[0].gyro[2]` |
| `gyrosq` | `float` | Gyro Strength | Sensor | derived: the length of `imu[0].gyro` |
| `mx` | `float` | Mag X | Sensor | done: `imu[0].mag[0]` |
| `my` | `float` | Mag Y | Sensor | done: `imu[0].mag[1]` |
| `mz` | `float` | Mag Z | Sensor | done: `imu[0].mag[2]` |
| `magfield` | `float` | Mag Field | Sensor | derived: the length of `imu[0].mag` |
| `imu1_temp` | `float` | IMU1 Temperature | Sensor | done: `imu[0].temperature` |
| `ax2` | `float` | Accel2 X | Sensor | done: `imu[1].accel[0]` |
| `ay2` | `float` | Accel2 Y | Sensor | done: `imu[1].accel[1]` |
| `az2` | `float` | Accel2 Z | Sensor | done: `imu[1].accel[2]` |
| `accelsq2` | `float` | Accel2 Strength | Sensor | derived: the length of `imu[1].accel`, / 1000 |
| `gx2` | `float` | Gyro2 X | Sensor | done: `imu[1].gyro[0]` |
| `gy2` | `float` | Gyro2 Y | Sensor | done: `imu[1].gyro[1]` |
| `gz2` | `float` | Gyro2 Z | Sensor | done: `imu[1].gyro[2]` |
| `gyrosq2` | `float` | Gyro2 Strength | Sensor | derived: the length of `imu[1].gyro` |
| `mx2` | `float` | Mag2 X | Sensor | done: `imu[1].mag[0]` |
| `my2` | `float` | Mag2 Y | Sensor | done: `imu[1].mag[1]` |
| `mz2` | `float` | Mag2 Z | Sensor | done: `imu[1].mag[2]` |
| `magfield2` | `float` | Mag2 Field | Sensor | derived: the length of `imu[1].mag` |
| `imu2_temp` | `float` | IMU2 Temperature | Sensor | done: `imu[1].temperature` |
| `ax3` | `float` | Accel3 X | Sensor | done: `imu[2].accel[0]` |
| `ay3` | `float` | Accel3 Y | Sensor | done: `imu[2].accel[1]` |
| `az3` | `float` | Accel3 Z | Sensor | done: `imu[2].accel[2]` |
| `accelsq3` | `float` | Accel3 Strength | Sensor | derived: the length of `imu[2].accel`, / 1000 |
| `gx3` | `float` | Gyro3 X | Sensor | done: `imu[2].gyro[0]` |
| `gy3` | `float` | Gyro3 Y | Sensor | done: `imu[2].gyro[1]` |
| `gz3` | `float` | Gyro3 Z | Sensor | done: `imu[2].gyro[2]` |
| `gyrosq3` | `float` | Gyro3 Strength | Sensor | derived: the length of `imu[2].gyro` |
| `mx3` | `float` | Mag3 X | Sensor | done: `imu[2].mag[0]` |
| `my3` | `float` | Mag3 Y | Sensor | done: `imu[2].mag[1]` |
| `mz3` | `float` | Mag3 Z | Sensor | done: `imu[2].mag[2]` |
| `magfield3` | `float` | Mag3 Field | Sensor | derived: the length of `imu[2].mag` |
| `imu3_temp` | `float` | IMU3 Temperature | Sensor | done: `imu[2].temperature` |
| `hygrotemp1` | `short` | hygrotemp1 (cdegC) | Sensor | done: `hygrometers[0].temperature` |
| `hygrohumi1` | `ushort` | hygrohumi1 (c%) | Sensor | done: `hygrometers[0].humidity` |
| `hygrotemp2` | `short` | hygrotemp2 (cdegC) | Sensor | done: `hygrometers[1].temperature` |
| `hygrohumi2` | `ushort` | hygrohumi2 (c%) | Sensor | done: `hygrometers[1].humidity` |
| `ch1in` | `float` |  | RadioIn | done: `rc.values[0]` |
| `ch2in` | `float` |  | RadioIn | done: `rc.values[1]` |
| `ch3in` | `float` |  | RadioIn | done: `rc.values[2]` |
| `ch4in` | `float` |  | RadioIn | done: `rc.values[3]` |
| `ch5in` | `float` |  | RadioIn | done: `rc.values[4]` |
| `ch6in` | `float` |  | RadioIn | done: `rc.values[5]` |
| `ch7in` | `float` |  | RadioIn | done: `rc.values[6]` |
| `ch8in` | `float` |  | RadioIn | done: `rc.values[7]` |
| `ch9in` | `float` |  | RadioIn | done: `rc.values[8]` |
| `ch10in` | `float` |  | RadioIn | done: `rc.values[9]` |
| `ch11in` | `float` |  | RadioIn | done: `rc.values[10]` |
| `ch12in` | `float` |  | RadioIn | done: `rc.values[11]` |
| `ch13in` | `float` |  | RadioIn | done: `rc.values[12]` |
| `ch14in` | `float` |  | RadioIn | done: `rc.values[13]` |
| `ch15in` | `float` |  | RadioIn | done: `rc.values[14]` |
| `ch16in` | `float` |  | RadioIn | done: `rc.values[15]` |
| `ch1out` | `float` |  | RadioOut | done: `servo_outputs[0]` |
| `ch2out` | `float` |  | RadioOut | done: `servo_outputs[1]` |
| `ch3out` | `float` |  | RadioOut | done: `servo_outputs[2]` |
| `ch4out` | `float` |  | RadioOut | done: `servo_outputs[3]` |
| `ch5out` | `float` |  | RadioOut | done: `servo_outputs[4]` |
| `ch6out` | `float` |  | RadioOut | done: `servo_outputs[5]` |
| `ch7out` | `float` |  | RadioOut | done: `servo_outputs[6]` |
| `ch8out` | `float` |  | RadioOut | done: `servo_outputs[7]` |
| `ch9out` | `float` |  | RadioOut | done: `servo_outputs[8]` |
| `ch10out` | `float` |  | RadioOut | done: `servo_outputs[9]` |
| `ch11out` | `float` |  | RadioOut | done: `servo_outputs[10]` |
| `ch12out` | `float` |  | RadioOut | done: `servo_outputs[11]` |
| `ch13out` | `float` |  | RadioOut | done: `servo_outputs[12]` |
| `ch14out` | `float` |  | RadioOut | done: `servo_outputs[13]` |
| `ch15out` | `float` |  | RadioOut | done: `servo_outputs[14]` |
| `ch16out` | `float` |  | RadioOut | done: `servo_outputs[15]` |
| `ch17out` | `float` |  | RadioOut | done: `servo_outputs[16]` |
| `ch18out` | `float` |  | RadioOut | done: `servo_outputs[17]` |
| `ch19out` | `float` |  | RadioOut | done: `servo_outputs[18]` |
| `ch20out` | `float` |  | RadioOut | done: `servo_outputs[19]` |
| `ch21out` | `float` |  | RadioOut | done: `servo_outputs[20]` |
| `ch22out` | `float` |  | RadioOut | done: `servo_outputs[21]` |
| `ch23out` | `float` |  | RadioOut | done: `servo_outputs[22]` |
| `ch24out` | `float` |  | RadioOut | done: `servo_outputs[23]` |
| `ch25out` | `float` |  | RadioOut | done: `servo_outputs[24]` |
| `ch26out` | `float` |  | RadioOut | done: `servo_outputs[25]` |
| `ch27out` | `float` |  | RadioOut | done: `servo_outputs[26]` |
| `ch28out` | `float` |  | RadioOut | done: `servo_outputs[27]` |
| `ch29out` | `float` |  | RadioOut | done: `servo_outputs[28]` |
| `ch30out` | `float` |  | RadioOut | done: `servo_outputs[29]` |
| `ch31out` | `float` |  | RadioOut | done: `servo_outputs[30]` |
| `ch32out` | `float` |  | RadioOut | done: `servo_outputs[31]` |
| `esc1_volt` | `float` |  | ESC | done: `escs[0].voltage` |
| `esc1_curr` | `float` |  | ESC | done: `escs[0].current` |
| `esc1_rpm` | `float` |  | ESC | done: `escs[0].rpm` |
| `esc1_temp` | `float` |  | ESC | done: `escs[0].temperature` |
| `esc2_volt` | `float` |  | ESC | done: `escs[1].voltage` |
| `esc2_curr` | `float` |  | ESC | done: `escs[1].current` |
| `esc2_rpm` | `float` |  | ESC | done: `escs[1].rpm` |
| `esc2_temp` | `float` |  | ESC | done: `escs[1].temperature` |
| `esc3_volt` | `float` |  | ESC | done: `escs[2].voltage` |
| `esc3_curr` | `float` |  | ESC | done: `escs[2].current` |
| `esc3_rpm` | `float` |  | ESC | done: `escs[2].rpm` |
| `esc3_temp` | `float` |  | ESC | done: `escs[2].temperature` |
| `esc4_volt` | `float` |  | ESC | done: `escs[3].voltage` |
| `esc4_curr` | `float` |  | ESC | done: `escs[3].current` |
| `esc4_rpm` | `float` |  | ESC | done: `escs[3].rpm` |
| `esc4_temp` | `float` |  | ESC | done: `escs[3].temperature` |
| `esc5_volt` | `float` |  | ESC | done: `escs[4].voltage` |
| `esc5_curr` | `float` |  | ESC | done: `escs[4].current` |
| `esc5_rpm` | `float` |  | ESC | done: `escs[4].rpm` |
| `esc5_temp` | `float` |  | ESC | done: `escs[4].temperature` |
| `esc6_volt` | `float` |  | ESC | done: `escs[5].voltage` |
| `esc6_curr` | `float` |  | ESC | done: `escs[5].current` |
| `esc6_rpm` | `float` |  | ESC | done: `escs[5].rpm` |
| `esc6_temp` | `float` |  | ESC | done: `escs[5].temperature` |
| `esc7_volt` | `float` |  | ESC | done: `escs[6].voltage` |
| `esc7_curr` | `float` |  | ESC | done: `escs[6].current` |
| `esc7_rpm` | `float` |  | ESC | done: `escs[6].rpm` |
| `esc7_temp` | `float` |  | ESC | done: `escs[6].temperature` |
| `esc8_volt` | `float` |  | ESC | done: `escs[7].voltage` |
| `esc8_curr` | `float` |  | ESC | done: `escs[7].current` |
| `esc8_rpm` | `float` |  | ESC | done: `escs[7].rpm` |
| `esc8_temp` | `float` |  | ESC | done: `escs[7].temperature` |
| `esc9_volt` | `float` |  | ESC | done: `escs[8].voltage` |
| `esc9_curr` | `float` |  | ESC | done: `escs[8].current` |
| `esc9_rpm` | `float` |  | ESC | done: `escs[8].rpm` |
| `esc9_temp` | `float` |  | ESC | done: `escs[8].temperature` |
| `esc10_volt` | `float` |  | ESC | done: `escs[9].voltage` |
| `esc10_curr` | `float` |  | ESC | done: `escs[9].current` |
| `esc10_rpm` | `float` |  | ESC | done: `escs[9].rpm` |
| `esc10_temp` | `float` |  | ESC | done: `escs[9].temperature` |
| `esc11_volt` | `float` |  | ESC | done: `escs[10].voltage` |
| `esc11_curr` | `float` |  | ESC | done: `escs[10].current` |
| `esc11_rpm` | `float` |  | ESC | done: `escs[10].rpm` |
| `esc11_temp` | `float` |  | ESC | done: `escs[10].temperature` |
| `esc12_volt` | `float` |  | ESC | done: `escs[11].voltage` |
| `esc12_curr` | `float` |  | ESC | done: `escs[11].current` |
| `esc12_rpm` | `float` |  | ESC | done: `escs[11].rpm` |
| `esc12_temp` | `float` |  | ESC | done: `escs[11].temperature` |
| `esc13_volt` | `float` |  | ESC | done: `escs[12].voltage` |
| `esc13_curr` | `float` |  | ESC | done: `escs[12].current` |
| `esc13_rpm` | `float` |  | ESC | done: `escs[12].rpm` |
| `esc13_temp` | `float` |  | ESC | done: `escs[12].temperature` |
| `esc14_volt` | `float` |  | ESC | done: `escs[13].voltage` |
| `esc14_curr` | `float` |  | ESC | done: `escs[13].current` |
| `esc14_rpm` | `float` |  | ESC | done: `escs[13].rpm` |
| `esc14_temp` | `float` |  | ESC | done: `escs[13].temperature` |
| `esc15_volt` | `float` |  | ESC | done: `escs[14].voltage` |
| `esc15_curr` | `float` |  | ESC | done: `escs[14].current` |
| `esc15_rpm` | `float` |  | ESC | done: `escs[14].rpm` |
| `esc15_temp` | `float` |  | ESC | done: `escs[14].temperature` |
| `esc16_volt` | `float` |  | ESC | done: `escs[15].voltage` |
| `esc16_curr` | `float` |  | ESC | done: `escs[15].current` |
| `esc16_rpm` | `float` |  | ESC | done: `escs[15].rpm` |
| `esc16_temp` | `float` |  | ESC | done: `escs[15].temperature` |
| `ch1percent` | `float` |  | RadioOut | derived: `servo_outputs[0]` scaled by the SERVO1_MIN, _TRIM, _MAX and _REVERSED parameters, or by 1000-1500-2000 without them |
| `ch3percent` | `float` |  | RadioOut | done: `throttle_percent` |
| `failsafe` | `bool` | Failsafe | Software | derived: `system_status` == MAV_STATE_CRITICAL (5), as the HUD derives it; `HIGH_LATENCY`'s failsafe byte is not held |
| `rxrssi` | `int` | RX Rssi | Telem | derived: `rc.rssi` * 100 / 254, 0 when 255 (`RC_CHANNELS`); the C# divides by 255 after `RC_CHANNELS_RAW` |
| `crit_AOA` | `float` | Crit AOA (deg) | Attitude | derived: the AOA_CRIT parameter, 25 without it |
| `lowgroundspeed` | `bool` |  |  | dropped: nothing in the C# sets it, so it is always false; the HUD's low-ground-speed warning is its own |
| `verticalspeed` | `float` | Vertical Speed (speed) | Position | done: `vertical_speed()` (the `alt` setter's filtered rate from `GLOBAL_POSITION_INT` and the high-latency messages, against `datetime`) |
| `verticalspeed_fpm` | `double` | Vertical Speed (fpm) | Position | derived: `velocity_down` * -3.28084 * 60 |
| `glide_ratio` | `double` | Glide Ratio | Position | derived: the horizontal length of (`velocity_north`, `velocity_east`) over `velocity_down` |
| `nav_roll` | `float` | Roll Target (deg) | NAV | done: `nav.roll` |
| `nav_pitch` | `float` | Pitch Target (deg) | NAV | done: `nav.pitch` |
| `nav_bearing` | `float` | Bearing Target (deg) | NAV | done: `nav.bearing` |
| `target_bearing` | `float` | Bearing Target (deg) | NAV | done: `nav.target_bearing` |
| `wp_dist` | `float` | Dist to WP (dist) | NAV | done: `nav.wp_distance` |
| `alt_error` | `float` | Altitude Error (dist) | NAV | done: `nav.alt_error` |
| `ber_error` | `float` | Bearing Error (deg) | NAV | derived: `nav.target_bearing` - `attitude.yaw` in degrees |
| `aspd_error` | `float` | Airspeed Error (speed) | NAV | derived: `nav.airspeed_error` / 100: the C# divides the wire's m/s by 100, which this deliberately does not |
| `xtrack_error` | `float` | Xtrack Error (m) | NAV | done: `nav.xtrack_error` |
| `wpno` | `float` | WP No | NAV | done: `mission_current` |
| `mode` | `string` | Mode | NAV | derived: `flight_mode_name(vehicle_type, custom_mode)` |
| `climbrate` | `float` | ClimbRate (speed) | Position | done: `climb_rate` (from `VFR_HUD`; until the first, the `alt` setter's unfiltered rate against `datetime`, as the C#'s) |
| `tot` | `int` | Time over Target (sec) | NAV | derived: `nav.wp_distance` / `ground_speed`, whole seconds, 0 when not moving |
| `toh` | `int` | Time over Home (sec) | NAV | derived: `DistToHome` / `ground_speed`, whole seconds, 0 when not moving |
| `distTraveled` | `float` | Dist Traveled (dist) | Position | done: `dist_traveled` (metres, where the C# adds display units; counted by `update_current_settings`, which the link must call on every vehicle after each read, as `VehicleRegistry::update_current_settings`) |
| `timeSinceArmInAir` | `float` | Time in Air (sec) | Position | done: `time_since_arm_in_air` (counted by `update_current_settings`; `HEARTBEAT` arming restarts it) |
| `timeInAir` | `float` | Time in Air (sec) | Position | done: `time_in_air` (counted by `update_current_settings`) |
| `timeInAirMinSec` | `float` | Time in Air (min.sec) | Position | done: `time_in_air_min_sec()` |
| `turnrate` | `float` | Turn Rate (speed) | Position | done: `turn_rate()` |
| `turng` | `float` | Turn Gs (load) | Position | derived: 1 / cos(`attitude.roll`) |
| `radius` | `float` | Turn Radius (dist) | Position | derived: `ground_speed`² / (9.80665 * tan(`attitude.roll`)), 0 at 1 m/s and below |
| `QNH` | `float` |  | Position | derived: `press_abs` / exp(ln(1 - `altitude_msl` / (153.8462 * 288.15)) / 0.190259) |
| `wind_dir` | `float` | Wind Direction (Deg) | Position | done: `wind_direction` (from `WIND`; the C#'s own estimate when no `WIND` arrives is not ported) |
| `wind_vel` | `float` | Wind Velocity (speed) | Position | done: `wind_speed` (m/s; the C# stores this one in display units) |
| `targetaltd100` | `float` |  | NAV | derived: `target_altitude()` / 100 % 10 |
| `targetalt` | `float` |  | NAV | derived: `target_altitude()`, without the C#'s low-pass |
| `messages` | `List<(DateTime time, string message)>` |  |  | plumbing: the vehicle's `STATUSTEXT` history, which the link keeps (`mp_link::messages::MessageLog`); the state here is `Copy` and holds no text |
| `message` | `string` |  |  | plumbing: the last `STATUSTEXT`; see `messages` |
| `messageHigh` | `string` |  |  | derived: mp-gui's `hud::high_priority_message`, from `ekf` and `sensors`. Not derived there yet: the `STATUSTEXT` messages MAVLinkInterface raises to it (severity at or below the setting, default 4, or text starting `Tuning:`, `PreArm:` or `Arm:`; MAVLinkInterface.cs:5397-5420), and the fence-breach (`fence_breach`), over-current (`board.voltage_flags`) and high-latency failure texts |
| `messageHighSeverity` | `MAVLink.MAV_SEVERITY` |  | Other | derived: EMERGENCY for the messages the C# raises itself; the `STATUSTEXT`'s own severity for the ones MAVLinkInterface raises (MAVLinkInterface.cs:5399-5420) |
| `battery_voltage` | `double` | Bat Voltage (V) | Battery | done: `battery.voltage` |
| `battery_voltage3` | `double` | Bat Voltage (V) | Battery | done: `batteries[1].voltage` |
| `battery_voltage4` | `double` | Bat Voltage (V) | Battery | done: `batteries[2].voltage` |
| `battery_voltage5` | `double` | Bat Voltage (V) | Battery | done: `batteries[3].voltage` |
| `battery_voltage6` | `double` | Bat Voltage (V) | Battery | done: `batteries[4].voltage` |
| `battery_voltage7` | `double` | Bat Voltage (V) | Battery | done: `batteries[5].voltage` |
| `battery_voltage8` | `double` | Bat Voltage (V) | Battery | done: `batteries[6].voltage` |
| `battery_voltage9` | `double` | Bat Voltage (V) | Battery | done: `batteries[7].voltage` |
| `battery_remaining` | `int` | Bat Remaining (%) | Battery | done: `battery.remaining_percent` |
| `battery_remaining2` | `int` | Bat Remaining (%) | Battery | done: `batteries[0].remaining_percent` |
| `battery_remaining3` | `int` | Bat Remaining (%) | Battery | done: `batteries[1].remaining_percent` |
| `battery_remaining4` | `int` | Bat Remaining (%) | Battery | done: `batteries[2].remaining_percent` |
| `battery_remaining5` | `int` | Bat Remaining (%) | Battery | done: `batteries[3].remaining_percent` |
| `battery_remaining6` | `int` | Bat Remaining (%) | Battery | done: `batteries[4].remaining_percent` |
| `battery_remaining7` | `int` | Bat Remaining (%) | Battery | done: `batteries[5].remaining_percent` |
| `battery_remaining8` | `int` | Bat Remaining (%) | Battery | done: `batteries[6].remaining_percent` |
| `battery_remaining9` | `int` | Bat Remaining (%) | Battery | done: `batteries[7].remaining_percent` |
| `current` | `double` | Bat Current (Amps) | Battery | done: `battery.current` |
| `current2` | `double` | Bat2 Current (Amps) | Battery | done: `batteries[0].current` |
| `current3` | `double` | Bat3 Current (Amps) | Battery | done: `batteries[1].current` |
| `current4` | `double` | Bat4 Current (Amps) | Battery | done: `batteries[2].current` |
| `current5` | `double` | Bat5 Current (Amps) | Battery | done: `batteries[3].current` |
| `current6` | `double` | Bat6 Current (Amps) | Battery | done: `batteries[4].current` |
| `current7` | `double` | Bat7 Current (Amps) | Battery | done: `batteries[5].current` |
| `current8` | `double` | Bat8 Current (Amps) | Battery | done: `batteries[6].current` |
| `current9` | `double` | Bat9 Current (Amps) | Battery | done: `batteries[7].current` |
| `watts` | `double` | Bat Watts | Battery | derived: `battery.voltage` * `battery.current` |
| `battery_mahperkm` | `double` | Bat efficiency (mah/km) | Battery | done: `battery_mah_per_km()` (unguarded: infinite or NaN before any distance) |
| `battery_kmleft` | `double` | Bat km left EST (km) | Battery | done: `battery_km_left()` |
| `battery_usedmah` | `double` | Bat used EST (mah) | Battery | done: `battery_used_mah` (`BATTERY_STATUS`'s `current_consumed`, which is also `battery.consumed_mah`, and between reports the `SYS_STATUS` current integrated against `datetime`) |
| `battery_cell1` | `double` |  | Battery | done: `battery.cells[0]` |
| `battery_cell2` | `double` |  | Battery | done: `battery.cells[1]` |
| `battery_cell3` | `double` |  | Battery | done: `battery.cells[2]` |
| `battery_cell4` | `double` |  | Battery | done: `battery.cells[3]` |
| `battery_cell5` | `double` |  | Battery | done: `battery.cells[4]` |
| `battery_cell6` | `double` |  | Battery | done: `battery.cells[5]` |
| `battery_cell7` | `double` |  | Battery | done: `battery.cells[6]` |
| `battery_cell8` | `double` |  | Battery | done: `battery.cells[7]` |
| `battery_cell9` | `double` |  | Battery | done: `battery.cells[8]` |
| `battery_cell10` | `double` |  | Battery | done: `battery.cells[9]` |
| `battery_cell11` | `double` |  | Battery | done: `battery.cells[10]` |
| `battery_cell12` | `double` |  | Battery | done: `battery.cells[11]` |
| `battery_cell13` | `double` |  | Battery | done: `battery.cells[12]` |
| `battery_cell14` | `double` |  | Battery | done: `battery.cells[13]` |
| `battery_temp` | `double` |  | Battery | done: `battery.temperature` |
| `battery_temp2` | `double` |  | Battery | done: `batteries[0].temperature` |
| `battery_temp3` | `double` |  | Battery | done: `batteries[1].temperature` |
| `battery_temp4` | `double` |  | Battery | done: `batteries[2].temperature` |
| `battery_temp5` | `double` |  | Battery | done: `batteries[3].temperature` |
| `battery_temp6` | `double` |  | Battery | done: `batteries[4].temperature` |
| `battery_temp7` | `double` |  | Battery | done: `batteries[5].temperature` |
| `battery_temp8` | `double` |  | Battery | done: `batteries[6].temperature` |
| `battery_temp9` | `double` |  | Battery | done: `batteries[7].temperature` |
| `battery_remainmin` | `double` |  | Battery | done: `battery.remaining_minutes` |
| `battery_remainmin2` | `double` |  | Battery | done: `batteries[0].remaining_minutes` |
| `battery_remainmin3` | `double` |  | Battery | done: `batteries[1].remaining_minutes` |
| `battery_remainmin4` | `double` |  | Battery | done: `batteries[2].remaining_minutes` |
| `battery_remainmin5` | `double` |  | Battery | done: `batteries[3].remaining_minutes` |
| `battery_remainmin6` | `double` |  | Battery | done: `batteries[4].remaining_minutes` |
| `battery_remainmin7` | `double` |  | Battery | done: `batteries[5].remaining_minutes` |
| `battery_remainmin8` | `double` |  | Battery | done: `batteries[6].remaining_minutes` |
| `battery_remainmin9` | `double` |  | Battery | done: `batteries[7].remaining_minutes` |
| `battery_usedmah2` | `double` | Bat used EST (mah) | Battery | done: `batteries[0].consumed_mah` |
| `battery_usedmah3` | `double` | Bat used EST (mah) | Battery | done: `batteries[1].consumed_mah` |
| `battery_usedmah4` | `double` | Bat used EST (mah) | Battery | done: `batteries[2].consumed_mah` |
| `battery_usedmah5` | `double` | Bat used EST (mah) | Battery | done: `batteries[3].consumed_mah` |
| `battery_usedmah6` | `double` | Bat used EST (mah) | Battery | done: `batteries[4].consumed_mah` |
| `battery_usedmah7` | `double` | Bat used EST (mah) | Battery | done: `batteries[5].consumed_mah` |
| `battery_usedmah8` | `double` | Bat used EST (mah) | Battery | done: `batteries[6].consumed_mah` |
| `battery_usedmah9` | `double` | Bat used EST (mah) | Battery | done: `batteries[7].consumed_mah` |
| `battery_voltage2` | `double` | Bat2 Voltage (V) | Battery | done: `batteries[0].voltage` |
| `HomeAlt` | `double` |  | Position | done: `home_altitude` |
| `HomeLocation` | `PointLatLngAlt` |  | Position | done: `home` (with `home_altitude`) |
| `PlannedHomeLocation` | `PointLatLngAlt` |  | Position | done: `VehicleState::planned_home()` (process-wide, as the C#'s static is; `VehicleState::set_planned_home` is what start-up must call with the `TXT_homelat`, `TXT_homelng` and `TXT_homealt` settings, as MainV2.cs:1012-1025 does) |
| `Base` | `PointLatLngAlt` |  | Position | done: `base` ((0, 0, 0) until the RTK injection page or the moving-base control writes it, ConfigSerialInjectGPS.cs:910, 1077, 1098 and Controls/MovingBase.cs:227) |
| `TrackerLocation` | `PointLatLngAlt` |  | Position | done: `tracker_location()` (home until `VehicleState::set_tracker_location` - the planner's Set Tracker Home, FlightPlanner.cs:760, 6977 - gives it a longitude) |
| `Location` | `PointLatLngAlt` |  | Position | derived: `position` with `altitude_msl` |
| `TargetLocation` | `PointLatLngAlt` |  | Position | done: `target_position` with `target_altitude_msl` (the C#'s tag on it, the type mask, is not kept) |
| `GeoFenceDist` | `float` |  | Other | done: `geo_fence_dist()` (given the fence items the C# reads from `MAVState.fencepoints`, which their owner must pass) |
| `DistToHome` | `float` | Dist to Home (dist) | Position | derived: from `tracker_location()` - home unless a tracker is set - to `position` on the C#'s flat projection, 111319.5 m a degree with longitude scaled by cos(latitude) |
| `DistFromMovingBase` | `float` | Dist to Moving Base (dist) | Position | done: `dist_from_moving_base()` (from 0° 0° until a base is set, as the C# is) |
| `ELToMAV` | `float` | Elevation to Mav (deg) | Position | derived: atan((`altitude_msl` - `home_altitude`) / `DistToHome`), degrees, from `home` |
| `AZToMAV` | `float` | Bearing to Mav (deg) | Position | derived: the bearing from `home` to `position` on the C#'s flat projection |
| `sonarrange` | `float` | Sonar Range (alt) | Sensor | done: `rangefinder.range` |
| `sonarvoltage` | `float` | Sonar Voltage (Volt) | Sensor | done: `rangefinder.voltage` |
| `rangefinder1` | `uint` | RangeFinder1 (cm) | Sensor | done: `rangefinder.distances[0]` |
| `rangefinder2` | `uint` | RangeFinder2 (cm) | Sensor | done: `rangefinder.distances[1]` |
| `rangefinder3` | `uint` | RangeFinder3 (cm) | Sensor | done: `rangefinder.distances[2]` |
| `rangefinder4` | `uint` | RangeFinder4 (cm) | Sensor | done: `rangefinder.distances[3]` |
| `rangefinder5` | `uint` | RangeFinder5 (cm) | Sensor | done: `rangefinder.distances[4]` |
| `rangefinder6` | `uint` | RangeFinder6 (cm) | Sensor | done: `rangefinder.distances[5]` |
| `rangefinder7` | `uint` | RangeFinder7 (cm) | Sensor | done: `rangefinder.distances[6]` |
| `rangefinder8` | `uint` | RangeFinder8 (cm) | Sensor | done: `rangefinder.distances[7]` |
| `rangefinder9` | `uint` | RangeFinder9 (cm) | Sensor | done: `rangefinder.distances[8]` |
| `rangefinder10` | `uint` | RangeFinder10 (cm) | Sensor | done: `rangefinder.distances[9]` |
| `freemem` | `float` |  | Software | done: `board.free_memory` |
| `load` | `float` |  | Software | done: `load` |
| `brklevel` | `float` |  | Software | done: `board.brk_level` |
| `armed` | `bool` |  | Software | done: `armed` |
| `rssi` | `float` | Sik Radio rssi | Telem | done: `radio.rssi` |
| `remrssi` | `float` | Sik Radio remote rssi | Telem | done: `radio.remrssi` |
| `txbuffer` | `byte` |  | Telem | done: `radio.txbuf` |
| `noise` | `float` | Sik Radio noise | Telem | done: `radio.noise` |
| `remnoise` | `float` | Sik Radio remote noise | Telem | done: `radio.remnoise` |
| `rxerrors` | `ushort` |  | Telem | done: `radio.rxerrors` |
| `fixedp` | `ushort` |  | Telem | done: `radio.fixed` |
| `localsnrdb` | `float` | Sik Radio snr | Telem | derived: (`radio.rssi` - `radio.noise`) / 1.9, low-passed once a second |
| `remotesnrdb` | `float` | Sik Radio remote snr | Telem | derived: (`radio.remrssi` - `radio.remnoise`) / 1.9, low-passed once a second |
| `DistRSSIRemain` | `float` | Sik Radio est dist (m) | Telem | derived: `DistToHome` * 2^((the lesser signal-to-noise - 5) / 6) |
| `packetdropremote` | `ushort` |  | Telem | done: `packet_drop_remote` |
| `linkqualitygcs` | `ushort` |  | Telem | done: `link.quality_percent()` (the C#'s zeroing after ten silent seconds is not kept) |
| `errors_count1` | `ushort` | Error Type | Hardware | done: `errors_count[0]` |
| `errors_count2` | `ushort` | Error Type | Hardware | done: `errors_count[1]` |
| `errors_count3` | `ushort` | Error Type | Hardware | done: `errors_count[2]` |
| `errors_count4` | `ushort` | Error Count | Hardware | done: `errors_count[3]` |
| `hwvoltage` | `float` | HW Voltage | Hardware | done: `board.hw_voltage` |
| `boardvoltage` | `float` | Board Voltage | Hardware | done: `board.board_voltage` (millivolts, as the C# shows them) |
| `servovoltage` | `float` | Servo Rail Voltage | Hardware | done: `board.servo_voltage` (millivolts, as the C# shows them) |
| `voltageflag` | `uint` | Voltage Flags | Hardware | done: `board.voltage_flags` |
| `i2cerrors` | `ushort` |  | Hardware | done: `board.i2c_errors` |
| `timesincelastshot` | `double` |  | Other | done: `time_since_last_shot` (0 until the flight screen sets it from `VehicleState::shot_interval` over the camera feedback it keeps, as FlightData.cs:4021-4038 does) |
| `press_abs` | `float` |  | Sensor | done: `press_abs` |
| `press_temp` | `int` |  | Sensor | done: `press_temp` |
| `press_abs2` | `float` |  | Sensor | done: `press_abs2` |
| `press_temp2` | `int` |  | Sensor | done: `press_temp2` |
| `rateattitude` | `int` |  | Telem | done: `rates.attitude` (4 Hz, or the saved default when the vehicle was first seen; the link's stream requests and Planner's rate combos read and write it) |
| `rateposition` | `int` |  | Telem | done: `rates.position` |
| `ratestatus` | `int` |  | Telem | done: `rates.status` |
| `ratesensors` | `int` |  | Telem | done: `rates.sensors` |
| `raterc` | `int` |  | Telem | done: `rates.rc` |
| `datetime` | `DateTime` |  |  | done: `datetime` (each packet's time, stamped by `VehicleRegistry::apply_at`, which the link thread must call with `DateTime::now()`, or a replay with the log's time, in place of `apply`) |
| `connected` | `bool` |  |  | plumbing: whether the link is open, which the link knows (`mp_link`), not the vehicle |
| `campointa` | `float` |  | Mount | done: `mount.pointing_a` |
| `campointb` | `float` |  | Mount | done: `mount.pointing_b` |
| `campointc` | `float` |  | Mount | done: `mount.pointing_c` |
| `GimbalPoint` | `PointLatLngAlt` |  | Mount | done: `gimbal_point` (`None` until the flight screen writes what `GimbalPoint.ProjectPoint` projects, FlightData.cs:3964-3995; that projection is not ported) |
| `gimballat` | `float` |  | Mount | done: `gimbal_lat()` |
| `gimballng` | `float` |  | Mount | done: `gimbal_lng()` |
| `landed` | `bool` |  | Software | derived: `system_status` == MAV_STATE_STANDBY (3); `HIGH_LATENCY`'s landed state is not held |
| `safetyactive` | `bool` |  | Software | done: `sensors.safety_active()` |
| `terrainactive` | `bool` |  | Terrain | done: `sensors.terrain_active()` |
| `ter_curalt` | `float` | Terrain AGL | Terrain | done: `terrain.current_height` |
| `ter_alt` | `float` | Terrain GL | Terrain | done: `terrain.terrain_height` |
| `ter_load` | `float` |  | Terrain | done: `terrain.loaded` |
| `ter_pend` | `float` |  | Terrain | done: `terrain.pending` |
| `ter_space` | `float` |  | Terrain | done: `terrain.spacing` |
| `KIndex` | `int` |  | Enviromental | done: `VehicleState::kindex()` (`KIndexstatic`) |
| `opt_m_x` | `float` | flow_comp_m_x | Flow | done: `optical_flow.comp_m_x` |
| `opt_m_y` | `float` | flow_comp_m_y | Flow | done: `optical_flow.comp_m_y` |
| `opt_x` | `short` | flow_x | Flow | done: `optical_flow.x` |
| `opt_y` | `short` | flow_y | Flow | done: `optical_flow.y` |
| `opt_qua` | `byte` | flow quality | Flow | done: `optical_flow.quality` |
| `ekfstatus` | `float` |  | EKF | done: `ekf_status()` |
| `ekfflags` | `int` |  | EKF | done: `ekf.flags` |
| `ekfvelv` | `float` |  | EKF | done: `ekf.velocity_variance` |
| `ekfcompv` | `float` |  | EKF | done: `ekf.compass_variance` |
| `ekfposhor` | `float` |  | EKF | done: `ekf.position_horizontal_variance` |
| `ekfposvert` | `float` |  | EKF | done: `ekf.position_vertical_variance` |
| `ekfteralt` | `float` |  | EKF | done: `ekf.terrain_altitude_variance` |
| `pidff` | `float` |  | PID | done: `pid.ff` |
| `pidP` | `float` |  | PID | done: `pid.p` |
| `pidI` | `float` |  | PID | done: `pid.i` |
| `pidD` | `float` |  | PID | done: `pid.d` |
| `pidaxis` | `byte` |  | PID | done: `pid.axis` |
| `piddesired` | `float` |  | PID | done: `pid.desired` |
| `pidachieved` | `float` |  | PID | done: `pid.achieved` |
| `pidSRate` | `float` |  | PID | done: `pid.srate` |
| `pidPDmod` | `float` |  | PID | done: `pid.pdmod` |
| `pidSRateRoll` | `float` |  | PID | done: `pid.srate_roll` |
| `pidSRatePitch` | `float` |  | PID | done: `pid.srate_pitch` |
| `pidSRateYaw` | `float` |  | PID | done: `pid.srate_yaw` |
| `pidSRateAccZ` | `float` |  | PID | done: `pid.srate_accz` |
| `pidSRateSteer` | `float` |  | PID | done: `pid.srate_steer` |
| `pidSRateLanding` | `float` |  | PID | done: `pid.srate_landing` |
| `vibeclip0` | `uint` |  | Vibe | done: `vibration.clipping[0]` |
| `vibeclip1` | `uint` |  | Vibe | done: `vibration.clipping[1]` |
| `vibeclip2` | `uint` |  | Vibe | done: `vibration.clipping[2]` |
| `vibex` | `float` |  | Vibe | done: `vibration.x` |
| `vibey` | `float` |  | Vibe | done: `vibration.y` |
| `vibez` | `float` |  | Vibe | done: `vibration.z` |
| `version` | `Version` |  | Software | done: `autopilot_info.version` (major, minor, patch and type, where the C# holds a `Version`) |
| `uid` | `ulong` |  | Software | done: `autopilot_info.uid` |
| `uid2` | `string` |  | Software | derived: `autopilot_info.uid2`, the bytes the C# writes out as hex |
| `rpm1` | `float` |  | Sensor | done: `rpm[0]` |
| `rpm2` | `float` |  | Sensor | done: `rpm[1]` |
| `capabilities` | `uint` |  | Software | done: `autopilot_info.capabilities` |
| `speedup` | `float` |  | Software | done: `speedup` (`RAW_IMU`'s clock against `datetime`, where the C# uses the wall clock - the same live, the recorded time in a replay) |
| `vtol_state` | `byte` |  | Software | done: `vtol_state` |
| `landed_state` | `byte` |  | Software | done: `landed_state` |
| `gen_status` | `float` |  | Generator | done: `generator.status` |
| `gen_speed` | `float` |  | Generator | done: `generator.speed` |
| `gen_current` | `float` |  | Generator | done: `generator.current` |
| `gen_voltage` | `float` |  | Generator | done: `generator.voltage` |
| `gen_runtime` | `uint` |  | Generator | done: `generator.runtime` |
| `gen_maint_time` | `int` |  | Generator | done: `generator.maintenance_in` |
| `efi_baro` | `float` | EFI Baro Pressure (kPa) | EFI | done: `efi.baro` |
| `efi_headtemp` | `float` | EFI Head Temp (C) | EFI | done: `efi.head_temperature` |
| `efi_load` | `float` | EFI Load (%) | EFI | done: `efi.load` |
| `efi_health` | `byte` | EFI Health | EFI | done: `efi.health` |
| `efi_exhasttemp` | `float` | EFI Exhast Temp (C) | EFI | done: `efi.exhaust_temperature` |
| `efi_intaketemp` | `float` | EFI Intake Temp (C) | EFI | done: `efi.intake_temperature` |
| `efi_rpm` | `float` | EFI rpm | EFI | done: `efi.rpm` |
| `efi_fuelflow` | `float` | EFI Fuel Flow (cm3/min) | EFI | done: `efi.fuel_flow` |
| `efi_fuelconsumed` | `float` | EFI Fuel Consumed (cm3) | EFI | done: `efi.fuel_consumed` |
| `efi_fuelpressure` | `float` | EFI Fuel Pressure (kPa) | EFI | done: `efi.fuel_pressure` |
| `xpdr_es1090_tx_enabled` | `bool` | Transponder 1090ES Tx Enabled | Transponder Status | done: `transponder.es1090_tx_enabled` |
| `xpdr_mode_S_enabled` | `bool` | Transponder Mode S Reply Enabled | Transponder Status | done: `transponder.mode_s_enabled` |
| `xpdr_mode_C_enabled` | `bool` | Transponder Mode C Reply Enabled | Transponder Status | done: `transponder.mode_c_enabled` |
| `xpdr_mode_A_enabled` | `bool` | Transponder Mode A Reply Enabled | Transponder Status | done: `transponder.mode_a_enabled` |
| `xpdr_ident_active` | `bool` | Ident Active | Transponder Status | done: `transponder.ident_active` |
| `xpdr_x_bit_status` | `bool` | X-bit Status | Transponder Status | done: `transponder.x_bit_status` |
| `xpdr_interrogated_since_last` | `bool` | Interrogated since last | Transponder Status | done: `transponder.interrogated_since_last` |
| `xpdr_airborne_status` | `bool` | Airborne | Transponder Status | done: `transponder.airborne` |
| `xpdr_mode_A_squawk_code` | `ushort` | Transponder Mode A squawk code | Transponder Status | done: `transponder.squawk` |
| `xpdr_nic` | `byte` | NIC | Transponder Status | done: `transponder.nic` |
| `xpdr_nacp` | `byte` | NACp | Transponder Status | done: `transponder.nacp` |
| `xpdr_board_temperature` | `byte` | Board Temperature in C | Transponder Status | done: `transponder.board_temperature` |
| `xpdr_maint_req` | `bool` | Maintainence Required | Transponder Status | done: `transponder.maintenance_required` |
| `xpdr_adsb_tx_sys_fail` | `bool` | ADSB Tx System Failure | Transponder Status | done: `transponder.adsb_tx_failure` |
| `xpdr_gps_unavail` | `bool` | GPS Unavailable | Transponder Status | done: `transponder.gps_unavailable` |
| `xpdr_gps_no_fix` | `bool` | GPS No Fix | Transponder Status | done: `transponder.gps_no_fix` |
| `xpdr_status_unavail` | `bool` | Ping200X No Status Message Recieved | Transponder Status | done: `transponder.status_unavailable` |
| `xpdr_status_pending` | `bool` | Status Update Pending | Transponder Status | done: `transponder.status_pending` (the C#'s display clears it once shown; a snapshot reader remembers what it has shown) |
| `xpdr_flight_id` | `byte[]` | Callsign/Flight ID | Transponder Status | done: `transponder.flight_id` |
| `ahrs2_roll` | `float` |  | AHRS2 | done: `ahrs2.roll` |
| `ahrs2_pitch` | `float` |  | AHRS2 | done: `ahrs2.pitch` |
| `ahrs2_yaw` | `float` |  | AHRS2 | done: `ahrs2.yaw` |
| `ahrs2_alt` | `float` |  | AHRS2 | done: `ahrs2.altitude` |
| `ahrs2_lat` | `double` |  | AHRS2 | done: `ahrs2.lat` |
| `ahrs2_lng` | `double` |  | AHRS2 | done: `ahrs2.lng` |
| `mcumaxvolt` | `float` | mcu max voltage | MCU | done: `mcu.voltage_max` |
| `mcuminvolt` | `float` | mcu min voltage | MCU | done: `mcu.voltage_min` |
| `mcuvoltage` | `float` | mcu voltage | MCU | done: `mcu.voltage` |
| `mcutemp` | `float` | mcu temperature | MCU | done: `mcu.temperature` |
| `fenceb_count` | `ushort` | Breach count | Fence | done: `fence_breach.count` |
| `fenceb_status` | `byte` | Breach status | Fence | done: `fence_breach.status` |
| `fenceb_type` | `byte` | Breach type | Fence | done: `fence_breach.breach_type` |
| `posn` | `float` | North | Position | done: `local_position[0]` |
| `pose` | `float` | East | Position | done: `local_position[1]` |
| `posd` | `float` | Down | Position | done: `local_position[2]` |
| `csCallBack` | `event EventHandler` |  |  | plumbing: an event raised on every update for the WinForms bindings; readers take snapshots from `StateHandle` instead |
