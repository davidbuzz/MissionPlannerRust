//! What Mission Planner's `CurrentState` holds, and what stands for each of it here
//! (DELIVERABLES.md D5).
//!
//! `ExtLibs/ArduPilot/CurrentState.cs` is the object the whole C# application reads the vehicle
//! through: the HUD binds to it, the quick view and the tuning graph offer its properties by
//! name, speech and scripts format them. D5's definition of done is "every C# `CurrentState`
//! field accounted for", and [`CURRENTSTATE`] is that account - one row per public property or
//! field, in the file's order, with the `[DisplayText]` and `[GroupText]` the quick view's field
//! chooser shows for it and what this crate has in its place. The report it renders lives at
//! `docs/coverage/currentstate.md`, and a test keeps it current.
//!
//! The tests hold the table to the truth. When the C# tree is present the class is parsed and
//! the table must list exactly its public members, with the same types and attribute text, in
//! the same order - so an upstream change or a typo fails; the parser itself is tested on every
//! shape of declaration the class uses. Every row that claims a field names a path on
//! [`crate::VehicleState`] (or on another type of this crate), and each step of that path must
//! exist in this crate's source. The committed report must match what the table renders, and
//! the counts must match the ones recorded, so a change in either direction is deliberate.
//!
//! "Done" means the same value in the same units. The C# converts distances, altitudes and
//! speeds to the user's display units in each property's getter; the value underneath is SI, and
//! that is what a row compares - see [`crate::state`] on units.

use std::fmt::Write as _;

/// What stands in for a C# `CurrentState` property here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ours {
    /// Held here, the same value in the same units: a path on a [`crate::VehicleState`] -
    /// `battery.voltage`, `batteries[0].current`, `ekf_status()` - or on another type of this
    /// crate named first, `DisplayUnits::dist`; optionally followed by a note.
    Done(&'static str),
    /// Not held, but computable from what is: how.
    Derived(&'static str),
    /// Not ported yet.
    Missing,
    /// Mechanics of the C# rather than vehicle state - an event, a back-reference, a service
    /// handle - or state that another part of this port owns; the reason says which.
    Plumbing(&'static str),
    /// Deliberately not carried over, with the reason.
    Dropped(&'static str),
}

/// One public property or field of `CurrentState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Field {
    /// The C# name.
    pub name: &'static str,
    /// The C# type, with `static` or `event` in front where the declaration has it.
    pub ty: &'static str,
    /// The `[DisplayText]` the quick view's field chooser shows for it, or empty.
    pub display: &'static str,
    /// The `[GroupText]` the chooser files it under, or empty.
    pub group: &'static str,
    /// What this crate has for it.
    pub ours: Ours,
}

const fn row(
    name: &'static str,
    ty: &'static str,
    display: &'static str,
    group: &'static str,
    ours: Ours,
) -> Field {
    Field {
        name,
        ty,
        display,
        group,
        ours,
    }
}

use Ours::{Derived, Done, Dropped, Missing, Plumbing};

/// Every public property and field of `ExtLibs/ArduPilot/CurrentState.cs`, in the file's order.
pub const CURRENTSTATE: &[Field] = &[
    row(
        "Speech",
        "static ISpeech",
        "",
        "",
        Plumbing(
            "the speech engine the C# hangs on this class, a service rather than vehicle state",
        ),
    ),
    row(
        "multiplierdist",
        "static float",
        "",
        "",
        Done("DisplayUnits::dist"),
    ),
    row(
        "DistanceUnit",
        "static string",
        "",
        "",
        Done("DisplayUnits::dist_unit"),
    ),
    row(
        "multiplierspeed",
        "static float",
        "",
        "",
        Done("DisplayUnits::speed"),
    ),
    row(
        "SpeedUnit",
        "static string",
        "",
        "",
        Done("DisplayUnits::speed_unit"),
    ),
    row(
        "multiplieralt",
        "static float",
        "",
        "",
        Done("DisplayUnits::alt"),
    ),
    row(
        "AltUnit",
        "static string",
        "",
        "",
        Done("DisplayUnits::alt_unit"),
    ),
    row(
        "rateattitudebackup",
        "static int",
        "",
        "",
        Plumbing(
            "the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from",
        ),
    ),
    row(
        "ratepositionbackup",
        "static int",
        "",
        "",
        Plumbing(
            "the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from",
        ),
    ),
    row(
        "ratestatusbackup",
        "static int",
        "",
        "",
        Plumbing(
            "the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from",
        ),
    ),
    row(
        "ratesensorsbackup",
        "static int",
        "",
        "",
        Plumbing(
            "the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from",
        ),
    ),
    row(
        "ratercbackup",
        "static int",
        "",
        "",
        Plumbing(
            "the saved default that `ResetInternals` copies into the matching `rate*` property: `StreamRates::backups`, which a newly seen vehicle starts from",
        ),
    ),
    row(
        "KIndexstatic",
        "static int",
        "",
        "",
        Done(
            "VehicleState::kindex() (process-wide, -1 until `VehicleState::set_kindex`, which start-up must call with the `kindex` setting and the K-index download, as MainV2.cs:3940-3981 does)",
        ),
    ),
    row(
        "firmware",
        "Firmwares",
        "",
        "",
        Derived(
            "from `autopilot` and `vehicle_type` as MAVLinkInterface.cs:6700-6815 does, `VehicleFamily::from_mav_type` for ArduPilot; the version-string lookup it tries first reads the firmware's `STATUSTEXT` banner (MAVLinkInterface.cs:1827-1830), which is not held here",
        ),
    ),
    row(
        "hilch1",
        "int",
        "",
        "",
        Done("hil_channels[0] (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' roll times 10000)"),
    ),
    row(
        "hilch2",
        "int",
        "",
        "",
        Done("hil_channels[1] (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' pitch times 10000)"),
    ),
    row(
        "hilch3",
        "int",
        "",
        "",
        Done("hil_channels[2] (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' throttle times 10000)"),
    ),
    row(
        "hilch4",
        "int",
        "",
        "",
        Done("hil_channels[3] (`RC_CHANNELS_SCALED`, or `HIL_CONTROLS`' yaw times 10000)"),
    ),
    row(
        "hilch5",
        "int",
        "",
        "",
        Done("hil_channels[4] (`RC_CHANNELS_SCALED`)"),
    ),
    row(
        "hilch6",
        "int",
        "",
        "",
        Done("hil_channels[5] (`RC_CHANNELS_SCALED`)"),
    ),
    row(
        "hilch7",
        "int",
        "",
        "",
        Done("hil_channels[6] (`RC_CHANNELS_SCALED`)"),
    ),
    row(
        "hilch8",
        "int",
        "",
        "",
        Done("hil_channels[7] (`RC_CHANNELS_SCALED`)"),
    ),
    row(
        "lastautowp",
        "int",
        "",
        "",
        Done("last_auto_wp (`None` is the C#'s -1)"),
    ),
    row(
        "parent",
        "MAVState",
        "",
        "",
        Plumbing(
            "the back-reference to the owning `MAVState`; `VehicleRegistry` holds that relationship",
        ),
    ),
    row(
        "rcoverridech1",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech10",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech11",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech12",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech13",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech14",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech15",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech16",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech17",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech18",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech2",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech3",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech4",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech5",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech6",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech7",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech8",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "rcoverridech9",
        "short",
        "",
        "",
        Plumbing(
            "the RC override the joystick sends: the GCS's output, not vehicle state (`mp_link::commands::rc_override`)",
        ),
    ),
    row(
        "sensors_enabled",
        "Mavlink_Sensors",
        "",
        "",
        Done("sensors.enabled"),
    ),
    row(
        "sensors_health",
        "Mavlink_Sensors",
        "",
        "",
        Done("sensors.health"),
    ),
    row(
        "sensors_present",
        "Mavlink_Sensors",
        "",
        "",
        Done("sensors.present"),
    ),
    row(
        "prearmstatus",
        "bool",
        "",
        "",
        Derived(
            "connected and (`sensors.health` or not `sensors.enabled`) at the pre-arm bit, 0x10000000",
        ),
    ),
    row(
        "custom_field_names",
        "static Dictionary<string, string>",
        "",
        "",
        Plumbing("the static name-to-slot map behind `customfield0` to `customfield19`"),
    ),
    row(
        "customfield0",
        "float",
        "",
        "",
        Done(
            "custom_fields[0] (a `NAMED_VALUE_FLOAT`, in the field its name claimed; the names, the C#'s static `custom_field_names`, are `VehicleState::custom_field_name`, and the `customfield<n>` settings start-up adds are `VehicleState::add_custom_field_name`)",
        ),
    ),
    row("customfield1", "float", "", "", Done("custom_fields[1]")),
    row("customfield2", "float", "", "", Done("custom_fields[2]")),
    row("customfield3", "float", "", "", Done("custom_fields[3]")),
    row("customfield4", "float", "", "", Done("custom_fields[4]")),
    row("customfield5", "float", "", "", Done("custom_fields[5]")),
    row("customfield6", "float", "", "", Done("custom_fields[6]")),
    row("customfield7", "float", "", "", Done("custom_fields[7]")),
    row("customfield8", "float", "", "", Done("custom_fields[8]")),
    row("customfield9", "float", "", "", Done("custom_fields[9]")),
    row("customfield10", "float", "", "", Done("custom_fields[10]")),
    row("customfield11", "float", "", "", Done("custom_fields[11]")),
    row("customfield12", "float", "", "", Done("custom_fields[12]")),
    row("customfield13", "float", "", "", Done("custom_fields[13]")),
    row("customfield14", "float", "", "", Done("custom_fields[14]")),
    row("customfield15", "float", "", "", Done("custom_fields[15]")),
    row("customfield16", "float", "", "", Done("custom_fields[16]")),
    row("customfield17", "float", "", "", Done("custom_fields[17]")),
    row("customfield18", "float", "", "", Done("custom_fields[18]")),
    row("customfield19", "float", "", "", Done("custom_fields[19]")),
    row(
        "roll",
        "float",
        "Roll (deg)",
        "Attitude",
        Derived("`attitude.roll`, in radians where the C# holds degrees"),
    ),
    row(
        "pitch",
        "float",
        "Pitch (deg)",
        "Attitude",
        Derived("`attitude.pitch`, in radians where the C# holds degrees"),
    ),
    row(
        "yaw",
        "float",
        "Yaw (deg)",
        "Attitude",
        Derived("`attitude.yaw`, in radians where the C# holds degrees from 0 to 360"),
    ),
    row("SSA", "float", "SSA (deg)", "Attitude", Done("ssa")),
    row("AOA", "float", "AOA (deg)", "Attitude", Done("aoa")),
    row(
        "groundcourse",
        "float",
        "GroundCourse (deg)",
        "Position",
        Done("gps.course"),
    ),
    row(
        "lat",
        "double",
        "Latitude (dd)",
        "Position",
        Done("position (`None` is the C#'s 0, 0)"),
    ),
    row(
        "lng",
        "double",
        "Longitude (dd)",
        "Position",
        Done("position (`None` is the C#'s 0, 0)"),
    ),
    row(
        "alt",
        "float",
        "Altitude (alt)",
        "Position",
        Done(
            "altitude_relative (the C# subtracts the user's `altoffsethome`, 0 unless set: `alt()` is that difference, in the C#'s single precision)",
        ),
    ),
    row(
        "altasl",
        "float",
        "Altitude (alt)",
        "Position",
        Done("altitude_msl"),
    ),
    row(
        "horizondist",
        "float",
        "Horizon Dist (dist)",
        "Position",
        Derived("3570 * sqrt(`altitude_relative`), metres"),
    ),
    row(
        "vx",
        "double",
        "Velocity X (ms)",
        "Position",
        Done("velocity_north"),
    ),
    row(
        "vy",
        "double",
        "Velocity Y (ms)",
        "Position",
        Done("velocity_east"),
    ),
    row(
        "vz",
        "double",
        "Velocity Z (ms)",
        "Position",
        Done("velocity_down"),
    ),
    row(
        "vlen",
        "double",
        "",
        "Position",
        Derived("the length of (`velocity_north`, `velocity_east`, `velocity_down`)"),
    ),
    row(
        "altoffsethome",
        "float",
        "Alt Home Offset (dist)",
        "Position",
        Done(
            "alt_offset_home (0 until the flight screen's Home Alt button writes it, FlightData.cs:1236-1247)",
        ),
    ),
    row(
        "gpsstatus",
        "float",
        "Gps Status",
        "Position",
        Done("gps.fix_type"),
    ),
    row("gpshdop", "float", "Gps HDOP", "Position", Done("gps.hdop")),
    row(
        "satcount",
        "float",
        "Sat Count",
        "Position",
        Done("gps.satellites_visible"),
    ),
    row(
        "gpsh_acc",
        "float",
        "H Acc (m)",
        "Position",
        Done("gps.h_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpsv_acc",
        "float",
        "V Acc (m)",
        "Position",
        Done("gps.v_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpsvel_acc",
        "float",
        "Velocity Accuracy",
        "Position",
        Done("gps.vel_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpshdg_acc",
        "float",
        "Heading Accuracy",
        "Position",
        Done("gps.hdg_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpsyaw",
        "float",
        "GPS Yaw (deg)",
        "Position",
        Done("gps.yaw (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "lat2",
        "double",
        "Latitude2 (dd)",
        "Position",
        Done("gps2.position (`None` is the C#'s 0, 0)"),
    ),
    row(
        "lng2",
        "double",
        "Longitude2 (dd)",
        "Position",
        Done("gps2.position (`None` is the C#'s 0, 0)"),
    ),
    row(
        "altasl2",
        "float",
        "Altitude2 (dist)",
        "Position",
        Done("gps2.altitude_msl"),
    ),
    row(
        "gpsstatus2",
        "float",
        "Gps Status2",
        "Position",
        Done("gps2.fix_type"),
    ),
    row(
        "gpshdop2",
        "float",
        "Gps HDOP2",
        "Position",
        Done("gps2.hdop"),
    ),
    row(
        "satcount2",
        "float",
        "Sat Count2",
        "Position",
        Done("gps2.satellites_visible"),
    ),
    row(
        "groundspeed2",
        "float",
        "",
        "Position",
        Done("gps2.ground_speed"),
    ),
    row(
        "groundcourse2",
        "float",
        "GroundCourse2 (deg)",
        "Position",
        Done("gps2.course"),
    ),
    row(
        "gpsh_acc2",
        "float",
        "H Acc2 (m)",
        "Position",
        Done("gps2.h_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpsv_acc2",
        "float",
        "V Acc2 (m)",
        "Position",
        Done("gps2.v_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpsvel_acc2",
        "float",
        "Velocity Accuracy",
        "Position",
        Done("gps2.vel_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpshdg_acc2",
        "float",
        "Heading Accuracy",
        "Position",
        Done("gps2.hdg_acc (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "gpsyaw2",
        "float",
        "GPS Yaw (deg)",
        "Position",
        Done("gps2.yaw (a MAVLink 1 frame reads 0 where the C# says -1)"),
    ),
    row(
        "satcountB",
        "float",
        "Sat Count Blend",
        "Position",
        Derived("`gps.satellites_visible` + `gps2.satellites_visible`"),
    ),
    row(
        "gpstime",
        "DateTime",
        "",
        "Position",
        Done("gps_time_unix_ms (milliseconds since 1970, where the C# holds a `DateTime`)"),
    ),
    row(
        "altd1000",
        "float",
        "",
        "Other",
        Derived("`altitude_relative` / 1000 % 10, the thousands digit"),
    ),
    row(
        "altd100",
        "float",
        "",
        "Other",
        Derived("`altitude_relative` / 100 % 10, the hundreds digit"),
    ),
    row(
        "airspeed",
        "float",
        "AirSpeed (speed)",
        "Sensor",
        Done("air_speed"),
    ),
    row(
        "targetairspeed",
        "float",
        "Airspeed Target (speed)",
        "NAV",
        Derived(
            "`target_airspeed()`, without the C#'s low-pass and its division of the error by 100",
        ),
    ),
    row(
        "lowairspeed",
        "bool",
        "",
        "",
        Done(
            "low_airspeed (from `VFR_HUD`; the `AIRSPEED_MIN` or `ARSPD_FBW_MIN` parameter the C# reads itself comes from `set_airspeed_min_params()`, which the parameter table's owner must call)",
        ),
    ),
    row(
        "asratio",
        "float",
        "Airspeed Ratio",
        "Calibration",
        Done("airspeed_ratio"),
    ),
    row(
        "airspeed1_temp",
        "float",
        "Airspeed1 Temperature",
        "Sensor",
        Done("airspeed1_temp"),
    ),
    row(
        "airspeed2_temp",
        "float",
        "Airspeed2 Temperature",
        "Sensor",
        Done("airspeed2_temp"),
    ),
    row(
        "groundspeed",
        "float",
        "GroundSpeed (speed)",
        "Position",
        Done("ground_speed"),
    ),
    row("ax", "float", "Accel X", "Sensor", Done("imu[0].accel[0]")),
    row("ay", "float", "Accel Y", "Sensor", Done("imu[0].accel[1]")),
    row("az", "float", "Accel Z", "Sensor", Done("imu[0].accel[2]")),
    row(
        "accelsq",
        "float",
        "Accel Strength",
        "Sensor",
        Derived("the length of `imu[0].accel`, / 1000"),
    ),
    row("gx", "float", "Gyro X", "Sensor", Done("imu[0].gyro[0]")),
    row("gy", "float", "Gyro Y", "Sensor", Done("imu[0].gyro[1]")),
    row("gz", "float", "Gyro Z", "Sensor", Done("imu[0].gyro[2]")),
    row(
        "gyrosq",
        "float",
        "Gyro Strength",
        "Sensor",
        Derived("the length of `imu[0].gyro`"),
    ),
    row("mx", "float", "Mag X", "Sensor", Done("imu[0].mag[0]")),
    row("my", "float", "Mag Y", "Sensor", Done("imu[0].mag[1]")),
    row("mz", "float", "Mag Z", "Sensor", Done("imu[0].mag[2]")),
    row(
        "magfield",
        "float",
        "Mag Field",
        "Sensor",
        Derived("the length of `imu[0].mag`"),
    ),
    row(
        "imu1_temp",
        "float",
        "IMU1 Temperature",
        "Sensor",
        Done("imu[0].temperature"),
    ),
    row(
        "ax2",
        "float",
        "Accel2 X",
        "Sensor",
        Done("imu[1].accel[0]"),
    ),
    row(
        "ay2",
        "float",
        "Accel2 Y",
        "Sensor",
        Done("imu[1].accel[1]"),
    ),
    row(
        "az2",
        "float",
        "Accel2 Z",
        "Sensor",
        Done("imu[1].accel[2]"),
    ),
    row(
        "accelsq2",
        "float",
        "Accel2 Strength",
        "Sensor",
        Derived("the length of `imu[1].accel`, / 1000"),
    ),
    row("gx2", "float", "Gyro2 X", "Sensor", Done("imu[1].gyro[0]")),
    row("gy2", "float", "Gyro2 Y", "Sensor", Done("imu[1].gyro[1]")),
    row("gz2", "float", "Gyro2 Z", "Sensor", Done("imu[1].gyro[2]")),
    row(
        "gyrosq2",
        "float",
        "Gyro2 Strength",
        "Sensor",
        Derived("the length of `imu[1].gyro`"),
    ),
    row("mx2", "float", "Mag2 X", "Sensor", Done("imu[1].mag[0]")),
    row("my2", "float", "Mag2 Y", "Sensor", Done("imu[1].mag[1]")),
    row("mz2", "float", "Mag2 Z", "Sensor", Done("imu[1].mag[2]")),
    row(
        "magfield2",
        "float",
        "Mag2 Field",
        "Sensor",
        Derived("the length of `imu[1].mag`"),
    ),
    row(
        "imu2_temp",
        "float",
        "IMU2 Temperature",
        "Sensor",
        Done("imu[1].temperature"),
    ),
    row(
        "ax3",
        "float",
        "Accel3 X",
        "Sensor",
        Done("imu[2].accel[0]"),
    ),
    row(
        "ay3",
        "float",
        "Accel3 Y",
        "Sensor",
        Done("imu[2].accel[1]"),
    ),
    row(
        "az3",
        "float",
        "Accel3 Z",
        "Sensor",
        Done("imu[2].accel[2]"),
    ),
    row(
        "accelsq3",
        "float",
        "Accel3 Strength",
        "Sensor",
        Derived("the length of `imu[2].accel`, / 1000"),
    ),
    row("gx3", "float", "Gyro3 X", "Sensor", Done("imu[2].gyro[0]")),
    row("gy3", "float", "Gyro3 Y", "Sensor", Done("imu[2].gyro[1]")),
    row("gz3", "float", "Gyro3 Z", "Sensor", Done("imu[2].gyro[2]")),
    row(
        "gyrosq3",
        "float",
        "Gyro3 Strength",
        "Sensor",
        Derived("the length of `imu[2].gyro`"),
    ),
    row("mx3", "float", "Mag3 X", "Sensor", Done("imu[2].mag[0]")),
    row("my3", "float", "Mag3 Y", "Sensor", Done("imu[2].mag[1]")),
    row("mz3", "float", "Mag3 Z", "Sensor", Done("imu[2].mag[2]")),
    row(
        "magfield3",
        "float",
        "Mag3 Field",
        "Sensor",
        Derived("the length of `imu[2].mag`"),
    ),
    row(
        "imu3_temp",
        "float",
        "IMU3 Temperature",
        "Sensor",
        Done("imu[2].temperature"),
    ),
    row(
        "hygrotemp1",
        "short",
        "hygrotemp1 (cdegC)",
        "Sensor",
        Done("hygrometers[0].temperature"),
    ),
    row(
        "hygrohumi1",
        "ushort",
        "hygrohumi1 (c%)",
        "Sensor",
        Done("hygrometers[0].humidity"),
    ),
    row(
        "hygrotemp2",
        "short",
        "hygrotemp2 (cdegC)",
        "Sensor",
        Done("hygrometers[1].temperature"),
    ),
    row(
        "hygrohumi2",
        "ushort",
        "hygrohumi2 (c%)",
        "Sensor",
        Done("hygrometers[1].humidity"),
    ),
    row("ch1in", "float", "", "RadioIn", Done("rc.values[0]")),
    row("ch2in", "float", "", "RadioIn", Done("rc.values[1]")),
    row("ch3in", "float", "", "RadioIn", Done("rc.values[2]")),
    row("ch4in", "float", "", "RadioIn", Done("rc.values[3]")),
    row("ch5in", "float", "", "RadioIn", Done("rc.values[4]")),
    row("ch6in", "float", "", "RadioIn", Done("rc.values[5]")),
    row("ch7in", "float", "", "RadioIn", Done("rc.values[6]")),
    row("ch8in", "float", "", "RadioIn", Done("rc.values[7]")),
    row("ch9in", "float", "", "RadioIn", Done("rc.values[8]")),
    row("ch10in", "float", "", "RadioIn", Done("rc.values[9]")),
    row("ch11in", "float", "", "RadioIn", Done("rc.values[10]")),
    row("ch12in", "float", "", "RadioIn", Done("rc.values[11]")),
    row("ch13in", "float", "", "RadioIn", Done("rc.values[12]")),
    row("ch14in", "float", "", "RadioIn", Done("rc.values[13]")),
    row("ch15in", "float", "", "RadioIn", Done("rc.values[14]")),
    row("ch16in", "float", "", "RadioIn", Done("rc.values[15]")),
    row("ch1out", "float", "", "RadioOut", Done("servo_outputs[0]")),
    row("ch2out", "float", "", "RadioOut", Done("servo_outputs[1]")),
    row("ch3out", "float", "", "RadioOut", Done("servo_outputs[2]")),
    row("ch4out", "float", "", "RadioOut", Done("servo_outputs[3]")),
    row("ch5out", "float", "", "RadioOut", Done("servo_outputs[4]")),
    row("ch6out", "float", "", "RadioOut", Done("servo_outputs[5]")),
    row("ch7out", "float", "", "RadioOut", Done("servo_outputs[6]")),
    row("ch8out", "float", "", "RadioOut", Done("servo_outputs[7]")),
    row("ch9out", "float", "", "RadioOut", Done("servo_outputs[8]")),
    row("ch10out", "float", "", "RadioOut", Done("servo_outputs[9]")),
    row(
        "ch11out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[10]"),
    ),
    row(
        "ch12out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[11]"),
    ),
    row(
        "ch13out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[12]"),
    ),
    row(
        "ch14out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[13]"),
    ),
    row(
        "ch15out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[14]"),
    ),
    row(
        "ch16out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[15]"),
    ),
    row(
        "ch17out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[16]"),
    ),
    row(
        "ch18out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[17]"),
    ),
    row(
        "ch19out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[18]"),
    ),
    row(
        "ch20out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[19]"),
    ),
    row(
        "ch21out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[20]"),
    ),
    row(
        "ch22out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[21]"),
    ),
    row(
        "ch23out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[22]"),
    ),
    row(
        "ch24out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[23]"),
    ),
    row(
        "ch25out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[24]"),
    ),
    row(
        "ch26out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[25]"),
    ),
    row(
        "ch27out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[26]"),
    ),
    row(
        "ch28out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[27]"),
    ),
    row(
        "ch29out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[28]"),
    ),
    row(
        "ch30out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[29]"),
    ),
    row(
        "ch31out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[30]"),
    ),
    row(
        "ch32out",
        "float",
        "",
        "RadioOut",
        Done("servo_outputs[31]"),
    ),
    row("esc1_volt", "float", "", "ESC", Done("escs[0].voltage")),
    row("esc1_curr", "float", "", "ESC", Done("escs[0].current")),
    row("esc1_rpm", "float", "", "ESC", Done("escs[0].rpm")),
    row("esc1_temp", "float", "", "ESC", Done("escs[0].temperature")),
    row("esc2_volt", "float", "", "ESC", Done("escs[1].voltage")),
    row("esc2_curr", "float", "", "ESC", Done("escs[1].current")),
    row("esc2_rpm", "float", "", "ESC", Done("escs[1].rpm")),
    row("esc2_temp", "float", "", "ESC", Done("escs[1].temperature")),
    row("esc3_volt", "float", "", "ESC", Done("escs[2].voltage")),
    row("esc3_curr", "float", "", "ESC", Done("escs[2].current")),
    row("esc3_rpm", "float", "", "ESC", Done("escs[2].rpm")),
    row("esc3_temp", "float", "", "ESC", Done("escs[2].temperature")),
    row("esc4_volt", "float", "", "ESC", Done("escs[3].voltage")),
    row("esc4_curr", "float", "", "ESC", Done("escs[3].current")),
    row("esc4_rpm", "float", "", "ESC", Done("escs[3].rpm")),
    row("esc4_temp", "float", "", "ESC", Done("escs[3].temperature")),
    row("esc5_volt", "float", "", "ESC", Done("escs[4].voltage")),
    row("esc5_curr", "float", "", "ESC", Done("escs[4].current")),
    row("esc5_rpm", "float", "", "ESC", Done("escs[4].rpm")),
    row("esc5_temp", "float", "", "ESC", Done("escs[4].temperature")),
    row("esc6_volt", "float", "", "ESC", Done("escs[5].voltage")),
    row("esc6_curr", "float", "", "ESC", Done("escs[5].current")),
    row("esc6_rpm", "float", "", "ESC", Done("escs[5].rpm")),
    row("esc6_temp", "float", "", "ESC", Done("escs[5].temperature")),
    row("esc7_volt", "float", "", "ESC", Done("escs[6].voltage")),
    row("esc7_curr", "float", "", "ESC", Done("escs[6].current")),
    row("esc7_rpm", "float", "", "ESC", Done("escs[6].rpm")),
    row("esc7_temp", "float", "", "ESC", Done("escs[6].temperature")),
    row("esc8_volt", "float", "", "ESC", Done("escs[7].voltage")),
    row("esc8_curr", "float", "", "ESC", Done("escs[7].current")),
    row("esc8_rpm", "float", "", "ESC", Done("escs[7].rpm")),
    row("esc8_temp", "float", "", "ESC", Done("escs[7].temperature")),
    row("esc9_volt", "float", "", "ESC", Done("escs[8].voltage")),
    row("esc9_curr", "float", "", "ESC", Done("escs[8].current")),
    row("esc9_rpm", "float", "", "ESC", Done("escs[8].rpm")),
    row("esc9_temp", "float", "", "ESC", Done("escs[8].temperature")),
    row("esc10_volt", "float", "", "ESC", Done("escs[9].voltage")),
    row("esc10_curr", "float", "", "ESC", Done("escs[9].current")),
    row("esc10_rpm", "float", "", "ESC", Done("escs[9].rpm")),
    row(
        "esc10_temp",
        "float",
        "",
        "ESC",
        Done("escs[9].temperature"),
    ),
    row("esc11_volt", "float", "", "ESC", Done("escs[10].voltage")),
    row("esc11_curr", "float", "", "ESC", Done("escs[10].current")),
    row("esc11_rpm", "float", "", "ESC", Done("escs[10].rpm")),
    row(
        "esc11_temp",
        "float",
        "",
        "ESC",
        Done("escs[10].temperature"),
    ),
    row("esc12_volt", "float", "", "ESC", Done("escs[11].voltage")),
    row("esc12_curr", "float", "", "ESC", Done("escs[11].current")),
    row("esc12_rpm", "float", "", "ESC", Done("escs[11].rpm")),
    row(
        "esc12_temp",
        "float",
        "",
        "ESC",
        Done("escs[11].temperature"),
    ),
    row("esc13_volt", "float", "", "ESC", Done("escs[12].voltage")),
    row("esc13_curr", "float", "", "ESC", Done("escs[12].current")),
    row("esc13_rpm", "float", "", "ESC", Done("escs[12].rpm")),
    row(
        "esc13_temp",
        "float",
        "",
        "ESC",
        Done("escs[12].temperature"),
    ),
    row("esc14_volt", "float", "", "ESC", Done("escs[13].voltage")),
    row("esc14_curr", "float", "", "ESC", Done("escs[13].current")),
    row("esc14_rpm", "float", "", "ESC", Done("escs[13].rpm")),
    row(
        "esc14_temp",
        "float",
        "",
        "ESC",
        Done("escs[13].temperature"),
    ),
    row("esc15_volt", "float", "", "ESC", Done("escs[14].voltage")),
    row("esc15_curr", "float", "", "ESC", Done("escs[14].current")),
    row("esc15_rpm", "float", "", "ESC", Done("escs[14].rpm")),
    row(
        "esc15_temp",
        "float",
        "",
        "ESC",
        Done("escs[14].temperature"),
    ),
    row("esc16_volt", "float", "", "ESC", Done("escs[15].voltage")),
    row("esc16_curr", "float", "", "ESC", Done("escs[15].current")),
    row("esc16_rpm", "float", "", "ESC", Done("escs[15].rpm")),
    row(
        "esc16_temp",
        "float",
        "",
        "ESC",
        Done("escs[15].temperature"),
    ),
    row(
        "ch1percent",
        "float",
        "",
        "RadioOut",
        Derived(
            "`servo_outputs[0]` scaled by the SERVO1_MIN, _TRIM, _MAX and _REVERSED parameters, or by 1000-1500-2000 without them",
        ),
    ),
    row(
        "ch3percent",
        "float",
        "",
        "RadioOut",
        Done("throttle_percent"),
    ),
    row(
        "failsafe",
        "bool",
        "Failsafe",
        "Software",
        Derived(
            "`system_status` == MAV_STATE_CRITICAL (5), as the HUD derives it; `HIGH_LATENCY`'s failsafe byte is not held",
        ),
    ),
    row(
        "rxrssi",
        "int",
        "RX Rssi",
        "Telem",
        Derived(
            "`rc.rssi` * 100 / 254, 0 when 255 (`RC_CHANNELS`); the C# divides by 255 after `RC_CHANNELS_RAW`",
        ),
    ),
    row(
        "crit_AOA",
        "float",
        "Crit AOA (deg)",
        "Attitude",
        Derived("the AOA_CRIT parameter, 25 without it"),
    ),
    row(
        "lowgroundspeed",
        "bool",
        "",
        "",
        Dropped(
            "nothing in the C# sets it, so it is always false; the HUD's low-ground-speed warning is its own",
        ),
    ),
    row(
        "verticalspeed",
        "float",
        "Vertical Speed (speed)",
        "Position",
        Done(
            "vertical_speed() (the `alt` setter's filtered rate from `GLOBAL_POSITION_INT` and the high-latency messages, against `datetime`)",
        ),
    ),
    row(
        "verticalspeed_fpm",
        "double",
        "Vertical Speed (fpm)",
        "Position",
        Derived("`velocity_down` * -3.28084 * 60"),
    ),
    row(
        "glide_ratio",
        "double",
        "Glide Ratio",
        "Position",
        Derived(
            "the horizontal length of (`velocity_north`, `velocity_east`) over `velocity_down`",
        ),
    ),
    row(
        "nav_roll",
        "float",
        "Roll Target (deg)",
        "NAV",
        Done("nav.roll"),
    ),
    row(
        "nav_pitch",
        "float",
        "Pitch Target (deg)",
        "NAV",
        Done("nav.pitch"),
    ),
    row(
        "nav_bearing",
        "float",
        "Bearing Target (deg)",
        "NAV",
        Done("nav.bearing"),
    ),
    row(
        "target_bearing",
        "float",
        "Bearing Target (deg)",
        "NAV",
        Done("nav.target_bearing"),
    ),
    row(
        "wp_dist",
        "float",
        "Dist to WP (dist)",
        "NAV",
        Done("nav.wp_distance"),
    ),
    row(
        "alt_error",
        "float",
        "Altitude Error (dist)",
        "NAV",
        Done("nav.alt_error"),
    ),
    row(
        "ber_error",
        "float",
        "Bearing Error (deg)",
        "NAV",
        Derived("`nav.target_bearing` - `attitude.yaw` in degrees"),
    ),
    row(
        "aspd_error",
        "float",
        "Airspeed Error (speed)",
        "NAV",
        Derived(
            "`nav.airspeed_error` / 100: the C# divides the wire's m/s by 100, which this deliberately does not",
        ),
    ),
    row(
        "xtrack_error",
        "float",
        "Xtrack Error (m)",
        "NAV",
        Done("nav.xtrack_error"),
    ),
    row("wpno", "float", "WP No", "NAV", Done("mission_current")),
    row(
        "mode",
        "string",
        "Mode",
        "NAV",
        Derived("`flight_mode_name(vehicle_type, custom_mode)`"),
    ),
    row(
        "climbrate",
        "float",
        "ClimbRate (speed)",
        "Position",
        Done(
            "climb_rate (from `VFR_HUD`; until the first, the `alt` setter's unfiltered rate against `datetime`, as the C#'s)",
        ),
    ),
    row(
        "tot",
        "int",
        "Time over Target (sec)",
        "NAV",
        Derived("`nav.wp_distance` / `ground_speed`, whole seconds, 0 when not moving"),
    ),
    row(
        "toh",
        "int",
        "Time over Home (sec)",
        "NAV",
        Derived("`DistToHome` / `ground_speed`, whole seconds, 0 when not moving"),
    ),
    row(
        "distTraveled",
        "float",
        "Dist Traveled (dist)",
        "Position",
        Done(
            "dist_traveled (metres, where the C# adds display units; counted by `update_current_settings`, which the link must call on every vehicle after each read, as `VehicleRegistry::update_current_settings`)",
        ),
    ),
    row(
        "timeSinceArmInAir",
        "float",
        "Time in Air (sec)",
        "Position",
        Done(
            "time_since_arm_in_air (counted by `update_current_settings`; `HEARTBEAT` arming restarts it)",
        ),
    ),
    row(
        "timeInAir",
        "float",
        "Time in Air (sec)",
        "Position",
        Done("time_in_air (counted by `update_current_settings`)"),
    ),
    row(
        "timeInAirMinSec",
        "float",
        "Time in Air (min.sec)",
        "Position",
        Done("time_in_air_min_sec()"),
    ),
    row(
        "turnrate",
        "float",
        "Turn Rate (speed)",
        "Position",
        Done("turn_rate()"),
    ),
    row(
        "turng",
        "float",
        "Turn Gs (load)",
        "Position",
        Derived("1 / cos(`attitude.roll`)"),
    ),
    row(
        "radius",
        "float",
        "Turn Radius (dist)",
        "Position",
        Derived("`ground_speed`² / (9.80665 * tan(`attitude.roll`)), 0 at 1 m/s and below"),
    ),
    row(
        "QNH",
        "float",
        "",
        "Position",
        Derived("`press_abs` / exp(ln(1 - `altitude_msl` / (153.8462 * 288.15)) / 0.190259)"),
    ),
    row(
        "wind_dir",
        "float",
        "Wind Direction (Deg)",
        "Position",
        Done(
            "wind_direction (from `WIND`; the C#'s own estimate when no `WIND` arrives is not ported)",
        ),
    ),
    row(
        "wind_vel",
        "float",
        "Wind Velocity (speed)",
        "Position",
        Done("wind_speed (m/s; the C# stores this one in display units)"),
    ),
    row(
        "targetaltd100",
        "float",
        "",
        "NAV",
        Derived("`target_altitude()` / 100 % 10"),
    ),
    row(
        "targetalt",
        "float",
        "",
        "NAV",
        Derived("`target_altitude()`, without the C#'s low-pass"),
    ),
    row(
        "messages",
        "List<(DateTime time, string message)>",
        "",
        "",
        Plumbing(
            "the vehicle's `STATUSTEXT` history, which the link keeps (`mp_link::messages::MessageLog`); the state here is `Copy` and holds no text",
        ),
    ),
    row(
        "message",
        "string",
        "",
        "",
        Plumbing("the last `STATUSTEXT`; see `messages`"),
    ),
    row(
        "messageHigh",
        "string",
        "",
        "",
        Derived(
            "mp-gui's `hud::high_priority_message`, from `ekf` and `sensors`. Not derived there yet: the `STATUSTEXT` messages MAVLinkInterface raises to it (severity at or below the setting, default 4, or text starting `Tuning:`, `PreArm:` or `Arm:`; MAVLinkInterface.cs:5397-5420), and the fence-breach (`fence_breach`), over-current (`board.voltage_flags`) and high-latency failure texts",
        ),
    ),
    row(
        "messageHighSeverity",
        "MAVLink.MAV_SEVERITY",
        "",
        "Other",
        Derived(
            "EMERGENCY for the messages the C# raises itself; the `STATUSTEXT`'s own severity for the ones MAVLinkInterface raises (MAVLinkInterface.cs:5399-5420)",
        ),
    ),
    row(
        "battery_voltage",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("battery.voltage"),
    ),
    row(
        "battery_voltage3",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[1].voltage"),
    ),
    row(
        "battery_voltage4",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[2].voltage"),
    ),
    row(
        "battery_voltage5",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[3].voltage"),
    ),
    row(
        "battery_voltage6",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[4].voltage"),
    ),
    row(
        "battery_voltage7",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[5].voltage"),
    ),
    row(
        "battery_voltage8",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[6].voltage"),
    ),
    row(
        "battery_voltage9",
        "double",
        "Bat Voltage (V)",
        "Battery",
        Done("batteries[7].voltage"),
    ),
    row(
        "battery_remaining",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("battery.remaining_percent"),
    ),
    row(
        "battery_remaining2",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[0].remaining_percent"),
    ),
    row(
        "battery_remaining3",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[1].remaining_percent"),
    ),
    row(
        "battery_remaining4",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[2].remaining_percent"),
    ),
    row(
        "battery_remaining5",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[3].remaining_percent"),
    ),
    row(
        "battery_remaining6",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[4].remaining_percent"),
    ),
    row(
        "battery_remaining7",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[5].remaining_percent"),
    ),
    row(
        "battery_remaining8",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[6].remaining_percent"),
    ),
    row(
        "battery_remaining9",
        "int",
        "Bat Remaining (%)",
        "Battery",
        Done("batteries[7].remaining_percent"),
    ),
    row(
        "current",
        "double",
        "Bat Current (Amps)",
        "Battery",
        Done("battery.current"),
    ),
    row(
        "current2",
        "double",
        "Bat2 Current (Amps)",
        "Battery",
        Done("batteries[0].current"),
    ),
    row(
        "current3",
        "double",
        "Bat3 Current (Amps)",
        "Battery",
        Done("batteries[1].current"),
    ),
    row(
        "current4",
        "double",
        "Bat4 Current (Amps)",
        "Battery",
        Done("batteries[2].current"),
    ),
    row(
        "current5",
        "double",
        "Bat5 Current (Amps)",
        "Battery",
        Done("batteries[3].current"),
    ),
    row(
        "current6",
        "double",
        "Bat6 Current (Amps)",
        "Battery",
        Done("batteries[4].current"),
    ),
    row(
        "current7",
        "double",
        "Bat7 Current (Amps)",
        "Battery",
        Done("batteries[5].current"),
    ),
    row(
        "current8",
        "double",
        "Bat8 Current (Amps)",
        "Battery",
        Done("batteries[6].current"),
    ),
    row(
        "current9",
        "double",
        "Bat9 Current (Amps)",
        "Battery",
        Done("batteries[7].current"),
    ),
    row(
        "watts",
        "double",
        "Bat Watts",
        "Battery",
        Derived("`battery.voltage` * `battery.current`"),
    ),
    row(
        "battery_mahperkm",
        "double",
        "Bat efficiency (mah/km)",
        "Battery",
        Done("battery_mah_per_km() (unguarded: infinite or NaN before any distance)"),
    ),
    row(
        "battery_kmleft",
        "double",
        "Bat km left EST (km)",
        "Battery",
        Done("battery_km_left()"),
    ),
    row(
        "battery_usedmah",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done(
            "battery_used_mah (`BATTERY_STATUS`'s `current_consumed`, which is also `battery.consumed_mah`, and between reports the `SYS_STATUS` current integrated against `datetime`)",
        ),
    ),
    row(
        "battery_cell1",
        "double",
        "",
        "Battery",
        Done("battery.cells[0]"),
    ),
    row(
        "battery_cell2",
        "double",
        "",
        "Battery",
        Done("battery.cells[1]"),
    ),
    row(
        "battery_cell3",
        "double",
        "",
        "Battery",
        Done("battery.cells[2]"),
    ),
    row(
        "battery_cell4",
        "double",
        "",
        "Battery",
        Done("battery.cells[3]"),
    ),
    row(
        "battery_cell5",
        "double",
        "",
        "Battery",
        Done("battery.cells[4]"),
    ),
    row(
        "battery_cell6",
        "double",
        "",
        "Battery",
        Done("battery.cells[5]"),
    ),
    row(
        "battery_cell7",
        "double",
        "",
        "Battery",
        Done("battery.cells[6]"),
    ),
    row(
        "battery_cell8",
        "double",
        "",
        "Battery",
        Done("battery.cells[7]"),
    ),
    row(
        "battery_cell9",
        "double",
        "",
        "Battery",
        Done("battery.cells[8]"),
    ),
    row(
        "battery_cell10",
        "double",
        "",
        "Battery",
        Done("battery.cells[9]"),
    ),
    row(
        "battery_cell11",
        "double",
        "",
        "Battery",
        Done("battery.cells[10]"),
    ),
    row(
        "battery_cell12",
        "double",
        "",
        "Battery",
        Done("battery.cells[11]"),
    ),
    row(
        "battery_cell13",
        "double",
        "",
        "Battery",
        Done("battery.cells[12]"),
    ),
    row(
        "battery_cell14",
        "double",
        "",
        "Battery",
        Done("battery.cells[13]"),
    ),
    row(
        "battery_temp",
        "double",
        "",
        "Battery",
        Done("battery.temperature"),
    ),
    row(
        "battery_temp2",
        "double",
        "",
        "Battery",
        Done("batteries[0].temperature"),
    ),
    row(
        "battery_temp3",
        "double",
        "",
        "Battery",
        Done("batteries[1].temperature"),
    ),
    row(
        "battery_temp4",
        "double",
        "",
        "Battery",
        Done("batteries[2].temperature"),
    ),
    row(
        "battery_temp5",
        "double",
        "",
        "Battery",
        Done("batteries[3].temperature"),
    ),
    row(
        "battery_temp6",
        "double",
        "",
        "Battery",
        Done("batteries[4].temperature"),
    ),
    row(
        "battery_temp7",
        "double",
        "",
        "Battery",
        Done("batteries[5].temperature"),
    ),
    row(
        "battery_temp8",
        "double",
        "",
        "Battery",
        Done("batteries[6].temperature"),
    ),
    row(
        "battery_temp9",
        "double",
        "",
        "Battery",
        Done("batteries[7].temperature"),
    ),
    row(
        "battery_remainmin",
        "double",
        "",
        "Battery",
        Done("battery.remaining_minutes"),
    ),
    row(
        "battery_remainmin2",
        "double",
        "",
        "Battery",
        Done("batteries[0].remaining_minutes"),
    ),
    row(
        "battery_remainmin3",
        "double",
        "",
        "Battery",
        Done("batteries[1].remaining_minutes"),
    ),
    row(
        "battery_remainmin4",
        "double",
        "",
        "Battery",
        Done("batteries[2].remaining_minutes"),
    ),
    row(
        "battery_remainmin5",
        "double",
        "",
        "Battery",
        Done("batteries[3].remaining_minutes"),
    ),
    row(
        "battery_remainmin6",
        "double",
        "",
        "Battery",
        Done("batteries[4].remaining_minutes"),
    ),
    row(
        "battery_remainmin7",
        "double",
        "",
        "Battery",
        Done("batteries[5].remaining_minutes"),
    ),
    row(
        "battery_remainmin8",
        "double",
        "",
        "Battery",
        Done("batteries[6].remaining_minutes"),
    ),
    row(
        "battery_remainmin9",
        "double",
        "",
        "Battery",
        Done("batteries[7].remaining_minutes"),
    ),
    row(
        "battery_usedmah2",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[0].consumed_mah"),
    ),
    row(
        "battery_usedmah3",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[1].consumed_mah"),
    ),
    row(
        "battery_usedmah4",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[2].consumed_mah"),
    ),
    row(
        "battery_usedmah5",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[3].consumed_mah"),
    ),
    row(
        "battery_usedmah6",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[4].consumed_mah"),
    ),
    row(
        "battery_usedmah7",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[5].consumed_mah"),
    ),
    row(
        "battery_usedmah8",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[6].consumed_mah"),
    ),
    row(
        "battery_usedmah9",
        "double",
        "Bat used EST (mah)",
        "Battery",
        Done("batteries[7].consumed_mah"),
    ),
    row(
        "battery_voltage2",
        "double",
        "Bat2 Voltage (V)",
        "Battery",
        Done("batteries[0].voltage"),
    ),
    row("HomeAlt", "double", "", "Position", Done("home_altitude")),
    row(
        "HomeLocation",
        "PointLatLngAlt",
        "",
        "Position",
        Done("home (with `home_altitude`)"),
    ),
    row(
        "PlannedHomeLocation",
        "PointLatLngAlt",
        "",
        "Position",
        Done(
            "VehicleState::planned_home() (process-wide, as the C#'s static is; `VehicleState::set_planned_home` is what start-up must call with the `TXT_homelat`, `TXT_homelng` and `TXT_homealt` settings, as MainV2.cs:1012-1025 does)",
        ),
    ),
    row(
        "Base",
        "PointLatLngAlt",
        "",
        "Position",
        Done(
            "base ((0, 0, 0) until the RTK injection page or the moving-base control writes it, ConfigSerialInjectGPS.cs:910, 1077, 1098 and Controls/MovingBase.cs:227)",
        ),
    ),
    row(
        "TrackerLocation",
        "PointLatLngAlt",
        "",
        "Position",
        Done(
            "tracker_location() (home until `VehicleState::set_tracker_location` - the planner's Set Tracker Home, FlightPlanner.cs:760, 6977 - gives it a longitude)",
        ),
    ),
    row(
        "Location",
        "PointLatLngAlt",
        "",
        "Position",
        Derived("`position` with `altitude_msl`"),
    ),
    row(
        "TargetLocation",
        "PointLatLngAlt",
        "",
        "Position",
        Done(
            "target_position with `target_altitude_msl` (the C#'s tag on it, the type mask, is not kept)",
        ),
    ),
    row(
        "GeoFenceDist",
        "float",
        "",
        "Other",
        Done(
            "geo_fence_dist() (given the fence items the C# reads from `MAVState.fencepoints`, which their owner must pass)",
        ),
    ),
    row(
        "DistToHome",
        "float",
        "Dist to Home (dist)",
        "Position",
        Derived(
            "from `tracker_location()` - home unless a tracker is set - to `position` on the C#'s flat projection, 111319.5 m a degree with longitude scaled by cos(latitude)",
        ),
    ),
    row(
        "DistFromMovingBase",
        "float",
        "Dist to Moving Base (dist)",
        "Position",
        Done("dist_from_moving_base() (from 0° 0° until a base is set, as the C# is)"),
    ),
    row(
        "ELToMAV",
        "float",
        "Elevation to Mav (deg)",
        "Position",
        Derived("atan((`altitude_msl` - `home_altitude`) / `DistToHome`), degrees, from `home`"),
    ),
    row(
        "AZToMAV",
        "float",
        "Bearing to Mav (deg)",
        "Position",
        Derived("the bearing from `home` to `position` on the C#'s flat projection"),
    ),
    row(
        "sonarrange",
        "float",
        "Sonar Range (alt)",
        "Sensor",
        Done("rangefinder.range"),
    ),
    row(
        "sonarvoltage",
        "float",
        "Sonar Voltage (Volt)",
        "Sensor",
        Done("rangefinder.voltage"),
    ),
    row(
        "rangefinder1",
        "uint",
        "RangeFinder1 (cm)",
        "Sensor",
        Done("rangefinder.distances[0]"),
    ),
    row(
        "rangefinder2",
        "uint",
        "RangeFinder2 (cm)",
        "Sensor",
        Done("rangefinder.distances[1]"),
    ),
    row(
        "rangefinder3",
        "uint",
        "RangeFinder3 (cm)",
        "Sensor",
        Done("rangefinder.distances[2]"),
    ),
    row(
        "rangefinder4",
        "uint",
        "RangeFinder4 (cm)",
        "Sensor",
        Done("rangefinder.distances[3]"),
    ),
    row(
        "rangefinder5",
        "uint",
        "RangeFinder5 (cm)",
        "Sensor",
        Done("rangefinder.distances[4]"),
    ),
    row(
        "rangefinder6",
        "uint",
        "RangeFinder6 (cm)",
        "Sensor",
        Done("rangefinder.distances[5]"),
    ),
    row(
        "rangefinder7",
        "uint",
        "RangeFinder7 (cm)",
        "Sensor",
        Done("rangefinder.distances[6]"),
    ),
    row(
        "rangefinder8",
        "uint",
        "RangeFinder8 (cm)",
        "Sensor",
        Done("rangefinder.distances[7]"),
    ),
    row(
        "rangefinder9",
        "uint",
        "RangeFinder9 (cm)",
        "Sensor",
        Done("rangefinder.distances[8]"),
    ),
    row(
        "rangefinder10",
        "uint",
        "RangeFinder10 (cm)",
        "Sensor",
        Done("rangefinder.distances[9]"),
    ),
    row(
        "freemem",
        "float",
        "",
        "Software",
        Done("board.free_memory"),
    ),
    row("load", "float", "", "Software", Done("load")),
    row("brklevel", "float", "", "Software", Done("board.brk_level")),
    row("armed", "bool", "", "Software", Done("armed")),
    row(
        "rssi",
        "float",
        "Sik Radio rssi",
        "Telem",
        Done("radio.rssi"),
    ),
    row(
        "remrssi",
        "float",
        "Sik Radio remote rssi",
        "Telem",
        Done("radio.remrssi"),
    ),
    row("txbuffer", "byte", "", "Telem", Done("radio.txbuf")),
    row(
        "noise",
        "float",
        "Sik Radio noise",
        "Telem",
        Done("radio.noise"),
    ),
    row(
        "remnoise",
        "float",
        "Sik Radio remote noise",
        "Telem",
        Done("radio.remnoise"),
    ),
    row("rxerrors", "ushort", "", "Telem", Done("radio.rxerrors")),
    row("fixedp", "ushort", "", "Telem", Done("radio.fixed")),
    row(
        "localsnrdb",
        "float",
        "Sik Radio snr",
        "Telem",
        Derived("(`radio.rssi` - `radio.noise`) / 1.9, low-passed once a second"),
    ),
    row(
        "remotesnrdb",
        "float",
        "Sik Radio remote snr",
        "Telem",
        Derived("(`radio.remrssi` - `radio.remnoise`) / 1.9, low-passed once a second"),
    ),
    row(
        "DistRSSIRemain",
        "float",
        "Sik Radio est dist (m)",
        "Telem",
        Derived("`DistToHome` * 2^((the lesser signal-to-noise - 5) / 6)"),
    ),
    row(
        "packetdropremote",
        "ushort",
        "",
        "Telem",
        Done("packet_drop_remote"),
    ),
    row(
        "linkqualitygcs",
        "ushort",
        "",
        "Telem",
        Done("link.quality_percent() (the C#'s zeroing after ten silent seconds is not kept)"),
    ),
    row(
        "errors_count1",
        "ushort",
        "Error Type",
        "Hardware",
        Done("errors_count[0]"),
    ),
    row(
        "errors_count2",
        "ushort",
        "Error Type",
        "Hardware",
        Done("errors_count[1]"),
    ),
    row(
        "errors_count3",
        "ushort",
        "Error Type",
        "Hardware",
        Done("errors_count[2]"),
    ),
    row(
        "errors_count4",
        "ushort",
        "Error Count",
        "Hardware",
        Done("errors_count[3]"),
    ),
    row(
        "hwvoltage",
        "float",
        "HW Voltage",
        "Hardware",
        Done("board.hw_voltage"),
    ),
    row(
        "boardvoltage",
        "float",
        "Board Voltage",
        "Hardware",
        Done("board.board_voltage (millivolts, as the C# shows them)"),
    ),
    row(
        "servovoltage",
        "float",
        "Servo Rail Voltage",
        "Hardware",
        Done("board.servo_voltage (millivolts, as the C# shows them)"),
    ),
    row(
        "voltageflag",
        "uint",
        "Voltage Flags",
        "Hardware",
        Done("board.voltage_flags"),
    ),
    row(
        "i2cerrors",
        "ushort",
        "",
        "Hardware",
        Done("board.i2c_errors"),
    ),
    row(
        "timesincelastshot",
        "double",
        "",
        "Other",
        Done(
            "time_since_last_shot (0 until the flight screen sets it from `VehicleState::shot_interval` over the camera feedback it keeps, as FlightData.cs:4021-4038 does)",
        ),
    ),
    row("press_abs", "float", "", "Sensor", Done("press_abs")),
    row("press_temp", "int", "", "Sensor", Done("press_temp")),
    row("press_abs2", "float", "", "Sensor", Done("press_abs2")),
    row("press_temp2", "int", "", "Sensor", Done("press_temp2")),
    row(
        "rateattitude",
        "int",
        "",
        "Telem",
        Done(
            "rates.attitude (4 Hz, or the saved default when the vehicle was first seen; the link's stream requests and Planner's rate combos read and write it)",
        ),
    ),
    row("rateposition", "int", "", "Telem", Done("rates.position")),
    row("ratestatus", "int", "", "Telem", Done("rates.status")),
    row("ratesensors", "int", "", "Telem", Done("rates.sensors")),
    row("raterc", "int", "", "Telem", Done("rates.rc")),
    row(
        "datetime",
        "DateTime",
        "",
        "",
        Done(
            "datetime (each packet's time, stamped by `VehicleRegistry::apply_at`, which the link thread must call with `DateTime::now()`, or a replay with the log's time, in place of `apply`)",
        ),
    ),
    row(
        "connected",
        "bool",
        "",
        "",
        Plumbing("whether the link is open, which the link knows (`mp_link`), not the vehicle"),
    ),
    row("campointa", "float", "", "Mount", Done("mount.pointing_a")),
    row("campointb", "float", "", "Mount", Done("mount.pointing_b")),
    row("campointc", "float", "", "Mount", Done("mount.pointing_c")),
    row(
        "GimbalPoint",
        "PointLatLngAlt",
        "",
        "Mount",
        Done(
            "gimbal_point (`None` until the flight screen writes what `GimbalPoint.ProjectPoint` projects, FlightData.cs:3964-3995; that projection is not ported)",
        ),
    ),
    row("gimballat", "float", "", "Mount", Done("gimbal_lat()")),
    row("gimballng", "float", "", "Mount", Done("gimbal_lng()")),
    row(
        "landed",
        "bool",
        "",
        "Software",
        Derived(
            "`system_status` == MAV_STATE_STANDBY (3); `HIGH_LATENCY`'s landed state is not held",
        ),
    ),
    row(
        "safetyactive",
        "bool",
        "",
        "Software",
        Done("sensors.safety_active()"),
    ),
    row(
        "terrainactive",
        "bool",
        "",
        "Terrain",
        Done("sensors.terrain_active()"),
    ),
    row(
        "ter_curalt",
        "float",
        "Terrain AGL",
        "Terrain",
        Done("terrain.current_height"),
    ),
    row(
        "ter_alt",
        "float",
        "Terrain GL",
        "Terrain",
        Done("terrain.terrain_height"),
    ),
    row("ter_load", "float", "", "Terrain", Done("terrain.loaded")),
    row("ter_pend", "float", "", "Terrain", Done("terrain.pending")),
    row("ter_space", "float", "", "Terrain", Done("terrain.spacing")),
    row(
        "KIndex",
        "int",
        "",
        "Enviromental",
        Done("VehicleState::kindex() (`KIndexstatic`)"),
    ),
    row(
        "opt_m_x",
        "float",
        "flow_comp_m_x",
        "Flow",
        Done("optical_flow.comp_m_x"),
    ),
    row(
        "opt_m_y",
        "float",
        "flow_comp_m_y",
        "Flow",
        Done("optical_flow.comp_m_y"),
    ),
    row("opt_x", "short", "flow_x", "Flow", Done("optical_flow.x")),
    row("opt_y", "short", "flow_y", "Flow", Done("optical_flow.y")),
    row(
        "opt_qua",
        "byte",
        "flow quality",
        "Flow",
        Done("optical_flow.quality"),
    ),
    row("ekfstatus", "float", "", "EKF", Done("ekf_status()")),
    row("ekfflags", "int", "", "EKF", Done("ekf.flags")),
    row("ekfvelv", "float", "", "EKF", Done("ekf.velocity_variance")),
    row("ekfcompv", "float", "", "EKF", Done("ekf.compass_variance")),
    row(
        "ekfposhor",
        "float",
        "",
        "EKF",
        Done("ekf.position_horizontal_variance"),
    ),
    row(
        "ekfposvert",
        "float",
        "",
        "EKF",
        Done("ekf.position_vertical_variance"),
    ),
    row(
        "ekfteralt",
        "float",
        "",
        "EKF",
        Done("ekf.terrain_altitude_variance"),
    ),
    row("pidff", "float", "", "PID", Done("pid.ff")),
    row("pidP", "float", "", "PID", Done("pid.p")),
    row("pidI", "float", "", "PID", Done("pid.i")),
    row("pidD", "float", "", "PID", Done("pid.d")),
    row("pidaxis", "byte", "", "PID", Done("pid.axis")),
    row("piddesired", "float", "", "PID", Done("pid.desired")),
    row("pidachieved", "float", "", "PID", Done("pid.achieved")),
    row("pidSRate", "float", "", "PID", Done("pid.srate")),
    row("pidPDmod", "float", "", "PID", Done("pid.pdmod")),
    row("pidSRateRoll", "float", "", "PID", Done("pid.srate_roll")),
    row("pidSRatePitch", "float", "", "PID", Done("pid.srate_pitch")),
    row("pidSRateYaw", "float", "", "PID", Done("pid.srate_yaw")),
    row("pidSRateAccZ", "float", "", "PID", Done("pid.srate_accz")),
    row("pidSRateSteer", "float", "", "PID", Done("pid.srate_steer")),
    row(
        "pidSRateLanding",
        "float",
        "",
        "PID",
        Done("pid.srate_landing"),
    ),
    row(
        "vibeclip0",
        "uint",
        "",
        "Vibe",
        Done("vibration.clipping[0]"),
    ),
    row(
        "vibeclip1",
        "uint",
        "",
        "Vibe",
        Done("vibration.clipping[1]"),
    ),
    row(
        "vibeclip2",
        "uint",
        "",
        "Vibe",
        Done("vibration.clipping[2]"),
    ),
    row("vibex", "float", "", "Vibe", Done("vibration.x")),
    row("vibey", "float", "", "Vibe", Done("vibration.y")),
    row("vibez", "float", "", "Vibe", Done("vibration.z")),
    row(
        "version",
        "Version",
        "",
        "Software",
        Done(
            "autopilot_info.version (major, minor, patch and type, where the C# holds a `Version`)",
        ),
    ),
    row("uid", "ulong", "", "Software", Done("autopilot_info.uid")),
    row(
        "uid2",
        "string",
        "",
        "Software",
        Derived("`autopilot_info.uid2`, the bytes the C# writes out as hex"),
    ),
    row("rpm1", "float", "", "Sensor", Done("rpm[0]")),
    row("rpm2", "float", "", "Sensor", Done("rpm[1]")),
    row(
        "capabilities",
        "uint",
        "",
        "Software",
        Done("autopilot_info.capabilities"),
    ),
    row(
        "speedup",
        "float",
        "",
        "Software",
        Done(
            "speedup (`RAW_IMU`'s clock against `datetime`, where the C# uses the wall clock - the same live, the recorded time in a replay)",
        ),
    ),
    row("vtol_state", "byte", "", "Software", Done("vtol_state")),
    row("landed_state", "byte", "", "Software", Done("landed_state")),
    row(
        "gen_status",
        "float",
        "",
        "Generator",
        Done("generator.status"),
    ),
    row(
        "gen_speed",
        "float",
        "",
        "Generator",
        Done("generator.speed"),
    ),
    row(
        "gen_current",
        "float",
        "",
        "Generator",
        Done("generator.current"),
    ),
    row(
        "gen_voltage",
        "float",
        "",
        "Generator",
        Done("generator.voltage"),
    ),
    row(
        "gen_runtime",
        "uint",
        "",
        "Generator",
        Done("generator.runtime"),
    ),
    row(
        "gen_maint_time",
        "int",
        "",
        "Generator",
        Done("generator.maintenance_in"),
    ),
    row(
        "efi_baro",
        "float",
        "EFI Baro Pressure (kPa)",
        "EFI",
        Done("efi.baro"),
    ),
    row(
        "efi_headtemp",
        "float",
        "EFI Head Temp (C)",
        "EFI",
        Done("efi.head_temperature"),
    ),
    row("efi_load", "float", "EFI Load (%)", "EFI", Done("efi.load")),
    row(
        "efi_health",
        "byte",
        "EFI Health",
        "EFI",
        Done("efi.health"),
    ),
    row(
        "efi_exhasttemp",
        "float",
        "EFI Exhast Temp (C)",
        "EFI",
        Done("efi.exhaust_temperature"),
    ),
    row(
        "efi_intaketemp",
        "float",
        "EFI Intake Temp (C)",
        "EFI",
        Done("efi.intake_temperature"),
    ),
    row("efi_rpm", "float", "EFI rpm", "EFI", Done("efi.rpm")),
    row(
        "efi_fuelflow",
        "float",
        "EFI Fuel Flow (cm3/min)",
        "EFI",
        Done("efi.fuel_flow"),
    ),
    row(
        "efi_fuelconsumed",
        "float",
        "EFI Fuel Consumed (cm3)",
        "EFI",
        Done("efi.fuel_consumed"),
    ),
    row(
        "efi_fuelpressure",
        "float",
        "EFI Fuel Pressure (kPa)",
        "EFI",
        Done("efi.fuel_pressure"),
    ),
    row(
        "xpdr_es1090_tx_enabled",
        "bool",
        "Transponder 1090ES Tx Enabled",
        "Transponder Status",
        Done("transponder.es1090_tx_enabled"),
    ),
    row(
        "xpdr_mode_S_enabled",
        "bool",
        "Transponder Mode S Reply Enabled",
        "Transponder Status",
        Done("transponder.mode_s_enabled"),
    ),
    row(
        "xpdr_mode_C_enabled",
        "bool",
        "Transponder Mode C Reply Enabled",
        "Transponder Status",
        Done("transponder.mode_c_enabled"),
    ),
    row(
        "xpdr_mode_A_enabled",
        "bool",
        "Transponder Mode A Reply Enabled",
        "Transponder Status",
        Done("transponder.mode_a_enabled"),
    ),
    row(
        "xpdr_ident_active",
        "bool",
        "Ident Active",
        "Transponder Status",
        Done("transponder.ident_active"),
    ),
    row(
        "xpdr_x_bit_status",
        "bool",
        "X-bit Status",
        "Transponder Status",
        Done("transponder.x_bit_status"),
    ),
    row(
        "xpdr_interrogated_since_last",
        "bool",
        "Interrogated since last",
        "Transponder Status",
        Done("transponder.interrogated_since_last"),
    ),
    row(
        "xpdr_airborne_status",
        "bool",
        "Airborne",
        "Transponder Status",
        Done("transponder.airborne"),
    ),
    row(
        "xpdr_mode_A_squawk_code",
        "ushort",
        "Transponder Mode A squawk code",
        "Transponder Status",
        Done("transponder.squawk"),
    ),
    row(
        "xpdr_nic",
        "byte",
        "NIC",
        "Transponder Status",
        Done("transponder.nic"),
    ),
    row(
        "xpdr_nacp",
        "byte",
        "NACp",
        "Transponder Status",
        Done("transponder.nacp"),
    ),
    row(
        "xpdr_board_temperature",
        "byte",
        "Board Temperature in C",
        "Transponder Status",
        Done("transponder.board_temperature"),
    ),
    row(
        "xpdr_maint_req",
        "bool",
        "Maintainence Required",
        "Transponder Status",
        Done("transponder.maintenance_required"),
    ),
    row(
        "xpdr_adsb_tx_sys_fail",
        "bool",
        "ADSB Tx System Failure",
        "Transponder Status",
        Done("transponder.adsb_tx_failure"),
    ),
    row(
        "xpdr_gps_unavail",
        "bool",
        "GPS Unavailable",
        "Transponder Status",
        Done("transponder.gps_unavailable"),
    ),
    row(
        "xpdr_gps_no_fix",
        "bool",
        "GPS No Fix",
        "Transponder Status",
        Done("transponder.gps_no_fix"),
    ),
    row(
        "xpdr_status_unavail",
        "bool",
        "Ping200X No Status Message Recieved",
        "Transponder Status",
        Done("transponder.status_unavailable"),
    ),
    row(
        "xpdr_status_pending",
        "bool",
        "Status Update Pending",
        "Transponder Status",
        Done(
            "transponder.status_count (one more for each status, where the C# sets the flag; the Transponder page, which clears it in the C#, keeps the count it last looked at, and a status is pending while the two differ - `transponder.status_pending` says only that one has ever arrived)",
        ),
    ),
    row(
        "xpdr_flight_id",
        "byte[]",
        "Callsign/Flight ID",
        "Transponder Status",
        Done("transponder.flight_id"),
    ),
    row("ahrs2_roll", "float", "", "AHRS2", Done("ahrs2.roll")),
    row("ahrs2_pitch", "float", "", "AHRS2", Done("ahrs2.pitch")),
    row("ahrs2_yaw", "float", "", "AHRS2", Done("ahrs2.yaw")),
    row("ahrs2_alt", "float", "", "AHRS2", Done("ahrs2.altitude")),
    row("ahrs2_lat", "double", "", "AHRS2", Done("ahrs2.lat")),
    row("ahrs2_lng", "double", "", "AHRS2", Done("ahrs2.lng")),
    row(
        "mcumaxvolt",
        "float",
        "mcu max voltage",
        "MCU",
        Done("mcu.voltage_max"),
    ),
    row(
        "mcuminvolt",
        "float",
        "mcu min voltage",
        "MCU",
        Done("mcu.voltage_min"),
    ),
    row(
        "mcuvoltage",
        "float",
        "mcu voltage",
        "MCU",
        Done("mcu.voltage"),
    ),
    row(
        "mcutemp",
        "float",
        "mcu temperature",
        "MCU",
        Done("mcu.temperature"),
    ),
    row(
        "fenceb_count",
        "ushort",
        "Breach count",
        "Fence",
        Done("fence_breach.count"),
    ),
    row(
        "fenceb_status",
        "byte",
        "Breach status",
        "Fence",
        Done("fence_breach.status"),
    ),
    row(
        "fenceb_type",
        "byte",
        "Breach type",
        "Fence",
        Done("fence_breach.breach_type"),
    ),
    row(
        "posn",
        "float",
        "North",
        "Position",
        Done("local_position[0]"),
    ),
    row(
        "pose",
        "float",
        "East",
        "Position",
        Done("local_position[1]"),
    ),
    row(
        "posd",
        "float",
        "Down",
        "Position",
        Done("local_position[2]"),
    ),
    row(
        "csCallBack",
        "event EventHandler",
        "",
        "",
        Plumbing(
            "an event raised on every update for the WinForms bindings; readers take snapshots from `StateHandle` instead",
        ),
    ),
];

/// How many rows are in each state.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    /// Held here.
    pub done: usize,
    /// Computable from what is held.
    pub derived: usize,
    /// Not ported yet.
    pub missing: usize,
    /// Not vehicle state.
    pub plumbing: usize,
    /// Deliberately not carried over.
    pub dropped: usize,
}

impl Counts {
    /// Every row.
    #[must_use]
    pub const fn total(&self) -> usize {
        self.done + self.derived + self.missing + self.plumbing + self.dropped
    }
}

/// How many rows are in each state.
#[must_use]
pub fn counts() -> Counts {
    let mut counts = Counts::default();
    for field in CURRENTSTATE {
        match field.ours {
            Done(_) => counts.done += 1,
            Derived(_) => counts.derived += 1,
            Missing => counts.missing += 1,
            Plumbing(_) => counts.plumbing += 1,
            Dropped(_) => counts.dropped += 1,
        }
    }
    counts
}

/// The groups with missing fields, most missing first: (group, missing, rows in the group).
///
/// The C#'s `[GroupText]`, or "(none)" for a property without one.
#[must_use]
pub fn missing_by_group() -> Vec<(&'static str, usize, usize)> {
    let mut groups: Vec<(&'static str, usize, usize)> = Vec::new();
    for field in CURRENTSTATE {
        let group = if field.group.is_empty() {
            "(none)"
        } else {
            field.group
        };
        let missing = usize::from(field.ours == Missing);
        match groups.iter_mut().find(|(name, _, _)| *name == group) {
            Some(entry) => {
                entry.1 += missing;
                entry.2 += 1;
            }
            None => groups.push((group, missing, 1)),
        }
    }
    groups.retain(|(_, missing, _)| *missing > 0);
    groups.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(b.0)));
    groups
}

/// The report, as Markdown: the counts, the groups with the most missing, then every row.
#[must_use]
pub fn report() -> String {
    let counts = counts();
    let mut out = String::new();
    out.push_str("# CurrentState field coverage\n\n");
    out.push_str(
        "Generated from `crates/mp-vehicle/src/coverage.rs` by `cargo test -p mp-vehicle \
         coverage -- --ignored update_report`; a test fails when this file is stale. One row per \
         public property or field of `ExtLibs/ArduPilot/CurrentState.cs`, in the file's order. \
         The display text and group are the C#'s `[DisplayText]` and `[GroupText]`, which the \
         quick view's field chooser shows. *Done* means a field or method of \
         `mp_vehicle::VehicleState` holds the same value in the same units (SI: the C# applies \
         the user's display units in each getter); *derived* means it can be computed from what \
         is held.\n\n",
    );
    let _ = write!(
        out,
        "| total | done | derived | missing | plumbing | dropped |\n\
         |---:|---:|---:|---:|---:|---:|\n\
         | {} | {} | {} | {} | {} | {} |\n\n",
        counts.total(),
        counts.done,
        counts.derived,
        counts.missing,
        counts.plumbing,
        counts.dropped
    );
    let groups = missing_by_group();
    out.push_str("## Missing, by group\n\n");
    if groups.is_empty() {
        out.push_str("None: every field is held, derivable, plumbing or deliberately dropped.\n");
    } else {
        out.push_str("| group | missing | of | fields |\n|---|---:|---:|---|\n");
    }
    for (group, missing, of) in groups {
        let names: Vec<String> = CURRENTSTATE
            .iter()
            .filter(|field| field.ours == Missing)
            .filter(|field| field.group == group || (field.group.is_empty() && group == "(none)"))
            .map(|field| format!("`{}`", field.name))
            .collect();
        let _ = writeln!(out, "| {group} | {missing} | {of} | {} |", names.join(", "));
    }
    out.push_str(
        "\n## Every field\n\n| C# | type | display text | group | ours |\n|---|---|---|---|---|\n",
    );
    for field in CURRENTSTATE {
        let ours = match field.ours {
            Done(claim) => match claim.split_once(' ') {
                Some((path, note)) => format!("done: `{path}` {note}"),
                None => format!("done: `{claim}`"),
            },
            Derived(how) => format!("derived: {how}"),
            Missing => "**missing**".to_owned(),
            Plumbing(why) => format!("plumbing: {why}"),
            Dropped(why) => format!("dropped: {why}"),
        };
        let _ = writeln!(
            out,
            "| `{}` | `{}` | {} | {} | {} |",
            field.name, field.ty, field.display, field.group, ours
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where the committed report lives, relative to this crate.
    const REPORT: &str = "../../docs/coverage/currentstate.md";

    /// This crate's source, for checking that a claimed field or method exists.
    const SOURCES: &[&str] = &[
        include_str!("state.rs"),
        include_str!("onboard.rs"),
        include_str!("sensors.rs"),
        include_str!("health.rs"),
        include_str!("link_quality.rs"),
        include_str!("rc.rs"),
        include_str!("units.rs"),
        include_str!("clock.rs"),
        include_str!("statics.rs"),
        include_str!("update.rs"),
        include_str!("fence.rs"),
        include_str!("registry.rs"),
    ];

    fn current_state() -> Option<String> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../references/missionplanner/ExtLibs/ArduPilot/CurrentState.cs");
        std::fs::read_to_string(path).ok()
    }

    /// A public property or field as the C# declares it.
    #[derive(Debug, Clone, PartialEq, Eq)]
    struct Declared {
        name: String,
        ty: String,
        display: String,
        group: String,
    }

    /// The source with comments blanked out, as (character, is code) pairs: string and character
    /// literals keep their text, for the attributes, but are not code, so a brace or semicolon
    /// inside one does not count.
    fn lex(source: &str) -> Vec<(char, bool)> {
        let chars: Vec<char> = source.chars().collect();
        let mut out = Vec::with_capacity(chars.len());
        let mut i = 0;
        while let Some(&c) = chars.get(i) {
            let next = chars.get(i + 1).copied();
            if c == '/' && next == Some('/') {
                while let Some(&c) = chars.get(i) {
                    if c == '\n' {
                        break;
                    }
                    out.push((' ', true));
                    i += 1;
                }
            } else if c == '/' && next == Some('*') {
                out.push((' ', true));
                out.push((' ', true));
                i += 2;
                while let Some(&c) = chars.get(i) {
                    if c == '*' && chars.get(i + 1) == Some(&'/') {
                        out.push((' ', true));
                        out.push((' ', true));
                        i += 2;
                        break;
                    }
                    out.push((if c == '\n' { '\n' } else { ' ' }, true));
                    i += 1;
                }
            } else if c == '"' || c == '\'' {
                // A literal, to its closing quote, honouring backslash escapes.
                out.push((c, false));
                i += 1;
                while let Some(&d) = chars.get(i) {
                    out.push((d, false));
                    i += 1;
                    if d == '\\' {
                        if let Some(&e) = chars.get(i) {
                            out.push((e, false));
                            i += 1;
                        }
                    } else if d == c {
                        break;
                    }
                }
            } else {
                out.push((c, true));
                i += 1;
            }
        }
        out
    }

    /// Every public property and field declared directly in the class body - brace depth 2,
    /// inside the namespace and the class - in the file's order. Methods, constructors and the
    /// nested `Mavlink_Sensors` class are not members of this list; their bodies are skipped.
    fn members(source: &str) -> Vec<Declared> {
        let lexed = lex(source);
        let mut found = Vec::new();
        let mut depth = 0_usize;
        let mut header = String::new();
        let mut i = 0;
        while let Some(&(c, code)) = lexed.get(i) {
            i += 1;
            if !code {
                if depth == 2 {
                    header.push(c);
                }
                continue;
            }
            match c {
                '{' => {
                    if depth == 2 {
                        found.extend(declaration(&header));
                        header.clear();
                    }
                    depth += 1;
                }
                '}' => depth = depth.saturating_sub(1),
                ';' if depth == 2 => {
                    found.extend(declaration(&header));
                    header.clear();
                }
                '=' if depth == 2 && lexed.get(i).is_some_and(|&(n, code)| code && n == '>') => {
                    // An expression body: the header is complete, and the body runs to the next
                    // semicolon at this depth.
                    found.extend(declaration(&header));
                    header.clear();
                    let mut inner = 0_usize;
                    while let Some(&(c, code)) = lexed.get(i) {
                        i += 1;
                        if !code {
                            continue;
                        }
                        match c {
                            '{' | '(' => inner += 1,
                            '}' | ')' => inner = inner.saturating_sub(1),
                            ';' if inner == 0 => break,
                            _ => {}
                        }
                    }
                }
                _ if depth == 2 => header.push(c),
                _ => {}
            }
        }
        found
    }

    /// One declaration's header - attributes, modifiers, type, name and anything up to the body
    /// or terminator - as a member, if it is a public property or field.
    fn declaration(header: &str) -> Option<Declared> {
        let mut rest = header.trim();
        let mut display = String::new();
        let mut group = String::new();
        while let Some(attribute) = rest.strip_prefix('[') {
            let end = attribute.find(']')?;
            let (inside, after) = attribute.split_at(end);
            let name: String = inside
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
                .collect();
            let text = inside
                .split_once('"')
                .and_then(|(_, t)| t.split_once('"'))
                .map(|(t, _)| t.to_owned())
                .unwrap_or_default();
            if name == "DisplayText" && display.is_empty() {
                display = text;
            } else if name == "GroupText" && group.is_empty() {
                group = text;
            }
            rest = after.get(1..)?.trim_start();
        }
        let mut rest = rest.strip_prefix("public ")?.trim_start();
        let mut prefix = String::new();
        loop {
            let word = rest.split_whitespace().next()?;
            match word {
                "static" | "event" => {
                    prefix.push_str(word);
                    prefix.push(' ');
                }
                "readonly" | "virtual" | "override" | "new" | "volatile" => {}
                "class" | "struct" | "enum" | "interface" | "delegate" => return None,
                _ => break,
            }
            rest = rest.get(word.len()..)?.trim_start();
        }
        // The type runs to the first space outside angle brackets and parentheses; a parenthesis
        // at the top level means a constructor.
        let mut nesting = 0_i32;
        let mut end = rest.len();
        for (at, c) in rest.char_indices() {
            match c {
                '(' if nesting == 0 => return None,
                '<' | '(' => nesting += 1,
                '>' | ')' => nesting -= 1,
                c if c.is_whitespace() && nesting == 0 => {
                    end = at;
                    break;
                }
                _ => {}
            }
        }
        let (ty, after) = rest.split_at(end);
        let after = after.trim_start();
        let name: String = after
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        if name.is_empty() || after.get(name.len()..)?.trim_start().starts_with('(') {
            // A method.
            return None;
        }
        Some(Declared {
            name,
            ty: format!("{prefix}{ty}"),
            display,
            group,
        })
    }

    /// Every shape of declaration the class uses: attributes on their own lines and inline,
    /// expression bodies, statics, fields with a comment where the accessors would be, a tuple
    /// type with spaces in it, a property body, an event - and the methods, constructor, nested
    /// class and private field that are not members of the table.
    #[test]
    fn the_parser_sees_every_kind_of_declaration() {
        let source = "namespace N { public class C {\n\
            [DisplayText(\"Roll (deg)\")]\n[GroupText(\"Attitude\")]\n public float roll { get; set; }\n\
            [GroupText(\"Position\")] public double vlen => Math.Sqrt(vx * vx);\n\
            public static float multiplierdist = 1; // { get; set; }\n\
            public int hilch1; // { get; set; }\n\
            public List<(DateTime time, string message)> messages { get; set; } = new List<(DateTime, string)>();\n\
            public float yaw\n { get => _yaw; set { if (value < 0) _yaw = value + 360; } }\n\
            public void Dispose() { }\n\
            public CurrentState() { }\n\
            public List<string> GetItemList(bool alpha = false) { return null; }\n\
            public class Mavlink_Sensors { public bool gyro { get; set; } }\n\
            public event EventHandler csCallBack;\n\
            private float hidden;\n\
            [DisplayText(\"Failsafe\")][GroupText(\"Software\")] public bool failsafe { get; set; }\n\
            } }";
        let found = members(source);
        let names: Vec<(&str, &str, &str, &str)> = found
            .iter()
            .map(|d| {
                (
                    d.name.as_str(),
                    d.ty.as_str(),
                    d.display.as_str(),
                    d.group.as_str(),
                )
            })
            .collect();
        assert_eq!(
            names,
            [
                ("roll", "float", "Roll (deg)", "Attitude"),
                ("vlen", "double", "", "Position"),
                ("multiplierdist", "static float", "", ""),
                ("hilch1", "int", "", ""),
                ("messages", "List<(DateTime time, string message)>", "", ""),
                ("yaw", "float", "", ""),
                ("csCallBack", "event EventHandler", "", ""),
                ("failsafe", "bool", "Failsafe", "Software"),
            ]
        );
    }

    /// Every public member of the C# class has one row, with the same type and attribute text,
    /// in the same order, and nothing else does.
    #[test]
    fn every_public_member_has_exactly_one_row_in_order() {
        let Some(source) = current_state() else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let declared = members(&source);
        // A hand count agrees: 570 `public` declarations in the class body, less 19 methods and
        // constructors and the nested `Mavlink_Sensors` class.
        assert_eq!(
            declared.len(),
            550,
            "CurrentState declares 550 public members"
        );
        for field in CURRENTSTATE {
            assert!(
                declared.iter().any(|d| d.name == field.name),
                "row {} is not a public member of CurrentState",
                field.name
            );
        }
        for (index, d) in declared.iter().enumerate() {
            let field = CURRENTSTATE.get(index).unwrap_or_else(|| {
                panic!("no row for {} (member {index})", d.name);
            });
            assert_eq!(
                (field.name, field.ty, field.display, field.group),
                (
                    d.name.as_str(),
                    d.ty.as_str(),
                    d.display.as_str(),
                    d.group.as_str()
                ),
                "row {index} does not match the C# declaration"
            );
        }
        assert_eq!(CURRENTSTATE.len(), declared.len());
    }

    /// Names are unique, so a row cannot be accounted twice.
    #[test]
    fn every_name_appears_once() {
        for (index, field) in CURRENTSTATE.iter().enumerate() {
            assert!(
                !CURRENTSTATE
                    .iter()
                    .skip(index + 1)
                    .any(|other| other.name == field.name),
                "{} has two rows",
                field.name
            );
        }
    }

    /// Nothing is claimed that the source does not have: each step of a claimed path is a `pub`
    /// field, a `fn` where it ends in `()`, or a `pub struct` where a type leads the path,
    /// somewhere in this crate.
    #[test]
    fn every_claimed_path_exists_in_this_crate() {
        let exists = |step: &str| {
            let step = step.split('[').next().unwrap_or(step);
            SOURCES.iter().any(|source| match step.strip_suffix("()") {
                Some(method) => source.contains(&format!("fn {method}(")),
                None => {
                    source.contains(&format!("pub {step}:"))
                        || source.contains(&format!("pub struct {step} "))
                }
            })
        };
        for field in CURRENTSTATE {
            let Done(claim) = field.ours else { continue };
            let path = claim.split(' ').next().unwrap_or(claim);
            for step in path.split(['.', ':']).filter(|step| !step.is_empty()) {
                assert!(
                    exists(step),
                    "{} claims `{path}`, and `{step}` is not in the source",
                    field.name
                );
            }
        }
    }

    /// The committed report matches the table.
    #[test]
    fn the_committed_report_is_current() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        let committed = std::fs::read_to_string(&path).unwrap_or_default();
        assert!(
            committed == report(),
            "docs/coverage/currentstate.md is stale; run \
             `cargo test -p mp-vehicle coverage -- --ignored update_report`"
        );
    }

    /// Rewrites the report. Run on purpose, not on every test.
    #[test]
    #[ignore = "writes docs/coverage/currentstate.md"]
    fn update_report() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        std::fs::write(&path, report()).expect("write the report");
    }

    /// The report leads with the counts and names the groups with the most missing.
    #[test]
    fn the_report_leads_with_the_counts_and_the_missing_groups() {
        let report = report();
        let counts = counts();
        let first_table = report
            .lines()
            .find(|line| line.starts_with("| ") && line.contains(|c: char| c.is_ascii_digit()))
            .expect("a counts row");
        assert_eq!(
            first_table,
            format!(
                "| {} | {} | {} | {} | {} | {} |",
                counts.total(),
                counts.done,
                counts.derived,
                counts.missing,
                counts.plumbing,
                counts.dropped
            )
        );
        let groups = missing_by_group();
        match groups.first().copied() {
            Some((worst, missing, _)) => assert!(
                report.contains(&format!("| {worst} | {missing} |")),
                "the worst group is named"
            ),
            // With nothing missing the section says so, and has no table to mislead.
            None => {
                assert_eq!(counts.missing, 0);
                assert!(report.contains("## Missing, by group\n\nNone: every field"));
                assert!(!report.contains("| group | missing |"));
            }
        }
        // And every missing field is named under its group.
        for field in CURRENTSTATE.iter().filter(|field| field.ours == Missing) {
            let group = if field.group.is_empty() {
                "(none)"
            } else {
                field.group
            };
            let row = report
                .lines()
                .find(|line| line.starts_with(&format!("| {group} | ")))
                .expect("the group has a row");
            assert!(
                row.contains(&format!("`{}`", field.name)),
                "{} is not named",
                field.name
            );
        }
        assert!(
            groups.windows(2).all(|pair| match pair {
                [a, b] => a.1 >= b.1,
                _ => true,
            }),
            "most missing first: {groups:?}"
        );
    }

    /// The count is what the plan says, so a change in either direction is a deliberate edit.
    #[test]
    fn the_counts_are_the_ones_the_plan_records() {
        let counts = counts();
        assert_eq!(counts.total(), CURRENTSTATE.len());
        eprintln!("CurrentState: {counts:?}");
        assert_eq!(
            (
                counts.done,
                counts.derived,
                counts.missing,
                counts.plumbing,
                counts.dropped
            ),
            (471, 48, 0, 30, 1)
        );
    }
}
