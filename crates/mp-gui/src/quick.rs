//! The flight screen's Quick page: `quickView1` to `quickView6`, and the chooser a double click
//! on one opens.
//!
//! `tabQuick` holds `tableLayoutPanelQuick`, two columns of three rows, each cell a `QuickView`:
//! a description centred along the top and a number under it, the number's font as large as
//! fits. Each is bound to one `CurrentState` property by name - `alt`, `groundspeed`, `wp_dist`,
//! `yaw`, `verticalspeed` and `DistToHome` unless the user chose others - and double-clicking one
//! lists every property to choose from.
//! `// C#: GCSViews/FlightData.Designer.cs:615-735, ExtLibs/Controls/QuickView.cs`
//!
//! What a property holds comes from `mp_vehicle::coverage::CURRENTSTATE`, the account of every
//! `CurrentState` member: a view can show any row that is held here in the same units ("done").
//! The C#'s chooser offers every numeric property, held or not; this one offers the ones this
//! application has a value for, and says the count in a fact.
//!
//! The C# keeps the choice in its settings, `Settings.Instance["quickView" + n]`. The settings
//! file is not this module's to extend, so a choice lasts for the session.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, Window, div, prelude::*, px, rgb};
use mp_units::{Metres, MetresPerSecond};
use mp_vehicle::VehicleState;
use mp_vehicle::coverage::{CURRENTSTATE, Ours};

use crate::MissionPlanner;
use crate::ui::{action, theme};

/// The six views as the Designer makes them: the property each is bound to and its
/// `numberColor`. `// C#: GCSViews/FlightData.Designer.cs:637-642, 674-679, 687-692, 700-705,
/// 713-718, 726-731`
pub const DEFAULTS: [(&str, u32); 6] = [
    ("alt", 0xd1_97_f8),
    ("groundspeed", 0xfe_84_2e),
    ("wp_dist", 0xff_60_5b),
    ("yaw", 0x00_ff_53),
    ("verticalspeed", 0xfe_fe_56),
    ("DistToHome", 0x00_ff_fc),
];

/// The ids a script clicks the views by: the Designer's names.
pub const IDS: [&str; 6] = [
    "fly-quick-1",
    "fly-quick-2",
    "fly-quick-3",
    "fly-quick-4",
    "fly-quick-5",
    "fly-quick-6",
];

/// Each view's place in `tableLayoutPanelQuick`: (column, row).
/// `// C#: GCSViews/FlightData.Designer.cs:626-631`
pub const CELLS: [(u8, u8); 6] = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 2), (1, 2)];

/// `QuickView.Font`: Microsoft Sans Serif 9.75pt, in pixels at 96 dpi.
/// `// C#: GCSViews/FlightData.resx (quickView1.Font)`
const DESC_SIZE: f32 = 13.0;

/// The line height gpui gives a font size, as `hud::paint` lays a label out.
const LINE: f32 = 1.2;

/// A value a view can show, as the number the C# binds: a bool is 1 or 0
/// (`BindingTypeToNumber`), everything else its number.
/// `// C#: GCSViews/FlightData.cs:2511-2518`
pub trait Number {
    /// The number.
    fn number(self) -> f64;
}

macro_rules! number_from {
    ($($ty:ty),*) => {
        $(impl Number for $ty {
            fn number(self) -> f64 {
                f64::from(self)
            }
        })*
    };
}
number_from!(f32, f64, u8, u16, u32, i8, i16, i32);

impl Number for bool {
    fn number(self) -> f64 {
        if self { 1.0 } else { 0.0 }
    }
}

impl Number for u64 {
    fn number(self) -> f64 {
        #[allow(clippy::cast_precision_loss)] // the C# binds a ulong through a double too
        let value = self as f64;
        value
    }
}

impl Number for Metres {
    fn number(self) -> f64 {
        self.0
    }
}

impl Number for MetresPerSecond {
    fn number(self) -> f64 {
        self.0
    }
}

/// How a property is read from the vehicle's state.
type Reader = fn(&VehicleState) -> f64;

/// A reader for every numeric `CurrentState` property this application holds in the C#'s units:
/// the rows of `CURRENTSTATE` that are done, each through the path its row names. Generated from
/// the table and held to it by a test, so a row that becomes done without a reader here fails.
#[rustfmt::skip]
const READERS: &[(&str, Reader)] = &[
    ("hilch1", |s| f64::from(s.hil_channels[0])),
    ("hilch2", |s| f64::from(s.hil_channels[1])),
    ("hilch3", |s| f64::from(s.hil_channels[2])),
    ("hilch4", |s| f64::from(s.hil_channels[3])),
    ("customfield0", |s| f64::from(s.custom_fields[0])),
    ("customfield1", |s| f64::from(s.custom_fields[1])),
    ("customfield2", |s| f64::from(s.custom_fields[2])),
    ("customfield3", |s| f64::from(s.custom_fields[3])),
    ("customfield4", |s| f64::from(s.custom_fields[4])),
    ("customfield5", |s| f64::from(s.custom_fields[5])),
    ("customfield6", |s| f64::from(s.custom_fields[6])),
    ("customfield7", |s| f64::from(s.custom_fields[7])),
    ("customfield8", |s| f64::from(s.custom_fields[8])),
    ("customfield9", |s| f64::from(s.custom_fields[9])),
    ("customfield10", |s| f64::from(s.custom_fields[10])),
    ("customfield11", |s| f64::from(s.custom_fields[11])),
    ("customfield12", |s| f64::from(s.custom_fields[12])),
    ("customfield13", |s| f64::from(s.custom_fields[13])),
    ("customfield14", |s| f64::from(s.custom_fields[14])),
    ("customfield15", |s| f64::from(s.custom_fields[15])),
    ("customfield16", |s| f64::from(s.custom_fields[16])),
    ("customfield17", |s| f64::from(s.custom_fields[17])),
    ("customfield18", |s| f64::from(s.custom_fields[18])),
    ("customfield19", |s| f64::from(s.custom_fields[19])),
    ("SSA", |s| s.ssa.number()),
    ("AOA", |s| s.aoa.number()),
    ("groundcourse", |s| s.gps.course.number()),
    ("lat", |s| s.position.map_or(0.0, |p| p.latitude())),
    ("lng", |s| s.position.map_or(0.0, |p| p.longitude())),
    ("alt", |s| s.altitude_relative.number()),
    ("altasl", |s| s.altitude_msl.number()),
    ("vx", |s| s.velocity_north.number()),
    ("vy", |s| s.velocity_east.number()),
    ("vz", |s| s.velocity_down.number()),
    ("altoffsethome", |s| f64::from(s.alt_offset_home)),
    ("gpsstatus", |s| s.gps.fix_type.number()),
    ("gpshdop", |s| s.gps.hdop.number()),
    ("satcount", |s| s.gps.satellites_visible.number()),
    ("gpsh_acc", |s| s.gps.h_acc.number()),
    ("gpsv_acc", |s| s.gps.v_acc.number()),
    ("gpsvel_acc", |s| s.gps.vel_acc.number()),
    ("gpshdg_acc", |s| s.gps.hdg_acc.number()),
    ("gpsyaw", |s| s.gps.yaw.number()),
    ("lat2", |s| s.gps2.position.map_or(0.0, |p| p.latitude())),
    ("lng2", |s| s.gps2.position.map_or(0.0, |p| p.longitude())),
    ("altasl2", |s| s.gps2.altitude_msl.number()),
    ("gpsstatus2", |s| s.gps2.fix_type.number()),
    ("gpshdop2", |s| s.gps2.hdop.number()),
    ("satcount2", |s| s.gps2.satellites_visible.number()),
    ("groundspeed2", |s| s.gps2.ground_speed.number()),
    ("groundcourse2", |s| s.gps2.course.number()),
    ("gpsh_acc2", |s| s.gps2.h_acc.number()),
    ("gpsv_acc2", |s| s.gps2.v_acc.number()),
    ("gpsvel_acc2", |s| s.gps2.vel_acc.number()),
    ("gpshdg_acc2", |s| s.gps2.hdg_acc.number()),
    ("gpsyaw2", |s| s.gps2.yaw.number()),
    ("airspeed", |s| s.air_speed.number()),
    ("lowairspeed", |s| s.low_airspeed.number()),
    ("asratio", |s| s.airspeed_ratio.number()),
    ("airspeed1_temp", |s| s.airspeed1_temp.number()),
    ("airspeed2_temp", |s| s.airspeed2_temp.number()),
    ("groundspeed", |s| s.ground_speed.number()),
    ("ax", |s| s.imu[0].accel[0].number()),
    ("ay", |s| s.imu[0].accel[1].number()),
    ("az", |s| s.imu[0].accel[2].number()),
    ("gx", |s| s.imu[0].gyro[0].number()),
    ("gy", |s| s.imu[0].gyro[1].number()),
    ("gz", |s| s.imu[0].gyro[2].number()),
    ("mx", |s| s.imu[0].mag[0].number()),
    ("my", |s| s.imu[0].mag[1].number()),
    ("mz", |s| s.imu[0].mag[2].number()),
    ("imu1_temp", |s| s.imu[0].temperature.number()),
    ("ax2", |s| s.imu[1].accel[0].number()),
    ("ay2", |s| s.imu[1].accel[1].number()),
    ("az2", |s| s.imu[1].accel[2].number()),
    ("gx2", |s| s.imu[1].gyro[0].number()),
    ("gy2", |s| s.imu[1].gyro[1].number()),
    ("gz2", |s| s.imu[1].gyro[2].number()),
    ("mx2", |s| s.imu[1].mag[0].number()),
    ("my2", |s| s.imu[1].mag[1].number()),
    ("mz2", |s| s.imu[1].mag[2].number()),
    ("imu2_temp", |s| s.imu[1].temperature.number()),
    ("ax3", |s| s.imu[2].accel[0].number()),
    ("ay3", |s| s.imu[2].accel[1].number()),
    ("az3", |s| s.imu[2].accel[2].number()),
    ("gx3", |s| s.imu[2].gyro[0].number()),
    ("gy3", |s| s.imu[2].gyro[1].number()),
    ("gz3", |s| s.imu[2].gyro[2].number()),
    ("mx3", |s| s.imu[2].mag[0].number()),
    ("my3", |s| s.imu[2].mag[1].number()),
    ("mz3", |s| s.imu[2].mag[2].number()),
    ("imu3_temp", |s| s.imu[2].temperature.number()),
    ("hygrotemp1", |s| s.hygrometers[0].temperature.number()),
    ("hygrohumi1", |s| s.hygrometers[0].humidity.number()),
    ("hygrotemp2", |s| s.hygrometers[1].temperature.number()),
    ("hygrohumi2", |s| s.hygrometers[1].humidity.number()),
    ("ch1in", |s| s.rc.values[0].number()),
    ("ch2in", |s| s.rc.values[1].number()),
    ("ch3in", |s| s.rc.values[2].number()),
    ("ch4in", |s| s.rc.values[3].number()),
    ("ch5in", |s| s.rc.values[4].number()),
    ("ch6in", |s| s.rc.values[5].number()),
    ("ch7in", |s| s.rc.values[6].number()),
    ("ch8in", |s| s.rc.values[7].number()),
    ("ch9in", |s| s.rc.values[8].number()),
    ("ch10in", |s| s.rc.values[9].number()),
    ("ch11in", |s| s.rc.values[10].number()),
    ("ch12in", |s| s.rc.values[11].number()),
    ("ch13in", |s| s.rc.values[12].number()),
    ("ch14in", |s| s.rc.values[13].number()),
    ("ch15in", |s| s.rc.values[14].number()),
    ("ch16in", |s| s.rc.values[15].number()),
    ("ch1out", |s| s.servo_outputs[0].number()),
    ("ch2out", |s| s.servo_outputs[1].number()),
    ("ch3out", |s| s.servo_outputs[2].number()),
    ("ch4out", |s| s.servo_outputs[3].number()),
    ("ch5out", |s| s.servo_outputs[4].number()),
    ("ch6out", |s| s.servo_outputs[5].number()),
    ("ch7out", |s| s.servo_outputs[6].number()),
    ("ch8out", |s| s.servo_outputs[7].number()),
    ("ch9out", |s| s.servo_outputs[8].number()),
    ("ch10out", |s| s.servo_outputs[9].number()),
    ("ch11out", |s| s.servo_outputs[10].number()),
    ("ch12out", |s| s.servo_outputs[11].number()),
    ("ch13out", |s| s.servo_outputs[12].number()),
    ("ch14out", |s| s.servo_outputs[13].number()),
    ("ch15out", |s| s.servo_outputs[14].number()),
    ("ch16out", |s| s.servo_outputs[15].number()),
    ("ch17out", |s| s.servo_outputs[16].number()),
    ("ch18out", |s| s.servo_outputs[17].number()),
    ("ch19out", |s| s.servo_outputs[18].number()),
    ("ch20out", |s| s.servo_outputs[19].number()),
    ("ch21out", |s| s.servo_outputs[20].number()),
    ("ch22out", |s| s.servo_outputs[21].number()),
    ("ch23out", |s| s.servo_outputs[22].number()),
    ("ch24out", |s| s.servo_outputs[23].number()),
    ("ch25out", |s| s.servo_outputs[24].number()),
    ("ch26out", |s| s.servo_outputs[25].number()),
    ("ch27out", |s| s.servo_outputs[26].number()),
    ("ch28out", |s| s.servo_outputs[27].number()),
    ("ch29out", |s| s.servo_outputs[28].number()),
    ("ch30out", |s| s.servo_outputs[29].number()),
    ("ch31out", |s| s.servo_outputs[30].number()),
    ("ch32out", |s| s.servo_outputs[31].number()),
    ("esc1_volt", |s| s.escs[0].voltage.number()),
    ("esc1_curr", |s| s.escs[0].current.number()),
    ("esc1_rpm", |s| s.escs[0].rpm.number()),
    ("esc1_temp", |s| s.escs[0].temperature.number()),
    ("esc2_volt", |s| s.escs[1].voltage.number()),
    ("esc2_curr", |s| s.escs[1].current.number()),
    ("esc2_rpm", |s| s.escs[1].rpm.number()),
    ("esc2_temp", |s| s.escs[1].temperature.number()),
    ("esc3_volt", |s| s.escs[2].voltage.number()),
    ("esc3_curr", |s| s.escs[2].current.number()),
    ("esc3_rpm", |s| s.escs[2].rpm.number()),
    ("esc3_temp", |s| s.escs[2].temperature.number()),
    ("esc4_volt", |s| s.escs[3].voltage.number()),
    ("esc4_curr", |s| s.escs[3].current.number()),
    ("esc4_rpm", |s| s.escs[3].rpm.number()),
    ("esc4_temp", |s| s.escs[3].temperature.number()),
    ("esc5_volt", |s| s.escs[4].voltage.number()),
    ("esc5_curr", |s| s.escs[4].current.number()),
    ("esc5_rpm", |s| s.escs[4].rpm.number()),
    ("esc5_temp", |s| s.escs[4].temperature.number()),
    ("esc6_volt", |s| s.escs[5].voltage.number()),
    ("esc6_curr", |s| s.escs[5].current.number()),
    ("esc6_rpm", |s| s.escs[5].rpm.number()),
    ("esc6_temp", |s| s.escs[5].temperature.number()),
    ("esc7_volt", |s| s.escs[6].voltage.number()),
    ("esc7_curr", |s| s.escs[6].current.number()),
    ("esc7_rpm", |s| s.escs[6].rpm.number()),
    ("esc7_temp", |s| s.escs[6].temperature.number()),
    ("esc8_volt", |s| s.escs[7].voltage.number()),
    ("esc8_curr", |s| s.escs[7].current.number()),
    ("esc8_rpm", |s| s.escs[7].rpm.number()),
    ("esc8_temp", |s| s.escs[7].temperature.number()),
    ("esc9_volt", |s| s.escs[8].voltage.number()),
    ("esc9_curr", |s| s.escs[8].current.number()),
    ("esc9_rpm", |s| s.escs[8].rpm.number()),
    ("esc9_temp", |s| s.escs[8].temperature.number()),
    ("esc10_volt", |s| s.escs[9].voltage.number()),
    ("esc10_curr", |s| s.escs[9].current.number()),
    ("esc10_rpm", |s| s.escs[9].rpm.number()),
    ("esc10_temp", |s| s.escs[9].temperature.number()),
    ("esc11_volt", |s| s.escs[10].voltage.number()),
    ("esc11_curr", |s| s.escs[10].current.number()),
    ("esc11_rpm", |s| s.escs[10].rpm.number()),
    ("esc11_temp", |s| s.escs[10].temperature.number()),
    ("esc12_volt", |s| s.escs[11].voltage.number()),
    ("esc12_curr", |s| s.escs[11].current.number()),
    ("esc12_rpm", |s| s.escs[11].rpm.number()),
    ("esc12_temp", |s| s.escs[11].temperature.number()),
    ("esc13_volt", |s| s.escs[12].voltage.number()),
    ("esc13_curr", |s| s.escs[12].current.number()),
    ("esc13_rpm", |s| s.escs[12].rpm.number()),
    ("esc13_temp", |s| s.escs[12].temperature.number()),
    ("esc14_volt", |s| s.escs[13].voltage.number()),
    ("esc14_curr", |s| s.escs[13].current.number()),
    ("esc14_rpm", |s| s.escs[13].rpm.number()),
    ("esc14_temp", |s| s.escs[13].temperature.number()),
    ("esc15_volt", |s| s.escs[14].voltage.number()),
    ("esc15_curr", |s| s.escs[14].current.number()),
    ("esc15_rpm", |s| s.escs[14].rpm.number()),
    ("esc15_temp", |s| s.escs[14].temperature.number()),
    ("esc16_volt", |s| s.escs[15].voltage.number()),
    ("esc16_curr", |s| s.escs[15].current.number()),
    ("esc16_rpm", |s| s.escs[15].rpm.number()),
    ("esc16_temp", |s| s.escs[15].temperature.number()),
    ("ch3percent", |s| s.throttle_percent.number()),
    ("verticalspeed", |s| f64::from(s.vertical_speed())),
    ("nav_roll", |s| s.nav.roll.number()),
    ("nav_pitch", |s| s.nav.pitch.number()),
    ("nav_bearing", |s| s.nav.bearing.number()),
    ("target_bearing", |s| s.nav.target_bearing.number()),
    ("wp_dist", |s| s.nav.wp_distance.number()),
    ("alt_error", |s| s.nav.alt_error.number()),
    ("xtrack_error", |s| s.nav.xtrack_error.number()),
    ("wpno", |s| s.mission_current.number()),
    ("climbrate", |s| s.climb_rate.number()),
    ("distTraveled", |s| f64::from(s.dist_traveled)),
    ("timeSinceArmInAir", |s| f64::from(s.time_since_arm_in_air)),
    ("timeInAir", |s| f64::from(s.time_in_air)),
    ("timeInAirMinSec", |s| f64::from(s.time_in_air_min_sec())),
    ("turnrate", |s| s.turn_rate().number()),
    ("wind_dir", |s| s.wind_direction.number()),
    ("wind_vel", |s| s.wind_speed.number()),
    ("battery_voltage", |s| s.battery.voltage.number()),
    ("battery_voltage3", |s| s.batteries[1].voltage.number()),
    ("battery_voltage4", |s| s.batteries[2].voltage.number()),
    ("battery_voltage5", |s| s.batteries[3].voltage.number()),
    ("battery_voltage6", |s| s.batteries[4].voltage.number()),
    ("battery_voltage7", |s| s.batteries[5].voltage.number()),
    ("battery_voltage8", |s| s.batteries[6].voltage.number()),
    ("battery_voltage9", |s| s.batteries[7].voltage.number()),
    ("battery_remaining", |s| s.battery.remaining_percent.number()),
    ("battery_remaining2", |s| s.batteries[0].remaining_percent.number()),
    ("battery_remaining3", |s| s.batteries[1].remaining_percent.number()),
    ("battery_remaining4", |s| s.batteries[2].remaining_percent.number()),
    ("battery_remaining5", |s| s.batteries[3].remaining_percent.number()),
    ("battery_remaining6", |s| s.batteries[4].remaining_percent.number()),
    ("battery_remaining7", |s| s.batteries[5].remaining_percent.number()),
    ("battery_remaining8", |s| s.batteries[6].remaining_percent.number()),
    ("battery_remaining9", |s| s.batteries[7].remaining_percent.number()),
    ("current", |s| s.battery.current.number()),
    ("current2", |s| s.batteries[0].current.number()),
    ("current3", |s| s.batteries[1].current.number()),
    ("current4", |s| s.batteries[2].current.number()),
    ("current5", |s| s.batteries[3].current.number()),
    ("current6", |s| s.batteries[4].current.number()),
    ("current7", |s| s.batteries[5].current.number()),
    ("current8", |s| s.batteries[6].current.number()),
    ("current9", |s| s.batteries[7].current.number()),
    ("battery_mahperkm", VehicleState::battery_mah_per_km),
    ("battery_kmleft", VehicleState::battery_km_left),
    ("battery_usedmah", |s| s.battery.consumed_mah.number()),
    ("battery_cell1", |s| s.battery.cells[0].number()),
    ("battery_cell2", |s| s.battery.cells[1].number()),
    ("battery_cell3", |s| s.battery.cells[2].number()),
    ("battery_cell4", |s| s.battery.cells[3].number()),
    ("battery_cell5", |s| s.battery.cells[4].number()),
    ("battery_cell6", |s| s.battery.cells[5].number()),
    ("battery_cell7", |s| s.battery.cells[6].number()),
    ("battery_cell8", |s| s.battery.cells[7].number()),
    ("battery_cell9", |s| s.battery.cells[8].number()),
    ("battery_cell10", |s| s.battery.cells[9].number()),
    ("battery_cell11", |s| s.battery.cells[10].number()),
    ("battery_cell12", |s| s.battery.cells[11].number()),
    ("battery_cell13", |s| s.battery.cells[12].number()),
    ("battery_cell14", |s| s.battery.cells[13].number()),
    ("battery_temp", |s| s.battery.temperature.number()),
    ("battery_temp2", |s| s.batteries[0].temperature.number()),
    ("battery_temp3", |s| s.batteries[1].temperature.number()),
    ("battery_temp4", |s| s.batteries[2].temperature.number()),
    ("battery_temp5", |s| s.batteries[3].temperature.number()),
    ("battery_temp6", |s| s.batteries[4].temperature.number()),
    ("battery_temp7", |s| s.batteries[5].temperature.number()),
    ("battery_temp8", |s| s.batteries[6].temperature.number()),
    ("battery_temp9", |s| s.batteries[7].temperature.number()),
    ("battery_remainmin", |s| s.battery.remaining_minutes.number()),
    ("battery_remainmin2", |s| s.batteries[0].remaining_minutes.number()),
    ("battery_remainmin3", |s| s.batteries[1].remaining_minutes.number()),
    ("battery_remainmin4", |s| s.batteries[2].remaining_minutes.number()),
    ("battery_remainmin5", |s| s.batteries[3].remaining_minutes.number()),
    ("battery_remainmin6", |s| s.batteries[4].remaining_minutes.number()),
    ("battery_remainmin7", |s| s.batteries[5].remaining_minutes.number()),
    ("battery_remainmin8", |s| s.batteries[6].remaining_minutes.number()),
    ("battery_remainmin9", |s| s.batteries[7].remaining_minutes.number()),
    ("battery_usedmah2", |s| s.batteries[0].consumed_mah.number()),
    ("battery_usedmah3", |s| s.batteries[1].consumed_mah.number()),
    ("battery_usedmah4", |s| s.batteries[2].consumed_mah.number()),
    ("battery_usedmah5", |s| s.batteries[3].consumed_mah.number()),
    ("battery_usedmah6", |s| s.batteries[4].consumed_mah.number()),
    ("battery_usedmah7", |s| s.batteries[5].consumed_mah.number()),
    ("battery_usedmah8", |s| s.batteries[6].consumed_mah.number()),
    ("battery_usedmah9", |s| s.batteries[7].consumed_mah.number()),
    ("battery_voltage2", |s| s.batteries[0].voltage.number()),
    ("HomeAlt", |s| s.home_altitude.number()),
    // The fence is the planning screen's, not the state's: 99999 until row 39 hands it over.
    ("GeoFenceDist", |s| f64::from(s.geo_fence_dist(&[]))),
    ("DistFromMovingBase", |s| f64::from(s.dist_from_moving_base())),
    ("sonarrange", |s| s.rangefinder.range.number()),
    ("sonarvoltage", |s| s.rangefinder.voltage.number()),
    ("rangefinder1", |s| s.rangefinder.distances[0].number()),
    ("rangefinder2", |s| s.rangefinder.distances[1].number()),
    ("rangefinder3", |s| s.rangefinder.distances[2].number()),
    ("rangefinder4", |s| s.rangefinder.distances[3].number()),
    ("rangefinder5", |s| s.rangefinder.distances[4].number()),
    ("rangefinder6", |s| s.rangefinder.distances[5].number()),
    ("rangefinder7", |s| s.rangefinder.distances[6].number()),
    ("rangefinder8", |s| s.rangefinder.distances[7].number()),
    ("rangefinder9", |s| s.rangefinder.distances[8].number()),
    ("rangefinder10", |s| s.rangefinder.distances[9].number()),
    ("freemem", |s| s.board.free_memory.number()),
    ("load", |s| s.load.number()),
    ("brklevel", |s| s.board.brk_level.number()),
    ("armed", |s| s.armed.number()),
    ("rssi", |s| s.radio.rssi.number()),
    ("remrssi", |s| s.radio.remrssi.number()),
    ("txbuffer", |s| s.radio.txbuf.number()),
    ("noise", |s| s.radio.noise.number()),
    ("remnoise", |s| s.radio.remnoise.number()),
    ("rxerrors", |s| s.radio.rxerrors.number()),
    ("fixedp", |s| s.radio.fixed.number()),
    ("packetdropremote", |s| s.packet_drop_remote.number()),
    ("linkqualitygcs", |s| s.link.quality_percent().number()),
    ("errors_count1", |s| s.errors_count[0].number()),
    ("errors_count2", |s| s.errors_count[1].number()),
    ("errors_count3", |s| s.errors_count[2].number()),
    ("errors_count4", |s| s.errors_count[3].number()),
    ("hwvoltage", |s| s.board.hw_voltage.number()),
    ("boardvoltage", |s| s.board.board_voltage.number()),
    ("servovoltage", |s| s.board.servo_voltage.number()),
    ("voltageflag", |s| s.board.voltage_flags.number()),
    ("i2cerrors", |s| s.board.i2c_errors.number()),
    ("timesincelastshot", |s| s.time_since_last_shot),
    ("press_abs", |s| s.press_abs.number()),
    ("press_temp", |s| s.press_temp.number()),
    ("press_abs2", |s| s.press_abs2.number()),
    ("press_temp2", |s| s.press_temp2.number()),
    ("rateattitude", |s| f64::from(s.rates.attitude)),
    ("rateposition", |s| f64::from(s.rates.position)),
    ("ratestatus", |s| f64::from(s.rates.status)),
    ("ratesensors", |s| f64::from(s.rates.sensors)),
    ("raterc", |s| f64::from(s.rates.rc)),
    ("campointa", |s| s.mount.pointing_a.number()),
    ("campointb", |s| s.mount.pointing_b.number()),
    ("campointc", |s| s.mount.pointing_c.number()),
    ("gimballat", |s| f64::from(s.gimbal_lat())),
    ("gimballng", |s| f64::from(s.gimbal_lng())),
    ("safetyactive", |s| s.sensors.safety_active().number()),
    ("terrainactive", |s| s.sensors.terrain_active().number()),
    ("ter_curalt", |s| s.terrain.current_height.number()),
    ("ter_alt", |s| s.terrain.terrain_height.number()),
    ("ter_load", |s| s.terrain.loaded.number()),
    ("ter_pend", |s| s.terrain.pending.number()),
    ("ter_space", |s| s.terrain.spacing.number()),
    ("KIndex", |_| f64::from(VehicleState::kindex())),
    ("opt_m_x", |s| s.optical_flow.comp_m_x.number()),
    ("opt_m_y", |s| s.optical_flow.comp_m_y.number()),
    ("opt_x", |s| s.optical_flow.x.number()),
    ("opt_y", |s| s.optical_flow.y.number()),
    ("opt_qua", |s| s.optical_flow.quality.number()),
    ("ekfstatus", |s| s.ekf_status().number()),
    ("ekfflags", |s| s.ekf.flags.number()),
    ("ekfvelv", |s| s.ekf.velocity_variance.number()),
    ("ekfcompv", |s| s.ekf.compass_variance.number()),
    ("ekfposhor", |s| s.ekf.position_horizontal_variance.number()),
    ("ekfposvert", |s| s.ekf.position_vertical_variance.number()),
    ("ekfteralt", |s| s.ekf.terrain_altitude_variance.number()),
    ("pidff", |s| s.pid.ff.number()),
    ("pidP", |s| s.pid.p.number()),
    ("pidI", |s| s.pid.i.number()),
    ("pidD", |s| s.pid.d.number()),
    ("pidaxis", |s| s.pid.axis.number()),
    ("piddesired", |s| s.pid.desired.number()),
    ("pidachieved", |s| s.pid.achieved.number()),
    ("pidSRate", |s| s.pid.srate.number()),
    ("pidPDmod", |s| s.pid.pdmod.number()),
    ("pidSRateRoll", |s| s.pid.srate_roll.number()),
    ("pidSRatePitch", |s| s.pid.srate_pitch.number()),
    ("pidSRateYaw", |s| s.pid.srate_yaw.number()),
    ("pidSRateAccZ", |s| s.pid.srate_accz.number()),
    ("pidSRateSteer", |s| s.pid.srate_steer.number()),
    ("pidSRateLanding", |s| s.pid.srate_landing.number()),
    ("vibeclip0", |s| s.vibration.clipping[0].number()),
    ("vibeclip1", |s| s.vibration.clipping[1].number()),
    ("vibeclip2", |s| s.vibration.clipping[2].number()),
    ("vibex", |s| s.vibration.x.number()),
    ("vibey", |s| s.vibration.y.number()),
    ("vibez", |s| s.vibration.z.number()),
    ("uid", |s| s.autopilot_info.uid.number()),
    ("rpm1", |s| s.rpm[0].number()),
    ("rpm2", |s| s.rpm[1].number()),
    ("capabilities", |s| s.autopilot_info.capabilities.number()),
    ("speedup", |s| f64::from(s.speedup)),
    ("vtol_state", |s| s.vtol_state.number()),
    ("landed_state", |s| s.landed_state.number()),
    ("gen_status", |s| s.generator.status.number()),
    ("gen_speed", |s| s.generator.speed.number()),
    ("gen_current", |s| s.generator.current.number()),
    ("gen_voltage", |s| s.generator.voltage.number()),
    ("gen_runtime", |s| s.generator.runtime.number()),
    ("gen_maint_time", |s| s.generator.maintenance_in.number()),
    ("efi_baro", |s| s.efi.baro.number()),
    ("efi_headtemp", |s| s.efi.head_temperature.number()),
    ("efi_load", |s| s.efi.load.number()),
    ("efi_health", |s| s.efi.health.number()),
    ("efi_exhasttemp", |s| s.efi.exhaust_temperature.number()),
    ("efi_intaketemp", |s| s.efi.intake_temperature.number()),
    ("efi_rpm", |s| s.efi.rpm.number()),
    ("efi_fuelflow", |s| s.efi.fuel_flow.number()),
    ("efi_fuelconsumed", |s| s.efi.fuel_consumed.number()),
    ("efi_fuelpressure", |s| s.efi.fuel_pressure.number()),
    ("xpdr_es1090_tx_enabled", |s| s.transponder.es1090_tx_enabled.number()),
    ("xpdr_mode_S_enabled", |s| s.transponder.mode_s_enabled.number()),
    ("xpdr_mode_C_enabled", |s| s.transponder.mode_c_enabled.number()),
    ("xpdr_mode_A_enabled", |s| s.transponder.mode_a_enabled.number()),
    ("xpdr_ident_active", |s| s.transponder.ident_active.number()),
    ("xpdr_x_bit_status", |s| s.transponder.x_bit_status.number()),
    ("xpdr_interrogated_since_last", |s| s.transponder.interrogated_since_last.number()),
    ("xpdr_airborne_status", |s| s.transponder.airborne.number()),
    ("xpdr_mode_A_squawk_code", |s| s.transponder.squawk.number()),
    ("xpdr_nic", |s| s.transponder.nic.number()),
    ("xpdr_nacp", |s| s.transponder.nacp.number()),
    ("xpdr_board_temperature", |s| s.transponder.board_temperature.number()),
    ("xpdr_maint_req", |s| s.transponder.maintenance_required.number()),
    ("xpdr_adsb_tx_sys_fail", |s| s.transponder.adsb_tx_failure.number()),
    ("xpdr_gps_unavail", |s| s.transponder.gps_unavailable.number()),
    ("xpdr_gps_no_fix", |s| s.transponder.gps_no_fix.number()),
    ("xpdr_status_unavail", |s| s.transponder.status_unavailable.number()),
    ("xpdr_status_pending", |s| s.transponder.status_pending.number()),
    ("ahrs2_roll", |s| s.ahrs2.roll.number()),
    ("ahrs2_pitch", |s| s.ahrs2.pitch.number()),
    ("ahrs2_yaw", |s| s.ahrs2.yaw.number()),
    ("ahrs2_alt", |s| s.ahrs2.altitude.number()),
    ("ahrs2_lat", |s| s.ahrs2.lat.number()),
    ("ahrs2_lng", |s| s.ahrs2.lng.number()),
    ("mcumaxvolt", |s| s.mcu.voltage_max.number()),
    ("mcuminvolt", |s| s.mcu.voltage_min.number()),
    ("mcuvoltage", |s| s.mcu.voltage.number()),
    ("mcutemp", |s| s.mcu.temperature.number()),
    ("fenceb_count", |s| s.fence_breach.count.number()),
    ("fenceb_status", |s| s.fence_breach.status.number()),
    ("fenceb_type", |s| s.fence_breach.breach_type.number()),
    ("posn", |s| s.local_position[0].number()),
    ("pose", |s| s.local_position[1].number()),
    ("posd", |s| s.local_position[2].number()),
];

/// Two of the defaults are not held but computed from what is, the way their rows say; a view
/// bound to one shows it rather than nothing.
const DERIVED: &[(&str, Reader)] = &[
    // `yaw`'s setter adds 360 to a negative angle. `// C#: ExtLibs/ArduPilot/CurrentState.cs:274-283`
    ("yaw", |s| {
        let degrees = s.attitude.yaw.0.to_degrees();
        if degrees < 0.0 {
            degrees + 360.0
        } else {
            degrees
        }
    }),
    ("DistToHome", dist_to_home),
];

/// `DistToHome`: from home to the vehicle on the C#'s flat projection, 0 without either.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1764-1782`
fn dist_to_home(s: &VehicleState) -> f64 {
    let (Some(home), Some(here)) = (s.home, s.position) else {
        return 0.0;
    };
    if here.latitude() == 0.0 && here.longitude() == 0.0 || home.latitude() == 0.0 {
        return 0.0;
    }
    let rads = home.latitude().abs() * 0.017_453_292_5;
    let scale_long_down = rads.cos();
    let dstlat = (home.latitude() - here.latitude()).abs() * 111_319.5;
    let dstlon = (home.longitude() - here.longitude()).abs() * 111_319.5 * scale_long_down;
    // `(float)Math.Sqrt(...) * multiplierdist`, with metres as the unit.
    #[allow(clippy::cast_possible_truncation)]
    let single = dstlat.hypot(dstlon) as f32;
    f64::from(single)
}

/// The C# members the chooser leaves out although they are held: public fields, which
/// `Type.GetProperties` does not return. `// C#: ExtLibs/ArduPilot/CurrentState.cs:119`
const FIELDS_NOT_PROPERTIES: &[&str] = &["hilch5", "hilch6", "hilch7", "hilch8", "lastautowp"];

/// What a view bound to `name` shows now, or `None` for a property this application does not
/// hold.
#[must_use]
pub fn value(name: &str, state: &VehicleState) -> Option<f64> {
    READERS
        .iter()
        .chain(DERIVED)
        .find(|(field, _)| *field == name)
        .map(|(_, read)| read(state))
}

/// The view's description: `GetNameandUnit`. The property's `[DisplayText]`, or its name when
/// it has none, with the first of `(dist)`, `(speed)` and `(alt)` it holds replaced by the unit -
/// metres and metres per second, the units `ChangeUnits` sets when nothing is configured.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4526-4552, MainV2.cs:4247-4330`
#[must_use]
pub fn label(name: &str) -> String {
    let desc = CURRENTSTATE
        .iter()
        .find(|field| field.name == name && !field.display.is_empty())
        .map_or(name, |field| field.display);
    let units = mp_vehicle::units::DisplayUnits::default().change_units(None, None, None);
    if desc.contains("(dist)") {
        desc.replace("(dist)", &format!("({})", units.dist_unit))
    } else if desc.contains("(speed)") {
        desc.replace("(speed)", &format!("({})", units.speed_unit))
    } else if desc.contains("(alt)") {
        desc.replace("(alt)", &format!("({})", units.alt_unit))
    } else {
        desc.to_owned()
    }
}

/// Every property the chooser offers, in its order.
///
/// Each check box's text is `GetFieldDesc(name)`: the `[DisplayFieldName]` translation, which
/// an English culture does not have, so the name itself. The list is sorted on that text with
/// `string.CompareTo`, a culture comparison; this sorts ignoring case first and puts lower case
/// first on a tie, which is how that comparison orders these names.
/// `// C#: GCSViews/FlightData.cs:4549-4601, ExtLibs/ArduPilot/CurrentState.cs:4488-4524`
#[must_use]
pub fn choices() -> Vec<&'static str> {
    let mut names: Vec<&'static str> = CURRENTSTATE
        .iter()
        .filter(|field| matches!(field.ours, Ours::Done(_)))
        .filter(|field| !FIELDS_NOT_PROPERTIES.contains(&field.name))
        .map(|field| field.name)
        .filter(|name| READERS.iter().any(|(held, _)| held == name))
        .collect();
    names.sort_by(|a, b| {
        a.to_lowercase()
            .cmp(&b.to_lowercase())
            .then_with(|| b.cmp(a))
    });
    names
}

/// The number as `numberformat` "0.00" writes it: two places, halves away from zero.
#[must_use]
pub fn number_text(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    let text = format!("{rounded:.2}");
    // "-0.00" is "0.00" in .NET.
    if text == "-0.00" {
        "0.00".to_owned()
    } else {
        text
    }
}

/// The number's font size in a view `width` by `height`: `QuickView.OnPaintSurface`.
///
/// It measures one more zero than the number has characters and scales the font until that fits
/// both the width and the height under the description, then takes the size down to a multiple
/// of five, and uses 8 for anything under 8. `zero_width` is one zero's width at a size of 1,
/// from the text system.
/// `// C#: ExtLibs/Controls/QuickView.cs:72-104`
#[must_use]
pub fn number_size(width: f32, height: f32, characters: usize, zero_width: f32) -> f32 {
    let below = height - DESC_SIZE * LINE;
    #[allow(clippy::cast_precision_loss)] // a handful of characters
    let wide = (characters + 1) as f32 * zero_width;
    let by_height = below / LINE;
    let by_width = if wide > 0.0 { width / wide } else { by_height };
    let mut size = by_height.min(by_width);
    size -= size % 5.0;
    if !(8.0..=999_999.0).contains(&size) {
        size = 8.0;
    }
    size
}

/// The six views' properties, and the view whose chooser is open.
#[derive(Debug, Clone)]
pub struct QuickViews {
    fields: [String; 6],
    choosing: Option<usize>,
}

impl Default for QuickViews {
    fn default() -> Self {
        Self {
            fields: DEFAULTS.map(|(name, _)| name.to_owned()),
            choosing: None,
        }
    }
}

impl QuickViews {
    /// The property view `index` (0 to 5) shows.
    #[must_use]
    pub fn field(&self, index: usize) -> &str {
        self.fields.get(index).map_or("", String::as_str)
    }

    /// `quickView_DoubleClick`: opens the chooser for a view.
    pub fn open(&mut self, index: usize) {
        if index < self.fields.len() {
            self.choosing = Some(index);
        }
    }

    /// The view being chosen for, if the chooser is open.
    #[must_use]
    pub const fn choosing(&self) -> Option<usize> {
        self.choosing
    }

    /// `chk_box_quickview_CheckedChanged`: a box checked binds the view to its property and
    /// closes the chooser. The box already checked is the view's own property; clicking it
    /// unchecks it, which does nothing, and the chooser stays.
    /// `// C#: GCSViews/FlightData.cs:2474-2506`
    pub fn choose(&mut self, name: &str) {
        let Some(index) = self.choosing else {
            return;
        };
        if self.field(index) == name {
            return;
        }
        if let Some(field) = self.fields.get_mut(index) {
            name.clone_into(field);
        }
        self.choosing = None;
    }

    /// Closes the chooser without choosing: the form's close box.
    pub fn close(&mut self) {
        self.choosing = None;
    }

    /// Publishes what a UI test asserts on: each view's property, description and number, and
    /// the chooser.
    pub fn record_facts(&self, state: Option<&VehicleState>) {
        for (index, field) in self.fields.iter().enumerate() {
            let key = format!("fly.quick.{}", index + 1);
            crate::facts::record(&key, field);
            crate::facts::record(format!("{key}.label"), label(field));
            crate::facts::record(
                format!("{key}.value"),
                state
                    .and_then(|state| value(field, state))
                    .map_or_else(|| "none".to_owned(), number_text),
            );
        }
        crate::facts::record(
            "fly.quick.chooser",
            self.choosing
                .map_or_else(|| "none".to_owned(), |index| (index + 1).to_string()),
        );
        crate::facts::record("fly.quick.choices", choices().len());
    }
}

/// The Quick page: the six views in `tableLayoutPanelQuick`'s two columns and three rows, each
/// a third of the page's height and half its width.
pub fn page(
    views: &QuickViews,
    state: Option<&VehicleState>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    // Named as the panels are, so a layout test finds the page's bottom edge.
    let mut grid = crate::probe::measured("panel:quick", div())
        .grid()
        .grid_cols(2)
        .grid_rows(3)
        .gap_1()
        .flex_1()
        .min_h(px(180.0));
    for (index, (column, row)) in CELLS.iter().enumerate() {
        let field = views.field(index).to_owned();
        let colour = DEFAULTS
            .get(index)
            .map_or(theme::TEXT, |(_, colour)| *colour);
        let desc = label(&field);
        let number = state
            .and_then(|state| value(&field, state))
            .map_or_else(|| "--".to_owned(), number_text);
        let id = IDS.get(index).copied().unwrap_or("fly-quick");
        grid = grid.child(
            crate::probe::measured(id, div())
                .id(id)
                .col_start(i16::from(*column) + 1)
                .row_start(i16::from(*row) + 1)
                .min_w(px(0.0))
                .min_h(px(0.0))
                .relative()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_sm()
                .cursor_pointer()
                .child(
                    gpui::canvas(
                        |_bounds, _window, _cx| (),
                        move |bounds, (), window, cx| {
                            paint_view(&desc, &number, colour, bounds, window, cx);
                        },
                    )
                    .absolute()
                    .inset_0(),
                )
                // `DoubleClick`, which Windows raises on the second press.
                .on_mouse_down(
                    gpui::MouseButton::Left,
                    cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                        if event.click_count == 2 {
                            this.fly_data.quick.open(index);
                            cx.notify();
                        }
                    }),
                ),
        );
    }
    grid.into_any_element()
}

/// One view painted as `OnPaintSurface` paints it: the description centred at the top, the
/// number centred in what is left, in its colour.
fn paint_view(
    desc: &str,
    number: &str,
    colour: u32,
    bounds: gpui::Bounds<gpui::Pixels>,
    window: &mut Window,
    cx: &mut gpui::App,
) {
    use crate::hud::{Align, Item, Scene};
    let width = f32::from(bounds.size.width);
    let height = f32::from(bounds.size.height);
    let zero_width = {
        let run = gpui::TextRun {
            len: 1,
            font: window.text_style().font(),
            color: gpui::Hsla::from(rgb(colour)),
            background_color: None,
            underline: None,
            strikethrough: None,
        };
        let line =
            window
                .text_system()
                .shape_line(gpui::SharedString::from("0"), px(100.0), &[run], None);
        f32::from(line.width()) / 100.0
    };
    let size = number_size(width, height, number.chars().count(), zero_width);
    let top = DESC_SIZE * LINE;
    let scene = Scene {
        items: vec![
            Item::Label {
                text: desc.to_owned(),
                at: (width / 2.0, 5.0),
                size: DESC_SIZE,
                colour: theme::TEXT,
                align: Align::Centre,
            },
            Item::Label {
                text: number.to_owned(),
                at: (width / 2.0, top + (height - top) / 2.0 - size * LINE / 2.0),
                size,
                colour,
                align: Align::Centre,
            },
        ],
        drawn: Vec::new(),
        owners: Vec::new(),
    };
    crate::hud::paint(&scene, bounds, window, cx);
}

/// The chooser, `selectform`: "Display This", every property as a check box in columns, the
/// view's own checked and green, as `ShowDialog` shows it - over everything, and everything
/// behind it inert. The columns are as wide as the longest text and 25 more, as many as fit in
/// four fifths of the window, filled top to bottom and then left to right.
/// `// C#: GCSViews/FlightData.cs:4549-4650`
pub fn chooser(
    views: &QuickViews,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let index = views.choosing()?;
    let current = views.field(index).to_owned();
    let names = choices();
    let size = window.viewport_size();
    let width = f32::from(size.width);
    // `TextRenderer.MeasureText` of the longest, at the form's 8.25pt font: about seven pixels a
    // character.
    #[allow(clippy::cast_precision_loss)]
    let max_length = names.iter().map(|name| name.len()).max().unwrap_or(1) as f32 * 7.0 + 25.0;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let columns = ((width * 0.8) / max_length).max(1.0) as usize;
    let rows = names.len().div_ceil(columns);

    let mut table = div().flex().gap_1();
    for column in names.chunks(rows.max(1)) {
        let mut list = div().flex().flex_col().w(px(max_length));
        for name in column {
            let checked = *name == current;
            let id = format!("fly-quick-choice-{name}");
            let chosen = (*name).to_owned();
            list = list.child(
                crate::probe::measured(id.clone(), div())
                    .id(gpui::SharedString::from(id))
                    .flex()
                    .items_center()
                    .gap_1()
                    .h(px(20.0))
                    .px_1()
                    .text_xs()
                    .cursor_pointer()
                    .bg(rgb(if checked { 0x00_80_00 } else { theme::PANEL }))
                    .text_color(rgb(theme::TEXT))
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(if checked { "\u{2611}" } else { "\u{2610}" })
                    .child((*name).to_owned())
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.fly_data.quick.choose(&chosen);
                        cx.notify();
                    })),
            );
        }
        table = table.child(list);
    }

    let dialog = crate::probe::measured("fly-quick-chooser", div())
        .id("fly-quick-chooser")
        .flex()
        .flex_col()
        .gap_2()
        .w(size.width - px(100.0))
        .h(size.height - px(100.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .child(
            div()
                .flex()
                .justify_between()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child("Display This"),
                )
                .child(action(
                    "fly-quick-close",
                    "\u{2715}",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.fly_data.quick.close();
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .id("fly-quick-choices")
                .flex_1()
                .min_h(px(0.0))
                .overflow_y_scroll()
                .child(table),
        );

    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("fly-quick-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(dialog),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn csharp(path: &str) -> Option<String> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../referneces/missionplanner")
            .join(path);
        std::fs::read_to_string(path).ok()
    }

    /// Every row of the table that is held, numeric and a property has a reader, and nothing
    /// else does - so the chooser is exactly the done rows.
    #[test]
    fn every_done_numeric_property_has_a_reader_and_nothing_else_does() {
        const NUMERIC: &[&str] = &[
            "float", "double", "bool", "uint", "int", "ushort", "byte", "short", "ulong",
        ];
        let wanted: Vec<&str> = CURRENTSTATE
            .iter()
            .filter(|field| matches!(field.ours, Ours::Done(_)))
            .filter(|field| NUMERIC.contains(&field.ty))
            .filter(|field| !FIELDS_NOT_PROPERTIES.contains(&field.name))
            .map(|field| field.name)
            .collect();
        let held: Vec<&str> = READERS.iter().map(|(name, _)| *name).collect();
        assert_eq!(held, wanted);
        assert_eq!(choices().len(), wanted.len());
        // The two computed defaults are not done rows.
        for (name, _) in DERIVED {
            let row = CURRENTSTATE.iter().find(|field| field.name == *name);
            assert!(
                matches!(row.map(|row| row.ours), Some(Ours::Derived(_))),
                "{name}"
            );
        }
    }

    /// The members left out for being fields are the held ones the class declares as fields.
    #[test]
    fn the_fields_left_out_are_the_ones_the_class_declares_as_fields() {
        let Some(source) = csharp("ExtLibs/ArduPilot/CurrentState.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let mut fields: Vec<&str> = Vec::new();
        for line in source.lines() {
            let line = line.trim();
            let Some(rest) = line.strip_prefix("public ") else {
                continue;
            };
            if rest.starts_with("static ") || rest.contains('(') || rest.contains("=>") {
                continue;
            }
            let mut words = rest.split_whitespace();
            let (Some(_ty), Some(name)) = (words.next(), words.next()) else {
                continue;
            };
            let name = name.trim_end_matches(';');
            if (rest.trim_end().ends_with(';') || rest.contains(" = "))
                && !rest.contains('{')
                && CURRENTSTATE
                    .iter()
                    .any(|field| field.name == name && matches!(field.ours, Ours::Done(_)))
            {
                fields.push(name);
            }
        }
        fields.sort_unstable();
        fields.dedup();
        assert_eq!(fields, FIELDS_NOT_PROPERTIES);
    }

    /// The six are the Designer's, bound to the properties it binds.
    #[test]
    fn the_views_start_bound_as_the_designer_binds_them() {
        let Some(designer) = csharp("GCSViews/FlightData.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        for (index, (name, _)) in DEFAULTS.iter().enumerate() {
            let binding = format!(
                "this.quickView{}.DataBindings.Add(new System.Windows.Forms.Binding(\"number\", this.bindingSourceQuickTab, \"{name}\", true));",
                index + 1
            );
            assert!(designer.contains(&binding), "{binding}");
        }
        let views = QuickViews::default();
        assert_eq!(views.field(0), "alt");
        assert_eq!(views.field(5), "DistToHome");
    }

    #[test]
    fn a_description_is_the_display_text_with_its_unit() {
        assert_eq!(label("alt"), "Altitude (m)");
        assert_eq!(label("groundspeed"), "GroundSpeed (m/s)");
        assert_eq!(label("wp_dist"), "Dist to WP (m)");
        assert_eq!(label("yaw"), "Yaw (deg)");
        assert_eq!(label("verticalspeed"), "Vertical Speed (m/s)");
        assert_eq!(label("DistToHome"), "Dist to Home (m)");
        // No display text: the name.
        let bare = CURRENTSTATE
            .iter()
            .find(|field| field.display.is_empty() && matches!(field.ours, Ours::Done(_)))
            .map(|field| field.name)
            .expect("a done row without display text");
        assert_eq!(label(bare), bare);
    }

    #[test]
    fn the_views_read_the_vehicle() {
        let mut state = VehicleState::default();
        state.altitude_relative = Metres(12.345);
        state.ground_speed = MetresPerSecond(3.0);
        state.nav.wp_distance = 40.0;
        state.attitude.yaw = mp_units::Radians(-std::f64::consts::FRAC_PI_2);
        state.armed = true;
        assert_eq!(value("alt", &state), Some(12.345));
        assert_eq!(value("groundspeed", &state), Some(3.0));
        assert_eq!(value("wp_dist", &state), Some(40.0));
        // -90 degrees is 270, as the C#'s setter makes it.
        assert!((value("yaw", &state).unwrap() - 270.0).abs() < 1e-9);
        // A bool binds as 1.
        assert_eq!(value("armed", &state), Some(1.0));
        // Held now: the C#'s vertical speed, 0 before the alt setter has run twice.
        assert_eq!(value("verticalspeed", &state), Some(0.0));
        // Not held: no value, rather than a made-up one.
        assert_eq!(value("lastautowp", &state), None);
        // No home: 0, as the C# returns.
        assert_eq!(value("DistToHome", &state), Some(0.0));
        state.home = Some(mp_units::LatLon::new(-35.0, 149.0).unwrap());
        state.position = Some(mp_units::LatLon::new(-35.001, 149.0).unwrap());
        let metres = value("DistToHome", &state).unwrap();
        assert!((metres - 111.3195).abs() < 0.01, "{metres}");
    }

    #[test]
    fn a_number_is_written_to_two_places() {
        assert_eq!(number_text(12.345), "12.35");
        assert_eq!(number_text(0.0), "0.00");
        assert_eq!(number_text(-0.001), "0.00");
        assert_eq!(number_text(-2.5), "-2.50");
        assert_eq!(number_text(270.0), "270.00");
    }

    #[test]
    fn the_number_fills_its_view_in_steps_of_five() {
        // A wide, short view: the height decides. 60 high, 15.6 of description: 44.4 over 1.2 is
        // 37, down to 35.
        assert_eq!(number_size(400.0, 60.0, 4, 0.55), 35.0);
        // A narrow one: the width decides. "12.35" and one more zero, six at 0.55: 3.3 a size,
        // 160 / 3.3 = 48.5, down to 45.
        assert_eq!(number_size(160.0, 200.0, 5, 0.55), 45.0);
        // Too small for 8: 8.
        assert_eq!(number_size(20.0, 20.0, 5, 0.55), 8.0);
    }

    #[test]
    fn the_chooser_binds_and_closes_and_its_own_box_does_nothing() {
        let mut views = QuickViews::default();
        views.open(2);
        assert_eq!(views.choosing(), Some(2));
        views.choose("wp_dist");
        assert_eq!(views.choosing(), Some(2));
        views.choose("battery_voltage");
        assert_eq!(views.choosing(), None);
        assert_eq!(views.field(2), "battery_voltage");
        assert_eq!(label("battery_voltage"), "Bat Voltage (V)");
        views.open(9);
        assert_eq!(views.choosing(), None);
    }

    #[test]
    fn the_chooser_is_sorted_as_the_csharp_sorts_its_text() {
        let names = choices();
        assert!(names.contains(&"alt"));
        assert!(!names.contains(&"lastautowp"));
        for pair in names.windows(2) {
            assert!(
                pair[0].to_lowercase() <= pair[1].to_lowercase(),
                "{} before {}",
                pair[0],
                pair[1]
            );
        }
    }
}
