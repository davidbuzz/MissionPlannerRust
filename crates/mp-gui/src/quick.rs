//! The flight screen's Quick page: `quickView1` to `quickView6`, the views Set View Count adds
//! or takes away, and the chooser a double click on one opens.
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
//! The C# keeps the choice in its settings, `Settings.Instance["quickView" + n]`, and the grid's
//! size in `quickViewCols` and `quickViewRows`; `settings.rs` writes and restores both from what
//! [`QuickViews`] holds.
//!
//! Set View Count's `setQuickViewRowsCols` makes the grid any number of columns and rows: views
//! whose cell falls outside are removed, and new ones, bound to nothing and showing 0, fill it up.
//! `// C#: GCSViews/FlightData.cs:4914-5060`
//!
//! # Units
//!
//! The vehicle's state is SI. The C#'s `CurrentState` getters multiply a dozen properties by the
//! user's `multiplierdist`, `multiplieralt` or `multiplierspeed` before a binding reads them, so
//! a view shows those in feet or knots where the Planner page says so; [`IN_DISPLAY_UNITS`] is
//! that list, held to the getters by a test that reads `CurrentState.cs`. The description names
//! the unit through `GetNameandUnit`. [`QuickViews`] holds the units the flight screen hands it.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, Window, div, prelude::*, px, rgb};
use mp_units::{Metres, MetresPerSecond};
use mp_vehicle::VehicleState;
use mp_vehicle::coverage::{CURRENTSTATE, Ours};
use mp_vehicle::units::DisplayUnits;

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

/// The Designer's six views' ids: [`view_id`]'s first six, spelt out for the coverage table.
const IDS: [&str; 6] = [
    "fly-quick-1",
    "fly-quick-2",
    "fly-quick-3",
    "fly-quick-4",
    "fly-quick-5",
    "fly-quick-6",
];

/// The id a script clicks a view by: `fly-quick-` and its place in `tableLayoutPanelQuick.Controls`
/// from 1, which for the Designer's six is the number in their names.
#[must_use]
pub fn view_id(index: usize) -> String {
    IDS.get(index)
        .map_or_else(|| format!("fly-quick-{}", index + 1), |id| (*id).to_owned())
}

/// `colorsForDefaultQuickView`: the number colours a view Set View Count adds is given one of -
/// `Blue`, `Yellow`, `Pink`, `LimeGreen`, `Orange`, `Aqua`, `LightCoral`, `LightSteelBlue`,
/// `DarkKhaki`, `LightYellow`, `Violet`, `YellowGreen`, `OrangeRed`, `Tomato`, `Teal` and
/// `CornflowerBlue`, as `System.Drawing.Color` defines them.
/// `// C#: GCSViews/FlightData.cs:169`
pub const COLOURS: [u32; 16] = [
    0x0000ff, 0xffff00, 0xffc0cb, 0x32cd32, 0xffa500, 0x00ffff, 0xf08080, 0xb0c4de, 0xbdb76b,
    0xffffe0, 0xee82ee, 0x9acd32, 0xff4500, 0xff6347, 0x008080, 0x6495ed,
];

/// The numbers `System.Random` hands `setQuickViewRowsCols`: each below the bound given.
pub type Random<'a> = &'a mut dyn FnMut(usize) -> usize;

/// `new Random()`: seeded from the clock, as .NET's parameterless constructor is. An xorshift, not
/// .NET's generator - the sequence is not the C#'s, only its being unpredictable.
fn clock_random() -> impl FnMut(usize) -> usize {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |since| since.as_nanos());
    #[allow(clippy::cast_possible_truncation)] // any 64 bits of the clock will do
    let mut state = (nanos as u64) | 1;
    move |below| {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        #[allow(clippy::cast_possible_truncation)] // the bound is sixteen
        let pick = (state % below.max(1) as u64) as usize;
        pick
    }
}

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

/// The fence `GeoFenceDist` measures from: `parent.fencepoints`, the shown vehicle's
/// `MAVState.fencepoints` - its fence as the traffic on the link has shown it, which the link
/// keeps (`mp_link::fence_points`) and not the vehicle state, so it is handed over here each frame
/// ([`set_fence`]). Empty, and `GeoFenceDist` 99999, until a fence has passed on the link.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1622-1636`
static FENCE: std::sync::Mutex<Vec<mp_vehicle::FenceItem>> = std::sync::Mutex::new(Vec::new());

/// Hands over the shown vehicle's fence for `GeoFenceDist`: [`crate::telemetry::Telemetry::fence_points`],
/// once a frame.
pub fn set_fence(fence: Vec<mp_vehicle::FenceItem>) {
    *FENCE
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner) = fence;
}

/// `GeoFenceDist` from the fence last handed over.
fn geo_fence_dist(state: &VehicleState) -> f64 {
    let fence = FENCE
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    f64::from(state.geo_fence_dist(&fence))
}

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
    // Measured from the vehicle's fence as the link has seen it, which is not vehicle state:
    // see `set_fence`.
    ("GeoFenceDist", geo_fence_dist),
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
    // The C# clears this flag each time the Transponder page looks (FlightData.cs:6481); the
    // page's last look is not in the state, so this reads "a status has ever arrived".
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

/// Which of `CurrentState`'s multipliers a property's value is shown through.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Multiplier {
    /// `multiplierdist`.
    Dist,
    /// `multiplieralt`.
    Alt,
    /// `multiplierspeed`.
    Speed,
}

/// The held properties the C# shows in the user's units, with the multiplier each one's getter
/// applies and the getter's line. The getter decides, not the text: `alt_error` says "(dist)"
/// and is multiplied as an altitude, `altasl2` says "(dist)" and is not multiplied at all.
///
/// `wind_vel` is the one multiplied where it is set rather than where it is read: the `WIND`
/// handler stores `wind.speed * multiplierspeed` (`CurrentState.cs:2858`). The C#'s other
/// source, `HIGH_LATENCY`, stores its speed unmultiplied (`:2540`); the vehicle's state does not
/// say which message set it, and this multiplies it as a vehicle sending `WIND` - ArduPilot on
/// a normal link - has it.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs`
pub const IN_DISPLAY_UNITS: &[(&str, Multiplier, u32)] = &[
    ("alt", Multiplier::Alt, 327),
    ("altasl", Multiplier::Alt, 354),
    ("airspeed", Multiplier::Speed, 496),
    ("groundspeed", Multiplier::Speed, 529),
    ("wp_dist", Multiplier::Dist, 1093),
    ("alt_error", Multiplier::Alt, 1102),
    ("climbrate", Multiplier::Speed, 1154),
    ("verticalspeed", Multiplier::Speed, 1048),
    ("DistFromMovingBase", Multiplier::Dist, 1781),
    ("wind_vel", Multiplier::Speed, 2858),
    ("DistToHome", Multiplier::Dist, 1781),
    ("sonarrange", Multiplier::Alt, 1861),
    ("ter_curalt", Multiplier::Alt, 2055),
    ("ter_alt", Multiplier::Alt, 2063),
];

/// The multiplier a property is shown through, if any.
#[must_use]
pub fn multiplier(name: &str) -> Option<Multiplier> {
    IN_DISPLAY_UNITS
        .iter()
        .find(|(field, _, _)| *field == name)
        .map(|(_, multiplier, _)| *multiplier)
}

/// A property's SI value as its getter returns it: through `toDistDisplayUnit`,
/// `toAltDisplayUnit` or `toSpeedDisplayUnit` where it has a multiplier, as it is otherwise.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4360-4373`
#[must_use]
pub fn to_display(name: &str, value: f64, units: &DisplayUnits) -> f64 {
    match multiplier(name) {
        Some(Multiplier::Dist) => units.to_dist(value),
        Some(Multiplier::Alt) => units.to_alt(value),
        Some(Multiplier::Speed) => units.to_speed(value),
        None => value,
    }
}

/// What a view bound to `name` shows now, in the user's units: [`value`] through
/// [`to_display`].
#[must_use]
pub fn display_value(name: &str, state: &VehicleState, units: &DisplayUnits) -> Option<f64> {
    value(name, state).map(|value| to_display(name, value, units))
}

/// The view's description: `GetNameandUnit`. The property's `[DisplayText]`, or its name when
/// it has none, with the first of `(dist)`, `(speed)` and `(alt)` it holds replaced by the unit.
///
/// **Divergence:** the C# works a view's description out when the flight screen loads and when
/// the view is bound anew (`FlightData.cs:476, 507, 2488`), so after the units change on the
/// Planner page its views show the old unit's name over a number in the new unit until the next
/// start. Here the description follows the units, so the name always says what the number is in.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4526-4552, MainV2.cs:4247-4330`
#[must_use]
pub fn label(name: &str, units: &DisplayUnits) -> String {
    let desc = CURRENTSTATE
        .iter()
        .find(|field| field.name == name && !field.display.is_empty())
        .map_or(name, |field| field.display);
    units.name_and_unit(desc)
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

/// One `QuickView` in `tableLayoutPanelQuick`.
#[derive(Debug, Clone, PartialEq, Eq)]
struct View {
    /// The `n` of its `Name`, `quickView<n>`: the key its choice is saved under. Two views can
    /// share one - see [`QuickViews::set_rows_cols_with`].
    name: usize,
    /// The cell the Designer puts it in; `None` for one Set View Count added, which the table
    /// lays out in the first free cell.
    cell: Option<(usize, usize)>,
    /// The property it is bound to, or empty for a view added and not yet chosen for.
    field: String,
    /// `numberColor`.
    colour: u32,
    /// `numberColorBackup`: the Designer's `Color.Empty`, a view added its own colour.
    backup: Option<u32>,
}

/// The views in `tableLayoutPanelQuick.Controls` order, the table's size, the view whose chooser
/// is open, and the units they show in.
#[derive(Debug, Clone)]
pub struct QuickViews {
    views: Vec<View>,
    /// `tableLayoutPanelQuick.ColumnCount` and `RowCount`: the Designer's two and three.
    cols: usize,
    rows: usize,
    /// `Settings.Instance["quickViewCols"]` and `["quickViewRows"]` as `setQuickViewRowsCols` last
    /// wrote them, or `None` before it has run.
    saved: Option<(i32, i32)>,
    /// `listQuickView`: the colours given in the current round of sixteen.
    /// `// C#: GCSViews/FlightData.cs:167`
    used: Vec<u32>,
    choosing: Option<usize>,
    /// `CurrentState`'s multipliers and unit names, as the Planner page last set them. Metres
    /// and metres per second until told otherwise: `MainV2` runs `ChangeUnits` before the
    /// flight screen first shows. `// C#: MainV2.cs:836`
    units: DisplayUnits,
    /// Every choice made, in order - the view's name number and the property - for `settings.rs`
    /// to save under the view's name as the C#'s handler does.
    chosen: Vec<(usize, String)>,
}

impl Default for QuickViews {
    fn default() -> Self {
        let views = DEFAULTS
            .iter()
            .zip(CELLS)
            .enumerate()
            .map(|(index, ((name, colour), (column, row)))| View {
                name: index + 1,
                cell: Some((usize::from(column), usize::from(row))),
                field: (*name).to_owned(),
                colour: *colour,
                backup: None,
            })
            .collect();
        Self {
            views,
            cols: 2,
            rows: 3,
            saved: None,
            used: Vec::new(),
            choosing: None,
            units: DisplayUnits::default().change_units(None, None, None),
            chosen: Vec::new(),
        }
    }
}

/// The colour `setQuickViewRowsCols` gives the view it adds, and `listQuickView` after it.
///
/// A colour drawn from the sixteen that is not in the list is taken and listed. One already listed,
/// with more than one listed, is replaced by the first of the sixteen not yet listed (the C# draws
/// a second colour first and then, whichever it drew, takes that first one - the second draw is
/// made and thrown away, so it is made here too). With exactly one listed the drawn colour is kept
/// even if it is that one, and the list does not grow: the C#'s `Count() > 1` test, ported as it
/// is. A full list is cleared first. The C#'s other clearing test, two `OrderBy` sequences compared
/// with `==`, compares references and is never true.
/// `// C#: GCSViews/FlightData.cs:4966-5019`
fn next_colour(used: &mut Vec<u32>, random: Random<'_>) -> u32 {
    if used.len() == COLOURS.len() {
        used.clear();
    }
    let drawn = COLOURS
        .get(random(COLOURS.len()) % COLOURS.len())
        .copied()
        .unwrap_or(COLOURS[0]);
    if used.contains(&drawn) && used.len() > 1 {
        let _different = random(COLOURS.len());
        let remaining = COLOURS
            .iter()
            .copied()
            .find(|colour| !used.contains(colour))
            .unwrap_or(drawn);
        used.push(remaining);
        remaining
    } else {
        if !used.contains(&drawn) {
            used.push(drawn);
        }
        drawn
    }
}

impl QuickViews {
    /// The user's units, handed over by the flight screen each frame.
    pub const fn set_units(&mut self, units: DisplayUnits) {
        self.units = units;
    }

    /// The units the views show in.
    #[must_use]
    pub const fn units(&self) -> &DisplayUnits {
        &self.units
    }

    /// How many views there are.
    #[cfg(test)]
    #[must_use]
    pub fn count(&self) -> usize {
        self.views.len()
    }

    /// `tableLayoutPanelQuick.ColumnCount` and `RowCount`.
    #[must_use]
    pub const fn grid(&self) -> (usize, usize) {
        (self.cols, self.rows)
    }

    /// `quickViewCols` and `quickViewRows` as last set, if they have been.
    #[must_use]
    pub const fn saved_grid(&self) -> Option<(i32, i32)> {
        self.saved
    }

    /// The property view `index` shows; empty for a view bound to nothing.
    #[must_use]
    pub fn field(&self, index: usize) -> &str {
        self.views.get(index).map_or("", |view| view.field.as_str())
    }

    /// The `n` of view `index`'s `Name`, `quickView<n>`.
    #[cfg(test)]
    #[must_use]
    pub fn name(&self, index: usize) -> Option<usize> {
        self.views.get(index).map(|view| view.name)
    }

    /// View `index`'s `numberColor`.
    #[must_use]
    pub fn colour(&self, index: usize) -> Option<u32> {
        self.views.get(index).map(|view| view.colour)
    }

    /// The first view named `quickView<name>`, as `Controls.Find` finds it.
    #[must_use]
    pub fn find(&self, name: usize) -> Option<usize> {
        self.views.iter().position(|view| view.name == name)
    }

    /// Every choice made so far: the view's name number and the property.
    #[must_use]
    pub fn chosen(&self) -> &[(usize, String)] {
        &self.chosen
    }

    /// Each view's cell, (column, row), as `TableLayoutPanel` lays them out in a table of `cols`:
    /// a view with a cell of its own in that cell, the others in `Controls` order in the free
    /// cells, a row at a time, left to right, rows added below when the table is full
    /// (`GrowStyle.AddRows`).
    fn cells_in(&self, cols: usize) -> Vec<(usize, usize)> {
        let cols = cols.max(1);
        let taken: Vec<(usize, usize)> = self.views.iter().filter_map(|view| view.cell).collect();
        let mut next = 0;
        self.views
            .iter()
            .map(|view| {
                view.cell.unwrap_or_else(|| {
                    loop {
                        let cell = (next % cols, next / cols);
                        next += 1;
                        if !taken.contains(&cell) {
                            break cell;
                        }
                    }
                })
            })
            .collect()
    }

    /// Each view's cell in the table as it is.
    #[must_use]
    pub fn cells(&self) -> Vec<(usize, usize)> {
        self.cells_in(self.cols)
    }

    /// `setQuickViewRowsCols(cols, rows)`, with `new Random()`'s numbers.
    ///
    /// # Errors
    ///
    /// `int.Parse`'s message, when a number `IsNumber` allowed is not whole; nothing changes.
    pub fn set_rows_cols(&mut self, cols: &str, rows: &str) -> Result<(), &'static str> {
        let mut random = clock_random();
        self.set_rows_cols_with(cols, rows, &mut random)
    }

    /// `setQuickViewRowsCols(cols, rows)`: each `Math.Max(1, int.Parse(..))`, kept as the
    /// settings; the views whose cell, as the table was last laid out, is outside the new size
    /// removed; then views added until there are `cols * rows`, each named `quickView` and the
    /// count so far plus one, bound to nothing, `number` 0, its colour from [`next_colour`] and
    /// `numberColorBackup` the same. `listQuickView` is cleared before and after when its count is
    /// a multiple of sixteen - the C#'s test before also asks whether the views are at most or at
    /// least the total, which is always so.
    ///
    /// The name is from the count, not the names in use, so after 2 x 3 to 1 x 3 (which removes
    /// `quickView2`, 4 and 6) and back, the first added is `quickView4` and the second a second
    /// `quickView5`: the C# does this, and both then save to one key. Each added view double
    /// clicks to the chooser (unless `lockQuickView`, [`page`]) and has the same context menu;
    /// every column and every row gets an equal share, as the grid [`page`] draws does.
    /// `// C#: GCSViews/FlightData.cs:4914-5060`
    ///
    /// # Errors
    ///
    /// As [`QuickViews::set_rows_cols`].
    pub fn set_rows_cols_with(
        &mut self,
        cols: &str,
        rows: &str,
        random: Random<'_>,
    ) -> Result<(), &'static str> {
        let (cols, rows) = crate::fly::view_count(cols, rows)?;
        self.saved = Some((cols, rows));
        let new_cols = usize::try_from(cols).unwrap_or(1);
        let new_rows = usize::try_from(rows).unwrap_or(1);
        // `PerformLayout` before the counts change: the positions are the old table's.
        let mut cells = self.cells_in(self.cols).into_iter();
        self.views.retain(|_| {
            cells
                .next()
                .is_some_and(|(column, row)| column < new_cols && row < new_rows)
        });
        self.cols = new_cols;
        self.rows = new_rows;
        self.choosing = None;
        let total = new_cols.saturating_mul(new_rows);
        if self.used.len().is_multiple_of(COLOURS.len()) {
            self.used.clear();
        }
        while total > self.views.len() {
            let colour = next_colour(&mut self.used, random);
            self.views.push(View {
                name: self.views.len() + 1,
                cell: None,
                field: String::new(),
                colour,
                backup: Some(colour),
            });
        }
        if self.used.len().is_multiple_of(COLOURS.len()) {
            self.used.clear();
        }
        Ok(())
    }

    /// `quickView_DoubleClick`: opens the chooser for a view.
    pub fn open(&mut self, index: usize) {
        if index < self.views.len() {
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
        if let Some(view) = self.views.get_mut(index) {
            name.clone_into(&mut view.field);
            self.chosen.push((view.name, name.to_owned()));
        }
        self.choosing = None;
    }

    /// Closes the chooser without choosing: the form's close box.
    pub fn close(&mut self) {
        self.choosing = None;
    }

    /// Publishes what a UI test asserts on: [`QuickViews::facts`].
    pub fn record_facts(&self, state: Option<&VehicleState>) {
        for (key, value) in self.facts(state) {
            crate::facts::record(key, value);
        }
    }

    /// The number view `index` shows as the page paints it: a view bound to nothing the 0
    /// `setQuickViewRowsCols` gave it; a bound one its property, or `None` with no vehicle.
    fn number(&self, index: usize, state: Option<&VehicleState>) -> Option<String> {
        let field = self.field(index);
        if field.is_empty() {
            return Some(number_text(0.0));
        }
        state
            .and_then(|state| display_value(field, state, &self.units))
            .map(number_text)
    }

    /// Each view's property, description and number - the description and the number as the
    /// page paints them, in the user's units - its name, cell and colours; the table's size and
    /// the saved one; and the chooser.
    #[must_use]
    pub fn facts(&self, state: Option<&VehicleState>) -> Vec<(String, String)> {
        let mut facts = Vec::new();
        let cells = self.cells();
        for (index, view) in self.views.iter().enumerate() {
            let key = format!("fly.quick.{}", index + 1);
            facts.push((format!("{key}.label"), label(&view.field, &self.units)));
            facts.push((
                format!("{key}.value"),
                self.number(index, state)
                    .unwrap_or_else(|| "none".to_owned()),
            ));
            facts.push((format!("{key}.name"), format!("quickView{}", view.name)));
            if let Some((column, row)) = cells.get(index) {
                facts.push((format!("{key}.cell"), format!("{column},{row}")));
            }
            facts.push((format!("{key}.colour"), format!("{:06x}", view.colour)));
            facts.push((
                format!("{key}.backup"),
                view.backup
                    .map_or_else(|| "none".to_owned(), |colour| format!("{colour:06x}")),
            ));
            facts.push((
                key,
                if view.field.is_empty() {
                    "none".to_owned()
                } else {
                    view.field.clone()
                },
            ));
        }
        facts.push(("fly.quick.count".to_owned(), self.views.len().to_string()));
        facts.push((
            "fly.quick.layout".to_owned(),
            format!("{}x{}", self.cols, self.rows),
        ));
        facts.push((
            "fly.quick.grid".to_owned(),
            self.saved.map_or_else(
                || "none".to_owned(),
                |(cols, rows)| format!("{cols}x{rows}"),
            ),
        ));
        facts.push((
            "fly.quick.chooser".to_owned(),
            self.choosing
                .map_or_else(|| "none".to_owned(), |index| (index + 1).to_string()),
        ));
        facts.push(("fly.quick.choices".to_owned(), choices().len().to_string()));
        facts
    }
}

/// The Quick page: `tableLayoutPanelQuick`'s views in its columns and rows, each column an equal
/// share of the width and each row of the height (`ColumnStyles` and `RowStyles` in percent).
/// `// C#: GCSViews/FlightData.cs:5036-5052`
pub fn page(
    views: &QuickViews,
    state: Option<&VehicleState>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (cols, rows) = views.grid();
    let cells = views.cells();
    // A view past the table's last row (none once `setQuickViewRowsCols` has run) adds a row.
    let rows = cells
        .iter()
        .map(|(_, row)| row + 1)
        .max()
        .unwrap_or(rows)
        .max(rows);
    // Named as the panels are, so a layout test finds the page's bottom edge.
    let mut grid = crate::probe::measured("panel:quick", div())
        .grid()
        .grid_cols(u16::try_from(cols).unwrap_or(u16::MAX))
        .grid_rows(u16::try_from(rows).unwrap_or(u16::MAX))
        .gap_1()
        .flex_1()
        .min_h(px(180.0));
    let locked = crate::display_view::flag("lockQuickView");
    for (index, (column, row)) in cells.into_iter().enumerate() {
        let field = views.field(index).to_owned();
        let colour = views.colour(index).unwrap_or(theme::TEXT);
        let desc = label(&field, views.units());
        let number = views
            .number(index, state)
            .unwrap_or_else(|| "--".to_owned());
        let id = view_id(index);
        grid = grid.child(
            crate::probe::measured(id.clone(), div())
                .id(gpui::SharedString::from(id))
                .col_start(i16::try_from(column + 1).unwrap_or(i16::MAX))
                .row_start(i16::try_from(row + 1).unwrap_or(i16::MAX))
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
                // `DoubleClick`, which Windows raises on the second press; `quickView_DoubleClick`
                // does nothing while `lockQuickView` is set.
                // `// C#: GCSViews/FlightData.cs:4549-4552, 5026-5027`
                .on_mouse_down(
                    gpui::MouseButton::Left,
                    cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                        if event.click_count == 2 && !locked {
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
        ..Scene::default()
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
            .join("../../references/missionplanner")
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

    /// Metres and metres per second, as `ChangeUnits` sets them with nothing configured.
    fn metric() -> DisplayUnits {
        DisplayUnits::default().change_units(None, None, None)
    }

    /// Feet for distance and altitude and knots for speed, as the Planner page sets them.
    fn feet_and_knots() -> DisplayUnits {
        DisplayUnits::default().change_units(Some("Feet"), Some("Feet"), Some("knots"))
    }

    #[test]
    fn a_description_is_the_display_text_with_its_unit() {
        let metric = metric();
        assert_eq!(label("alt", &metric), "Altitude (m)");
        assert_eq!(label("groundspeed", &metric), "GroundSpeed (m/s)");
        assert_eq!(label("wp_dist", &metric), "Dist to WP (m)");
        assert_eq!(label("yaw", &metric), "Yaw (deg)");
        assert_eq!(label("verticalspeed", &metric), "Vertical Speed (m/s)");
        assert_eq!(label("DistToHome", &metric), "Dist to Home (m)");
        // No display text: the name.
        let bare = CURRENTSTATE
            .iter()
            .find(|field| field.display.is_empty() && matches!(field.ours, Ours::Done(_)))
            .map(|field| field.name)
            .expect("a done row without display text");
        assert_eq!(label(bare, &metric), bare);
    }

    /// In feet and knots each default view names its unit, and the ones with none keep theirs.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4526-4552`
    #[test]
    fn a_description_names_the_users_unit() {
        let units = feet_and_knots();
        assert_eq!(label("alt", &units), "Altitude (ft)");
        assert_eq!(label("groundspeed", &units), "GroundSpeed (kts)");
        assert_eq!(label("wp_dist", &units), "Dist to WP (ft)");
        assert_eq!(label("yaw", &units), "Yaw (deg)");
        assert_eq!(label("verticalspeed", &units), "Vertical Speed (kts)");
        assert_eq!(label("DistToHome", &units), "Dist to Home (ft)");
        // "(dist)" is named by the distance unit even where the getter multiplies otherwise.
        let mixed = DisplayUnits::default().change_units(Some("Feet"), Some("Meters"), Some("kph"));
        assert_eq!(label("alt_error", &mixed), "Altitude Error (ft)");
        assert_eq!(label("altasl", &mixed), "Altitude (m)");
        assert_eq!(label("airspeed", &mixed), "AirSpeed (kph)");
    }

    /// Each held property whose getter multiplies is in the table with that multiplier, and no
    /// other is: read from `CurrentState.cs`, the property's declaration to the end of its body.
    /// `wind_vel`, multiplied where the `WIND` handler sets it, is checked there.
    #[test]
    fn the_units_table_is_the_csharps_getters() {
        let Some(source) = csharp("ExtLibs/ArduPilot/CurrentState.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let lines: Vec<&str> = source.lines().collect();
        // The property's declaration and body, from `public <type> <name>` to where its braces
        // close, or the one line of an auto-property or an expression body.
        let body = |name: &str| -> Option<(usize, String)> {
            let start = lines.iter().position(|line| {
                let Some(at) = line.find("public ") else {
                    return false;
                };
                let mut words = line[at + 7..].split_whitespace();
                let first = words.next();
                let first = if first == Some("static") {
                    words.next()
                } else {
                    first
                };
                // The type, then the name: a method's name has its `(` attached and never
                // matches.
                first.is_some()
                    && words
                        .next()
                        .is_some_and(|word| word.trim_end_matches(';') == name)
            })?;
            let mut text = String::new();
            let mut depth = 0i32;
            let mut opened = false;
            for (offset, line) in lines.iter().enumerate().skip(start) {
                text.push_str(line);
                text.push('\n');
                depth += i32::try_from(line.matches('{').count()).unwrap_or(0);
                depth -= i32::try_from(line.matches('}').count()).unwrap_or(0);
                opened |= line.contains('{');
                let one_line = offset == start && (line.contains(';') || line.contains("=>"));
                if (opened && depth <= 0) || (one_line && !opened) {
                    break;
                }
            }
            Some((start + 1, text))
        };
        let found = |text: &str| {
            if text.contains("multiplierdist") || text.contains("toDistDisplayUnit") {
                Some(Multiplier::Dist)
            } else if text.contains("multiplieralt") || text.contains("toAltDisplayUnit") {
                Some(Multiplier::Alt)
            } else if text.contains("multiplierspeed") || text.contains("toSpeedDisplayUnit") {
                Some(Multiplier::Speed)
            } else {
                None
            }
        };
        for (name, _) in READERS.iter().chain(DERIVED) {
            let (line, text) = body(name).unwrap_or_else(|| panic!("no property {name}"));
            let wanted = if *name == "wind_vel" {
                assert!(
                    source.contains("wind_vel = wind.speed * multiplierspeed;"),
                    "the WIND handler no longer multiplies wind_vel"
                );
                Some(Multiplier::Speed)
            } else {
                found(&text)
            };
            assert_eq!(
                multiplier(name),
                wanted,
                "{name} at CurrentState.cs:{line}:\n{text}"
            );
        }
        // Every row names a held property, and the line of its multiplier.
        for (name, _, line) in IN_DISPLAY_UNITS {
            assert!(
                READERS.iter().chain(DERIVED).any(|(held, _)| held == name),
                "{name} is not held"
            );
            let at = lines
                .get(usize::try_from(*line).unwrap_or(0).saturating_sub(1))
                .copied()
                .unwrap_or("");
            assert!(
                at.contains("multiplier") || at.contains("DisplayUnit"),
                "CurrentState.cs:{line} is {at:?}, not {name}'s multiplier"
            );
        }
    }

    /// The views read the vehicle through the getters' multipliers: metres to feet, metres per
    /// second to knots, and a heading as it is. The facts are what the page paints.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:327, 529, 1093, 1781, MainV2.cs:4262, 4317`
    #[test]
    fn the_views_show_the_vehicle_in_the_users_units() {
        let mut state = VehicleState::default();
        state.altitude_relative = Metres(100.0);
        state.ground_speed = MetresPerSecond(10.0);
        state.nav.wp_distance = 250.0;
        state.attitude.yaw = mp_units::Radians(std::f64::consts::FRAC_PI_2);
        state.home = Some(mp_units::LatLon::new(-35.0, 149.0).unwrap());
        state.position = Some(mp_units::LatLon::new(-35.001, 149.0).unwrap());
        let units = feet_and_knots();
        let feet = f64::from("3.2808399".parse::<f32>().unwrap());
        let knots = f64::from("1.94384449".parse::<f32>().unwrap());
        assert_eq!(display_value("alt", &state, &units), Some(100.0 * feet));
        assert_eq!(
            display_value("groundspeed", &state, &units),
            Some(10.0 * knots)
        );
        assert_eq!(display_value("wp_dist", &state, &units), Some(250.0 * feet));
        let yaw = display_value("yaw", &state, &units).unwrap();
        assert!((yaw - 90.0).abs() < 1e-9, "{yaw}");
        let home = display_value("DistToHome", &state, &units).unwrap();
        assert!((home - 111.3195 * feet).abs() < 0.05, "{home}");

        let mut views = QuickViews::default();
        views.set_units(units);
        let facts = views.facts(Some(&state));
        let fact = |key: &str| {
            facts
                .iter()
                .find(|(name, _)| name == key)
                .map(|(_, value)| value.as_str())
        };
        assert_eq!(fact("fly.quick.1"), Some("alt"));
        assert_eq!(fact("fly.quick.1.label"), Some("Altitude (ft)"));
        assert_eq!(fact("fly.quick.1.value"), Some("328.08"));
        assert_eq!(fact("fly.quick.2.label"), Some("GroundSpeed (kts)"));
        assert_eq!(fact("fly.quick.2.value"), Some("19.44"));
        assert_eq!(fact("fly.quick.3.label"), Some("Dist to WP (ft)"));
        assert_eq!(fact("fly.quick.3.value"), Some("820.21"));
        assert_eq!(fact("fly.quick.4.value"), Some("90.00"));
        // Vertical speed is held now: 0 m/s is 0.00 in any unit.
        assert_eq!(fact("fly.quick.5.value"), Some("0.00"));
        assert_eq!(fact("fly.quick.6.label"), Some("Dist to Home (ft)"));
        // Back to metres, the same views read metres again.
        views.set_units(metric());
        let facts = views.facts(Some(&state));
        assert!(facts.contains(&("fly.quick.1.label".to_owned(), "Altitude (m)".to_owned())));
        assert!(facts.contains(&("fly.quick.1.value".to_owned(), "100.00".to_owned())));
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
        assert_eq!(label("battery_voltage", &metric()), "Bat Voltage (V)");
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

    /// The facts of one view, by key.
    fn fact(views: &QuickViews, key: &str) -> Option<String> {
        views
            .facts(None)
            .into_iter()
            .find(|(k, _)| k == key)
            .map(|(_, value)| value)
    }

    /// A `Random` that hands out the numbers given, then zeros.
    fn scripted(numbers: &[usize]) -> impl FnMut(usize) -> usize + '_ {
        let mut next = numbers.iter();
        move |_below| next.next().copied().unwrap_or(0)
    }

    /// 2 x 3 to 3 x 4 keeps the six in their cells and adds `quickView7` to 12 in the free cells,
    /// a row at a time; back to 2 x 3 removes the six again.
    #[test]
    fn set_view_count_grows_and_shrinks_the_grid() {
        let mut views = QuickViews::default();
        assert_eq!(views.grid(), (2, 3));
        assert_eq!(views.saved_grid(), None);
        assert_eq!(fact(&views, "fly.quick.grid").as_deref(), Some("none"));
        assert_eq!(fact(&views, "fly.quick.layout").as_deref(), Some("2x3"));

        views.set_rows_cols("3", "4").expect("whole numbers");
        assert_eq!(views.grid(), (3, 4));
        assert_eq!(views.saved_grid(), Some((3, 4)));
        assert_eq!(views.count(), 12);
        let names: Vec<_> = (0..12).filter_map(|index| views.name(index)).collect();
        assert_eq!(names, (1..=12).collect::<Vec<_>>());
        assert_eq!(
            views.cells(),
            vec![
                (0, 0),
                (1, 0),
                (0, 1),
                (1, 1),
                (0, 2),
                (1, 2),
                (2, 0),
                (2, 1),
                (2, 2),
                (0, 3),
                (1, 3),
                (2, 3),
            ]
        );
        for (index, (name, colour)) in DEFAULTS.iter().enumerate() {
            assert_eq!(views.field(index), *name);
            assert_eq!(views.colour(index), Some(*colour));
        }
        // The added views: bound to nothing, showing 0, no description, a colour of the sixteen.
        assert_eq!(views.field(11), "");
        assert_eq!(view_id(11), "fly-quick-12");
        assert_eq!(fact(&views, "fly.quick.12").as_deref(), Some("none"));
        assert_eq!(
            fact(&views, "fly.quick.12.name").as_deref(),
            Some("quickView12")
        );
        assert_eq!(fact(&views, "fly.quick.12.cell").as_deref(), Some("2,3"));
        assert_eq!(fact(&views, "fly.quick.12.value").as_deref(), Some("0.00"));
        assert_eq!(fact(&views, "fly.quick.12.label").as_deref(), Some(""));
        assert!(COLOURS.contains(&views.colour(11).expect("twelve")));
        assert_eq!(
            fact(&views, "fly.quick.12.backup"),
            fact(&views, "fly.quick.12.colour")
        );
        assert_eq!(fact(&views, "fly.quick.1.backup").as_deref(), Some("none"));
        assert_eq!(fact(&views, "fly.quick.count").as_deref(), Some("12"));
        assert_eq!(fact(&views, "fly.quick.grid").as_deref(), Some("3x4"));
        // An added view double clicks to the chooser, and binds.
        views.open(11);
        views.choose("satcount");
        assert_eq!(views.field(11), "satcount");
        assert_eq!(views.chosen(), &[(12, "satcount".to_owned())]);

        views.set_rows_cols("2", "3").expect("whole numbers");
        assert_eq!(views.count(), 6);
        assert_eq!(views.grid(), (2, 3));
        for (index, (name, _)) in DEFAULTS.iter().enumerate() {
            assert_eq!(views.name(index), Some(index + 1));
            assert_eq!(views.field(index), *name);
        }
        assert_eq!(fact(&views, "fly.quick.7"), None);
    }

    /// The table keeps a view whose cell, as last laid out, is inside the new size; the views then
    /// added are named from the count, so a name can come twice - as in the C#.
    #[test]
    fn set_view_count_removes_by_cell_and_names_by_count() {
        let mut views = QuickViews::default();
        // One column: quickView2, 4 and 6, in the second, go.
        views.set_rows_cols("1", "3").expect("whole numbers");
        let names: Vec<_> = (0..views.count()).filter_map(|i| views.name(i)).collect();
        assert_eq!(names, vec![1, 3, 5]);
        views.set_rows_cols("2", "3").expect("whole numbers");
        let names: Vec<_> = (0..views.count()).filter_map(|i| views.name(i)).collect();
        assert_eq!(names, vec![1, 3, 5, 4, 5, 6]);
        assert_eq!(views.cells()[3..], [(1, 0), (1, 1), (1, 2)]);
        // `Controls.Find` finds the first.
        assert_eq!(views.find(5), Some(2));
        assert_eq!(views.find(2), None);

        // Views laid out by the table move as the columns change: 3 x 4 then 4 x 3 keeps the
        // added views in the cells a four-column table gives them, and removes the fourth row.
        let mut views = QuickViews::default();
        views.set_rows_cols("3", "4").expect("whole numbers");
        views.set_rows_cols("4", "3").expect("whole numbers");
        // Laid out in three columns, quickView10 to 12 were in the fourth row: gone.
        let names: Vec<_> = (0..views.count()).filter_map(|i| views.name(i)).collect();
        assert_eq!(names, vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
        assert_eq!(views.count(), 12);
        assert_eq!(views.cells()[6..9], [(2, 0), (3, 0), (2, 1)]);
    }

    /// `Math.Max(1, int.Parse(..))`: nought and less are one; a number that is not whole changes
    /// nothing.
    #[test]
    fn set_view_count_keeps_at_least_one_view() {
        let mut views = QuickViews::default();
        views.set_rows_cols("0", "-3").expect("whole numbers");
        assert_eq!(views.grid(), (1, 1));
        assert_eq!(views.saved_grid(), Some((1, 1)));
        assert_eq!(views.count(), 1);
        assert_eq!(views.field(0), "alt");
        assert!(views.set_rows_cols("2.5", "3").is_err());
        assert_eq!(views.grid(), (1, 1));
        assert_eq!(views.count(), 1);
    }

    /// `listQuickView`'s rules: a colour drawn not yet listed is taken; one listed, with more than
    /// one listed, gives way to the first of the sixteen not listed (after a second draw that is
    /// thrown away); with exactly one listed a repeat is kept; the list clears at sixteen.
    #[test]
    fn a_new_views_colour_is_one_not_used_this_round() {
        let (blue, yellow, pink, lime) = (COLOURS[0], COLOURS[1], COLOURS[2], COLOURS[3]);
        let mut views = QuickViews::default();
        views.set_rows_cols("1", "1").expect("whole numbers");
        // Draws: Yellow (listed); Yellow again (listed, but only one listed: kept, the list as it
        // was); Blue (listed); Blue again (two listed: the second draw, 5, thrown away, and the
        // first not listed is Pink); Yellow again (the draw 7 thrown away: Lime).
        let mut random = scripted(&[1, 1, 0, 0, 5, 1, 7]);
        views
            .set_rows_cols_with("1", "6", &mut random)
            .expect("whole numbers");
        let colours: Vec<_> = (1..6).filter_map(|i| views.colour(i)).collect();
        assert_eq!(colours, vec![yellow, yellow, blue, pink, lime]);
        assert_eq!(views.used, vec![yellow, blue, pink, lime]);

        // The C#'s `Count() > 1`: with only one listed, the same colour twice.
        let mut views = QuickViews::default();
        views.set_rows_cols("1", "1").expect("whole numbers");
        let mut random = scripted(&[0, 0, 0]);
        views
            .set_rows_cols_with("1", "3", &mut random)
            .expect("whole numbers");
        let colours: Vec<_> = (1..3).filter_map(|i| views.colour(i)).collect();
        assert_eq!(colours, vec![blue, blue]);

        // Seventeen added in one go, the first two drawn apart: every one of the first sixteen a
        // different colour, then the round starts again.
        let mut views = QuickViews::default();
        views.set_rows_cols("1", "1").expect("whole numbers");
        let mut clock = super::clock_random();
        let mut draws = vec![0, 1];
        draws.extend((0..40).map(|_| clock(16)));
        let mut random = scripted(&draws);
        views
            .set_rows_cols_with("1", "18", &mut random)
            .expect("whole numbers");
        let colours: Vec<_> = (1..17).filter_map(|i| views.colour(i)).collect();
        let mut distinct = colours.clone();
        distinct.sort_unstable();
        distinct.dedup();
        assert_eq!(distinct.len(), 16, "{colours:x?}");
        assert!(COLOURS.contains(&views.colour(17).expect("eighteen")));
        // Sixteen listed and one more: cleared at the sixteenth, one listed after.
        assert_eq!(views.used.len(), 1);

        // Sixteen exactly: cleared after the loop, ready for the next round.
        let mut views = QuickViews::default();
        views.set_rows_cols("1", "1").expect("whole numbers");
        let mut random = scripted(&draws);
        views
            .set_rows_cols_with("1", "17", &mut random)
            .expect("whole numbers");
        assert!(views.used.is_empty());
    }

    /// The product's own randomiser: every colour one of the sixteen, and a run of sixteen with
    /// its first two apart never repeats one.
    #[test]
    fn the_clock_randomiser_picks_from_the_sixteen() {
        let mut random = super::clock_random();
        for _ in 0..200 {
            assert!(random(16) < 16);
        }
        let mut views = QuickViews::default();
        views.set_rows_cols("1", "1").expect("whole numbers");
        views.set_rows_cols("1", "11").expect("whole numbers");
        for index in 1..11 {
            assert!(COLOURS.contains(&views.colour(index).expect("eleven")));
        }
    }

    /// The sixteen are `System.Drawing.Color`'s, in the C#'s order.
    #[test]
    fn the_sixteen_colours_are_the_csharps() {
        let Some(source) = csharp("GCSViews/FlightData.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        assert!(source.contains(
            "Color[] colorsForDefaultQuickView = new Color[] { Color.Blue, Color.Yellow, \
             Color.Pink, Color.LimeGreen, Color.Orange, Color.Aqua, Color.LightCoral, \
             Color.LightSteelBlue, Color.DarkKhaki, Color.LightYellow, Color.Violet, \
             Color.YellowGreen, Color.OrangeRed, Color.Tomato, Color.Teal, Color.CornflowerBlue };"
        ));
        assert_eq!(COLOURS[0], 0x0000ff);
        assert_eq!(COLOURS[15], 0x6495ed);
    }
}
