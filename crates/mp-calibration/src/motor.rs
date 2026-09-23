//! Motor test: the protocol half of `GCSViews/ConfigurationView/ConfigMotorTest.cs`, without its
//! form.
//!
//! The page counts the vehicle's motors from its frame (`get_motormax`), gives each a button
//! lettered `A`, `B`, `C`... in test order, and labels each from the frame's layout in
//! `APMotorLayout.json` ([`crate::motor_layouts`]). Every button is one or more calls of
//! `testMotor(motor, speed, time, motorcount)`, which is `MAV_CMD_DO_MOTOR_TEST` with the throttle
//! as a percentage: a motor button sends its own number, "Test all motors" every number in turn,
//! "Test all in Sequence" motor 1 with the motor count, and "Stop all motors" every number at zero
//! throttle for zero seconds.
//!
//! What this module holds is what those decisions are: [`motor_count`] is `get_motormax` over a
//! parameter lookup and the heartbeat's `MAV_TYPE`, [`MotorCommand`] is one `testMotor` call and
//! the functions below it are the buttons. Nothing here bounds the throttle or the duration: the
//! C# sends what its boxes hold, a throttle from -100 to 100 and a duration from 0 to 999 seconds,
//! and the vehicle decides what to do with them.

use mp_mavlink_dialects::all::MavMessage;
use mp_vehicle::VehicleId;

use crate::motor_layouts::{LAYOUT_VERSION, LAYOUTS, Layout};

/// `MAV_CMD_DO_MOTOR_TEST`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1041`
pub const CMD_DO_MOTOR_TEST: u16 = 209;

/// `MOTOR_TEST_THROTTLE_PERCENT`, the throttle type every `testMotor` sends.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:4527-4532; GCSViews/ConfigurationView/ConfigMotorTest.cs:313`
pub const MOTOR_TEST_THROTTLE_PERCENT: u8 = 0;

/// `MAV_TYPE_QUADROTOR`.
const TYPE_QUADROTOR: u8 = 2;
/// `MAV_TYPE_HELICOPTER`.
const TYPE_HELICOPTER: u8 = 4;
/// `MAV_TYPE_GROUND_ROVER`.
const TYPE_GROUND_ROVER: u8 = 10;
/// `MAV_TYPE_SURFACE_BOAT`.
const TYPE_SURFACE_BOAT: u8 = 11;
/// `MAV_TYPE_HEXAROTOR`.
const TYPE_HEXAROTOR: u8 = 13;
/// `MAV_TYPE_OCTOROTOR`.
const TYPE_OCTOROTOR: u8 = 14;
/// `MAV_TYPE_TRICOPTER`.
const TYPE_TRICOPTER: u8 = 15;
/// `MAV_TYPE_DODECAROTOR`.
const TYPE_DODECAROTOR: u8 = 29;

/// The motor count `get_motormax` starts from, and keeps for a vehicle type it does not name.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:113`
pub const DEFAULT_MOTORS: usize = 8;

/// One `testMotor(motor, speed, time, motorcount)` call.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:305-327`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorCommand {
    /// `motor`: param1, the motor's place in the test order, 1 first.
    pub motor: i32,
    /// `speed`: param3, the throttle percentage, `(int)NUM_thr_percent.Value`.
    pub throttle: i32,
    /// `time`: param4, seconds, `(int)NUM_duration.Value`.
    pub duration: i32,
    /// `motorcount`: param5, how many motors to test in sequence; 0 for one.
    pub count: i32,
}

impl MotorCommand {
    /// The seven parameters `testMotor` gives `doCommand`: the motor, `MOTOR_TEST_THROTTLE_PERCENT`,
    /// the throttle, the duration, the motor count, and two zeros - each cast to `float`.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:309-318`
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // the C#'s `(float)` casts, of numbers under 1000
    pub fn params(self) -> [f32; 7] {
        [
            self.motor as f32,
            f32::from(MOTOR_TEST_THROTTLE_PERCENT),
            self.throttle as f32,
            self.duration as f32,
            self.count as f32,
            0.0,
            0.0,
        ]
    }

    /// The `COMMAND_LONG` `doCommand` puts on the wire for it, to `target`, confirmation 0.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2697-2710`
    #[must_use]
    pub fn message(self, target: VehicleId) -> MavMessage {
        crate::command(target, CMD_DO_MOTOR_TEST, self.params())
    }
}

/// A motor button, `but_Click`: its `Tag`, the number `Activate` gave it, at the boxes' values.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:290-303`
#[must_use]
pub const fn single(motor: i32, speed: i32, time: i32) -> MotorCommand {
    MotorCommand {
        motor,
        throttle: speed,
        duration: time,
        count: 0,
    }
}

/// "Test all motors", `but_TestAll`: every motor from 1 to `motormax`, one after another, each
/// on its own at the boxes' values.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:263-272`
#[must_use]
pub fn test_all(motormax: usize, speed: i32, time: i32) -> Vec<MotorCommand> {
    (1..=motormax)
        .map(|motor| single(to_i32(motor), speed, time))
        .collect()
}

/// "Test all in Sequence", `but_TestAllSeq`: motor 1, with `motormax` as the count, so the
/// vehicle steps through them itself.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:274-280`
#[must_use]
pub fn sequence(motormax: usize, speed: i32, time: i32) -> MotorCommand {
    MotorCommand {
        motor: 1,
        throttle: speed,
        duration: time,
        count: to_i32(motormax),
    }
}

/// "Stop all motors", `but_StopAll`: every motor from 1 to `motormax` at zero throttle for zero
/// seconds.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:282-288`
#[must_use]
pub fn stop_all(motormax: usize) -> Vec<MotorCommand> {
    (1..=motormax)
        .map(|motor| single(to_i32(motor), 0, 0))
        .collect()
}

/// A count as the C#'s `int`; a motor count never comes near its limit.
fn to_i32(value: usize) -> i32 {
    i32::try_from(value).unwrap_or(i32::MAX)
}

/// The letter of button `a`: `(char)((a - 1) + 'A')`.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:54`
#[must_use]
pub fn letter(a: usize) -> char {
    u32::try_from(a)
        .ok()
        .and_then(|a| a.checked_sub(1))
        .and_then(|offset| char::from_u32(u32::from('A') + offset))
        .unwrap_or('?')
}

/// Button `a`'s text: "Test motor " and its letter.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:54`
#[must_use]
pub fn button_text(a: usize) -> String {
    format!("Test motor {}", letter(a))
}

/// The labels `Activate` puts beside button `a`: one per motor of the layout whose `TestOrder` is
/// `a` - "Motor Number: " and its number, then ", " and its rotation unless that is "?".
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:61-77`
#[must_use]
pub fn labels(layout: Option<&Layout>, a: usize) -> Vec<String> {
    let Some(layout) = layout else {
        return Vec::new();
    };
    layout
        .motors
        .iter()
        .filter(|motor| usize::try_from(motor.test_order).ok() == Some(a))
        .map(|motor| {
            let mut text = format!("Motor Number: {}", motor.number);
            if motor.rotation != "?" {
                text.push_str(", ");
                text.push_str(motor.rotation);
            }
            text
        })
        .collect()
}

/// `lookup_frame_layout`: the first layout of the file for this class and type, if the file is the
/// version the C# reads.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:236-261`
#[must_use]
pub fn lookup_frame_layout(frame_class: i32, frame_type: i32) -> Option<&'static Layout> {
    if LAYOUT_VERSION != "AP_Motors library test ver 1.2" {
        return None;
    }
    LAYOUTS
        .iter()
        .find(|layout| layout.class == frame_class && layout.frame_type == frame_type)
}

/// The pair of parameters `set_frame_class_and_type` found, and what they hold, `(int)`-cast.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameParams {
    /// `FRAME_CLASS` or `Q_FRAME_CLASS`.
    pub class_param: &'static str,
    /// Its value.
    pub class: i32,
    /// `FRAME_TYPE` or `Q_FRAME_TYPE`.
    pub type_param: &'static str,
    /// Its value.
    pub frame_type: i32,
}

/// What `get_motormax` decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorCount {
    /// `motormax`: how many motor buttons, and how many motors the other three buttons address.
    pub count: usize,
    /// False where the C# sets the page's `Enabled` false: the vehicle has none of `FRAME`,
    /// `Q_FRAME_TYPE` and `FRAME_TYPE`.
    pub enabled: bool,
    /// The class and type parameters the labels were read from, when a pair was found.
    pub frame: Option<FrameParams>,
    /// `motor_layout` afterwards: the layout found, else the one the page already held - the C#
    /// only ever assigns the field when a layout matches.
    pub layout: Option<&'static Layout>,
}

/// A parameter's value as `(int)MAV.param[name].Value` reads it: truncated towards zero.
#[allow(clippy::cast_possible_truncation)] // the C#'s `(int)` cast of a small whole number
fn int_of(value: f64) -> i32 {
    value as i32
}

/// `get_motormax` and the `set_frame_class_and_type` calls in it.
///
/// `param` is the vehicle's parameter table (`MAV.param`), `aptype` the heartbeat's `MAV_TYPE`
/// (`MAV.aptype`), and `held` the layout the page found last time it was activated: the C#'s
/// `motor_layout` is a field, assigned only when a layout matches, so a frame the file lacks
/// keeps the one before.
///
/// In order: a ground rover or a boat has 4, before anything else is read; a vehicle with none of
/// `FRAME`, `Q_FRAME_TYPE` and `FRAME_TYPE` has 8 and the page disabled; `FRAME_CLASS` and
/// `FRAME_TYPE`, else `Q_FRAME_CLASS` and `Q_FRAME_TYPE`, name a layout whose motors are counted;
/// and failing that the count follows the vehicle type - `Q_FRAME_CLASS` mapped to one where the
/// vehicle has it, else the heartbeat's - 4 for a tricopter or a quad, 6 for a hexa, 8 for an
/// octo, 0 for a helicopter, 12 for a dodecarotor, and 8 for anything else.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:111-233`
#[must_use]
pub fn motor_count(
    param: impl Fn(&str) -> Option<f64>,
    aptype: u8,
    held: Option<&'static Layout>,
) -> MotorCount {
    let mut result = MotorCount {
        count: DEFAULT_MOTORS,
        enabled: true,
        frame: None,
        layout: held,
    };

    // C#: ConfigMotorTest.cs:115-118
    if aptype == TYPE_GROUND_ROVER || aptype == TYPE_SURFACE_BOAT {
        result.count = 4;
        return result;
    }

    // C#: ConfigMotorTest.cs:120-126
    let has = |name: &str| param(name).is_some();
    if !(has("FRAME") || has("Q_FRAME_TYPE") || has("FRAME_TYPE")) {
        result.enabled = false;
        return result;
    }

    // C#: ConfigMotorTest.cs:128-135, 202-233. The first pair the vehicle has both of is read,
    // and the second is not tried.
    let pair = [
        ("FRAME_CLASS", "FRAME_TYPE"),
        ("Q_FRAME_CLASS", "Q_FRAME_TYPE"),
    ]
    .into_iter()
    .find_map(|(class_param, type_param)| {
        Some(FrameParams {
            class_param,
            class: int_of(param(class_param)?),
            type_param,
            frame_type: int_of(param(type_param)?),
        })
    });
    if let Some(frame) = pair {
        result.frame = Some(frame);
        if let Some(found) = lookup_frame_layout(frame.class, frame.frame_type) {
            result.layout = Some(found);
        }
        if let Some(layout) = result.layout {
            result.count = layout.motors.len();
            return result;
        }
    }

    // C#: ConfigMotorTest.cs:137-172
    let mav_type = match param("Q_FRAME_CLASS").map(int_of) {
        Some(0 | 1) => TYPE_QUADROTOR,
        Some(2 | 5) => TYPE_HEXAROTOR,
        Some(3 | 4) => TYPE_OCTOROTOR,
        Some(6) => TYPE_HELICOPTER,
        Some(7) => TYPE_TRICOPTER,
        Some(_) => TYPE_QUADROTOR,
        // `FRAME` or `FRAME_TYPE`: the heartbeat's type. A vehicle with only `Q_FRAME_TYPE`
        // keeps the quadrotor the C# starts from.
        None if has("FRAME") || has("FRAME_TYPE") => aptype,
        None => TYPE_QUADROTOR,
    };

    // C#: ConfigMotorTest.cs:174-197
    result.count = match mav_type {
        TYPE_TRICOPTER | TYPE_QUADROTOR => 4,
        TYPE_HEXAROTOR => 6,
        TYPE_OCTOROTOR => 8,
        TYPE_HELICOPTER => 0,
        TYPE_DODECAROTOR => 12,
        _ => DEFAULT_MOTORS,
    };
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// A parameter table.
    fn table(entries: &[(&str, f64)]) -> impl Fn(&str) -> Option<f64> {
        let map: BTreeMap<String, f64> = entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect();
        move |name: &str| map.get(name).copied()
    }

    /// The seven parameters of a built command.
    fn wire(message: &MavMessage) -> (u16, [f32; 7]) {
        match message {
            MavMessage::CommandLong(c) => (
                c.command,
                [
                    c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
                ],
            ),
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    #[test]
    fn a_motor_button_sends_its_number_the_throttle_type_the_boxes_and_no_count() {
        // doCommand(DO_MOTOR_TEST, motor, MOTOR_TEST_THROTTLE_PERCENT, speed, time, 0, 0, 0).
        let (command, params) = wire(&single(3, 5, 2).message(VehicleId::new(1, 1)));
        assert_eq!(command, CMD_DO_MOTOR_TEST);
        assert_eq!(params, [3.0, 0.0, 5.0, 2.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn nothing_bounds_what_the_boxes_hold() {
        // The C# sends the box's value as it is: NUM_thr_percent runs from -100 to 100 and
        // NUM_duration from 0 to 999, and neither is clamped on the way to the wire.
        assert_eq!(
            single(1, 100, 999).params(),
            [1.0, 0.0, 100.0, 999.0, 0.0, 0.0, 0.0]
        );
        assert_eq!(
            single(1, -100, 0).params(),
            [1.0, 0.0, -100.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn test_all_sends_every_motor_in_turn_each_on_its_own() {
        assert_eq!(
            test_all(4, 5, 2),
            [
                single(1, 5, 2),
                single(2, 5, 2),
                single(3, 5, 2),
                single(4, 5, 2)
            ]
        );
        assert!(test_all(0, 5, 2).is_empty(), "a helicopter has no motors");
    }

    #[test]
    fn the_sequence_is_motor_one_with_the_count() {
        let command = sequence(6, 7, 3);
        assert_eq!(command.params(), [1.0, 0.0, 7.0, 3.0, 6.0, 0.0, 0.0]);
    }

    #[test]
    fn stop_all_sends_every_motor_at_zero_for_zero_seconds() {
        let commands = stop_all(4);
        assert_eq!(commands.len(), 4);
        for (index, command) in commands.iter().enumerate() {
            assert_eq!(
                command.params(),
                [
                    f32::from(u8::try_from(index + 1).unwrap()),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ]
            );
        }
    }

    #[test]
    fn buttons_are_lettered_from_a() {
        assert_eq!(letter(1), 'A');
        assert_eq!(letter(4), 'D');
        assert_eq!(letter(12), 'L');
        assert_eq!(button_text(2), "Test motor B");
    }

    #[test]
    fn a_quad_plus_labels_each_button_with_the_motor_tested_at_that_place() {
        // QUAD PLUS: motor 3 is tested first, then 1, 4 and 2.
        let layout = lookup_frame_layout(1, 0);
        assert_eq!(labels(layout, 1), ["Motor Number: 3, CW"]);
        assert_eq!(labels(layout, 2), ["Motor Number: 1, CCW"]);
        assert_eq!(labels(layout, 3), ["Motor Number: 4, CW"]);
        assert_eq!(labels(layout, 4), ["Motor Number: 2, CCW"]);
        assert!(labels(layout, 5).is_empty());
        assert!(labels(None, 1).is_empty());
    }

    #[test]
    fn an_unknown_rotation_is_left_off_the_label() {
        let layout = LAYOUTS
            .iter()
            .find(|layout| layout.motors.iter().any(|motor| motor.rotation == "?"))
            .expect("the file has a layout with an unknown rotation");
        let motor = layout
            .motors
            .iter()
            .find(|motor| motor.rotation == "?")
            .unwrap();
        let a = usize::try_from(motor.test_order).unwrap();
        assert!(
            labels(Some(layout), a).contains(&format!("Motor Number: {}", motor.number)),
            "{:?}",
            labels(Some(layout), a)
        );
    }

    /// SITL's copter: `FRAME_CLASS` 1 and `FRAME_TYPE` 0, a quad plus with four motors.
    #[test]
    fn sitls_quad_has_four_motors_from_its_layout() {
        let count = motor_count(
            table(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 0.0)]),
            TYPE_QUADROTOR,
            None,
        );
        assert_eq!(count.count, 4);
        assert!(count.enabled);
        assert_eq!(count.layout, lookup_frame_layout(1, 0));
        assert_eq!(
            count.frame,
            Some(FrameParams {
                class_param: "FRAME_CLASS",
                class: 1,
                type_param: "FRAME_TYPE",
                frame_type: 0,
            })
        );
    }

    #[test]
    fn the_layout_decides_the_count_whatever_the_heartbeat_says() {
        // A DodecaHexa (class 12) reports as a hexarotor or as anything; the layout has twelve.
        let count = motor_count(
            table(&[("FRAME_CLASS", 12.0), ("FRAME_TYPE", 0.0)]),
            TYPE_HEXAROTOR,
            None,
        );
        assert_eq!(count.count, 12);
        // Deca, class 14: ten.
        let count = motor_count(
            table(&[("FRAME_CLASS", 14.0), ("FRAME_TYPE", 1.0)]),
            TYPE_QUADROTOR,
            None,
        );
        assert_eq!(count.count, 10);
    }

    #[test]
    fn a_rover_or_a_boat_has_four_before_anything_is_read() {
        for aptype in [TYPE_GROUND_ROVER, TYPE_SURFACE_BOAT] {
            let count = motor_count(table(&[]), aptype, None);
            assert_eq!(count.count, 4);
            assert!(
                count.enabled,
                "the rover return comes before the Enabled check"
            );
            assert_eq!(count.frame, None);
        }
    }

    #[test]
    fn a_vehicle_with_no_frame_parameter_disables_the_page_and_has_eight() {
        let count = motor_count(table(&[("ARSPD_TYPE", 1.0)]), 1, None);
        assert!(!count.enabled);
        assert_eq!(count.count, DEFAULT_MOTORS);
    }

    #[test]
    fn a_frame_the_file_lacks_counts_by_vehicle_type() {
        // Heli, class 6: no layout. FRAME_TYPE is there, so the heartbeat's type decides: none.
        let count = motor_count(
            table(&[("FRAME_CLASS", 6.0), ("FRAME_TYPE", 0.0)]),
            TYPE_HELICOPTER,
            None,
        );
        assert_eq!(count.count, 0);
        assert_eq!(count.layout, None);
        // A tricopter with an old FRAME parameter: 4.
        let count = motor_count(table(&[("FRAME", 1.0)]), TYPE_TRICOPTER, None);
        assert_eq!(count.count, 4);
        // An unnamed type keeps the 8 it started from.
        let count = motor_count(table(&[("FRAME_TYPE", 0.0)]), 1, None);
        assert_eq!(count.count, DEFAULT_MOTORS);
        // A dodecarotor by heartbeat: 12.
        let count = motor_count(table(&[("FRAME_TYPE", 0.0)]), TYPE_DODECAROTOR, None);
        assert_eq!(count.count, 12);
    }

    #[test]
    fn a_quadplane_reads_q_frame_class_and_q_frame_type() {
        // A plane's heartbeat, a quadplane's Q_ pair: the layout's four.
        let count = motor_count(
            table(&[("Q_FRAME_CLASS", 1.0), ("Q_FRAME_TYPE", 1.0)]),
            1,
            None,
        );
        assert_eq!(count.count, 4);
        assert_eq!(
            count.frame.map(|frame| frame.class_param),
            Some("Q_FRAME_CLASS")
        );
        // And a Q_FRAME_CLASS the file has no layout for maps to a type: 7 is a tricopter.
        let count = motor_count(
            table(&[("Q_FRAME_CLASS", 7.0), ("Q_FRAME_TYPE", 99.0)]),
            1,
            None,
        );
        assert_eq!(count.count, 4);
        let count = motor_count(
            table(&[("Q_FRAME_CLASS", 3.0), ("Q_FRAME_TYPE", 99.0)]),
            1,
            None,
        );
        assert_eq!(count.count, 8);
        // Only Q_FRAME_TYPE: enabled, no pair, and the quadrotor the C# starts from - whatever
        // the heartbeat says.
        let count = motor_count(table(&[("Q_FRAME_TYPE", 1.0)]), TYPE_HEXAROTOR, None);
        assert!(count.enabled);
        assert_eq!(count.count, 4);
    }

    #[test]
    fn a_layout_found_before_is_kept_when_the_frame_has_none() {
        // `motor_layout` is only ever assigned on a match, so the one found last time stays.
        let quad = lookup_frame_layout(1, 1);
        let count = motor_count(
            table(&[("FRAME_CLASS", 6.0), ("FRAME_TYPE", 0.0)]),
            TYPE_HELICOPTER,
            quad,
        );
        assert_eq!(count.layout, quad);
        assert_eq!(count.count, 4, "the held layout's motors are counted");
    }

    #[test]
    fn parameter_values_are_truncated_as_the_csharps_int_cast() {
        let count = motor_count(
            table(&[("FRAME_CLASS", 1.9), ("FRAME_TYPE", 0.5)]),
            TYPE_QUADROTOR,
            None,
        );
        assert_eq!(
            count.frame.map(|frame| (frame.class, frame.frame_type)),
            Some((1, 0))
        );
    }

    /// Every layout's test orders are 1 to its motor count, so each button has a label.
    #[test]
    fn every_layout_numbers_its_test_order_from_one() {
        for layout in LAYOUTS {
            let mut orders: Vec<i32> = layout.motors.iter().map(|m| m.test_order).collect();
            orders.sort_unstable();
            let expected: Vec<i32> = (1..=i32::try_from(layout.motors.len()).unwrap()).collect();
            assert_eq!(
                orders, expected,
                "class {} type {}",
                layout.class, layout.frame_type
            );
        }
    }

    /// A layout as the file and the table are compared: class, type, and each motor's number,
    /// test order and rotation.
    type Row = (i32, i32, Vec<(i32, i32, String)>);

    /// The table is `APMotorLayout.json`: the version, and every layout's class, type and motors
    /// in the file's order. Skipped when the C# tree is not checked out.
    #[test]
    fn the_table_is_the_csharps_file() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../referneces/missionplanner/APMotorLayout.json");
        let Ok(text) = std::fs::read_to_string(&path) else {
            eprintln!("{} not present; skipped", path.display());
            return;
        };
        // The file is one key per line; read the keys the table carries, in order.
        let mut version = None;
        let mut layouts: Vec<Row> = Vec::new();
        let mut motor: (Option<i32>, Option<i32>) = (None, None);
        for line in text.lines() {
            let line = line.trim().trim_end_matches(',');
            let Some((key, value)) = line.split_once(':') else {
                continue;
            };
            let key = key.trim().trim_matches('"');
            let value = value.trim().trim_matches('"');
            match key {
                "Version" => version = Some(value.to_owned()),
                "Class" => layouts.push((value.parse().unwrap(), -1, Vec::new())),
                "Type" => layouts.last_mut().unwrap().1 = value.parse().unwrap(),
                "Number" => motor.0 = Some(value.parse().unwrap()),
                "TestOrder" => motor.1 = Some(value.parse().unwrap()),
                "Rotation" => {
                    let (Some(number), Some(order)) = motor else {
                        panic!("a rotation before its number and order");
                    };
                    layouts
                        .last_mut()
                        .unwrap()
                        .2
                        .push((number, order, value.to_owned()));
                    motor = (None, None);
                }
                _ => {}
            }
        }
        assert_eq!(version.as_deref(), Some(LAYOUT_VERSION));
        let ours: Vec<Row> = LAYOUTS
            .iter()
            .map(|layout| {
                (
                    layout.class,
                    layout.frame_type,
                    layout
                        .motors
                        .iter()
                        .map(|m| (m.number, m.test_order, m.rotation.to_owned()))
                        .collect(),
                )
            })
            .collect();
        assert_eq!(ours.len(), 75);
        assert_eq!(ours, layouts);
    }
}
