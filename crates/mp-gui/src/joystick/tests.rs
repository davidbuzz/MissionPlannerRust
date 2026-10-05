// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! The page, driven as a person would drive it, against a device the test feeds and - where what
//! goes on the wire matters - the link's scripted vehicle.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_os::Lock as _;
use std::sync::mpsc::{self, Receiver, Sender};
use web_time::{Duration, Instant};

use mp_input::event::{self, AXIS, BUTTON, INIT};
use mp_link::ProtocolTimeouts;
use mp_mavlink_dialects::all::MavMessage;

use super::*;
use crate::telemetry::scripted::{Vehicle, until};

/// A directory of its own, removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let path = mp_os::temp_dir().join(format!(
            "mp-gui-joystick-{name}-{}-{:?}",
            mp_os::process_id(),
            wasm_thread::current().id()
        ));
        let _ = mp_os::fs::remove_dir_all(&path);
        mp_os::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = mp_os::fs::remove_dir_all(&self.0);
    }
}

/// One device, "Test Stick", each opening of it a fresh stream the test writes into.
#[derive(Debug, Clone, Default)]
struct Feeds(Arc<Mutex<Vec<Sender<Vec<u8>>>>>);

impl Feeds {
    /// Bytes to every opening still open.
    fn send(&self, bytes: &[u8]) {
        let feeds = self.0.os_lock().unwrap();
        for feed in feeds.iter() {
            let _ = feed.send(bytes.to_vec());
        }
    }

    /// The device unplugged: every stream ends.
    fn unplug(&self) {
        self.0.os_lock().unwrap().clear();
    }

    fn openings(&self) -> usize {
        self.0.os_lock().unwrap().len()
    }
}

#[derive(Debug)]
struct TestSource(Feeds);

struct Stream {
    chunks: Receiver<Vec<u8>>,
    leftover: Vec<u8>,
}

impl Read for Stream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.leftover.is_empty() {
            match self.chunks.recv() {
                Ok(chunk) => self.leftover = chunk,
                Err(_) => return Ok(0),
            }
        }
        let count = buf.len().min(self.leftover.len());
        buf[..count].copy_from_slice(&self.leftover[..count]);
        self.leftover.drain(..count);
        Ok(count)
    }
}

const NAME: &str = "Test Stick";

impl DeviceSource for TestSource {
    fn devices(&self) -> Vec<Device> {
        vec![Device {
            id: "test".to_owned(),
            name: NAME.to_owned(),
            axes: 4,
            buttons: 3,
        }]
    }

    fn open(&self, _device: &Device) -> std::io::Result<Box<dyn Read + Send>> {
        let (feed, chunks) = mpsc::channel();
        self.0.0.os_lock().unwrap().push(feed);
        Ok(Box::new(Stream {
            chunks,
            leftover: Vec::new(),
        }))
    }
}

/// The opening burst: four axes centred, three buttons up.
fn init() -> Vec<u8> {
    let mut bytes = Vec::new();
    for number in 0..4 {
        bytes.extend(event::encode(0, AXIS | INIT, number, 0));
    }
    for number in 0..3 {
        bytes.extend(event::encode(0, BUTTON | INIT, number, 0));
    }
    bytes
}

fn axis(number: u8, value: i16) -> Vec<u8> {
    event::encode(1, AXIS, number, value).to_vec()
}

fn button(number: u8, down: bool) -> Vec<u8> {
    event::encode(1, BUTTON, number, i16::from(down)).to_vec()
}

fn sticks(scratch: &Scratch) -> (Sticks, Feeds) {
    let feeds = Feeds::default();
    let sticks = Sticks::with_source(Box::new(TestSource(feeds.clone())), scratch.0.clone());
    (sticks, feeds)
}

fn copter_files(scratch: &Scratch) -> ConfigFiles {
    ConfigFiles::for_firmware("ArduCopter2", &scratch.0)
}

/// Frames until `check` holds, the page's timer running with no vehicle.
fn tick_until(sticks: &mut Sticks, what: &str, mut check: impl FnMut(&Sticks) -> bool) {
    let view = TelemetryView::disconnected("test");
    until(what, || {
        sticks.tick(None, &view, None);
        check(sticks)
    });
}

/// The page shown, the display joystick acquired and its buttons reported.
fn loaded(scratch: &Scratch) -> (Sticks, Feeds, Persisted) {
    let (mut sticks, feeds) = sticks(scratch);
    let settings = Persisted::at(None);
    sticks.load(Host::Setup, &settings);
    tick_until(&mut sticks, "the joystick to be acquired", |_| true);
    until("the device to be opened", || feeds.openings() > 0);
    feeds.send(&init());
    tick_until(&mut sticks, "the button rows", |sticks| {
        sticks.page.buttons.len() == 3
    });
    (sticks, feeds, settings)
}

fn fast() -> ProtocolTimeouts {
    ProtocolTimeouts::default().faster(20)
}

/// `Joystick_Load`: the device list and the one chosen, Elevons from the settings, and the rows
/// from the files named for the firmware - a copter's with no vehicle heard.
#[test]
fn load_shows_the_devices_and_the_firmwares_files() {
    let scratch = Scratch::new("load");
    let mut config = JoystickConfig::new();
    config.set_channel(1, JoystickAxis::X, true, 30);
    config.set_axis(3, JoystickAxis::Slider1);
    config.save(&copter_files(&scratch)).unwrap();
    let (mut sticks, _feeds) = sticks(&scratch);
    let mut settings = Persisted::at(None);
    settings.set("joy_elevons", "True");
    sticks.load(Host::Setup, &settings);
    assert_eq!(sticks.page.items, [NAME]);
    assert_eq!(sticks.page.text, NAME);
    assert!(sticks.page.elevons);
    assert_eq!(sticks.page.label, "Loaded Config for ArduCopter2");
    assert_eq!(sticks.page.rows.len(), 16);
    assert_eq!(sticks.page.rows[0].axis, "X");
    assert_eq!(sticks.page.rows[0].expo.value(), "30");
    assert!(sticks.page.rows[0].reverse);
    assert_eq!(sticks.page.rows[2].axis, "Slider1");
    assert_eq!(sticks.page.rows[1].axis, "None");
    assert!(sticks.page.timer, "the Designer starts the timer");
    assert_eq!(sticks.enable_text(), "Enable");
}

/// The timer makes the display joystick, acquires the device, makes a row per button, and the
/// bars follow the stick through the joystick's settings.
#[test]
fn the_timer_acquires_the_device_and_the_bars_follow_the_stick() {
    let scratch = Scratch::new("timer");
    let mut config = JoystickConfig::new();
    config.set_axis(1, JoystickAxis::X);
    config.set_axis(2, JoystickAxis::Y);
    config.save(&copter_files(&scratch)).unwrap();
    let (mut sticks, feeds, _settings) = loaded(&scratch);
    assert!(sticks.joystick().unwrap().is_acquired());
    assert_eq!(sticks.page.buttons[0].number, "-1");
    assert_eq!(sticks.page.buttons[0].action, "ChangeMode");
    tick_until(&mut sticks, "the centred bars", |s| s.page.rc[0] == 1500);
    feeds.send(&axis(0, 16_384));
    feeds.send(&axis(1, -32_767));
    tick_until(&mut sticks, "the bar to follow", |s| {
        s.page.rc[0] == 1749 && s.page.rc[1] == 1000
    });
    assert_eq!(
        sticks.page.rc[2], 1500,
        "a channel with no axis shows its trim"
    );
}

/// A row's axis, reverse and expo go to the joystick as they change - the bar shows them - and
/// Save writes them where the joystick found its files, with Elevons to the settings.
#[test]
fn the_rows_drive_the_joystick_and_save_writes_them() {
    let scratch = Scratch::new("save");
    let (mut sticks, feeds, mut settings) = loaded(&scratch);
    sticks.choose_axis(0, JoystickAxis::X);
    sticks.toggle_reverse(0);
    sticks.begin_expo(0);
    sticks.page.rows[0].expo.set("");
    sticks.type_text("5");
    sticks.type_text("0");
    assert_eq!(sticks.joystick().unwrap().config().channel(1).expo, 50);
    feeds.send(&axis(0, 16_384));
    tick_until(&mut sticks, "reversed and softened", |s| {
        s.page.rc[0] == 1375
    });

    sticks.toggle_elevons();
    sticks.save_click(&mut settings);
    assert_eq!(sticks.page.saves, 1);
    let saved = JoystickConfig::load(&copter_files(&scratch));
    assert_eq!(saved.channel(1).axis, JoystickAxis::X);
    assert!(saved.channel(1).reverse);
    assert_eq!(saved.channel(1).expo, 50);
    // The timer's `setChannel` numbered the sixteen, as the C#'s file numbers them; the slots
    // beyond keep the struct's zero.
    for channel in 1..=CHANNEL_ROWS {
        assert_eq!(
            saved.channel(channel).channel,
            i32::try_from(channel).unwrap()
        );
    }
    assert_eq!(saved.channel(17).channel, 0);
    assert_eq!(settings.get("joy_elevons"), Some("True"));
    assert_eq!(sticks.page.saved.as_ref().unwrap().channel(1).expo, 50);
}

/// Save with no joystick says so on the status line, not in a box.
#[test]
fn save_without_a_joystick_is_said_on_the_status_line() {
    let scratch = Scratch::new("nosave");
    let (mut sticks, _feeds) = sticks(&scratch);
    let mut settings = Persisted::at(None);
    sticks.save_click(&mut settings);
    assert_eq!(sticks.take_said(), [SELECT_JOYSTICK]);
    assert!(sticks.page.message.is_none());
}

/// The frames RC_CHANNELS_OVERRIDE carries while flying, as the vehicle hears them.
fn overrides_heard(vehicle: &mut Vehicle) -> Vec<[u16; 8]> {
    vehicle
        .read()
        .into_iter()
        .filter_map(|message| match message {
            MavMessage::RcChannelsOverride(rc) => Some([
                rc.chan1_raw,
                rc.chan2_raw,
                rc.chan3_raw,
                rc.chan4_raw,
                rc.chan5_raw,
                rc.chan6_raw,
                rc.chan7_raw,
                rc.chan8_raw,
            ]),
            _ => None,
        })
        .collect()
}

/// Enable makes a joystick from the files - not from the rows - and flies it: the vehicle hears
/// `RC_CHANNELS_OVERRIDE` with the stick's position through the saved reverse and expo, and
/// "ignore" on channels with no axis. Disable hands control back.
#[test]
fn enable_flies_the_saved_settings_and_disable_releases() {
    let scratch = Scratch::new("enable");
    let mut config = JoystickConfig::new();
    config.set_channel(1, JoystickAxis::X, true, 50);
    config.set_axis(3, JoystickAxis::Y);
    config.save(&copter_files(&scratch)).unwrap();
    let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
    let (mut sticks, feeds, mut settings) = loaded(&scratch);
    // A row changed and not saved: Enable does not fly it.
    sticks.choose_axis(1, JoystickAxis::Z);
    let view = telemetry.view();
    sticks.tick(telemetry.send_handle(), &view, None);
    sticks.enable_click(&mut settings);
    assert!(sticks.is_enabled());
    assert_eq!(sticks.enable_text(), "Disable");
    assert_eq!(settings.get("joystick_name"), Some(NAME));
    feeds.send(&init());
    feeds.send(&axis(0, 16_384));
    feeds.send(&axis(1, -32_767));
    // The two axes are two reads, and a frame may go out between them: the one to judge is the
    // frame carrying both.
    let mut last = None;
    until("the stick on the wire", || {
        if let Some(frame) = overrides_heard(&mut vehicle).last() {
            last = Some(*frame);
        }
        last.is_some_and(|frame| frame[0] == 1375 && frame[2] == 1000)
    });
    let frame = last.unwrap();
    assert_eq!(frame[1], u16::MAX, "channel 2's unsaved axis is not flown");
    assert_eq!(frame[3], u16::MAX);
    let _ = telemetry.take_reports();

    sticks.enable_click(&mut settings);
    assert!(!sticks.is_enabled());
    assert!(sticks.joystick().is_none());
    until("the release", || {
        overrides_heard(&mut vehicle)
            .iter()
            .any(|frame| frame.iter().all(|channel| *channel == 0))
    });
}

/// Enable with no device says "Please Connect a Joystick" on the status line.
#[test]
fn enable_without_the_device_is_said_on_the_status_line() {
    let scratch = Scratch::new("nodevice");
    let (mut sticks, _feeds) = sticks(&scratch);
    let mut settings = Persisted::at(None);
    sticks.load(Host::Setup, &settings);
    sticks.page.text = "Another Stick".to_owned();
    sticks.enable_click(&mut settings);
    assert!(!sticks.is_enabled());
    assert_eq!(sticks.take_said(), [NO_JOYSTICK]);
    assert_eq!(settings.get("joystick_name"), None);
}

/// Manual Control: `MANUAL_CONTROL` rather than `RC_CHANNELS_OVERRIDE`, -1000 to 1000, its
/// target the vehicle's component id as the C# fills it.
#[test]
fn manual_control_sends_manual_control() {
    let scratch = Scratch::new("manual");
    let mut config = JoystickConfig::new();
    config.set_axis(1, JoystickAxis::X);
    config.set_axis(3, JoystickAxis::Y);
    config.save(&copter_files(&scratch)).unwrap();
    let (telemetry, mut vehicle) = Vehicle::connect(fast());
    let (mut sticks, feeds, mut settings) = loaded(&scratch);
    let view = telemetry.view();
    sticks.tick(telemetry.send_handle(), &view, None);
    sticks.enable_click(&mut settings);
    sticks.toggle_manual();
    feeds.send(&axis(0, 16_384));
    let mut heard = None;
    until("MANUAL_CONTROL", || {
        for message in vehicle.read() {
            if let MavMessage::ManualControl(manual) = message
                && manual.x == 498
            {
                heard = Some(manual);
            }
        }
        heard.is_some()
    });
    let manual = heard.unwrap();
    assert_eq!(manual.z, 0, "throttle centred: trim (min + max) / 2");
    assert_eq!(manual.y, 0, "channel 2 has no axis");
    assert_eq!(manual.target, 1);
}

/// A button given `Do_Set_Servo` and pressed while flying sends `DO_SET_SERVO` with its servo and
/// PWM; one given `ChangeMode` sends the mode.
#[test]
fn a_buttons_function_sends_its_command() {
    let scratch = Scratch::new("buttons");
    let mut config = JoystickConfig::new();
    config.set_button(
        0,
        JoyButton {
            buttonno: 1,
            function: ButtonFunction::DoSetServo,
            p1: 9.0,
            p2: 1700.0,
            ..JoyButton::unassigned()
        },
    );
    config.set_button(
        1,
        JoyButton {
            buttonno: 2,
            function: ButtonFunction::ChangeMode,
            mode: Some("Loiter".to_owned()),
            ..JoyButton::unassigned()
        },
    );
    config.save(&copter_files(&scratch)).unwrap();
    let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
    let (mut sticks, feeds, mut settings) = loaded(&scratch);
    let view = telemetry.view();
    sticks.tick(telemetry.send_handle(), &view, None);
    sticks.enable_click(&mut settings);
    feeds.send(&button(1, true));
    feeds.send(&button(2, true));
    let mut servo = None;
    let mut mode = None;
    until("the two functions", || {
        for event in sticks.take_button_events() {
            assert_eq!(
                perform(&event, &mut telemetry, &view, Firmware::ArduCopter2),
                None
            );
        }
        for message in vehicle.read() {
            match message {
                MavMessage::CommandLong(command) if command.command == 183 => {
                    servo = Some((command.param1, command.param2));
                }
                MavMessage::SetMode(set) => mode = Some(set.custom_mode),
                _ => {}
            }
        }
        servo.is_some() && mode.is_some()
    });
    assert_eq!(servo, Some((9.0, 1700.0)));
    assert_eq!(mode, Some(5), "Loiter is copter mode 5");
}

/// `Toggle_Pan_Stab` on a vehicle without `MNT_STAB_PAN` says the C#'s failure on the status line.
#[test]
fn a_function_that_fails_says_so() {
    let (mut telemetry, _vehicle) = Vehicle::connect(fast());
    let view = telemetry.view();
    let event = ButtonEvent {
        slot: 0,
        button: JoyButton {
            buttonno: 0,
            function: ButtonFunction::TogglePanStab,
            ..JoyButton::unassigned()
        },
        down: true,
    };
    assert_eq!(
        perform(&event, &mut telemetry, &view, Firmware::ArduCopter2).as_deref(),
        Some("Error: Failed to Toggle_Pan_Stab")
    );
}

/// A row's number and function go to the joystick; Settings opens the function's form, whose
/// boxes write the function's parameters; a function with no form says "No settings to set".
#[test]
fn the_button_rows_and_their_forms() {
    let scratch = Scratch::new("forms");
    let (mut sticks, _feeds, _settings) = loaded(&scratch);
    sticks.toggle_list(List::Number(0));
    let (rows, _) = sticks.list_rows(List::Number(0));
    assert_eq!(rows.len(), 128);
    let two = rows.iter().position(|(suffix, _)| suffix == "2").unwrap();
    sticks.choose_from_list(two);
    assert_eq!(sticks.page.buttons[0].number, "2");
    assert_eq!(sticks.joystick().unwrap().config().button(0).buttonno, 2);

    sticks.choose_action(0, ButtonFunction::DoSetServo);
    assert_eq!(
        sticks.joystick().unwrap().config().button(0).function,
        ButtonFunction::DoSetServo
    );
    sticks.settings_click(0);
    let form = sticks.page.form.as_ref().unwrap();
    assert_eq!(form.kind, FormKind::DoSetServo);
    sticks.form_step(0, true);
    sticks.form_step(1, true);
    let written = sticks.joystick().unwrap().config().button(0);
    assert!((written.p1 - 1.0).abs() < f32::EPSILON);
    assert!((written.p2 - 901.0).abs() < f32::EPSILON);
    sticks.form_begin(1);
    sticks.page.form.as_mut().unwrap().numbers[1]
        .text
        .set("1234");
    sticks.form_close();
    assert!(sticks.page.form.is_none());
    let written = sticks.joystick().unwrap().config().button(0);
    assert!(
        (written.p2 - 1234.0).abs() < f32::EPSILON,
        "read as it lost the focus"
    );

    sticks.choose_action(1, ButtonFunction::Arm);
    sticks.settings_click(1);
    assert!(sticks.page.form.is_none());
    assert_eq!(sticks.page.message.as_ref().unwrap().text, NO_SETTINGS);
    sticks.message_ok();
    assert!(sticks.page.message.is_none());
}

/// Auto Detect: after the instruction's OK, the first axis to move 16000 is the row's.
#[test]
fn auto_detect_finds_the_axis_that_moves() {
    let scratch = Scratch::new("detect");
    let (mut sticks, feeds, _settings) = loaded(&scratch);
    sticks.detect_axis(4);
    assert_eq!(sticks.page.message.as_ref().unwrap().text, MOVE_AXIS);
    until("the detector's device", || feeds.openings() > 1);
    feeds.send(&init());
    wasm_thread::sleep(Duration::from_millis(50));
    sticks.message_ok();
    feeds.send(&axis(1, 30_000));
    tick_until(&mut sticks, "the axis found", |s| s.page.detect.is_none());
    assert_eq!(sticks.page.rows[4].axis, "Y");
    assert_eq!(
        sticks.joystick().unwrap().config().channel(5).axis,
        JoystickAxis::Y
    );
}

/// Detect on a button row: the first button to change is the row's.
#[test]
fn detect_finds_the_button_pressed() {
    let scratch = Scratch::new("detectbutton");
    let (mut sticks, feeds, _settings) = loaded(&scratch);
    sticks.detect_button(1);
    assert_eq!(sticks.page.message.as_ref().unwrap().text, PRESS_BUTTON);
    until("the detector's device", || feeds.openings() > 1);
    feeds.send(&init());
    wasm_thread::sleep(Duration::from_millis(50));
    sticks.message_ok();
    feeds.send(&button(2, true));
    tick_until(&mut sticks, "the button found", |s| s.page.detect.is_none());
    assert_eq!(sticks.page.buttons[1].number, "2");
    assert_eq!(sticks.joystick().unwrap().config().button(1).buttonno, 2);
}

/// Detect for a device that is not there answers at once: `ARx` for an axis, -1 for a button.
#[test]
fn detect_for_a_missing_device_answers_at_once() {
    let scratch = Scratch::new("detectnone");
    let (mut sticks, _feeds, _settings) = loaded(&scratch);
    sticks.choose_button_number(0, 1);
    sticks.page.text = "Gone".to_owned();
    sticks.detect_axis(0);
    assert_eq!(sticks.page.rows[0].axis, "ARx");
    sticks.detect_button(0);
    assert_eq!(sticks.page.buttons[0].number, "-1");
    assert!(sticks.page.message.is_none());
}

/// An entry's name is taken as `ZipArchiveEntry.Name` takes it on Windows - after the last `/`,
/// `\` or `:` - so no entry of an imported file is written outside the joystick folder.
#[test]
fn an_import_entry_cannot_climb_out_of_the_folder() {
    let climbing = "joystickaxis\\..\\..\\evil";
    assert_eq!(entry_name(climbing), "evil");
    assert!(!mp_input::config::is_config_file(entry_name(climbing)));
    assert_eq!(
        entry_name("folder/joystickaxis_copter.xml"),
        "joystickaxis_copter.xml"
    );
    assert_eq!(entry_name("C:joystickaxis.xml"), "joystickaxis.xml");
}

/// Export zips the joystick files; Import puts them back, loads the default names, and closes
/// the page after its instruction - and the Planner's window with it.
#[test]
fn export_and_import_round_trip() {
    let scratch = Scratch::new("export");
    let (mut sticks, _feeds) = sticks(&scratch);
    let settings = Persisted::at(None);
    sticks.load(Host::Planner, &settings);
    tick_until(&mut sticks, "the joystick", |s| s.joystick().is_some());
    sticks.choose_axis(0, JoystickAxis::Rz);
    let archive = scratch.0.join("mine.joycfg");
    sticks.export_click();
    assert_eq!(sticks.page.editing, Some(Editing::Path));
    sticks
        .page
        .path
        .as_mut()
        .unwrap()
        .1
        .field
        .set(archive.display().to_string());
    sticks.path_done(true);
    assert_eq!(sticks.page.exported, Some((archive.clone(), 2)));

    // The files gone, the archive brings them back - by their own names.
    let files = copter_files(&scratch);
    mp_os::fs::remove_file(&files.axis).unwrap();
    mp_os::fs::remove_file(&files.buttons).unwrap();
    sticks.import_click();
    assert!(sticks.page.question);
    sticks.question_answer(true);
    sticks
        .page
        .path
        .as_mut()
        .unwrap()
        .1
        .field
        .set(archive.display().to_string());
    sticks.path_done(true);
    assert_eq!(
        JoystickConfig::load(&files).channel(1).axis,
        JoystickAxis::Rz
    );
    assert_eq!(sticks.page.message.as_ref().unwrap().text, REOPEN);
    assert_eq!(
        sticks.joystick().unwrap().files.axis,
        scratch.0.join("joystickaxis.xml"),
        "loadconfig() takes the default names"
    );
    sticks.message_ok();
    assert_eq!(sticks.page.host, None);
    assert!(sticks.take_close_parent());
    assert!(sticks.joystick().is_none(), "not flying, so let go");
    assert!(export_config(&scratch.0.join("empty"), &archive).is_err());
}

/// The flight screen's Disable Joystick hands control back - the vehicle hears the release - and,
/// unlike the page's Disable, keeps the joystick, acquired and not flying.
#[test]
fn the_flight_screens_disable_releases_and_keeps_the_joystick() {
    let scratch = Scratch::new("flydisable");
    let mut config = JoystickConfig::new();
    config.set_axis(1, JoystickAxis::X);
    config.save(&copter_files(&scratch)).unwrap();
    let (telemetry, mut vehicle) = Vehicle::connect(fast());
    let (mut sticks, feeds, mut settings) = loaded(&scratch);
    let view = telemetry.view();
    sticks.tick(telemetry.send_handle(), &view, None);
    sticks.enable_click(&mut settings);
    feeds.send(&init());
    feeds.send(&axis(0, 16_384));
    until("the stick on the wire", || {
        overrides_heard(&mut vehicle)
            .iter()
            .any(|frame| frame[0] == 1749)
    });
    sticks.disable_joystick();
    assert!(!sticks.is_enabled());
    assert_eq!(sticks.enable_text(), "Enable");
    let kept = sticks.joystick().expect("the joystick is kept");
    assert!(kept.is_acquired(), "and its device with it");
    until("the release", || {
        overrides_heard(&mut vehicle)
            .iter()
            .any(|frame| frame.iter().all(|channel| *channel == 0))
    });
}

/// Leaving the page lets a joystick that is not flying go, and keeps one that is.
#[test]
fn leaving_the_page_keeps_only_a_flying_joystick() {
    let scratch = Scratch::new("leave");
    let (mut sticks, _feeds, mut settings) = loaded(&scratch);
    sticks.close();
    assert!(sticks.joystick().is_none());
    assert!(!sticks.page.timer);
    sticks.load(Host::Setup, &settings);
    sticks.enable_click(&mut settings);
    assert!(sticks.is_enabled());
    sticks.close();
    assert!(sticks.is_enabled(), "flying on after the page is left");
    sticks.stop_flying();
    assert!(!sticks.is_enabled());
}

/// A drop-down shows thirty rows, opens with its selected row in view, and the wheel moves them
/// within the list's length: the 128 button numbers of `getButtonNumbers`.
#[test]
fn the_button_number_list_opens_on_its_row_and_scrolls_within_itself() {
    let scratch = Scratch::new("scroll");
    let (mut sticks, _feeds, _settings) = loaded(&scratch);
    sticks.choose_button_number(0, 126);
    sticks.toggle_list(List::Number(0));
    let (rows, selected) = sticks.list_rows(List::Number(0));
    assert_eq!(rows.first().map(|(_, text)| text.as_str()), Some("-1"));
    assert_eq!(rows.last().map(|(_, text)| text.as_str()), Some("126"));
    assert_eq!(selected, Some(127));
    assert_eq!(
        sticks.page.top, 98,
        "the selected row is the last of the thirty shown"
    );
    sticks.scroll_list(-500);
    assert_eq!(sticks.page.top, 0);
    sticks.scroll_list(3);
    assert_eq!(sticks.page.top, 3);
    sticks.scroll_list(500);
    assert_eq!(sticks.page.top, 98, "no further than the last thirty");
    sticks.toggle_list(List::Number(0));
    assert_eq!(sticks.page.open, None, "a second click puts it away");
}

/// Clicking the device list lists again and selects the first, which lets the display
/// joystick's device go, as the C#'s `SelectedIndexChanged` does.
#[test]
fn clicking_the_device_list_lets_the_display_joystick_go() {
    let scratch = Scratch::new("devices");
    let (mut sticks, _feeds, _settings) = loaded(&scratch);
    assert!(sticks.joystick().unwrap().is_acquired());
    sticks.devices_click();
    assert_eq!(sticks.page.open, Some(List::Devices));
    assert_eq!(sticks.page.text, NAME);
    assert!(!sticks.joystick().unwrap().is_acquired());
    sticks.choose_from_list(0);
    assert_eq!(sticks.page.open, None);
}

/// A device lost while flying: control handed back by the reader, and "Lost Joystick" said.
#[test]
fn a_device_lost_while_flying_is_said() {
    let scratch = Scratch::new("lost");
    let (mut sticks, feeds, mut settings) = loaded(&scratch);
    sticks.enable_click(&mut settings);
    tick_until(&mut sticks, "flying", Sticks::is_enabled);
    feeds.unplug();
    let started = Instant::now();
    tick_until(&mut sticks, "the loss", |s| s.page.status.is_some());
    assert!(started.elapsed() < Duration::from_secs(2));
    assert_eq!(sticks.take_said().last().map(String::as_str), Some(LOST));
    assert!(!sticks.is_enabled());
}

/// Matches `text` against a format string, each `{...}` standing for one or more characters
/// without a dot or a dash.
fn matches_format(format: &str, text: &str) -> bool {
    match format.find('{') {
        None => format == text,
        Some(open) => {
            let Some(close) = format[open..].find('}') else {
                return false;
            };
            let (head, rest) = (&format[..open], &format[open + close + 1..]);
            let Some(tail) = text.strip_prefix(head) else {
                return false;
            };
            (1..=tail.len()).any(|taken| {
                tail.is_char_boundary(taken)
                    && !tail[..taken].contains(['.', '-'])
                    && matches_format(rest, &tail[taken..])
            })
        }
    }
}

/// The string literals in a source file that contain `needle`.
fn literals(source: &str, needle: &str) -> Vec<String> {
    source
        .lines()
        .filter(|line| !line.trim_start().starts_with("//"))
        .flat_map(|line| line.split('"').skip(1).step_by(2))
        .filter(|literal| literal.contains(needle))
        .map(str::to_owned)
        .collect()
}

/// Every fact the GUI script asserts on is one this page records, and every control it clicks is
/// one this page draws - a list's rows being the list's id and a row's suffix.
#[test]
fn the_gui_script_names_facts_and_controls_this_page_has() {
    let script = include_str!("../../../../tests/gui/config-joystick.gui");
    let facts = literals(include_str!("../joystick.rs"), "config.joystick.");
    let controls = literals(include_str!("draw.rs"), "joystick-");
    let (mut expected, mut clicked) = (0, 0);
    for line in script.lines() {
        let line = line.split('#').next().unwrap_or("");
        let mut words = line.split_whitespace();
        match (words.next(), words.next()) {
            (Some("expect"), Some(key)) if key.starts_with("config.joystick.") => {
                assert!(
                    facts.iter().any(|format| matches_format(format, key)),
                    "{key} is not recorded"
                );
                expected += 1;
            }
            (Some("click"), Some(id)) if id.starts_with("joystick-") => {
                let id = id.split('@').next().unwrap_or(id);
                let drawn = controls.iter().any(|format| {
                    matches_format(format, id)
                        || id
                            .strip_prefix(format.as_str())
                            .is_some_and(|rest| rest.starts_with('-'))
                        || (format.contains('{')
                            && id
                                .char_indices()
                                .any(|(at, c)| c == '-' && matches_format(format, &id[..at])))
                });
                assert!(drawn, "{id} is not drawn");
                clicked += 1;
            }
            _ => {}
        }
    }
    assert!(
        expected > 15 && clicked > 8,
        "{expected} facts, {clicked} clicks"
    );
}
