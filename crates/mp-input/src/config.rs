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

//! The joystick's settings as Mission Planner keeps them, and the files it keeps them in.
//!
//! `JoystickBase` holds two arrays: `JoyChannels`, one [`JoyChannel`] per RC channel - which axis
//! drives it, reversed or not, and how much expo - and `JoyButtons`, one [`JoyButton`] per button
//! function - which of the device's buttons triggers it, what it does, and that function's
//! settings. `saveconfig` writes each array with .NET's `XmlSerializer` to `joystickaxis.xml` and
//! `joystickbuttons.xml` in the user data directory (with the firmware's name in the file name for
//! a plane, a copter and a rover), and `loadconfig` reads them back. The files here are written as
//! that serializer writes them and read as it reads them, so a configuration moves between Mission
//! Planner and this application in either direction.
//! `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:29-30, 78-96, 107-182; JoyChannel.cs; JoyButton.cs`

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

/// How many `JoyChannel`s the configuration holds: `new JoyChannel[20]`, "we are base 1", and
/// `Array.Resize(ref JoyChannels, 20)` after every load.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:157, 181`
pub const CHANNEL_SLOTS: usize = 20;

/// How many `JoyButton`s: `new JoyButton[128]`, "base 0".
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:182`
pub const BUTTON_SLOTS: usize = 128;

/// `joystickaxis`: what drives a channel.
///
/// The named axes are DirectInput's. On Linux, where this application reads `/dev/input/js*`,
/// Mission Planner's `LinuxJoystickState` gives eleven of them a `js` axis number and leaves the
/// rest reading zero; [`crate::mapping`] reads them the same way.
/// `// C#: ExtLibs/ArduPilot/Joystick/joystickaxis.cs; MyJoystickState.cs:8-101`
#[allow(missing_docs)] // the C#'s names, each documented by `crate::mapping::axis_value`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum JoystickAxis {
    /// Not driven: the channel is not overridden.
    #[default]
    None,
    /// The channel's trim, whatever the stick does.
    Pass,
    ARx,
    ARy,
    ARz,
    AX,
    AY,
    AZ,
    FRx,
    FRy,
    FRz,
    FX,
    FY,
    FZ,
    Rx,
    Ry,
    Rz,
    VRx,
    VRy,
    VRz,
    VX,
    VY,
    VZ,
    X,
    Y,
    Z,
    Slider1,
    Slider2,
    Hatud1,
    Hatlr2,
    /// Driven by the buttons given `Button_axis0`.
    Custom1,
    /// Driven by the buttons given `Button_axis1`.
    Custom2,
    /// `UINT16_MAX`: the channel's field sent as 65535, "ignore this field".
    Uint16Max,
}

impl JoystickAxis {
    /// Every value, in the enum's order - the order `Enum.GetValues` gives the axis pickers.
    /// `// C#: Joystick/JoystickSetup.cs:83`
    pub const ALL: [Self; 33] = [
        Self::None,
        Self::Pass,
        Self::ARx,
        Self::ARy,
        Self::ARz,
        Self::AX,
        Self::AY,
        Self::AZ,
        Self::FRx,
        Self::FRy,
        Self::FRz,
        Self::FX,
        Self::FY,
        Self::FZ,
        Self::Rx,
        Self::Ry,
        Self::Rz,
        Self::VRx,
        Self::VRy,
        Self::VRz,
        Self::VX,
        Self::VY,
        Self::VZ,
        Self::X,
        Self::Y,
        Self::Z,
        Self::Slider1,
        Self::Slider2,
        Self::Hatud1,
        Self::Hatlr2,
        Self::Custom1,
        Self::Custom2,
        Self::Uint16Max,
    ];

    /// The C#'s name: what the picker shows and the file holds.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "None",
            Self::Pass => "Pass",
            Self::ARx => "ARx",
            Self::ARy => "ARy",
            Self::ARz => "ARz",
            Self::AX => "AX",
            Self::AY => "AY",
            Self::AZ => "AZ",
            Self::FRx => "FRx",
            Self::FRy => "FRy",
            Self::FRz => "FRz",
            Self::FX => "FX",
            Self::FY => "FY",
            Self::FZ => "FZ",
            Self::Rx => "Rx",
            Self::Ry => "Ry",
            Self::Rz => "Rz",
            Self::VRx => "VRx",
            Self::VRy => "VRy",
            Self::VRz => "VRz",
            Self::VX => "VX",
            Self::VY => "VY",
            Self::VZ => "VZ",
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
            Self::Slider1 => "Slider1",
            Self::Slider2 => "Slider2",
            Self::Hatud1 => "Hatud1",
            Self::Hatlr2 => "Hatlr2",
            Self::Custom1 => "Custom1",
            Self::Custom2 => "Custom2",
            Self::Uint16Max => "UINT16_MAX",
        }
    }

    /// `Enum.Parse(typeof(joystickaxis), name)`: the value of that name, exactly as spelled.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|axis| axis.name() == name)
    }
}

/// `buttonfunction`: what a button does.
/// `// C#: ExtLibs/ArduPilot/Joystick/buttonfunction.cs`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum ButtonFunction {
    /// Changes flight mode to the button's `mode`.
    #[default]
    ChangeMode,
    /// `MAV_CMD_DO_SET_RELAY`: relay `p1` on while held, off when let go.
    DoSetRelay,
    /// `MAV_CMD_DO_REPEAT_RELAY`: relay `p1`, `p2` times, `p3` seconds apart.
    DoRepeatRelay,
    /// `MAV_CMD_DO_SET_SERVO`: servo `p1` to `p2`.
    DoSetServo,
    /// `MAV_CMD_DO_REPEAT_SERVO`: servo `p1` to `p2`, `p3` times, `p4` apart.
    DoRepeatServo,
    /// Arms.
    Arm,
    /// Disarms.
    Disarm,
    /// Triggers the camera.
    DigicamControl,
    /// Guided, then a take-off.
    TakeOff,
    /// `MNT_MODE` set to `p1`.
    MountMode,
    /// `MNT_STAB_PAN` toggled.
    TogglePanStab,
    /// `MAV_CMD_DO_SET_ROI` at the point the gimbal looks at.
    GimbalPntTrack,
    /// The mount pointed at zero.
    MountControl0,
    /// The `Custom1` axis: `p2` while held, `p1` when let go.
    ButtonAxis0,
    /// The `Custom2` axis, likewise.
    ButtonAxis1,
}

impl ButtonFunction {
    /// Every value, in the enum's order - the order `Enum.GetNames` gives the function pickers.
    /// `// C#: Joystick/JoystickSetup.cs:412`
    pub const ALL: [Self; 15] = [
        Self::ChangeMode,
        Self::DoSetRelay,
        Self::DoRepeatRelay,
        Self::DoSetServo,
        Self::DoRepeatServo,
        Self::Arm,
        Self::Disarm,
        Self::DigicamControl,
        Self::TakeOff,
        Self::MountMode,
        Self::TogglePanStab,
        Self::GimbalPntTrack,
        Self::MountControl0,
        Self::ButtonAxis0,
        Self::ButtonAxis1,
    ];

    /// The C#'s name: what the picker shows and the file holds.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::ChangeMode => "ChangeMode",
            Self::DoSetRelay => "Do_Set_Relay",
            Self::DoRepeatRelay => "Do_Repeat_Relay",
            Self::DoSetServo => "Do_Set_Servo",
            Self::DoRepeatServo => "Do_Repeat_Servo",
            Self::Arm => "Arm",
            Self::Disarm => "Disarm",
            Self::DigicamControl => "Digicam_Control",
            Self::TakeOff => "TakeOff",
            Self::MountMode => "Mount_Mode",
            Self::TogglePanStab => "Toggle_Pan_Stab",
            Self::GimbalPntTrack => "Gimbal_pnt_track",
            Self::MountControl0 => "Mount_Control_0",
            Self::ButtonAxis0 => "Button_axis0",
            Self::ButtonAxis1 => "Button_axis1",
        }
    }

    /// `Enum.Parse(typeof(buttonfunction), name)`.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|function| function.name() == name)
    }
}

/// One RC channel's settings.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoyChannel.cs`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct JoyChannel {
    /// `channel`: the channel number, which `setChannel` fills in and `setAxis` does not.
    pub channel: i32,
    /// `axis`.
    pub axis: JoystickAxis,
    /// `reverse`.
    pub reverse: bool,
    /// `expo`, a percentage.
    pub expo: i32,
}

/// One button function's settings.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoyButton.cs`
#[derive(Debug, Clone, PartialEq, Default)]
pub struct JoyButton {
    /// `buttonno`: the device's button that triggers it, or -1 for none.
    pub buttonno: i32,
    /// `function`.
    pub function: ButtonFunction,
    /// `mode`: the mode `ChangeMode` changes to; `null` until chosen.
    pub mode: Option<String>,
    /// `p1`.
    pub p1: f32,
    /// `p2`.
    pub p2: f32,
    /// `p3`.
    pub p3: f32,
    /// `p4`.
    pub p4: f32,
    /// `state`, which nothing reads.
    pub state: bool,
}

impl JoyButton {
    /// A slot as the constructor leaves it: `buttonno = -1`, everything else the struct's zero.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:72-73`
    #[must_use]
    pub fn unassigned() -> Self {
        Self {
            buttonno: -1,
            ..Self::default()
        }
    }
}

/// `JoyChannels` and `JoyButtons`.
#[derive(Debug, Clone, PartialEq)]
pub struct JoystickConfig {
    /// `JoyChannels`, base 1: slot 0 is never a channel.
    pub channels: Vec<JoyChannel>,
    /// `JoyButtons`, base 0.
    pub buttons: Vec<JoyButton>,
}

impl Default for JoystickConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl JoystickConfig {
    /// What the constructor makes before it loads anything: twenty channels driven by nothing,
    /// and 128 button functions triggered by no button.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:62-73, 181-182`
    #[must_use]
    pub fn new() -> Self {
        Self {
            channels: vec![JoyChannel::default(); CHANNEL_SLOTS],
            buttons: vec![JoyButton::unassigned(); BUTTON_SLOTS],
        }
    }

    /// `getChannel(channel)`; the struct's zero for a channel beyond the array, where the C#
    /// would throw.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:255-258`
    #[must_use]
    pub fn channel(&self, channel: usize) -> JoyChannel {
        self.channels.get(channel).copied().unwrap_or_default()
    }

    fn channel_mut(&mut self, channel: usize) -> Option<&mut JoyChannel> {
        self.channels.get_mut(channel)
    }

    /// `setAxis(channel, axis)`.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:229-232`
    pub fn set_axis(&mut self, channel: usize, axis: JoystickAxis) {
        if let Some(slot) = self.channel_mut(channel) {
            slot.axis = axis;
        }
    }

    /// `setReverse(channel, reverse)`.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:224-227`
    pub fn set_reverse(&mut self, channel: usize, reverse: bool) {
        if let Some(slot) = self.channel_mut(channel) {
            slot.reverse = reverse;
        }
    }

    /// `setExpo(channel, expo)`.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:234-237`
    pub fn set_expo(&mut self, channel: usize, expo: i32) {
        if let Some(slot) = self.channel_mut(channel) {
            slot.expo = expo;
        }
    }

    /// `setChannel(channel, axis, reverse, expo)`: the slot replaced, its number filled in.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:239-248`
    pub fn set_channel(&mut self, channel: usize, axis: JoystickAxis, reverse: bool, expo: i32) {
        if let Some(slot) = self.channel_mut(channel) {
            *slot = JoyChannel {
                channel: i32::try_from(channel).unwrap_or(i32::MAX),
                axis,
                reverse,
                expo,
            };
        }
    }

    /// `getButton(offset)`; an unassigned slot beyond the array, where the C# would throw.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:265-268`
    #[must_use]
    pub fn button(&self, offset: usize) -> JoyButton {
        self.buttons
            .get(offset)
            .cloned()
            .unwrap_or_else(JoyButton::unassigned)
    }

    /// `setButton(offset, config)`.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:260-263`
    pub fn set_button(&mut self, offset: usize, button: JoyButton) {
        if let Some(slot) = self.buttons.get_mut(offset) {
            *slot = button;
        }
    }

    /// `changeButton(buttonid, newid)`: which device button triggers a function.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:270-273`
    pub fn change_button(&mut self, offset: usize, buttonno: i32) {
        if let Some(slot) = self.buttons.get_mut(offset) {
            slot.buttonno = buttonno;
        }
    }

    /// `loadconfig`'s reading: the constructor's arrays, each replaced by its file's when both
    /// files exist and that one reads - a file that does not read leaves its array as it was, as
    /// the C#'s empty `catch` does - and the channels resized to twenty.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:124-157`
    #[must_use]
    pub fn load(files: &ConfigFiles) -> Self {
        let mut config = Self::new();
        if files.buttons.exists() && files.axis.exists() {
            if let Some(buttons) = std::fs::read(&files.buttons)
                .ok()
                .and_then(|bytes| parse_buttons(&text_of(&bytes)).ok())
            {
                config.buttons = buttons;
            }
            if let Some(channels) = std::fs::read(&files.axis)
                .ok()
                .and_then(|bytes| parse_channels(&text_of(&bytes)).ok())
            {
                config.channels = channels;
            }
        }
        config
            .channels
            .resize(CHANNEL_SLOTS, JoyChannel::default());
        config
    }

    /// `saveconfig`: both arrays, each to its file.
    ///
    /// # Errors
    /// A file that cannot be written, where the C#'s `StreamWriter` throws.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:160-179`
    pub fn save(&self, files: &ConfigFiles) -> std::io::Result<()> {
        std::fs::write(&files.buttons, buttons_xml(&self.buttons))?;
        std::fs::write(&files.axis, channels_xml(&self.channels))
    }
}

/// The text of a file, a UTF-8 byte-order mark dropped: `StreamReader` detects and skips one.
fn text_of(bytes: &[u8]) -> String {
    let bytes = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]).unwrap_or(bytes);
    String::from_utf8_lossy(bytes).into_owned()
}

/// The two files: `joystickconfigbutton` and `joystickconfigaxis`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigFiles {
    /// The buttons' file.
    pub buttons: PathBuf,
    /// The channels' file.
    pub axis: PathBuf,
}

/// The default names, which `loadconfig`'s parameters default to.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:29-30, 107-108`
pub const BUTTONS_FILE: &str = "joystickbuttons.xml";
/// The channels' default name.
pub const AXIS_FILE: &str = "joystickaxis.xml";

impl ConfigFiles {
    /// The names the constructor loads for a firmware - `cs.firmware`'s name in them for
    /// `ArduPlane`, `ArduCopter2` and `ArduRover`, the default names for anything else - and where
    /// `loadconfig` looks for them: as given, relative to the working directory, when the axis
    /// file is there, and in the user data directory otherwise.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:78-96, 112-122`
    #[must_use]
    pub fn for_firmware(firmware: &str, user_data: &Path) -> Self {
        let (buttons, axis) = match firmware {
            "ArduPlane" | "ArduCopter2" | "ArduRover" => (
                format!("joystickbuttons{firmware}.xml"),
                format!("joystickaxis{firmware}.xml"),
            ),
            _ => (BUTTONS_FILE.to_owned(), AXIS_FILE.to_owned()),
        };
        if Path::new(&axis).exists() {
            Self {
                buttons: PathBuf::from(buttons),
                axis: PathBuf::from(axis),
            }
        } else {
            Self {
                buttons: user_data.join(buttons),
                axis: user_data.join(axis),
            }
        }
    }
}

/// What `ExportConfig` takes from the user data directory and `ImportConfig` puts back: a
/// `joystickbutton*.xml` or `joystickaxis*.xml` file, by name.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1314-1315, 1353`
#[must_use]
pub fn is_config_file(name: &str) -> bool {
    name.starts_with("joystickbutton") || name.starts_with("joystickaxis")
}

// ---------------------------------------------------------------------------------------------
// XmlSerializer's format.
// ---------------------------------------------------------------------------------------------

/// The declaration and root attributes `XmlSerializer.Serialize(TextWriter, ...)` writes.
const DECLARATION: &str = "<?xml version=\"1.0\" encoding=\"utf-8\"?>";
const NAMESPACES: &str = "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
                          xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\"";
/// `XmlTextWriter`'s line ending, `Environment.NewLine` where Mission Planner runs.
const NEWLINE: &str = "\r\n";

/// Text escaped as `XmlWriter.WriteString` escapes it.
fn escaped(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            _ => out.push(c),
        }
    }
    out
}

/// A `float` as `XmlConvert.ToString` writes it: the shortest text that reads back as itself.
fn float_text(value: f32) -> String {
    if value.is_nan() {
        "NaN".to_owned()
    } else if value.is_infinite() {
        if value > 0.0 { "INF" } else { "-INF" }.to_owned()
    } else {
        format!("{value}")
    }
}

/// `JoyButton[]` serialized.
#[must_use]
pub fn buttons_xml(buttons: &[JoyButton]) -> String {
    let mut out = format!("{DECLARATION}{NEWLINE}<ArrayOfJoyButton {NAMESPACES}>{NEWLINE}");
    for button in buttons {
        let _ = write!(
            out,
            "  <JoyButton>{NEWLINE}    <buttonno>{}</buttonno>{NEWLINE}    <function>{}</function>{NEWLINE}",
            button.buttonno,
            button.function.name()
        );
        // A null string is left out, as `XmlSerializer` leaves out a null reference.
        if let Some(mode) = &button.mode {
            let _ = write!(out, "    <mode>{}</mode>{NEWLINE}", escaped(mode));
        }
        for (name, value) in [
            ("p1", button.p1),
            ("p2", button.p2),
            ("p3", button.p3),
            ("p4", button.p4),
        ] {
            let _ = write!(out, "    <{name}>{}</{name}>{NEWLINE}", float_text(value));
        }
        let _ = write!(
            out,
            "    <state>{}</state>{NEWLINE}  </JoyButton>{NEWLINE}",
            button.state
        );
    }
    out.push_str("</ArrayOfJoyButton>");
    out
}

/// `JoyChannel[]` serialized.
#[must_use]
pub fn channels_xml(channels: &[JoyChannel]) -> String {
    let mut out = format!("{DECLARATION}{NEWLINE}<ArrayOfJoyChannel {NAMESPACES}>{NEWLINE}");
    for channel in channels {
        let _ = write!(
            out,
            "  <JoyChannel>{NEWLINE}    <channel>{}</channel>{NEWLINE}    <axis>{}</axis>{NEWLINE}    \
             <reverse>{}</reverse>{NEWLINE}    <expo>{}</expo>{NEWLINE}  </JoyChannel>{NEWLINE}",
            channel.channel,
            channel.axis.name(),
            channel.reverse,
            channel.expo
        );
    }
    out.push_str("</ArrayOfJoyChannel>");
    out
}

/// Why a file did not read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError(pub String);

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ParseError {}

/// The elements of an `ArrayOf<item>` document, each as its children's names and text.
fn items(text: &str, root: &str, item: &str) -> Result<Vec<Vec<(String, String)>>, ParseError> {
    let document = roxmltree::Document::parse(text).map_err(|err| ParseError(err.to_string()))?;
    let top = document.root_element();
    if top.tag_name().name() != root {
        return Err(ParseError(format!(
            "<{}> where <{root}> was expected",
            top.tag_name().name()
        )));
    }
    Ok(top
        .children()
        .filter(|node| node.is_element() && node.tag_name().name() == item)
        .map(|node| {
            node.children()
                .filter(roxmltree::Node::is_element)
                .map(|field| {
                    (
                        field.tag_name().name().to_owned(),
                        field.text().unwrap_or("").to_owned(),
                    )
                })
                .collect()
        })
        .collect())
}

/// An `int` as `XmlConvert.ToInt32` reads it.
fn int_of(field: &str, text: &str) -> Result<i32, ParseError> {
    text.trim()
        .parse()
        .map_err(|_| ParseError(format!("{field}: {text:?} is not an int")))
}

/// A `float` as `XmlConvert.ToSingle` reads it.
fn float_of(field: &str, text: &str) -> Result<f32, ParseError> {
    match text.trim() {
        "INF" => Ok(f32::INFINITY),
        "-INF" => Ok(f32::NEG_INFINITY),
        "NaN" => Ok(f32::NAN),
        other => other
            .parse()
            .map_err(|_| ParseError(format!("{field}: {text:?} is not a float"))),
    }
}

/// A `bool` as `XmlConvert.ToBoolean` reads it: `true`, `false`, `1` or `0`.
fn bool_of(field: &str, text: &str) -> Result<bool, ParseError> {
    match text.trim() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(ParseError(format!("{field}: {text:?} is not a bool"))),
    }
}

/// `JoyButton[]` deserialized: an element left out keeps the struct's zero - `buttonno` 0, not
/// the constructor's -1 - as `XmlSerializer` leaves a field it finds nothing for.
///
/// # Errors
/// Not XML, not the array, or a value that does not read as its field's type.
pub fn parse_buttons(text: &str) -> Result<Vec<JoyButton>, ParseError> {
    items(text, "ArrayOfJoyButton", "JoyButton")?
        .into_iter()
        .map(|fields| {
            let mut button = JoyButton::default();
            for (name, value) in fields {
                match name.as_str() {
                    "buttonno" => button.buttonno = int_of(&name, &value)?,
                    "function" => {
                        button.function = ButtonFunction::from_name(value.trim()).ok_or_else(
                            || ParseError(format!("function: {value:?} is not a buttonfunction")),
                        )?;
                    }
                    "mode" => button.mode = Some(value),
                    "p1" => button.p1 = float_of(&name, &value)?,
                    "p2" => button.p2 = float_of(&name, &value)?,
                    "p3" => button.p3 = float_of(&name, &value)?,
                    "p4" => button.p4 = float_of(&name, &value)?,
                    "state" => button.state = bool_of(&name, &value)?,
                    _ => {}
                }
            }
            Ok(button)
        })
        .collect()
}

/// `JoyChannel[]` deserialized.
///
/// # Errors
/// Not XML, not the array, or a value that does not read as its field's type.
pub fn parse_channels(text: &str) -> Result<Vec<JoyChannel>, ParseError> {
    items(text, "ArrayOfJoyChannel", "JoyChannel")?
        .into_iter()
        .map(|fields| {
            let mut channel = JoyChannel::default();
            for (name, value) in fields {
                match name.as_str() {
                    "channel" => channel.channel = int_of(&name, &value)?,
                    "axis" => {
                        channel.axis = JoystickAxis::from_name(value.trim()).ok_or_else(|| {
                            ParseError(format!("axis: {value:?} is not a joystickaxis"))
                        })?;
                    }
                    "reverse" => channel.reverse = bool_of(&name, &value)?,
                    "expo" => channel.expo = int_of(&name, &value)?,
                    _ => {}
                }
            }
            Ok(channel)
        })
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    /// A directory of its own, removed afterwards.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "mp-input-config-{name}-{}-{:?}",
                std::process::id(),
                std::thread::current().id()
            ));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// The enums in the C#'s order, with the C#'s spellings - what the pickers list and the
    /// files hold.
    #[test]
    fn the_enums_are_the_csharps() {
        let axes: Vec<&str> = JoystickAxis::ALL.iter().map(|axis| axis.name()).collect();
        assert_eq!(axes.first(), Some(&"None"));
        assert_eq!(axes[23..26], ["X", "Y", "Z"]);
        assert_eq!(axes.last(), Some(&"UINT16_MAX"));
        assert_eq!(axes.len(), 33);
        for axis in JoystickAxis::ALL {
            assert_eq!(JoystickAxis::from_name(axis.name()), Some(axis));
        }
        let functions: Vec<&str> = ButtonFunction::ALL.iter().map(|f| f.name()).collect();
        assert_eq!(
            functions,
            [
                "ChangeMode",
                "Do_Set_Relay",
                "Do_Repeat_Relay",
                "Do_Set_Servo",
                "Do_Repeat_Servo",
                "Arm",
                "Disarm",
                "Digicam_Control",
                "TakeOff",
                "Mount_Mode",
                "Toggle_Pan_Stab",
                "Gimbal_pnt_track",
                "Mount_Control_0",
                "Button_axis0",
                "Button_axis1",
            ]
        );
        assert_eq!(JoystickAxis::from_name("x"), None, "Enum.Parse is exact");
    }

    /// The constructor's arrays: twenty channels on nothing, 128 functions on no button.
    #[test]
    fn a_new_configuration_drives_nothing() {
        let config = JoystickConfig::new();
        assert_eq!(config.channels.len(), 20);
        assert!(config.channels.iter().all(|c| c.axis == JoystickAxis::None));
        assert_eq!(config.buttons.len(), 128);
        assert!(config.buttons.iter().all(|b| b.buttonno == -1));
    }

    /// One element of each file as `XmlSerializer` writes it.
    #[test]
    fn the_files_are_written_as_xmlserializer_writes_them() {
        let mut button = JoyButton::unassigned();
        button.function = ButtonFunction::DoSetServo;
        button.p1 = 9.0;
        button.p2 = 1500.5;
        let xml = buttons_xml(&[button]);
        assert_eq!(
            xml,
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n\
             <ArrayOfJoyButton xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
             xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\r\n\
             \x20 <JoyButton>\r\n\
             \x20   <buttonno>-1</buttonno>\r\n\
             \x20   <function>Do_Set_Servo</function>\r\n\
             \x20   <p1>9</p1>\r\n\
             \x20   <p2>1500.5</p2>\r\n\
             \x20   <p3>0</p3>\r\n\
             \x20   <p4>0</p4>\r\n\
             \x20   <state>false</state>\r\n\
             \x20 </JoyButton>\r\n\
             </ArrayOfJoyButton>"
        );
        let mut config = JoystickConfig::new();
        config.set_channel(1, JoystickAxis::X, true, 30);
        let xml = channels_xml(&config.channels[..2]);
        assert!(xml.contains(
            "  <JoyChannel>\r\n    <channel>1</channel>\r\n    <axis>X</axis>\r\n    \
             <reverse>true</reverse>\r\n    <expo>30</expo>\r\n  </JoyChannel>\r\n"
        ));
        assert!(xml.starts_with("<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n<ArrayOfJoyChannel "));
    }

    /// What is written reads back as itself, a mode with markup in it included.
    #[test]
    fn the_files_read_back_as_written() {
        let mut config = JoystickConfig::new();
        config.set_channel(3, JoystickAxis::Slider1, false, -20);
        config.set_axis(4, JoystickAxis::Uint16Max);
        let mut button = JoyButton::unassigned();
        button.buttonno = 7;
        button.function = ButtonFunction::ChangeMode;
        button.mode = Some("A<B & C>".to_owned());
        config.set_button(2, button.clone());
        assert_eq!(parse_buttons(&buttons_xml(&config.buttons)).unwrap(), config.buttons);
        assert_eq!(
            parse_channels(&channels_xml(&config.channels)).unwrap(),
            config.channels
        );
        assert_eq!(config.button(2), button);
    }

    /// A file Mission Planner wrote - a byte-order mark, LF endings, a field missing - reads as
    /// `XmlSerializer` reads it: the missing field is the struct's zero.
    #[test]
    fn a_file_mission_planner_wrote_reads() {
        let scratch = Scratch::new("mp");
        let files = ConfigFiles::for_firmware("ArduCopter2", &scratch.0);
        let mut buttons = b"\xEF\xBB\xBF".to_vec();
        buttons.extend_from_slice(
            b"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<ArrayOfJoyButton xmlns:xsi=\"x\" xmlns:xsd=\"y\">\n  <JoyButton>\n    <function>Arm</function>\n    <p1>1E+20</p1>\n  </JoyButton>\n</ArrayOfJoyButton>",
        );
        std::fs::write(&files.buttons, buttons).unwrap();
        std::fs::write(
            &files.axis,
            "<ArrayOfJoyChannel><JoyChannel><channel>1</channel><axis>Rz</axis><reverse>true</reverse><expo>50</expo></JoyChannel></ArrayOfJoyChannel>",
        )
        .unwrap();
        let config = JoystickConfig::load(&files);
        assert_eq!(config.buttons.len(), 1, "the array is the file's");
        assert_eq!(config.buttons[0].buttonno, 0, "a missing field is the struct's zero");
        assert_eq!(config.buttons[0].function, ButtonFunction::Arm);
        assert!((config.buttons[0].p1 - 1e20).abs() < 1e14);
        assert_eq!(config.channels.len(), 20, "resized to twenty");
        assert_eq!(config.channels[0].axis, JoystickAxis::Rz);
        assert!(config.channels[0].reverse);
        assert_eq!(config.channels[1], JoyChannel::default());
    }

    /// Nothing is read unless both files exist, and a file that does not read leaves its array
    /// as the constructor made it.
    #[test]
    fn a_missing_or_broken_file_leaves_the_defaults() {
        let scratch = Scratch::new("broken");
        let files = ConfigFiles::for_firmware("ArduPlane", &scratch.0);
        let mut config = JoystickConfig::new();
        config.set_axis(1, JoystickAxis::Y);
        std::fs::write(&files.axis, channels_xml(&config.channels)).unwrap();
        assert_eq!(
            JoystickConfig::load(&files),
            JoystickConfig::new(),
            "the buttons' file is missing"
        );
        std::fs::write(&files.buttons, "<ArrayOfJoyButton><JoyButton><function>Fly</function></JoyButton></ArrayOfJoyButton>").unwrap();
        let loaded = JoystickConfig::load(&files);
        assert_eq!(loaded.buttons, JoystickConfig::new().buttons, "Fly is not a function");
        assert_eq!(loaded.channel(1).axis, JoystickAxis::Y);
    }

    /// The names per firmware, in the user data directory; the save writes both.
    #[test]
    fn the_file_names_carry_the_firmware_for_plane_copter_and_rover() {
        let scratch = Scratch::new("names");
        for (firmware, axis) in [
            ("ArduPlane", "joystickaxisArduPlane.xml"),
            ("ArduCopter2", "joystickaxisArduCopter2.xml"),
            ("ArduRover", "joystickaxisArduRover.xml"),
            ("ArduSub", "joystickaxis.xml"),
            ("PX4", "joystickaxis.xml"),
        ] {
            let files = ConfigFiles::for_firmware(firmware, &scratch.0);
            assert_eq!(files.axis, scratch.0.join(axis), "{firmware}");
            assert_eq!(
                files.buttons,
                scratch.0.join(axis.replace("joystickaxis", "joystickbuttons"))
            );
        }
        let files = ConfigFiles::for_firmware("ArduRover", &scratch.0);
        let mut config = JoystickConfig::new();
        config.set_expo(2, 40);
        config.save(&files).unwrap();
        assert_eq!(JoystickConfig::load(&files), config);
        assert!(is_config_file("joystickbuttonsArduRover.xml"));
        assert!(is_config_file("joystickaxis.xml"));
        assert!(!is_config_file("config.xml"));
    }
}
