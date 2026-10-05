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

//! The flight map's Gimbal Video: `GimbalVideoControl` - a gimbal camera's video with the
//! keyboard and mouse steering the gimbal - shown full sized in the map's place with the map as a
//! mini map, as a mini video over the map, or popped out in a window of its own; its settings
//! form, `GimbalControlSettingsForm`; and its Video Stream form, `VideoStreamSelector`.
//! `// C#: GCSViews/FlightData.cs:6534-6712; Controls/GimbalVideoControl.cs:1-797;
//! Controls/GimbalVideoControl.Designer.cs; Controls/GimbalControlSettingsForm.cs:1-433;
//! Controls/VideoStreamSelector.cs:1-55; ExtLibs/Controls/KeyBindingButton.cs;
//! ExtLibs/Controls/ClickBindingButton.cs`
//!
//! The map's menu has the Gimbal Video drop-down - Full Sized, Mini, Pop Out - and the Payload
//! tab's Video Control is Pop Out. The first of them makes the control (`gimbalVideoControl`'s
//! getter), which adds three items to the control's own menu: Mini map (checked, the mini map
//! shown), Swap with map and Close. `splitContainer1_Panel2_Resize` sizes the mini video to 30%
//! of the map's panel, fitted to the picture's shape, at the bottom right, and the mini map to 30%
//! at the bottom right; it runs when those items are chosen and when the panel changes size, and
//! the video's shape is the one it had then. The flight map here has no zoom bar at its right
//! (`TRK_zoom`), so the mini video sits at the panel's edge.
//!
//! The control plays its video with `GStreamer` - the pipeline Video Stream is given, or the one
//! `CameraProtocol.GStreamerPipeline` makes for the first stream the camera reports (the
//! auto-connect timer, which asks the camera for its information once a second until it has a
//! stream) - and draws each frame zoomed into its box, with the tracking box or point over it.
//! The keys it is set to slew and zoom the gimbal while they are held and take a picture,
//! record, lock or follow, retract, centre, point down or point home when pressed; a click points
//! the camera where it lands, points it at the ground there, or tracks what is there; the mouse
//! over the picture puts a marker on the map where the pointer is looking. Each command goes to
//! the shown vehicle's gimbal manager or camera (`mp_link::gimbal_manager`, `mp_link::camera`),
//! and is not sent where the manager has not said it can - as the C#'s return `false`.
//!
//! Where this differs from the C#, and why:
//! * The windows are drawn in the main window, as this application's other forms are: the pop-out
//!   is a 600 by 400 box titled "Gimbal Control" centred over it, the settings form and the Video
//!   Stream form modal boxes over everything.
//! * The key filter (`IMessageFilter`) acts while the control's parent contains the focus; here
//!   while the control itself has it, which a click on the video gives it (`VideoBox.Focus()`).
//!   Shift, Ctrl and Alt pressed alone arrive as a change of modifiers and are held and released
//!   as the C#'s `ShiftKey`, `ControlKey` and `Menu`.
//! * `UITimer`'s writes of the preferences' fields of view into the camera
//!   (`selectedCamera.HFOV = ...`) are made on a copy of the camera where its geometry is worked
//!   out, which is the only place they are read.
//! * The "No Video" picture (`Properties.Resources.no_video`, 640 by 480) is drawn rather than
//!   carried: black, a dark red cross corner to corner and "No Video" in a white box.
//! * With no GStreamer runtime the C# offers to download one (`GStreamerUI.DownloadGStreamer`);
//!   here the status line says the control needs it and the control plays nothing, as when that
//!   question is answered No. The download is not ported in this pass: the downloader is the HUD
//!   menu's, in `fly.rs`.
//! * `Console.WriteLine`'s traces are not written.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::{Cell, RefCell};
use std::collections::BTreeSet;
use std::rc::Rc;
use std::sync::Arc;
use web_time::{Duration, Instant};

use gpui::{
    AnyElement, Bounds, Context, FocusHandle, Hsla, KeyDownEvent, KeyUpEvent, Modifiers,
    ModifiersChangedEvent, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent, PathBuilder,
    Pixels, SharedString, Window, canvas, div, point, prelude::*, px, rgb, size,
};
use mp_link::camera::Camera;
use mp_link::gimbal_manager::{GimbalManager, Quaternion};
use mp_mavlink_dialects::all::{CameraZoomType, MavMessage};
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

// --- Keys and mouse buttons, as WinForms numbers them ------------------------------------------

/// `System.Windows.Forms.Keys`: a key code in the low 16 bits and the modifiers above, as the
/// preferences hold them and Json.NET writes them.
pub mod keys {
    /// `Keys.None`.
    pub const NONE: u32 = 0;
    /// `Keys.Shift`.
    pub const SHIFT: u32 = 0x1_0000;
    /// `Keys.Control`.
    pub const CONTROL: u32 = 0x2_0000;
    /// `Keys.Alt`.
    pub const ALT: u32 = 0x4_0000;
    /// `Keys.ShiftKey`.
    pub const SHIFT_KEY: u32 = 16;
    /// `Keys.ControlKey`.
    pub const CONTROL_KEY: u32 = 17;
    /// `Keys.Menu`: Alt.
    pub const MENU: u32 = 18;
    /// `Keys.Escape`.
    pub const ESCAPE: u32 = 27;
    /// `Keys.Enter`.
    pub const ENTER: u32 = 13;

    /// `MouseButtons.None`.
    pub const MOUSE_NONE: u32 = 0;
    /// `MouseButtons.Left`.
    pub const MOUSE_LEFT: u32 = 0x10_0000;
    /// `MouseButtons.Right`.
    pub const MOUSE_RIGHT: u32 = 0x20_0000;
    /// `MouseButtons.Middle`.
    pub const MOUSE_MIDDLE: u32 = 0x40_0000;

    /// The key code of a gpui key name - what `(Keys)m.WParam` is for it - or `None` for one
    /// WinForms has no code this application maps.
    #[must_use]
    pub fn code(key: &str) -> Option<u32> {
        let key = key.to_ascii_lowercase();
        if let [c] = key.as_bytes() {
            let c = *c;
            if c.is_ascii_lowercase() {
                return Some(u32::from(c.to_ascii_uppercase()));
            }
            if c.is_ascii_digit() {
                return Some(u32::from(c));
            }
            return match c {
                b' ' => Some(32),
                b';' => Some(186),
                b'=' => Some(187),
                b',' => Some(188),
                b'-' => Some(189),
                b'.' => Some(190),
                b'/' => Some(191),
                b'`' => Some(192),
                b'[' => Some(219),
                b'\\' => Some(220),
                b']' => Some(221),
                b'\'' => Some(222),
                _ => None,
            };
        }
        if let Some(n) = key.strip_prefix('f').and_then(|n| n.parse::<u32>().ok())
            && (1..=24).contains(&n)
        {
            return Some(111 + n);
        }
        Some(match key.as_str() {
            "space" => 32,
            "enter" => ENTER,
            "escape" => ESCAPE,
            "tab" => 9,
            "backspace" => 8,
            "delete" => 46,
            "insert" => 45,
            "home" => 36,
            "end" => 35,
            "pageup" => 33,
            "pagedown" => 34,
            "left" => 37,
            "up" => 38,
            "right" => 39,
            "down" => 40,
            "shift" => SHIFT_KEY,
            "control" => CONTROL_KEY,
            "alt" => MENU,
            _ => return None,
        })
    }

    /// `Keys.ToString()` of a key code.
    #[must_use]
    pub fn name(code: u32) -> String {
        let named = match code {
            8 => "Back",
            9 => "Tab",
            13 => "Return",
            16 => "ShiftKey",
            17 => "ControlKey",
            18 => "Menu",
            27 => "Escape",
            32 => "Space",
            33 => "PageUp",
            34 => "PageDown",
            35 => "End",
            36 => "Home",
            37 => "Left",
            38 => "Up",
            39 => "Right",
            40 => "Down",
            45 => "Insert",
            46 => "Delete",
            186 => "OemSemicolon",
            187 => "Oemplus",
            188 => "Oemcomma",
            189 => "OemMinus",
            190 => "OemPeriod",
            191 => "OemQuestion",
            192 => "Oemtilde",
            219 => "OemOpenBrackets",
            220 => "OemPipe",
            221 => "OemCloseBrackets",
            222 => "OemQuotes",
            48..=57 => return format!("D{}", code - 48),
            65..=90 => return char::from_u32(code).map(String::from).unwrap_or_default(),
            112..=135 => return format!("F{}", code - 111),
            _ => return code.to_string(),
        };
        named.to_owned()
    }

    /// `Control.ModifierKeys` from gpui's modifiers.
    #[must_use]
    pub const fn modifiers(shift: bool, control: bool, alt: bool) -> u32 {
        (if shift { SHIFT } else { 0 })
            | (if control { CONTROL } else { 0 })
            | (if alt { ALT } else { 0 })
    }

    /// `GetModifiersString`: "Shift + ", "Ctrl + ", "Alt + ", in that order.
    /// `// C#: ExtLibs/Controls/KeyBindingButton.cs:229-245`
    #[must_use]
    pub fn modifiers_string(key: u32) -> String {
        let mut out = String::new();
        if key & SHIFT != 0 {
            out.push_str("Shift + ");
        }
        if key & CONTROL != 0 {
            out.push_str("Ctrl + ");
        }
        if key & ALT != 0 {
            out.push_str("Alt + ");
        }
        out
    }

    /// `KeyBindingButton.GetKeyString`: the modifiers, then the key, or "None" for no key; a
    /// modifiers-only button shows the modifiers alone, "" for none.
    /// `// C#: ExtLibs/Controls/KeyBindingButton.cs:203-222`
    #[must_use]
    pub fn key_string(key: u32, modifiers_only: bool) -> String {
        let modifiers = modifiers_string(key);
        if modifiers_only {
            return modifiers;
        }
        let code = key & !(SHIFT | CONTROL | ALT);
        if code == NONE {
            return "None".to_owned();
        }
        modifiers + &name(code)
    }

    /// `ClickBindingButton.GetClickBindingString`: the modifiers and "Left Click", or "None".
    /// `// C#: ExtLibs/Controls/ClickBindingButton.cs:415-430`
    #[must_use]
    pub fn click_string(binding: (u32, u32)) -> String {
        let button = match binding.1 {
            MOUSE_NONE => return "None".to_owned(),
            MOUSE_LEFT => "Left",
            MOUSE_RIGHT => "Right",
            MOUSE_MIDDLE => "Middle",
            other => return format!("{}{other} Click", modifiers_string(binding.0)),
        };
        format!("{}{button} Click", modifiers_string(binding.0))
    }
}

// --- GimbalControlSettings --------------------------------------------------------------------

/// `ControlType`: which control a preference is set with.
/// `// C#: Controls/GimbalControlSettingsForm.cs:280-287`
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlType {
    /// `KeyBindingButton`.
    KeyBinding,
    /// `ClickBindingButton`.
    ClickBinding,
    /// `KeyBindingButton` with `ModifiersOnly`.
    ModifierBinding,
    /// `NumericUpDown`: minimum, maximum, increment, decimal places.
    Decimal(f64, f64, f64, usize),
    /// `CheckBox`.
    Checkbox,
}

/// One preference: its property's name, its `[Preferences]` label and control.
#[derive(Debug, Clone, Copy)]
pub struct Preference {
    /// The property, which is the key Json.NET writes.
    pub name: &'static str,
    /// `LabelText`.
    pub label: &'static str,
    /// `ControlType`, with the up-down's numbers.
    pub control: ControlType,
}

/// Every `[Preferences]` property, in declaration order - the form's rows and the JSON's keys.
/// `// C#: Controls/GimbalControlSettingsForm.cs:310-381`
pub const PREFERENCES: [Preference; 30] = {
    use ControlType::{Checkbox, ClickBinding, Decimal, KeyBinding, ModifierBinding};
    const fn p(name: &'static str, label: &'static str, control: ControlType) -> Preference {
        Preference {
            name,
            label,
            control,
        }
    }
    [
        p("SlewLeft", "Slew Left", KeyBinding),
        p("SlewRight", "Slew Right", KeyBinding),
        p("SlewUp", "Slew Up", KeyBinding),
        p("SlewDown", "Slew Down", KeyBinding),
        p("ZoomIn", "Zoom In", KeyBinding),
        p("ZoomOut", "Zoom Out", KeyBinding),
        p("SlewFastModifier", "Slew Fast Modifier", ModifierBinding),
        p("SlewSlowModifier", "Slew Slow Modifier", ModifierBinding),
        p("TakePicture", "Take Picture", KeyBinding),
        p("ToggleRecording", "Toggle Recording", KeyBinding),
        p("StartRecording", "Start Recording", KeyBinding),
        p("StopRecording", "Stop Recording", KeyBinding),
        p("ToggleLockFollow", "Toggle Lock/Follow", KeyBinding),
        p("SetLock", "Set Lock", KeyBinding),
        p("SetFollow", "Set Follow", KeyBinding),
        p("Retract", "Retract", KeyBinding),
        p("Neutral", "Neutral", KeyBinding),
        p("PointDown", "Point Down", KeyBinding),
        p("Home", "Home", KeyBinding),
        p("MoveCameraToMouseLocation", "Click Pan/Tilt", ClickBinding),
        p(
            "MoveCameraPOIToMouseLocation",
            "Click Point-of-Interest",
            ClickBinding,
        ),
        p("TrackObjectUnderMouse", "Click Track Object", ClickBinding),
        p(
            "SlewSpeedSlow",
            "Slew Speed Slow (deg/s)",
            Decimal(0.1, 360.0, 1.0, 1),
        ),
        p(
            "SlewSpeedNormal",
            "Slew Speed Normal (deg/s)",
            Decimal(0.1, 360.0, 1.0, 1),
        ),
        p(
            "SlewSpeedFast",
            "Slew Speed Fast (deg/s)",
            Decimal(0.1, 360.0, 1.0, 1),
        ),
        p(
            "ZoomSpeed",
            "Zoom Speed (unitless)",
            Decimal(0.01, 1.0, 0.1, 2),
        ),
        p(
            "CameraHFOV",
            "Camera Horizontal FOV (deg)",
            Decimal(0.01, 180.0, 1.0, 2),
        ),
        p(
            "CameraVFOV",
            "Camera Vertical FOV (deg)",
            Decimal(0.01, 180.0, 1.0, 2),
        ),
        p(
            "UseFOVReportedByCamera",
            "Use FOV Reported by Camera",
            Checkbox,
        ),
        p("DefaultLockedMode", "Default Locked Mode", Checkbox),
    ]
};

/// A preference's value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value {
    /// `Keys`.
    Key(u32),
    /// `(Keys, MouseButtons)`.
    Click(u32, u32),
    /// `decimal`.
    Decimal(f64),
    /// `bool`.
    Bool(bool),
}

/// `GimbalControlSettings`: the values, in [`PREFERENCES`]' order.
#[derive(Debug, Clone, PartialEq)]
pub struct GimbalControlSettings {
    /// One per preference.
    pub values: [Value; 30],
}

impl Default for GimbalControlSettings {
    /// The constructor's defaults. `// C#: Controls/GimbalControlSettingsForm.cs:383-423`
    fn default() -> Self {
        use keys::{ALT, CONTROL, MOUSE_LEFT, NONE, SHIFT};
        let k = |c: char| Value::Key(u32::from(c));
        Self {
            values: [
                k('A'),
                k('D'),
                k('W'),
                k('S'),
                k('E'),
                k('Q'),
                Value::Key(SHIFT),
                Value::Key(CONTROL),
                Value::Key(ALT | u32::from('F')),
                Value::Key(ALT | u32::from('R')),
                Value::Key(NONE),
                Value::Key(NONE),
                k('L'),
                Value::Key(NONE),
                Value::Key(NONE),
                Value::Key(NONE),
                k('N'),
                Value::Key(NONE),
                k('H'),
                Value::Click(NONE, MOUSE_LEFT),
                Value::Click(CONTROL, MOUSE_LEFT),
                Value::Click(ALT, MOUSE_LEFT),
                Value::Decimal(1.0),
                Value::Decimal(5.0),
                Value::Decimal(25.0),
                Value::Decimal(1.0),
                Value::Decimal(40.0),
                Value::Decimal(30.0),
                Value::Bool(true),
                Value::Bool(false),
            ],
        }
    }
}

/// The settings key the preferences are kept under.
pub const PREFERENCES_KEY: &str = "GimbalControlPreferences";
/// The settings key of the last stream auto-connected to.
pub const STREAM_KEY: &str = "gimbal_video_stream";

impl GimbalControlSettings {
    fn index(name: &str) -> usize {
        PREFERENCES.iter().position(|p| p.name == name).unwrap_or(0)
    }

    /// The value of the preference named.
    #[must_use]
    pub fn get(&self, name: &str) -> Value {
        self.values
            .get(Self::index(name))
            .copied()
            .unwrap_or(Value::Bool(false))
    }

    /// A `Keys` preference.
    #[must_use]
    pub fn key(&self, name: &str) -> u32 {
        match self.get(name) {
            Value::Key(k) => k,
            _ => 0,
        }
    }

    /// A click preference.
    #[must_use]
    pub fn click(&self, name: &str) -> (u32, u32) {
        match self.get(name) {
            Value::Click(k, b) => (k, b),
            _ => (0, 0),
        }
    }

    /// A `decimal` preference.
    #[must_use]
    pub fn decimal(&self, name: &str) -> f64 {
        match self.get(name) {
            Value::Decimal(d) => d,
            _ => 0.0,
        }
    }

    /// A `bool` preference.
    #[must_use]
    pub fn flag(&self, name: &str) -> bool {
        matches!(self.get(name), Value::Bool(true))
    }

    /// `JsonConvert.SerializeObject(preferences)`: the properties in order, `Keys` as numbers,
    /// a click as its tuple's `Item1` and `Item2`.
    #[must_use]
    pub fn to_json(&self) -> String {
        let mut map = serde_json::Map::new();
        for (preference, value) in PREFERENCES.iter().zip(self.values) {
            let json = match value {
                Value::Key(k) => serde_json::json!(k),
                Value::Click(k, b) => serde_json::json!({ "Item1": k, "Item2": b }),
                Value::Decimal(d) => serde_json::json!(d),
                Value::Bool(b) => serde_json::json!(b),
            };
            map.insert(preference.name.to_owned(), json);
        }
        serde_json::Value::Object(map).to_string()
    }

    /// `loadPreferences`' reading: the defaults with whatever the JSON sets; text that is not a
    /// JSON object is the C#'s "Invalid GimbalControlPreferences, reverting to default".
    /// `// C#: Controls/GimbalVideoControl.cs:170-182`
    #[must_use]
    pub fn from_json(text: &str) -> Self {
        let mut settings = Self::default();
        if text.is_empty() {
            return settings;
        }
        let Ok(serde_json::Value::Object(map)) = serde_json::from_str::<serde_json::Value>(text)
        else {
            return settings;
        };
        #[allow(clippy::cast_possible_truncation)] // Keys are 32 bits
        let as_u32 = |v: &serde_json::Value| v.as_u64().map(|n| n as u32);
        let mut out = settings.clone();
        for (slot, preference) in out.values.iter_mut().zip(PREFERENCES) {
            let Some(json) = map.get(preference.name) else {
                continue;
            };
            let parsed = match *slot {
                Value::Key(_) => as_u32(json).map(Value::Key),
                Value::Click(..) => json
                    .get("Item1")
                    .and_then(as_u32)
                    .zip(json.get("Item2").and_then(as_u32))
                    .map(|(k, b)| Value::Click(k, b)),
                Value::Decimal(_) => json.as_f64().map(Value::Decimal),
                Value::Bool(_) => json.as_bool().map(Value::Bool),
            };
            match parsed {
                Some(value) => *slot = value,
                // A value of the wrong type fails the whole deserialisation in Json.NET.
                None => return settings,
            }
        }
        settings = out;
        settings
    }

    /// `boundPressKeys`: the keys acted on when pressed. `// C#: GimbalVideoControl.cs:185-196`
    #[must_use]
    pub fn press_keys(&self) -> BTreeSet<u32> {
        [
            "TakePicture",
            "ToggleRecording",
            "StartRecording",
            "StopRecording",
            "ToggleLockFollow",
            "SetLock",
            "SetFollow",
            "Retract",
            "Neutral",
            "PointDown",
            "Home",
        ]
        .into_iter()
        .map(|name| self.key(name))
        .collect()
    }

    /// `boundHoldKeys`: the keys acted on while held, and the slew modifiers' keys through
    /// `mod2key`. A modifier that is not exactly one of Shift, Ctrl and Alt throws in the C#'s
    /// dictionary lookup; here it adds nothing. `// C#: GimbalVideoControl.cs:199-208`
    #[must_use]
    pub fn hold_keys(&self) -> BTreeSet<u32> {
        let mut out: BTreeSet<u32> = [
            "SlewLeft",
            "SlewRight",
            "SlewUp",
            "SlewDown",
            "ZoomIn",
            "ZoomOut",
        ]
        .into_iter()
        .map(|name| self.key(name))
        .collect();
        for name in ["SlewFastModifier", "SlewSlowModifier"] {
            if let Some(key) = mod2key(self.key(name)) {
                out.insert(key);
            }
        }
        out
    }

    /// `GetClashes`: the labels of the other preferences of the same type holding `value`.
    /// `// C#: Controls/GimbalControlSettingsForm.cs:236-277`
    #[must_use]
    pub fn clashes(&self, index: usize, value: Value) -> Vec<&'static str> {
        PREFERENCES
            .iter()
            .zip(self.values)
            .enumerate()
            .filter(|(other, (_, held))| {
                *other != index
                    && std::mem::discriminant(held) == std::mem::discriminant(&value)
                    && *held == value
            })
            .map(|(_, (preference, _))| preference.label)
            .collect()
    }

    /// A value as its row's control shows it.
    #[must_use]
    pub fn text(&self, index: usize) -> String {
        let (Some(preference), Some(value)) = (PREFERENCES.get(index), self.values.get(index))
        else {
            return String::new();
        };
        match (preference.control, *value) {
            (ControlType::ModifierBinding, Value::Key(k)) => keys::key_string(k, true),
            (_, Value::Key(k)) => keys::key_string(k, false),
            (_, Value::Click(k, b)) => keys::click_string((k, b)),
            (ControlType::Decimal(_, _, _, places), Value::Decimal(d)) => {
                format!("{d:.places$}")
            }
            (_, Value::Decimal(d)) => d.to_string(),
            (_, Value::Bool(b)) => b.to_string(),
        }
    }
}

/// `mod2key`: `Keys.Shift` to `ShiftKey`, `Control` to `ControlKey`, `Alt` to `Menu`.
/// `// C#: Controls/GimbalVideoControl.cs:40-45`
#[must_use]
pub const fn mod2key(modifier: u32) -> Option<u32> {
    match modifier {
        keys::SHIFT => Some(keys::SHIFT_KEY),
        keys::CONTROL => Some(keys::CONTROL_KEY),
        keys::ALT => Some(keys::MENU),
        _ => None,
    }
}

// --- What the control asks of the gimbal and the camera ---------------------------------------

/// One call the control makes on `selectedGimbalManager` or `selectedCamera`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Call {
    /// `SetRatesCommandAsync(pitch, yaw, yaw_lock, id)`.
    SetRates(f32, f32, bool),
    /// `SetZoomAsync(zoom, ZOOM_TYPE_CONTINUOUS)`.
    Zoom(f32),
    /// `TakeSinglePictureAsync()`.
    TakePicture,
    /// `StartRecordingAsync()`.
    StartRecording,
    /// `StopRecordingAsync()`.
    StopRecording,
    /// `RetractAsync()`.
    Retract,
    /// `NeutralAsync()`.
    Neutral,
    /// `SetAnglesCommandAsync(-90, 0, false, id)`: Point Down.
    PointDown,
    /// `SetROILocationAsync` at the home location: Home.
    Home,
    /// Click Pan/Tilt at an image point.
    PanTilt(f64, f64),
    /// Click Point-of-Interest at an image point.
    PointOfInterest(f64, f64),
    /// Click Track Object: the point, and where the drag began if there was one.
    Track(f64, f64, Option<(f64, f64)>),
}

impl Call {
    /// The C#'s member, for the fact.
    #[must_use]
    pub const fn member(self) -> &'static str {
        match self {
            Self::SetRates(..) => "SetRatesCommandAsync",
            Self::Zoom(_) => "SetZoomAsync",
            Self::TakePicture => "TakeSinglePictureAsync",
            Self::StartRecording => "StartRecordingAsync",
            Self::StopRecording => "StopRecordingAsync",
            Self::Retract => "RetractAsync",
            Self::Neutral => "NeutralAsync",
            Self::PointDown => "SetAnglesCommandAsync",
            Self::Home | Self::PointOfInterest(..) => "SetROILocationAsync",
            Self::PanTilt(..) => "SetAttitudeAsync",
            Self::Track(..) => "SetTracking",
        }
    }
}

/// What the vehicle side offers a call: the shown vehicle, its gimbal manager and camera, its
/// yaw in degrees and its home.
#[derive(Debug, Clone, Default)]
pub struct Selected {
    /// The vehicle the commands go to (`sysidcurrent`, `compidcurrent`).
    pub target: Option<VehicleId>,
    /// `selectedGimbalManager`.
    pub manager: Option<GimbalManager>,
    /// `selectedCamera`, with the preferences' fields of view in it.
    pub camera: Option<Camera>,
    /// `cs.yaw`, degrees.
    pub yaw: f64,
    /// `cs.HomeLocation`: latitude, longitude, altitude.
    pub home: (f64, f64, f64),
}

/// `MAV_FRAME.GLOBAL`.
const FRAME_GLOBAL: u8 = 0;

/// The messages a call puts on the wire, each with whether its answer is waited for (`false`
/// for `RequestTrackingMessageInterval`'s), or why nothing is sent.
///
/// # Errors
///
/// The reason nothing goes out: no manager or camera (`?.` on null), or the manager or camera
/// has not said it can (`Task.FromResult(false)`), or the geometry had no answer.
pub fn messages(
    call: Call,
    selected: &mut Selected,
    selected_gimbal_id: u8,
    yaw_lock: bool,
) -> Result<Vec<(MavMessage, bool)>, &'static str> {
    const NO_MANAGER: &str = "no gimbal manager";
    const NO_CAMERA: &str = "no camera";
    const UNABLE: &str = "not able";
    let target = selected.target;
    let manager = |selected: &Selected| selected.manager.clone().zip(target).ok_or(NO_MANAGER);
    let once = |message: Option<MavMessage>| message.map(|m| vec![(m, true)]).ok_or(UNABLE);
    match call {
        Call::SetRates(pitch, yaw, lock) => {
            let (manager, target) = manager(selected)?;
            once(manager.set_rates(target, pitch, yaw, lock, selected_gimbal_id))
        }
        Call::Retract => {
            let (manager, target) = manager(selected)?;
            once(manager.retract(target, 0))
        }
        Call::Neutral => {
            let (manager, target) = manager(selected)?;
            once(manager.neutral(target, 0))
        }
        Call::PointDown => {
            let (manager, target) = manager(selected)?;
            once(manager.set_angles(target, -90.0, 0.0, false, selected_gimbal_id))
        }
        Call::Home => {
            let (manager, target) = manager(selected)?;
            let (lat, lng, alt) = selected.home;
            once(manager.set_roi_location(target, lat, lng, alt, 0, FRAME_GLOBAL))
        }
        Call::PanTilt(x, y) => {
            let (manager, target) = manager(selected)?;
            let attitude = manager
                .attitude(selected_gimbal_id, selected.yaw)
                .ok_or("no attitude")?;
            let q = selected
                .camera
                .as_ref()
                .map(|camera| camera.image_point_rotation(x, y))
                .ok_or(NO_CAMERA)?;
            let q: Quaternion = attitude * q;
            once(manager.set_attitude(target, q, yaw_lock, selected_gimbal_id, selected.yaw))
        }
        Call::PointOfInterest(x, y) => {
            let camera = selected.camera.as_ref().ok_or(NO_CAMERA)?;
            let (lat, lng, alt) = image_point_location(camera, x, y).ok_or("no location")?;
            let (manager, target) = manager(selected)?;
            once(manager.set_roi_location(target, lat, lng, alt, 0, FRAME_GLOBAL))
        }
        Call::Zoom(zoom) => {
            let camera = selected.camera.as_ref().ok_or(NO_CAMERA)?;
            Ok(vec![(
                camera.set_zoom(zoom, CameraZoomType::ZOOM_TYPE_CONTINUOUS),
                true,
            )])
        }
        Call::TakePicture => {
            let camera = selected.camera.as_mut().ok_or(NO_CAMERA)?;
            Ok(vec![(camera.take_single_picture(0), true)])
        }
        Call::StartRecording => {
            let camera = selected.camera.as_ref().ok_or(NO_CAMERA)?;
            Ok(vec![(camera.start_recording(0), true)])
        }
        Call::StopRecording => {
            let camera = selected.camera.as_ref().ok_or(NO_CAMERA)?;
            Ok(vec![(camera.stop_recording(0), true)])
        }
        Call::Track(x, y, start) => {
            let camera = selected.camera.as_ref().ok_or(NO_CAMERA)?;
            // `RequestTrackingMessageInterval(5)` first, not waited on.
            let mut out: Vec<(MavMessage, bool)> = camera
                .tracking_message_interval(5)
                .map(|m| (m, false))
                .into_iter()
                .collect();
            #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
            let command = match start {
                Some((sx, sy)) => {
                    camera.set_tracking_rectangle(sx as f32, sy as f32, x as f32, y as f32)
                }
                None => camera.set_tracking_point(x as f32, y as f32),
            };
            match command {
                Some(m) => out.push((m, true)),
                None if out.is_empty() => return Err(UNABLE),
                None => {}
            }
            Ok(out)
        }
    }
}

/// `CameraProtocol.CalculateImagePointLocation(x, y)`: where on the ground the image point is
/// looking, from `CAMERA_FOV_STATUS`'s camera and image-centre positions - the centre itself for
/// (0, 0), and `None` where the image centre is above the camera. The fields of view are the
/// camera's ([`Camera::hfov`]); positions are `PointLatLngAlt`'s spherical `newpos`,
/// `GetBearing` and `GetDistance`. `// C#: ExtLibs/ArduPilot/Mavlink/CameraProtocol.cs:565-595`
#[must_use]
#[allow(clippy::float_cmp)] // the C#'s `== 0`
pub fn image_point_location(camera: &Camera, x: f64, y: f64) -> Option<(f64, f64, f64)> {
    use mp_georef::projection::Point;
    let fov = camera.fov_status;
    let field = |f: fn(&mp_mavlink_dialects::all::CameraFovStatus) -> i32| {
        fov.as_ref().map_or(0.0, |s| f64::from(f(s)))
    };
    let image = Point {
        lat: field(|s| s.lat_image) * 1e-7,
        lng: field(|s| s.lon_image) * 1e-7,
        alt: field(|s| s.alt_image) * 1e-3,
    };
    if x == 0.0 && y == 0.0 {
        return Some((image.lat, image.lng, image.alt));
    }
    let cam = Point {
        lat: field(|s| s.lat_camera) * 1e-7,
        lng: field(|s| s.lon_camera) * 1e-7,
        alt: field(|s| s.alt_camera) * 1e-3,
    };
    let height = cam.alt - image.alt;
    if height < 0.0 {
        return None;
    }
    let dist = cam.get_distance(image);
    let mut down_elevation = height.atan2(dist);
    down_elevation += y / 2.0 * f64::from(camera.vfov()) * std::f64::consts::PI / 180.0;
    down_elevation = down_elevation.max(0.0001);
    let out_distance = (height * down_elevation.cos() / down_elevation.sin()).min(1e5);
    let side_angle = x / 2.0 * f64::from(camera.hfov()) * std::f64::consts::PI / 180.0;
    let side_distance = out_distance.hypot(height) * side_angle.tan();
    let bearing = cam.get_bearing(image);
    let pos = cam
        .newpos(bearing, out_distance)
        .newpos(bearing + 90.0, side_distance);
    Some((pos.lat, pos.lng, image.alt))
}

/// `getMousePosition(x, y)`: a point in the box as -1 to 1 across the picture zoomed into it,
/// with the C#'s integer arithmetic - `x *= VideoBox.Width / imageWidth` divides whole numbers,
/// so it multiplies by 1 unless the box is twice the picture's width - and the clamp to the
/// picture's own width, not the box's. `None` for a box or picture with no size.
/// `// C#: Controls/GimbalVideoControl.cs:670-694`
#[must_use]
pub fn mouse_position(
    x: i32,
    y: i32,
    box_size: (i32, i32),
    image: (i32, i32),
) -> Option<(f64, f64)> {
    let (bw, bh) = box_size;
    let (iw, ih) = image;
    if iw <= 0 || ih <= 0 {
        return None;
    }
    let image_width = bw.min(bh * iw / ih);
    let image_height = bh.min(bw * ih / iw);
    if image_width <= 0 || image_height <= 0 {
        return None;
    }
    let (mut x, mut y) = (x, y);
    if image_width < bw {
        x -= (bw - image_width) / 2;
        x *= bw / image_width;
        x = x.min(image_width).max(0);
    }
    if image_height < bh {
        y -= (bh - image_height) / 2;
        y *= bh / image_height;
        y = y.min(image_height).max(0);
    }
    Some((
        2.0 * f64::from(x) / f64::from(image_width) - 1.0,
        2.0 * f64::from(y) / f64::from(image_height) - 1.0,
    ))
}

// --- The control's keys, rates and presses ----------------------------------------------------

/// The keyboard half of `GimbalVideoControl`: what is held, the rates last sent, the lock.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Steering {
    /// `heldKeys`.
    pub held: BTreeSet<u32>,
    /// `previousPitchRate`.
    pub pitch_rate: f32,
    /// `previousYawRate`.
    pub yaw_rate: f32,
    /// `previousZoomRate`.
    pub zoom_rate: f32,
    /// `yaw_lock`, which `yawLockToolStripMenuItem` shows.
    pub yaw_lock: bool,
    /// `isRecording`, tracked by hand as the C# does ("ArduPilot hard-codes this to 0").
    pub recording: bool,
}

impl Steering {
    /// `HandleKeyDown(key)` with `Control.ModifierKeys`: a bound held key held and the rates
    /// worked out, else a bound press acted on; whether it was taken.
    /// `// C#: Controls/GimbalVideoControl.cs:370-384`
    pub fn key_down(
        &mut self,
        key: u32,
        modifiers: u32,
        preferences: &GimbalControlSettings,
        calls: &mut Vec<Call>,
    ) -> bool {
        if preferences.hold_keys().contains(&key) {
            self.held.insert(key);
            self.held_keys(modifiers, preferences, calls);
            return true;
        }
        if preferences.press_keys().contains(&(key | modifiers)) {
            self.press(key | modifiers, preferences, calls);
            return true;
        }
        false
    }

    /// `HandleKeyUp(key)`: let go of, whether bound or not, and the rates worked out again for a
    /// bound one. `// C#: Controls/GimbalVideoControl.cs:386-395`
    pub fn key_up(
        &mut self,
        key: u32,
        modifiers: u32,
        preferences: &GimbalControlSettings,
        calls: &mut Vec<Call>,
    ) -> bool {
        self.held.remove(&key);
        let bound = preferences.hold_keys().contains(&key);
        if bound {
            self.held_keys(modifiers, preferences, calls);
        }
        bound
    }

    /// `PreFilterMessage` without the focus: every key let go and the rates worked out.
    /// `// C#: Controls/GimbalVideoControl.cs:338-346`
    pub fn focus_lost(
        &mut self,
        modifiers: u32,
        preferences: &GimbalControlSettings,
        calls: &mut Vec<Call>,
    ) {
        if !self.held.is_empty() {
            self.held.clear();
            self.held_keys(modifiers, preferences, calls);
        }
    }

    /// `HandleHeldKeys`: pitch and yaw from the slew keys at the speed the modifiers choose - the
    /// fast or slow one only when the modifiers are exactly it - sent when they change; the zoom
    /// likewise. `// C#: Controls/GimbalVideoControl.cs:397-457`
    #[allow(clippy::cast_possible_truncation, clippy::float_cmp)] // the C#'s `(float)` and `!=`
    pub fn held_keys(
        &mut self,
        modifiers: u32,
        preferences: &GimbalControlSettings,
        calls: &mut Vec<Call>,
    ) {
        let held = |name: &str| self.held.contains(&preferences.key(name));
        let mut pitch: f32 = 0.0;
        let mut yaw: f32 = 0.0;
        if held("SlewDown") {
            pitch -= 1.0;
        }
        if held("SlewUp") {
            pitch += 1.0;
        }
        if held("SlewLeft") {
            yaw -= 1.0;
        }
        if held("SlewRight") {
            yaw += 1.0;
        }
        let speed = if modifiers == preferences.key("SlewFastModifier") {
            preferences.decimal("SlewSpeedFast")
        } else if modifiers == preferences.key("SlewSlowModifier") {
            preferences.decimal("SlewSpeedSlow")
        } else {
            preferences.decimal("SlewSpeedNormal")
        } as f32;
        pitch *= speed;
        yaw *= speed;
        if pitch != self.pitch_rate || yaw != self.yaw_rate {
            self.pitch_rate = pitch;
            self.yaw_rate = yaw;
            calls.push(Call::SetRates(pitch, yaw, self.yaw_lock));
        }
        let mut zoom: f32 = 0.0;
        if held("ZoomIn") {
            zoom += 1.0;
        }
        if held("ZoomOut") {
            zoom -= 1.0;
        }
        zoom *= preferences.decimal("ZoomSpeed") as f32;
        if zoom != self.zoom_rate {
            self.zoom_rate = zoom;
            calls.push(Call::Zoom(zoom));
        }
    }

    /// `HandleKeyPress(key)`: each binding the key matches, in the C#'s order - so a key bound
    /// twice does both. `// C#: Controls/GimbalVideoControl.cs:459-505`
    pub fn press(&mut self, key: u32, preferences: &GimbalControlSettings, calls: &mut Vec<Call>) {
        let is = |name: &str| key == preferences.key(name);
        if is("TakePicture") {
            calls.push(Call::TakePicture);
        }
        if is("ToggleRecording") {
            self.set_recording(!self.recording, calls);
        }
        if is("StartRecording") {
            self.set_recording(true, calls);
        }
        if is("StopRecording") {
            self.set_recording(false, calls);
        }
        if is("ToggleLockFollow") {
            self.set_yaw_lock(!self.yaw_lock, calls);
        }
        if is("SetLock") {
            self.set_yaw_lock(true, calls);
        }
        if is("SetFollow") {
            self.set_yaw_lock(false, calls);
        }
        if is("Retract") {
            calls.push(Call::Retract);
        }
        if is("Neutral") {
            calls.push(Call::Neutral);
        }
        if is("PointDown") {
            calls.push(Call::PointDown);
        }
        if is("Home") {
            calls.push(Call::Home);
        }
    }

    /// `SetRecording(start)`. `// C#: Controls/GimbalVideoControl.cs:513-526`
    pub fn set_recording(&mut self, start: bool, calls: &mut Vec<Call>) {
        self.recording = start;
        calls.push(if start {
            Call::StartRecording
        } else {
            Call::StopRecording
        });
    }

    /// `SetYawLock(locked)`: the lock, and the last rates sent again with it.
    /// `// C#: Controls/GimbalVideoControl.cs:528-535`
    pub fn set_yaw_lock(&mut self, locked: bool, calls: &mut Vec<Call>) {
        self.yaw_lock = locked;
        calls.push(Call::SetRates(self.pitch_rate, self.yaw_rate, locked));
    }
}

// --- The control ------------------------------------------------------------------------------

/// Where the control is: `splitContainer1.Panel2` or a form of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parent {
    /// In the map's panel: `Dock.Fill` (full sized) or `Dock.None` (the mini video).
    Panel {
        /// `Dock == DockStyle.Fill`.
        fill: bool,
    },
    /// In the pop-out form.
    Form,
}

/// The control's own menu, `VideoBoxContextMenu`, and the three items `FlightData` adds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuItem {
    /// `videoStreamToolStripMenuItem`, "Video Stream".
    VideoStream,
    /// `retractToolStripMenuItem`.
    Retract,
    /// `neutralToolStripMenuItem`.
    Neutral,
    /// `pointDownToolStripMenuItem`.
    PointDown,
    /// `pointHomeToolStripMenuItem`.
    PointHome,
    /// `yawLockToolStripMenuItem`, checked while locked.
    YawLock,
    /// `takePictureToolStripMenuItem`.
    TakePicture,
    /// `startRecordingToolStripMenuItem`.
    StartRecording,
    /// `stopRecordingToolStripMenuItem`.
    StopRecording,
    /// `settingsToolStripMenuItem`.
    Settings,
    /// `gimbalVideoShowMiniMap`, "Mini map", checked on click.
    MiniMap,
    /// `gimbalVideoSwapPosition`, "Swap with map".
    Swap,
    /// `gimbalVideoClose`, "Close".
    Close,
}

impl MenuItem {
    /// The menu in the Designer's order, a separator before the entries marked, then the three
    /// `FlightData` adds. `// C#: Controls/GimbalVideoControl.Designer.cs:61-74;
    /// GCSViews/FlightData.cs:6581-6583`
    pub const ALL: [(Self, bool); 13] = [
        (Self::VideoStream, false),
        (Self::Retract, true),
        (Self::Neutral, false),
        (Self::PointDown, false),
        (Self::PointHome, false),
        (Self::YawLock, false),
        (Self::TakePicture, true),
        (Self::StartRecording, false),
        (Self::StopRecording, false),
        (Self::Settings, true),
        (Self::MiniMap, false),
        (Self::Swap, false),
        (Self::Close, false),
    ];

    /// Its text. `// C#: Controls/GimbalVideoControl.Designer.cs:76-133; GCSViews/FlightData.cs:6534-6536`
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::VideoStream => "Video Stream",
            Self::Retract => "Retract",
            Self::Neutral => "Neutral",
            Self::PointDown => "Point Down",
            Self::PointHome => "Point Home",
            Self::YawLock => "Yaw Lock",
            Self::TakePicture => "Take Picture",
            Self::StartRecording => "Start Recording",
            Self::StopRecording => "Stop Recording",
            Self::Settings => "Settings",
            Self::MiniMap => "Mini map",
            Self::Swap => "Swap with map",
            Self::Close => "Close",
        }
    }

    /// The id a script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::VideoStream => "fly-gimbal-menu-videostream",
            Self::Retract => "fly-gimbal-menu-retract",
            Self::Neutral => "fly-gimbal-menu-neutral",
            Self::PointDown => "fly-gimbal-menu-pointdown",
            Self::PointHome => "fly-gimbal-menu-pointhome",
            Self::YawLock => "fly-gimbal-menu-yawlock",
            Self::TakePicture => "fly-gimbal-menu-takepicture",
            Self::StartRecording => "fly-gimbal-menu-startrecording",
            Self::StopRecording => "fly-gimbal-menu-stoprecording",
            Self::Settings => "fly-gimbal-menu-settings",
            Self::MiniMap => "fly-gimbal-menu-minimap",
            Self::Swap => "fly-gimbal-menu-swap",
            Self::Close => "fly-gimbal-menu-close",
        }
    }
}

/// `VideoStreamSelector`, while it is open.
#[derive(Debug)]
pub struct StreamSelector {
    /// `cmb_detectedstreams`' choice.
    pub selected: Option<usize>,
    /// `txt_gstreamraw`.
    pub pipeline: TextField,
}

/// A binding button listening for its key or click.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Listening {
    /// The row.
    pub row: usize,
    /// The modifiers shown so far, as the button's text shows them while they are held.
    pub shown: Option<u32>,
}

/// `GimbalControlSettingsForm`, while it is open.
#[derive(Debug)]
pub struct SettingsForm {
    /// `preferences`: the copy being edited.
    pub preferences: GimbalControlSettings,
    /// The binding button listening, if one is.
    pub listening: Option<Listening>,
    /// The number being typed into an up-down, and its row.
    pub typing: Option<(usize, TextField)>,
    /// "Key Binding Clash": the row, the binding asked for, and the message.
    pub clash: Option<(usize, Value, String)>,
}

impl SettingsForm {
    /// `new GimbalControlSettingsForm(preferences)`: a copy.
    #[must_use]
    pub fn new(preferences: &GimbalControlSettings) -> Self {
        Self {
            preferences: preferences.clone(),
            listening: None,
            typing: None,
            clash: None,
        }
    }

    /// `HandleKeyBindingChanged` / `HandleClickBindingChanged`: None always taken, otherwise
    /// taken unless it clashes, when "Key Binding Clash" asks first.
    /// `// C#: Controls/GimbalControlSettingsForm.cs:186-227, 262-274`
    pub fn assign(&mut self, row: usize, value: Value) {
        let none = matches!(value, Value::Key(0) | Value::Click(0, 0));
        let clashes = self.preferences.clashes(row, value);
        if none || clashes.is_empty() {
            if let Some(slot) = self.preferences.values.get_mut(row) {
                *slot = value;
            }
            return;
        }
        let mut message = String::from("This binding is already used by the following:\n\n");
        for label in clashes {
            message.push_str(&format!("- {label}\n"));
        }
        message.push_str("\nAre you sure you want to use this binding?\n");
        self.clash = Some((row, value, message));
    }

    /// The clash question answered: Yes takes the binding, No leaves the one there was.
    pub fn answer_clash(&mut self, yes: bool) {
        if let Some((row, value, _)) = self.clash.take()
            && yes
            && let Some(slot) = self.preferences.values.get_mut(row)
        {
            *slot = value;
        }
    }

    /// A key down while a button listens: Escape cancels; a modifier alone only shows; a
    /// modifiers-only button takes the modifiers on Enter; any other key is the binding with its
    /// modifiers. A click button takes the modifiers as they are pressed.
    /// `// C#: ExtLibs/Controls/KeyBindingButton.cs:126-163; ClickBindingButton.cs:361-376`
    pub fn listening_key(&mut self, key: u32, modifiers: u32) {
        let Some(listening) = self.listening else {
            return;
        };
        let Some(preference) = PREFERENCES.get(listening.row) else {
            return;
        };
        if key == keys::ESCAPE {
            self.listening = None;
            return;
        }
        if preference.control == ControlType::ClickBinding {
            self.listening = Some(Listening {
                shown: Some(modifiers),
                ..listening
            });
            return;
        }
        self.listening = Some(Listening {
            shown: Some(modifiers),
            ..listening
        });
        if matches!(key, keys::SHIFT_KEY | keys::CONTROL_KEY | keys::MENU) {
            return;
        }
        if preference.control == ControlType::ModifierBinding {
            if key == keys::ENTER {
                self.listening = None;
                self.assign(listening.row, Value::Key(modifiers));
            }
            return;
        }
        self.listening = None;
        self.assign(listening.row, Value::Key(key | modifiers));
    }

    /// A click on a binding button: a key button starts listening; a click button starts
    /// listening, or while listening takes the click with the modifiers held. Another button
    /// listening stops (its `OnLeave`). `// C#: KeyBindingButton.cs:83-110;
    /// ClickBindingButton.cs:313-355`
    pub fn press_binding(&mut self, row: usize, button: u32, modifiers: u32) {
        let listening_here = self.listening.is_some_and(|l| l.row == row);
        let is_click = PREFERENCES
            .get(row)
            .is_some_and(|p| p.control == ControlType::ClickBinding);
        if listening_here && is_click {
            self.listening = None;
            self.assign(row, Value::Click(modifiers, button));
            return;
        }
        if listening_here {
            return;
        }
        // Only a left click starts one listening; another button over a button does nothing.
        if button != keys::MOUSE_LEFT {
            return;
        }
        self.listening = Some(Listening { row, shown: None });
    }

    /// The ❌ beside a binding: `None`, which is always taken.
    pub fn clear(&mut self, row: usize) {
        let value = match self.preferences.values.get(row) {
            Some(Value::Click(..)) => Value::Click(0, 0),
            _ => Value::Key(0),
        };
        self.listening = None;
        self.assign(row, value);
    }

    /// An up-down stepped by its increment, held within its minimum and maximum - which
    /// `Math.Min(Min, value)` and `Math.Max(Max, value)` stretch to the value it opened with.
    pub fn step(&mut self, row: usize, up: bool, opened: &GimbalControlSettings) {
        let Some(ControlType::Decimal(min, max, increment, _)) =
            PREFERENCES.get(row).map(|p| p.control)
        else {
            return;
        };
        let Some(Value::Decimal(value)) = self.preferences.values.get(row).copied() else {
            return;
        };
        let (min, max) = bounds(min, max, opened.values.get(row).copied());
        let next = if up {
            value + increment
        } else {
            value - increment
        };
        if let Some(slot) = self.preferences.values.get_mut(row) {
            *slot = Value::Decimal(next.clamp(min, max));
        }
    }

    /// A number typed into an up-down: taken within its bounds, or the old value kept for text
    /// that is not a number, as `NumericUpDown` validates.
    pub fn typed(&mut self, row: usize, text: &str, opened: &GimbalControlSettings) {
        let Some(ControlType::Decimal(min, max, _, places)) =
            PREFERENCES.get(row).map(|p| p.control)
        else {
            return;
        };
        let Ok(value) = text.trim().parse::<f64>() else {
            return;
        };
        let (min, max) = bounds(min, max, opened.values.get(row).copied());
        let factor = 10f64.powi(i32::try_from(places).unwrap_or(2));
        let rounded = (value * factor).round() / factor;
        if let Some(slot) = self.preferences.values.get_mut(row) {
            *slot = Value::Decimal(rounded.clamp(min, max));
        }
    }

    /// A checkbox clicked.
    pub fn toggle(&mut self, row: usize) {
        if let Some(Value::Bool(b)) = self.preferences.values.get_mut(row) {
            *b = !*b;
        }
    }
}

/// An up-down's bounds: `Minimum = Math.Min(Min, value)`, `Maximum = Math.Max(Max, value)`.
fn bounds(min: f64, max: f64, opened: Option<Value>) -> (f64, f64) {
    match opened {
        Some(Value::Decimal(value)) => (min.min(value), max.max(value)),
        _ => (min, max),
    }
}

/// `GimbalVideoControl` itself.
pub struct GimbalVideoControl {
    /// `preferences`.
    pub preferences: GimbalControlSettings,
    /// `_stream`.
    pub stream: mp_video::gstreamer::GStreamer,
    /// `GStreamer.GstLaunch` when `initializeGStreamer` found it; `None` plays nothing.
    pub gst_launch: Option<std::path::PathBuf>,
    /// The keys and rates.
    pub steering: Steering,
    /// `selectedGimbalID`: 0, all gimbals.
    pub selected_gimbal_id: u8,
    /// `mouseMapMarker`'s marker: latitude and longitude.
    pub mouse_marker: Option<(f64, f64)>,
    /// `lastMouseMove`.
    last_mouse_move: Option<Instant>,
    /// `dragStartPoint` and `dragEndPoint`.
    pub drag: Drag,
    /// The menu, where it was opened.
    pub menu: Option<(f32, f32)>,
    /// The settings form, while it is open.
    pub settings: Option<SettingsForm>,
    /// Video Stream's form, while it is open.
    pub selector: Option<StreamSelector>,
    /// The last call made and what went out, for the facts.
    pub last_command: String,
    /// The modifiers last seen, for Shift, Ctrl and Alt pressed alone.
    modifiers: u32,
    /// The keyboard focus: the video box's.
    pub focus: FocusHandle,
    /// The settings form's and the pipeline box's focus.
    pub form_focus: FocusHandle,
    /// The video box, where it was laid out.
    box_bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
    /// The frame shown and the image made of it.
    image: ImageCache,
    /// `AutoConnectTimer`, and the repaints while the video plays; dropped with the control.
    tasks: Vec<gpui::Task<()>>,
    /// The focus subscription that lets the keys go.
    _blur: Option<gpui::Subscription>,
}

impl std::fmt::Debug for GimbalVideoControl {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GimbalVideoControl")
            .field("steering", &self.steering)
            .finish_non_exhaustive()
    }
}

/// The frame shown and the image made of it, shared with the painter.
type ImageCache = Rc<RefCell<Option<(Arc<mp_video::Frame>, Arc<gpui::RenderImage>)>>>;

/// `dragStartPoint` and `dragEndPoint`.
pub type Drag = (Option<(f64, f64)>, Option<(f64, f64)>);

/// The "No Video" picture's size. `// C#: Resources/no-video.png`
pub const NO_VIDEO_SIZE: (u32, u32) = (640, 480);

impl GimbalVideoControl {
    /// `VideoBox.Image`'s size: the frame's, or the "No Video" picture's.
    #[must_use]
    pub fn image_size(&self) -> (u32, u32) {
        self.stream
            .latest()
            .map_or(NO_VIDEO_SIZE, |frame| (frame.width, frame.height))
    }
}

// --- FlightData's side: where the control and the map are --------------------------------------

/// `FlightData`'s gimbal video fields: the control, its three menu items, and the map's dock.
pub struct GimbalVideo {
    /// `_gimbalVideoControl`: `None` before it is made and once disposed.
    pub control: Option<GimbalVideoControl>,
    /// `gimbalVideoShowMiniMap.Checked`.
    pub show_mini_map: bool,
    /// `gimbalVideoShowMiniMap.Visible`, `gimbalVideoSwapPosition.Visible`,
    /// `gimbalVideoClose.Visible`.
    pub items_visible: (bool, bool, bool),
    /// `gMapControl1.Dock == DockStyle.Fill`.
    pub map_fill: bool,
    /// `gMapControl1.Visible`.
    pub map_visible: bool,
    /// The map's panel, where it was laid out.
    panel: Rc<Cell<Option<Bounds<Pixels>>>>,
    /// `splitContainer1_Panel2_Resize`'s last run: the panel's size and the boxes it set, the
    /// mini video's and the mini map's.
    resized: Cell<Option<Resized>>,
    /// Where the control is and whether it is `Visible`: `Some` while there is a control.
    pub placed: Option<(Parent, bool)>,
}

/// One run of `splitContainer1_Panel2_Resize`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Resized {
    /// The panel's size.
    pub panel: (f32, f32),
    /// The mini video's box: x, y, width, height.
    pub video: (i32, i32, i32, i32),
    /// The mini map's box.
    pub map: (i32, i32, i32, i32),
}

impl std::fmt::Debug for GimbalVideo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GimbalVideo")
            .field("control", &self.control)
            .field("show_mini_map", &self.show_mini_map)
            .field("map_fill", &self.map_fill)
            .field("map_visible", &self.map_visible)
            .finish_non_exhaustive()
    }
}

impl Default for GimbalVideo {
    fn default() -> Self {
        Self {
            control: None,
            show_mini_map: false,
            items_visible: (true, true, true),
            map_fill: true,
            map_visible: true,
            panel: Rc::new(Cell::new(None)),
            resized: Cell::new(None),
            placed: None,
        }
    }
}

/// Where the video shows, for the facts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shown {
    /// No control, or not visible.
    Hidden,
    /// Filling the map's panel.
    Full,
    /// The mini video over the map.
    Mini,
    /// In its own window.
    PopOut,
}

impl Shown {
    /// The fact's word.
    #[must_use]
    pub const fn word(self) -> &'static str {
        match self {
            Self::Hidden => "hidden",
            Self::Full => "full",
            Self::Mini => "mini",
            Self::PopOut => "popout",
        }
    }
}

/// `splitContainer1_Panel2_Resize`'s mini video: 30% of the panel each way (truncated), the
/// smaller fitted to the picture's shape, at the bottom right left of the zoom bar.
/// `// C#: GCSViews/FlightData.cs:6597-6610`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // the C#'s `(int)`
pub fn mini_video_box(panel: (i32, i32), image: (u32, u32), zoom_bar: i32) -> (i32, i32, i32, i32) {
    let width = (f64::from(panel.0) * 0.3) as i32;
    let height = (f64::from(panel.1) * 0.3) as i32;
    let aspect = f64::from(image.0) / f64::from(image.1.max(1));
    let (width, height) = (
        width.min((f64::from(height) * aspect) as i32),
        height.min((f64::from(width) / aspect) as i32),
    );
    (panel.0 - width - zoom_bar, panel.1 - height, width, height)
}

/// `splitContainer1_Panel2_Resize`'s mini map: 30% each way at the bottom right.
/// `// C#: GCSViews/FlightData.cs:6611-6619`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // the C#'s `(int)`
pub fn mini_map_box(panel: (i32, i32)) -> (i32, i32, i32, i32) {
    let width = (f64::from(panel.0) * 0.3) as i32;
    let height = (f64::from(panel.1) * 0.3) as i32;
    (panel.0 - width, panel.1 - height, width, height)
}

/// The flight map has no `TRK_zoom` beside it here: the mini video's zoom bar margin.
const ZOOM_BAR: i32 = 0;

impl GimbalVideo {
    /// Where the video shows.
    #[must_use]
    pub fn shown(&self) -> Shown {
        match self.placed {
            Some((parent, true)) => match parent {
                Parent::Form => Shown::PopOut,
                Parent::Panel { fill: true } => Shown::Full,
                Parent::Panel { fill: false } => Shown::Mini,
            },
            _ => Shown::Hidden,
        }
    }

    /// Where the map shows: "full", "mini" or "hidden".
    #[must_use]
    pub const fn map_word(&self) -> &'static str {
        if !self.map_visible {
            "hidden"
        } else if self.map_fill {
            "full"
        } else {
            "mini"
        }
    }

    /// `gimbalVideoShowMiniMap.CheckedChanged`: the map shown with it, and Swap with map.
    /// `// C#: GCSViews/FlightData.cs:6547-6551`
    pub fn set_show_mini_map(&mut self, checked: bool) {
        if self.show_mini_map == checked {
            return;
        }
        self.show_mini_map = checked;
        self.map_visible = checked;
        self.items_visible.1 = checked;
    }

    /// `gimbalVideoFullSizedToolStripMenuItem_Click`, the control already made: the video
    /// filling the panel, the map the mini map where Mini map is checked, a pop-out closed.
    /// `// C#: GCSViews/FlightData.cs:6624-6649`
    pub fn full_sized(&mut self) {
        if self.placed.is_none() {
            return;
        }
        self.placed = Some((Parent::Panel { fill: true }, true));
        self.map_fill = false;
        self.map_visible = self.show_mini_map;
        self.resized.set(None);
        self.items_visible = (true, self.show_mini_map, true);
    }

    /// `gimbalVideoMiniToolStripMenuItem_Click`: the map filling the panel, the video the mini
    /// video, a pop-out closed. `// C#: GCSViews/FlightData.cs:6651-6676`
    pub fn mini(&mut self) {
        self.map_fill = true;
        self.map_visible = true;
        if self.placed.is_none() {
            return;
        }
        self.placed = Some((Parent::Panel { fill: false }, true));
        self.resized.set(None);
        self.items_visible = (false, true, true);
    }

    /// `gimbalVideoPopOutToolStripMenuItem_Click`: the map full sized and the video in a new
    /// form - any form it was in closed first. `// C#: GCSViews/FlightData.cs:6678-6712`
    pub fn pop_out(&mut self) {
        self.map_fill = true;
        self.map_visible = true;
        if self.placed.is_none() {
            return;
        }
        self.placed = Some((Parent::Form, true));
        if let Some(control) = self.control.as_mut() {
            control.menu = None;
        }
        self.items_visible = (false, false, false);
    }

    /// `gimbalVideoSwapPosition.Click`: Full Sized from the mini video, Mini otherwise.
    /// `// C#: GCSViews/FlightData.cs:6552-6562`
    pub fn swap(&mut self) {
        let mini = matches!(self.placed, Some((Parent::Panel { fill: false }, _)));
        if mini {
            self.full_sized();
        } else {
            self.mini();
        }
    }

    /// `gimbalVideoClose.Click`: Mini, hidden, `Stop`, `Dispose` - so the next item makes a new
    /// control. `// C#: GCSViews/FlightData.cs:6563-6569`
    pub fn close(&mut self) {
        self.mini();
        self.dispose();
    }

    /// The pop-out form closed: the control goes with it, as a form disposes its controls.
    pub fn form_closed(&mut self) {
        if self.shown() == Shown::PopOut {
            self.dispose();
        }
    }

    /// `Dispose`: `Stop`, and gone.
    fn dispose(&mut self) {
        self.placed = None;
        if let Some(mut control) = self.control.take() {
            // `Stop`: `_stream.Stop()`. `// C#: Controls/GimbalVideoControl.cs:305-309`
            control.stream.stop();
            control.tasks.clear();
        }
    }

    /// `splitContainer1_Panel2_Resize`, when the panel is `panel` in size: the boxes set again
    /// if it has changed size or a menu item has asked.
    fn resize(&self, panel: (f32, f32)) -> Resized {
        if let Some(resized) = self.resized.get()
            && resized.panel == panel
        {
            return resized;
        }
        #[allow(clippy::cast_possible_truncation)] // pixels
        let whole = (panel.0 as i32, panel.1 as i32);
        let image = self
            .control
            .as_ref()
            .map_or(NO_VIDEO_SIZE, GimbalVideoControl::image_size);
        let resized = Resized {
            panel,
            video: mini_video_box(whole, image, ZOOM_BAR),
            map: mini_map_box(whole),
        };
        self.resized.set(Some(resized));
        resized
    }

    /// Publishes what a UI test asserts on.
    pub fn record_facts(&self) {
        use crate::facts::record;
        record("fly.gimbal.video", self.shown().word());
        record("fly.gimbal.minimap", self.show_mini_map);
        record("fly.gimbal.map", self.map_word());
        let items: Vec<&str> = [
            (self.items_visible.0, "minimap"),
            (self.items_visible.1, "swap"),
            (self.items_visible.2, "close"),
        ]
        .into_iter()
        .filter_map(|(shown, name)| shown.then_some(name))
        .collect();
        record(
            "fly.gimbal.items",
            if items.is_empty() {
                "none".to_owned()
            } else {
                items.join(",")
            },
        );
        match self.resized.get() {
            Some(resized) if self.shown() == Shown::Mini => {
                let (x, y, w, h) = resized.video;
                record("fly.gimbal.minivideo", format!("{x},{y},{w},{h}"));
            }
            _ => record("fly.gimbal.minivideo", "none"),
        }
        match self.resized.get() {
            Some(resized) if self.shown() == Shown::Full && self.map_visible => {
                let (x, y, w, h) = resized.map;
                record("fly.gimbal.minimapbox", format!("{x},{y},{w},{h}"));
            }
            _ => record("fly.gimbal.minimapbox", "none"),
        }
        let Some(control) = &self.control else {
            for key in [
                "fly.gimbal.frames",
                "fly.gimbal.frame",
                "fly.gimbal.pipeline",
                "fly.gimbal.menu",
                "fly.gimbal.settings",
                "fly.gimbal.selector",
                "fly.gimbal.marker",
                "fly.gimbal.held",
            ] {
                record(key, "none");
            }
            record("fly.gimbal.frames", 0);
            record("fly.gimbal.menu", false);
            record("fly.gimbal.settings", false);
            record("fly.gimbal.selector", false);
            return;
        };
        record("fly.gimbal.frames", control.stream.frames());
        record(
            "fly.gimbal.frame",
            control.stream.latest().map_or_else(
                || "none".to_owned(),
                |f| format!("{}x{}", f.width, f.height),
            ),
        );
        record(
            "fly.gimbal.pipeline",
            control.stream.pipeline().unwrap_or("none"),
        );
        record(
            "fly.gimbal.error",
            control.stream.error().unwrap_or_else(|| "none".to_owned()),
        );
        record("fly.gimbal.menu", control.menu.is_some());
        record("fly.gimbal.yawlock", control.steering.yaw_lock);
        record("fly.gimbal.recording", control.steering.recording);
        record(
            "fly.gimbal.rates",
            format!(
                "{},{},{}",
                control.steering.pitch_rate, control.steering.yaw_rate, control.steering.zoom_rate
            ),
        );
        record(
            "fly.gimbal.held",
            if control.steering.held.is_empty() {
                "none".to_owned()
            } else {
                control
                    .steering
                    .held
                    .iter()
                    .map(|k| keys::name(*k))
                    .collect::<Vec<_>>()
                    .join(",")
            },
        );
        record(
            "fly.gimbal.command",
            if control.last_command.is_empty() {
                "none"
            } else {
                control.last_command.as_str()
            },
        );
        record(
            "fly.gimbal.marker",
            control.mouse_marker.map_or_else(
                || "none".to_owned(),
                |(lat, lng)| format!("{lat:.6},{lng:.6}"),
            ),
        );
        record(
            "fly.gimbal.drag",
            match control.drag {
                (Some((sx, sy)), Some((ex, ey))) => format!("{sx:.3},{sy:.3},{ex:.3},{ey:.3}"),
                _ => "none".to_owned(),
            },
        );
        for (index, preference) in PREFERENCES.iter().enumerate() {
            record(
                format!("fly.gimbal.pref.{}", preference.name),
                control.preferences.text(index),
            );
        }
        record("fly.gimbal.settings", control.settings.is_some());
        if let Some(form) = &control.settings {
            record(
                "fly.gimbal.settings.listening",
                form.listening
                    .and_then(|l| PREFERENCES.get(l.row))
                    .map_or("none", |p| p.name),
            );
            record(
                "fly.gimbal.settings.clash",
                form.clash
                    .as_ref()
                    .map_or_else(|| "none".to_owned(), |(_, _, message)| message.clone()),
            );
            for (index, preference) in PREFERENCES.iter().enumerate() {
                record(
                    format!("fly.gimbal.settings.{}", preference.name),
                    form.preferences.text(index),
                );
            }
        }
        record("fly.gimbal.selector", control.selector.is_some());
        if let Some(selector) = &control.selector {
            record("fly.gimbal.selector.pipeline", selector.pipeline.value());
        }
    }
}

// --- The handlers, on the application ---------------------------------------------------------

/// The runtime's launcher, looked for as `GStreamer.LookForGstreamer()` looks - the `PATH`, the
/// program's directory, the data directory - and saved as `gstlaunchexe`.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1401-1485`
fn look_for_gstreamer() -> Option<std::path::PathBuf> {
    let path = std::env::var_os("PATH");
    let program = std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(std::path::Path::to_path_buf));
    let data = mp_settings::data_directory();
    let (on_path, dirs) = mp_video::gstreamer::search_dirs(
        path.as_ref(),
        program.as_deref(),
        data.as_deref(),
        &mp_video::gstreamer::drives(),
    );
    mp_video::gstreamer::look_for_gstreamer(&on_path, &dirs)
}

/// How often the picture is redrawn while the video plays.
const REPAINT: Duration = Duration::from_millis(33);
/// `AutoConnectTimer.Interval`.
const AUTO_CONNECT: Duration = Duration::from_secs(1);

impl MissionPlanner {
    /// `gimbalVideoControl`'s getter: the control made if there is none - its preferences read,
    /// its yaw lock their default, GStreamer looked for, the auto-connect timer started - and the
    /// three menu items shown on it, Mini map checked.
    /// `// C#: GCSViews/FlightData.cs:6539-6587; Controls/GimbalVideoControl.cs:99-209`
    fn gimbal_video_control(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if self.fly_data.gimbal_video.control.is_some() {
            return;
        }
        let preferences = GimbalControlSettings::from_json(
            self.persisted.get(PREFERENCES_KEY).unwrap_or_default(),
        );
        let yaw_lock = preferences.flag("DefaultLockedMode");
        // `initializeGStreamer`: `GStreamer.GstLaunch = GStreamer.LookForGstreamer()`.
        let found = look_for_gstreamer();
        let gst_launch = found
            .as_ref()
            .map(|path| path.display().to_string())
            .unwrap_or_default();
        self.persisted.set("gstlaunchexe", gst_launch.as_str());
        let gst_launch = mp_video::gstreamer::gst_launch_exists(&gst_launch)
            .then_some(found)
            .flatten();
        if gst_launch.is_none() {
            self.file_status = Some(
                "This feature requires GStreamer: the GStreamer runtime (gst-launch-1.0) was not found"
                    .to_owned(),
            );
        }
        let focus = cx.focus_handle();
        let blur = cx.on_blur(&focus, window, |this, _window, cx| {
            gimbal_calls(this, |control, calls| {
                let modifiers = control.modifiers;
                let preferences = control.preferences.clone();
                control.steering.focus_lost(modifiers, &preferences, calls);
            });
            cx.notify();
        });
        let mut control = GimbalVideoControl {
            preferences,
            stream: mp_video::gstreamer::GStreamer::default(),
            gst_launch,
            steering: Steering {
                yaw_lock,
                ..Steering::default()
            },
            selected_gimbal_id: 0,
            mouse_marker: None,
            last_mouse_move: None,
            drag: (None, None),
            menu: None,
            settings: None,
            selector: None,
            last_command: String::new(),
            modifiers: 0,
            focus,
            form_focus: cx.focus_handle(),
            box_bounds: Rc::new(Cell::new(None)),
            image: Rc::new(RefCell::new(None)),
            tasks: Vec::new(),
            _blur: Some(blur),
        };
        if control.gst_launch.is_some() {
            control.tasks.push(auto_connect_task(cx));
            control.tasks.push(repaint_task(cx));
        }
        let video = &mut self.fly_data.gimbal_video;
        video.control = Some(control);
        video.placed = Some((Parent::Panel { fill: true }, false));
        // `gimbalVideoShowMiniMap.Checked = true`, its handler run if that changes it.
        video.set_show_mini_map(true);
    }

    /// Full Sized. `// C#: GCSViews/FlightData.cs:6624-6649`
    pub fn gimbal_video_full_sized(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.gimbal_video_control(window, cx);
        self.fly_data.gimbal_video.full_sized();
    }

    /// Mini. `// C#: GCSViews/FlightData.cs:6651-6676`
    pub fn gimbal_video_mini(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.gimbal_video_control(window, cx);
        self.fly_data.gimbal_video.mini();
    }

    /// Pop Out, and the Payload tab's Video Control. `// C#: GCSViews/FlightData.cs:6678-6712`
    pub fn gimbal_video_pop_out(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.gimbal_video_control(window, cx);
        self.fly_data.gimbal_video.pop_out();
    }

    /// The vehicle side of a call: the shown vehicle's manager and camera, the camera given the
    /// preferences' fields of view as `UITimer` gives them.
    fn gimbal_selected(&self, preferences: &GimbalControlSettings) -> Selected {
        let view = self.telemetry.view();
        let state = view.state.as_deref();
        let yaw = state.map_or(0.0, |s| s.attitude.yaw.0.to_degrees());
        let home = state.map_or((0.0, 0.0, 0.0), |s| {
            let at = s.home.map_or((0.0, 0.0), |h| (h.latitude(), h.longitude()));
            (at.0, at.1, s.home_altitude.0)
        });
        let manager = self.telemetry.gimbal_manager();
        let camera = self.telemetry.camera().map(|(_, mut camera)| {
            // `UITimer_Tick`. `// C#: Controls/GimbalVideoControl.cs:705-710`
            #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
            {
                camera.hfov_set = preferences.decimal("CameraHFOV") as f32;
                camera.vfov_set = preferences.decimal("CameraVFOV") as f32;
            }
            camera.use_fov_status = preferences.flag("UseFOVReportedByCamera");
            camera
        });
        Selected {
            target: manager
                .as_ref()
                .map(|(id, _)| *id)
                .or_else(|| camera.as_ref().map(|c| c.vehicle)),
            manager: manager.map(|(_, m)| m),
            camera,
            yaw: (yaw + 360.0) % 360.0,
            home,
        }
    }

    /// Each call made: its messages sent, a command waited on as `doCommandAsync` waits, and the
    /// last one said in the facts.
    fn gimbal_perform(&mut self, calls: Vec<Call>) {
        let Some(control) = self.fly_data.gimbal_video.control.as_ref() else {
            return;
        };
        let preferences = control.preferences.clone();
        let gimbal_id = control.selected_gimbal_id;
        let yaw_lock = control.steering.yaw_lock;
        let mut selected = self.gimbal_selected(&preferences);
        let mut said = None;
        for call in calls {
            let text = match messages(call, &mut selected, gimbal_id, yaw_lock) {
                Ok(out) => {
                    let mut words = Vec::new();
                    for (message, wait) in out {
                        words.push(crate::fly::describe(&message));
                        if !wait {
                            if let Some((sender, _)) = self.telemetry.send_handle() {
                                let _ = sender.send(&message);
                            }
                        } else if let MavMessage::CommandInt(c) = &message {
                            let _ = self.telemetry.command_int(
                                VehicleId::new(c.target_system, c.target_component),
                                c.command,
                                c.frame,
                                [c.param1, c.param2, c.param3, c.param4],
                                c.x,
                                c.y,
                                c.z,
                                crate::telemetry::Report::default(),
                            );
                        } else {
                            let _ = self
                                .telemetry
                                .command_message(&message, crate::telemetry::Report::default());
                        }
                    }
                    format!("{}: {}", call.member(), words.join("; "))
                }
                Err(why) => format!("{}: not sent ({why})", call.member()),
            };
            said = Some(text);
        }
        // The camera's picture sequence counts on in the link's copy only through what is sent;
        // the copy made here is let go.
        if let (Some(text), Some(control)) = (said, self.fly_data.gimbal_video.control.as_mut()) {
            control.last_command = text;
        }
    }

    /// A menu item of the control's. `// C#: Controls/GimbalVideoControl.cs:311-333, 713-763;
    /// GCSViews/FlightData.cs:6547-6569`
    pub fn gimbal_menu(&mut self, item: MenuItem, window: &mut Window, cx: &mut Context<Self>) {
        let video = &mut self.fly_data.gimbal_video;
        if let Some(control) = video.control.as_mut() {
            control.menu = None;
        }
        let mut calls = Vec::new();
        match item {
            MenuItem::VideoStream => {
                let Some(control) = video.control.as_mut() else {
                    return;
                };
                // `GStreamer.GstLaunch = LookForGstreamer()`; without it, nothing.
                if control.gst_launch.is_none() {
                    self.file_status = Some(
                        "This feature requires GStreamer: the GStreamer runtime (gst-launch-1.0) was not found"
                            .to_owned(),
                    );
                    return;
                }
                control.selector = Some(StreamSelector {
                    selected: None,
                    pipeline: TextField::new(""),
                });
                control.form_focus.focus(window, cx);
            }
            MenuItem::Settings => {
                let Some(control) = video.control.as_mut() else {
                    return;
                };
                control.settings = Some(SettingsForm::new(&control.preferences));
                control.form_focus.focus(window, cx);
            }
            MenuItem::MiniMap => {
                let checked = !video.show_mini_map;
                video.set_show_mini_map(checked);
            }
            MenuItem::Swap => video.swap(),
            MenuItem::Close => video.close(),
            MenuItem::Retract => calls.push(Call::Retract),
            MenuItem::Neutral => calls.push(Call::Neutral),
            MenuItem::PointDown => calls.push(Call::PointDown),
            MenuItem::PointHome => calls.push(Call::Home),
            MenuItem::TakePicture => calls.push(Call::TakePicture),
            MenuItem::YawLock | MenuItem::StartRecording | MenuItem::StopRecording => {
                if let Some(control) = video.control.as_mut() {
                    match item {
                        // `SetYawLock(!item.Checked)`.
                        MenuItem::YawLock => {
                            let locked = !control.steering.yaw_lock;
                            control.steering.set_yaw_lock(locked, &mut calls);
                        }
                        MenuItem::StartRecording => {
                            control.steering.set_recording(true, &mut calls)
                        }
                        _ => control.steering.set_recording(false, &mut calls),
                    }
                }
            }
        }
        self.gimbal_perform(calls);
        cx.notify();
    }
}

/// Runs `act` on the control with a list for its calls, then performs them.
fn gimbal_calls(
    this: &mut MissionPlanner,
    act: impl FnOnce(&mut GimbalVideoControl, &mut Vec<Call>),
) {
    let mut calls = Vec::new();
    if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
        act(control, &mut calls);
    }
    this.gimbal_perform(calls);
}

/// `AutoConnectTimerCallback`, once a second until a stream plays: with no stream reported yet,
/// the camera asked for its information (and the next tick waits for that to end, as `.Wait()`
/// does); then the stream last used, by its URI, or the first, saved as `gimbal_video_stream`.
/// `// C#: Controls/GimbalVideoControl.cs:765-795`
fn auto_connect_task(cx: &mut Context<MissionPlanner>) -> gpui::Task<()> {
    cx.spawn(async move |this, cx| {
        loop {
            cx.background_executor().timer(AUTO_CONNECT).await;
            let Ok(done) = this.update(cx, |this, cx| {
                let done = auto_connect(this);
                cx.notify();
                done
            }) else {
                break;
            };
            if done {
                break;
            }
        }
    })
}

/// One tick of the auto-connect timer; true once it has started a stream.
fn auto_connect(this: &mut MissionPlanner) -> bool {
    let streams = this.telemetry.video_streams();
    if streams.is_empty() {
        if !this.telemetry.camera_information_pending() {
            this.telemetry.request_camera_information();
        }
        return false;
    }
    let previous = this
        .persisted
        .get(STREAM_KEY)
        .unwrap_or_default()
        .to_owned();
    let uri =
        |s: &mp_mavlink_dialects::all::VideoStreamInformation| mp_link::camera::c_string(&s.uri);
    let chosen = streams
        .iter()
        .find(|(_, s)| uri(s) == previous)
        .or_else(|| streams.first())
        .map(|(_, s)| *s);
    let Some(stream) = chosen else {
        return false;
    };
    if uri(&stream) != previous {
        this.persisted.set(STREAM_KEY, uri(&stream).as_str());
    }
    let pipeline = mp_link::camera::gstreamer_pipeline(&stream);
    if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
        start_stream(control, &pipeline);
    }
    true
}

/// `_stream.Start(pipeline)`: a refusal kept as the stream's error, as the C#'s shows its own.
fn start_stream(control: &mut GimbalVideoControl, pipeline: &str) {
    let Some(launch) = control.gst_launch.clone() else {
        return;
    };
    if let Err(why) = control.stream.start(&launch, pipeline) {
        control.last_command = format!("GStreamer: {why}");
    }
}

/// The picture redrawn while the control lives.
fn repaint_task(cx: &mut Context<MissionPlanner>) -> gpui::Task<()> {
    cx.spawn(async move |this, cx| {
        loop {
            cx.background_executor().timer(REPAINT).await;
            let playing = this.update(cx, |this, cx| {
                let playing = this
                    .fly_data
                    .gimbal_video
                    .control
                    .as_ref()
                    .is_some_and(|c| c.stream.is_running());
                if playing {
                    cx.notify();
                }
            });
            if playing.is_err() {
                break;
            }
        }
    })
}

// --- Drawing ------------------------------------------------------------------------------------

/// The map's panel with the gimbal video in it: the map as drawn, the mini video over it, or the
/// video filling it with the mini map over that - and the pop-out window, the menu, the forms and
/// the mouse marker. `// C#: GCSViews/FlightData.cs:6589-6712`
pub fn map_place(
    this: &MissionPlanner,
    map: AnyElement,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let video = &this.fly_data.gimbal_video;
    video.record_facts();
    let shown = video.shown();
    let panel = Rc::clone(&video.panel);
    let measure = canvas(
        move |bounds, _window, _cx| panel.set(Some(bounds)),
        |_bounds, (), _window, _cx| {},
    )
    .absolute()
    .size_full();
    let size_now = video
        .panel
        .get()
        .map(|b| (f32::from(b.size.width), f32::from(b.size.height)));
    let resized = size_now.map(|s| video.resize(s));
    let mut place = crate::probe::measured("fly-gimbal-panel", div())
        .relative()
        .flex()
        .flex_1()
        .min_w(px(0.0))
        .min_h(px(0.0))
        .child(measure);
    let control = video.control.as_ref();
    match (shown, control) {
        (Shown::Full, Some(control)) => {
            place = place.child(
                div()
                    .absolute()
                    .size_full()
                    .child(video_box(control, this, cx)),
            );
            if video.map_visible
                && let Some(resized) = resized
            {
                let (x, y, w, h) = resized.map;
                // The map itself, small: its right button opens `contextMenuStripMap` as the
                // full map's does, through the map's own handler.
                place = place.child(
                    crate::probe::measured("fly-gimbal-minimap", div())
                        .id("fly-gimbal-minimap")
                        .absolute()
                        .left(px(x as f32))
                        .top(px(y as f32))
                        .w(px(w as f32))
                        .h(px(h as f32))
                        .flex()
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .child(map),
                );
            }
        }
        (Shown::Mini, Some(control)) => {
            // `min_w(0)` is load-bearing: a flex item's minimum is otherwise its content's
            // width, and the strip under the map is a nowrap line whose full width - a long
            // status, a fetch's URL - then became this wrapper's minimum, the pane's and the
            // map's: 1622 px in a 1184 px column, the map's right 438 px and its follow button
            // off the window and the vehicle drawn off centre (the layout guard found it,
            // 2026-10-03; main.rs's `map_status` had the same fix one level down).
            place = place.child(div().flex().flex_1().min_w(px(0.0)).child(map));
            if let Some(resized) = resized {
                let (x, y, w, h) = resized.video;
                place = place.child(
                    div()
                        .absolute()
                        .left(px(x as f32))
                        .top(px(y as f32))
                        .w(px(w as f32))
                        .h(px(h as f32))
                        .child(video_box(control, this, cx)),
                );
            }
        }
        _ => {
            if video.map_visible {
                place = place.child(div().flex().flex_1().min_w(px(0.0)).child(map));
            }
        }
    }
    if let Some(control) = control {
        if shown == Shown::PopOut {
            place = place.child(pop_out_window(control, this, window, cx));
        }
        if let Some(marker) = control.mouse_marker {
            place = place.children(mouse_marker(this, marker));
        }
        place = place.children(context_menu(control, this, window, cx));
        place = place.children(settings_form(control, window, cx));
        place = place.children(stream_selector(control, this, window, cx));
    }
    place.into_any_element()
}

/// `mouseMapMarker`'s `GMarkerGoogle` (`blue_small`) where the map drew the point.
fn mouse_marker(this: &MissionPlanner, (lat, lng): (f64, f64)) -> Option<AnyElement> {
    let at = mp_units::LatLon::new(lat, lng).ok()?;
    let (x, y) = this.map.borrow().screen_of(at)?;
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(point(px(x - 5.0), px(y - 14.0)))
                .child(
                    crate::probe::measured("fly-gimbal-marker", div())
                        .flex()
                        .flex_col()
                        .items_center()
                        .child(
                            div()
                                .size(px(10.0))
                                .rounded_full()
                                .bg(rgb(0x3b_6f_e0))
                                .border_1()
                                .border_color(rgb(0xff_ff_ff)),
                        )
                        .child(div().w(px(2.0)).h(px(4.0)).bg(rgb(0x3b_6f_e0))),
                ),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// `VideoBox`: the frame zoomed into the box (`PictureBoxSizeMode.Zoom`), or the "No Video"
/// picture, with `RenderFrame`'s drag box or tracked target over it; its mouse and keys.
/// `// C#: Controls/GimbalVideoControl.cs:211-290, 335-668`
fn video_box(
    control: &GimbalVideoControl,
    this: &MissionPlanner,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let latest = control.stream.latest();
    let no_video = latest.is_none();
    // Its frames drawn as they come; while none has, looked for as a job in flight (repaint.rs).
    crate::repaint::again_in(if no_video {
        crate::repaint::IN_FLIGHT
    } else {
        crate::repaint::VIDEO_FRAME
    });
    let image_cache = Rc::clone(&control.image);
    let box_bounds = Rc::clone(&control.box_bounds);
    let drag = control.drag;
    let tracking = this
        .telemetry
        .camera()
        .and_then(|(_, camera)| camera.tracking_image_status);
    let image_size = control.image_size();
    let painter = canvas(
        move |bounds, _window, _cx| box_bounds.set(Some(bounds)),
        move |bounds, (), window, cx| {
            paint_video(
                bounds,
                latest.as_ref(),
                &image_cache,
                image_size,
                drag,
                tracking,
                window,
                cx,
            );
        },
    )
    .absolute()
    .size_full();
    crate::probe::measured("fly-gimbal-video-box", div())
        .id("fly-gimbal-video-box")
        .track_focus(&control.focus)
        .relative()
        .size_full()
        .overflow_hidden()
        .bg(rgb(0x00_00_00))
        .child(painter)
        .children(no_video.then(|| {
            div()
                .absolute()
                .size_full()
                .flex()
                .items_center()
                .justify_center()
                .child(
                    div()
                        .border_2()
                        .border_color(rgb(0xff_ff_ff))
                        .px_2()
                        .text_color(rgb(0xff_ff_ff))
                        .font_weight(gpui::FontWeight::BOLD)
                        .child("No Video"),
                )
        }))
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            // A repeat from a key held down is not handled again.
            if event.is_held {
                return;
            }
            let Some(key) = keys::code(&event.keystroke.key) else {
                return;
            };
            let modifiers = modifier_bits(event.keystroke.modifiers);
            let mut taken = false;
            gimbal_calls(this, |control, calls| {
                let preferences = control.preferences.clone();
                taken = control
                    .steering
                    .key_down(key, modifiers, &preferences, calls);
            });
            if taken {
                cx.stop_propagation();
            }
            cx.notify();
        }))
        .on_key_up(cx.listener(|this, event: &KeyUpEvent, _window, cx| {
            let Some(key) = keys::code(&event.keystroke.key) else {
                return;
            };
            let modifiers = modifier_bits(event.keystroke.modifiers);
            gimbal_calls(this, |control, calls| {
                let preferences = control.preferences.clone();
                control.steering.key_up(key, modifiers, &preferences, calls);
            });
            cx.notify();
        }))
        .on_modifiers_changed(
            cx.listener(|this, event: &ModifiersChangedEvent, window, cx| {
                let focused = this
                    .fly_data
                    .gimbal_video
                    .control
                    .as_ref()
                    .is_some_and(|c| c.focus.is_focused(window));
                let now = modifier_bits(event.modifiers);
                gimbal_calls(this, |control, calls| {
                    let before = control.modifiers;
                    control.modifiers = now;
                    if !focused {
                        return;
                    }
                    let preferences = control.preferences.clone();
                    for (bit, key) in [
                        (keys::SHIFT, keys::SHIFT_KEY),
                        (keys::CONTROL, keys::CONTROL_KEY),
                        (keys::ALT, keys::MENU),
                    ] {
                        if now & bit != 0 && before & bit == 0 {
                            // The modifier's own key down, with the modifiers as they now are.
                            control.steering.key_down(key, now, &preferences, calls);
                        } else if now & bit == 0 && before & bit != 0 {
                            control.steering.key_up(key, now, &preferences, calls);
                        }
                    }
                });
                cx.notify();
            }),
        )
        .on_mouse_move(cx.listener(|this, event: &MouseMoveEvent, _window, cx| {
            video_mouse_move(this, event);
            cx.notify();
        }))
        .on_hover(cx.listener(|this, hovered: &bool, _window, cx| {
            if !*hovered {
                // `VideoBox_MouseLeave`. `// C#: Controls/GimbalVideoControl.cs:602-607`
                if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
                    control.mouse_marker = None;
                    control.drag = (None, None);
                }
                cx.notify();
            }
        }))
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, _event: &MouseDownEvent, window, cx| {
                focus_video(this, window, cx);
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, event: &MouseUpEvent, window, cx| {
                video_click(this, event, keys::MOUSE_LEFT, window, cx);
            }),
        )
        .on_mouse_up(
            MouseButton::Middle,
            cx.listener(|this, event: &MouseUpEvent, window, cx| {
                video_click(this, event, keys::MOUSE_MIDDLE, window, cx);
            }),
        )
        .on_mouse_up(
            MouseButton::Right,
            cx.listener(|this, event: &MouseUpEvent, window, cx| {
                video_click(this, event, keys::MOUSE_RIGHT, window, cx);
                // `VideoBox.ContextMenuStrip`: the menu where the button came up - this one, not
                // the map's under the box, whose own right button opens `contextMenuStripMap`.
                if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
                    control.menu = Some((f32::from(event.position.x), f32::from(event.position.y)));
                }
                cx.stop_propagation();
                cx.notify();
            }),
        )
        .into_any_element()
}

/// `Control.ModifierKeys` from gpui's.
const fn modifier_bits(modifiers: Modifiers) -> u32 {
    keys::modifiers(modifiers.shift, modifiers.control, modifiers.alt)
}

/// `VideoBox.Focus()`.
fn focus_video(this: &mut MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    if let Some(control) = this.fly_data.gimbal_video.control.as_ref() {
        control.focus.focus(window, cx);
    }
}

/// A point in the box, from the window position.
fn box_point(control: &GimbalVideoControl, x: f32, y: f32) -> Option<(f64, f64)> {
    let bounds = control.box_bounds.get()?;
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)] // pixels
    let size = |v: f32| v as i32;
    let (iw, ih) = control.image_size();
    mouse_position(
        size(x - f32::from(bounds.origin.x)),
        size(y - f32::from(bounds.origin.y)),
        (
            size(f32::from(bounds.size.width)),
            size(f32::from(bounds.size.height)),
        ),
        (
            i32::try_from(iw).unwrap_or(i32::MAX),
            i32::try_from(ih).unwrap_or(i32::MAX),
        ),
    )
}

/// `VideoBox_MouseMove`: the map's marker where the pointer looks, every 100 ms, and the drag
/// box while the tracking click is held. `// C#: Controls/GimbalVideoControl.cs:565-600`
fn video_mouse_move(this: &mut MissionPlanner, event: &MouseMoveEvent) {
    let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
    let now = Instant::now();
    let due = this
        .fly_data
        .gimbal_video
        .control
        .as_ref()
        .is_some_and(|c| {
            c.last_mouse_move
                .is_none_or(|at| now > at + Duration::from_millis(100))
        });
    if due {
        let preferences = this
            .fly_data
            .gimbal_video
            .control
            .as_ref()
            .map(|c| c.preferences.clone())
            .unwrap_or_default();
        let selected = this.gimbal_selected(&preferences);
        if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
            control.last_mouse_move = Some(now);
            control.mouse_marker = None;
            if let Some((px_, py_)) = box_point(control, x, y) {
                control.mouse_marker = selected
                    .camera
                    .as_ref()
                    .and_then(|camera| image_point_location(camera, px_, py_))
                    .map(|(lat, lng, _)| (lat, lng));
            }
        }
    }
    let Some(control) = this.fly_data.gimbal_video.control.as_mut() else {
        return;
    };
    let button = match event.pressed_button {
        Some(MouseButton::Left) => keys::MOUSE_LEFT,
        Some(MouseButton::Right) => keys::MOUSE_RIGHT,
        Some(MouseButton::Middle) => keys::MOUSE_MIDDLE,
        _ => keys::MOUSE_NONE,
    };
    let pressed = (modifier_bits(event.modifiers), button);
    if pressed == control.preferences.click("TrackObjectUnderMouse") {
        let point = box_point(control, x, y);
        if control.drag.0.is_none() {
            control.drag.0 = point;
        }
        control.drag.1 = point;
    } else {
        control.drag = (None, None);
    }
}

/// `VideoBox_Click`: focus, then the binding the modifiers and button match - pan and tilt to the
/// point, the point of interest there, or track what is there.
/// `// C#: Controls/GimbalVideoControl.cs:609-668`
fn video_click(
    this: &mut MissionPlanner,
    event: &MouseUpEvent,
    button: u32,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    focus_video(this, window, cx);
    let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
    let modifiers = modifier_bits(event.modifiers);
    let mut call = None;
    if let Some(control) = this.fly_data.gimbal_video.control.as_ref()
        && let Some((px_, py_)) = box_point(control, x, y)
    {
        let pressed = (modifiers, button);
        let preferences = &control.preferences;
        if pressed == preferences.click("MoveCameraToMouseLocation") {
            call = Some(Call::PanTilt(px_, py_));
        } else if pressed == preferences.click("MoveCameraPOIToMouseLocation") {
            call = Some(Call::PointOfInterest(px_, py_));
        } else if pressed == preferences.click("TrackObjectUnderMouse") {
            call = Some(Call::Track(px_, py_, control.drag.0));
        }
    }
    if let Some(call) = call {
        this.gimbal_perform(vec![call]);
    }
    cx.notify();
}

/// Paints the picture and `RenderFrame`'s overlays.
#[allow(clippy::too_many_arguments)]
fn paint_video(
    bounds: Bounds<Pixels>,
    latest: Option<&Arc<mp_video::Frame>>,
    cache: &ImageCache,
    image_size: (u32, u32),
    drag: Drag,
    tracking: Option<mp_mavlink_dialects::all::CameraTrackingImageStatus>,
    window: &mut Window,
    cx: &mut gpui::App,
) {
    #[allow(clippy::cast_precision_loss)] // pixels
    let (iw, ih) = (image_size.0 as f32, image_size.1 as f32);
    let area = (f32::from(bounds.size.width), f32::from(bounds.size.height));
    let Some(&(x, y, w, h)) =
        crate::pictures::placements(crate::pictures::Layout::ZoomImage, area, (iw, ih)).first()
    else {
        return;
    };
    let picture = Bounds {
        origin: point(bounds.origin.x + px(x), bounds.origin.y + px(y)),
        size: size(px(w), px(h)),
    };
    let red = Hsla::from(gpui::rgb(0xff_00_00));
    if let Some(frame) = latest {
        let fresh = cache
            .borrow()
            .as_ref()
            .is_none_or(|(shown, _)| !Arc::ptr_eq(shown, frame));
        if fresh
            && let Some(buffer) =
                image::RgbaImage::from_raw(frame.width, frame.height, frame.to_bgra())
        {
            let image = Arc::new(gpui::RenderImage::new(vec![image::Frame::new(buffer)]));
            if let Some((_, old)) = cache.replace(Some((Arc::clone(frame), image))) {
                cx.drop_image(old, Some(window));
            }
        }
        if let Some((_, image)) = cache.borrow().as_ref() {
            let _ = window.paint_image(
                picture,
                picture,
                gpui::Corners::default(),
                Arc::clone(image),
                0,
                false,
            );
        }
    } else {
        // The "No Video" picture's cross.
        let dark_red = Hsla::from(gpui::rgb(0x8b_00_00));
        for (from, to) in [((0.0, 0.0), (1.0, 1.0)), ((1.0, 0.0), (0.0, 1.0))] {
            let mut line = PathBuilder::stroke(px(2.0));
            line.move_to(point(
                picture.origin.x + picture.size.width * from.0,
                picture.origin.y + picture.size.height * from.1,
            ));
            line.line_to(point(
                picture.origin.x + picture.size.width * to.0,
                picture.origin.y + picture.size.height * to.1,
            ));
            if let Ok(path) = line.build() {
                window.paint_path(path, dark_red);
            }
        }
        return;
    }
    // Image coordinates to the box's.
    #[allow(clippy::cast_possible_truncation)] // pixels
    let at = |ix: f64, iy: f64| {
        point(
            picture.origin.x + px((ix * f64::from(w) / f64::from(iw)) as f32),
            picture.origin.y + px((iy * f64::from(h) / f64::from(ih)) as f32),
        )
    };
    let rectangle = |window: &mut Window, x: f64, y: f64, rw: f64, rh: f64| {
        let mut path = PathBuilder::stroke(px(1.0));
        path.move_to(at(x, y));
        path.line_to(at(x + rw, y));
        path.line_to(at(x + rw, y + rh));
        path.line_to(at(x, y + rh));
        path.close();
        if let Ok(path) = path.build() {
            window.paint_path(path, red);
        }
    };
    let (fw, fh) = (f64::from(iw), f64::from(ih));
    if let (Some(start), Some(end)) = drag {
        // The drag box. `// C#: Controls/GimbalVideoControl.cs:237-249`
        let x = (start.0.min(end.0) + 1.0) * fw / 2.0;
        let y = (start.1.min(end.1) + 1.0) * fh / 2.0;
        let rw = (start.0 - end.0).abs() * fw / 2.0;
        let rh = (start.1 - end.1).abs() * fh / 2.0;
        rectangle(window, x, y, rw, rh);
    } else if let Some(status) = tracking
        && u32::from(status.tracking_status)
            == mp_mavlink_dialects::all::CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_ACTIVE.0
        && status.target_data & 2 == 0
        && status.target_data & 4 != 0
    {
        // The target the camera tracks, unless the camera has drawn it: a point's circle or a
        // rectangle. `// C#: Controls/GimbalVideoControl.cs:250-281`
        if u32::from(status.tracking_mode)
            == mp_mavlink_dialects::all::CameraTrackingMode::CAMERA_TRACKING_MODE_POINT.0
            && !status.point_x.is_nan()
            && !status.point_y.is_nan()
        {
            let size = if status.radius.is_nan() { 10.0 } else { f64::from(status.radius) };
            // `(int)x - size / 2`.
            let x = f64::from(status.point_x) * fw;
            let y = f64::from(status.point_y) * fh;
            let (cx_, cy_) = (x.trunc(), y.trunc());
            let mut path = PathBuilder::stroke(px(1.0));
            let steps = 32;
            for step in 0..=steps {
                let angle = f64::from(step) / f64::from(steps) * std::f64::consts::TAU;
                let p = at(cx_ + size / 2.0 * angle.cos(), cy_ + size / 2.0 * angle.sin());
                if step == 0 {
                    path.move_to(p);
                } else {
                    path.line_to(p);
                }
            }
            if let Ok(path) = path.build() {
                window.paint_path(path, red);
            }
        } else if u32::from(status.tracking_mode)
            == mp_mavlink_dialects::all::CameraTrackingMode::CAMERA_TRACKING_MODE_RECTANGLE.0
            && ![status.rec_top_x, status.rec_top_y, status.rec_bottom_x, status.rec_bottom_y]
                .iter()
                .any(|v| v.is_nan())
        {
            let x = f64::from(status.rec_top_x.min(status.rec_bottom_x)) * fw;
            let y = f64::from(status.rec_top_y.min(status.rec_bottom_y)) * fh;
            let rw = f64::from((status.rec_top_x - status.rec_bottom_x).abs()) * fw;
            let rh = f64::from((status.rec_top_y - status.rec_bottom_y).abs()) * fh;
            rectangle(window, x, y, rw, rh);
        }
    }
}

/// A menu row, a form button: its id, its text, enabled or not, checked or not.
fn row_button(
    id: impl Into<SharedString>,
    text: impl Into<SharedString>,
    enabled: bool,
    checked: bool,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id: SharedString = id.into();
    let base = crate::probe::measured(id.to_string(), div())
        .id(id)
        .flex()
        .gap_2()
        .px_2()
        .py(px(2.0))
        .text_xs()
        .child(
            div()
                .w(px(10.0))
                .child(if checked { "\u{2713}" } else { "" }),
        )
        .child(text.into());
    if enabled {
        base.text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::SELECTION)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                on_click(this, window, cx);
                cx.notify();
            }))
            .into_any_element()
    } else {
        base.text_color(rgb(theme::DIM)).into_any_element()
    }
}

/// `VideoBoxContextMenu`, where the right button came up: the Designer's items with
/// `UITimer_Tick`'s enabling of the capture items, and the three `FlightData` adds as their
/// `Visible` has them. `// C#: Controls/GimbalVideoControl.Designer.cs:61-133;
/// Controls/GimbalVideoControl.cs:699-711; GCSViews/FlightData.cs:6534-6583`
fn context_menu(
    control: &GimbalVideoControl,
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let (x, y) = control.menu?;
    let video = &this.fly_data.gimbal_video;
    let camera = this.telemetry.camera().map(|(_, c)| c);
    let can_image = camera.as_ref().is_some_and(Camera::can_capture_image);
    let can_video = camera.as_ref().is_some_and(Camera::can_capture_video);
    let mut column = div()
        .flex()
        .flex_col()
        .py_1()
        .min_w(px(156.0))
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm();
    for (item, separator) in MenuItem::ALL {
        let visible = match item {
            MenuItem::MiniMap => video.items_visible.0,
            MenuItem::Swap => video.items_visible.1,
            MenuItem::Close => video.items_visible.2,
            _ => true,
        };
        if !visible {
            continue;
        }
        if separator {
            column = column.child(div().h(px(1.0)).my_1().bg(rgb(theme::BORDER)));
        }
        let enabled = match item {
            MenuItem::TakePicture => can_image,
            MenuItem::StartRecording | MenuItem::StopRecording => can_video,
            _ => true,
        };
        let checked = match item {
            MenuItem::YawLock => control.steering.yaw_lock,
            MenuItem::MiniMap => video.show_mini_map,
            _ => false,
        };
        column = column.child(row_button(
            item.id(),
            item.text(),
            enabled,
            checked,
            move |this, window, cx| this.gimbal_menu(item, window, cx),
            cx,
        ));
    }
    let viewport = window.viewport_size();
    let left = x.min(f32::from(viewport.width) - 160.0).max(0.0);
    let top = y.min(f32::from(viewport.height) - 300.0).max(0.0);
    let close = cx.listener(|this, _event: &MouseDownEvent, _window, cx| {
        if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
            control.menu = None;
        }
        cx.notify();
    });
    let close_right = cx.listener(|this, _event: &MouseDownEvent, _window, cx| {
        if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
            control.menu = None;
        }
        cx.notify();
    });
    Some(
        gpui::deferred(
            gpui::anchored().position(point(px(0.0), px(0.0))).child(
                div()
                    .relative()
                    .w(viewport.width)
                    .h(viewport.height)
                    .child(
                        div()
                            .id("fly-gimbal-menu-backdrop")
                            .absolute()
                            .inset_0()
                            .occlude()
                            .on_mouse_down(MouseButton::Left, close)
                            .on_mouse_down(MouseButton::Right, close_right),
                    )
                    .child(
                        crate::probe::measured("fly-gimbal-menu", div())
                            .absolute()
                            .left(px(left))
                            .top(px(top))
                            .occlude()
                            .child(column),
                    ),
            ),
        )
        .with_priority(3)
        .into_any_element(),
    )
}

/// A titled box over the window.
#[allow(clippy::too_many_arguments)] // where, how big, modal, what it holds and its close
fn form_frame(
    id: &'static str,
    title: &'static str,
    close_id: &'static str,
    at: (f32, f32),
    size_: (f32, f32),
    modal: bool,
    body: gpui::Div,
    window: &Window,
    on_close: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let frame = crate::probe::measured(id, div())
        .id(id)
        .absolute()
        .left(px(at.0))
        .top(px(at.1))
        .w(px(size_.0))
        .h(px(size_.1))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .overflow_hidden()
        .occlude()
        .child(
            div()
                .flex()
                .justify_between()
                .items_center()
                .px_2()
                .py_1()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .child(div().text_xs().text_color(rgb(theme::TEXT)).child(title))
                .child(row_button(close_id, "\u{2715}", true, false, on_close, cx)),
        )
        .child(body.flex_1().min_h(px(0.0)));
    let viewport = window.viewport_size();
    let mut layer = div().relative().w(viewport.width).h(viewport.height);
    if modal {
        layer = layer.child(
            div()
                .id(SharedString::from(format!("{id}-backdrop")))
                .absolute()
                .inset_0()
                .occlude(),
        );
    }
    gpui::deferred(
        gpui::anchored()
            .position(point(px(0.0), px(0.0)))
            .child(if modal {
                layer.child(frame)
            } else {
                div().child(frame)
            }),
    )
    .with_priority(if modal { 4 } else { 2 })
    .into_any_element()
}

/// The pop-out: `new Form { Text = "Gimbal Control", Size = (600, 400), CenterParent }` with the
/// control filling it, shown owned by the main window. Its close box closes it, and the control
/// with it. `// C#: GCSViews/FlightData.cs:6692-6711`
fn pop_out_window(
    control: &GimbalVideoControl,
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let viewport = window.viewport_size();
    let at = (
        ((f32::from(viewport.width) - 600.0) / 2.0).max(0.0),
        ((f32::from(viewport.height) - 400.0) / 2.0).max(0.0),
    );
    let body = div().relative().flex().child(video_box(control, this, cx));
    let frame = crate::probe::measured("fly-gimbal-popout", div())
        .id("fly-gimbal-popout")
        .absolute()
        .left(px(at.0))
        .top(px(at.1))
        .w(px(600.0))
        .h(px(400.0))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(
            div()
                .flex()
                .justify_between()
                .items_center()
                .px_2()
                .py_1()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child("Gimbal Control"),
                )
                .child(row_button(
                    "fly-gimbal-popout-close",
                    "\u{2715}",
                    true,
                    false,
                    |this, _window, _cx| this.fly_data.gimbal_video.form_closed(),
                    cx,
                )),
        )
        .child(body.flex_1().min_h(px(0.0)));
    gpui::deferred(
        gpui::anchored()
            .position(point(px(0.0), px(0.0)))
            .child(div().relative().child(frame)),
    )
    .with_priority(2)
    .into_any_element()
}

/// `GimbalControlSettingsForm`, 533 by 504, modal: a row per preference - its label, its binding
/// button and ❌, its up-down or its checkbox - then Save and Cancel. While a binding button
/// listens the keys go to it; "Key Binding Clash" asks over it.
/// `// C#: Controls/GimbalControlSettingsForm.cs:19-176; GimbalControlSettingsForm.Designer.cs`
fn settings_form(
    control: &GimbalVideoControl,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = control.settings.as_ref()?;
    let mut rows = div().flex().flex_col().gap_1().p_2();
    for (row, preference) in PREFERENCES.iter().enumerate() {
        let name = preference.name;
        let mut line = div().flex().items_center().gap_2().child(
            div()
                .w(px(200.0))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(preference.label),
        );
        match preference.control {
            ControlType::KeyBinding | ControlType::ClickBinding | ControlType::ModifierBinding => {
                let listening = form.listening.filter(|l| l.row == row);
                let text = match listening {
                    Some(Listening { shown: Some(m), .. }) if m != 0 => keys::modifiers_string(m),
                    Some(_) => match preference.control {
                        ControlType::ClickBinding => {
                            "Click a mouse button... (Esc to cancel)".to_owned()
                        }
                        ControlType::ModifierBinding => "Press modifier keys + Enter...".to_owned(),
                        _ => "Press a key... (Esc to cancel)".to_owned(),
                    },
                    None => form.preferences.text(row),
                };
                let id = format!("fly-gimbal-settings-{name}");
                let binding = crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .flex_1()
                    .px_2()
                    .py(px(2.0))
                    .rounded_sm()
                    .border_1()
                    .border_color(rgb(if listening.is_some() {
                        0xff_00_00
                    } else {
                        theme::ACCENT
                    }))
                    .bg(rgb(theme::ACTION))
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .cursor_pointer()
                    .child(text);
                let mut binding = binding;
                for (button, bit) in [
                    (MouseButton::Left, keys::MOUSE_LEFT),
                    (MouseButton::Right, keys::MOUSE_RIGHT),
                    (MouseButton::Middle, keys::MOUSE_MIDDLE),
                ] {
                    binding = binding.on_mouse_up(
                        button,
                        cx.listener(move |this, event: &MouseUpEvent, window, cx| {
                            if let Some(control) = this.fly_data.gimbal_video.control.as_mut()
                                && let Some(form) = control.settings.as_mut()
                            {
                                form.press_binding(row, bit, modifier_bits(event.modifiers));
                                control.form_focus.focus(window, cx);
                            }
                            cx.notify();
                        }),
                    );
                }
                line = line.child(binding).child(row_button(
                    format!("fly-gimbal-settings-{name}-clear"),
                    "\u{274c}",
                    true,
                    false,
                    move |this, _window, _cx| {
                        if let Some(form) = this
                            .fly_data
                            .gimbal_video
                            .control
                            .as_mut()
                            .and_then(|c| c.settings.as_mut())
                        {
                            form.clear(row);
                        }
                    },
                    cx,
                ));
            }
            ControlType::Decimal(..) => {
                let typing = form.typing.as_ref().filter(|(r, _)| *r == row);
                let shown: SharedString = match typing {
                    Some((_, field)) => format!("{}|", field.value()).into(),
                    None => form.preferences.text(row).into(),
                };
                let value_id = format!("fly-gimbal-settings-{name}");
                line = line
                    .child(
                        crate::probe::measured(value_id.clone(), div())
                            .id(SharedString::from(value_id))
                            .w(px(120.0))
                            .px_2()
                            .py(px(2.0))
                            .rounded_sm()
                            .border_1()
                            .border_color(rgb(if typing.is_some() {
                                theme::ACCENT
                            } else {
                                theme::BORDER
                            }))
                            .bg(rgb(theme::BG))
                            .text_xs()
                            .text_color(rgb(theme::TEXT))
                            .child(shown)
                            .on_click(cx.listener(move |this, _event, window, cx| {
                                if let Some(control) = this.fly_data.gimbal_video.control.as_mut()
                                    && let Some(form) = control.settings.as_mut()
                                {
                                    let mut field = TextField::new("");
                                    field.set(form.preferences.text(row));
                                    form.typing = Some((row, field));
                                    form.listening = None;
                                    control.form_focus.focus(window, cx);
                                }
                                cx.notify();
                            })),
                    )
                    .child(row_button(
                        format!("fly-gimbal-settings-{name}-up"),
                        "\u{25b2}",
                        true,
                        false,
                        move |this, _window, _cx| settings_step(this, row, true),
                        cx,
                    ))
                    .child(row_button(
                        format!("fly-gimbal-settings-{name}-down"),
                        "\u{25bc}",
                        true,
                        false,
                        move |this, _window, _cx| settings_step(this, row, false),
                        cx,
                    ));
            }
            ControlType::Checkbox => {
                let checked = matches!(form.preferences.values.get(row), Some(Value::Bool(true)));
                line = line.child(row_button(
                    format!("fly-gimbal-settings-{name}"),
                    "",
                    true,
                    checked,
                    move |this, _window, _cx| {
                        if let Some(form) = this
                            .fly_data
                            .gimbal_video
                            .control
                            .as_mut()
                            .and_then(|c| c.settings.as_mut())
                        {
                            form.toggle(row);
                        }
                    },
                    cx,
                ));
            }
        }
        rows = rows.child(line);
    }
    let buttons = div()
        .flex()
        .justify_end()
        .gap_2()
        .p_1()
        .child(row_button(
            "fly-gimbal-settings-save",
            "Save",
            true,
            false,
            settings_save,
            cx,
        ))
        .child(row_button(
            "fly-gimbal-settings-cancel",
            "Cancel",
            true,
            false,
            |this, _window, _cx| {
                if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
                    control.settings = None;
                }
            },
            cx,
        ));
    // `min_h(0)` on each flex column, or the thirty rows set the column's height and spill out
    // of the 504 px form past the buttons (seen on 2026-09-26: Save drawn below the window).
    // The C#'s `SettingsPanel.AutoScroll` is the rows' scroll here, the buttons docked under it.
    let mut body = div()
        .id("fly-gimbal-settings-body")
        .track_focus(&control.form_focus)
        .flex()
        .flex_col()
        .min_h(px(0.0))
        .child(
            div()
                .id("fly-gimbal-settings-rows")
                .flex_1()
                .min_h(px(0.0))
                .overflow_y_scroll()
                .child(rows),
        )
        .child(buttons)
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            settings_key(this, event);
            cx.notify();
        }));
    if let Some((_, _, message)) = &form.clash {
        body = body.child(
            crate::probe::measured("fly-gimbal-clash", div())
                .absolute()
                .top(px(120.0))
                .left(px(60.0))
                .w(px(400.0))
                .p_3()
                .flex()
                .flex_col()
                .gap_2()
                .bg(rgb(theme::BG))
                .border_1()
                .border_color(rgb(theme::WARN))
                .occlude()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::WARN))
                        .child("Key Binding Clash"),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(message.clone()),
                )
                .child(
                    div()
                        .flex()
                        .justify_end()
                        .gap_2()
                        .child(row_button(
                            "fly-gimbal-clash-yes",
                            "Yes",
                            true,
                            false,
                            |this, _w, _c| settings_clash(this, true),
                            cx,
                        ))
                        .child(row_button(
                            "fly-gimbal-clash-no",
                            "No",
                            true,
                            false,
                            |this, _w, _c| settings_clash(this, false),
                            cx,
                        )),
                ),
        );
    }
    let viewport = window.viewport_size();
    let at = (
        ((f32::from(viewport.width) - 533.0) / 2.0).max(0.0),
        ((f32::from(viewport.height) - 504.0) / 2.0).max(0.0),
    );
    Some(form_frame(
        "fly-gimbal-settings",
        "GimbalVideoControlSettings",
        "fly-gimbal-settings-close",
        at,
        (533.0, 504.0),
        true,
        div()
            .relative()
            .flex()
            .flex_col()
            .min_h(px(0.0))
            .child(body.flex_1().min_h(px(0.0))),
        window,
        |this, _window, _cx| {
            if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
                control.settings = None;
            }
        },
        cx,
    ))
}

/// An up-down's arrow.
fn settings_step(this: &mut MissionPlanner, row: usize, up: bool) {
    if let Some(control) = this.fly_data.gimbal_video.control.as_mut()
        && let Some(form) = control.settings.as_mut()
    {
        form.typing = None;
        form.step(row, up, &control.preferences);
    }
}

/// "Key Binding Clash" answered.
fn settings_clash(this: &mut MissionPlanner, yes: bool) {
    if let Some(form) = this
        .fly_data
        .gimbal_video
        .control
        .as_mut()
        .and_then(|c| c.settings.as_mut())
    {
        form.answer_clash(yes);
    }
}

/// A key in the settings form: to the up-down being typed into, or the listening button.
fn settings_key(this: &mut MissionPlanner, event: &KeyDownEvent) {
    let Some(control) = this.fly_data.gimbal_video.control.as_mut() else {
        return;
    };
    let opened = control.preferences.clone();
    let Some(form) = control.settings.as_mut() else {
        return;
    };
    if let Some((row, field)) = form.typing.as_mut() {
        let row = *row;
        match field.key(event) {
            KeyOutcome::Submitted => {
                let text = field.value().to_owned();
                form.typing = None;
                form.typed(row, &text, &opened);
            }
            KeyOutcome::Cancelled => form.typing = None,
            KeyOutcome::Changed | KeyOutcome::Ignored => {}
        }
        return;
    }
    if form.listening.is_some()
        && let Some(key) = keys::code(&event.keystroke.key)
    {
        form.listening_key(key, modifier_bits(event.keystroke.modifiers));
    }
}

/// Save: `DialogResult.OK`, the preferences saved as `GimbalControlPreferences` and read again.
/// `// C#: Controls/GimbalVideoControl.cs:755-763; GimbalControlSettingsForm.cs:166-170`
fn settings_save(
    this: &mut MissionPlanner,
    _window: &mut Window,
    _cx: &mut Context<MissionPlanner>,
) {
    let Some(control) = this.fly_data.gimbal_video.control.as_mut() else {
        return;
    };
    let Some(mut form) = control.settings.take() else {
        return;
    };
    // An up-down being typed into is validated as the form closes.
    if let Some((row, field)) = form.typing.take() {
        let opened = control.preferences.clone();
        form.typed(row, field.value(), &opened);
    }
    let json = form.preferences.to_json();
    // `loadPreferences`: read back from the setting.
    control.preferences = GimbalControlSettings::from_json(&json);
    this.persisted.set(PREFERENCES_KEY, json.as_str());
}

/// `VideoStreamSelector`, 430 by 93, modal: Detected Streams (each stream's name) and GStreamer
/// Pipeline; choosing a stream puts its pipeline in the box; Connect plays a pipeline that is
/// not empty and closes. `// C#: Controls/VideoStreamSelector.cs:19-53; VideoStreamSelector.Designer.cs`
fn stream_selector(
    control: &GimbalVideoControl,
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let selector = control.selector.as_ref()?;
    let streams = this.telemetry.video_streams();
    let mut list = div().flex().flex_wrap().gap_1().flex_1();
    for (index, (_, stream)) in streams.iter().enumerate() {
        let name = mp_link::camera::c_string(&stream.name);
        list = list.child(row_button(
            format!("fly-gimbal-stream-{index}"),
            name,
            true,
            selector.selected == Some(index),
            move |this, _window, _cx| {
                let streams = this.telemetry.video_streams();
                if let Some(selector) = this
                    .fly_data
                    .gimbal_video
                    .control
                    .as_mut()
                    .and_then(|c| c.selector.as_mut())
                    && let Some((_, stream)) = streams.get(index)
                {
                    selector.selected = Some(index);
                    selector
                        .pipeline
                        .set(mp_link::camera::gstreamer_pipeline(stream));
                }
            },
            cx,
        ));
    }
    let field = crate::textfield::text_field(
        "fly-gimbal-pipeline",
        &selector.pipeline,
        &control.form_focus,
        control.form_focus.is_focused(window),
        px(303.0),
        cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            if let Some(selector) = this
                .fly_data
                .gimbal_video
                .control
                .as_mut()
                .and_then(|c| c.selector.as_mut())
            {
                let _ = selector.pipeline.key(event);
            }
            cx.notify();
        }),
    );
    let label = |text: &'static str| {
        div()
            .w(px(103.0))
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(text)
    };
    let body = div()
        .flex()
        .flex_col()
        .gap_1()
        .p_2()
        .child(
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(label("Detected Streams"))
                .child(list),
        )
        .child(
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(label("GStreamer Pipeline"))
                .child(field),
        )
        .child(div().flex().justify_end().child(row_button(
            "fly-gimbal-connect",
            "Connect",
            true,
            false,
            |this, _window, _cx| {
                if let Some(control) = this.fly_data.gimbal_video.control.as_mut()
                    && let Some(selector) = control.selector.take()
                {
                    let pipeline = selector.pipeline.value().to_owned();
                    if !pipeline.is_empty() {
                        start_stream(control, &pipeline);
                    }
                }
            },
            cx,
        )));
    let viewport = window.viewport_size();
    let at = (
        ((f32::from(viewport.width) - 430.0) / 2.0).max(0.0),
        ((f32::from(viewport.height) - 150.0) / 2.0).max(0.0),
    );
    Some(form_frame(
        "fly-gimbal-selector",
        "VideoStreamSelector",
        "fly-gimbal-selector-close",
        at,
        (430.0, 150.0),
        true,
        body,
        window,
        |this, _window, _cx| {
            if let Some(control) = this.fly_data.gimbal_video.control.as_mut() {
                control.selector = None;
            }
        },
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_link::gimbal_manager::Quaternion;
    use mp_mavlink_dialects::all::{
        CameraFovStatus, GimbalManagerCapFlags, GimbalManagerInformation,
    };

    fn defaults() -> GimbalControlSettings {
        GimbalControlSettings::default()
    }

    #[test]
    fn the_defaults_are_the_constructors_and_show_as_the_buttons_do() {
        let p = defaults();
        assert_eq!(p.text(0), "A");
        assert_eq!(
            p.text(6),
            "Shift + ",
            "a modifiers-only button shows the modifiers"
        );
        assert_eq!(p.text(8), "Alt + F");
        assert_eq!(p.text(10), "None");
        assert_eq!(p.text(19), "Left Click");
        assert_eq!(p.text(20), "Ctrl + Left Click");
        assert_eq!(p.text(22), "1.0");
        assert_eq!(p.text(25), "1.00");
        assert_eq!(p.text(28), "true");
        assert!(p.hold_keys().contains(&keys::SHIFT_KEY));
        assert!(p.hold_keys().contains(&keys::CONTROL_KEY));
        assert!(p.press_keys().contains(&(keys::ALT | u32::from('F'))));
    }

    #[test]
    fn preferences_round_trip_as_json_net_writes_them() {
        let mut p = defaults();
        p.values[0] = Value::Key(u32::from('J'));
        p.values[19] = Value::Click(keys::SHIFT, keys::MOUSE_RIGHT);
        let json = p.to_json();
        assert!(json.starts_with("{\"SlewLeft\":74,"), "{json}");
        assert!(json.contains("\"MoveCameraToMouseLocation\":{\"Item1\":65536,\"Item2\":2097152}"));
        assert_eq!(GimbalControlSettings::from_json(&json), p);
        // Missing keys keep their defaults; a wrong type is Json.NET's failure, all defaults.
        let partial = GimbalControlSettings::from_json("{\"ZoomIn\":82}");
        assert_eq!(partial.key("ZoomIn"), 82);
        assert_eq!(partial.key("ZoomOut"), u32::from('Q'));
        assert_eq!(
            GimbalControlSettings::from_json("{\"ZoomIn\":\"x\"}"),
            defaults()
        );
        assert_eq!(GimbalControlSettings::from_json("not json"), defaults());
    }

    #[test]
    fn held_keys_slew_at_the_modifiers_speed_and_send_on_change() {
        let p = defaults();
        let mut s = Steering::default();
        let mut calls = Vec::new();
        assert!(s.key_down(u32::from('W'), 0, &p, &mut calls));
        assert_eq!(calls, vec![Call::SetRates(5.0, 0.0, false)]);
        calls.clear();
        // Shift held: the fast speed, and ShiftKey itself is a held key.
        assert!(s.key_down(keys::SHIFT_KEY, keys::SHIFT, &p, &mut calls));
        assert_eq!(calls, vec![Call::SetRates(25.0, 0.0, false)]);
        calls.clear();
        assert!(s.key_down(u32::from('A'), keys::SHIFT, &p, &mut calls));
        assert_eq!(calls, vec![Call::SetRates(25.0, -25.0, false)]);
        calls.clear();
        // Unchanged: nothing sent.
        s.held_keys(keys::SHIFT, &p, &mut calls);
        assert!(calls.is_empty());
        // Zoom: E in, Q out, the zoom speed.
        assert!(s.key_down(u32::from('E'), keys::SHIFT, &p, &mut calls));
        assert_eq!(calls, vec![Call::Zoom(1.0)]);
        calls.clear();
        // Let go of all: focus lost.
        s.focus_lost(0, &p, &mut calls);
        assert_eq!(
            calls,
            vec![Call::SetRates(0.0, 0.0, false), Call::Zoom(0.0)]
        );
        assert!(s.held.is_empty());
        // Unbound keys are not taken.
        assert!(!s.key_down(u32::from('Z'), 0, &p, &mut calls));
    }

    #[test]
    fn presses_act_on_their_binding_with_the_modifiers() {
        let p = defaults();
        let mut s = Steering::default();
        let mut calls = Vec::new();
        // F alone is not Take Picture; Alt+F is.
        assert!(!s.key_down(u32::from('F'), 0, &p, &mut calls));
        assert!(s.key_down(u32::from('F'), keys::ALT, &p, &mut calls));
        assert_eq!(calls, vec![Call::TakePicture]);
        calls.clear();
        // Alt+R toggles recording.
        s.key_down(u32::from('R'), keys::ALT, &p, &mut calls);
        s.key_down(u32::from('R'), keys::ALT, &p, &mut calls);
        assert_eq!(calls, vec![Call::StartRecording, Call::StopRecording]);
        calls.clear();
        // L toggles the lock and sends the last rates with it; N is Neutral; H is Home.
        s.key_down(u32::from('L'), 0, &p, &mut calls);
        assert!(s.yaw_lock);
        s.key_down(u32::from('N'), 0, &p, &mut calls);
        s.key_down(u32::from('H'), 0, &p, &mut calls);
        assert_eq!(
            calls,
            vec![Call::SetRates(0.0, 0.0, true), Call::Neutral, Call::Home]
        );
    }

    #[test]
    fn the_mouse_position_is_the_csharps_integer_arithmetic() {
        // 640 x 480 box, 1280 x 720 picture: letterboxed 640 x 360, 60 above and below.
        assert_eq!(
            mouse_position(320, 240, (640, 480), (1280, 720)),
            Some((0.0, 0.0))
        );
        assert_eq!(
            mouse_position(0, 60, (640, 480), (1280, 720)),
            Some((-1.0, -1.0))
        );
        // Above the picture clamps to its top.
        assert_eq!(
            mouse_position(640, 0, (640, 480), (1280, 720)),
            Some((1.0, -1.0))
        );
        // A box twice the picture's width: `x *= Width / imageWidth` doubles it.
        let (x, _) = mouse_position(300, 50, (400, 100), (100, 100)).unwrap();
        // imageWidth 100, x -= 150 -> 150, *= 4 -> 600, clamped to 100.
        assert!((x - 1.0).abs() < f64::EPSILON);
        assert_eq!(mouse_position(1, 1, (0, 0), (640, 480)), None);
    }

    #[test]
    fn the_mini_boxes_are_the_resize_handlers() {
        // 1000 x 600 panel, 640 x 480 picture: 300 x 180 -> width min(300, 240) = 240.
        assert_eq!(
            mini_video_box((1000, 600), (640, 480), 0),
            (760, 420, 240, 180)
        );
        assert_eq!(
            mini_video_box((1000, 600), (640, 480), 30),
            (730, 420, 240, 180)
        );
        assert_eq!(mini_map_box((1000, 600)), (700, 420, 300, 180));
    }

    /// What `gimbalVideoControl`'s getter leaves, without a window: in the panel, not yet
    /// visible, Mini map checked.
    fn made(video: &mut GimbalVideo) {
        video.placed = Some((Parent::Panel { fill: true }, false));
        video.set_show_mini_map(true);
    }

    #[test]
    fn full_mini_swap_pop_out_and_close_move_the_video_and_the_map_as_the_csharp() {
        let mut video = GimbalVideo::default();
        assert_eq!(video.shown(), Shown::Hidden);
        // No control: nothing moves.
        video.full_sized();
        assert_eq!(video.shown(), Shown::Hidden);
        made(&mut video);
        assert_eq!(video.shown(), Shown::Hidden);
        video.full_sized();
        assert_eq!((video.shown(), video.map_word()), (Shown::Full, "mini"));
        assert_eq!(video.items_visible, (true, true, true));
        // Mini map unchecked: the map and Swap with map hidden; checked, back.
        video.set_show_mini_map(false);
        assert_eq!(video.map_word(), "hidden");
        assert_eq!(video.items_visible, (true, false, true));
        video.set_show_mini_map(true);
        assert_eq!(video.map_word(), "mini");
        // Unchecked, Full Sized leaves the map hidden.
        video.set_show_mini_map(false);
        video.full_sized();
        assert_eq!(video.map_word(), "hidden");
        video.set_show_mini_map(true);
        // Swap: the mini video, and back.
        video.swap();
        assert_eq!((video.shown(), video.map_word()), (Shown::Mini, "full"));
        assert_eq!(video.items_visible, (false, true, true));
        video.swap();
        assert_eq!((video.shown(), video.map_word()), (Shown::Full, "mini"));
        // Pop Out: the map full sized, none of the items.
        video.pop_out();
        assert_eq!((video.shown(), video.map_word()), (Shown::PopOut, "full"));
        assert_eq!(video.items_visible, (false, false, false));
        // The window's close box disposes it; Close from the panel too, after Mini.
        video.form_closed();
        assert_eq!(video.shown(), Shown::Hidden);
        assert!(video.placed.is_none());
        made(&mut video);
        video.full_sized();
        video.close();
        assert_eq!((video.shown(), video.map_word()), (Shown::Hidden, "full"));
        assert_eq!(video.items_visible, (false, true, true));
        // Closing a form that is not open does nothing.
        made(&mut video);
        video.mini();
        video.form_closed();
        assert_eq!(video.shown(), Shown::Mini);
    }

    #[test]
    fn the_mini_boxes_follow_the_panel_and_keep_the_shape_they_were_made_with() {
        let mut video = GimbalVideo::default();
        made(&mut video);
        video.mini();
        let first = video.resize((1000.0, 600.0));
        assert_eq!(first.video, (760, 420, 240, 180));
        assert_eq!(first.map, (700, 420, 300, 180));
        // The same size: the boxes kept; a new size: made again.
        assert_eq!(video.resize((1000.0, 600.0)), first);
        assert_eq!(video.resize((500.0, 600.0)).video, (350, 488, 150, 112));
    }

    #[test]
    fn a_call_is_not_sent_without_the_capability() {
        let target = VehicleId::new(1, 1);
        let mut manager = GimbalManager::default();
        let _ = manager.discover();
        let mut selected = Selected {
            target: Some(target),
            manager: Some(manager.clone()),
            camera: Some(Camera::new(target)),
            ..Selected::default()
        };
        assert_eq!(
            messages(Call::SetRates(5.0, 0.0, false), &mut selected, 0, false),
            Err("not able")
        );
        // The camera's commands need no capability.
        let sent = messages(Call::TakePicture, &mut selected, 0, false).unwrap();
        assert_eq!(
            crate::fly::describe(&sent[0].0),
            "COMMAND_LONG MAV_CMD_IMAGE_START_CAPTURE 0,0,1,1,0,0,0"
        );
        let sent = messages(Call::TakePicture, &mut selected, 0, false).unwrap();
        assert!(
            crate::fly::describe(&sent[0].0).contains(",2,"),
            "the sequence counts"
        );
        manager.observe(&MavMessage::GimbalManagerInformation(
            GimbalManagerInformation {
                time_boot_ms: 0,
                cap_flags: GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS.0
                    | GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL.0,
                roll_min: 0.0,
                roll_max: 0.0,
                pitch_min: 0.0,
                pitch_max: 0.0,
                yaw_min: 0.0,
                yaw_max: 0.0,
                gimbal_device_id: 1,
            },
        ));
        selected.manager = Some(manager);
        let sent = messages(Call::SetRates(5.0, 0.0, false), &mut selected, 0, false).unwrap();
        assert!(
            crate::fly::describe(&sent[0].0)
                .starts_with("COMMAND_LONG MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW")
        );
        assert!(messages(Call::Neutral, &mut selected, 0, false).is_ok());
        assert_eq!(
            messages(Call::Retract, &mut selected, 0, false),
            Err("not able")
        );
        // No attitude reported: Click Pan/Tilt sends nothing.
        assert_eq!(
            messages(Call::PanTilt(0.5, 0.5), &mut selected, 0, false),
            Err("no attitude")
        );
        // No manager at all.
        let mut none = Selected::default();
        assert_eq!(
            messages(Call::Neutral, &mut none, 0, false),
            Err("no gimbal manager")
        );
    }

    #[test]
    fn the_image_point_is_placed_on_the_ground() {
        let target = VehicleId::new(1, 1);
        let mut camera = Camera::new(target);
        camera.start();
        let mut streams = std::collections::BTreeMap::new();
        // Camera 100 m up, looking at a point 100 m north.
        camera.observe(
            target,
            &MavMessage::CameraFovStatus(CameraFovStatus {
                time_boot_ms: 0,
                lat_camera: -353_632_610,
                lon_camera: 1_491_652_300,
                alt_camera: 684_000,
                lat_image: -353_623_617,
                lon_image: 1_491_652_300,
                alt_image: 584_000,
                q: [1.0, 0.0, 0.0, 0.0],
                hfov: 60.0,
                vfov: 40.0,
            }),
            &mut streams,
        );
        let (lat, lng, alt) = image_point_location(&camera, 0.0, 0.0).unwrap();
        assert!((lat + 35.362_361_7).abs() < 1e-9 && (lng - 149.165_23).abs() < 1e-9);
        assert!((alt - 584.0).abs() < 1e-9);
        // The top edge looks further away, at the image's height.
        let (far, _, alt) = image_point_location(&camera, 0.1, -1.0).unwrap();
        assert!(far > lat, "further north");
        assert!((alt - 584.0).abs() < 1e-9);
        let q = camera.image_point_rotation(0.0, 0.0);
        assert_eq!(q, Quaternion::default());
    }

    #[test]
    fn the_settings_form_binds_clears_and_asks_on_a_clash() {
        let opened = defaults();
        let mut form = SettingsForm::new(&opened);
        // Slew Left listens, and Escape cancels.
        form.press_binding(0, keys::MOUSE_LEFT, 0);
        assert_eq!(
            form.listening,
            Some(Listening {
                row: 0,
                shown: None
            })
        );
        form.listening_key(keys::ESCAPE, 0);
        assert!(form.listening.is_none());
        // J is free: taken.
        form.press_binding(0, keys::MOUSE_LEFT, 0);
        form.listening_key(u32::from('J'), 0);
        assert_eq!(form.preferences.text(0), "J");
        // D is Slew Right's: asked, and No keeps J.
        form.press_binding(0, keys::MOUSE_LEFT, 0);
        form.listening_key(u32::from('D'), 0);
        let (_, _, message) = form.clash.clone().expect("a clash");
        assert!(message.contains("- Slew Right"), "{message}");
        form.answer_clash(false);
        assert_eq!(form.preferences.text(0), "J");
        // A modifier binding waits for Enter.
        form.press_binding(6, keys::MOUSE_LEFT, 0);
        form.listening_key(keys::CONTROL_KEY, keys::CONTROL);
        assert!(form.listening.is_some());
        form.listening_key(keys::ENTER, keys::CONTROL | keys::ALT);
        assert_eq!(form.preferences.text(6), "Ctrl + Alt + ");
        // A click binding takes the second click with its modifiers.
        form.press_binding(21, keys::MOUSE_LEFT, 0);
        form.press_binding(21, keys::MOUSE_RIGHT, keys::SHIFT);
        assert_eq!(form.preferences.text(21), "Shift + Right Click");
        // The ❌: None.
        form.clear(8);
        assert_eq!(form.preferences.text(8), "None");
        // Up-downs step and clamp; typing rounds to its places.
        form.step(22, false, &opened);
        assert_eq!(form.preferences.text(22), "0.1");
        form.typed(25, "0.456", &opened);
        assert_eq!(form.preferences.text(25), "0.46");
        form.typed(25, "9", &opened);
        assert_eq!(form.preferences.text(25), "1.00");
        form.toggle(29);
        assert_eq!(form.preferences.text(29), "true");
    }

    #[test]
    fn keys_are_named_as_winforms_names_them() {
        assert_eq!(keys::code("a"), Some(65));
        assert_eq!(keys::code("5"), Some(53));
        assert_eq!(keys::code("f5"), Some(116));
        assert_eq!(keys::code("left"), Some(37));
        assert_eq!(keys::name(53), "D5");
        assert_eq!(keys::name(116), "F5");
        assert_eq!(
            keys::key_string(keys::CONTROL | keys::SHIFT | 65, false),
            "Shift + Ctrl + A"
        );
        assert_eq!(keys::click_string((0, 0)), "None");
    }
}
