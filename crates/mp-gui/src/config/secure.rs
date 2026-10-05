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

//! Secure: `GCSViews/ConfigurationView/ConfigSecureAP.cs`, the page Initial Setup lists at the top
//! while no vehicle is connected (`GCSViews/InitialSetup.cs:176-179`, `isDisConnected`), for
//! signing a bootloader and firmware with a key of one's own - ArduPilot's secure boot.
//!
//! What it shows, as its Designer places it in a 511 x 224 page: "Do Only Once", a group with
//! Generate Key; and "Files", a group with Private Key, BootLoader and Firmware, each with a text
//! box beside it - the public key, the bootloader's path and the firmware's path.
//!
//! What its four buttons do (`ConfigSecureAP.cs:30-111`, `ExtLibs/Utilities/SignedFW.cs`, ported
//! as [`mp_firmware::signed`]):
//!
//! * Generate Key makes an Ed25519 key pair - the page's key from then on - asks where to save it,
//!   writes the private key as PEM and as `_private_key.dat` and the public key as
//!   `_public_key.dat` beside it, shows the public key in base64 in the first box and says
//!   "Protect your private key, if lost there is no method to get it back.";
//! * Private Key opens a `.pem` or `.dat` and reads the key pair from it - PEM, or
//!   `PRIVATE_KEYV1:` and the base64 seed - and shows its public key;
//! * BootLoader opens a `.bin`, shows its path, writes ArduPilot's three public keys and the page's
//!   after its key table's descriptor, and saves `<name>-signed.bin` beside it;
//! * Firmware opens an `.apj`, shows its path, signs its image with the page's key, and saves
//!   `<name>-signed.apj` beside it.
//!
//! The page object keeps its key and its boxes while its screen lives, as the C#'s control does;
//! a new screen is a new page, with no key.
//!
//! The file dialogs are typed paths ([`crate::config::firmware::PathBox`]), as every file dialog
//! in this application is, captioned "Open" or "Save As" with the dialog's filter under the box;
//! an Open takes a file that exists, as `OpenFileDialog` checks. Save As appends `DefaultExt`
//! (`.pem`) to a name typed without an extension.
//!
//! What throws in a handler - no key loaded when signing (the C#'s `NullReferenceException`), a
//! file that is not a key or has no descriptor, a write refused - reaches Mission Planner's
//! unhandled-exception box; here it is said on the status line, by the owner's ruling of
//! 2026-09-25. The success report after Generate Key keeps its box.
//!
//! Divergences, each for a reason:
//!
//! * the text boxes show their text and take no typing: nothing reads what is typed into them
//!   (`txt_bl` and `txt_fwapj` are set from the dialog just before they are read), and this
//!   application's text box has no clipboard to copy the key out with;
//! * `SaveFileDialog`'s "already exists, replace it?" is not asked: the typed-path dialog writes
//!   the path given;
//! * the `.apj`'s image is deflated by another zlib than DotNetZip's, so its compressed bytes
//!   differ; the image they inflate to, and every other field, are the C#'s
//!   (`mp_firmware::signed`'s tests hold them to Mission Planner's own output).
//!
//! The colours are this application's.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::path::{Path, PathBuf};

use gpui::{AnyElement, Context, Div, FocusHandle, KeyDownEvent, Window, div, prelude::*, px, rgb};
use mp_firmware::signed::{self, KeyPair};

use crate::MissionPlanner;
use crate::config::firmware::{BoxIds, PathBox, path_box};
use crate::config::optional::{button, message_box, plain, text_box};
use crate::config::servo_output::Message;
use crate::plan::SAVE_FILE;
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::textfield::KeyOutcome;
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:178`
pub const TITLE: &str = "Secure";

/// What Generate Key says once the key is saved: a plain box.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:109`
pub const PROTECT: &str = "Protect your private key, if lost there is no method to get it back.";

/// Each dialog's filter, as its handler sets it.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:33, 56, 73, 100`
pub const KEY_FILTER: &str = "*.pem;*.dat|*.pem;*.dat";
/// The bootloader's.
pub const BL_FILTER: &str = "*.bin|*.bin";
/// The firmware's.
pub const APJ_FILTER: &str = "*.apj|*.apj";
/// The key's `SaveFileDialog`'s.
pub const PEM_FILTER: &str = "*.pem|*.pem";

/// A control's `Location` and `Size`.
pub type Place = (f32, f32, f32, f32);

/// `this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:136`
pub const PAGE_SIZE: (f32, f32) = (511.0, 224.0);

/// A group box: its name, text and place in the page.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Group {
    /// The Designer's name.
    pub name: &'static str,
    /// Its `Text`.
    pub text: &'static str,
    /// Its `Location` and `Size`.
    pub place: Place,
}

/// `groupBox5` and `groupBox4`.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:86-93, 129-135`
pub const GROUPS: [Group; 2] = [
    Group {
        name: "groupBox5",
        text: "Do Only Once",
        place: (59.0, 3.0, 335.0, 55.0),
    },
    Group {
        name: "groupBox4",
        text: "Files",
        place: (59.0, 64.0, 335.0, 120.0),
    },
];

/// A button: its name, text, the group it is in (an index into [`GROUPS`]), its place in the
/// group, and the handler the Designer wires to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Button {
    /// The Designer's name.
    pub name: &'static str,
    /// Its `Text`.
    pub text: &'static str,
    /// The group it is in.
    pub group: usize,
    /// Its `Location` and `Size` in the group.
    pub place: Place,
    /// Its `Click` handler.
    pub handler: &'static str,
}

/// The four buttons.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:50-81, 119-128`
pub const BUTTONS: [Button; 4] = [
    Button {
        name: "but_generatekey",
        text: "Generate Key",
        group: 0,
        place: (6.0, 19.0, 75.0, 23.0),
        handler: "but_generatekey_Click",
    },
    Button {
        name: "but_privkey",
        text: "Private Key",
        group: 1,
        place: (6.0, 19.0, 75.0, 23.0),
        handler: "but_privkey_Click",
    },
    Button {
        name: "but_bootloader",
        text: "BootLoader",
        group: 1,
        place: (6.0, 48.0, 75.0, 23.0),
        handler: "but_bootloader_Click",
    },
    Button {
        name: "but_firmware",
        text: "Firmware",
        group: 1,
        place: (6.0, 77.0, 75.0, 23.0),
        handler: "but_firmware_Click",
    },
];

/// The three text boxes in `groupBox4`: the public key, the bootloader's path, the firmware's.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:97-115`
pub const TEXT_BOXES: [(&str, Place); 3] = [
    ("txt_pubkey", (88.0, 21.0, 230.0, 20.0)),
    ("txt_bl", (88.0, 51.0, 230.0, 20.0)),
    ("txt_fwapj", (87.0, 80.0, 230.0, 20.0)),
];

/// Which handler a file dialog is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pick {
    /// Generate Key's `SaveFileDialog`.
    SaveKey,
    /// Private Key's `OpenFileDialog`.
    PrivateKey,
    /// BootLoader's.
    Bootloader,
    /// Firmware's.
    Firmware,
}

impl Pick {
    /// The dialog's filter.
    #[must_use]
    pub const fn filter(self) -> &'static str {
        match self {
            Self::SaveKey => PEM_FILTER,
            Self::PrivateKey => KEY_FILTER,
            Self::Bootloader => BL_FILTER,
            Self::Firmware => APJ_FILTER,
        }
    }
}

/// The page object: the key, the three boxes, the dialog open and the boxes to show.
#[derive(Debug, Default)]
pub struct Secure {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// `keyPair`.
    key: Option<KeyPair>,
    /// `txt_pubkey.Text`.
    pubkey: String,
    /// `txt_bl.Text`.
    bl: String,
    /// `txt_fwapj.Text`.
    fwapj: String,
    /// The file dialog open, and which handler it is for.
    dialog: Option<(Pick, PathBox)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// What a handler threw, for the status line.
    thrown: Option<String>,
    /// The files the last handler wrote.
    written: Vec<PathBuf>,
}

impl Secure {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Whether a key is loaded.
    #[must_use]
    pub const fn has_key(&self) -> bool {
        self.key.is_some()
    }

    /// The three boxes' text, in [`TEXT_BOXES`]' order.
    #[must_use]
    pub fn boxes(&self) -> [&str; 3] {
        [&self.pubkey, &self.bl, &self.fwapj]
    }

    /// The dialog open.
    #[must_use]
    pub fn dialog(&self) -> Option<(Pick, &PathBox)> {
        self.dialog.as_ref().map(|(pick, path)| (*pick, path))
    }

    /// The box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The files the last handler wrote.
    #[must_use]
    pub fn written(&self) -> &[PathBuf] {
        &self.written
    }

    /// What a handler threw since the last call, for the status line.
    pub const fn take_thrown(&mut self) -> Option<String> {
        self.thrown.take()
    }

    /// The page shown: a new page object for a new screen.
    pub fn show(&mut self, key: Key) {
        if self.made_for != Some(key) {
            *self = Self {
                made_for: Some(key),
                ..Self::default()
            };
        }
        self.active = true;
    }

    /// Another page chosen.
    pub fn hide(&mut self) {
        self.active = false;
        self.dialog = None;
    }

    /// Once a frame: a page object whose screen has gone is let go.
    pub fn tick(&mut self, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            *self = Self::default();
        }
    }

    /// A handler's exception: the C#'s unhandled-exception box, the status line here.
    fn threw(&mut self, why: impl Into<String>) {
        self.thrown = Some(why.into());
    }

    /// A button: Generate Key makes the key first, then asks where to save it; the others ask
    /// which file to open.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:30-35, 53-58, 70-75, 88-101`
    pub fn click(&mut self, pick: Pick) {
        if !self.active || self.dialog.is_some() {
            return;
        }
        if pick == Pick::SaveKey {
            match KeyPair::generate() {
                Ok(key) => self.key = Some(key),
                Err(why) => {
                    self.threw(why);
                    return;
                }
            }
        }
        let mut path = PathBox::new("", pick.filter());
        if pick == Pick::SaveKey {
            path.caption = SAVE_FILE;
        }
        self.dialog = Some((pick, path));
    }

    /// A key for the dialog: Enter is OK, Escape Cancel.
    pub fn path_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some((_, path)) = &mut self.dialog else {
            return false;
        };
        match path.field.key(event) {
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted => {
                self.path_done(true);
                true
            }
            KeyOutcome::Cancelled => {
                self.path_done(false);
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// Replaces the dialog's text as typing would.
    #[cfg(test)]
    pub fn type_path(&mut self, text: &str) {
        if let Some((_, path)) = &mut self.dialog {
            path.field.set(text);
        }
    }

    /// The dialog closed. Cancel does nothing; OK runs the rest of the handler - on a file that
    /// exists, for an Open.
    pub fn path_done(&mut self, ok: bool) {
        let Some((pick, path)) = self.dialog.take() else {
            return;
        };
        if !ok {
            return;
        }
        if pick == Pick::SaveKey {
            let typed = path.field.value().trim().to_owned();
            if !typed.is_empty() {
                self.save_key(&with_default_extension(&typed, ".pem"));
            }
            return;
        }
        let Some(file) = path.chosen() else {
            return;
        };
        self.written.clear();
        match pick {
            Pick::PrivateKey => self.read_key(&file),
            Pick::Bootloader => self.sign_bootloader(&file),
            Pick::Firmware => self.sign_firmware(&file),
            Pick::SaveKey => {}
        }
    }

    /// Generate Key, after the dialog: the PEM, the two `.dat`s named by replacing `.pem` in the
    /// path, the public key shown, and the warning.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:92-110`
    fn save_key(&mut self, file: &str) {
        self.written.clear();
        let Some(key) = self.key.clone() else {
            return;
        };
        let files = [
            (file.to_owned(), key.to_pem()),
            (file.replace(".pem", "_private_key.dat"), key.private_dat()),
            (file.replace(".pem", "_public_key.dat"), key.public_dat()),
        ];
        for (path, text) in files {
            if let Err(err) = std::fs::write(&path, text) {
                self.threw(format!("{path}: {err}"));
                return;
            }
            self.written.push(PathBuf::from(path));
        }
        self.pubkey = key.public_base64();
        self.messages.push_back(plain(PROTECT));
    }

    /// Private Key, after the dialog: the pair read, and its public key shown.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:34-50`
    fn read_key(&mut self, file: &Path) {
        let read = std::fs::read_to_string(file)
            .map_err(|err| err.to_string())
            .and_then(|text| KeyPair::from_key_file(&text));
        match read {
            Ok(key) => {
                self.pubkey = key.public_base64();
                self.key = Some(key);
            }
            Err(why) => self.threw(why),
        }
    }

    /// BootLoader, after the dialog: its path shown, then the keys written into a copy saved as
    /// `<name>-signed.bin`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:60-66`
    fn sign_bootloader(&mut self, file: &Path) {
        self.bl = file.display().to_string();
        let Some(key) = self.key.clone() else {
            self.threw("System.NullReferenceException: no key: Generate Key or Private Key first");
            return;
        };
        let signed = std::fs::read(file)
            .map_err(|err| err.to_string())
            .and_then(|bl| signed::create_signed_bl(&key, &bl));
        self.save_signed(signed, &signed::signed_path(file, ".bin"));
    }

    /// Firmware, after the dialog: its path shown, then the image signed into a copy saved as
    /// `<name>-signed.apj`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:77-85`
    fn sign_firmware(&mut self, file: &Path) {
        self.fwapj = file.display().to_string();
        let Some(key) = self.key.clone() else {
            self.threw("System.NullReferenceException: no key: Generate Key or Private Key first");
            return;
        };
        let signed = std::fs::read_to_string(file)
            .map_err(|err| err.to_string())
            .and_then(|apj| signed::create_signed_apj(&key, &apj));
        self.save_signed(signed, &signed::signed_path(file, ".apj"));
    }

    /// `File.WriteAllBytes` of what was signed, or the exception.
    fn save_signed(&mut self, signed: Result<Vec<u8>, String>, to: &Path) {
        match signed.and_then(|bytes| std::fs::write(to, bytes).map_err(|err| err.to_string())) {
            Ok(()) => self.written.push(to.to_path_buf()),
            Err(why) => self.threw(why),
        }
    }
}

/// `SaveFileDialog`'s `AddExtension`: `DefaultExt` after a name typed without an extension.
fn with_default_extension(typed: &str, extension: &str) -> String {
    if Path::new(typed).extension().is_some() {
        typed.to_owned()
    } else {
        format!("{typed}{extension}")
    }
}

/// Facts a UI test asserts on: the page's groups, buttons and text boxes, what the boxes show,
/// whether a key is loaded, the dialog, the box and the files written.
pub fn record_facts(secure: &Secure) {
    use crate::facts::record;
    record("config.secure.active", secure.is_active());
    record(
        "config.secure.groups",
        GROUPS
            .iter()
            .map(|group| group.text)
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "config.secure.buttons",
        BUTTONS
            .iter()
            .map(|button| button.text)
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.secure.buttons.enabled", secure.is_active());
    record(
        "config.secure.boxes",
        TEXT_BOXES
            .iter()
            .map(|(name, _)| *name)
            .collect::<Vec<_>>()
            .join(","),
    );
    let [pubkey, bl, fwapj] = secure.boxes();
    record(
        "config.secure.pubkey",
        if pubkey.is_empty() { "none" } else { pubkey },
    );
    record("config.secure.bl", if bl.is_empty() { "none" } else { bl });
    record(
        "config.secure.fwapj",
        if fwapj.is_empty() { "none" } else { fwapj },
    );
    record("config.secure.key", secure.has_key());
    record(
        "config.secure.dialog",
        secure.dialog().map_or("none", |(_, path)| path.caption),
    );
    record(
        "config.secure.message",
        secure
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.secure.written",
        secure
            .written()
            .iter()
            .map(|path| path.display().to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
}

/// An absolutely placed box.
fn at((x, y, width, height): Place) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// The handler each button runs.
const fn pick_of(name: &str) -> Pick {
    match name.as_bytes() {
        b"but_generatekey" => Pick::SaveKey,
        b"but_privkey" => Pick::PrivateKey,
        b"but_bootloader" => Pick::Bootloader,
        _ => Pick::Firmware,
    }
}

/// The page, as the Designer lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:29-143`
pub fn page(secure: &Secure, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut body = div().relative().w(px(PAGE_SIZE.0)).h(px(PAGE_SIZE.1));
    for (index, group) in GROUPS.iter().enumerate() {
        let mut groupbox = at(group.place)
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .child(
                div()
                    .absolute()
                    .left(px(6.0))
                    .top(px(-8.0))
                    .px_1()
                    .bg(rgb(theme::PANEL))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(group.text),
            );
        // `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:30-111`
        for button_spec in BUTTONS.iter().filter(|button| button.group == index) {
            let pick = pick_of(button_spec.name);
            let id = match pick {
                Pick::SaveKey => "secure-but_generatekey",
                Pick::PrivateKey => "secure-but_privkey",
                Pick::Bootloader => "secure-but_bootloader",
                Pick::Firmware => "secure-but_firmware",
            };
            groupbox = groupbox.child(button(
                id,
                button_spec.text,
                button_spec.place,
                secure.is_active(),
                move |this, window, cx| {
                    this.secure.click(pick);
                    this.secure_focus.focus(window, cx);
                },
                cx,
            ));
        }
        if index == 1 {
            let ids = ["secure-txt_pubkey", "secure-txt_bl", "secure-txt_fwapj"];
            for ((id, (_, place)), text) in ids.into_iter().zip(TEXT_BOXES).zip(secure.boxes()) {
                groupbox = groupbox.child(text_box(
                    id,
                    text,
                    None,
                    false,
                    true,
                    place,
                    |_| {},
                    |_, _| false,
                    cx,
                ));
            }
        }
        body = body.child(groupbox);
    }
    panel(TITLE, body).into_any_element()
}

/// This page's dialog ids.
const IDS: BoxIds = BoxIds {
    question: "secure-question",
    yes: "secure-question-yes",
    no: "secure-question-no",
    message: "secure-message",
    ok: "secure-message-ok",
    path: "secure-path",
    path_value: "secure-path-value",
    path_ok: "secure-path-ok",
    path_cancel: "secure-path-cancel",
};

/// The dialog or box showing, over the whole window.
pub fn overlay(
    secure: &Secure,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(message) = secure.message() {
        return Some(message_box(
            IDS.message,
            IDS.ok,
            message,
            window,
            |this| this.secure.dismiss_message(),
            cx,
        ));
    }
    let (_, path) = secure.dialog()?;
    Some(path_box(
        IDS,
        path,
        handle,
        window,
        |this, event| this.secure.path_key(event),
        |this, ok| this.secure.path_done(ok),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Designer's controls, places, texts and handlers, read from the tree when it is here:
    /// this page's Designer sets them in code rather than in the `.resx`.
    #[test]
    fn the_controls_are_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let has = |line: &str| {
            assert!(designer.contains(line), "the Designer has no `{line}`");
        };
        let point = |name: &str, (x, y, w, h): Place| {
            has(&format!(
                "this.{name}.Location = new System.Drawing.Point({x}, {y});"
            ));
            has(&format!(
                "this.{name}.Size = new System.Drawing.Size({w}, {h});"
            ));
        };
        for group in GROUPS {
            point(group.name, group.place);
            has(&format!("this.{}.Text = \"{}\";", group.name, group.text));
        }
        for button in BUTTONS {
            point(button.name, button.place);
            has(&format!("this.{}.Text = \"{}\";", button.name, button.text));
            has(&format!(
                "this.{}.Click += new System.EventHandler(this.{});",
                button.name, button.handler
            ));
            let parent = GROUPS.get(button.group).map_or("", |group| group.name);
            has(&format!(
                "this.{parent}.Controls.Add(this.{});",
                button.name
            ));
        }
        for (name, place) in TEXT_BOXES {
            point(name, place);
            has(&format!("this.groupBox4.Controls.Add(this.{name});"));
        }
        has(&format!(
            "this.Size = new System.Drawing.Size({}, {});",
            PAGE_SIZE.0, PAGE_SIZE.1
        ));
        assert_eq!(designer.matches(" += new ").count(), BUTTONS.len());
    }

    /// Each handler's dialog filter and the words it shows are the C#'s.
    #[test]
    fn the_filters_and_words_are_the_csharps() {
        let Some(source) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigSecureAP.cs")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for filter in [KEY_FILTER, BL_FILTER, APJ_FILTER, PEM_FILTER] {
            assert!(
                source.contains(&format!("Filter = \"{filter}\"")),
                "{filter}"
            );
        }
        assert!(source.contains(&format!("CustomMessageBox.Show(\"{PROTECT}\")")));
        for button in BUTTONS {
            let pick = pick_of(button.name);
            let start = source
                .find(&format!("void {}(", button.handler))
                .expect("the handler");
            let body = source.get(start..).unwrap_or_default();
            let end = body.find("\n        }").expect("its end");
            assert!(
                body.get(..end).unwrap_or_default().contains(pick.filter()),
                "{} opens its own dialog",
                button.handler
            );
        }
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-gui-secure-{}-{name}", mp_os::process_id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    fn shown() -> Secure {
        let mut secure = Secure::default();
        secure.show(Key::of(&TelemetryView::disconnected("test")));
        secure
    }

    /// Runs one handler through its dialog.
    fn pick(secure: &mut Secure, pick: Pick, path: &Path) {
        secure.click(pick);
        assert_eq!(secure.dialog().map(|(which, _)| which), Some(pick));
        secure.type_path(&path.display().to_string());
        secure.path_done(true);
        assert!(secure.dialog().is_none());
    }

    /// Generate Key: the three files beside each other, the public key shown and the warning;
    /// each file then read back by Private Key gives the same key.
    #[test]
    fn generate_key_saves_three_files_that_read_back() {
        let dir = scratch("generate");
        let mut secure = shown();
        secure.click(Pick::SaveKey);
        assert!(secure.has_key(), "the key is made before the dialog");
        assert_eq!(
            secure.dialog().map(|(_, path)| path.caption),
            Some(SAVE_FILE)
        );
        secure.type_path(&dir.join("mine").display().to_string());
        secure.path_done(true);
        let pem = dir.join("mine.pem");
        let private = dir.join("mine_private_key.dat");
        let public = dir.join("mine_public_key.dat");
        assert_eq!(
            secure.written(),
            [pem.clone(), private.clone(), public.clone()]
        );
        let pubkey = secure.boxes()[0].to_owned();
        assert_eq!(pubkey.len(), 44, "32 bytes in base64");
        assert_eq!(
            std::fs::read_to_string(&public).ok(),
            Some(format!("PUBLIC_KEYV1:{pubkey}"))
        );
        assert!(
            std::fs::read_to_string(&pem)
                .is_ok_and(|text| text.starts_with("-----BEGIN PRIVATE KEY-----"))
        );
        assert_eq!(secure.message().map(|m| m.text.as_str()), Some(PROTECT));
        secure.dismiss_message();

        for file in [&private, &pem] {
            let mut reader = shown();
            pick(&mut reader, Pick::PrivateKey, file);
            assert!(reader.has_key());
            assert_eq!(reader.boxes()[0], pubkey, "{}", file.display());
            assert_eq!(reader.take_thrown(), None);
        }
        let _ = std::fs::remove_dir_all(dir);
    }

    /// BootLoader and Firmware sign with the key loaded and save beside the file, showing its
    /// path; with no key they throw, which is a status line.
    #[test]
    fn the_bootloader_and_firmware_are_signed_beside_themselves() {
        let dir = scratch("sign");
        let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/secure");
        let bl = dir.join("bl.bin");
        let apj = dir.join("fw.apj");
        std::fs::copy(fixtures.join("bl.bin"), &bl).expect("copied");
        std::fs::copy(fixtures.join("unsigned.apj"), &apj).expect("copied");

        let mut secure = shown();
        pick(&mut secure, Pick::Bootloader, &bl);
        assert_eq!(secure.boxes()[1], bl.display().to_string());
        assert!(
            secure
                .take_thrown()
                .is_some_and(|why| why.contains("NullReferenceException"))
        );
        assert!(secure.written().is_empty());

        let seed: [u8; 32] = std::array::from_fn(|i| u8::try_from(i * 7 + 1).unwrap_or(0));
        let key = KeyPair::from_seed(&seed).expect("a pair");
        let dat = dir.join("key.dat");
        std::fs::write(&dat, key.private_dat()).expect("written");
        pick(&mut secure, Pick::PrivateKey, &dat);

        pick(&mut secure, Pick::Bootloader, &bl);
        assert_eq!(secure.take_thrown(), None);
        let signed_bl = dir.join("bl-signed.bin");
        assert_eq!(secure.written(), std::slice::from_ref(&signed_bl));
        assert_eq!(
            std::fs::read(&signed_bl).ok(),
            std::fs::read(fixtures.join("bl-signed-by-mp.bin")).ok(),
            "Mission Planner's own output"
        );

        pick(&mut secure, Pick::Firmware, &apj);
        assert_eq!(secure.take_thrown(), None);
        assert_eq!(secure.boxes()[2], apj.display().to_string());
        let signed_apj = dir.join("fw-signed.apj");
        assert_eq!(secure.written(), std::slice::from_ref(&signed_apj));
        assert!(
            std::fs::read_to_string(&signed_apj)
                .is_ok_and(|text| text.contains("\"signed_firmware\": true"))
        );

        // A bin with no key table throws.
        let empty = dir.join("empty.bin");
        std::fs::write(&empty, [0u8; 32]).expect("written");
        pick(&mut secure, Pick::Bootloader, &empty);
        assert_eq!(
            secure.take_thrown().as_deref(),
            Some("Invalid bin, descriptor not found")
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Cancel, and an Open on a file that is not there, do nothing.
    #[test]
    fn cancel_or_a_missing_file_does_nothing() {
        let mut secure = shown();
        secure.click(Pick::Bootloader);
        secure.type_path("/nonexistent/bl.bin");
        secure.path_done(true);
        assert_eq!(
            secure.boxes()[1],
            "",
            "OpenFileDialog checks the file exists"
        );
        secure.click(Pick::SaveKey);
        secure.path_done(false);
        assert!(secure.written().is_empty());
        assert!(secure.message().is_none());
        assert!(secure.has_key(), "the key was made before the dialog");
    }

    #[test]
    fn a_name_without_an_extension_gets_pem() {
        assert_eq!(with_default_extension("/k/mine", ".pem"), "/k/mine.pem");
        assert_eq!(with_default_extension("/k/mine.pem", ".pem"), "/k/mine.pem");
        assert_eq!(with_default_extension("/k/mine.key", ".pem"), "/k/mine.key");
    }

    /// The page object lives with its screen: a new one has no key and empty boxes.
    #[test]
    fn a_new_screen_is_a_new_page() {
        let mut secure = shown();
        secure.click(Pick::SaveKey);
        secure.path_done(false);
        assert!(secure.has_key());
        secure.hide();
        let view = TelemetryView::disconnected("test");
        secure.tick(&view, true);
        assert!(secure.has_key(), "hidden, the page keeps its key");
        secure.tick(&view, false);
        assert!(!secure.has_key(), "the screen left, the page is let go");
    }

    /// Every fact the script asserts on is recorded here, and each control it clicks is drawn.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-secure.gui");
        let source = include_str!("secure.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.secure.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("secure-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id}");
                }
                _ => {}
            }
        }
        assert!(facts >= 10, "{facts} facts");
    }
}
