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

//! Mission Planner's window-wide keys, `MainV2.ProcessCmdKey`: the F keys that switch screens
//! and connect, and the Control keys that open the tool forms.
//!
//! WinForms gives `ProcessCmdKey` a key after the focused control and its parents have had it,
//! so a page's own keys (the parameter grid's Ctrl+S and F2, Install Firmware's Ctrl+Q) come
//! first; here the main window's root takes a key on its way out, after every element inside
//! has had it and none took it.
//!
//! * `if (ConfigTerminal.SSHTerminal) return false;` has nothing to test: the Terminal page and
//!   its SSH session are out of scope (the ledger's ConfigTerminal.cs and SSHTerminal.cs rows).
//! * A key compares as `keyData ==` does: the F keys with no modifier, the letters with Control
//!   and nothing else. Control on every platform, as Mission Planner under mono on a Mac.
//!
//! The forms the Control keys open that nothing else opens are ported under this module: Ctrl+X's
//! map cache (`gmap_cache`), Ctrl+J's DevOps (`devops_ui`), Ctrl+W's propagation settings
//! (`propagation_settings`), and Ctrl+Z's camera test (`camera`). Ctrl+G's NMEA output and Ctrl+L's
//! spectrogram are CONFIG > Advanced's windows (`config::nmea_output`, `config::spectrogram`),
//! opened as its buttons open them and drawn here over every screen but the two that draw the
//! Advanced page's windows already.
//!
//! `// C#: MainV2.cs:4067-4208`

mod camera;
mod devops_ui;
mod gmap_cache;
mod propagation_settings;

use gpui::{AnyElement, Context, Keystroke, Window};

use crate::{MissionPlanner, Screen};

/// `FlightData.ProcessCmdKey`'s Control and a digit: the tab's place, Ctrl+1 the first and
/// Ctrl+0 the tenth. `// C#: GCSViews/FlightData.cs:865-916`
#[must_use]
pub(crate) fn fly_tab_key(keystroke: &Keystroke) -> Option<usize> {
    let modifiers = &keystroke.modifiers;
    if !modifiers.control || modifiers.alt || modifiers.shift || modifiers.platform {
        return None;
    }
    match keystroke.key.as_str() {
        "0" => Some(9),
        digit => digit
            .parse::<usize>()
            .ok()
            .filter(|n| (1..=9).contains(n))
            .map(|n| n - 1),
    }
}

/// `FlightData.ProcessCmdKey`'s keys for a log being played.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PlaybackKey {
    /// `Keys.Space`: `BUT_playlog_Click`, with a log loaded.
    PlayPause,
    /// `Keys.Subtract`, the keypad's -: `LogPlayBackSpeed` down.
    Slower,
    /// `Keys.Add`, the keypad's +: `LogPlayBackSpeed` up.
    Faster,
}

/// Whether gpui names the keypad's - and + apart from the main keyboard's: on Linux it gives them
/// the keysym's name less `KP_`, `subtract` and `add`; on Windows, macOS and in a browser it gives
/// the character, `-` and `+`, the main keyboard's too, so there those are taken as the keypad's.
const KEYPAD_NAMED: bool = cfg!(target_os = "linux");

/// The playback key `keystroke` is, compared as `keyData ==` does: the key alone.
/// `// C#: GCSViews/FlightData.cs:918-941`
#[must_use]
pub(crate) fn playback_key(keystroke: &Keystroke, keypad_named: bool) -> Option<PlaybackKey> {
    let modifiers = &keystroke.modifiers;
    if modifiers.control || modifiers.alt || modifiers.shift || modifiers.platform {
        return None;
    }
    match keystroke.key.as_str() {
        "space" => Some(PlaybackKey::PlayPause),
        "subtract" => Some(PlaybackKey::Slower),
        "add" => Some(PlaybackKey::Faster),
        "-" if !keypad_named => Some(PlaybackKey::Slower),
        "+" if !keypad_named => Some(PlaybackKey::Faster),
        _ => None,
    }
}

/// The forms `ProcessCmdKey` opens that nothing else does, the camera test it runs, and the
/// keyboard focus of their boxes.
pub(crate) struct KeyForms {
    /// Ctrl+X: `GMAPCache`.
    map_cache: gmap_cache::MapCache,
    /// Ctrl+J: `DevopsUI`.
    devops: devops_ui::Devops,
    devops_focus: devops_ui::FocusHandles,
    /// Ctrl+W: `PropagationSettings`, and its number being typed into.
    propagation: propagation_settings::Propagation,
    propagation_focus: gpui::FocusHandle,
    /// Ctrl+Z: `Camera.test`.
    camera: camera::CameraTest,
}

impl KeyForms {
    /// None open.
    pub(crate) fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            map_cache: gmap_cache::MapCache::default(),
            devops: devops_ui::Devops::default(),
            devops_focus: devops_ui::FocusHandles::new(cx),
            propagation: propagation_settings::Propagation::default(),
            propagation_focus: cx.focus_handle(),
            camera: camera::CameraTest::default(),
        }
    }
}

/// Facts a UI test asserts on: each form's, and the camera test's.
pub(crate) fn record_facts(forms: &KeyForms, settings: &crate::settings::Persisted) {
    gmap_cache::record_facts(&forms.map_cache);
    devops_ui::record_facts(&forms.devops);
    propagation_settings::record_facts(&forms.propagation, settings);
    camera::record_facts(&forms.camera);
}

/// Whether this module draws Ctrl+G's and Ctrl+L's windows over `screen`: every screen but SETUP,
/// whose Advanced page opens them, and EXPERIMENTAL, which opens them as that page does - both
/// draw them with that page's other windows (`extra_setup_overlay`), and twice would be two.
#[must_use]
const fn draws_advanced_windows(screen: Screen) -> bool {
    !matches!(screen, Screen::Setup | Screen::Experimental)
}

/// The forms over the window, whichever screen shows: the first open of Ctrl+X's, Ctrl+W's and
/// Ctrl+J's; then Ctrl+L's spectrogram and Ctrl+G's NMEA output, which SETUP and EXPERIMENTAL
/// draw with the Advanced page's other windows (`extra_setup_overlay`), so here only elsewhere.
/// `// C#: MainV2.cs:4124-4152, 4195-4200`
pub(crate) fn overlay(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let forms = &this.key_forms;
    gmap_cache::overlay(&forms.map_cache, window, cx)
        .or_else(|| {
            propagation_settings::overlay(&forms.propagation, &forms.propagation_focus, window, cx)
        })
        .or_else(|| devops_ui::overlay(&forms.devops, &forms.devops_focus, window, cx))
        .or_else(|| {
            if !draws_advanced_windows(this.screen) {
                return None;
            }
            crate::config::spectrogram::overlay(
                &this.extra.spectrogram,
                &this.extra_focus.spectrogram,
                window,
                cx,
            )
            .or_else(|| {
                crate::config::nmea_output::overlay(
                    &this.extra.nmea_output,
                    &this.extra_focus.nmea_prompt,
                    window,
                    cx,
                )
            })
        })
}

/// What a form's box does when it is used: clicked into, a key while it has the keyboard, and,
/// for a number, an arrow (up when true).
struct BoxHandlers<B, K, S> {
    begin: B,
    key: K,
    step: S,
}

/// A form's box at its place, `(x, y, width, height)`: its text - typed into, with a caret, while
/// it has the keyboard - and for a number its arrows, `<id>-up` and `<id>-down`; dimmed and inert
/// while disabled. While it has the keyboard every key is its own, kept from the main window: the
/// C#'s forms are windows of their own, whose keys `MainV2.ProcessCmdKey` never sees.
#[allow(clippy::too_many_arguments)]
fn form_box<B, K, S>(
    id: &'static str,
    text: String,
    (x, y, width, height): (f32, f32, f32, f32),
    arrows: bool,
    editing: bool,
    enabled: bool,
    handle: &gpui::FocusHandle,
    handlers: BoxHandlers<B, K, S>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement
where
    B: Fn(&mut MissionPlanner) + 'static,
    K: Fn(&mut MissionPlanner, &gpui::KeyDownEvent) -> bool + 'static,
    S: Fn(&mut MissionPlanner, bool) + Clone + 'static,
{
    use crate::ui::theme;
    use gpui::{SharedString, div, prelude::*, px, rgb};

    let focused = enabled && editing && handle.is_focused(window);
    let body = crate::probe::measured(id, div())
        .id(id)
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text)
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let body = if !enabled {
        body
    } else if editing {
        let key = handlers.key;
        body.track_focus(handle)
            .key_context("TextField")
            .cursor_text()
            .on_key_down(
                cx.listener(move |this, event: &gpui::KeyDownEvent, _window, cx| {
                    cx.stop_propagation();
                    if key(this, event) {
                        cx.notify();
                    }
                }),
            )
    } else {
        let begin = handlers.begin;
        let handle = handle.clone();
        body.cursor_text()
            .on_click(cx.listener(move |this, _event, window, cx| {
                begin(this);
                handle.focus(window, cx);
                cx.notify();
            }))
    };
    let mut boxed = div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        .child(body);
    if arrows {
        let mut column = div()
            .w(px(14.0))
            .h_full()
            .flex()
            .flex_col()
            .border_l_1()
            .border_color(rgb(theme::BORDER));
        for (suffix, glyph, up) in [("up", "\u{25b2}", true), ("down", "\u{25bc}", false)] {
            let arrow_id = format!("{id}-{suffix}");
            let base = crate::probe::measured(arrow_id.clone(), div())
                .id(SharedString::from(arrow_id))
                .flex_1()
                .flex()
                .items_center()
                .justify_center()
                .text_size(px(7.0));
            column = column.child(if enabled {
                let step = handlers.step.clone();
                base.text_color(rgb(theme::TEXT))
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(glyph)
                    .on_click(cx.listener(move |this, _event, window, cx| {
                        window.blur(cx);
                        step(this, up);
                        cx.notify();
                    }))
                    .into_any_element()
            } else {
                base.text_color(rgb(theme::DIM))
                    .child(glyph)
                    .into_any_element()
            });
        }
        boxed = boxed.child(column);
    }
    boxed.into_any_element()
}

/// Ctrl+Y's message once the command has been answered or refused, and when it went unanswered
/// or there was no link to send it on (`doCommand`'s exception). On the status line, as the
/// owner ruled for such boxes. `// C#: MainV2.cs:4158-4174`
const STORAGE_WRITE_DONE: &str = "Done MAV_ACTION_STORAGE_WRITE";

/// What each of `ProcessCmdKey`'s keys does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CmdKey {
    /// F12: `MenuConnect_Click`.
    Connect,
    /// F2: `MenuFlightData_Click`.
    FlightData,
    /// F3: `MenuFlightPlanner_Click`.
    FlightPlanner,
    /// F4: `MenuTuning_Click`.
    Tuning,
    /// F5: `comPort.getParamList()` and the current screen shown again.
    RefreshParams,
    /// Ctrl+F: `new temp().Show()`.
    Temp,
    /// Ctrl+P: `new PluginUI().Show()`.
    PluginManager,
    /// Ctrl+G: `new SerialOutputNMEA().Show()`.
    NmeaOut,
    /// Ctrl+X: `new GMAPCache().ShowUserControl()`.
    MapCache,
    /// Ctrl+L: `new SpectrogramUI().Show()`.
    Spectrogram,
    /// Ctrl+W: `new PropagationSettings().Show()`.
    Propagation,
    /// Ctrl+Z: `new Camera().test(comPort)`.
    CameraTest,
    /// Ctrl+T: `comPort.Open(false)`.
    OverrideConnect,
    /// Ctrl+Y: `PREFLIGHT_STORAGE` 1, then "Done MAV_ACTION_STORAGE_WRITE".
    StorageWrite,
    /// Ctrl+J: `new DevopsUI().ShowUserControl()`.
    Devops,
}

/// The key `keystroke` is to `ProcessCmdKey`, if any.
/// `// C#: MainV2.cs:4073-4200`
#[must_use]
pub(crate) fn cmd_key(keystroke: &Keystroke) -> Option<CmdKey> {
    let modifiers = &keystroke.modifiers;
    // The fn key a laptop's F row may report is not one `Keys` has.
    let others = modifiers.alt || modifiers.shift || modifiers.platform;
    let key = keystroke.key.to_ascii_lowercase();
    if !modifiers.control && !others {
        return match key.as_str() {
            "f12" => Some(CmdKey::Connect),
            "f2" => Some(CmdKey::FlightData),
            "f3" => Some(CmdKey::FlightPlanner),
            "f4" => Some(CmdKey::Tuning),
            "f5" => Some(CmdKey::RefreshParams),
            _ => None,
        };
    }
    if !modifiers.control || others {
        return None;
    }
    match key.as_str() {
        "f" => Some(CmdKey::Temp),
        "p" => Some(CmdKey::PluginManager),
        "g" => Some(CmdKey::NmeaOut),
        "x" => Some(CmdKey::MapCache),
        "l" => Some(CmdKey::Spectrogram),
        "w" => Some(CmdKey::Propagation),
        "z" => Some(CmdKey::CameraTest),
        "t" => Some(CmdKey::OverrideConnect),
        "y" => Some(CmdKey::StorageWrite),
        "j" => Some(CmdKey::Devops),
        _ => None,
    }
}

impl MissionPlanner {
    /// `ProcessCmdKey`: whether the key was one of its, and taken. The flight screen's own come
    /// first, as its control's `ProcessCmdKey` runs before the form's.
    /// `// C#: MainV2.cs:4067-4208; GCSViews/FlightData.cs:865-943`
    pub(crate) fn process_cmd_key(
        &mut self,
        keystroke: &Keystroke,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) -> bool {
        if self.screen == Screen::Fly
            && let Some(index) = fly_tab_key(keystroke)
        {
            self.fly_pages.select_index(index);
            return true;
        }
        // Only with the window itself holding the keyboard: a space or a minus typed into a box
        // reaches the window too, and is the box's.
        if self.screen == Screen::Fly
            && self.root_focus.is_focused(window)
            && let Some(key) = playback_key(keystroke, KEYPAD_NAMED)
        {
            return self.run_playback_key(key, cx);
        }
        let Some(key) = cmd_key(keystroke) else {
            return false;
        };
        match key {
            CmdKey::Connect => self.connect_clicked(window, cx),
            CmdKey::FlightData => self.choose_screen(Screen::Fly),
            CmdKey::FlightPlanner => self.choose_screen(Screen::Plan),
            // `MenuTuning_Click`: CONFIG/TUNING, as its tab shows it.
            CmdKey::Tuning => self.choose_screen(Screen::Config),
            CmdKey::RefreshParams => self.refresh_param_list(),
            // The owner's (2026-10-04): the PLUGINS tab, where the C# opens a form of its own.
            CmdKey::PluginManager => self.choose_screen(Screen::Plugins),
            CmdKey::OverrideConnect => self.override_connect(window, cx),
            CmdKey::StorageWrite => self.storage_write(),
            // The owner's (2026-10-04): the EXPERIMENTAL tab, where the C# opens the temp form.
            CmdKey::Temp => self.choose_screen(Screen::Experimental),
            // `new SerialOutputNMEA().Show()` and `new SpectrogramUI().Show()`: the Advanced
            // page's NMEA and Spectrogram buttons' own. `// C#: MainV2.cs:4124-4130, 4138-4145`
            CmdKey::NmeaOut => {
                self.open_advanced_tool("BUT_outputnmea", window, cx);
            }
            CmdKey::Spectrogram => {
                self.open_advanced_tool("BUT_spect", window, cx);
            }
            // `new GMAPCache().ShowUserControl()`, over `CacheLocator.Location`, the map's cache.
            // `// C#: MainV2.cs:4132-4136`
            CmdKey::MapCache => self
                .key_forms
                .map_cache
                .show(&mp_tiles::TileCache::default_root()),
            // `new PropagationSettings().Show()`; what its constructor throws, on the status line.
            // `// C#: MainV2.cs:4147-4152`
            CmdKey::Propagation => {
                if let Err(why) = self.key_forms.propagation.show(&mut self.persisted) {
                    self.file_status = Some(why);
                }
            }
            // `new Camera().test(MainV2.comPort)`. `// C#: MainV2.cs:4154-4159`
            CmdKey::CameraTest => self.key_forms.camera.press(&mut self.telemetry),
            // `new DevopsUI().ShowUserControl()`. `// C#: MainV2.cs:4195-4200`
            CmdKey::Devops => self.key_forms.devops.show(),
        }
        true
    }

    /// `FlightData.ProcessCmdKey`'s playback keys: Space toggles a loaded log's play and pause,
    /// and is taken; the keypad's - and + step `LogPlayBackSpeed` and leave the key untaken, as
    /// the C# returns false after them.
    /// `// C#: GCSViews/FlightData.cs:918-943`
    fn run_playback_key(&mut self, key: PlaybackKey, cx: &mut Context<Self>) -> bool {
        let playback = &mut self.fly_data.playback;
        match key {
            PlaybackKey::PlayPause => {
                if !playback.loaded() {
                    return false;
                }
                playback.toggle();
                true
            }
            PlaybackKey::Slower | PlaybackKey::Faster => {
                playback.step_speed(key == PlaybackKey::Faster);
                cx.notify();
                false
            }
        }
    }

    /// Once a frame: the forms' boxes the keyboard has left, DevOps' answer awaited, and the
    /// camera test's next command; what DevOps' test throws, on the status line.
    pub(crate) fn key_forms_tick(&mut self, window: &Window) {
        let forms = &mut self.key_forms;
        forms.devops_focus.tick(&mut forms.devops, window);
        propagation_settings::tick(
            &mut forms.propagation,
            &mut self.persisted,
            &forms.propagation_focus,
            window,
        );
        if let Some(why) = forms.devops.tick(&self.telemetry, web_time::Instant::now()) {
            self.file_status = Some(why);
        }
        forms.camera.tick(&mut self.telemetry);
    }

    /// F5: `comPort.getParamList()`, then `MyView.ShowScreen(MyView.current.Name)`. With no
    /// vehicle there is nothing to fetch; the screen is shown again either way.
    /// `// C#: MainV2.cs:4093-4098`
    fn refresh_param_list(&mut self) {
        if self.telemetry.view().vehicle.is_some() {
            self.telemetry.download_parameters();
        }
        let screen = self.screen;
        self.choose_screen(screen);
    }

    /// Ctrl+T, `comPort.Open(false)`: the box's port opened past `Connect`'s checks - no
    /// question about a moving model, no disconnect - the transport asking what its `Open` asks,
    /// and the parameters not fetched. Already open, nothing: `Open` returns at once.
    /// `// C#: MainV2.cs:4146-4157; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-671`
    fn override_connect(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let view = self.telemetry.view();
        if view.connected && !view.target.starts_with("file:") {
            return;
        }
        self.do_connect_with(true, window, cx);
    }

    /// Ctrl+Y: `PREFLIGHT_STORAGE` 1 to the vehicle, then "Done MAV_ACTION_STORAGE_WRITE" - the
    /// C# shows it whatever the answer, and "Invalid command" when `doCommand` throws.
    /// `// C#: MainV2.cs:4158-4174`
    fn storage_write(&mut self) {
        let report = crate::telemetry::Report {
            timed_out: Some(crate::raw_params::INVALID_COMMAND.to_owned()),
            refused: Some(STORAGE_WRITE_DONE.to_owned()),
            accepted: Some(STORAGE_WRITE_DONE.to_owned()),
            fallback: None,
        };
        let sent = self.telemetry.send_handle().and_then(|(_, vehicle)| {
            self.telemetry.command(
                vehicle,
                crate::raw_params::PREFLIGHT_STORAGE,
                crate::raw_params::COMMIT,
                report,
            )
        });
        if sent.is_none() {
            self.file_status = Some(crate::raw_params::INVALID_COMMAND.to_owned());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(text: &str) -> Keystroke {
        Keystroke::parse(text).expect("a keystroke")
    }

    /// Each of `ProcessCmdKey`'s keys, as `keyData ==` compares them.
    #[test]
    fn every_key_of_process_cmd_key() {
        let table = [
            ("f12", CmdKey::Connect),
            ("f2", CmdKey::FlightData),
            ("f3", CmdKey::FlightPlanner),
            ("f4", CmdKey::Tuning),
            ("f5", CmdKey::RefreshParams),
            ("ctrl-f", CmdKey::Temp),
            ("ctrl-p", CmdKey::PluginManager),
            ("ctrl-g", CmdKey::NmeaOut),
            ("ctrl-x", CmdKey::MapCache),
            ("ctrl-l", CmdKey::Spectrogram),
            ("ctrl-w", CmdKey::Propagation),
            ("ctrl-z", CmdKey::CameraTest),
            ("ctrl-t", CmdKey::OverrideConnect),
            ("ctrl-y", CmdKey::StorageWrite),
            ("ctrl-j", CmdKey::Devops),
        ];
        for (text, expected) in table {
            assert_eq!(cmd_key(&key(text)), Some(expected), "{text}");
        }
    }

    /// The flight screen's tabs: Ctrl+1 to Ctrl+9 the first nine, Ctrl+0 the tenth; nothing else.
    #[test]
    fn control_and_a_digit_is_a_flight_tab() {
        assert_eq!(fly_tab_key(&key("ctrl-1")), Some(0));
        assert_eq!(fly_tab_key(&key("ctrl-9")), Some(8));
        assert_eq!(fly_tab_key(&key("ctrl-0")), Some(9));
        for text in ["1", "ctrl-shift-1", "alt-1", "ctrl-p", "ctrl-f1"] {
            assert_eq!(fly_tab_key(&key(text)), None, "{text}");
        }
    }

    /// The playback keys: Space, and the keypad's - and + by gpui's Linux names; by the
    /// characters only where the keypad is not named apart; nothing with a modifier.
    #[test]
    fn the_playback_keys() {
        assert_eq!(
            playback_key(&key("space"), true),
            Some(PlaybackKey::PlayPause)
        );
        assert_eq!(
            playback_key(&key("subtract"), true),
            Some(PlaybackKey::Slower)
        );
        assert_eq!(playback_key(&key("add"), true), Some(PlaybackKey::Faster));
        assert_eq!(playback_key(&key("-"), true), None);
        assert_eq!(playback_key(&key("+"), true), None);
        assert_eq!(playback_key(&key("-"), false), Some(PlaybackKey::Slower));
        assert_eq!(playback_key(&key("+"), false), Some(PlaybackKey::Faster));
        assert_eq!(
            playback_key(&key("subtract"), false),
            Some(PlaybackKey::Slower)
        );
        for text in [
            "ctrl-space",
            "shift-space",
            "alt-subtract",
            "ctrl-add",
            "p",
            "enter",
        ] {
            assert_eq!(playback_key(&key(text), true), None, "{text}");
        }
        // The keypad is named apart on Linux, where the GUI scripts press KP_Subtract and KP_Add.
        assert_eq!(KEYPAD_NAMED, cfg!(target_os = "linux"));
    }

    /// Ctrl+G's and Ctrl+L's windows over every screen, drawn here but where the Advanced page's
    /// windows are drawn already.
    #[test]
    fn the_advanced_windows_are_drawn_once_on_every_screen() {
        for screen in [
            Screen::Fly,
            Screen::Plan,
            Screen::Config,
            Screen::Params,
            Screen::Logs,
            Screen::Sitl,
            Screen::Help,
            Screen::Plugins,
        ] {
            assert!(draws_advanced_windows(screen), "{screen:?}");
        }
        assert!(!draws_advanced_windows(Screen::Setup));
        assert!(!draws_advanced_windows(Screen::Experimental));
    }

    /// Each key's form is in `ProcessCmdKey` as this module opens it, and the playback keys in
    /// the flight screen's, read from the tree when it is here.
    #[test]
    fn the_forms_are_the_keys_csharp() {
        let Some(main) = crate::config_coverage::source::csharp("MainV2.cs") else {
            eprintln!(
                "skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner"
            );
            return;
        };
        let body = main
            .split("protected override bool ProcessCmdKey")
            .nth(1)
            .and_then(|rest| rest.split("ProcessCmdKeyCallback != null").next())
            .unwrap_or_default();
        let opened = |letter: &str, call: &str| {
            body.split(&format!("(Keys.Control | Keys.{letter})"))
                .nth(1)
                .and_then(|rest| rest.split("return true;").next())
                .is_some_and(|arm| arm.contains(call))
        };
        assert!(opened("G", "new SerialOutputNMEA()"));
        assert!(opened("X", "new GMAPCache().ShowUserControl()"));
        assert!(opened("L", "new SpectrogramUI().Show()"));
        assert!(opened("W", "new PropagationSettings().Show()"));
        assert!(opened("Z", "new Camera().test(MainV2.comPort)"));
        assert!(opened("J", "new DevopsUI().ShowUserControl()"));
        let Some(flight) = crate::config_coverage::source::csharp("GCSViews/FlightData.cs") else {
            return;
        };
        for (keys, call) in [
            ("Keys.Space", "BUT_playlog_Click(null, null)"),
            ("Keys.Subtract", "LogPlayBackSpeed /= 2"),
            ("Keys.Add", "LogPlayBackSpeed *= 2"),
        ] {
            assert!(
                flight
                    .split(&format!("keyData == ({keys})"))
                    .nth(1)
                    .is_some_and(|rest| rest.contains(call)),
                "{keys}"
            );
        }
    }

    /// Anything else is not its: a letter alone, another modifier with Control, an F key with a
    /// modifier, Ctrl+S (commented out in the C#), a key it does not name.
    #[test]
    fn other_keys_are_not_its() {
        for text in [
            "p", "f", "ctrl-shift-p", "ctrl-alt-p", "cmd-p", "shift-f2", "ctrl-f2", "ctrl-s",
            "ctrl-q", "f1", "f6", "escape",
        ] {
            assert_eq!(cmd_key(&key(text)), None, "{text}");
        }
    }
}
