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

//! The computer's serial ports in a web page (the browser build's serial row: "a USB autopilot,
//! Install Firmware and SiK radios need WebSerial").
//!
//! A page sees only the serial ports the browser has let it use, each chosen once in the
//! browser's own chooser, which only a click may open. Those are the page's serial ports, listed
//! in the port box as the desktop lists its own (mp-transport's `list_ports`); the box's list also
//! has [`CHOOSE`], whose click opens the chooser, and the port chosen joins the list and is
//! selected as a click on its row selects it. web/www/serial.js is the page's half. In a browser
//! without WebSerial (Firefox, Safari), and on the desktop, nothing is added.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;

/// The port list's entry that opens the browser's chooser.
pub const CHOOSE: &str = "Choose a serial port...";

thread_local! {
    /// The port the chooser gave, by its name, for the port box to select.
    static CHOSEN: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// The port box's list as `PopulateSerialportList` makes it (`connect::port_list`), with
/// [`CHOOSE`] after the serial ports when the page has the browser's chooser.
#[must_use]
pub fn with_choose(mut list: Vec<String>, serial_ports: usize, available: bool) -> Vec<String> {
    if available {
        // `AUTO` first, then the serial ports.
        list.insert((1 + serial_ports).min(list.len()), CHOOSE.to_owned());
    }
    list
}

/// Whether the page has the browser's serial chooser.
#[must_use]
pub fn available() -> bool {
    page::available()
}

/// Opens the browser's chooser; the port chosen comes back through [`take_chosen`].
pub fn choose() {
    page::choose();
}

/// The port the chooser gave since the last call, by its name.
#[must_use]
pub fn take_chosen() -> Option<String> {
    CHOSEN.with(|chosen| chosen.borrow_mut().take())
}

/// The port chosen, kept for the port box and a frame asked for.
#[cfg_attr(not(target_family = "wasm"), allow(dead_code))]
fn chosen(name: String) {
    CHOSEN.with(|chosen| *chosen.borrow_mut() = Some(name));
    crate::repaint::again_in(std::time::Duration::ZERO);
}

#[cfg(target_family = "wasm")]
mod page {
    use wasm_bindgen::JsCast as _;
    use wasm_bindgen::prelude::wasm_bindgen;

    fn chooser() -> Option<js_sys::Function> {
        js_sys::Reflect::get(&js_sys::global(), &"mprSerialChoose".into())
            .ok()?
            .dyn_into()
            .ok()
    }

    pub(super) fn available() -> bool {
        chooser().is_some()
    }

    pub(super) fn choose() {
        if let Some(open) = chooser()
            && let Err(err) = open.call0(&wasm_bindgen::JsValue::NULL)
        {
            log::warn!("the browser's serial chooser: {err:?}");
        }
    }

    /// The page (serial.js): the port chosen in the browser's chooser, by its name in the list.
    #[wasm_bindgen]
    pub fn planner_serial_chosen(name: String) {
        super::chosen(name);
    }
}

#[cfg(not(target_family = "wasm"))]
mod page {
    pub(super) const fn available() -> bool {
        false
    }

    pub(super) const fn choose() {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_chooser_follows_the_serial_ports_when_the_page_has_one() {
        let list = crate::connect::port_list(&["WebSerial 1 (1209:5741)".to_owned()]);
        assert_eq!(
            with_choose(list.clone(), 1, true),
            [
                "AUTO",
                "WebSerial 1 (1209:5741)",
                CHOOSE,
                "TCP",
                "UDP",
                "UDPCl",
                "WS"
            ]
        );
        // None granted yet: straight after AUTO.
        assert_eq!(
            with_choose(crate::connect::port_list(&[]), 0, true)
                .get(1)
                .map(String::as_str),
            Some(CHOOSE)
        );
        // On the desktop, or a browser without WebSerial, the list is the C#'s.
        assert_eq!(with_choose(list.clone(), 1, false), list);
        assert!(!available());
    }

    #[test]
    fn a_chosen_port_is_taken_once() {
        chosen("WebSerial 2".to_owned());
        assert_eq!(take_chosen().as_deref(), Some("WebSerial 2"));
        assert_eq!(take_chosen(), None);
    }
}
