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

//! Mission Planner's plugins, ported to the WebAssembly plugin world (`../wit/plugin.wit`).
//!
//! The C# ships four real plugins - OpenDroneID, TerrainMaker, Dowding and AnonymizeBinlog - and
//! some twenty examples under `plugins/` and `Plugins/`. Each port here is an example of this
//! crate, built as its own module for `wasm32-unknown-unknown`, and the host's tests load and
//! drive every one of them (`crates/mp-plugin-host/tests/plugins.rs`). This library is what they
//! share: the WIT bindings, and the small helpers the C#'s `PointLatLngAlt` gives them.
//!
//! What a port leaves out is said at the site, in the example's header; the examples that cannot
//! be ported at all are listed below with the reason.
//!
//! Not ported, and why:
//!
//! * `example11-trace.cs`, `example12-forwarding.cs`, `example19-multiforward.cs`,
//!   `example18-externalapi.cs`, `example13-herelink2.cs`, `example4-herelink.cs`: TCP and UDP
//!   sockets (`TcpListener`, `UdpClient`, `comPort.MirrorStream`). A plugin here has no sockets,
//!   and the host lends none: unimplementable in this world.
//! * `example-watchbutton.cs`, `example5-latencytracker.cs`, `example20-multiplepositions.cs`,
//!   `generator.cs`: `comPort.OnPacketReceived` / `SubscribeToPacketType`, a callback per
//!   received packet; the world has no packet subscription (the `cs` reads carry what the link
//!   decoded).
//! * `example9-hudonoff.cs`, `example22-fontsize.cs`, `example16-donate.cs`,
//!   `example17-menuremove.cs`: they change the main window itself (the HUD's items, the form's
//!   scale, the main menu, the map controls' menus removed); the world reaches the two map menus
//!   only.
//! * `example23-switch.cs`: a CubeLAN switch over `comPort.device` (a serial device of its own).
//! * `example7-canrtcm.cs`: its file is CAN frames it replays through `comPort` as DroneCAN; this
//!   tree has no DroneCAN.
//! * `FaceMap`: its survey runs on `Host.FPDrawnPolygon` and the planner's own grid code, which
//!   the world does not carry.
//! * `Shortcuts/Plugin.cs`: keyboard shortcuts through `MainV2.instance.ProcessCmdKeyCallback`.

// wit-bindgen's generated glue is `unsafe` (the canonical ABI's pointers) and undocumented.
#[allow(unsafe_code, missing_docs, unreachable_pub, clippy::all)]
#[doc(hidden)]
pub mod bindings {
    wit_bindgen::generate!({
        path: "../wit",
        world: "plugin",
        pub_export_macro: true,
        export_macro_name: "export_plugin",
        default_bindings_module: "mp_plugins::bindings",
    });
}

pub use bindings::Guest;
pub use bindings::export_plugin;
pub use bindings::missionplanner::plugin::host;

/// The earth's radius `PointLatLngAlt.GetDistance` uses, in metres.
const EARTH_RADIUS: f64 = 6_371_000.0;

/// `PointLatLngAlt.GetDistance`: the haversine distance between two points in degrees, in
/// metres. `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
#[must_use]
pub fn distance(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    let d_lat = (lat2 - lat1).to_radians();
    let d_lng = (lng2 - lng1).to_radians();
    let a = (d_lat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lng / 2.0).sin().powi(2);
    EARTH_RADIUS * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
}

/// `PointLatLngAlt.GetBearing`: the initial bearing from the first point to the second, in
/// degrees 0-360. `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:350-360`
#[must_use]
pub fn bearing(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    let (lat1, lat2) = (lat1.to_radians(), lat2.to_radians());
    let d_lng = (lng2 - lng1).to_radians();
    let y = d_lng.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * d_lng.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

/// `Host.cs.<name>` as a number: a number field's value, a flag as 1 or 0, none for text or a
/// missing field.
#[must_use]
pub fn cs_number(name: &str) -> Option<f64> {
    match host::cs(name)? {
        host::CsValue::Number(value) => Some(value),
        host::CsValue::Flag(flag) => Some(if flag { 1.0 } else { 0.0 }),
        host::CsValue::Text(_) => None,
    }
}

/// `Host.cs.<name>` as text: a text field's value, a number or flag formatted as the C#'s
/// `ToString` would, none for a missing field.
#[must_use]
pub fn cs_text(name: &str) -> Option<String> {
    Some(match host::cs(name)? {
        host::CsValue::Text(text) => text,
        host::CsValue::Number(value) => value.to_string(),
        host::CsValue::Flag(flag) => if flag { "True" } else { "False" }.to_owned(),
    })
}

/// `CustomMessageBox.Show(text)`: one OK button, no caption.
pub fn show(text: &str) {
    let _ = host::message_box(text, "", host::MessageButtons::Ok);
}

/// A control of a form, built in one line.
#[must_use]
pub fn control(
    id: &str,
    kind: host::ControlKind,
    label: &str,
    value: &str,
    options: &[&str],
) -> host::Control {
    host::Control {
        id: id.to_owned(),
        kind,
        label: label.to_owned(),
        value: value.to_owned(),
        options: options.iter().map(|&option| option.to_owned()).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A degree of latitude is 111.2 km on this sphere; a bearing due east is 90.
    #[test]
    fn distance_and_bearing_are_the_csharps() {
        assert!((distance(0.0, 0.0, 1.0, 0.0) - 111_194.9).abs() < 1.0);
        assert!((bearing(0.0, 0.0, 0.0, 1.0) - 90.0).abs() < 1e-9);
        assert!((bearing(0.0, 0.0, -1.0, 0.0) - 180.0).abs() < 1e-9);
    }
}
