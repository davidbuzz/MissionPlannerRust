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

//! The world's packet subscription - `SubscribeToPacketType` as `subscribe-packet`, the C#'s
//! function as `packet-received` - with the plugin that needs it, Open Drone ID, whose backend
//! starts when the vehicle's Remote ID transmitter is first heard.
//!
//! Each runs twice: on the desktop's engine (Cranelift) and on the browser build's - wasmtime's
//! Pulley interpreter with `web_config`'s settings, at this machine's pointer width
//! (`common::web_engine`). The page's own 32-bit bytecode cannot run here, wasmtime running
//! Pulley only at its host's width; everything else of the page's engine is this one.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use std::sync::Arc;
use web_time::{Duration, Instant};

use common::{Scripted, load_on, path, web_engine};
use mp_plugin_host::{
    CsValue, DialogResult, Limits, PluginHost, PluginSource, RequestBody, engine,
};

/// `MAVLINK_MSG_ID.OPEN_DRONE_ID_ARM_STATUS`.
const ARM_STATUS: u32 = 12918;
/// `MAVLINK_MSG_ID.HEARTBEAT`.
const HEARTBEAT: u32 = 0;
/// The backend's messages: `OPEN_DRONE_ID_BASIC_ID` and `OPEN_DRONE_ID_SYSTEM_UPDATE`.
const BASIC_ID: u32 = 12900;
const SYSTEM_UPDATE: u32 = 12919;

/// An arm status's payload: its status, and its error in fifty bytes.
fn arm_status(status: u8, error: &str) -> Vec<u8> {
    let mut payload = vec![status];
    payload.extend(error.bytes());
    payload.resize(51, 0);
    payload
}

/// The value the form shows for control `id`.
fn shown(script: &Scripted, id: &str) -> String {
    script.control(id).map(|c| c.value).unwrap_or_default()
}

/// The status lines, newest first.
fn status_lines(script: &Scripted) -> Vec<String> {
    shown(script, "log").lines().map(str::to_owned).collect()
}

/// Open Drone ID on `engine`, against a scripted application, through the transmitter's life:
/// nothing subscribed before a vehicle is heard, then the C#'s four subscriptions for it at the
/// next second; nothing sent before the transmitter is heard; its first arm status the start -
/// the C#'s lines, the update and the basic ID on the next two Loops - its error said and shown,
/// then ready, then silent five seconds: "Timeout.".
/// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:99-214; OpenDroneID_Backend.cs:54-134`
fn drone_id_hears_its_transmitter(engine: &wasmtime::Engine) {
    let scripted = Scripted::new();
    scripted.with(|r| {
        r.config
            .insert("tabcontrolactions".to_owned(), "tabQuick;".to_owned());
    });
    let Some((mut plugin, script)) = load_on(engine, "opendroneid", scripted, Limits::default())
    else {
        return;
    };
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert_eq!(shown(&script, "odid_reason"), "{Loading....}");

    // No link: a second of Loops, nothing subscribed; the GCS GPS's light orange without one.
    for _ in 0..10 {
        plugin.run_loop().unwrap();
    }
    assert!(script.record().subscriptions.is_empty());
    assert_eq!(shown(&script, "gcs_gps"), "Orange");

    // A vehicle heard: at the next second its transmitters and its autopilot.
    script.with(|r| {
        r.cs.insert("connected".to_owned(), CsValue::Flag(true));
        r.sysid = 1;
    });
    for _ in 0..10 {
        plugin.run_loop().unwrap();
    }
    assert_eq!(
        script.record().subscriptions,
        [
            (ARM_STATUS, 1, 236),
            (ARM_STATUS, 1, 237),
            (ARM_STATUS, 1, 238),
            (ARM_STATUS, 1, 1)
        ]
    );
    assert_eq!(
        status_lines(&script),
        ["Sub. to ODID ARM_STATUS for SysId: 1"]
    );
    assert!(script.record().packets.is_empty(), "nothing sent unheard");

    // A message the plugin did not subscribe to is not its handler's.
    plugin.packet_received(HEARTBEAT, 1, 1, &[0; 9]).unwrap();
    assert_eq!(status_lines(&script).len(), 1);

    // The transmitter: detected, the backend started for its system.
    plugin
        .packet_received(ARM_STATUS, 1, 236, &arm_status(1, "SYSTEM not received"))
        .unwrap();
    assert_eq!(
        script.record().log,
        ["[DRONE_ID] Detected and Starting on System ID: 1"]
    );
    assert_eq!(
        status_lines(&script),
        [
            "Double Click Primary status indicator to declare Emergency over ODID.",
            "Detected and Starting on System ID: 1",
            "Sub. to ODID ARM_STATUS for SysId: 1",
        ]
    );
    plugin.run_loop().unwrap();
    plugin.run_loop().unwrap();
    {
        let r = script.record();
        let ids: Vec<u32> = r.packets.iter().map(|(id, _)| *id).collect();
        assert_eq!(ids, [SYSTEM_UPDATE, BASIC_ID]);
        // Addressed to the system it was heard from, every component.
        assert_eq!(r.packets[1].1[..2], [1, 0]);
    }
    assert_eq!(shown(&script, "rid_comms"), "Green");
    assert_eq!(shown(&script, "arm_status"), "Red");
    assert_eq!(shown(&script, "status"), "SYSTEM not received");
    assert_eq!(shown(&script, "odid_ok"), "Remote ID Fail");
    assert_eq!(shown(&script, "odid_reason"), "GCS GPS Invalid");

    // The error said; then ready.
    plugin
        .packet_received(ARM_STATUS, 1, 236, &arm_status(1, "SYSTEM not received"))
        .unwrap();
    assert_eq!(status_lines(&script)[0], "Arm Error: SYSTEM not received");
    plugin
        .packet_received(ARM_STATUS, 1, 236, &arm_status(0, ""))
        .unwrap();
    plugin.run_loop().unwrap();
    assert_eq!(status_lines(&script)[0], "Arm Stats: Ready");
    assert_eq!(shown(&script, "status"), "Ready");
    assert_eq!(shown(&script, "arm_status"), "Green");

    // Five seconds without one.
    for _ in 0..50 {
        plugin.run_loop().unwrap();
    }
    assert_eq!(shown(&script, "status"), "Timeout.");
    assert_eq!(shown(&script, "rid_comms"), "Red");
}

#[test]
fn drone_id_hears_its_transmitter_on_the_desktops_engine() {
    drone_id_hears_its_transmitter(&engine().unwrap());
}

#[test]
fn drone_id_hears_its_transmitter_on_the_browsers_engine() {
    drone_id_hears_its_transmitter(&web_engine());
}

/// What the window saw of the plugin while it pumped.
#[derive(Debug, Default)]
struct Seen {
    sent: Vec<u32>,
    log: String,
    status: String,
}

/// How long a pump waits for what it is waiting for: the plugin's compile first, on a loaded
/// machine, then a second of its Loops.
const WAIT: Duration = Duration::from_secs(30);

/// Drains `host` until `done` or `wait` has passed, answering as the window would: the saved
/// tabs, so no restart is asked; every packet sent; the form's status lines and arm status kept.
fn pump(host: &mut PluginHost, seen: &mut Seen, wait: Duration, done: impl Fn(&Seen) -> bool) {
    let started = Instant::now();
    while started.elapsed() < wait && !done(seen) {
        for request in host.drain() {
            match request.body {
                RequestBody::ConfigGet { reply, .. } => reply.send(Some("tabQuick;".to_owned())),
                RequestBody::MessageBox { reply, .. } => reply.send(DialogResult::Ok),
                RequestBody::SendPacket {
                    message_id, reply, ..
                } => {
                    seen.sent.push(message_id);
                    reply.send(true);
                }
                RequestBody::FormShow { controls, .. } => {
                    for control in controls {
                        match control.id.as_str() {
                            "log" => seen.log = control.value,
                            "status" => seen.status = control.value,
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
        wasm_thread::sleep(Duration::from_millis(5));
    }
}

/// The host's side, on the plugins' threads: the plugin's four subscriptions in the host's table
/// as it makes them; a packet of a message nobody subscribed to, or from a component or system
/// nobody did, handed to nobody; the transmitter's handed to the plugin, which shows it and
/// starts sending. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5508-5577`
fn the_host_hands_over_what_was_subscribed(engine: &wasmtime::Engine) {
    let Some(opendroneid) = path("opendroneid") else {
        return;
    };
    let mut host = PluginHost::load_sources_on(
        engine,
        vec![PluginSource::File(opendroneid)],
        Limits::default(),
    );
    assert!(!host.wants_packets(), "nothing subscribed yet");
    // The window's snapshot: a link open, its vehicle 1, 1.
    host.update_snapshot(|snapshot| {
        snapshot.cs = Arc::new(|name: &str| (name == "connected").then_some(CsValue::Flag(true)));
        snapshot.vehicle = Some((1, 1));
    });
    let mut seen = Seen::default();
    pump(&mut host, &mut seen, WAIT, |seen| {
        seen.log.contains("Sub. to ODID ARM_STATUS for SysId: 1")
    });
    let subscribed: Vec<(usize, u32, u32, u8)> = host
        .subscriptions()
        .iter()
        .map(|s| (s.plugin, s.message_id, s.sysid, s.compid))
        .collect();
    assert_eq!(
        subscribed,
        [
            (0, ARM_STATUS, 1, 236),
            (0, ARM_STATUS, 1, 237),
            (0, ARM_STATUS, 1, 238),
            (0, ARM_STATUS, 1, 1)
        ]
    );
    assert!(host.wants_packets());

    // Not subscribed: another message, another component, another system.
    let sink = host.packet_sink();
    assert_eq!(sink.offer(HEARTBEAT, 1, 1, || vec![0; 9]), 0);
    assert_eq!(
        sink.offer(ARM_STATUS, 1, 100, || arm_status(1, "SYSTEM not received")),
        0
    );
    assert_eq!(
        sink.offer(ARM_STATUS, 2, 236, || arm_status(1, "SYSTEM not received")),
        0
    );
    // Three Loops more, to be sure.
    pump(&mut host, &mut seen, Duration::from_millis(300), |_| false);
    assert!(
        seen.sent.is_empty(),
        "no transmitter heard: {:?}",
        seen.sent
    );
    assert!(!seen.log.contains("Detected"), "{}", seen.log);

    // Subscribed: the transmitter, from its first component.
    assert_eq!(
        sink.offer(ARM_STATUS, 1, 236, || arm_status(1, "SYSTEM not received")),
        1
    );
    pump(&mut host, &mut seen, WAIT, |seen| {
        seen.status == "SYSTEM not received" && seen.sent.len() >= 2
    });
    assert!(
        seen.log.contains("Detected and Starting on System ID: 1"),
        "{}",
        seen.log
    );
    assert_eq!(
        seen.sent.get(..2),
        Some(&[SYSTEM_UPDATE, BASIC_ID][..]),
        "{:?}",
        seen.sent
    );
    assert_eq!(seen.status, "SYSTEM not received");
    host.shutdown();
}

#[test]
fn the_host_hands_over_what_was_subscribed_on_the_desktops_engine() {
    the_host_hands_over_what_was_subscribed(&engine().unwrap());
}

#[test]
fn the_host_hands_over_what_was_subscribed_on_the_browsers_engine() {
    the_host_hands_over_what_was_subscribed(&web_engine());
}
