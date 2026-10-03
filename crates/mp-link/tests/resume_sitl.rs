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

//! Resume Mission's sequence against a real vehicle, step by step and without a window.
//!
//! `tests/gui/fly-resumemis.gui` fails at the take-off: the copter, armed in Guided, answers
//! `MAV_CMD_NAV_TAKEOFF` with FAILED, while the same take-off asked for on its own is accepted
//! (`commands_sitl.rs`). So the sequence before it is suspect, and these replay it as
//! `FlightData.cs:1481-1640` and `fly.rs`'s `ResumeMission` do - upload the trimmed mission, set
//! the current item to 1, read the mission back, Guided, arm, take off - leaving out one step at a
//! time. Ignored by default: they need SITL on tcp:127.0.0.1:5760 and must run one at a time
//! (`--test-threads=1`), since one vehicle cannot fly two of them.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::messages::LogMessage;
use mp_link::mission_transfer::TransferState;
use mp_link::requests::RequestState;
use mp_link::{Link, LinkConfig, commands};
use mp_mission::MissionItem;
use mp_vehicle::VehicleId;

const MODE_STABILIZE: u32 = 0;
const MODE_GUIDED: u32 = 4;
const MODE_LAND: u32 = 9;
/// `MAV_CMD.LAST` and `MAV_CMD.DO_LAST`, the bounds `BUT_resumemis_Click` trims with.
const MAV_CMD_LAST: u16 = 95;
const MAV_CMD_DO_LAST: u16 = 240;

fn connect() -> (Link, VehicleId) {
    let link = Link::connect("tcp:127.0.0.1:5760", LinkConfig::default()).expect("SITL on 5760");
    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let (id, _) = link.primary_vehicle().expect("a vehicle");
    (link, id)
}

fn await_transfer(link: &Link, id: VehicleId) -> Result<Vec<MissionItem>, String> {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(transfer) = link.mission_transfer(id) {
            match transfer.state() {
                TransferState::Complete => return Ok(transfer.items().to_vec()),
                TransferState::Failed(err) => return Err(err.to_string()),
                _ => {}
            }
        }
        assert!(Instant::now() < deadline, "the transfer never finished");
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// A transfer the way a user runs one: started, waited for, and started once more if it failed,
/// as the user presses the button again. The link's retries are the C#'s (`ProtocolTimeouts`),
/// and the hosted runner's simulator let them run out once: a MISSION_REQUEST_LIST sent right
/// after a set-current got no count in six tries of 700 ms (CI run 37110906200, 2026-10-03),
/// where this machine's answers at once. The second try is the test's, not the link's.
fn transfer(link: &Link, id: VehicleId, start: impl Fn() -> bool) -> Vec<MissionItem> {
    assert!(start(), "the transfer did not start");
    match await_transfer(link, id) {
        Ok(items) => items,
        Err(err) => {
            println!("transfer failed once ({err}); started again, as a user would");
            assert!(start(), "the second transfer did not start");
            await_transfer(link, id).unwrap_or_else(|err| panic!("transfer failed twice: {err}"))
        }
    }
}

fn await_message(
    link: &Link,
    what: &str,
    mut matches: impl FnMut(&LogMessage) -> bool,
) -> LogMessage {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        if let Some(found) = link.recent_messages(200).into_iter().find(&mut matches) {
            return found;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The script's mission: home, a take-off, a waypoint, a speed change, a waypoint.
fn script_mission() -> Vec<MissionItem> {
    let item =
        |seq: u16, frame: u8, command: u16, param2: f64, x: f64, y: f64, z: f64| MissionItem {
            seq,
            current: u8::from(seq == 0),
            frame,
            command,
            param1: 0.0,
            param2,
            param3: 0.0,
            param4: 0.0,
            x,
            y,
            z,
            autocontinue: 1,
        };
    vec![
        item(0, 0, 16, 0.0, -35.363_262, 149.165_237, 584.09),
        item(1, 3, 22, 0.0, 0.0, 0.0, 10.0),
        item(2, 3, 16, 0.0, -35.3629, 149.165_237, 10.0),
        item(3, 3, 178, 5.0, 0.0, 0.0, 0.0),
        item(4, 3, 16, 0.0, -35.3625, 149.1657, 10.0),
    ]
}

/// `fly.rs`'s `resume_items`: from the resume point on, and before it only home, take-offs and
/// the "do" commands. `// C#: GCSViews/FlightData.cs:1508-1541`
fn trimmed(items: &[MissionItem], resume_at: u16) -> Vec<MissionItem> {
    items
        .iter()
        .enumerate()
        .filter(|&(index, item)| {
            let index = u16::try_from(index).unwrap();
            if index < resume_at && index != 0 {
                if item.command != 22 && item.command < MAV_CMD_LAST {
                    return false;
                }
                if item.command > MAV_CMD_DO_LAST {
                    return false;
                }
            }
            true
        })
        .enumerate()
        .map(|(seq, (_, item))| MissionItem {
            seq: u16::try_from(seq).unwrap(),
            current: 0,
            autocontinue: 1,
            ..*item
        })
        .collect()
}

/// Waits until the vehicle's state passes `check`.
fn wait(
    link: &Link,
    id: VehicleId,
    what: &str,
    secs: u64,
    check: &dyn Fn(&mp_vehicle::VehicleState) -> bool,
) {
    let handle = link.vehicle(id).expect("the vehicle's state");
    let deadline = Instant::now() + Duration::from_secs(secs);
    while !check(&handle.load()) {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Guided until in it, arm until armed, a second between asks, as the C#'s two loops.
/// `// C#: GCSViews/FlightData.cs:1555-1584`
fn guided_and_armed(link: &Link, id: VehicleId) {
    let handle = link.vehicle(id).expect("the vehicle's state");
    let deadline = Instant::now() + Duration::from_secs(30);
    while handle.load().custom_mode != MODE_GUIDED {
        assert!(
            Instant::now() < deadline,
            "the vehicle never entered Guided"
        );
        link.send(&commands::set_mode(id, MODE_GUIDED));
        std::thread::sleep(Duration::from_secs(1));
    }
    let deadline = Instant::now() + Duration::from_secs(30);
    while !handle.load().armed {
        assert!(
            Instant::now() < deadline,
            "the vehicle never armed in Guided"
        );
        link.send(&commands::arm(id, true, false));
        std::thread::sleep(Duration::from_secs(1));
    }
}

/// Sends a take-off and returns the vehicle's answer to it: the first `NAV_TAKEOFF` line the
/// log gains after the send.
fn takeoff(link: &Link, id: VehicleId, altitude: f32) -> LogMessage {
    let after = link.recent_messages(1).last().map_or(0, |m| m.seq);
    assert!(link.send(&commands::takeoff(id, altitude)), "send failed");
    await_message(link, "a take-off acknowledgement", |m| {
        m.seq > after && m.text.contains("NAV_TAKEOFF")
    })
}

/// Down, disarmed, and left in Stabilize for the next test.
fn land(link: &Link, id: VehicleId) {
    link.send(&commands::set_mode(id, MODE_LAND));
    wait(link, id, "the vehicle to land and disarm", 120, &|s| {
        !s.armed
    });
    link.send(&commands::set_mode(id, MODE_STABILIZE));
    wait(link, id, "Stabilize", 10, &|s| {
        s.custom_mode == MODE_STABILIZE
    });
}

/// The sequence, with the steps chosen, ending at the take-off's answer. Lands afterwards.
fn resume(with_upload: bool, with_set_current: bool) -> String {
    let (link, id) = connect();
    let handle = link.vehicle(id).expect("the vehicle's state");

    // The script's setup: the five items on the vehicle before the button is pressed.
    transfer(&link, id, || link.upload_mission(id, script_mission()));

    // BUT_resumemis_Click: read the mission (getWPs), trim, write it, set current to 1, read it
    // back into the planner.
    let held = transfer(&link, id, || link.download_mission(id));
    assert_eq!(held.len(), 5, "the vehicle holds the script's mission");
    if with_upload {
        let items = trimmed(&held, 4);
        assert_eq!(
            items.len(),
            4,
            "home, the take-off, the speed change, waypoint 4"
        );
        transfer(&link, id, || link.upload_mission(id, items.clone()));
    }
    if with_set_current {
        let request = link.set_current_waypoint(id, 1);
        let deadline = Instant::now() + Duration::from_secs(15);
        loop {
            match link.request(request).map(|r| r.state()) {
                Some(RequestState::Finished(outcome)) => {
                    println!("set current 1: {outcome:?}");
                    break;
                }
                None => panic!("the set-current request was forgotten"),
                _ => {}
            }
            assert!(Instant::now() < deadline, "set current never answered");
            std::thread::sleep(Duration::from_millis(50));
        }
        transfer(&link, id, || link.download_mission(id));
    }

    // Guided until in it, arm until armed, then the take-off, as the C#'s three loops.
    guided_and_armed(&link, id);
    let state = handle.load();
    println!(
        "armed, mode {}, {:.1} m relative, mission current {}",
        state.custom_mode, state.altitude_relative.0, state.mission_current
    );
    let ack = takeoff(&link, id, 10.0);
    println!(
        "upload {with_upload}, set current {with_set_current}: {}",
        ack.text
    );
    if ack.text.contains("accepted") {
        wait(&link, id, "8 m of climb", 60, &|s| {
            s.altitude_relative.0 > 8.0
        });
    }
    land(&link, id);
    ack.text
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_whole_resume_sequence_takes_off() {
    let answer = resume(true, true);
    assert!(answer.contains("accepted"), "{answer}");
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_resume_sequence_without_set_current_takes_off() {
    let answer = resume(true, false);
    assert!(answer.contains("accepted"), "{answer}");
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_resume_sequence_without_the_upload_takes_off() {
    let answer = resume(false, true);
    assert!(answer.contains("accepted"), "{answer}");
}

/// Why `tests/gui/fly-resumemis.gui` ends in "Command Failed". The C# does not send the take-off
/// once: it sends it, sleeps a second, and sends it again until the vehicle is within 2 m of the
/// waypoint's height, and a take-off `doCommand` returns false for is "Command Failed" and the
/// end of the resume. ArduCopter refuses a take-off once the vehicle has left the ground
/// (`Mode::do_user_takeoff_U_m`: "can't takeoff again!"), and a climb to 8 m takes more than a
/// second, so the repeat is refused. The tests above send the take-off once, and it was
/// accepted every time; this sends it as the C# does.
/// `// C#: GCSViews/FlightData.cs:1587-1605`
#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_csharps_repeated_takeoff_is_refused_once_the_vehicle_is_climbing() {
    let (link, id) = connect();
    guided_and_armed(&link, id);
    let first = takeoff(&link, id, 10.0);
    // The C# sleeps a second between sends. Here the second send waits until the vehicle has
    // left the ground, which is what the refusal turns on (`ap.land_complete`): a second sufficed
    // on this machine, and on the hosted runner (2026-10-03) the vehicle was still at 0.0 m after
    // one, and the repeat was accepted.
    let deadline = Instant::now() + Duration::from_secs(20);
    let climbed = loop {
        let climbed = link
            .vehicle(id)
            .expect("the vehicle's state")
            .load()
            .altitude_relative
            .0;
        if climbed > 0.3 {
            break climbed;
        }
        assert!(
            Instant::now() < deadline,
            "the vehicle did not leave the ground within 20 s: {climbed:.1} m"
        );
        std::thread::sleep(Duration::from_millis(100));
    };
    let second = takeoff(&link, id, 10.0);
    println!(
        "first: {}; off the ground at {climbed:.1} m: {}",
        first.text, second.text
    );
    land(&link, id);
    assert!(first.text.contains("accepted"), "{}", first.text);
    assert!(climbed < 8.0, "still within 2 m of the height when sent again: {climbed:.1} m");
    assert!(second.text.contains("failed"), "{}", second.text);
}
