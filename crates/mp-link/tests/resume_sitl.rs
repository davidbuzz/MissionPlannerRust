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

fn await_transfer(link: &Link, id: VehicleId) -> Vec<MissionItem> {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(transfer) = link.mission_transfer(id) {
            match transfer.state() {
                TransferState::Complete => return transfer.items().to_vec(),
                TransferState::Failed(err) => panic!("transfer failed: {err}"),
                _ => {}
            }
        }
        assert!(Instant::now() < deadline, "the transfer never finished");
        std::thread::sleep(Duration::from_millis(50));
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

/// The sequence, with the steps chosen, ending at the take-off's answer. Lands afterwards.
fn resume(with_upload: bool, with_set_current: bool) -> String {
    let (link, id) = connect();
    let handle = link.vehicle(id).expect("the vehicle's state");
    let wait = |what: &str, secs: u64, check: &dyn Fn(&mp_vehicle::VehicleState) -> bool| {
        let deadline = Instant::now() + Duration::from_secs(secs);
        while !check(&handle.load()) {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            std::thread::sleep(Duration::from_millis(100));
        }
    };

    // The script's setup: the five items on the vehicle before the button is pressed.
    link.upload_mission(id, script_mission());
    await_transfer(&link, id);

    // BUT_resumemis_Click: read the mission (getWPs), trim, write it, set current to 1, read it
    // back into the planner.
    link.download_mission(id);
    let held = await_transfer(&link, id);
    assert_eq!(held.len(), 5, "the vehicle holds the script's mission");
    if with_upload {
        let items = trimmed(&held, 4);
        assert_eq!(
            items.len(),
            4,
            "home, the take-off, the speed change, waypoint 4"
        );
        link.upload_mission(id, items);
        await_transfer(&link, id);
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
        link.download_mission(id);
        await_transfer(&link, id);
    }

    // Guided until in it, arm until armed, then the take-off, as the C#'s three loops.
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
    let state = handle.load();
    println!(
        "armed, mode {}, {:.1} m relative, mission current {}",
        state.custom_mode, state.altitude_relative.0, state.mission_current
    );
    assert!(link.send(&commands::takeoff(id, 10.0)), "send failed");
    let ack = await_message(&link, "a take-off acknowledgement", |m| {
        m.text.contains("NAV_TAKEOFF")
    });
    println!(
        "upload {with_upload}, set current {with_set_current}: {}",
        ack.text
    );
    if ack.text.contains("accepted") {
        wait("8 m of climb", 60, &|s| s.altitude_relative.0 > 8.0);
    }
    link.send(&commands::set_mode(id, MODE_LAND));
    wait("the vehicle to land and disarm", 120, &|s| !s.armed);
    link.send(&commands::set_mode(id, MODE_STABILIZE));
    wait("Stabilize", 10, &|s| s.custom_mode == MODE_STABILIZE);
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
