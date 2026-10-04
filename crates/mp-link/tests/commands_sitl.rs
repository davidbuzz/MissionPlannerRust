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

//! The commands the flight screen sends, against a real vehicle.
//!
//! Ignored by default because it needs ArduPilot SITL, but ignored rather than silently skipped:
//! the test output says it exists and was not run. Start SITL with `tools/sitl/run-sitl.sh copter`
//! and run `cargo test -p mp-link -- --ignored`.
//!
//! These exist because the in-memory tests prove our own plumbing and nothing else. Whether real
//! firmware answers an arm request at all, whether it says why it refused, and whether a mode
//! change takes effect are questions only the firmware can answer.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use web_time::{Duration, Instant};

use mp_link::messages::LogMessage;
use mp_link::{Link, LinkConfig, commands};
use mp_vehicle::VehicleId;

/// Copter mode numbers, from the generated table.
const MODE_STABILIZE: u32 = 0;
const MODE_GUIDED: u32 = 4;

fn connect() -> (Link, VehicleId) {
    let link = Link::connect("tcp:127.0.0.1:5760", LinkConfig::default()).expect("SITL on 5760");
    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        wasm_thread::sleep(Duration::from_millis(100));
    }
    let (id, _) = link.primary_vehicle().expect("a vehicle");
    (link, id)
}

/// Waits for a message matching a predicate, returning it.
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
        wasm_thread::sleep(Duration::from_millis(50));
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_real_vehicle_acknowledges_an_arm_request() {
    // The flight screen's arm button is worthless without this: a ground station that cannot tell
    // whether a command was accepted can only show that a button was pressed.
    let (link, id) = connect();
    assert!(link.send(&commands::arm(id, true, false)), "send failed");

    let ack = await_message(&link, "an arm acknowledgement", |message| {
        message.text.contains("ARM_DISARM")
    });

    // Either answer is a pass. A freshly started SITL may still be running pre-arm checks, and a
    // test that demanded success would fail for reasons that have nothing to do with the code
    // under test. What matters is that the vehicle answered and that we read the answer.
    assert!(
        ack.text.contains("accepted")
            || ack.text.contains("denied")
            || ack.text.contains("temporarily rejected")
            || ack.text.contains("failed"),
        "unexpected ack: {}",
        ack.text
    );
    assert_eq!(ack.from, id);

    // Leave the vehicle as we found it.
    link.send(&commands::arm(id, false, false));
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_forced_arm_is_understood_by_the_firmware() {
    // The point of this test is the magic number. 21196 in param2 tells the vehicle to arm
    // regardless of its checks; a wrong value there would not fail loudly - the vehicle would
    // simply refuse, and the operator would be pressing a button that does nothing at the moment
    // they most need it. What is asserted is that the firmware treats it as a valid command
    // rather than rejecting it as malformed.
    let (link, id) = connect();
    assert!(link.send(&commands::arm(id, true, true)), "send failed");

    let ack = await_message(&link, "a forced arm acknowledgement", |message| {
        message.text.contains("ARM_DISARM")
    });
    assert!(
        !ack.text.contains("unsupported") && !ack.text.contains("command long only"),
        "the firmware did not understand a forced arm: {}",
        ack.text
    );

    link.send(&commands::arm(id, false, false));
}

/// Sets a parameter and waits for the vehicle to report the new value back.
///
/// Reading back is the whole point. `PARAM_SET` has no acknowledgement of its own: the vehicle
/// answers by broadcasting the parameter, and a write to a name the firmware does not have looks
/// exactly like one that worked. An earlier version of this test wrote `ARMING_CHECK`, which
/// ArduPilot 4.7 renamed, and passed anyway - because the vehicle it was testing against could
/// arm regardless. A test that cannot tell a no-op from a success is not a test.
fn set_param_and_confirm(link: &Link, id: VehicleId, name: &str, value: f32) -> bool {
    link.send(&commands::param_set(id, name, value));
    // Ask for it as well. The vehicle answers a successful write by broadcasting the parameter,
    // but that single message is easy to miss on a busy link - and a missed broadcast is
    // indistinguishable from a parameter that does not exist, which is the thing being tested.
    link.send(&commands::request_param_by_name(id, name));

    // Wait for the table to show the *new* value, not merely some value. The table may already
    // hold this parameter from an earlier request, and returning on the first value seen reports
    // the old one - which looks exactly like a write the vehicle refused.
        // Thirty seconds, asking again every five: a hosted CI runner at SITL's real-time speed took
    // longer than ten to answer the ninth test's write (2026-10-03), and the ask is cheap.
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut asked = Instant::now();
    while Instant::now() < deadline {
        if let Some(table) = link.params(id)
            && let Some(current) = table.get(name)
        {
            #[allow(clippy::cast_possible_truncation)]
            let read_back = current.as_f64() as f32;
            if (read_back - value).abs() < 0.001 {
                return true;
            }
        }
        if asked.elapsed() > Duration::from_secs(5) {
            link.send(&commands::request_param_by_name(id, name));
            asked = Instant::now();
        }
        wasm_thread::sleep(Duration::from_millis(100));
    }
    false
}

/// The arming-check parameter this vehicle has, trying the newer name first.
///
/// ArduPilot 4.7 renamed `ARMING_CHECK` to `ARMING_SKIPCHK` and inverted its sense. Try one, and
/// if the vehicle does not have it, try the other.
fn arming_check_param(link: &Link, id: VehicleId) -> Option<(&'static str, f32, f32)> {
    for (name, disable, restore) in [("ARMING_SKIPCHK", -1.0, 0.0), ("ARMING_CHECK", 0.0, 1.0)] {
        link.send(&commands::request_param_by_name(id, name));
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            if let Some(table) = link.params(id)
                && table.get(name).is_some()
            {
                return Some((name, disable, restore));
            }
            wasm_thread::sleep(Duration::from_millis(100));
        }
    }
    None
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_arming_check_parameter_is_one_this_firmware_actually_has() {
    // ArduPilot 4.7 renamed ARMING_CHECK to ARMING_SKIPCHK and inverted its sense: the old one was
    // a mask of checks to perform, the new one a mask of checks to skip, and -1 skips all. Writing
    // the wrong name is silently ignored, so the code tries one and falls back to the other - and
    // this asserts the vehicle under test has one of them.
    let (link, id) = connect();

    let (name, disable, restore) =
        arming_check_param(&link, id).expect("this firmware has neither arming-check parameter");

    assert!(
        set_param_and_confirm(&link, id, name, disable),
        "{name} exists but would not take {disable}"
    );
    assert!(
        set_param_and_confirm(&link, id, name, restore),
        "{name} would not take {restore} again"
    );
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn disabling_the_arming_checks_arms_a_vehicle_that_was_refusing() {
    // The magic 21196 skips the pre-arm checks and nothing else. A real board refused a forced arm
    // and listed an uncalibrated accelerometer and a bad GPS fix - those are arming checks, and
    // only the skip parameter turns them off. This is the whole force-arm path end to end.
    let (link, id) = connect();
    let handle = link.vehicle(id).expect("a state handle");

    let (name, disable, restore) =
        arming_check_param(&link, id).expect("this firmware has neither arming-check parameter");
    // Confirmed, not assumed: the assertion below is meaningless if this did nothing. An earlier
    // version of this test wrote a parameter the firmware had renamed, and passed anyway.
    assert!(
        set_param_and_confirm(&link, id, name, disable),
        "{name} would not take {disable}"
    );

    let deadline = Instant::now() + Duration::from_secs(20);
    let mut armed = false;
    while Instant::now() < deadline {
        link.send(&commands::arm(id, true, true));
        wasm_thread::sleep(Duration::from_millis(400));
        if handle.load().armed {
            armed = true;
            break;
        }
    }

    // Put the vehicle and its parameters back before asserting, so a failure here does not leave
    // SITL armed with its checks off for the next test.
    link.send(&commands::arm(id, false, true));
    set_param_and_confirm(&link, id, name, restore);

    assert!(
        armed,
        "the vehicle never armed with {name} = {disable} and a forced arm"
    );
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_refusal_comes_with_a_reason_the_pilot_can_act_on() {
    // ArduPilot explains itself in STATUSTEXT, not in the ack. "PreArm: Compass not calibrated" is
    // the difference between a pilot who fixes the problem and one who keeps pressing Arm. If a
    // fresh SITL arms cleanly there is nothing to explain, so this asserts only that when the
    // answer is a refusal, text accompanies it.
    let (link, id) = connect();
    link.send(&commands::arm(id, true, false));

    let ack = await_message(&link, "an arm acknowledgement", |message| {
        message.text.contains("ARM_DISARM")
    });

    if ack.text.contains("accepted") {
        link.send(&commands::arm(id, false, false));
        return;
    }

    let explanation = await_message(&link, "a reason for the refusal", |message| {
        message.severity.is_urgent() && !message.text.contains("ARM_DISARM")
    });
    assert!(
        !explanation.text.is_empty(),
        "a refusal with an empty explanation is no better than silence"
    );
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_mode_change_reaches_the_vehicle_and_comes_back_in_telemetry() {
    // The mode buttons are the most-used control on the screen. This proves the whole loop: the
    // number we send is the number the firmware adopts and reports back in its heartbeat.
    let (link, id) = connect();
    let handle = link.vehicle(id).expect("a state handle");
    let started = handle.load().custom_mode;
    let target = if started == MODE_GUIDED {
        MODE_STABILIZE
    } else {
        MODE_GUIDED
    };

    assert!(link.send(&commands::set_mode(id, target)), "send failed");

    let deadline = Instant::now() + Duration::from_secs(20);
    while handle.load().custom_mode != target {
        assert!(
            Instant::now() < deadline,
            "the vehicle stayed in mode {} instead of adopting {target}",
            handle.load().custom_mode
        );
        wasm_thread::sleep(Duration::from_millis(50));
    }

    assert_eq!(handle.load().custom_mode, target);
    assert_eq!(
        mp_vehicle::flight_mode_name(handle.load().vehicle_type, target),
        Some(if target == MODE_GUIDED {
            "Guided"
        } else {
            "Stabilize"
        }),
        "the generated mode table disagrees with the firmware"
    );

    link.send(&commands::set_mode(id, started));
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_real_vehicle_says_something_about_itself_on_connect() {
    // A sanity check on the message pane as a whole. ArduPilot narrates its own state - EKF
    // alignment, GPS acquisition, failsafe transitions - and a flight screen that shows none of
    // it has lost the running commentary that explains everything else on the display.
    //
    // Prompted rather than passive: much of the boot narration happens before a ground station
    // connects, so this asks for a parameter that does not exist, which ArduPilot answers with
    // text rather than silence.
    let (link, id) = connect();
    link.send(&commands::request_param_by_name(id, "NO_SUCH_PARAM"));
    link.send(&commands::arm(id, true, false));

    let message = await_message(&link, "anything at all from the vehicle", |_| true);
    assert!(!message.text.is_empty());
    assert_eq!(message.from.sysid, id.sysid);

    link.send(&commands::arm(id, false, false));
}

/// Copter's Land mode, from the generated table.
const MODE_LAND: u32 = 9;

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_user_takeoff_in_guided_is_accepted_once_armed() {
    // Resume Mission's third loop (`FlightData.cs:1587-1600`): Guided until the vehicle is in
    // it, arm until armed, then `doCommand(TAKEOFF, 0,0,0,0,0,0, alt)` - and the C# gives up
    // with "The Command failed to execute" the moment the vehicle refuses. `fly-resumemis.gui`
    // has been failing at exactly that step, so this asks the firmware the same question
    // without a window: is a user take-off accepted in Guided once armed on the ground?
    // `Mode::do_user_takeoff_U_m` (ArduCopter/takeoff.cpp) refuses when not armed, not landed,
    // in a mode without user take-off, or for a target no higher than the current altitude.
    let (link, id) = connect();
    let handle = link.vehicle(id).expect("the vehicle's state");
    let wait = |what: &str, secs: u64, check: &dyn Fn(&mp_vehicle::VehicleState) -> bool| {
        let deadline = Instant::now() + Duration::from_secs(secs);
        while !check(&handle.load()) {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            wasm_thread::sleep(Duration::from_millis(100));
        }
    };

    // Guided, asked for once a second until the vehicle reports it, as the C# loops.
    let deadline = Instant::now() + Duration::from_secs(30);
    while handle.load().custom_mode != MODE_GUIDED {
        assert!(
            Instant::now() < deadline,
            "the vehicle never entered Guided"
        );
        link.send(&commands::set_mode(id, MODE_GUIDED));
        wasm_thread::sleep(Duration::from_secs(1));
    }
    // Arm, the same way.
    let deadline = Instant::now() + Duration::from_secs(30);
    while !handle.load().armed {
        assert!(
            Instant::now() < deadline,
            "the vehicle never armed in Guided"
        );
        link.send(&commands::arm(id, true, false));
        wasm_thread::sleep(Duration::from_secs(1));
    }
    let before = handle.load();
    println!(
        "armed in Guided at {:.1} m relative, mode {}",
        before.altitude_relative.0, before.custom_mode
    );

    assert!(link.send(&commands::takeoff(id, 10.0)), "send failed");
    let ack = await_message(&link, "a take-off acknowledgement", |message| {
        message.text.contains("NAV_TAKEOFF")
    });
    println!("firmware: {}", ack.text);
    let accepted = ack.text.contains("accepted");
    if accepted {
        wait("8 m of climb", 60, &|s| s.altitude_relative.0 > 8.0);
    }

    // Down and as we found it, whatever the answer was.
    link.send(&commands::set_mode(id, MODE_LAND));
    wait("the vehicle to land and disarm", 120, &|s| !s.armed);
    link.send(&commands::set_mode(id, MODE_STABILIZE));
    wait("Stabilize", 10, &|s| s.custom_mode == MODE_STABILIZE);

    assert!(accepted, "the firmware refused the take-off: {}", ack.text);
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_user_takeoff_sent_the_instant_the_heartbeat_shows_armed() {
    // The same as above with no second's grace: the take-off goes out within 10 ms of the first
    // heartbeat that shows the vehicle armed, which is when a screen that acts on telemetry would
    // send it. ArduCopter holds `in_arming_delay` for ARMING_DELAY_SEC after arming; whether a
    // take-off asked for inside that window is refused is what this records.
    let (link, id) = connect();
    let handle = link.vehicle(id).expect("the vehicle's state");
    let deadline = Instant::now() + Duration::from_secs(30);
    while handle.load().custom_mode != MODE_GUIDED {
        assert!(
            Instant::now() < deadline,
            "the vehicle never entered Guided"
        );
        link.send(&commands::set_mode(id, MODE_GUIDED));
        wasm_thread::sleep(Duration::from_secs(1));
    }
    assert!(link.send(&commands::arm(id, true, false)), "send failed");
    let asked = Instant::now();
    let deadline = asked + Duration::from_secs(30);
    while !handle.load().armed {
        assert!(
            Instant::now() < deadline,
            "the vehicle never armed in Guided"
        );
        wasm_thread::sleep(Duration::from_millis(10));
    }
    let armed_after = asked.elapsed();
    assert!(link.send(&commands::takeoff(id, 10.0)), "send failed");
    let ack = await_message(&link, "a take-off acknowledgement", |message| {
        message.text.contains("NAV_TAKEOFF")
    });
    println!(
        "armed {armed_after:?} after the arm was sent; take-off sent at once; firmware: {}",
        ack.text
    );
    let accepted = ack.text.contains("accepted");
    let wait = |what: &str, secs: u64, check: &dyn Fn(&mp_vehicle::VehicleState) -> bool| {
        let deadline = Instant::now() + Duration::from_secs(secs);
        while !check(&handle.load()) {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            wasm_thread::sleep(Duration::from_millis(100));
        }
    };
    if accepted {
        wait("8 m of climb", 60, &|s| s.altitude_relative.0 > 8.0);
    }
    link.send(&commands::set_mode(id, MODE_LAND));
    wait("the vehicle to land and disarm", 120, &|s| !s.armed);
    link.send(&commands::set_mode(id, MODE_STABILIZE));
    wait("Stabilize", 10, &|s| s.custom_mode == MODE_STABILIZE);
    // Recorded, not asserted: the answer is the firmware's to give, and either is a finding.
    println!("accepted = {accepted}");
}
