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

use std::time::{Duration, Instant};

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
        std::thread::sleep(Duration::from_millis(100));
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
        std::thread::sleep(Duration::from_millis(50));
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
        std::thread::sleep(Duration::from_millis(50));
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
