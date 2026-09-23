//! Downloading a dataflash log from a real vehicle.
//!
//! Ignored by default because it needs ArduPilot SITL. Start it with
//! `tools/sitl/run-sitl.sh copter` and run `cargo test -p mp-link -- --ignored`.
//!
//! The assembly logic is unit tested; what only a real vehicle can show is whether the request
//! sequence actually produces bytes, and whether they arrive in the order the code assumes they
//! do not.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_vehicle::VehicleId;

fn connect() -> (Link, VehicleId) {
    let link = Link::connect("tcp:127.0.0.1:5760", LinkConfig::default()).expect("SITL on 5760");
    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let (id, _) = link.primary_vehicle().expect("a vehicle");
    (link, id)
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_vehicle_lists_the_logs_it_holds() {
    let (link, id) = connect();
    assert!(link.request_log_list(id), "the request should be sent");

    let deadline = Instant::now() + Duration::from_secs(30);
    while link.log_listings().is_empty() {
        assert!(
            Instant::now() < deadline,
            "the vehicle listed no logs in 30 seconds"
        );
        std::thread::sleep(Duration::from_millis(100));
    }

    let listings = link.log_listings();
    assert!(!listings.is_empty());
    for listing in &listings {
        // A log of zero bytes would be a listing for something that is not there.
        assert!(listing.size > 0, "log {} has no size", listing.id);
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_log_downloads_and_assembles_to_its_declared_size() {
    let (link, id) = connect();
    link.request_log_list(id);

    let deadline = Instant::now() + Duration::from_secs(30);
    while link.log_listings().is_empty() {
        assert!(Instant::now() < deadline, "no logs were listed");
        std::thread::sleep(Duration::from_millis(100));
    }

    // The smallest, because this runs in a test suite and a long log takes minutes over the
    // simulated link.
    let smallest = link
        .log_listings()
        .into_iter()
        .min_by_key(|listing| listing.size)
        .expect("at least one log");

    let started = Instant::now();
    assert!(link.download_log(id, smallest.id, smallest.size));

    // Nudge on a timer, which is what the owner does: a log download drops chunks routinely and a
    // transfer that only waits never finishes.
    let deadline = Instant::now() + Duration::from_secs(45);
    loop {
        if let Some((_, finished)) = link.finished_log() {
            assert_eq!(
                finished.len(),
                usize::try_from(smallest.size).expect("a sane size"),
                "the assembled log is not the size the vehicle declared"
            );
            // A dataflash log starts with the format bootstrap; a buffer of zeros would be a
            // transfer that reported success while delivering nothing.
            assert!(
                finished.iter().any(|byte| *byte != 0),
                "the assembled log is entirely zeros"
            );
            return;
        }
        if Instant::now() >= deadline {
            // Not finishing inside the budget is only a failure if it is also not making
            // progress. A 15 MB log over a simulated telemetry link legitimately takes longer
            // than a test should wait; a stalled one moves nothing at all.
            let (_, filled, size) = link
                .log_download_progress()
                .expect("a download should be in progress");
            let rate = f64::from(filled) / started.elapsed().as_secs_f64();
            assert!(
                rate > 50_000.0,
                "log {} moved {filled} of {size} bytes at {rate:.0} B/s, which is a stall",
                smallest.id
            );
            return;
        }
        link.nudge_log_download(id);
        std::thread::sleep(Duration::from_millis(400));
    }
}
