//! Geofence and rally point transfer against a real vehicle.
//!
//! Ignored by default because it needs ArduPilot SITL. Start it with
//! `tools/sitl/run-sitl.sh copter` and run `cargo test -p mp-link -- --ignored`.
//!
//! This is the test that matters for the mission_type work. Fences, rally points and the mission
//! share one protocol and one set of messages, distinguished only by a single field, and the only
//! way to know we distinguish them correctly is to ask real firmware.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::mission_transfer::TransferState;
use mp_link::{Link, LinkConfig};
use mp_mission::fence::{FenceItem, MISSION_TYPE_FENCE, MISSION_TYPE_RALLY, RallyPoint};
use mp_mission::{MISSION_TYPE_MISSION, MissionItem, fences_from_items};
use mp_units::LatLon;
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

/// Waits for a transfer of one list to finish, returning what it holds.
fn await_list(link: &Link, id: VehicleId, kind: u8, what: &str) -> Vec<MissionItem> {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(transfer) = link.list_transfer(id, kind) {
            match transfer.state() {
                TransferState::Complete => return transfer.items().to_vec(),
                TransferState::Failed(err) => panic!("{what} transfer failed: {err}"),
                _ => {}
            }
        }
        assert!(Instant::now() < deadline, "{what} transfer timed out");
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// A square fence around SITL's default home at the Canberra Model Aircraft Club.
fn square_fence() -> FenceItem {
    FenceItem::Polygon {
        inclusion: true,
        vertices: vec![
            LatLon::new(-35.3640, 149.1640).expect("valid"),
            LatLon::new(-35.3640, 149.1680).expect("valid"),
            LatLon::new(-35.3610, 149.1680).expect("valid"),
            LatLon::new(-35.3610, 149.1640).expect("valid"),
        ],
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_geofence_uploaded_to_a_real_vehicle_reads_back_unchanged() {
    let (link, id) = connect();

    let fence = square_fence();
    fence.validate().expect("the fence should be valid");
    let items = fence.to_items(0);
    assert_eq!(items.len(), 4);

    assert!(
        link.upload_list(id, items.clone(), MISSION_TYPE_FENCE),
        "the upload should be accepted for queueing"
    );
    await_list(&link, id, MISSION_TYPE_FENCE, "fence upload");

    assert!(link.download_list(id, MISSION_TYPE_FENCE));
    let read_back = await_list(&link, id, MISSION_TYPE_FENCE, "fence download");

    let rebuilt = fences_from_items(&read_back).expect("the fence should rebuild");
    assert_eq!(rebuilt.len(), 1, "expected one polygon, got {rebuilt:?}");
    match (&rebuilt[0], &fence) {
        (
            FenceItem::Polygon {
                inclusion: got_inclusion,
                vertices: got,
            },
            FenceItem::Polygon {
                inclusion: sent_inclusion,
                vertices: sent,
            },
        ) => {
            assert_eq!(got_inclusion, sent_inclusion);
            assert_eq!(got.len(), sent.len());
            for (got, sent) in got.iter().zip(sent) {
                // The wire carries degrees times 1e7, so a centimetre is the most that can be
                // expected and far more than a fence needs.
                assert!(
                    (got.latitude() - sent.latitude()).abs() < 1e-6,
                    "{got:?} vs {sent:?}"
                );
                assert!(
                    (got.longitude() - sent.longitude()).abs() < 1e-6,
                    "{got:?} vs {sent:?}"
                );
            }
        }
        (got, sent) => panic!("shape changed in transit: sent {sent:?}, got {got:?}"),
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn rally_points_round_trip_too() {
    let (link, id) = connect();

    let rally = RallyPoint {
        position: LatLon::new(-35.3625, 149.1655).expect("valid"),
        altitude: 60.0,
        break_altitude: None,
    };
    let items = vec![rally.to_item(0)];

    assert!(link.upload_list(id, items, MISSION_TYPE_RALLY));
    await_list(&link, id, MISSION_TYPE_RALLY, "rally upload");

    assert!(link.download_list(id, MISSION_TYPE_RALLY));
    let read_back = await_list(&link, id, MISSION_TYPE_RALLY, "rally download");

    assert_eq!(read_back.len(), 1);
    assert!(
        (read_back[0].x - -35.3625).abs() < 1e-6,
        "{:?}",
        read_back[0]
    );
    assert!(
        (read_back[0].y - 149.1655).abs() < 1e-6,
        "{:?}",
        read_back[0]
    );
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_fence_does_not_end_up_in_the_flight_plan() {
    // The whole reason mission_type had to be threaded through. These share one protocol and one
    // set of messages; routing by vehicle alone would let a fence's MISSION_COUNT be answered as
    // if it were the mission's, and the geofence would be written into the flight plan.
    let (link, id) = connect();

    // A mission of two waypoints, and a four-vertex fence.
    let waypoints: Vec<MissionItem> = (0..2)
        .map(|seq| MissionItem {
            seq,
            frame: 3,
            command: 16,
            autocontinue: 1,
            x: -35.363 - f64::from(seq) * 0.001,
            y: 149.165,
            z: 50.0,
            ..MissionItem::default()
        })
        .collect();

    assert!(link.upload_mission(id, waypoints.clone()));
    await_list(&link, id, MISSION_TYPE_MISSION, "mission upload");

    assert!(link.upload_list(id, square_fence().to_items(0), MISSION_TYPE_FENCE));
    await_list(&link, id, MISSION_TYPE_FENCE, "fence upload");

    // The mission must still be the mission.
    assert!(link.download_mission(id));
    let mission = await_list(&link, id, MISSION_TYPE_MISSION, "mission download");

    let fence_commands = mission
        .iter()
        .filter(|item| item.command >= 5000 && item.command < 5100)
        .count();
    assert_eq!(
        fence_commands, 0,
        "fence items leaked into the flight plan: {mission:?}"
    );
    // ArduPilot prepends home as item 0, so the two waypoints arrive as items 1 and 2.
    assert!(
        mission.len() >= waypoints.len(),
        "the mission lost items: {mission:?}"
    );
}
