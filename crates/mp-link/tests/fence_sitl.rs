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

/// Waits for a transfer of one list to finish, returning what it holds. A transfer still queued
/// behind the last one is waited for: until the link has picked it up, `list_transfer` still
/// holds the one before - an upload's own items, which a read-back would compare with themselves.
fn await_list(link: &Link, id: VehicleId, kind: u8, what: &str) -> Vec<MissionItem> {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if !link.list_transfer_queued(id, kind)
            && let Some(transfer) = link.list_transfer(id, kind)
        {
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

/// A `.fen` file, uploaded to a real vehicle, read back, and compared against the file.
///
/// The done-when for this item, and a different claim from the round-trip tests above. Those say
/// the transfer preserved what was sent; this says the *fence on the vehicle is the one in the
/// file*, which is what a pilot needs before flying inside it. The two differ whenever the file
/// reader and the wire encoder disagree - and they nearly did: `.fen` puts the return point on the
/// first line and the first vertex again on the last, so a reader that takes every line as a
/// vertex uploads a fence with two corners that are not in the file.
#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_fence_file_matches_what_the_vehicle_reports() {
    use mp_mission::fence_file::{FenceFile, read_fence, write_fence};

    let (link, id) = connect();

    // Written and read back through the file format, so the test exercises the format rather than
    // an in-memory structure that happens to agree with it.
    let drawn = FenceFile {
        return_point: Some(LatLon::new(-35.3630, 149.1660).expect("valid")),
        vertices: vec![
            LatLon::new(-35.3640, 149.1640).expect("valid"),
            LatLon::new(-35.3640, 149.1680).expect("valid"),
            LatLon::new(-35.3610, 149.1680).expect("valid"),
            LatLon::new(-35.3610, 149.1640).expect("valid"),
        ],
    };
    let from_file = read_fence(&write_fence(&drawn));
    assert_eq!(
        from_file, drawn,
        "the file format lost something before the upload"
    );

    let fence = FenceItem::Polygon {
        inclusion: true,
        vertices: from_file.vertices.clone(),
    };
    assert!(link.upload_list(id, fence.to_items(0), MISSION_TYPE_FENCE));
    await_list(&link, id, MISSION_TYPE_FENCE, "fence upload");

    assert!(link.download_list(id, MISSION_TYPE_FENCE));
    let read_back = await_list(&link, id, MISSION_TYPE_FENCE, "fence download");

    let rebuilt = fences_from_items(&read_back).expect("the fence should rebuild");
    let FenceItem::Polygon { vertices, .. } = &rebuilt[0] else {
        panic!("expected a polygon, got {:?}", rebuilt[0]);
    };
    let reported = FenceFile {
        // ArduPilot stores a return point only when one was uploaded; this test uploads the
        // polygon alone, so the comparison is of the vertices.
        return_point: from_file.return_point,
        vertices: vertices.clone(),
    };

    let differences = from_file.compare(&reported);
    assert!(
        differences.is_empty(),
        "the fence on the vehicle is not the one in the file: {differences:?}"
    );
}

/// Waits for a request to end, returning it.
fn await_request(link: &Link, id: mp_link::RequestId, what: &str) -> mp_link::requests::Request {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        if let Some(request) = link.request(id)
            && request.is_finished()
        {
            return request;
        }
        assert!(Instant::now() < deadline, "{what} never ended");
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Reads a parameter into the link's table, returning its value.
fn read_param(link: &Link, id: VehicleId, name: &str) -> Option<f64> {
    let request = await_request(link, link.read_param(id, name), name);
    match request.outcome() {
        Some(mp_link::requests::RequestOutcome::Accepted { value }) => value.map(|v| v.as_f64()),
        _ => None,
    }
}

/// Geo-Fence > Upload's sequence against the real vehicle, then its Download's, and the same
/// return point and three corners through the mission protocol and back.
///
/// `GeoFenceuploadToolStripMenuItem_Click` and `DoGeofencePointsUpload`: `FENCE_ACTION` 0,
/// `FENCE_TOTAL` the corners plus two, then `setFencePoint` for the return point, each corner
/// and the first corner again, `FENCE_ACTION` put back. `GeoFencedownloadToolStripMenuItem_Click`
/// reads them with `getFencePoint`. ArduCopter 4.8's SITL lists `FENCE_TOTAL` but no longer
/// answers `FENCE_POINT` or `FENCE_FETCH_POINT` - its legacy fence protocol is compiled out - so
/// there the first point times out after four fetches, as the C#'s `getFencePoint` does, and the
/// read-back is made the way the vehicle allows: through `MAV_MISSION_TYPE_FENCE`.
/// `// C#: GCSViews/FlightPlanner.cs:824-913, 3691-3900`
#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn the_geofence_menu_sequence_uploads_a_return_point_and_three_corners() {
    use mp_link::requests::{FencePointSet, RequestOutcome};

    let (link, id) = connect();
    let old_action = read_param(&link, id, "FENCE_ACTION").expect("FENCE_ACTION listed");
    let old_total = read_param(&link, id, "FENCE_TOTAL").expect("FENCE_TOTAL listed");

    let return_point = (-35.3632, 149.1652);
    let corners = [(-35.3640, 149.1640), (-35.3640, 149.1680), (-35.3610, 149.1660)];
    // points + return + close
    let count = u8::try_from(corners.len() + 2).expect("few");

    let set = |name: &str, value: f64| {
        await_request(&link, link.set_param(id, name, value, false), name).outcome()
    };
    assert!(matches!(
        set("FENCE_ACTION", 0.0),
        Some(RequestOutcome::Accepted { .. } | RequestOutcome::Unchanged)
    ));
    assert!(matches!(
        set("FENCE_TOTAL", f64::from(count)),
        Some(RequestOutcome::Accepted { .. } | RequestOutcome::Unchanged)
    ));

    let mut points = vec![return_point];
    points.extend(corners);
    points.push(corners[0]);
    let mut legacy = true;
    for (idx, (lat, lng)) in points.iter().enumerate() {
        let point = FencePointSet {
            idx: u8::try_from(idx).expect("few"),
            count,
            lat: *lat,
            lng: *lng,
        };
        let request = await_request(&link, link.set_fence_point(id, point), "a fence point");
        match request.outcome() {
            Some(RequestOutcome::Accepted { .. }) => {}
            Some(RequestOutcome::TimedOut) if idx == 0 => {
                // No legacy protocol: the point and four fetches went, and nothing came.
                assert_eq!(request.sends(), 5);
                legacy = false;
                break;
            }
            other => panic!("point {idx}: {other:?}"),
        }
    }

    if legacy {
        for (idx, (lat, lng)) in points.iter().enumerate() {
            let request = await_request(
                &link,
                link.get_fence_point(id, u8::try_from(idx).expect("few")),
                "a fence point read",
            );
            let read = request.fence_point().expect("the point read");
            assert_eq!(read.count, count);
            assert!((f64::from(read.lat) - lat).abs() < 1e-5, "{read:?}");
            assert!((f64::from(read.lng) - lng).abs() < 1e-5, "{read:?}");
        }
        // Every point read on the link lands in `MAV.fencepoints`.
        assert_eq!(link.fence_points(id).len(), points.len());
    } else {
        let request = await_request(&link, link.get_fence_point(id, 0), "a fence point read");
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
    }

    // `FENCE_ACTION` back as it was, and the total left as this test found it.
    assert!(matches!(
        set("FENCE_ACTION", old_action),
        Some(RequestOutcome::Accepted { .. } | RequestOutcome::Unchanged)
    ));
    let _ = set("FENCE_TOTAL", old_total);

    // The same return point and corners the other way the vehicle allows: the mission protocol.
    let mut items = FenceItem::ReturnPoint {
        position: LatLon::new(return_point.0, return_point.1).expect("valid"),
    }
    .to_items(0);
    items.extend(
        FenceItem::Polygon {
            inclusion: true,
            vertices: corners
                .iter()
                .map(|(lat, lng)| LatLon::new(*lat, *lng).expect("valid"))
                .collect(),
        }
        .to_items(1),
    );
    assert!(link.upload_list(id, items, MISSION_TYPE_FENCE));
    await_list(&link, id, MISSION_TYPE_FENCE, "fence upload");
    assert!(link.download_list(id, MISSION_TYPE_FENCE));
    let read_back = await_list(&link, id, MISSION_TYPE_FENCE, "fence download");
    assert_eq!(read_back.len(), 4, "{read_back:?}");
    assert_eq!(read_back[0].command, 5000);
    assert!((read_back[0].x - return_point.0).abs() < 1e-6);
    for (item, (lat, lng)) in read_back[1..].iter().zip(corners) {
        assert!((item.x - lat).abs() < 1e-6, "{item:?}");
        assert!((item.y - lng).abs() < 1e-6, "{item:?}");
    }
}
