//! `COMPASSMOT_STATUS` arriving over a link and handed to the Compass/Motor Calib page, as the
//! C#'s `SubscribeToPacketType(COMPASSMOT_STATUS, ...)` hands it each packet
//! (C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:29, 87-119).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_mavlink::{Message as _, encode_v2};
use mp_mavlink_dialects::all::{CompassmotStatus, Heartbeat};
use mp_transport::Transport;
use mp_transport::testing::Loopback;
use mp_vehicle::VehicleId;

fn frame<M: mp_mavlink::Message>(seq: u8, message: &M, len: usize) -> Vec<u8> {
    let mut payload = vec![0u8; len];
    message.encode(&mut payload);
    let mut frame = [0u8; 128];
    let n = encode_v2(&mut frame, seq, 1, 1, M::ID, &payload, M::CRC_EXTRA, 0).unwrap();
    frame[..n].to_vec()
}

fn heartbeat(seq: u8) -> Vec<u8> {
    let message = Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 3,
        mavlink_version: 3,
    };
    frame(seq, &message, Heartbeat::LEN)
}

fn status(seq: u8, throttle: u16, current: f32, interference: u16) -> Vec<u8> {
    let message = CompassmotStatus {
        current,
        compensationx: 0.5,
        compensationy: -0.25,
        compensationz: 1.0,
        throttle,
        interference,
    };
    frame(seq, &message, CompassmotStatus::LEN)
}

fn wait_for(what: &str, mut check: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if check() {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("timed out waiting for {what}");
}

#[test]
fn each_status_is_handed_over_once_in_order_with_its_sender() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    vehicle_side.write_all(&status(1, 250, 3.5, 12)).unwrap();
    vehicle_side.write_all(&status(2, 500, 7.25, 30)).unwrap();

    let mut taken = Vec::new();
    wait_for("both statuses", || {
        taken.extend(link.take_compassmot_status());
        taken.len() >= 2
    });
    assert_eq!(taken.len(), 2);
    assert_eq!(taken[0].0, VehicleId::new(1, 1));
    assert_eq!(taken[0].1.throttle, 250);
    assert_eq!(taken[0].1.interference, 12);
    assert!((taken[1].1.current - 7.25).abs() < 1e-6);
    assert!((taken[1].1.compensationy - -0.25).abs() < 1e-6);
    // Taken is gone: a second take has nothing new.
    std::thread::sleep(Duration::from_millis(50));
    assert!(link.take_compassmot_status().is_empty());
}
