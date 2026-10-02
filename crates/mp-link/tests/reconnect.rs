//! A link whose transport closes under it opens another and keeps what it knew (the owner's
//! ruling, PLAN.md section 12 D23): the vehicle stays listed, the link stays running, nothing is
//! asked; without a way to open again - an in-memory double, a replay - the link ends as it did.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::io;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mp_link::testing::heartbeat;
use mp_link::{Link, LinkConfig, RECONNECT_INTERVAL, Reopen};
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_transport::{OpenError, Transport};

/// Polls `what` every 10 ms until it holds or `patience` runs out.
fn within(patience: Duration, what: impl Fn() -> bool) -> bool {
    let deadline = Instant::now() + patience;
    while Instant::now() < deadline {
        if what() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    what()
}

/// A factory that fails `refusals` times - "not yet", as a port that is still gone - and then
/// hands over `fresh`, once.
fn reopening_after(refusals: u32, fresh: LoopbackEnd, attempts: &Arc<AtomicU32>) -> Reopen {
    let attempts = Arc::clone(attempts);
    let fresh = Arc::new(Mutex::new(Some(fresh)));
    Box::new(move || {
        let attempt = attempts.fetch_add(1, Ordering::AcqRel) + 1;
        if attempt <= refusals {
            return Err(OpenError::io(
                "opening again",
                io::Error::new(io::ErrorKind::NotFound, "not yet"),
            ));
        }
        fresh
            .lock()
            .unwrap()
            .take()
            .map(|end| Box::new(end) as Box<dyn Transport>)
            .ok_or_else(|| {
                OpenError::io(
                    "opening again",
                    io::Error::new(io::ErrorKind::NotFound, "already taken"),
                )
            })
    })
}

fn quiet() -> LinkConfig {
    LinkConfig {
        send_heartbeat: false,
        record_path: None,
        ..LinkConfig::default()
    }
}

#[test]
fn a_closed_transport_is_opened_again_and_the_vehicle_kept() {
    let (first, mut vehicle_first) = Loopback::pair();
    let (second, mut vehicle_second) = Loopback::pair();
    let attempts = Arc::new(AtomicU32::new(0));
    let link = Link::from_transport_reopening(
        Box::new(first),
        quiet(),
        Some(reopening_after(2, second, &attempts)),
    );

    vehicle_first.write_all(&heartbeat(0)).unwrap();
    assert!(
        within(Duration::from_secs(3), || !link.vehicles().is_empty()),
        "the vehicle is heard over the first transport"
    );
    assert!(!link.reconnecting());
    assert_eq!(link.reconnects(), 0);

    // The cable out: the link's next read fails, and it says so while it tries again.
    let started = Instant::now();
    vehicle_first.disconnect();
    assert!(
        within(Duration::from_secs(2), || link.reconnecting()),
        "the loss is noticed"
    );
    assert!(link.is_running(), "a reconnecting link is still a link");
    assert!(
        !link.vehicles().is_empty(),
        "what the link knew is kept while it reconnects"
    );
    assert!(
        within(Duration::from_secs(1), || link.reconnect_error().is_some()),
        "the refused open is reported"
    );
    assert!(link.reconnect_error().unwrap().contains("not yet"));

    // Two refusals a second apart, then the second transport takes over.
    assert!(
        within(RECONNECT_INTERVAL * 4, || link.reconnects() == 1),
        "the third try opens: attempts {}",
        attempts.load(Ordering::Acquire)
    );
    let took = started.elapsed();
    assert!(
        took >= RECONNECT_INTERVAL * 2 && took < RECONNECT_INTERVAL * 4,
        "two intervals waited between the three tries, not {took:?}"
    );
    assert_eq!(attempts.load(Ordering::Acquire), 3);
    assert!(!link.reconnecting());
    assert_eq!(link.reconnect_attempts(), 0);
    assert!(link.reconnect_error().is_none());
    assert!(link.is_running());

    // Frames flow again over the fresh transport, to the same vehicle entry.
    let before = link.frames_received();
    vehicle_second.write_all(&heartbeat(1)).unwrap();
    vehicle_second.write_all(&heartbeat(2)).unwrap();
    assert!(
        within(Duration::from_secs(3), || link.frames_received()
            >= before + 2),
        "frames arrive over the second transport"
    );
    assert_eq!(
        link.vehicles().len(),
        1,
        "the same vehicle, not a second entry"
    );
    assert_eq!(link.reconnects(), 1);
}

#[test]
fn a_link_with_nothing_to_reopen_ends_as_before() {
    let (end, mut vehicle) = Loopback::pair();
    let link = Link::from_transport(Box::new(end), quiet());
    vehicle.write_all(&heartbeat(0)).unwrap();
    assert!(within(Duration::from_secs(3), || !link
        .vehicles()
        .is_empty()));
    vehicle.disconnect();
    assert!(
        within(Duration::from_secs(2), || !link.is_running()),
        "a replay or an in-memory link ends when its transport closes"
    );
    assert!(!link.reconnecting());
    assert_eq!(link.reconnects(), 0);
}

#[test]
fn closing_a_reconnecting_link_does_not_wait_for_the_next_try() {
    let (end, mut vehicle) = Loopback::pair();
    let attempts = Arc::new(AtomicU32::new(0));
    let attempts_seen = Arc::clone(&attempts);
    // Every try refused: the link would wait for ever.
    let never: Reopen = Box::new(move || {
        attempts_seen.fetch_add(1, Ordering::AcqRel);
        Err(OpenError::io(
            "opening again",
            io::Error::new(io::ErrorKind::NotFound, "gone"),
        ))
    });
    let mut link = Link::from_transport_reopening(Box::new(end), quiet(), Some(never));
    vehicle.disconnect();
    assert!(within(Duration::from_secs(2), || link.reconnecting()));
    let started = Instant::now();
    link.close();
    assert!(
        started.elapsed() < RECONNECT_INTERVAL,
        "the close returned within the interval, after {:?}",
        started.elapsed()
    );
    assert!(!link.is_running());
    assert!(!link.reconnecting(), "a closed link is not reconnecting");
    assert!(attempts.load(Ordering::Acquire) >= 1);
}
