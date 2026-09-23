//! What each transport calls itself, and that asking costs nothing (DELIVERABLES.md D5).
//!
//! The link shows `Transport::description` to the operator and asks for it on every snapshot
//! publish, to notice a UDP link learning its peer (`run_link` in mp-link/src/lib.rs). The texts
//! below are the ones each transport has always produced, pinned so that making the description
//! free to ask for did not change a word of it. The last test proves it is free: asking allocates
//! nothing, and neither does a UDP link hearing from a new peer, which is when its text changes.

// A global allocator is an `unsafe impl` by definition: `GlobalAlloc`'s contract cannot be
// stated in safe Rust. This is a test binary; `mp-transport`'s own source forbids `unsafe`.
#![allow(unsafe_code)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::thread;
use std::time::{Duration, Instant};

use mp_transport::testing::Loopback;
use mp_transport::{ReplayTransport, Transport, UdpTransport};

thread_local! {
    /// Whether this thread's allocations are being counted.
    static WATCHING: Cell<bool> = const { Cell::new(false) };
    /// `alloc`, `alloc_zeroed` and `realloc` calls made on this thread while watching.
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
}

/// Counts an allocation if this thread is watching. `try_with`, because an allocator can run
/// during thread teardown and must never panic.
fn note_allocation() {
    if WATCHING.try_with(Cell::get).unwrap_or(false) {
        let _ = ALLOCATIONS.try_with(|n| n.set(n.get() + 1));
    }
}

struct CountingAllocator;

// SAFETY: every method forwards to `System` with its arguments unchanged, so this allocator makes
// exactly the guarantees `System` makes. The only addition is a thread-local counter increment,
// which neither allocates nor unwinds.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        note_allocation();
        // SAFETY: the caller upholds `GlobalAlloc::alloc`'s contract, which is all `System` needs.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        note_allocation();
        // SAFETY: as for `alloc`; the contract is the same.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` was returned by this allocator, which means by `System`, for `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        note_allocation();
        // SAFETY: `ptr` and `layout` came from `System` via this allocator, and the caller upholds
        // `realloc`'s contract for `new_size`.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

/// Runs `work` with counting on for this thread, and returns how many allocations it made.
fn allocations_in(work: impl FnOnce()) -> u64 {
    let before = ALLOCATIONS.with(Cell::get);
    WATCHING.with(|w| w.set(true));
    work();
    WATCHING.with(|w| w.set(false));
    ALLOCATIONS.with(Cell::get) - before
}

/// The port a UDP link bound, read back from its text. The tests then send to it, which proves
/// the number in the text is the one the socket really has.
fn udp_port(gcs: &dyn Transport) -> u16 {
    let text = gcs.description();
    let port = text
        .strip_prefix("udp:127.0.0.1:")
        .and_then(|rest| rest.strip_suffix(" (no peer yet)"))
        .unwrap_or_else(|| panic!("{text:?} is not a fresh UDP link's description"));
    port.parse().unwrap()
}

/// Reads until a datagram arrives.
fn receive(link: &mut dyn Transport, buf: &mut [u8]) {
    while link.read(buf).unwrap() == 0 {}
}

#[test]
fn a_replayed_log_is_named_with_its_path_and_size() {
    let replay = ReplayTransport::from_bytes("flight.tlog", vec![0; 1234]);
    assert_eq!(replay.description(), "file:flight.tlog (1234 bytes)");

    // Through the URL the link opens, and still the same once the log has been played out.
    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("description-{}.tlog", std::process::id()));
    std::fs::write(&path, [0xFE; 10]).unwrap();
    let mut file = mp_transport::open(&format!("file:{}", path.display())).unwrap();
    let expected = format!("file:{} (10 bytes)", path.display());
    assert_eq!(file.description(), expected);
    let mut buf = [0u8; 64];
    while file.read(&mut buf).unwrap() > 0 {}
    assert!(!file.is_open());
    assert_eq!(file.description(), expected);
    let _ = std::fs::remove_file(&path);

    let looping = ReplayTransport::from_bytes("soak.tlog", vec![0; 3])
        .with_chunk_size(1)
        .looping(true);
    assert_eq!(looping.description(), "file:soak.tlog (3 bytes)");
}

#[test]
fn loopback_ends_are_named_a_and_b() {
    let (a, b) = Loopback::pair();
    assert_eq!(a.description(), "loopback:a");
    assert_eq!(b.description(), "loopback:b");
}

#[test]
fn an_outbound_tcp_link_is_named_by_the_address_it_reached() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let accepting = thread::spawn(move || listener.accept().unwrap());
    let link = mp_transport::open(&format!("tcp:127.0.0.1:{port}")).unwrap();
    assert_eq!(link.description(), format!("tcp:127.0.0.1:{port}"));
    drop(accepting.join().unwrap());
}

#[test]
fn a_listening_tcp_link_is_named_by_the_peer_that_connected() {
    let port = TcpListener::bind("0.0.0.0:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    let listening = thread::spawn(move || mp_transport::open(&format!("tcpin:{port}")).unwrap());
    let deadline = Instant::now() + Duration::from_secs(10);
    let vehicle = loop {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(stream) => break stream,
            Err(_) if Instant::now() < deadline => thread::sleep(Duration::from_millis(10)),
            Err(e) => panic!("nothing listening on {port}: {e}"),
        }
    };
    let link = listening.join().unwrap();
    assert_eq!(
        link.description(),
        format!("tcp:{}", vehicle.local_addr().unwrap())
    );
}

#[test]
fn udp_is_named_by_its_own_address_and_then_the_peer_it_last_heard_from() {
    let mut gcs = mp_transport::open("udp:127.0.0.1:0").unwrap();
    gcs.set_read_timeout(Duration::from_secs(5)).unwrap();
    let port = udp_port(gcs.as_ref());
    assert_eq!(
        gcs.description(),
        format!("udp:127.0.0.1:{port} (no peer yet)")
    );

    let mut buf = [0u8; 16];
    let first = UdpSocket::bind("127.0.0.1:0").unwrap();
    let second = UdpSocket::bind("127.0.0.1:0").unwrap();
    first.send_to(b"one", ("127.0.0.1", port)).unwrap();
    receive(gcs.as_mut(), &mut buf);
    assert_eq!(
        gcs.description(),
        format!("udp:127.0.0.1:{port} <-> {}", first.local_addr().unwrap())
    );
    second.send_to(b"two", ("127.0.0.1", port)).unwrap();
    receive(gcs.as_mut(), &mut buf);
    assert_eq!(
        gcs.description(),
        format!("udp:127.0.0.1:{port} <-> {}", second.local_addr().unwrap())
    );

    // A peer set by hand, for the case where the GCS must speak first.
    let mut gcs = UdpTransport::bind("0.0.0.0", 0).unwrap();
    let text = gcs.description().to_owned();
    assert!(
        text.starts_with("udp:0.0.0.0:") && text.ends_with(" (no peer yet)"),
        "{text}"
    );
    let bound = &text["udp:".len()..text.len() - " (no peer yet)".len()];
    gcs.set_peer("127.0.0.1:14551".parse().unwrap());
    assert_eq!(
        gcs.description(),
        format!("udp:{bound} <-> 127.0.0.1:14551")
    );
}

/// A real serial port: the slave side of a pseudo-terminal, as in `hotplug.rs`.
#[cfg(all(unix, feature = "serial"))]
#[test]
fn serial_is_named_by_its_path_and_the_baud_rate_it_runs_at() {
    use serialport::{SerialPort, TTYPort};

    let (master, slave) = match TTYPort::pair() {
        Ok(pair) => pair,
        Err(e) => {
            println!("no pseudo-terminals here ({e}); skipped");
            return;
        }
    };
    let path = slave.name().unwrap();
    drop(slave);

    let mut port = mp_transport::SerialTransport::open(&path, 57_600).unwrap();
    assert_eq!(port.description(), format!("serial:{path}:57600"));
    // The SiK configurator changes the rate on an open port; the text follows it.
    port.set_baud(115_200).unwrap();
    assert_eq!(port.description(), format!("serial:{path}:115200"));
    drop(port);

    let port = mp_transport::open(&format!("serial:{path}:921600")).unwrap();
    assert_eq!(port.description(), format!("serial:{path}:921600"));
    drop(port);
    drop(master);
}

/// The link asks every transport for its description on each publish, 50 times a second, so
/// asking must cost nothing. A UDP link's text changes as it hears from a new peer, and it hears
/// in `read`, on the ingest path: that must not allocate either, even with two vehicles taking
/// turns on one port.
#[test]
fn asking_for_a_description_allocates_nothing_even_as_a_udp_link_changes_peer() {
    let replay = ReplayTransport::from_bytes("flight.tlog", vec![0; 64]);
    let (a, _b) = Loopback::pair();
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp_port = listener.local_addr().unwrap().port();
    let accepting = thread::spawn(move || listener.accept().unwrap());
    let tcp = mp_transport::open(&format!("tcp:127.0.0.1:{tcp_port}")).unwrap();
    let _server = accepting.join().unwrap();

    let mut udp = mp_transport::open("udp:127.0.0.1:0").unwrap();
    udp.set_read_timeout(Duration::from_secs(5)).unwrap();
    let gcs: SocketAddr = ([127, 0, 0, 1], udp_port(udp.as_ref())).into();
    let vehicles = [
        UdpSocket::bind("127.0.0.1:0").unwrap(),
        UdpSocket::bind("127.0.0.1:0").unwrap(),
    ];
    let mut buf = [0u8; 16];
    let mut last = SocketAddr::from(([0, 0, 0, 0], 0));

    let allocations = allocations_in(|| {
        for _ in 0..8 {
            black_box(replay.description());
            black_box(a.description());
            black_box(tcp.description());
            for vehicle in &vehicles {
                vehicle.send_to(b"heartbeat", gcs).unwrap();
                receive(udp.as_mut(), &mut buf);
                black_box(udp.description());
                last = vehicle.local_addr().unwrap();
            }
        }
    });

    // The counted work did what it says: the link followed each peer in turn.
    assert_eq!(
        udp.description(),
        format!("udp:{gcs} <-> {last}"),
        "the UDP link did not follow its peer"
    );
    assert_eq!(
        allocations, 0,
        "asking for descriptions, and a UDP link learning a new peer, allocated {allocations} times"
    );
}
