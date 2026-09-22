//! Transport behaviour, including the failure modes that only show up on real links.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::Duration;

use mp_transport::testing::{Fault, Loopback};
use mp_transport::{LinkUrl, ReplayTransport, Transport, UrlError};

#[test]
fn link_urls_parse_the_forms_users_actually_type() {
    let cases = [
        (
            "serial:/dev/ttyACM0:115200",
            LinkUrl::Serial {
                path: "/dev/ttyACM0".into(),
                baud: 115_200,
            },
        ),
        (
            "serial:/dev/ttyUSB0",
            LinkUrl::Serial {
                path: "/dev/ttyUSB0".into(),
                baud: 115_200,
            },
        ),
        (
            "serial:COM3:57600",
            LinkUrl::Serial {
                path: "COM3".into(),
                baud: 57_600,
            },
        ),
        (
            "serial:COM12",
            LinkUrl::Serial {
                path: "COM12".into(),
                baud: 115_200,
            },
        ),
        (
            "tcp:127.0.0.1:5760",
            LinkUrl::Tcp {
                host: "127.0.0.1".into(),
                port: 5760,
            },
        ),
        (
            "tcp:sitl.local",
            LinkUrl::Tcp {
                host: "sitl.local".into(),
                port: 5760,
            },
        ),
        ("tcpin:5762", LinkUrl::TcpListen { port: 5762 }),
        (
            "udp:0.0.0.0:14550",
            LinkUrl::Udp {
                bind: "0.0.0.0".into(),
                port: 14_550,
            },
        ),
        (
            "udp:14550",
            LinkUrl::Udp {
                bind: "0.0.0.0".into(),
                port: 14_550,
            },
        ),
        (
            "file:flight.tlog",
            LinkUrl::File {
                path: "flight.tlog".into(),
            },
        ),
        (
            "  tcp:127.0.0.1:5760  ",
            LinkUrl::Tcp {
                host: "127.0.0.1".into(),
                port: 5760,
            },
        ),
        (
            "TCP:127.0.0.1:5760",
            LinkUrl::Tcp {
                host: "127.0.0.1".into(),
                port: 5760,
            },
        ),
    ];
    for (input, expected) in cases {
        assert_eq!(
            input.parse::<LinkUrl>().unwrap(),
            expected,
            "parsing {input:?}"
        );
    }
}

#[test]
fn link_urls_round_trip_through_display() {
    for input in [
        "serial:/dev/ttyACM0:115200",
        "tcp:127.0.0.1:5760",
        "tcpin:5762",
        "udp:0.0.0.0:14550",
        "file:x.tlog",
    ] {
        let parsed: LinkUrl = input.parse().unwrap();
        assert_eq!(parsed.to_string(), input);
        assert_eq!(parsed.to_string().parse::<LinkUrl>().unwrap(), parsed);
    }
}

#[test]
fn bad_link_urls_explain_themselves() {
    assert!(matches!(
        "/dev/ttyACM0".parse::<LinkUrl>(),
        Err(UrlError::MissingScheme(_))
    ));
    assert!(matches!(
        "ftp:host:21".parse::<LinkUrl>(),
        Err(UrlError::UnknownScheme(_))
    ));
    assert!(matches!(
        "tcp:host:notaport".parse::<LinkUrl>(),
        Err(UrlError::BadNumber(_))
    ));
    assert!(matches!(
        "file:".parse::<LinkUrl>(),
        Err(UrlError::Malformed(_))
    ));
}

#[test]
fn loopback_moves_bytes_both_ways() {
    let (mut a, mut b) = Loopback::pair();
    a.write_all(b"hello").unwrap();
    let mut buf = [0u8; 16];
    assert_eq!(b.read(&mut buf).unwrap(), 5);
    assert_eq!(&buf[..5], b"hello");

    b.write_all(b"world").unwrap();
    assert_eq!(a.read(&mut buf).unwrap(), 5);
    assert_eq!(&buf[..5], b"world");
}

#[test]
fn reading_an_idle_link_returns_zero_not_an_error() {
    // A timeout is an idle tick, not a failure. Treating it as an error is how link code ends up
    // reconnecting every 100 ms.
    let (mut a, _b) = Loopback::pair();
    let mut buf = [0u8; 16];
    assert_eq!(a.read(&mut buf).unwrap(), 0);
    assert!(a.is_open());
}

#[test]
fn writing_to_a_disconnected_link_fails_loudly() {
    let (mut a, _b) = Loopback::pair();
    a.disconnect();
    assert!(a.write_all(b"x").is_err());
    assert!(!a.is_open());
}

#[test]
fn fault_injection_corrupts_the_stream_as_configured() {
    let (mut a, b) = Loopback::pair();
    let mut b = b.with_fault(Fault {
        drop_every: 3,
        ..Fault::default()
    });
    a.write_all(&[1, 2, 3, 4, 5, 6]).unwrap();

    let mut buf = [0u8; 16];
    let n = b.read(&mut buf).unwrap();
    assert_eq!(
        &buf[..n],
        &[1, 2, 4, 5],
        "every third byte should be dropped"
    );
}

#[test]
fn replay_returns_the_recorded_bytes_exactly() {
    let data: Vec<u8> = (0..=255u8).cycle().take(4096).collect();
    let mut replay = ReplayTransport::from_bytes("synthetic", data.clone()).with_chunk_size(37);

    let mut out = Vec::new();
    let mut buf = [0u8; 512];
    loop {
        let n = replay.read(&mut buf).unwrap();
        if n == 0 {
            break;
        }
        assert!(n <= 37, "chunk size must be respected, got {n}");
        out.extend_from_slice(&buf[..n]);
    }
    assert_eq!(out, data);
    assert!(replay.is_exhausted());
    assert!(
        !replay.is_open(),
        "an exhausted, non-looping replay is closed"
    );
}

#[test]
fn looping_replay_never_runs_dry() {
    let mut replay = ReplayTransport::from_bytes("synthetic", vec![1, 2, 3]).looping(true);
    let mut buf = [0u8; 8];
    let mut total = 0;
    for _ in 0..100 {
        total += replay.read(&mut buf).unwrap();
    }
    assert_eq!(total, 300);
    assert!(replay.is_open());
}

#[test]
fn tcp_round_trips_over_localhost() {
    use std::net::TcpListener;
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();

    let server = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        use std::io::{Read, Write};
        let mut buf = [0u8; 16];
        let n = stream.read(&mut buf).unwrap();
        stream.write_all(&buf[..n]).unwrap();
    });

    let mut client = mp_transport::TcpTransport::connect("127.0.0.1", port).unwrap();
    client.set_read_timeout(Duration::from_secs(5)).unwrap();
    client.write_all(b"ping").unwrap();

    let mut buf = [0u8; 16];
    let mut got = 0;
    while got == 0 {
        got = client.read(&mut buf).unwrap();
    }
    assert_eq!(&buf[..got], b"ping");
    assert!(client.description().starts_with("tcp:"));
    server.join().unwrap();
}

#[test]
fn udp_learns_its_peer_from_the_first_datagram() {
    use std::net::UdpSocket;
    let mut gcs = mp_transport::UdpTransport::bind("127.0.0.1", 0).unwrap();
    gcs.set_read_timeout(Duration::from_secs(5)).unwrap();
    let gcs_port: u16 = gcs
        .description()
        .split(':')
        .nth(2)
        .and_then(|s| s.split(' ').next())
        .and_then(|s| s.parse().ok())
        .unwrap();

    assert!(gcs.peer().is_none(), "no peer before the first packet");
    // Writing with no peer must not error - it is the normal startup state.
    gcs.write_all(b"ignored").unwrap();

    let vehicle = UdpSocket::bind("127.0.0.1:0").unwrap();
    vehicle
        .send_to(b"heartbeat", ("127.0.0.1", gcs_port))
        .unwrap();

    let mut buf = [0u8; 32];
    let mut got = 0;
    while got == 0 {
        got = gcs.read(&mut buf).unwrap();
    }
    assert_eq!(&buf[..got], b"heartbeat");
    assert!(
        gcs.peer().is_some(),
        "peer must be learned from the received datagram"
    );

    // Now a reply reaches the vehicle.
    gcs.write_all(b"ack").unwrap();
    vehicle
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let (n, _) = vehicle.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"ack");
}

#[cfg(feature = "serial")]
#[test]
fn serial_enumeration_is_safe_on_a_machine_with_no_hardware() {
    // Must not panic and must not error: a CI runner with no serial ports is normal.
    let ports = mp_transport::list_ports();
    for port in &ports {
        assert!(!port.name.is_empty());
        assert!(!port.label().is_empty());
    }
}
