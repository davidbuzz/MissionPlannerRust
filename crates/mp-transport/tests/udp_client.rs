//! The UDP client against a vehicle on 127.0.0.1, carrying real MAVLink frames from a recorded
//! flight both ways.
//!
//! The comparison with what the C# did - its reads, writes and `BytesToRead` - is
//! `csharp_goldens.rs`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]

mod common;

use std::net::{SocketAddr, UdpSocket};

use common::{decode, real_frames};
use mp_transport::{LinkUrl, Transport, UdpClientTransport};

fn a_vehicle() -> (UdpSocket, u16) {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket.set_read_timeout(Some(common::GUARD)).unwrap();
    let port = socket.local_addr().unwrap().port();
    (socket, port)
}

#[test]
fn real_frames_go_out_one_datagram_each_and_come_back_whole() {
    let frames = real_frames(300);
    let (vehicle, port) = a_vehicle();
    let mut client = UdpClientTransport::open("127.0.0.1", port).unwrap();
    client.set_read_timeout(common::GUARD).unwrap();

    let mut from = None;
    let mut buf = [0u8; 512];
    for frame in &frames {
        client.write_all(frame).unwrap();
        let (n, sender) = vehicle.recv_from(&mut buf).unwrap();
        assert_eq!(&buf[..n], frame.as_slice());
        from = Some(sender);
    }
    // The client sends from a port of its own, not the vehicle's.
    let client_address = from.unwrap();
    assert_ne!(client_address.port(), port);
    assert_eq!(client.local_addr().unwrap().port(), client_address.port());

    // Read as they come, as the link engine does: a socket's buffer holds only so many, and UDP
    // drops the rest.
    let mut got = Vec::new();
    let mut buf = [0u8; 4096];
    for frame in &frames {
        vehicle.send_to(frame, client_address).unwrap();
        let n = client.read(&mut buf).unwrap();
        assert_eq!(&buf[..n], frame.as_slice(), "one read, one datagram");
        got.extend_from_slice(&buf[..n]);
    }
    assert_eq!(decode(&got), frames);
    assert_eq!(
        client.remote(),
        Some(SocketAddr::from(([127, 0, 0, 1], port)))
    );
}

#[test]
fn a_datagram_longer_than_the_read_waits_for_the_next_read() {
    let (vehicle, port) = a_vehicle();
    let mut client = UdpClientTransport::open("127.0.0.1", port).unwrap();
    client.set_read_timeout(common::GUARD).unwrap();
    client.write_all(b"hello").unwrap();
    let (_, client_address) = vehicle.recv_from(&mut [0u8; 16]).unwrap();

    let datagram: Vec<u8> = (0..1000).map(|i| (i % 256) as u8).collect();
    vehicle.send_to(&datagram, client_address).unwrap();
    let mut got = Vec::new();
    let mut buf = [0u8; 7];
    while got.len() < datagram.len() {
        let n = client.read(&mut buf).unwrap();
        assert!(n > 0);
        got.extend_from_slice(&buf[..n]);
    }
    assert_eq!(got, datagram, "nothing cut off, nothing out of order");
}

#[test]
fn whoever_answers_is_read_and_writes_still_go_to_the_host() {
    let (vehicle, port) = a_vehicle();
    let (stranger, stranger_port) = a_vehicle();
    let mut client = UdpClientTransport::open("127.0.0.1", port).unwrap();
    client.set_read_timeout(common::GUARD).unwrap();
    client.write_all(b"hi").unwrap();
    let (_, client_address) = vehicle.recv_from(&mut [0u8; 16]).unwrap();

    stranger.send_to(b"not the host", client_address).unwrap();
    let mut buf = [0u8; 64];
    let n = client.read(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"not the host");
    assert_eq!(client.remote().unwrap().port(), stranger_port);

    client.write_all(b"still to the host").unwrap();
    let mut buf = [0u8; 64];
    let (n, _) = vehicle.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"still to the host");
    assert_eq!(client.host().port(), port);
}

#[test]
fn a_host_that_is_not_there_is_no_error_and_the_link_stays_open() {
    // UDP has no connection to lose: the C#'s IsOpen is true from Open to Close, and a failed
    // send is swallowed.
    let port = {
        let (socket, port) = a_vehicle();
        drop(socket);
        port
    };
    let mut client = UdpClientTransport::open("127.0.0.1", port).unwrap();
    client
        .set_read_timeout(std::time::Duration::from_millis(1))
        .unwrap();
    for _ in 0..3 {
        client.write_all(&[0xFD; 20]).unwrap();
    }
    let mut buf = [0u8; 16];
    // On Linux an unconnected socket hears nothing of the port being shut.
    #[cfg(target_os = "linux")]
    assert_eq!(client.read(&mut buf).unwrap(), 0);
    assert!(client.is_open());
    client.close();
    assert!(!client.is_open());
    assert_eq!(client.read(&mut buf).unwrap(), 0);
}

#[test]
fn a_multicast_host_is_heard_on_its_own_port() {
    // CommsUDPSerialConnect.cs:92-95: bound to the group's port, not to one the system picks.
    let port = {
        let (socket, port) = a_vehicle();
        drop(socket);
        port
    };
    let client = UdpClientTransport::open("239.255.77.1", port).unwrap();
    assert_eq!(client.local_addr().unwrap().port(), port);
    assert!(client.is_open());

    let unicast = UdpClientTransport::open("127.0.0.1", port).unwrap();
    assert_ne!(unicast.local_addr().unwrap().port(), port);
}

#[test]
fn a_name_is_looked_up() {
    let (vehicle, port) = a_vehicle();
    let mut client = UdpClientTransport::open("localhost", port).unwrap();
    assert!(client.host().ip().is_loopback());
    if client.host().is_ipv4() {
        client.write_all(b"by name").unwrap();
        let mut buf = [0u8; 16];
        let (n, _) = vehicle.recv_from(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"by name");
    }
}

#[test]
fn a_link_url_opens_the_udp_client() {
    let (vehicle, port) = a_vehicle();
    let url: LinkUrl = format!("udpcl:127.0.0.1:{port}").parse().unwrap();
    let mut link = mp_transport::open_url(&url).unwrap();
    assert_eq!(link.description(), format!("udpcl:127.0.0.1:{port}"));
    link.write_all(b"via url").unwrap();
    let mut buf = [0u8; 16];
    let (n, _) = vehicle.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"via url");
}
