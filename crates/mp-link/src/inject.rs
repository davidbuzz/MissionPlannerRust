//! Injecting GPS corrections into the vehicle: `MAVLinkInterface.InjectGpsData`.
//!
//! The RTK/GPS Inject page (`GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs`) reads RTCM, SBP
//! or raw bytes from a base station or a caster and hands each message to every link's
//! `InjectGpsData`, which cuts it into MAVLink messages:
//!
//! * `GPS_RTCM_DATA` (233), the newer message: 180-byte fragments, at most four, so a message of
//!   more than 720 bytes is dropped whole. `flags` is fragmented (bit 0, set when there is more
//!   than one fragment), the fragment's number (bits 1-2), and the link's sequence number modulo
//!   32 (bits 3-7), which goes up by one per message. The number of fragments is `length / 180 +
//!   1` - so a message that is a whole number of fragments long is followed by an empty one, which
//!   is how the autopilot knows it has ended - capped at four;
//! * `GPS_INJECT_DATA` (123), for a vehicle too old for the other (the page's hidden "Inject MSG
//!   Type" box unticked): 110-byte pieces, as many as the data needs, none empty, each addressed
//!   to the vehicle.
//!
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3891-3972`

use mp_mavlink_dialects::all::{GpsInjectData, GpsRtcmData, MavMessage};
use mp_vehicle::VehicleId;

/// `msglen` of the new message: the bytes one `GPS_RTCM_DATA` carries.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3904`
pub const RTCM_FRAGMENT: usize = 180;

/// The most fragments a message is cut into, and so the longest message sent: `msglen * 4`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3906-3916`
pub const RTCM_FRAGMENTS: usize = 4;

/// `msglen` of the old message: the bytes one `GPS_INJECT_DATA` carries.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3954`
pub const INJECT_CHUNK: usize = 110;

/// The messages `InjectGpsData(sysid, compid, data, length, rtcm_message)` sends for `data`, and
/// `inject_seq_no` moved on as it moves it.
///
/// `seq` is the link's `inject_seq_no`: read for the new message's flags and incremented once per
/// message sent that way. A message too long to send leaves it alone, as the C# returns before
/// the increment; so does the old message, which has no sequence number.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3897-3972`
#[must_use]
pub fn gps_inject_messages(
    target: VehicleId,
    data: &[u8],
    rtcm_message: bool,
    seq: &mut u32,
) -> Vec<MavMessage> {
    let length = data.len();
    let mut out = Vec::new();
    if rtcm_message {
        // C#: MAVLinkInterface.cs:3906-3910 - "Message too large", and nothing sent.
        if length > RTCM_FRAGMENT * RTCM_FRAGMENTS {
            return out;
        }
        // C#: MAVLinkInterface.cs:3913 - both arms of the conditional are `length / msglen + 1`:
        // a message that fills its fragments exactly gets an empty one after them.
        let packets = (length / RTCM_FRAGMENT + 1).min(RTCM_FRAGMENTS);
        for a in 0..packets {
            // C#: MAVLinkInterface.cs:3922-3932
            let mut flags: u8 = u8::from(packets > 1);
            #[allow(clippy::cast_possible_truncation)] // masked to two bits, then five
            {
                flags = flags.wrapping_add(((a & 0x3) << 1) as u8);
                flags = flags.wrapping_add(((*seq & 0x1f) << 3) as u8);
            }
            // C#: MAVLinkInterface.cs:3935-3944 - an empty 180-byte buffer, what is left copied in.
            let start = a * RTCM_FRAGMENT;
            let copy = length.saturating_sub(start).min(RTCM_FRAGMENT);
            let mut fragment = [0u8; RTCM_FRAGMENT];
            if let (Some(to), Some(from)) =
                (fragment.get_mut(..copy), data.get(start..start + copy))
            {
                to.copy_from_slice(from);
            }
            out.push(MavMessage::GpsRtcmData(GpsRtcmData {
                flags,
                #[allow(clippy::cast_possible_truncation)] // at most 180
                len: copy as u8,
                data: fragment,
            }));
        }
        // C#: MAVLinkInterface.cs:3949
        *seq = seq.wrapping_add(1);
    } else {
        // C#: MAVLinkInterface.cs:3956 - no empty piece after a whole number of them.
        let pieces = length.div_ceil(INJECT_CHUNK);
        for a in 0..pieces {
            let start = a * INJECT_CHUNK;
            let copy = (length - start).min(INJECT_CHUNK);
            let mut piece = [0u8; INJECT_CHUNK];
            if let (Some(to), Some(from)) = (piece.get_mut(..copy), data.get(start..start + copy)) {
                to.copy_from_slice(from);
            }
            out.push(MavMessage::GpsInjectData(GpsInjectData {
                target_system: target.sysid,
                target_component: target.compid,
                #[allow(clippy::cast_possible_truncation)] // at most 110
                len: copy as u8,
                data: piece,
            }));
        }
    }
    out
}

#[cfg(test)]
#[allow(clippy::cast_possible_truncation)]
mod tests {
    use super::*;

    const TARGET: VehicleId = VehicleId::new(1, 1);

    fn rtcm(message: &MavMessage) -> (u8, u8, [u8; 180]) {
        match message {
            MavMessage::GpsRtcmData(m) => (m.flags, m.len, m.data),
            other => panic!("expected GPS_RTCM_DATA, got {other:?}"),
        }
    }

    /// A message shorter than a fragment: one `GPS_RTCM_DATA`, not fragmented, fragment 0, the
    /// sequence number in the top five bits; the rest of the 180 bytes zero.
    /// `// C#: MAVLinkInterface.cs:3913-3946` by hand: length 25, nopackets = 25/180 + 1 = 1,
    /// flags = 0 + (0 << 1) + (5 << 3) = 40.
    #[test]
    fn a_short_message_is_one_fragment() {
        let data: Vec<u8> = (1..=25).collect();
        let mut seq = 5;
        let sent = gps_inject_messages(TARGET, &data, true, &mut seq);
        assert_eq!(sent.len(), 1);
        let (flags, len, bytes) = rtcm(&sent[0]);
        assert_eq!(flags, 40);
        assert_eq!(len, 25);
        assert_eq!(&bytes[..25], data.as_slice());
        assert!(bytes[25..].iter().all(|&b| b == 0));
        assert_eq!(seq, 6);
    }

    /// 400 bytes: 400/180 + 1 = 3 fragments of 180, 180 and 40, each flagged fragmented with its
    /// number, all with the same sequence number.
    /// flags = 1 + (a << 1) + (31 << 3): 249, 251, 253.
    #[test]
    fn a_long_message_is_cut_into_numbered_fragments() {
        let data: Vec<u8> = (0..400u32).map(|n| (n % 251) as u8).collect();
        let mut seq = 31;
        let sent = gps_inject_messages(TARGET, &data, true, &mut seq);
        let got: Vec<(u8, u8)> = sent.iter().map(|m| (rtcm(m).0, rtcm(m).1)).collect();
        assert_eq!(got, [(249, 180), (251, 180), (253, 40)]);
        assert_eq!(&rtcm(&sent[1]).2[..], &data[180..360]);
        assert_eq!(&rtcm(&sent[2]).2[..40], &data[360..]);
        // 31 + 1 wraps in the flags, not in the counter.
        assert_eq!(seq, 32);
        let again = gps_inject_messages(TARGET, &data[..10], true, &mut seq);
        assert_eq!(rtcm(&again[0]).0, 0, "32 & 0x1f is 0");
    }

    /// Exactly 180 bytes: 180/180 + 1 = 2 - a full fragment and an empty one after it, flags 1
    /// and 3. Exactly 720: 5, capped at 4, and no empty one. 721: dropped, and the sequence
    /// number not moved.
    #[test]
    fn whole_fragments_are_followed_by_an_empty_one_and_the_longest_is_720() {
        let mut seq = 0;
        let sent = gps_inject_messages(TARGET, &[7u8; 180], true, &mut seq);
        let got: Vec<(u8, u8)> = sent.iter().map(|m| (rtcm(m).0, rtcm(m).1)).collect();
        assert_eq!(got, [(1, 180), (3, 0)]);
        assert!(rtcm(&sent[1]).2.iter().all(|&b| b == 0));

        let sent = gps_inject_messages(TARGET, &[7u8; 720], true, &mut seq);
        let got: Vec<(u8, u8)> = sent.iter().map(|m| (rtcm(m).0, rtcm(m).1)).collect();
        assert_eq!(got, [(9, 180), (11, 180), (13, 180), (15, 180)]);
        assert_eq!(seq, 2);

        assert!(gps_inject_messages(TARGET, &[7u8; 721], true, &mut seq).is_empty());
        assert_eq!(seq, 2);

        // Nothing at all still sends the empty fragment, and counts.
        let sent = gps_inject_messages(TARGET, &[], true, &mut seq);
        assert_eq!(sent.len(), 1);
        assert_eq!((rtcm(&sent[0]).0, rtcm(&sent[0]).1), (2 << 3, 0));
    }

    /// The old message: 110-byte pieces addressed to the vehicle, none empty, the sequence number
    /// untouched. 250 bytes: 250 % 110 != 0, so 250/110 + 1 = 3 pieces of 110, 110 and 30;
    /// 220 bytes: 2 pieces.
    /// `// C#: MAVLinkInterface.cs:3953-3970`
    #[test]
    fn the_old_message_is_cut_into_addressed_pieces() {
        let data: Vec<u8> = (0..250u32).map(|n| n as u8).collect();
        let mut seq = 9;
        let target = VehicleId::new(3, 7);
        let sent = gps_inject_messages(target, &data, false, &mut seq);
        let got: Vec<(u8, u8, u8)> = sent
            .iter()
            .map(|m| match m {
                MavMessage::GpsInjectData(m) => (m.target_system, m.target_component, m.len),
                other => panic!("expected GPS_INJECT_DATA, got {other:?}"),
            })
            .collect();
        assert_eq!(got, [(3, 7, 110), (3, 7, 110), (3, 7, 30)]);
        if let MavMessage::GpsInjectData(last) = &sent[2] {
            assert_eq!(&last.data[..30], &data[220..]);
            assert!(last.data[30..].iter().all(|&b| b == 0));
        }
        assert_eq!(seq, 9);
        assert_eq!(
            gps_inject_messages(target, &[0u8; 220], false, &mut seq).len(),
            2
        );
        assert!(gps_inject_messages(target, &[], false, &mut seq).is_empty());
    }

    /// The encoded payload of one fragment, byte for byte: flags, len, then the 180 data bytes,
    /// as `mavlink_gps_rtcm_data_t` lays them out.
    #[test]
    fn the_fragment_encodes_as_the_csharp_struct() {
        let mut seq = 3;
        let sent = gps_inject_messages(TARGET, &[0xD3, 0x00, 0x13], true, &mut seq);
        let mut payload = [0u8; 255];
        let n = sent[0].encode(&mut payload);
        assert_eq!(n, 182);
        assert_eq!(&payload[..5], &[3 << 3, 3, 0xD3, 0x00, 0x13]);
        assert!(payload[5..182].iter().all(|&b| b == 0));
    }

    /// Through a running link: every sender shares the link's `inject_seq_no`, the fragments go
    /// out as frames, and a base written through a sender is in the vehicle's next snapshot.
    #[test]
    fn a_senders_injections_and_base_reach_the_wire_and_the_state() {
        use std::time::{Duration, Instant};

        use mp_mavlink::FrameDecoder;
        use mp_mavlink_dialects::all::DIALECT;
        use mp_transport::Transport as _;
        use mp_transport::testing::Loopback;
        use mp_vehicle::LatLngAlt;

        let (mut vehicle, gcs) = Loopback::pair();
        let config = crate::LinkConfig {
            send_heartbeat: false,
            stream_rate_hz: 0,
            ..crate::LinkConfig::default()
        };
        let link = crate::Link::from_transport(Box::new(gcs), config);
        vehicle
            .write_all(&crate::testing::heartbeat(0))
            .expect("heartbeat");
        let until = |what: &str, check: &dyn Fn() -> bool| {
            let deadline = Instant::now() + Duration::from_secs(5);
            while !check() {
                assert!(Instant::now() < deadline, "timed out waiting for {what}");
                std::thread::sleep(Duration::from_millis(2));
            }
        };
        until("the vehicle", &|| link.vehicles().contains(&TARGET));

        let first = link.sender();
        let second = link.sender();
        assert_eq!(first.inject_gps_data(TARGET, &[1, 2, 3], true), 1);
        assert_eq!(second.inject_gps_data(TARGET, &[0u8; 180], true), 2);

        let mut decoder = FrameDecoder::new();
        let mut heard = Vec::new();
        let mut buf = [0u8; 4096];
        let deadline = Instant::now() + Duration::from_secs(5);
        while heard.len() < 3 && Instant::now() < deadline {
            let n = vehicle.read(&mut buf).unwrap_or(0);
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(MavMessage::GpsRtcmData(m)) =
                    MavMessage::decode(frame.msgid, frame.payload)
                {
                    heard.push((m.flags, m.len));
                }
            });
        }
        // Sequence 0 from the first sender, 1 from the second: one counter per link.
        assert_eq!(heard, [(0, 3), (1 | (1 << 3), 180), (3 | (1 << 3), 0)]);

        let base = LatLngAlt {
            lat: -35.36,
            lng: 149.16,
            alt: 584.5,
        };
        first.set_base(TARGET, base);
        until("the base in the snapshot", &|| {
            link.vehicle(TARGET)
                .is_some_and(|handle| handle.load().base == base)
        });
    }
}
