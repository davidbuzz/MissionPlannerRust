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

//! MAVFTP on the link: one [`MavFtp`] client per vehicle, fed by the link thread.
//!
//! The C# makes a `MAVFtp` over a `MAVLinkInterface`, which subscribes each command's handler to
//! `FILE_TRANSFER_PROTOCOL` from that vehicle (`SubscribeToPacketType(..., _sysid, _compid)`,
//! ExtLibs/ArduPilot/Mavlink/MAVFtp.cs:612; MAVLinkInterface.cs:5567-5593) and sends with
//! `sendPacket`. Here the link thread plays both parts: every `FILE_TRANSFER_PROTOCOL` from a
//! vehicle goes to that vehicle's client ([`route`]), every pass of the loop lets the clients' waits
//! run out ([`tick`]), and what they want sent is sent. What a reply means is `mp-ftp`'s.
//!
//! Replies are matched by who sent them, not by whom they are addressed to, as the C#'s
//! subscription matches them (MAVLinkInterface.cs:5541-5543).

use std::sync::Arc;
use web_time::Instant;

use mp_ftp::FtpError;
use mp_ftp::mavftp::wire::Header;
use mp_ftp::mavftp::{FtpOutcome, FtpRequest, MavFtp, Progress};
use mp_mavlink_dialects::all::{FileTransferProtocol, MavMessage};
use mp_vehicle::VehicleId;

use crate::{Link, Shared};

/// `FILE_TRANSFER_PROTOCOL` to `target` carrying `payload`.
///
/// Always addressed to the vehicle, network 0; see `mp_ftp::mavftp`'s notes for why not the C#'s
/// system 0 on a fresh client.
#[must_use]
pub fn ftp_message(target: VehicleId, payload: &Header) -> MavMessage {
    MavMessage::FileTransferProtocol(FileTransferProtocol {
        target_network: 0,
        target_system: target.sysid,
        target_component: target.compid,
        payload: payload.encode(),
    })
}

/// A `FILE_TRANSFER_PROTOCOL` from vehicle `from`: to its client, if it has one. What the client
/// wants sent is added to `out`.
pub(crate) fn route(
    shared: &Arc<Shared>,
    from: VehicleId,
    message: &FileTransferProtocol,
    now: Instant,
    out: &mut Vec<(VehicleId, Header)>,
) {
    let Ok(mut clients) = shared.ftp.lock() else {
        return;
    };
    let Some(client) = clients.get_mut(&from) else {
        return;
    };
    let mut sends = Vec::new();
    client.on_message(&message.payload, now, &mut sends);
    out.extend(sends.into_iter().map(|payload| (from, payload)));
}

/// Lets every client's wait run out, if it has. What they want sent is added to `out`.
pub(crate) fn tick(shared: &Arc<Shared>, now: Instant, out: &mut Vec<(VehicleId, Header)>) {
    let Ok(mut clients) = shared.ftp.lock() else {
        return;
    };
    let mut sends = Vec::new();
    for (id, client) in clients.iter_mut() {
        if !client.is_busy() {
            continue;
        }
        client.on_tick(now, &mut sends);
        out.extend(sends.drain(..).map(|payload| (*id, payload)));
    }
}

impl Link {
    /// Starts a MAVFTP request on a vehicle and returns at once.
    ///
    /// The vehicle's client is made the first time and kept, as Mission Planner's MAVFtp page
    /// keeps its `MAVFtp` (Controls/MavFTPUI.cs:32): its sequence numbers carry on, and once the
    /// vehicle has refused timed listings it is not asked again. False, and nothing sent, if a
    /// request is already running on this vehicle or the link has stopped.
    pub fn ftp(&self, target: VehicleId, request: FtpRequest) -> bool {
        let mut sends = Vec::new();
        {
            let Ok(mut clients) = self.shared.ftp.lock() else {
                return false;
            };
            let client = clients
                .entry(target)
                .or_insert_with(|| MavFtp::new(target, self.config.timeouts.ftp));
            if !client.start(request, Instant::now(), &mut sends) {
                return false;
            }
        }
        sends
            .iter()
            .all(|payload| self.send(&ftp_message(target, payload)))
    }

    /// Whether a request is running on this vehicle, and its last progress report.
    #[must_use]
    pub fn ftp_progress(&self, target: VehicleId) -> Option<(bool, Progress)> {
        let clients = self.shared.ftp.lock().ok()?;
        let client = clients.get(&target)?;
        Some((client.is_busy(), client.progress().clone()))
    }

    /// Takes the finished request's outcome, once there is one.
    pub fn take_ftp_outcome(&self, target: VehicleId) -> Option<Result<FtpOutcome, FtpError>> {
        self.shared
            .ftp
            .lock()
            .ok()?
            .get_mut(&target)?
            .take_outcome()
    }

    /// Asks the running request to stop, as the C#'s progress dialog's Cancel does.
    pub fn cancel_ftp(&self, target: VehicleId) {
        if let Ok(mut clients) = self.shared.ftp.lock()
            && let Some(client) = clients.get_mut(&target)
        {
            client.cancel();
        }
    }
}

#[cfg(test)]
mod tests {
    //! The FTP path through the real link thread, against [`FakeVehicle`] on the far end of an
    //! in-memory transport, with Mission Planner's retry counts and its waits divided by twenty.

    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    use mp_ftp::mavftp::testing::FakeVehicle;
    use mp_ftp::mavftp::wire::{ErrorCode, Opcode};
    use mp_ftp::mavftp::{RW_SIZE, crc_crc32};
    use mp_mavlink::{FrameDecoder, encode_v2};
    use mp_mavlink_dialects::all::{DIALECT, Heartbeat};
    use mp_transport::Transport;
    use mp_transport::testing::{Loopback, LoopbackEnd};

    use super::*;
    use crate::{LinkConfig, ProtocolTimeouts};

    const VEHICLE: VehicleId = VehicleId::new(1, 1);
    const GCS: VehicleId = VehicleId::new(255, 190);

    fn frame(from: VehicleId, seq: u8, message: &MavMessage) -> Vec<u8> {
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v2(
            &mut out,
            seq,
            from.sysid,
            from.compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .unwrap();
        out[..n].to_vec()
    }

    /// What the vehicle heard, as `(target, request)`.
    type Heard = Vec<(VehicleId, Header)>;

    /// The vehicle on a thread of its own: announces itself, then answers every
    /// `FILE_TRANSFER_PROTOCOL` with `vehicle`, losing the replies `lost` picks, until stopped.
    fn run_vehicle(
        mut end: LoopbackEnd,
        stop: Arc<AtomicBool>,
        mut vehicle: FakeVehicle,
        mut lost: impl FnMut(&Header) -> bool + Send + 'static,
    ) -> wasm_thread::JoinHandle<(FakeVehicle, Heard)> {
        wasm_thread::spawn(move || {
            let mut seq = 0u8;
            let heartbeat = MavMessage::Heartbeat(Heartbeat {
                custom_mode: 0,
                r#type: 2,
                autopilot: 3,
                base_mode: 81,
                system_status: 3,
                mavlink_version: 3,
            });
            end.write_all(&frame(VEHICLE, seq, &heartbeat)).unwrap();
            let mut decoder = FrameDecoder::new();
            let mut heard = Vec::new();
            let mut buf = [0u8; 4096];
            while !stop.load(Ordering::Acquire) {
                let n = end.read(&mut buf).unwrap_or(0);
                if n == 0 {
                    wasm_thread::sleep(Duration::from_millis(1));
                    continue;
                }
                let mut requests = Vec::new();
                decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                    if let Some(MavMessage::FileTransferProtocol(ftp)) =
                        MavMessage::decode(frame.msgid, frame.payload)
                    {
                        requests.push(ftp);
                    }
                });
                for request in requests {
                    let head = Header::decode(&request.payload);
                    heard.push((
                        VehicleId::new(request.target_system, request.target_component),
                        head.clone(),
                    ));
                    for reply in vehicle.answer(&head) {
                        if lost(&reply) {
                            continue;
                        }
                        seq = seq.wrapping_add(1);
                        let message = ftp_message(GCS, &reply);
                        end.write_all(&frame(VEHICLE, seq, &message)).unwrap();
                    }
                }
            }
            (vehicle, heard)
        })
    }

    fn link(end: LoopbackEnd) -> Link {
        let config = LinkConfig {
            send_heartbeat: false,
            stream_rate_hz: 0,
            timeouts: ProtocolTimeouts::default().faster(20),
            ..LinkConfig::default()
        };
        let link = Link::from_transport(Box::new(end), config);
        let deadline = Instant::now() + Duration::from_secs(5);
        while link.vehicle(VEHICLE).is_none() {
            assert!(Instant::now() < deadline, "the vehicle was never heard");
            wasm_thread::sleep(Duration::from_millis(1));
        }
        link
    }

    /// Runs `request` to its outcome through the link.
    fn run(link: &Link, request: FtpRequest) -> Result<FtpOutcome, FtpError> {
        assert!(link.ftp(VEHICLE, request));
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Some(outcome) = link.take_ftp_outcome(VEHICLE) {
                return outcome;
            }
            assert!(Instant::now() < deadline, "the request never finished");
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }

    fn numbered(len: usize) -> Vec<u8> {
        (0..len).map(|i| u8::try_from(i % 253).unwrap()).collect()
    }

    /// A listing, a burst read with a chunk lost on the way, and the CRC of the file, through the
    /// link: every request addressed to the vehicle, every reply routed back by who sent it.
    #[test]
    fn list_get_and_crc_through_the_link() {
        let content = numbered(3000);
        let mut vehicle = FakeVehicle::new().with_file("/APM/param.pck", &content);
        for i in 0..20 {
            vehicle = vehicle.with_file(&format!("/APM/LOGS/{i:08}.BIN"), b"log");
        }
        let (vehicle_side, gcs_side) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let mut lost_one = false;
        let script = run_vehicle(vehicle_side, Arc::clone(&stop), vehicle, move |reply| {
            if reply.req_opcode == Opcode::BURST_READ_FILE && reply.offset == 800 && !lost_one {
                lost_one = true;
                return true;
            }
            false
        });
        let link = link(gcs_side);

        let listing = run(
            &link,
            FtpRequest::List {
                path: "/APM/LOGS".to_owned(),
            },
        );
        let Ok(FtpOutcome::Listing { entries, ended }) = listing else {
            panic!("{listing:?}");
        };
        assert!(ended);
        assert_eq!(entries.len(), 20);
        assert_eq!(entries[19].name, "00000019.BIN");

        let got = run(
            &link,
            FtpRequest::Get {
                path: "/APM/param.pck".to_owned(),
                burst: true,
                readsize: RW_SIZE,
            },
        );
        let Ok(FtpOutcome::File {
            data: Some(data),
            ended: true,
        }) = got
        else {
            panic!("{got:?}");
        };
        assert_eq!(data, content);
        // The burst's last report, as MAVFtp.cs:864 makes it.
        let (busy, progress) = link.ftp_progress(VEHICLE).unwrap();
        assert!(!busy);
        assert_eq!(progress.message, "/APM/param.pck");
        assert_eq!(progress.percent, 100);

        let crc = run(
            &link,
            FtpRequest::Crc32 {
                path: "/APM/param.pck".to_owned(),
            },
        );
        assert_eq!(
            crc,
            Ok(FtpOutcome::Crc32 {
                crc: crc_crc32(0, &content),
                answered: true
            })
        );

        stop.store(true, Ordering::Release);
        let (_, heard) = script.join().unwrap();
        assert!(
            heard.iter().all(|(to, _)| *to == VEHICLE),
            "never broadcast"
        );
        let reads: Vec<u32> = heard
            .iter()
            .filter(|(_, h)| h.opcode == Opcode::READ_FILE)
            .map(|(_, h)| h.offset)
            .collect();
        assert!(
            !reads.is_empty() && reads.iter().all(|at| *at == 800),
            "{reads:?}"
        );
        // One client for the vehicle: its sequence numbers run on across requests.
        let seqs: Vec<u16> = heard.iter().map(|(_, h)| h.seq_number).collect();
        assert!(seqs.windows(2).all(|w| w[1] >= w[0]), "{seqs:?}");
    }

    /// An upload through the link, then a refusal: removing a file that is not there is
    /// `kErrFileNotFound`, reported when `kCmdRemoveFile`'s wait runs out.
    #[test]
    fn put_then_a_refused_remove_through_the_link() {
        let content = numbered(700);
        let (vehicle_side, gcs_side) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let script = run_vehicle(vehicle_side, Arc::clone(&stop), FakeVehicle::new(), |_| {
            false
        });
        let link = link(gcs_side);

        assert_eq!(
            run(
                &link,
                FtpRequest::Put {
                    path: "/APM/scripts/hello.lua".to_owned(),
                    data: content.clone(),
                }
            ),
            Ok(FtpOutcome::Uploaded)
        );
        let started = Instant::now();
        let removed = run(
            &link,
            FtpRequest::RemoveFile {
                path: "/APM/nothing".to_owned(),
            },
        );
        assert_eq!(
            removed,
            Err(FtpError::FileNotFound {
                path: "/APM/nothing".to_owned()
            })
        );
        // kCmdRemoveFile's one second, divided by twenty.
        assert!(started.elapsed() >= Duration::from_millis(50));

        stop.store(true, Ordering::Release);
        let (vehicle, _) = script.join().unwrap();
        assert_eq!(vehicle.files["/APM/scripts/hello.lua"], content);
    }

    /// `kErrNoSessionsAvailable` through the link: the client resets the sessions and opens again,
    /// and the plain read of an `@SYS` file that claims more than it has ends on end of file.
    #[test]
    fn no_sessions_available_then_a_plain_read_through_the_link() {
        let content = numbered(400);
        let mut vehicle = FakeVehicle::new().with_file("@SYS/uarts.txt", &content);
        vehicle
            .reported_sizes
            .insert("@SYS/uarts.txt".to_owned(), 100_000);
        vehicle.refuse_once = Some((
            Opcode::OPEN_FILE_RO,
            ErrorCode::NO_SESSIONS_AVAILABLE,
            mp_ftp::mavftp::wire::Errno(0),
        ));
        let (vehicle_side, gcs_side) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let script = run_vehicle(vehicle_side, Arc::clone(&stop), vehicle, |_| false);
        let link = link(gcs_side);

        let got = run(
            &link,
            FtpRequest::Get {
                path: "@SYS/uarts.txt".to_owned(),
                burst: false,
                readsize: RW_SIZE,
            },
        );
        assert_eq!(
            got,
            Ok(FtpOutcome::File {
                data: Some(content),
                ended: true
            })
        );

        stop.store(true, Ordering::Release);
        let (_, heard) = script.join().unwrap();
        let opcodes: Vec<Opcode> = heard.iter().map(|(_, h)| h.opcode).take(5).collect();
        assert_eq!(
            opcodes,
            [
                Opcode::RESET_SESSIONS,
                Opcode::OPEN_FILE_RO,
                Opcode::RESET_SESSIONS,
                Opcode::OPEN_FILE_RO,
                Opcode::READ_FILE
            ]
        );
    }
}
