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

//! The parameter fetch as Mission Planner does it: `getParamListMavftp`, the whole table read
//! as one file over MAVFTP, with the classic `PARAM_REQUEST_LIST` stream as the fallback.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1778-1928`
//!
//! `getParamList` (`:1778-1797`) - the button on the parameter screen, and the connect - runs
//! `getParamListMavftp`: when the vehicle's `AUTOPILOT_VERSION` capabilities include FTP and the
//! `UseMavFtpParams` setting is on (its default), `MAVFtp.GetFile("@PARAM/param.pck?withdefaults=1")`
//! in 110-byte reads (`:1856-1887`), `parampck.unpack` on what comes back, and the vehicle's table
//! replaced whole - `param.Clear()`, `TotalReported = count`, `AddRange` (`:1899-1902`). Anything
//! else - no FTP capability, a file that will not open, a pack that will not unpack, an exception
//! on the way - falls through to `getParamListAsync`, the stream with its gap recovery
//! (`:1921-1927`; here [`crate::param_download`]).
//!
//! The owner's rule (2026-09-25): MAVFTP is always tried first. So a vehicle whose capabilities
//! have not been heard yet is asked over MAVFTP too; one that has said it has no FTP is not made
//! to wait for the request to time out, as the C# does not ask it either. The C# also sends
//! `DO_SEND_BANNER` first and collects the banner lines (`:1812-1854`); the link asks for the
//! banner on its own when a vehicle appears, so that is not repeated here.
//!
//! The link thread runs the fetch ([`tick`]): it watches the FTP client for the file, unpacks it
//! and fills the table, or starts the classic download and watches that. The owner reads where
//! it is with [`crate::Link::param_fetch`].

use mp_os::Lock as _;
use std::collections::BTreeMap;
use std::sync::Arc;
use web_time::Instant;

use mp_ftp::mavftp::{FtpOutcome, FtpRequest};
use mp_mavlink::Message as _;
use mp_mavlink_dialects::all::ParamValue as ParamValueMessage;
use mp_params::{ParamTable, encode_param_id, parampck};
use mp_vehicle::VehicleId;

use crate::param_download::{ParamAction, ParamDownload};
use crate::timeouts::ProtocolTimeouts;
use crate::{Link, Shared};

/// The file `getParamListMavftp` reads.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1874`
pub const PARAM_FILE: &str = "@PARAM/param.pck?withdefaults=1";

/// `GetFile(..., true, 110)`: a burst read, 110 bytes at a time.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1874`
pub const READ_SIZE: u8 = 110;

/// `MAV_PROTOCOL_CAPABILITY_FTP`.
pub const CAPABILITY_FTP: u32 = 32;

/// How the parameters came.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FetchVia {
    /// The file over MAVFTP.
    MavFtp,
    /// The `PARAM_REQUEST_LIST` stream.
    Stream,
}

/// Where a fetch is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParamFetchState {
    /// Reading `@PARAM/param.pck` over MAVFTP.
    Ftp,
    /// The classic stream, started because MAVFTP did not do (`why`) or was not offered.
    Stream {
        /// What sent it here: the C#'s logged exception, or "no FTP capability".
        why: String,
    },
    /// The table is full.
    Complete {
        /// Which way it came.
        via: FetchVia,
        /// How many parameters.
        count: usize,
    },
    /// The stream was cancelled with holes outstanding.
    Cancelled,
}

/// One vehicle's fetch.
#[derive(Debug, Clone)]
pub struct ParamFetch {
    /// Which vehicle.
    pub target: VehicleId,
    /// Where it is.
    pub state: ParamFetchState,
    /// When it began.
    pub started: Instant,
    /// The defaults the file carried, by name, for the screens that show them. Empty over the
    /// stream, which has no defaults to give.
    pub defaults: Arc<BTreeMap<String, f64>>,
}

impl ParamFetch {
    /// Whether it has stopped, either way.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        matches!(
            self.state,
            ParamFetchState::Complete { .. } | ParamFetchState::Cancelled
        )
    }
}

impl Link {
    /// `getParamList`: the table fetched MAVFTP first, the stream as the fallback, and returns at
    /// once; the outcome is read with [`Link::param_fetch`]. A fetch already running for this
    /// vehicle starts again from nothing, as a second call to the C# does. False if nothing could
    /// be sent.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1778-1797, 1810-1928`
    pub fn fetch_params(&self, target: VehicleId) -> bool {
        let now = Instant::now();
        let no_ftp = self.vehicle(target).is_some_and(|handle| {
            let state = handle.load();
            let info = &state.autopilot_info;
            // Capabilities are known once AUTOPILOT_VERSION has been heard; before that the
            // word is 0 and FTP is tried.
            info.version != [0; 4] && info.capabilities & CAPABILITY_FTP == 0
        });
        let mut fetch = ParamFetch {
            target,
            state: ParamFetchState::Ftp,
            started: now,
            defaults: Arc::default(),
        };
        let sent = if no_ftp {
            fetch.state = ParamFetchState::Stream {
                why: "no FTP capability".to_owned(),
            };
            self.download_params(target)
        } else {
            // A request already on the client (the MAVFtp page's, say) is not interrupted: the
            // fetch falls back to the stream, as the C# does when GetFile throws.
            let started = self.ftp(
                target,
                FtpRequest::Get {
                    path: PARAM_FILE.to_owned(),
                    burst: true,
                    readsize: READ_SIZE,
                },
            );
            if started {
                true
            } else {
                fetch.state = ParamFetchState::Stream {
                    why: "the MAVFTP client is busy".to_owned(),
                };
                self.download_params(target)
            }
        };
        if let Ok(mut fetches) = self.shared.param_fetches.os_lock() {
            fetches.insert(target, fetch);
        }
        sent
    }

    /// Where a vehicle's fetch is, if one has been started.
    #[must_use]
    pub fn param_fetch(&self, target: VehicleId) -> Option<ParamFetch> {
        self.shared
            .param_fetches
            .os_lock()
            .ok()?
            .get(&target)
            .cloned()
    }

    /// Stops a fetch: the file read cancelled, or the stream, as the progress dialog's Cancel
    /// does. What arrived stays in the table.
    pub fn cancel_param_fetch(&self, target: VehicleId) {
        let running = self
            .param_fetch(target)
            .is_some_and(|fetch| !fetch.is_finished());
        if !running {
            return;
        }
        self.cancel_ftp(target);
        self.cancel_param_download(target);
        if let Ok(mut fetches) = self.shared.param_fetches.os_lock()
            && let Some(fetch) = fetches.get_mut(&target)
        {
            fetch.state = ParamFetchState::Cancelled;
        }
    }
}

/// One pass of the link loop: each fetch moved on. A fetch reading the file takes the client's
/// outcome once it has one - the table replaced from the pack, or the stream started; a fetch on
/// the stream ends when the download does. What the stream wants sent is added to `actions`, and
/// what the link's recording is to hold, to `recorded`.
pub(crate) fn tick(
    shared: &Arc<Shared>,
    timeouts: ProtocolTimeouts,
    now: Instant,
    actions: &mut Vec<(VehicleId, ParamAction)>,
    recorded: &mut Vec<Vec<u8>>,
) {
    let Ok(mut fetches) = shared.param_fetches.os_lock() else {
        return;
    };
    for (id, fetch) in fetches.iter_mut() {
        match &fetch.state {
            ParamFetchState::Ftp => {
                let outcome = {
                    let Ok(mut clients) = shared.ftp.os_lock() else {
                        continue;
                    };
                    let Some(client) = clients.get_mut(id) else {
                        continue;
                    };
                    if client.is_busy() {
                        continue;
                    }
                    client.take_outcome()
                };
                let Some(outcome) = outcome else {
                    continue;
                };
                match unpack_outcome(outcome) {
                    Ok(list) => {
                        recorded_frames(*id, &list, recorded);
                        // `param.Clear(); TotalReported = count; AddRange(mavlist)`: the table
                        // replaced whole, each entry at its place in the file.
                        // C#: MAVLinkInterface.cs:1898-1902
                        let count = u16::try_from(list.len()).unwrap_or(u16::MAX);
                        let mut table = ParamTable::new();
                        let mut defaults = BTreeMap::new();
                        for (index, entry) in list.into_iter().enumerate() {
                            if let Some(default) = entry.default {
                                defaults.insert(entry.name.clone(), default);
                            }
                            table.insert(
                                entry.name,
                                entry.value,
                                u16::try_from(index).unwrap_or(u16::MAX),
                                count,
                            );
                        }
                        let held = table.len();
                        if let Ok(mut tables) = shared.params.os_lock() {
                            tables.insert(*id, table);
                        }
                        fetch.defaults = Arc::new(defaults);
                        fetch.state = ParamFetchState::Complete {
                            via: FetchVia::MavFtp,
                            count: held,
                        };
                    }
                    Err(why) => {
                        // `log.Error(e)` and `return await getParamListAsync(...)`.
                        // C#: MAVLinkInterface.cs:1921-1927
                        if let Ok(mut downloads) = shared.param_downloads.os_lock() {
                            let download = ParamDownload::new(*id, timeouts, now);
                            actions.push((*id, download.begin()));
                            downloads.insert(*id, download);
                        }
                        fetch.state = ParamFetchState::Stream { why };
                    }
                }
            }
            ParamFetchState::Stream { .. } => {
                let Ok(downloads) = shared.param_downloads.os_lock() else {
                    continue;
                };
                let Some(download) = downloads.get(id) else {
                    continue;
                };
                if !download.is_finished() {
                    continue;
                }
                let cancelled = matches!(
                    download.state(),
                    crate::param_download::ParamDownloadState::Cancelled
                );
                drop(downloads);
                fetch.state = if cancelled {
                    ParamFetchState::Cancelled
                } else {
                    let count = shared
                        .params
                        .os_lock()
                        .ok()
                        .and_then(|tables| tables.get(id).map(ParamTable::len))
                        .unwrap_or(0);
                    ParamFetchState::Complete {
                        via: FetchVia::Stream,
                        count,
                    }
                };
            }
            ParamFetchState::Complete { .. } | ParamFetchState::Cancelled => {}
        }
    }
}

/// The pack's parameters as the `PARAM_VALUE`s `getParamListMavftp` hands `SaveToTlog`, one per
/// parameter in the file's order and from the vehicle: `(float)a.Value`, the count, index 0, the
/// name and the type, framed by a `MavlinkParse` made for them, so numbered from 0 - MAVLink 1,
/// or 2 for a vehicle whose id is over 255 (upstream's e6454ccdd).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1903-1912; ExtLibs/Mavlink/MavlinkParse.cs:242-330`
fn recorded_frames(id: VehicleId, list: &[parampck::PackedParam], recorded: &mut Vec<Vec<u8>>) {
    // `(ushort)mavlist.Count` and `(byte)packetcount`, both unchecked.
    #[allow(clippy::cast_possible_truncation)]
    let count = list.len() as u16;
    for (sequence, entry) in list.iter().enumerate() {
        let value = entry.value;
        // `GetValue` reads the pack's own type: an integer exactly, a float through its seven
        // significant digits. C#: ExtLibs/Mavlink/MAVLinkParam.cs:109-136
        #[allow(clippy::cast_possible_truncation)]
        let param_value = if value.param_type().is_integer() {
            value.to_param_value_field()
        } else {
            value.as_f64() as f32
        };
        let message = ParamValueMessage {
            param_value,
            param_count: count,
            param_index: 0,
            param_id: encode_param_id(&entry.name),
            param_type: value.param_type().to_wire(),
        };
        let mut payload = [0u8; ParamValueMessage::LEN];
        let len = message.encode(&mut payload);
        let Some(payload) = payload.get(..len) else {
            continue;
        };
        #[allow(clippy::cast_possible_truncation)]
        let sequence = sequence as u8;
        let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let written = match u8::try_from(id.sysid) {
            Ok(sysid) => mp_mavlink::encode_v1(
                &mut frame,
                sequence,
                sysid,
                id.compid,
                ParamValueMessage::ID,
                payload,
                ParamValueMessage::CRC_EXTRA,
            ),
            Err(_) => mp_mavlink::encode_v2(
                &mut frame,
                sequence,
                id.sysid,
                id.compid,
                ParamValueMessage::ID,
                payload,
                ParamValueMessage::CRC_EXTRA,
                0,
            ),
        };
        if let Some(bytes) = written.ok().and_then(|n| frame.get(..n)) {
            recorded.push(bytes.to_vec());
        }
    }
}

/// The file's parameters, or why the C# would have logged and fallen back: `GetFile` returned
/// null or nothing (`paramfile != null && paramfile.Length > 0`), the read failed, or `unpack`
/// returned null.
fn unpack_outcome(
    outcome: Result<FtpOutcome, mp_ftp::FtpError>,
) -> Result<Vec<parampck::PackedParam>, String> {
    match outcome {
        Ok(FtpOutcome::File {
            data: Some(data),
            ended: true,
        }) if !data.is_empty() => parampck::unpack(&data).map_err(|error| error.to_string()),
        Ok(FtpOutcome::File { ended: false, .. }) => {
            Err("the MAVFTP read did not finish".to_owned())
        }
        Ok(FtpOutcome::File { .. }) => Err("param.pck was empty".to_owned()),
        Ok(other) => Err(format!("unexpected MAVFTP outcome: {other:?}")),
        Err(error) => Err(error.to_string()),
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::indexing_slicing,
        clippy::cast_possible_truncation
    )]

    use super::*;
    use mp_mavlink_dialects::all::DIALECT;
    use mp_params::{ParamType, ParamValue};

    fn entry(name: &str, value: f32, kind: ParamType) -> parampck::PackedParam {
        parampck::PackedParam {
            name: name.to_owned(),
            value: ParamValue::from_ardupilot(value, kind),
            default: None,
        }
    }

    /// A vehicle whose id is over 255 has its parameters recorded as MAVLink 2, its whole id in
    /// the header (`GenerateMAVLinkPacket20`, upstream's e6454ccdd); one under it, as MAVLink 1.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1910-1912`
    #[test]
    fn a_wide_vehicles_parameters_are_recorded_as_mavlink_2() {
        let list = [
            entry("ARMING_CHECK", 1.0, ParamType::Int32),
            entry("WPNAV_SPEED", 0.1, ParamType::Real32),
        ];
        let mut recorded = Vec::new();
        recorded_frames(VehicleId::new(70_000, 1), &list, &mut recorded);
        assert_eq!(recorded.len(), 2);
        for (n, bytes) in recorded.iter().enumerate() {
            let (frame, used) = mp_mavlink::parse(bytes, &DIALECT).unwrap();
            assert_eq!(used, bytes.len());
            assert_eq!(frame.version, mp_mavlink::MavVersion::V2);
            assert_eq!(frame.incompat_flags, mp_mavlink::INCOMPAT_FLAG_SYSID32);
            assert_eq!((frame.sysid, frame.compid, frame.seq), (70_000, 1, n as u8));
            assert_eq!(frame.msgid, ParamValueMessage::ID);
        }
        let (frame, _) = mp_mavlink::parse(&recorded[1], &DIALECT).unwrap();
        let mut payload = [0u8; ParamValueMessage::LEN];
        frame.payload_into(&mut payload);
        let value = ParamValueMessage::decode(&payload);
        // `RoundToSignificantDigits(0.1f, 7)`, then `(float)`.
        assert_eq!(value.param_value, 0.1);
        assert_eq!(value.param_type, ParamType::Real32.to_wire());

        recorded.clear();
        recorded_frames(VehicleId::new(255, 1), &list, &mut recorded);
        let (frame, _) = mp_mavlink::parse(&recorded[0], &DIALECT).unwrap();
        assert_eq!(
            (frame.version, frame.sysid),
            (mp_mavlink::MavVersion::V1, 255)
        );
    }
}
