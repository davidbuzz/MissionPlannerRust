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

//! The mission upload and download state machine.
//!
//! Replaces the transfer half of `MAVLinkInterface`'s waypoint handling: `getWPCountAsync`,
//! `getWPAsync`, `setWPTotalAsync`, `setWPAsync` and `setWPACK`, and the loops in
//! `ExtLibs/ArduPilot/mav_mission.cs` that drive them.
//!
//! # Why this is a state machine and not a loop
//!
//! The MAVLink mission protocol is lock-step and driven from both ends: a download is
//! `MISSION_REQUEST_LIST` → `MISSION_COUNT` → N × (`MISSION_REQUEST_INT` → `MISSION_ITEM_INT`) →
//! `MISSION_ACK`, and an upload is the mirror image where the *vehicle* asks for each item. Either
//! side can drop a packet, and a vehicle will happily re-request an item it already sent. Writing
//! this as a blocking loop means a dropped `MISSION_ITEM_INT` stalls the link until a timeout,
//! and re-requests arrive out of order and confuse the sequence.
//!
//! As a state machine driven by the link thread, a repeat request is just answered again.
//!
//! # What each step waits for, and how often it asks again
//!
//! Every wait and count is the C#'s, from [`ProtocolTimeouts`]: the list request 700 ms × 6
//! retries, each item request 2500 ms × 5, the count 700 ms × 3, each item sent 450 ms × 10. The
//! retry counter resets whenever the vehicle makes progress, as the C#'s does by starting each
//! `getWP`/`setWP` call with a fresh `retrys`.
//!
//! # How an upload ends
//!
//! `mav_mission.upload` (C#: ExtLibs/ArduPilot/mav_mission.cs:88-151) reads the vehicle's
//! `MISSION_ACK` and decides, per `MAV_MISSION_RESULT`:
//!
//! | result | C# | here |
//! |---|---|---|
//! | `ACCEPTED` | done, then `setWPACK` (:151) | [`TransferState::Complete`], then a `MISSION_ACK` |
//! | `ERROR` | `MISSION_WRITE_PARTIAL_LIST` from the item, and the item once more (:104-111) | the same, once per item |
//! | `NO_SPACE` | "Upload failed, please reduce the number of wp's" (:113-116) | [`TransferFailure::NoSpace`] |
//! | `INVALID` | "...item had a bad option wp# N" (:117-122) | [`TransferFailure::Invalid`] |
//! | `INVALID_SEQUENCE` | wait 1500 ms for the vehicle's next request and resume there (:123-143) | the same |
//! | anything else | "Upload MISSION failed WAYPOINT MAV_MISSION_DENIED" (:144-148) | [`TransferFailure::Rejected`] |
//!
//! # Where this differs from the C#, and why
//!
//! The C# drives an upload from the ground: it sends item N and waits for a request for N+1,
//! treating a request for anything else as a reason to send item N again
//! (C#: MAVLinkInterface.cs:4318-4356). The MAVLink protocol has the vehicle drive, and so does
//! this machine: it answers the item asked for. Against a well-behaved vehicle the wire is the
//! same. Against a vehicle that re-asks, skips or jumps, the C# sends an item nobody asked for.
//! The consequences, each pinned by a test in `tests/retries.rs`:
//!
//! * A vehicle that acknowledges `ACCEPTED` without having asked for every item leaves the C#
//!   resending the item before the gap until `setWP` times out. Here it is a failure at once,
//!   [`TransferFailure::NeverRequested`], naming the item.
//! * `INVALID_SEQUENCE` followed by silence: the C#'s `getRequestedWPNo` times out, it sends the
//!   partial list and `continue`s - past the item it meant to resend (mav_mission.cs:135-142) -
//!   and on the last item that ends the loop and reports the upload a success. Here the partial
//!   list goes out and the item's own retries run, ending in [`TransferFailure::TimedOut`].
//! * An ack refusing the count itself - ArduPilot sends `NO_SPACE` for a mission too big to
//!   hold - is taken by the C#'s `setWPTotal` as permission to go on (:3863-3876): it returns
//!   without reading the result, and `mav_mission.upload` sends item 0 to a vehicle that is no
//!   longer receiving. What the operator is then told depends on how the vehicle answers an item
//!   it did not ask for - a `setWP` timeout if it says nothing - and not on why it refused. Here
//!   the refusal is the answer.
//! * The C# asks for items with `MISSION_REQUEST` when the vehicle lacks the `MISSION_INT`
//!   capability, and sends `MISSION_ITEM` then (:3420-3453, :4000-4058). This always uses the
//!   `_INT` forms; see [`crate::commands::request_mission_item`] for why.

//! # Write Fast
//!
//! `saveWPsFast` (C#: GCSViews/FlightPlanner.cs:6340-6582) is the planner's other upload: after
//! `setWPTotal` it sends the items without waiting to be asked, pausing every tenth item until
//! the vehicle's `MISSION_REQUEST` catches up - up to 1.1 s, after which it goes on from
//! whatever the vehicle last asked for (`a = reqno`). A `MISSION_ACK` seen while it pauses ends
//! it: `NO_SPACE` and `INVALID` with the ordinary words, `ERROR` with a partial list from the
//! last request and the items from there again, anything else but `ACCEPTED` as "Upload wps
//! failed". Once the last item has gone it sends its own `MISSION_ACK` and is done - it does not
//! wait for the vehicle's. [`MissionTransfer::upload_fast`] is that machine; the bursts come out
//! as [`Action::SendItems`].

use std::time::{Duration, Instant};

use mp_mavlink_dialects::all::{MavCmd, MavMissionResult, MavMissionType};
use mp_mission::{MissionItem, WireItem};
use mp_vehicle::VehicleId;

use crate::timeouts::{ProtocolTimeouts, Retry};

/// How long `saveWPsFast` waits at each tenth item for the vehicle's request to catch up before
/// going on from the last request: `start.AddSeconds(1.1) < DateTime.Now`.
/// `// C#: GCSViews/FlightPlanner.cs:6444`
pub const FAST_CHECKPOINT_WAIT: Duration = Duration::from_millis(1100);

/// Every tenth item, `saveWPsFast` pauses: `if (a % 10 == 0 && a != 0)`.
/// `// C#: GCSViews/FlightPlanner.cs:6423`
pub const FAST_BURST: u16 = 10;

/// `MAV_MISSION_ACCEPTED`.
pub const MISSION_ACCEPTED: u8 = 0;
/// `MAV_MISSION_ERROR`: retried once per item with a partial upload.
pub const MISSION_ERROR: u8 = 1;
/// `MAV_MISSION_NO_SPACE`.
pub const MISSION_NO_SPACE: u8 = 4;
/// `MAV_MISSION_INVALID`.
pub const MISSION_INVALID: u8 = 5;
/// `MAV_MISSION_INVALID_SEQUENCE`: resumed from wherever the vehicle asks next.
pub const MISSION_INVALID_SEQUENCE: u8 = 13;

/// What a transfer is currently doing.
#[derive(Debug, Clone, PartialEq)]
pub enum TransferState {
    /// Nothing in progress.
    Idle,
    /// Waiting for the vehicle to say how many items it has.
    AwaitingCount,
    /// Fetching items; `next` is the sequence number we still need.
    Downloading {
        /// How many items the vehicle reported.
        count: u16,
        /// The next sequence number required.
        next: u16,
    },
    /// Sending items; the vehicle asks for each one.
    Uploading {
        /// The last sequence the vehicle asked for.
        last_requested: Option<u16>,
    },
    /// Write Fast: items sent in bursts without waiting to be asked.
    UploadingFast {
        /// The C#'s `a`: the next item to send.
        next: u16,
        /// The C#'s `reqno`: the vehicle's last request, `None` until the count is answered.
        requested: Option<u16>,
        /// When the pause at a tenth item began; `None` while nothing is awaited.
        waiting_since: Option<Instant>,
    },
    /// Finished successfully.
    Complete,
    /// Gave up.
    Failed(TransferFailure),
}

/// Which step ran out of retries, named as the C#'s `TimeoutException` names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferStep {
    /// `MISSION_REQUEST_LIST`, waiting for `MISSION_COUNT`.
    RequestList,
    /// `MISSION_REQUEST_INT`, waiting for the item.
    RequestItem,
    /// `MISSION_COUNT`, waiting for the first request.
    SendCount,
    /// `MISSION_ITEM_INT`, waiting for the next request or the ack.
    SendItem,
}

impl std::fmt::Display for TransferStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // C#: MAVLinkInterface.cs:3314, 3478, 3795, 4267.
        f.write_str(match self {
            Self::RequestList => "getWPCount",
            Self::RequestItem => "getWP",
            Self::SendCount => "setWPTotal",
            Self::SendItem => "setWP",
        })
    }
}

/// Why a transfer stopped, in the words Mission Planner uses where it has words for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum TransferFailure {
    /// A step went unanswered through all its retries.
    #[error("Timeout on read - {0}")]
    TimedOut(TransferStep),
    /// `MAV_MISSION_NO_SPACE`. C#: ExtLibs/ArduPilot/mav_mission.cs:113-116.
    #[error("Upload failed, please reduce the number of wp's")]
    NoSpace,
    /// `MAV_MISSION_INVALID`. C#: mav_mission.cs:117-122.
    #[error(
        "Upload failed, mission was rejected by the Mav,\n item had a bad option wp# {seq} MAV_MISSION_INVALID"
    )]
    Invalid {
        /// The item the vehicle refused.
        seq: u16,
    },
    /// Any other refusal. C#: mav_mission.cs:144-148.
    #[error(
        "Upload {} failed {} {}",
        mission_type_name(.mission_type),
        command_name(.command),
        result_name(.result)
    )]
    Rejected {
        /// `MAV_MISSION_RESULT`.
        result: u8,
        /// The item the vehicle refused.
        seq: u16,
        /// That item's `MAV_CMD`.
        command: u16,
        /// Which list.
        mission_type: u8,
    },
    /// Write Fast's `MAV_MISSION_INVALID`, with its own misspelling and the checkpoint's item.
    /// `// C#: GCSViews/FlightPlanner.cs:6474-6478`
    #[error(
        "Upload failed, mission was rejected byt the Mav,\n item had a bad option wp# {seq} MAV_MISSION_INVALID"
    )]
    FastInvalid {
        /// The item the loop stood at, the C#'s `a`.
        seq: u16,
    },
    /// Write Fast's other refusals: `"Upload wps failed " + reqno + " " + result`.
    /// `// C#: GCSViews/FlightPlanner.cs:6484-6488`
    #[error("Upload wps failed {seq} {}", result_name(.result))]
    FastRejected {
        /// The vehicle's last request, the C#'s `reqno`.
        seq: u16,
        /// The `MAV_MISSION_RESULT` it sent.
        result: u8,
    },
    /// The vehicle asked for an item that is not in the mission being uploaded.
    #[error("the vehicle asked for item {0}, which is not in this mission")]
    RequestOutOfRange(u16),
    /// The vehicle said the upload was accepted without asking for this item.
    #[error("the vehicle accepted the mission without asking for item {0}")]
    NeverRequested(u16),
}

/// `MAV_MISSION_TYPE` as the C# enum prints it: `MISSION`, `FENCE`, `RALLY`.
fn mission_type_name(value: &u8) -> String {
    MavMissionType(u32::from(*value))
        .name()
        .and_then(|name| name.strip_prefix("MAV_MISSION_TYPE_"))
        .map_or_else(|| value.to_string(), ToOwned::to_owned)
}

/// `MAV_CMD` as the C# enum prints it: the prefix and any `NAV_` dropped, so `WAYPOINT`.
fn command_name(value: &u16) -> String {
    MavCmd(u32::from(*value))
        .name()
        .and_then(|name| name.strip_prefix("MAV_CMD_"))
        .map(|name| name.strip_prefix("NAV_").unwrap_or(name))
        .map_or_else(|| value.to_string(), ToOwned::to_owned)
}

/// `MAV_MISSION_RESULT` as the C# enum prints it, which is its full name.
fn result_name(value: &u8) -> String {
    MavMissionResult(u32::from(*value))
        .name()
        .map_or_else(|| value.to_string(), ToOwned::to_owned)
}

/// One side of a mission transfer.
#[derive(Debug, Clone)]
pub struct MissionTransfer {
    /// Which vehicle.
    pub target: VehicleId,
    /// Which list: the mission, the geofence or the rally points.
    ///
    /// The three share one protocol and one set of messages, distinguished only by this field. A
    /// transfer that ignored it would answer a fence's MISSION_COUNT as if it were the mission's
    /// and write a geofence into the flight plan.
    pub mission_type: u8,
    state: TransferState,
    items: Vec<MissionItem>,
    last_activity: Instant,
    retries: u8,
    timeouts: ProtocolTimeouts,
    /// Upload: which items the vehicle has asked for, so an early "accepted" is caught.
    requested: Vec<bool>,
    /// Upload: the item whose `MAV_MISSION_ERROR` has had its one partial re-upload.
    error_retried: Option<u16>,
    /// Upload: after `MAV_MISSION_INVALID_SEQUENCE`, until when to wait for the vehicle's request.
    resync_until: Option<Instant>,
    /// Write Fast: a burst is owed on the next tick, after a partial list went out.
    burst_due: bool,
}

/// What the link thread should send next, decided by the state machine rather than by the caller.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Nothing to do right now.
    Nothing,
    /// Ask for the item count.
    RequestList,
    /// Ask for one item.
    RequestItem(u16),
    /// Announce an upload of this many items.
    SendCount(u16),
    /// Send one item.
    SendItem(MissionItem),
    /// Acknowledge a completed transfer: `setWPACK`.
    SendAck,
    /// `MISSION_WRITE_PARTIAL_LIST`: upload again from `start`, `setWPPartialUpdate`.
    SendPartialList {
        /// First item to upload again.
        start: u16,
        /// The mission's length, as the C# sends it (mav_mission.cs:107).
        end: u16,
    },
    /// Write Fast's burst: each of these as a `MISSION_ITEM_INT`, in order, without waiting.
    SendItems(Vec<MissionItem>),
}

impl MissionTransfer {
    fn new(target: VehicleId, mission_type: u8, state: TransferState) -> Self {
        Self {
            target,
            mission_type,
            state,
            items: Vec::new(),
            last_activity: Instant::now(),
            retries: 0,
            timeouts: ProtocolTimeouts::default(),
            requested: Vec::new(),
            error_retried: None,
            resync_until: None,
            burst_due: false,
        }
    }

    /// Starts a download.
    #[must_use]
    pub fn download(target: VehicleId, mission_type: u8) -> Self {
        Self::new(target, mission_type, TransferState::AwaitingCount)
    }

    /// Starts an upload.
    #[must_use]
    pub fn upload(target: VehicleId, items: Vec<MissionItem>, mission_type: u8) -> Self {
        let mut transfer = Self::new(
            target,
            mission_type,
            TransferState::Uploading {
                last_requested: None,
            },
        );
        transfer.requested = vec![false; items.len()];
        transfer.items = items;
        transfer
    }

    /// Write Fast: `saveWPsFast`'s upload of `items`, the count first, then the items in bursts.
    /// `// C#: GCSViews/FlightPlanner.cs:6340-6582`
    #[must_use]
    pub fn upload_fast(target: VehicleId, items: Vec<MissionItem>, mission_type: u8) -> Self {
        let mut transfer = Self::new(
            target,
            mission_type,
            TransferState::UploadingFast {
                next: 0,
                requested: None,
                waiting_since: None,
            },
        );
        transfer.items = items;
        transfer
    }

    /// The same transfer with different waits. The link applies its own on picking one up.
    #[must_use]
    pub const fn with_timeouts(mut self, timeouts: ProtocolTimeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    /// The current state.
    #[must_use]
    pub fn state(&self) -> &TransferState {
        &self.state
    }

    /// The items downloaded, or the items being uploaded.
    #[must_use]
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }

    /// Whether this transfer is waiting for an item to arrive.
    ///
    /// Used to route `MISSION_ITEM_INT`, which carries no list type of its own. Only one of a
    /// vehicle's transfers can be in this state at a time, because each is lock-step and asks for
    /// the next item only once the last has arrived.
    #[must_use]
    pub const fn expects_item(&self) -> bool {
        matches!(self.state, TransferState::Downloading { .. })
    }

    /// Whether the transfer has finished, either way.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        matches!(
            self.state,
            TransferState::Complete | TransferState::Failed(_)
        )
    }

    /// Progress as a fraction.
    #[must_use]
    pub fn progress(&self) -> f32 {
        match &self.state {
            TransferState::Downloading { count, next } if *count > 0 => {
                f32::from(*next) / f32::from(*count)
            }
            TransferState::Uploading { last_requested } if !self.items.is_empty() => {
                let done = last_requested.map_or(0, |seq| u32::from(seq) + 1);
                done as f32 / self.items.len() as f32
            }
            TransferState::UploadingFast { next, .. } if !self.items.is_empty() => {
                f32::from(*next) / self.items.len() as f32
            }
            TransferState::Complete => 1.0,
            _ => 0.0,
        }
    }

    fn count(&self) -> u16 {
        u16::try_from(self.items.len()).unwrap_or(u16::MAX)
    }

    /// The first action to take, immediately after construction.
    #[must_use]
    pub fn begin(&self) -> Action {
        match self.state {
            TransferState::AwaitingCount => Action::RequestList,
            TransferState::Uploading { .. } | TransferState::UploadingFast { .. } => {
                Action::SendCount(self.count())
            }
            _ => Action::Nothing,
        }
    }

    /// The end of the burst that starts at `from`: the next multiple of ten above it, or the
    /// count - the C#'s loop pauses at every `a % 10 == 0 && a != 0` before sending that item.
    fn burst_end(&self, from: u16) -> u16 {
        let next_pause = (from / FAST_BURST + 1) * FAST_BURST;
        next_pause.min(self.count())
    }

    /// Write Fast: the items from `next` to the next pause, sent at once; the pause begins if
    /// items remain, else the transfer is complete and the ack follows on the next tick.
    fn burst(&mut self) -> Action {
        let TransferState::UploadingFast {
            next, requested, ..
        } = self.state
        else {
            return Action::Nothing;
        };
        self.burst_due = false;
        let end = self.burst_end(next);
        let items: Vec<MissionItem> = self
            .items
            .get(usize::from(next)..usize::from(end))
            .map(<[MissionItem]>::to_vec)
            .unwrap_or_default();
        self.touch();
        self.state = TransferState::UploadingFast {
            next: end,
            requested,
            waiting_since: (end < self.count()).then_some(self.last_activity),
        };
        if items.is_empty() {
            return Action::Nothing;
        }
        Action::SendItems(items)
    }

    /// The vehicle reported how many items it holds.
    pub fn on_count(&mut self, count: u16) -> Action {
        if !matches!(self.state, TransferState::AwaitingCount) {
            return Action::Nothing;
        }
        self.touch();
        if count == 0 {
            // An empty mission is a valid answer, and the protocol still wants an ack.
            self.state = TransferState::Complete;
            return Action::SendAck;
        }
        self.items = Vec::with_capacity(usize::from(count));
        self.state = TransferState::Downloading { count, next: 0 };
        Action::RequestItem(0)
    }

    /// An item arrived during a download.
    pub fn on_item(&mut self, wire: &WireItem) -> Action {
        let TransferState::Downloading { count, next } = self.state else {
            return Action::Nothing;
        };
        // A vehicle can resend an item we already have, usually because our request and its reply
        // crossed. Treating it as the next item shifts every later waypoint by one.
        //
        // The C# asks again for the one it still needs on *any* wrong item, at once
        // (C#: MAVLinkInterface.cs:3530-3534). For an item we already hold that is an echo
        // chamber: over a link that delivers each frame twice, every duplicate draws another
        // request, every request two more replies, and the traffic doubles per item - 2^n for an
        // n-item mission. So a stale item is dropped here, and only an item from the future - one
        // nobody asked for - gets the C#'s immediate request for the right one.
        if wire.seq < next {
            return Action::Nothing;
        }
        if wire.seq != next {
            return Action::RequestItem(next);
        }

        self.touch();
        self.items.push(MissionItem::from_wire(wire));
        let following = next + 1;
        if following >= count {
            // `setWPACK` once every item is in (C#: mav_mission.cs:50).
            self.state = TransferState::Complete;
            Action::SendAck
        } else {
            self.state = TransferState::Downloading {
                count,
                next: following,
            };
            Action::RequestItem(following)
        }
    }

    /// The vehicle asked for an item during an upload.
    pub fn on_request(&mut self, seq: u16) -> Action {
        if let TransferState::UploadingFast {
            next,
            requested,
            waiting_since,
        } = self.state
        {
            // `reqno = data.seq`, whatever it is.
            self.state = TransferState::UploadingFast {
                next,
                requested: Some(seq),
                waiting_since,
            };
            // `setWPTotal` returns on a request for 0 or 1; the loop then starts sending.
            // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3800-3803`
            if requested.is_none() && next == 0 {
                return if seq <= 1 {
                    self.burst()
                } else {
                    Action::Nothing
                };
            }
            // At a pause, `if (reqno == a) break;` - the vehicle has all of them; go on.
            // `// C#: GCSViews/FlightPlanner.cs:6439-6443`
            if waiting_since.is_some() && seq == next {
                return self.burst();
            }
            return Action::Nothing;
        }
        let TransferState::Uploading { last_requested } = self.state else {
            return Action::Nothing;
        };
        let Some(item) = self.items.get(usize::from(seq)).copied() else {
            self.state = TransferState::Failed(TransferFailure::RequestOutOfRange(seq));
            return Action::Nothing;
        };
        let resyncing = self.resync_until.take().is_some();
        if !resyncing && last_requested == Some(seq) {
            // Asked again for the item just sent: it was lost, or the request was duplicated.
            // Sending it again costs one of the item's retries, as the C#'s immediate resend does
            // (C#: MAVLinkInterface.cs:4354, `start = DateTime.MinValue`), so a vehicle stuck
            // asking for one item cannot hold the transfer open for ever.
            if self.retries >= self.timeouts.mission_item_send.retries {
                self.state =
                    TransferState::Failed(TransferFailure::TimedOut(TransferStep::SendItem));
                return Action::Nothing;
            }
            self.retries += 1;
            self.last_activity = Instant::now();
            return Action::SendItem(item);
        }
        self.touch();
        if let Some(asked) = self.requested.get_mut(usize::from(seq)) {
            *asked = true;
        }
        self.state = TransferState::Uploading {
            last_requested: Some(seq),
        };
        Action::SendItem(item)
    }

    /// The vehicle acknowledged an upload, or refused part of it.
    ///
    /// See the module documentation for what each `MAV_MISSION_RESULT` does.
    pub fn on_ack(&mut self, result: u8) -> Action {
        if let TransferState::UploadingFast {
            next, requested, ..
        } = self.state
        {
            return self.on_fast_ack(result, next, requested.unwrap_or(0));
        }
        let TransferState::Uploading { last_requested } = self.state else {
            return Action::Nothing;
        };
        // The item the ack answers: the C#'s loop index `a`. An ack before any request answers
        // the count, and the C# would be on item 0 by then.
        let seq = last_requested.unwrap_or(0);
        let failure = match result {
            MISSION_ACCEPTED => {
                if let Some(unasked) = self.requested.iter().position(|asked| !asked) {
                    TransferFailure::NeverRequested(u16::try_from(unasked).unwrap_or(u16::MAX))
                } else {
                    self.state = TransferState::Complete;
                    // `setWPACK` after the last item (C#: mav_mission.cs:151).
                    return Action::SendAck;
                }
            }
            MISSION_ERROR if self.error_retried != Some(seq) => {
                self.error_retried = Some(seq);
                self.touch();
                return Action::SendPartialList {
                    start: seq,
                    end: self.count(),
                };
            }
            MISSION_NO_SPACE => TransferFailure::NoSpace,
            MISSION_INVALID => TransferFailure::Invalid { seq },
            MISSION_INVALID_SEQUENCE => {
                self.last_activity = Instant::now();
                self.resync_until = Some(self.last_activity + self.timeouts.mission_resync);
                return Action::Nothing;
            }
            other => TransferFailure::Rejected {
                result: other,
                seq,
                command: self
                    .items
                    .get(usize::from(seq))
                    .map_or(0, |item| item.command),
                mission_type: self.mission_type,
            },
        };
        self.state = TransferState::Failed(failure);
        Action::Nothing
    }

    /// Write Fast's reading of a `MISSION_ACK`, as the pause loop reads `result`: `a` is the
    /// item the loop stands at (`next`), `reqno` the vehicle's last request.
    /// `// C#: GCSViews/FlightPlanner.cs:6446-6489`
    fn on_fast_ack(&mut self, result: u8, a: u16, reqno: u16) -> Action {
        let failure = match result {
            // `result` stays accepted; INVALID_SEQUENCE only sleeps 500 ms inside the pause.
            MISSION_ACCEPTED | MISSION_INVALID_SEQUENCE => return Action::Nothing,
            // "resend for partial upload": the list from `reqno`, then `a = reqno` and on.
            MISSION_ERROR => {
                self.state = TransferState::UploadingFast {
                    next: reqno,
                    requested: Some(reqno),
                    waiting_since: None,
                };
                self.burst_due = true;
                self.touch();
                return Action::SendPartialList {
                    start: reqno,
                    end: self.count(),
                };
            }
            MISSION_NO_SPACE => TransferFailure::NoSpace,
            MISSION_INVALID => TransferFailure::FastInvalid { seq: a },
            other => TransferFailure::FastRejected {
                seq: reqno,
                result: other,
            },
        };
        self.state = TransferState::Failed(failure);
        Action::Nothing
    }

    /// Write Fast's clock: the count's retries until the vehicle asks (`setWPTotal`), the 1.1 s
    /// pause at each tenth item, and the ack once everything has gone.
    fn fast_tick(&mut self, now: Instant) -> Action {
        let TransferState::UploadingFast {
            next,
            requested,
            waiting_since,
        } = self.state
        else {
            return Action::Nothing;
        };
        if self.burst_due {
            return self.burst();
        }
        if next >= self.count() && requested.is_some() {
            // `MainV2.comPort.setWPACK()`: done, without waiting for the vehicle's ack.
            // `// C#: GCSViews/FlightPlanner.cs:6567`
            self.state = TransferState::Complete;
            return Action::SendAck;
        }
        if requested.is_none() {
            // `setWPTotal`: 700 ms, three more tries, then "Timeout on read - setWPTotal".
            let policy = self.timeouts.mission_count;
            if now.saturating_duration_since(self.last_activity) < policy.timeout {
                return Action::Nothing;
            }
            if self.retries >= policy.retries {
                self.state =
                    TransferState::Failed(TransferFailure::TimedOut(TransferStep::SendCount));
                return Action::Nothing;
            }
            self.retries += 1;
            self.last_activity = now;
            return Action::SendCount(self.count());
        }
        if let Some(since) = waiting_since
            && now.saturating_duration_since(since) >= FAST_CHECKPOINT_WAIT
        {
            // "do next 10 starting at reqno": `a = reqno`.
            // `// C#: GCSViews/FlightPlanner.cs:6444-6449`
            self.state = TransferState::UploadingFast {
                next: requested.unwrap_or(0),
                requested,
                waiting_since: None,
            };
            return self.burst();
        }
        Action::Nothing
    }

    /// Called periodically. Retries the outstanding step, or gives up.
    pub fn on_tick(&mut self) -> Action {
        let now = Instant::now();
        if matches!(self.state, TransferState::UploadingFast { .. }) {
            return self.fast_tick(now);
        }
        let (policy, step): (Retry, TransferStep) = match self.state {
            TransferState::AwaitingCount => (self.timeouts.mission_list, TransferStep::RequestList),
            TransferState::Downloading { .. } => (
                self.timeouts.mission_item_request,
                TransferStep::RequestItem,
            ),
            TransferState::Uploading { last_requested } => {
                if let Some(until) = self.resync_until {
                    // `getRequestedWPNo` ran out: tell the vehicle where to resume
                    // (C#: mav_mission.cs:135-139).
                    if now < until {
                        return Action::Nothing;
                    }
                    self.resync_until = None;
                    self.last_activity = now;
                    return Action::SendPartialList {
                        start: last_requested.unwrap_or(0),
                        end: self.count(),
                    };
                }
                if last_requested.is_some() {
                    (self.timeouts.mission_item_send, TransferStep::SendItem)
                } else {
                    (self.timeouts.mission_count, TransferStep::SendCount)
                }
            }
            TransferState::Idle
            | TransferState::Complete
            | TransferState::Failed(_)
            | TransferState::UploadingFast { .. } => {
                return Action::Nothing;
            }
        };
        if now.saturating_duration_since(self.last_activity) < policy.timeout {
            return Action::Nothing;
        }
        if self.retries >= policy.retries {
            self.state = TransferState::Failed(TransferFailure::TimedOut(step));
            return Action::Nothing;
        }
        self.retries += 1;
        self.last_activity = now;

        match &self.state {
            TransferState::AwaitingCount => Action::RequestList,
            TransferState::Downloading { next, .. } => Action::RequestItem(*next),
            TransferState::Uploading {
                last_requested: None,
            } => Action::SendCount(self.count()),
            // The item the vehicle last asked for, sent again unasked: the C#'s `setWP` retry
            // (C#: MAVLinkInterface.cs:4254-4262).
            TransferState::Uploading {
                last_requested: Some(seq),
            } => self
                .items
                .get(usize::from(*seq))
                .copied()
                .map_or(Action::Nothing, Action::SendItem),
            TransferState::Idle
            | TransferState::Complete
            | TransferState::Failed(_)
            | TransferState::UploadingFast { .. } => Action::Nothing,
        }
    }

    fn touch(&mut self) {
        self.last_activity = Instant::now();
        self.retries = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mission::MISSION_TYPE_MISSION;
    use std::time::Duration;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    fn item(seq: u16) -> MissionItem {
        MissionItem {
            seq,
            command: 16,
            x: -35.0,
            y: 149.0,
            z: 50.0,
            ..MissionItem::default()
        }
    }

    /// Winds the clock back so the next tick sees the step as timed out.
    fn expire(transfer: &mut MissionTransfer, timeout: Duration) {
        transfer.last_activity = Instant::now() - timeout - Duration::from_millis(1);
    }

    #[test]
    fn a_download_walks_the_whole_mission() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        assert_eq!(transfer.begin(), Action::RequestList);

        assert_eq!(transfer.on_count(3), Action::RequestItem(0));
        assert_eq!(transfer.on_item(&item(0).to_wire()), Action::RequestItem(1));
        assert_eq!(transfer.on_item(&item(1).to_wire()), Action::RequestItem(2));
        assert_eq!(transfer.on_item(&item(2).to_wire()), Action::SendAck);

        assert_eq!(transfer.state(), &TransferState::Complete);
        assert_eq!(transfer.items().len(), 3);
        assert!((transfer.progress() - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn a_duplicated_item_is_ignored_rather_than_accepted_as_the_next_one() {
        // A request and its reply crossing is normal on a lossy link. Counting the duplicate as
        // the next item shifts every subsequent waypoint by one - a mission that flies a
        // different shape than the one on screen.
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(3);
        transfer.on_item(&item(0).to_wire());
        assert_eq!(
            transfer.on_item(&item(0).to_wire()),
            Action::Nothing,
            "duplicate ignored"
        );
        assert_eq!(transfer.items().len(), 1);
        assert_eq!(transfer.on_item(&item(1).to_wire()), Action::RequestItem(2));
    }

    #[test]
    fn an_item_nobody_asked_for_is_not_kept_and_the_right_one_is_asked_for() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(3);
        assert_eq!(transfer.on_item(&item(2).to_wire()), Action::RequestItem(0));
        assert!(transfer.items().is_empty());
    }

    #[test]
    fn an_empty_mission_is_a_valid_answer() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        assert_eq!(transfer.on_count(0), Action::SendAck);
        assert_eq!(transfer.state(), &TransferState::Complete);
        assert!(transfer.items().is_empty());
    }

    #[test]
    fn an_upload_answers_whatever_the_vehicle_asks_for() {
        let items = vec![item(0), item(1), item(2)];
        let mut transfer = MissionTransfer::upload(target(), items, MISSION_TYPE_MISSION);
        assert_eq!(transfer.begin(), Action::SendCount(3));

        assert_eq!(transfer.on_request(0), Action::SendItem(item(0)));
        // Vehicles re-request items; answering again is correct.
        assert_eq!(transfer.on_request(0), Action::SendItem(item(0)));
        assert_eq!(transfer.on_request(1), Action::SendItem(item(1)));
        assert_eq!(transfer.on_request(2), Action::SendItem(item(2)));

        assert_eq!(transfer.on_ack(MISSION_ACCEPTED), Action::SendAck);
        assert_eq!(transfer.state(), &TransferState::Complete);
    }

    #[test]
    fn a_request_beyond_the_mission_fails_the_transfer() {
        let mut transfer = MissionTransfer::upload(target(), vec![item(0)], MISSION_TYPE_MISSION);
        transfer.on_request(7);
        assert_eq!(
            transfer.state(),
            &TransferState::Failed(TransferFailure::RequestOutOfRange(7))
        );
        assert!(transfer.is_finished());
    }

    #[test]
    fn a_rejected_upload_reports_the_reason_in_the_words_mission_planner_uses() {
        let mut transfer = MissionTransfer::upload(target(), vec![item(0)], MISSION_TYPE_MISSION);
        transfer.on_request(0);
        transfer.on_ack(14);
        let TransferState::Failed(failure) = transfer.state() else {
            panic!("failed");
        };
        assert_eq!(
            failure.to_string(),
            "Upload MISSION failed WAYPOINT MAV_MISSION_DENIED"
        );
    }

    #[test]
    fn a_silent_vehicle_eventually_gives_up_rather_than_hanging() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(2);
        let policy = ProtocolTimeouts::default().mission_item_request;

        for _ in 0..=policy.retries {
            expire(&mut transfer, policy.timeout);
            transfer.on_tick();
        }
        assert_eq!(
            transfer.state(),
            &TransferState::Failed(TransferFailure::TimedOut(TransferStep::RequestItem))
        );
    }

    #[test]
    fn a_retry_repeats_the_outstanding_request_only() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(3);
        transfer.on_item(&item(0).to_wire());

        expire(
            &mut transfer,
            ProtocolTimeouts::default().mission_item_request.timeout,
        );
        assert_eq!(
            transfer.on_tick(),
            Action::RequestItem(1),
            "retries the item we still need"
        );

        // Progress is not lost by a retry.
        assert_eq!(transfer.items().len(), 1);
    }

    #[test]
    fn command_names_print_as_the_csharp_enum_prints_them() {
        assert_eq!(command_name(&16), "WAYPOINT");
        assert_eq!(command_name(&176), "DO_SET_MODE");
        assert_eq!(command_name(&5001), "FENCE_POLYGON_VERTEX_INCLUSION");
        assert_eq!(command_name(&65_000), "65000");
        assert_eq!(mission_type_name(&1), "FENCE");
        assert_eq!(result_name(&13), "MAV_MISSION_INVALID_SEQUENCE");
    }

    fn fast(count: u16) -> MissionTransfer {
        MissionTransfer::upload_fast(target(), (0..count).map(item).collect(), 0)
    }

    fn sent(action: &Action) -> Vec<u16> {
        match action {
            Action::SendItems(items) => items.iter().map(|item| item.seq).collect(),
            other => panic!("expected a burst, got {other:?}"),
        }
    }

    /// Winds a Write Fast pause back so the next tick sees it as over.
    fn expire_pause(transfer: &mut MissionTransfer) {
        if let TransferState::UploadingFast {
            next, requested, ..
        } = transfer.state
        {
            transfer.state = TransferState::UploadingFast {
                next,
                requested,
                waiting_since: Some(
                    Instant::now() - FAST_CHECKPOINT_WAIT - Duration::from_millis(1),
                ),
            };
        }
    }

    fn failure(transfer: &MissionTransfer) -> String {
        match transfer.state() {
            TransferState::Failed(why) => why.to_string(),
            other => panic!("not failed: {other:?}"),
        }
    }

    #[test]
    fn a_fast_upload_sends_the_count_then_the_items_ten_at_a_time_as_the_requests_catch_up() {
        let mut transfer = fast(25);
        assert_eq!(transfer.begin(), Action::SendCount(25));
        // Nothing goes until the vehicle asks for item 0 (`setWPTotal` returns on 0 or 1).
        assert_eq!(transfer.on_tick(), Action::Nothing);
        assert_eq!(sent(&transfer.on_request(0)), (0..10).collect::<Vec<_>>());
        // Paused at 10: a request for 10 means the vehicle has 0..9.
        assert_eq!(transfer.on_tick(), Action::Nothing);
        assert_eq!(transfer.on_request(4), Action::Nothing);
        assert_eq!(sent(&transfer.on_request(10)), (10..20).collect::<Vec<_>>());
        assert_eq!(sent(&transfer.on_request(20)), (20..25).collect::<Vec<_>>());
        assert!((transfer.progress() - 1.0).abs() < f32::EPSILON);
        // Everything has gone: the C# sends its own ack and is done, without the vehicle's.
        assert_eq!(transfer.on_tick(), Action::SendAck);
        assert!(matches!(transfer.state(), TransferState::Complete));
        assert_eq!(transfer.on_ack(MISSION_ACCEPTED), Action::Nothing);
    }

    #[test]
    fn a_fast_upload_goes_on_from_the_last_request_when_the_pause_runs_out() {
        let mut transfer = fast(25);
        let _ = transfer.begin();
        transfer.on_request(0);
        // The vehicle got as far as asking for 7 and went quiet.
        assert_eq!(transfer.on_request(7), Action::Nothing);
        assert_eq!(transfer.on_tick(), Action::Nothing);
        expire_pause(&mut transfer);
        // `a = reqno`: 7, 8, 9 again, then the pause at 10.
        assert_eq!(sent(&transfer.on_tick()), vec![7, 8, 9]);
        assert_eq!(transfer.on_tick(), Action::Nothing);
        assert_eq!(sent(&transfer.on_request(10)), (10..20).collect::<Vec<_>>());
    }

    #[test]
    fn a_fast_upload_told_error_writes_a_partial_list_from_the_request_and_resends() {
        let mut transfer = fast(25);
        let _ = transfer.begin();
        transfer.on_request(0);
        transfer.on_request(4);
        assert_eq!(
            transfer.on_ack(MISSION_ERROR),
            Action::SendPartialList { start: 4, end: 25 }
        );
        assert_eq!(sent(&transfer.on_tick()), (4..10).collect::<Vec<_>>());
    }

    #[test]
    fn a_fast_upload_refused_says_why_in_save_wps_fasts_words() {
        let mut transfer = fast(25);
        let _ = transfer.begin();
        transfer.on_request(0);
        transfer.on_ack(MISSION_NO_SPACE);
        assert_eq!(
            failure(&transfer),
            "Upload failed, please reduce the number of wp's"
        );

        let mut transfer = fast(25);
        let _ = transfer.begin();
        transfer.on_request(0);
        transfer.on_ack(MISSION_INVALID);
        assert_eq!(
            failure(&transfer),
            "Upload failed, mission was rejected byt the Mav,\n item had a bad option wp# 10 MAV_MISSION_INVALID"
        );

        let mut transfer = fast(25);
        let _ = transfer.begin();
        transfer.on_request(0);
        transfer.on_request(6);
        // MAV_MISSION_UNSUPPORTED.
        transfer.on_ack(3);
        assert_eq!(
            failure(&transfer),
            "Upload wps failed 6 MAV_MISSION_UNSUPPORTED"
        );
        assert!(transfer.is_finished());
    }

    #[test]
    fn a_fast_upload_whose_count_is_unanswered_retries_it_then_gives_up() {
        let mut transfer = fast(3);
        assert_eq!(transfer.begin(), Action::SendCount(3));
        let policy = transfer.timeouts.mission_count;
        for _ in 0..policy.retries {
            expire(&mut transfer, policy.timeout);
            assert_eq!(transfer.on_tick(), Action::SendCount(3));
        }
        expire(&mut transfer, policy.timeout);
        assert_eq!(transfer.on_tick(), Action::Nothing);
        assert_eq!(
            failure(&transfer),
            TransferFailure::TimedOut(TransferStep::SendCount).to_string()
        );
    }

    #[test]
    fn a_fast_upload_of_fewer_than_ten_items_needs_one_burst() {
        let mut transfer = fast(4);
        let _ = transfer.begin();
        assert_eq!(sent(&transfer.on_request(1)), vec![0, 1, 2, 3]);
        assert_eq!(transfer.on_tick(), Action::SendAck);
    }
}
