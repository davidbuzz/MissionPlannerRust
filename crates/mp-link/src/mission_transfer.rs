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

use std::time::Instant;

use mp_mavlink_dialects::all::{MavCmd, MavMissionResult, MavMissionType};
use mp_mission::{MissionItem, WireItem};
use mp_vehicle::VehicleId;

use crate::timeouts::{ProtocolTimeouts, Retry};

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
            TransferState::Uploading { .. } => Action::SendCount(self.count()),
            _ => Action::Nothing,
        }
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

    /// Called periodically. Retries the outstanding step, or gives up.
    pub fn on_tick(&mut self) -> Action {
        let now = Instant::now();
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
            TransferState::Idle | TransferState::Complete | TransferState::Failed(_) => {
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
            TransferState::Idle | TransferState::Complete | TransferState::Failed(_) => {
                Action::Nothing
            }
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
}
