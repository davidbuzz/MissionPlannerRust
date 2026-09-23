//! The mission upload and download state machine.
//!
//! Replaces the transfer half of `MAVLinkInterface`'s waypoint handling.
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

use std::time::{Duration, Instant};

use mp_mission::{MissionItem, WireItem};
use mp_vehicle::VehicleId;

/// How long to wait for the next expected message before retrying.
pub const STEP_TIMEOUT: Duration = Duration::from_millis(1200);

/// How many times a single step is retried before the transfer is abandoned.
pub const MAX_RETRIES: u8 = 5;

/// `MAV_MISSION_ACCEPTED`.
pub const MISSION_ACCEPTED: u8 = 0;

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

/// Why a transfer stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum TransferFailure {
    /// The vehicle stopped responding.
    #[error("the vehicle stopped responding")]
    TimedOut,
    /// The vehicle rejected the mission; the value is `MAV_MISSION_RESULT`.
    #[error("the vehicle rejected the mission (result {0})")]
    Rejected(u8),
    /// The vehicle asked for an item that is not in the mission being uploaded.
    #[error("the vehicle asked for item {0}, which is not in this mission")]
    RequestOutOfRange(u16),
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
    /// Acknowledge a completed download.
    SendAck,
}

impl MissionTransfer {
    /// Starts a download.
    #[must_use]
    pub fn download(target: VehicleId, mission_type: u8) -> Self {
        Self {
            target,
            mission_type,
            state: TransferState::AwaitingCount,
            items: Vec::new(),
            last_activity: Instant::now(),
            retries: 0,
        }
    }

    /// Starts an upload.
    #[must_use]
    pub fn upload(target: VehicleId, items: Vec<MissionItem>, mission_type: u8) -> Self {
        Self {
            target,
            mission_type,
            state: TransferState::Uploading {
                last_requested: None,
            },
            items,
            last_activity: Instant::now(),
            retries: 0,
        }
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

    /// The first action to take, immediately after construction.
    #[must_use]
    pub fn begin(&self) -> Action {
        match self.state {
            TransferState::AwaitingCount => Action::RequestList,
            TransferState::Uploading { .. } => {
                Action::SendCount(u16::try_from(self.items.len()).unwrap_or(u16::MAX))
            }
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
        // crossed. Ignoring the duplicate is correct; treating it as the next item is not.
        if wire.seq != next {
            return Action::Nothing;
        }

        self.touch();
        self.items.push(MissionItem::from_wire(wire));
        let following = next + 1;
        if following >= count {
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
        if !matches!(self.state, TransferState::Uploading { .. }) {
            return Action::Nothing;
        }
        let Some(item) = self.items.get(usize::from(seq)).copied() else {
            self.state = TransferState::Failed(TransferFailure::RequestOutOfRange(seq));
            return Action::Nothing;
        };
        self.touch();
        self.state = TransferState::Uploading {
            last_requested: Some(seq),
        };
        Action::SendItem(item)
    }

    /// The vehicle acknowledged an upload.
    pub fn on_ack(&mut self, result: u8) -> Action {
        if matches!(self.state, TransferState::Uploading { .. }) {
            self.touch();
            self.state = if result == MISSION_ACCEPTED {
                TransferState::Complete
            } else {
                TransferState::Failed(TransferFailure::Rejected(result))
            };
        }
        Action::Nothing
    }

    /// Called periodically. Retries the outstanding step, or gives up.
    pub fn on_tick(&mut self) -> Action {
        if self.is_finished() || self.last_activity.elapsed() < STEP_TIMEOUT {
            return Action::Nothing;
        }
        if self.retries >= MAX_RETRIES {
            self.state = TransferState::Failed(TransferFailure::TimedOut);
            return Action::Nothing;
        }
        self.retries += 1;
        self.last_activity = Instant::now();

        match &self.state {
            TransferState::AwaitingCount => Action::RequestList,
            TransferState::Downloading { next, .. } => Action::RequestItem(*next),
            // An upload is driven by the vehicle's requests, so the only thing to repeat is the
            // count that starts it.
            TransferState::Uploading {
                last_requested: None,
            } => Action::SendCount(u16::try_from(self.items.len()).unwrap_or(u16::MAX)),
            TransferState::Uploading { .. }
            | TransferState::Idle
            | TransferState::Complete
            | TransferState::Failed(_) => Action::Nothing,
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
    fn an_out_of_order_item_is_ignored() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(3);
        assert_eq!(
            transfer.on_item(&item(2).to_wire()),
            Action::Nothing,
            "item 2 is not next"
        );
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

        transfer.on_ack(MISSION_ACCEPTED);
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
    fn a_rejected_upload_reports_the_reason() {
        let mut transfer = MissionTransfer::upload(target(), vec![item(0)], MISSION_TYPE_MISSION);
        transfer.on_request(0);
        transfer.on_ack(13);
        assert_eq!(
            transfer.state(),
            &TransferState::Failed(TransferFailure::Rejected(13))
        );
    }

    #[test]
    fn a_silent_vehicle_eventually_gives_up_rather_than_hanging() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(2);

        // Force the clock past the step timeout repeatedly.
        for _ in 0..=MAX_RETRIES {
            transfer.last_activity = Instant::now() - STEP_TIMEOUT - Duration::from_millis(1);
            transfer.on_tick();
        }
        assert_eq!(
            transfer.state(),
            &TransferState::Failed(TransferFailure::TimedOut)
        );
    }

    #[test]
    fn a_retry_repeats_the_outstanding_request_only() {
        let mut transfer = MissionTransfer::download(target(), MISSION_TYPE_MISSION);
        transfer.on_count(3);
        transfer.on_item(&item(0).to_wire());

        transfer.last_activity = Instant::now() - STEP_TIMEOUT - Duration::from_millis(1);
        assert_eq!(
            transfer.on_tick(),
            Action::RequestItem(1),
            "retries the item we still need"
        );

        // Progress is not lost by a retry.
        assert_eq!(transfer.items().len(), 1);
    }
}
