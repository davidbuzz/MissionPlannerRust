//! The live link: one transport, one I/O thread, many vehicles.
//!
//! Replaces `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`.
//!
//! # Threading model
//!
//! One thread per link owns the transport, the frame decoder and every vehicle's working state.
//! Nothing else touches them, so the hot path needs no locks at all. The thread publishes
//! immutable snapshots on a fixed cadence; readers (the UI, a recorder, a script) take those
//! with a single atomic load.
//!
//! Outbound messages go through a queue rather than being written by the caller's thread, so a
//! slow or blocked link can never stall a UI frame.
//!
//! The C# design instead shares a mutable `MAVLinkInterface` across threads with locks around it,
//! which is why Mission Planner's UI can hitch when a link degrades.
//!
//! # Protocol state machines
//!
//! Every conversation that asks the vehicle something and waits is an explicit state machine,
//! fed each message and each pass of this thread's loop, holding Mission Planner's retry counts
//! and waits from [`ProtocolTimeouts`] (DELIVERABLES.md D4). `tests/retries.rs` drives each one
//! against a scripted vehicle that drops, repeats, delays, skips and refuses, and asserts it
//! stops - complete or a clean failure - within its waits, having sent exactly the C#'s number of
//! retries; `tests/routing.rs` runs them among fifty vehicles on one link. C# lines are in
//! `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` unless named.
//!
//! * [`param_download::ParamDownload`] - `getParamListAsync`. `Streaming` → `Recovering` →
//!   `Complete`, or `Cancelled` by the caller. Recovery after 4000 ms quiet or the last index
//!   arriving short (:2089, :2114); the whole list again at most twice under three quarters
//!   (:2117); then rounds of 10 reads (:2187) every 1000 ms (:2135); never gives up by itself
//!   (:2226). Tests: `parameters_arriving_in_any_order_*`, `the_last_index_arriving_short_*`,
//!   `holes_are_read_ten_at_a_time_*`, `a_stream_under_three_quarters_*`,
//!   `a_hole_never_filled_*`, `a_parameter_outside_a_download_*`,
//!   `a_parameter_download_over_a_bad_link_*`.
//! * [`requests::Request`] - one machine for four C# loops, `Queued` → `Waiting` → `Finished`:
//!   - `SetParam`, `setParamAsync`: 3 retries, 700 ms (:1748, :1754); unknown names and
//!     unchanged values not sent (:1640-1651). Tests: `a_set_whose_echo_never_comes_*`,
//!     `a_late_echo_*`, `an_echo_of_a_different_value_*`, `an_echo_of_another_parameter_*`,
//!     `a_set_that_cannot_or_need_not_be_sent_*`.
//!   - `ReadParam`, `GetParamAsync`: 3, 700 ms (:2329, :2333). Test:
//!     `a_read_answered_only_wrongly_*`.
//!   - `Command`, `doCommandAsync`: 3, 2000 ms (:2729, :2731); arming 10 s (:2764-2768);
//!     calibration and bootloader 1 retry, 25 s (:2748-2757); `IN_PROGRESS` waits again with no
//!     retries (:2818-2823); any other result ends it (:2829-2833); reboot and the rest not
//!     waited for (:2720-2773). Tests: `a_command_never_acknowledged_*`, `in_progress_then_*`,
//!     `every_refusal_*`, `an_ack_for_another_command_*`, `acks_arriving_in_the_other_order_*`,
//!     `arming_waits_*`, `a_calibration_is_sent_twice_*`, `the_commands_not_waited_for_*`.
//!   - `SetCurrent`, `setWPCurrentAsync`: 5, 2000 ms (:2472, :2476). Test:
//!     `set_current_is_sent_six_times_*`.
//! * [`mission_transfer::MissionTransfer`] - mission, fence and rally alike.
//!   - Download, `AwaitingCount` → `Downloading` → `Complete` or `Failed`: `getWPCountAsync` 6,
//!     700 ms (:3297, :3301); `getWPAsync` 5, 2500 ms (:3459, :3463). Tests: `a_download_*`.
//!   - Upload, `Uploading` → `Complete` or `Failed`: `setWPTotalAsync` 3, 700 ms (:3779, :3783);
//!     `setWPAsync` 10, 450 ms (:4250, :4254); each `MAV_MISSION_RESULT` as `mav_mission.upload`
//!     treats it (ExtLibs/ArduPilot/mav_mission.cs:101-151). Tests: `an_upload_*`,
//!     `every_mission_result_*`, `a_second_error_*`, `an_item_never_followed_up_*`,
//!     `a_vehicle_stuck_on_one_item_*`, `a_refused_count_*`, `invalid_sequence_then_silence_*`.
//!
//! Where a machine departs from the C# on purpose, its module says why, and the test pinning the
//! difference cites the C# lines.

#![forbid(unsafe_code)]

pub mod commands;
pub mod current_settings;
pub mod fence_points;
pub mod ftp;
pub mod messages;
pub mod mission_transfer;
pub mod param_download;
pub mod requests;
pub mod testing;
pub mod timeouts;
pub mod tlog;
pub mod traffic;

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mission_transfer::{Action, MissionTransfer};
use mp_mavlink::{DecodeStats, FrameDecoder, Message as _, encode_v2};
use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavCmd, MavMessage, MissionWritePartialList};
use mp_mission::{MISSION_TYPE_MISSION, MissionItem, WireItem};
use mp_params::{ParamTable, ParamType, ParamValue, decode_param_id};
use mp_transport::{OpenError, ReadTime, Transport};
use mp_vehicle::{DateTime, FenceItem, StateHandle, StreamRates, VehicleId, VehicleRegistry};
use param_download::{ParamAction, ParamDownload};
use requests::{ParamKey, Request, RequestKind};
pub use timeouts::{ProtocolTimeouts, Retry};

/// MAVFTP's requests, outcomes and errors, for callers of [`Link::ftp`] that do not depend on
/// `mp-ftp` themselves.
pub use mp_ftp::{FtpError, mavftp};

/// MAVLink component id for a ground control station.
pub const MAV_COMP_ID_MISSIONPLANNER: u8 = 190;
/// `MAV_TYPE_GCS`.
const MAV_TYPE_GCS: u8 = 6;
/// `MAV_AUTOPILOT_INVALID`, which is what a GCS reports.
const MAV_AUTOPILOT_INVALID: u8 = 8;

/// `ADSB_FLAGS_VALID_HEADING`. Zero is a legitimate heading, so absence needs its own signal.
const ADSB_VALID_HEADING: u16 = 0x0004;
/// `ADSB_FLAGS_VALID_VELOCITY`.
const ADSB_VALID_VELOCITY: u16 = 0x0008;

/// `MAV_AUTOPILOT_ARDUPILOTMEGA`. ArduPilot encodes parameters differently from the
/// specification, so which autopilot is on the other end is not a cosmetic detail.
const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;

/// How many finished requests the link keeps for their callers to read.
///
/// A caller reads a request's outcome some time after it ends - a frame later, or a second
/// later on a slow screen - so a finished request cannot be dropped at once. Nor can every one be
/// kept, or a script setting parameters in a loop grows the link without bound.
const FINISHED_REQUESTS_KEPT: usize = 256;

/// Minimum time the I/O loop spends per iteration when there is nothing to read.
const IDLE_POLL: Duration = Duration::from_millis(1);

/// The parameters the low-airspeed warning reads, `AIRSPEED_MIN` first.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3863-3871`
const AIRSPEED_MIN_PARAMS: [&str; 2] = ["AIRSPEED_MIN", "ARSPD_FBW_MIN"];

/// How the link should behave.
#[derive(Debug, Clone)]
pub struct LinkConfig {
    /// Our own system id, as seen by the vehicle.
    pub sysid: u8,
    /// Our own component id.
    pub compid: u8,
    /// How often to publish state snapshots. 50 Hz is well above any display refresh while
    /// keeping publish overhead negligible.
    pub publish_interval: Duration,
    /// How often to announce ourselves. Vehicles use this to detect GCS loss (failsafe).
    pub heartbeat_interval: Duration,
    /// Whether to send heartbeats at all. A passive observer or log replay should not.
    pub send_heartbeat: bool,
    /// Where to record every received frame, in Mission Planner's `.tlog` format.
    pub record_path: Option<std::path::PathBuf>,
    /// Whether to ask each vehicle for its telemetry streams: zero never asks, and anything else
    /// asks at the vehicle's own rates, [`mp_vehicle::VehicleState::rates`], as
    /// `UpdateCurrentSettings` does - see [`current_settings`]. The number is not a rate: the
    /// rates are the vehicle state's, which start from [`StreamRates::backups`] and which
    /// [`Link::set_stream_rates`] changes, as the Planner page's combos change `cs.rateX`.
    ///
    /// ArduPilot streams almost nothing until a GCS asks: a fresh SITL sends only heartbeats.
    /// **Not the C#'s:** Mission Planner always asks. Zero is for a tool or a test that must
    /// leave a vehicle's streams alone.
    pub stream_rate_hz: u16,
    /// How long each protocol step waits and how often it retries: Mission Planner's numbers by
    /// default, which is the only thing to fly with. Tests shorten the waits and keep the counts.
    pub timeouts: ProtocolTimeouts,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            sysid: 255,
            compid: MAV_COMP_ID_MISSIONPLANNER,
            publish_interval: Duration::from_millis(20),
            heartbeat_interval: Duration::from_secs(1),
            send_heartbeat: true,
            record_path: None,
            stream_rate_hz: 4,
            timeouts: ProtocolTimeouts::default(),
        }
    }
}

/// A request handed to the link: see [`Link::set_param`], [`Link::command`] and their kin.
///
/// Numbered in the order they were made, which is also the order an answer is offered to them:
/// one `COMMAND_ACK` answers the oldest command waiting for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RequestId(u64);

/// Errors from running a link.
#[derive(Debug, thiserror::Error)]
pub enum LinkError {
    /// The transport could not be opened.
    #[error(transparent)]
    Open(#[from] OpenError),
    /// The link thread could not be started.
    #[error("could not start link thread: {0}")]
    Thread(String),
    /// A recording could not be created or written.
    #[error("{context}: {source}")]
    Record {
        /// What was being attempted.
        context: String,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
}

/// Counters describing the link as a whole.
#[derive(Debug, Clone, Copy, Default)]
pub struct LinkStats {
    /// Bytes read from the transport.
    pub bytes_read: u64,
    /// Bytes written to the transport.
    pub bytes_written: u64,
    /// Frames sent.
    pub frames_sent: u64,
    /// Snapshot publishes performed.
    pub publishes: u64,
    /// Decoder counters.
    pub decode: DecodeStats,
}

/// Shared between the link thread and its owners.
#[derive(Debug, Default)]
struct Shared {
    /// How the transport describes itself *now*. A UDP link learns its peer from the first
    /// datagram, so a description captured at connect time says "no peer yet" for the rest of the
    /// session - which is exactly wrong on the one screen a pilot looks at.
    description: Mutex<String>,
    handles: Mutex<BTreeMap<VehicleId, StateHandle>>,
    /// Parameters received per vehicle.
    params: Mutex<BTreeMap<VehicleId, ParamTable>>,
    /// Parameter downloads the caller has started, finished ones included so their outcome can
    /// be read. Gaps are only chased inside one of these: a single `PARAM_VALUE` - the echo of a
    /// parameter set - carries the vehicle's full count, and treating that as a download with a
    /// thousand holes would flood the link with requests nobody made.
    param_downloads: Mutex<BTreeMap<VehicleId, ParamDownload>>,
    /// Requests the link thread has picked up, finished ones included.
    requests: Mutex<BTreeMap<RequestId, Request>>,
    /// Requests the link thread has yet to pick up.
    request_queue: Mutex<Vec<(RequestId, Request)>>,
    /// The number the next request gets.
    next_request: AtomicU64,
    /// Transfers the caller has started, and their current state.
    ///
    /// Keyed by vehicle *and* list type: a mission download and a fence download are separate
    /// conversations that use the same messages, and a single key per vehicle would let one
    /// answer the other's questions.
    missions: Mutex<BTreeMap<(VehicleId, u8), MissionTransfer>>,
    /// Transfers the link thread has yet to pick up.
    mission_requests: Mutex<Vec<MissionTransfer>>,
    /// What the vehicle has said, and how it answered our commands.
    messages: Mutex<messages::MessageLog>,
    /// The state of an accelerometer calibration, if one is running.
    accel_calibration: Mutex<mp_calibration::AccelCalibration>,
    /// Every compass calibration message since the last clear, as the Compass page reads them.
    compass_calibration: Mutex<mp_calibration::MagCalLog>,
    /// Other aircraft, from ADS-B.
    traffic: Mutex<traffic::TrafficReport>,
    /// Dataflash logs the vehicle has listed.
    log_listings: Mutex<BTreeMap<u16, mp_ftp::logs::LogListing>>,
    /// A log download in progress.
    log_download: Mutex<Option<mp_ftp::logs::LogDownload>>,
    /// Each vehicle's MAVFTP client, made on its first request and kept (see [`ftp`]).
    ftp: Mutex<BTreeMap<VehicleId, mp_ftp::mavftp::MavFtp>>,
    /// `MAVState.fencepoints` for every vehicle (see [`fence_points`]).
    fence_points: Mutex<fence_points::FencePoints>,
    /// What the screens write into a vehicle's state, for the link thread to apply.
    state_writes: Mutex<Vec<(VehicleId, StateWrite)>>,
    stats: Mutex<LinkStats>,
    running: AtomicBool,
    frames_received: AtomicU64,
}

/// A field the C#'s screens write into `MainV2.comPort.MAV.cs` from outside it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum StateWrite {
    /// `cs.rateattitude` to `cs.raterc`.
    StreamRates(StreamRates),
    /// `cs.altoffsethome`.
    AltOffsetHome(f32),
}

/// A way to send on a link without holding the link.
///
/// See [`Link::sender`]. Sends are queued for the link thread, which numbers and writes them;
/// a `false` from [`LinkSender::send`] means the link is gone, not that the frame was slow.
#[derive(Debug, Clone)]
pub struct LinkSender {
    outbound: std::sync::mpsc::Sender<Vec<u8>>,
    sysid: u8,
    compid: u8,
}

impl LinkSender {
    /// Queues a message for transmission. Never blocks; false once the link has stopped.
    pub fn send(&self, message: &MavMessage) -> bool {
        queue_frame(&self.outbound, self.sysid, self.compid, message)
    }
}

/// Encodes a message as a v2 frame from `sysid`/`compid` and queues it for the link thread.
///
/// Sequence numbering is the link thread's job, so it is applied there; zero here. Shared by
/// [`Link::send`] and [`LinkSender::send`] so the two cannot drift - a sender that framed
/// differently from the link would be a bug found on the wire, by a vehicle.
fn queue_frame(
    outbound: &std::sync::mpsc::Sender<Vec<u8>>,
    sysid: u8,
    compid: u8,
    message: &MavMessage,
) -> bool {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let Some(payload) = payload.get(..len) else {
        return false;
    };
    let Ok(n) = encode_v2(
        &mut frame,
        0,
        sysid,
        compid,
        message.id(),
        payload,
        message.crc_extra(),
        0,
    ) else {
        return false;
    };
    frame
        .get(..n)
        .is_some_and(|bytes| outbound.send(bytes.to_vec()).is_ok())
}

/// A running link.
#[derive(Debug)]
pub struct Link {
    /// Bytes received at the previous nudge, so a stall can be told from a transfer in flight.
    last_log_progress: AtomicU32,
    shared: Arc<Shared>,
    outbound: std::sync::mpsc::Sender<Vec<u8>>,
    thread: Option<std::thread::JoinHandle<()>>,
    description: String,
    config: LinkConfig,
}

impl Link {
    /// Opens a link from a URL such as `udp:14550`, `tcp:127.0.0.1:5760` or `file:flight.tlog`.
    pub fn connect(url: &str, config: LinkConfig) -> Result<Self, LinkError> {
        let transport = mp_transport::open(url)?;
        Ok(Self::from_transport(transport, config))
    }

    /// Runs a link over an already-open transport. Used by tests with in-memory doubles.
    #[must_use]
    pub fn from_transport(transport: Box<dyn Transport>, config: LinkConfig) -> Self {
        let shared = Arc::new(Shared::default());
        shared.running.store(true, Ordering::Release);
        let description = transport.description().to_owned();
        shared
            .description
            .lock()
            .map(|mut d| *d = description.clone())
            .unwrap_or(());
        let (tx, rx) = std::sync::mpsc::channel::<Vec<u8>>();

        let thread_shared = Arc::clone(&shared);
        let thread_config = config.clone();
        let thread = std::thread::Builder::new()
            .name("mp-link".to_owned())
            .spawn(move || run_link(transport, thread_config, &thread_shared, &rx))
            .ok();

        Self {
            last_log_progress: AtomicU32::new(0),
            shared,
            outbound: tx,
            thread,
            description,
            config,
        }
    }

    /// Every vehicle heard from so far.
    #[must_use]
    pub fn vehicles(&self) -> Vec<VehicleId> {
        self.shared
            .handles
            .lock()
            .map(|h| h.keys().copied().collect())
            .unwrap_or_default()
    }

    /// A snapshot reader for one vehicle.
    #[must_use]
    pub fn vehicle(&self, id: VehicleId) -> Option<StateHandle> {
        self.shared.handles.lock().ok()?.get(&id).cloned()
    }

    /// The first vehicle that looks like an autopilot, which is what a single-vehicle UI shows.
    #[must_use]
    pub fn primary_vehicle(&self) -> Option<(VehicleId, StateHandle)> {
        let handles = self.shared.handles.lock().ok()?;
        // Component 1 is the autopilot; prefer it over gimbals, companions and other GCSs.
        handles
            .iter()
            .find(|(id, _)| id.compid == 1)
            .or_else(|| handles.iter().next())
            .map(|(id, handle)| (*id, handle.clone()))
    }

    /// Starts a parameter download and returns immediately.
    ///
    /// Progress is observable through [`Link::params`] and [`Link::param_download`]; the link
    /// thread re-requests any gaps, as `getParamListAsync` does, so the caller does not need to
    /// implement retry logic. A download already running for this vehicle starts again from
    /// nothing, as a second call to the C# does.
    pub fn download_params(&self, target: VehicleId) -> bool {
        let download = ParamDownload::new(target, self.config.timeouts, Instant::now());
        let first = download.begin();
        if let Ok(mut downloads) = self.shared.param_downloads.lock() {
            downloads.insert(target, download);
        }
        match first {
            ParamAction::RequestList => self.send(&commands::request_param_list(target)),
            ParamAction::Nothing | ParamAction::RequestIndices(_) => true,
        }
    }

    /// The state of a vehicle's parameter download, if one has been started.
    #[must_use]
    pub fn param_download(&self, target: VehicleId) -> Option<ParamDownload> {
        self.shared
            .param_downloads
            .lock()
            .ok()?
            .get(&target)
            .cloned()
    }

    /// Stops a parameter download, as the C#'s progress dialog's Cancel does. What arrived stays
    /// in the table.
    pub fn cancel_param_download(&self, target: VehicleId) {
        if let Ok(mut downloads) = self.shared.param_downloads.lock()
            && let Some(download) = downloads.get_mut(&target)
        {
            download.cancel();
        }
    }

    /// Sets a parameter and waits, on the link thread, for the vehicle to echo it: `setParam`.
    ///
    /// Refused without sending when the vehicle has not listed the parameter, and skipped when it
    /// already holds `value` unless `force`, as the C# does. The outcome is read with
    /// [`Link::request`].
    pub fn set_param(&self, target: VehicleId, name: &str, value: f64, force: bool) -> RequestId {
        self.queue_request(
            target,
            RequestKind::SetParam {
                name: name.to_owned(),
                value,
                force,
            },
        )
    }

    /// Reads one parameter by name: `GetParam`.
    pub fn read_param(&self, target: VehicleId, name: &str) -> RequestId {
        self.queue_request(
            target,
            RequestKind::ReadParam(ParamKey::Name(name.to_owned())),
        )
    }

    /// Sends a `COMMAND_LONG` and waits for its `COMMAND_ACK`: `doCommand`.
    ///
    /// `require_ack` false sends once and does not wait, as the C#'s `requireack` does.
    pub fn command(
        &self,
        target: VehicleId,
        command: u16,
        params: [f32; 7],
        require_ack: bool,
    ) -> RequestId {
        self.queue_request(
            target,
            RequestKind::Command {
                command,
                params,
                require_ack,
            },
        )
    }

    /// Makes a mission item the current one and waits for `MISSION_CURRENT`: `setWPCurrent`.
    pub fn set_current_waypoint(&self, target: VehicleId, seq: u16) -> RequestId {
        self.queue_request(target, RequestKind::SetCurrent { seq })
    }

    /// Where a request is, or `None` if the link has forgotten it or never had it.
    #[must_use]
    pub fn request(&self, id: RequestId) -> Option<Request> {
        // The table is held while the queue is read, in the link thread's lock order (requests,
        // then the queue), so the request cannot be picked up between the two reads and be in
        // neither. It was, once: a caller that took `None` for "forgotten" abandoned a write the
        // vehicle then accepted.
        let held = self.shared.requests.lock().ok()?;
        if let Some(request) = held.get(&id) {
            return Some(request.clone());
        }
        let queue = self.shared.request_queue.lock().ok()?;
        queue
            .iter()
            .find(|(queued, _)| *queued == id)
            .map(|(_, request)| request.clone())
    }

    fn queue_request(&self, target: VehicleId, kind: RequestKind) -> RequestId {
        let id = RequestId(self.shared.next_request.fetch_add(1, Ordering::Relaxed));
        if let Ok(mut queue) = self.shared.request_queue.lock() {
            queue.push((id, Request::new(target, kind)));
        }
        id
    }

    /// Starts downloading the vehicle's mission. Progress is observable through
    /// [`Link::mission_transfer`].
    pub fn download_mission(&self, target: VehicleId) -> bool {
        self.download_list(target, MISSION_TYPE_MISSION)
    }

    /// Starts uploading a mission to the vehicle.
    pub fn upload_mission(&self, target: VehicleId, items: Vec<MissionItem>) -> bool {
        self.upload_list(target, items, MISSION_TYPE_MISSION)
    }

    /// Starts downloading one of the vehicle's lists: mission, geofence or rally points.
    ///
    /// They share one protocol, so this is the same transfer with a different `mission_type`.
    pub fn download_list(&self, target: VehicleId, mission_type: u8) -> bool {
        self.queue_transfer(MissionTransfer::download(target, mission_type))
    }

    /// Starts uploading one of the vehicle's lists.
    pub fn upload_list(
        &self,
        target: VehicleId,
        items: Vec<MissionItem>,
        mission_type: u8,
    ) -> bool {
        self.queue_transfer(MissionTransfer::upload(target, items, mission_type))
    }

    fn queue_transfer(&self, transfer: MissionTransfer) -> bool {
        self.shared
            .mission_requests
            .lock()
            .map(|mut queue| queue.push(transfer))
            .is_ok()
    }

    /// The state of a vehicle's mission transfer, if one has been started.
    #[must_use]
    pub fn mission_transfer(&self, target: VehicleId) -> Option<MissionTransfer> {
        self.list_transfer(target, MISSION_TYPE_MISSION)
    }

    /// The state of a transfer of one particular list.
    #[must_use]
    pub fn list_transfer(&self, target: VehicleId, mission_type: u8) -> Option<MissionTransfer> {
        self.shared
            .missions
            .lock()
            .ok()?
            .get(&(target, mission_type))
            .cloned()
    }

    /// A vehicle's geofence as the traffic on this link has shown it, in sequence order:
    /// `MAVState.fencepoints`, which `GeoFenceDist` measures from (see [`fence_points`]).
    #[must_use]
    pub fn fence_points(&self, target: VehicleId) -> Vec<FenceItem> {
        self.shared
            .fence_points
            .lock()
            .map(|held| held.items(target))
            .unwrap_or_default()
    }

    /// Sets a vehicle's stream rates, [`mp_vehicle::VehicleState::rates`], as the Planner page's
    /// rate combos set `MainV2.comPort.MAV.cs.rateX`; the link's next stream requests ask for
    /// them. Applied by the link thread on its next pass and carried by the next snapshot.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:573-640`
    pub fn set_stream_rates(&self, target: VehicleId, rates: StreamRates) {
        self.write_state(target, StateWrite::StreamRates(rates));
    }

    /// Sets a vehicle's [`mp_vehicle::VehicleState::alt_offset_home`], as the flight screen's
    /// Set Home Alt sets `MainV2.comPort.MAV.cs.altoffsethome`. Applied by the link thread on its
    /// next pass and carried by the next snapshot. `// C#: GCSViews/FlightData.cs:1236-1247`
    pub fn set_alt_offset_home(&self, target: VehicleId, offset: f32) {
        self.write_state(target, StateWrite::AltOffsetHome(offset));
    }

    fn write_state(&self, target: VehicleId, write: StateWrite) {
        if let Ok(mut writes) = self.shared.state_writes.lock() {
            writes.push((target, write));
        }
    }

    /// A snapshot of a vehicle's parameters.
    #[must_use]
    pub fn params(&self, target: VehicleId) -> Option<ParamTable> {
        self.shared.params.lock().ok()?.get(&target).cloned()
    }

    /// Which state a vehicle's parameter table is in, without copying it: its
    /// [`ParamTable::generation`], which moves each time a parameter arrives.
    ///
    /// For a caller that keeps what it made from [`Link::params`] and asks every frame whether
    /// that is still current - the copy is fourteen hundred names, the question is a number.
    #[must_use]
    pub fn params_generation(&self, target: VehicleId) -> Option<u64> {
        self.shared
            .params
            .lock()
            .ok()?
            .get(&target)
            .map(ParamTable::generation)
    }

    /// The most recent messages from the vehicle, newest last.
    ///
    /// Bounded by `count` because the caller is usually a render pass, and a render pass that
    /// copies an unbounded history gets slower the longer the flight lasts.
    #[must_use]
    pub fn recent_messages(&self, count: usize) -> Vec<messages::LogMessage> {
        self.shared
            .messages
            .lock()
            .map(|log| log.recent(count))
            .unwrap_or_default()
    }

    /// How many messages the log had to evict.
    #[must_use]
    pub fn messages_dropped(&self) -> u64 {
        self.shared
            .messages
            .lock()
            .map(|log| log.dropped())
            .unwrap_or(0)
    }

    /// What an accelerometer calibration is waiting for, if one is running.
    #[must_use]
    pub fn accel_calibration(&self) -> mp_calibration::AccelCalibration {
        self.shared
            .accel_calibration
            .lock()
            .map(|held| *held)
            .unwrap_or(mp_calibration::AccelCalibration::Idle)
    }

    /// Forgets any calibration state, so a finished run does not look like a running one.
    pub fn clear_accel_calibration(&self) {
        if let Ok(mut held) = self.shared.accel_calibration.lock() {
            *held = mp_calibration::AccelCalibration::Idle;
        }
    }

    /// What the vehicle has said about a compass calibration since the last clear: the last
    /// progress and report per compass, in the order each compass was first heard.
    #[must_use]
    pub fn compass_calibration(&self) -> mp_calibration::MagCalLog {
        self.shared
            .compass_calibration
            .lock()
            .map(|held| held.clone())
            .unwrap_or_default()
    }

    /// Forgets compass calibration progress: the C#'s `mprog.Clear()` and `mrep.Clear()`.
    pub fn clear_compass_calibration(&self) {
        if let Ok(mut held) = self.shared.compass_calibration.lock() {
            *held = mp_calibration::MagCalLog::default();
        }
    }

    /// Asks the vehicle to list its dataflash logs.
    ///
    /// Clears what was listed before, so a second listing does not leave logs that have since
    /// been erased sitting in the list.
    pub fn request_log_list(&self, target: VehicleId) -> bool {
        if let Ok(mut held) = self.shared.log_listings.lock() {
            held.clear();
        }
        self.send(&commands::request_log_list(target))
    }

    /// The logs the vehicle has listed, smallest id first.
    #[must_use]
    pub fn log_listings(&self) -> Vec<mp_ftp::logs::LogListing> {
        self.shared
            .log_listings
            .lock()
            .map(|held| held.values().copied().collect())
            .unwrap_or_default()
    }

    /// Starts downloading one log.
    pub fn download_log(&self, target: VehicleId, id: u16, size: u32) -> bool {
        if let Ok(mut held) = self.shared.log_download.lock() {
            *held = Some(mp_ftp::logs::LogDownload::new(target, id, size));
        }
        self.last_log_progress.store(0, Ordering::Release);
        self.send(&commands::request_log_data(
            target,
            id,
            0,
            mp_ftp::logs::WINDOW_BYTES.min(size),
        ))
    }

    /// How far a log download has got, if one is running.
    #[must_use]
    pub fn log_download_progress(&self) -> Option<(u16, u32, u32)> {
        let held = self.shared.log_download.lock().ok()?;
        let download = held.as_ref()?;
        Some((download.id, download.filled(), download.size))
    }

    /// Asks again for the first gap, but only if nothing has arrived since the last nudge.
    ///
    /// Called on a timer by the owner. Two failure modes to avoid, and they pull in opposite
    /// directions: a transfer that only waits never finishes over a lossy link, and a transfer
    /// that re-requests while data is still flowing throttles itself to one window per nudge.
    /// Measured against SITL, the second cost a factor of thirty. So: nudge only on a stall.
    pub fn nudge_log_download(&self, target: VehicleId) -> bool {
        let Ok(held) = self.shared.log_download.lock() else {
            return false;
        };
        let Some(download) = held.as_ref() else {
            return false;
        };
        let id = download.id;
        let filled = download.filled();
        let gap = download.first_gap();
        let size = download.size;
        drop(held);

        // Still arriving. Leave it alone; the vehicle is working through the window it was given.
        let previous = self.last_log_progress.swap(filled, Ordering::AcqRel);
        if filled > previous {
            return true;
        }

        match gap {
            Some(offset) => {
                let remaining = size.saturating_sub(offset);
                self.send(&commands::request_log_data(
                    target,
                    id,
                    offset,
                    mp_ftp::logs::WINDOW_BYTES.min(remaining),
                ))
            }
            None => self.send(&commands::log_request_end(target)),
        }
    }

    /// The assembled log, once every byte has arrived.
    #[must_use]
    pub fn finished_log(&self) -> Option<(u16, Vec<u8>)> {
        let held = self.shared.log_download.lock().ok()?;
        let download = held.as_ref()?;
        download
            .is_complete()
            .then(|| (download.id, download.assemble()))
    }

    /// Forgets a download.
    pub fn clear_log_download(&self) {
        if let Ok(mut held) = self.shared.log_download.lock() {
            *held = None;
        }
    }

    /// Other aircraft currently known about, oldest sightings already dropped.
    ///
    /// Forgetting happens on read rather than on a timer: there is no other thread to run it on,
    /// and a caller that has stopped looking does not need the list pruned.
    #[must_use]
    pub fn traffic(&self) -> Vec<traffic::Traffic> {
        let Ok(mut held) = self.shared.traffic.lock() else {
            return Vec::new();
        };
        held.forget_old(Instant::now());
        held.all()
    }

    /// Link counters.
    #[must_use]
    pub fn stats(&self) -> LinkStats {
        self.shared.stats.lock().map(|s| *s).unwrap_or_default()
    }

    /// Total frames received.
    #[must_use]
    pub fn frames_received(&self) -> u64 {
        self.shared.frames_received.load(Ordering::Relaxed)
    }

    /// Whether the link thread is still running.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.shared.running.load(Ordering::Acquire)
    }

    /// Queues a message for transmission.
    ///
    /// Returns false if the link has stopped. Never blocks: a wedged link must not stall the
    /// caller.
    pub fn send(&self, message: &MavMessage) -> bool {
        queue_frame(
            &self.outbound,
            self.config.sysid,
            self.config.compid,
            message,
        )
    }

    /// A handle that can send on this link from another thread.
    ///
    /// For the thing that must not wait for a frame: the joystick reader (D15's 5 ms from stick
    /// to wire is not reachable from a UI timer, so it runs on a thread of its own and sends
    /// from there). Everything `send` needs is cheap to copy - the outbound queue is a channel
    /// and the ids are two bytes - so the handle carries copies rather than a borrow of the link,
    /// and it cannot keep anything alive: when the link thread stops, the channel closes and
    /// [`LinkSender::send`] reports false, exactly as [`Link::send`] does.
    #[must_use]
    pub fn sender(&self) -> LinkSender {
        LinkSender {
            outbound: self.outbound.clone(),
            sysid: self.config.sysid,
            compid: self.config.compid,
        }
    }

    /// How the link describes itself right now, e.g. `udp:0.0.0.0:14550 <-> 127.0.0.1:52341`.
    #[must_use]
    pub fn description(&self) -> String {
        self.shared
            .description
            .lock()
            .map(|d| d.clone())
            .unwrap_or_else(|_| self.description.clone())
    }

    /// Stops the link thread and waits for it.
    pub fn close(&mut self) {
        self.shared.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        self.close();
    }
}

/// Drops the oldest finished requests beyond [`FINISHED_REQUESTS_KEPT`]. One still waiting is
/// never dropped, however old: something is still going to answer it or time it out.
fn forget_finished_requests(held: &mut BTreeMap<RequestId, Request>) {
    let mut finished = held
        .values()
        .filter(|request| request.is_finished())
        .count();
    while finished > FINISHED_REQUESTS_KEPT {
        let Some(oldest) = held
            .iter()
            .find(|(_, request)| request.is_finished())
            .map(|(id, _)| *id)
        else {
            break;
        };
        held.remove(&oldest);
        finished -= 1;
    }
}

/// Encodes a message from this link, numbers it, and writes it to the transport and recording.
fn send_message(
    transport: &mut dyn Transport,
    recorder: Option<&mut tlog::TlogWriter>,
    stats: &mut LinkStats,
    config: &LinkConfig,
    tx_seq: &mut u8,
    message: &MavMessage,
) -> bool {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let Some(body) = payload.get(..len) else {
        return false;
    };
    let Ok(n) = encode_v2(
        &mut frame,
        *tx_seq,
        config.sysid,
        config.compid,
        message.id(),
        body,
        message.crc_extra(),
        0,
    ) else {
        return false;
    };
    *tx_seq = tx_seq.wrapping_add(1);
    frame
        .get(..n)
        .is_some_and(|bytes| send_frame(transport, recorder, stats, bytes))
}

/// Writes a frame to the transport and to the recording, if one is running.
///
/// Both directions belong in a telemetry log. Mission Planner records what it sends as well as
/// what it receives, and a recording missing every command the ground station sent is exactly the
/// recording you cannot use to work out why a vehicle did what it did.
fn send_frame(
    transport: &mut dyn Transport,
    recorder: Option<&mut tlog::TlogWriter>,
    stats: &mut LinkStats,
    bytes: &[u8],
) -> bool {
    if transport.write_all(bytes).is_err() {
        return false;
    }
    stats.bytes_written += bytes.len() as u64;
    stats.frames_sent += 1;
    if let Some(writer) = recorder {
        // A failed write to the log must not stop the link. The recording is a record of the
        // flight; the flight matters more.
        let _ = writer.write_frame(bytes);
    }
    true
}

/// The link thread.
fn run_link(
    mut transport: Box<dyn Transport>,
    config: LinkConfig,
    shared: &Arc<Shared>,
    outbound: &std::sync::mpsc::Receiver<Vec<u8>>,
) {
    let mut decoder = FrameDecoder::new();
    let mut registry = VehicleRegistry::new();
    let mut recorder = config.record_path.as_ref().and_then(|path| {
        tlog::TlogWriter::create(path)
            .inspect_err(|e| eprintln!("recording disabled: {e}"))
            .ok()
    });
    let mut last_flush = Instant::now();
    let mut buf = [0u8; 4096];
    let mut stats = LinkStats::default();
    let mut tx_seq: u8 = 0;

    let mut pending_actions: Vec<(VehicleId, u8, Action)> = Vec::new();
    // Reused every pass, so a pass with nothing in flight allocates nothing.
    let mut param_actions: Vec<(VehicleId, ParamAction)> = Vec::new();
    let mut picked_up: Vec<(RequestId, Request)> = Vec::new();
    let mut request_sends: Vec<requests::Outgoing> = Vec::new();
    let mut ftp_sends: Vec<(VehicleId, mp_ftp::mavftp::wire::Header)> = Vec::new();
    let mut last_publish = Instant::now();
    let mut last_heartbeat = Instant::now() - config.heartbeat_interval;
    // Every vehicle heard, with `UpdateCurrentSettings`' clocks for it (see `current_settings`).
    let mut known: BTreeMap<VehicleId, current_settings::Clocks> = BTreeMap::new();
    let gcs = VehicleId::new(config.sysid, config.compid);

    while shared.running.load(Ordering::Acquire) {
        // Inbound.
        let read_started = Instant::now();
        match transport.read(&mut buf) {
            Ok(0) => {
                if !transport.is_open() {
                    break;
                }
                // A blocking transport has already spent its timeout here. One that returns
                // immediately - an in-memory double, an exhausted replay, a non-blocking socket -
                // would otherwise spin a core flat. Idle CPU is a stated budget for this port, so
                // yield only when the read cost us nothing.
                if read_started.elapsed() < IDLE_POLL {
                    std::thread::sleep(IDLE_POLL);
                }
            }
            Ok(n) => {
                stats.bytes_read += n as u64;
                // Each packet is stamped with when it was sent before it is applied: now on a
                // live link, the recording's clock in a replay, whose reads never cross a record.
                // C#: MAVLinkInterface.cs:4721, 6649
                let sent_at = match transport.read_time() {
                    ReadTime::Live => DateTime::now(),
                    ReadTime::Recorded(stamp) => stamp
                        .and_then(DateTime::from_tlog_micros)
                        .unwrap_or(DateTime::MIN),
                };
                let arrived = Instant::now();
                if let Some(chunk) = buf.get(..n) {
                    decoder.push_and_drain(chunk, &DIALECT, |frame| {
                        shared.frames_received.fetch_add(1, Ordering::Relaxed);
                        if let Some(writer) = recorder.as_mut() {
                            // Record the frame exactly as received, before any interpretation:
                            // a recording must not depend on our decoder understanding it.
                            let _ = writer.write_frame(frame.raw);
                        }
                        if let Some(msg) = MavMessage::decode(frame.msgid, frame.payload) {
                            let id = registry.apply_at(
                                frame.sysid,
                                frame.compid,
                                frame.seq,
                                &msg,
                                sent_at,
                            );
                            known
                                .entry(id)
                                .or_insert_with(|| current_settings::Clocks::new(arrived))
                                .heard(&msg);
                            // The fence the vehicle holds, as MAVState.fencepoints has it: from
                            // this link's own upload, before the transfer moves on, and from
                            // whatever passes (see `fence_points`).
                            file_fence_upload(shared, id, gcs, &msg);
                            if matches!(
                                msg,
                                MavMessage::MissionCount(_)
                                    | MavMessage::MissionItem(_)
                                    | MavMessage::MissionItemInt(_)
                                    | MavMessage::FencePoint(_)
                            ) && let Ok(mut held) = shared.fence_points.lock()
                            {
                                held.observe(frame.sysid, frame.compid, config.sysid, &msg);
                            }
                            // Mission transfer is lock-step, so every relevant message may
                            // produce exactly one reply. The state machine decides which.
                            //
                            // Which state machine is decided by the list type on the message: the
                            // mission, the geofence and the rally points use the same messages,
                            // so routing by vehicle alone would let a fence's MISSION_COUNT be
                            // answered as if it were the mission's - and write a geofence into
                            // the flight plan.
                            if let Some((kind, action)) = route_transfer(shared, id, &msg)
                                && action != Action::Nothing
                            {
                                pending_actions.push((id, kind, action));
                            }

                            // What the vehicle says about itself, and about our commands. Both
                            // go to one log because that is how an operator reads them: "Arm
                            // denied" and "PreArm: Compass not calibrated" arrive together and
                            // only make sense together.
                            match &msg {
                                MavMessage::Statustext(text) => {
                                    if let Ok(mut log) = shared.messages.lock() {
                                        log.push(
                                            id,
                                            messages::Severity::from_wire(text.severity),
                                            messages::decode_status_text(&text.text),
                                        );
                                    }
                                }
                                // The vehicle asking for the airframe to be moved. It uses
                                // COMMAND_LONG in the reverse of its usual direction, which is
                                // unusual enough that it is easy to miss: the same command id
                                // carries the request and our confirmation.
                                MavMessage::CommandLong(long)
                                    if long.command == mp_calibration::CMD_ACCELCAL_VEHICLE_POS =>
                                {
                                    #[allow(
                                        clippy::cast_possible_truncation,
                                        clippy::cast_sign_loss
                                    )]
                                    let value = long.param1 as u32;
                                    let state = mp_calibration::AccelCalibration::from_wire(value);
                                    if let Ok(mut held) = shared.accel_calibration.lock() {
                                        *held = state;
                                    }
                                }
                                // Compass calibration reports per compass, because ArduPilot
                                // calibrates every enabled one at once. A vehicle can have its
                                // external compass pass and its internal one fail.
                                MavMessage::MagCalProgress(progress) => {
                                    if let Ok(mut held) = shared.compass_calibration.lock() {
                                        held.observe_progress(progress);
                                    }
                                }
                                MavMessage::MagCalReport(report) => {
                                    if let Ok(mut held) = shared.compass_calibration.lock() {
                                        held.observe_report(report);
                                    }
                                }
                                // The vehicle listing what it holds. One message per log.
                                // Other aircraft. ArduPilot forwards what its ADS-B receiver
                                // hears, one message per aircraft per update.
                                MavMessage::AdsbVehicle(adsb) => {
                                    if let Ok(position) =
                                        mp_units::LatLon::from_mavlink_e7(adsb.lat, adsb.lon)
                                        && let Ok(mut held) = shared.traffic.lock()
                                    {
                                        held.observe(traffic::Traffic {
                                            icao: adsb.icao_address,
                                            callsign: traffic::decode_callsign(&adsb.callsign),
                                            position,
                                            // Millimetres on the wire, which is the same unit
                                            // GLOBAL_POSITION_INT uses and is easy to read as
                                            // metres by mistake.
                                            altitude: f64::from(adsb.altitude) / 1000.0,
                                            // Centi-degrees, and 0 is a legitimate heading, so
                                            // absence is signalled by the flags rather than by
                                            // the value.
                                            heading: (adsb.flags & ADSB_VALID_HEADING != 0)
                                                .then(|| f64::from(adsb.heading) / 100.0),
                                            speed: (adsb.flags & ADSB_VALID_VELOCITY != 0)
                                                .then(|| f64::from(adsb.hor_velocity) / 100.0),
                                            last_seen: Instant::now(),
                                        });
                                    }
                                }
                                MavMessage::LogEntry(entry) => {
                                    if let Ok(mut held) = shared.log_listings.lock() {
                                        // num_logs of zero means the vehicle holds none, and it
                                        // still sends one LOG_ENTRY to say so. Recording that as
                                        // a log would offer the operator a download of nothing.
                                        if entry.num_logs > 0 {
                                            held.insert(
                                                entry.id,
                                                mp_ftp::logs::LogListing {
                                                    id: entry.id,
                                                    size: entry.size,
                                                    time_utc: entry.time_utc,
                                                },
                                            );
                                        }
                                    }
                                }
                                // MAVFTP: to the sending vehicle's client (see `ftp`).
                                MavMessage::FileTransferProtocol(message) => {
                                    ftp::route(shared, id, message, Instant::now(), &mut ftp_sends);
                                }
                                MavMessage::LogData(data) => {
                                    if let Ok(mut held) = shared.log_download.lock()
                                        && let Some(download) = held.as_mut()
                                        && download.id == data.id
                                    {
                                        let count = usize::from(data.count);
                                        if let Some(slice) = data.data.get(..count) {
                                            download.receive(data.ofs, slice);
                                        }
                                    }
                                }
                                // Any MISSION_CURRENT answers a set-current (setWPCurrentAsync).
                                MavMessage::MissionCurrent(_) => {
                                    if let Ok(mut held) = shared.requests.lock() {
                                        for request in held.values_mut() {
                                            request.on_mission_current(id);
                                        }
                                    }
                                }
                                MavMessage::CommandAck(ack) => {
                                    // The oldest command waiting for this ack takes it.
                                    if let Ok(mut held) = shared.requests.lock() {
                                        let now = Instant::now();
                                        for request in held.values_mut() {
                                            if request.on_command_ack(
                                                id,
                                                ack.command,
                                                ack.result,
                                                now,
                                            ) {
                                                break;
                                            }
                                        }
                                    }
                                    if let Ok(mut log) = shared.messages.lock() {
                                        let severity = if messages::command_failed(ack.result) {
                                            messages::Severity::Error
                                        } else {
                                            messages::Severity::Info
                                        };
                                        let command =
                                            MavCmd(u32::from(ack.command)).name().map_or_else(
                                                || format!("command {}", ack.command),
                                                ToOwned::to_owned,
                                            );
                                        log.push(
                                            id,
                                            severity,
                                            format!(
                                                "{command}: {}",
                                                messages::command_result_name(ack.result)
                                            ),
                                        );
                                    }
                                }
                                _ => {}
                            }

                            if let MavMessage::ParamValue(param) = msg {
                                let name = decode_param_id(&param.param_id);
                                // ArduPilot sends every parameter as a float and uses param_type
                                // only to describe how it stores the value; PX4 and the
                                // specification put the declared type's bytes in the field.
                                // Decoding with the wrong rule turns a 3,264 mAh battery capacity
                                // into 1,162,756,096 - a number that looks like data.
                                let is_ardupilot = registry.working(id).is_some_and(|state| {
                                    state.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA
                                });
                                // Every PARAM_VALUE counts toward a download, whatever its
                                // type, as every one does in getParamListAsync.
                                if let Ok(mut downloads) = shared.param_downloads.lock()
                                    && let Some(download) = downloads.get_mut(&id)
                                {
                                    download.on_param_value(
                                        param.param_index,
                                        param.param_count,
                                        Instant::now(),
                                    );
                                }
                                if let Some(kind) = ParamType::from_wire(param.param_type) {
                                    let airspeed = AIRSPEED_MIN_PARAMS.contains(&name.as_str());
                                    let value = if is_ardupilot {
                                        ParamValue::from_ardupilot(param.param_value, kind)
                                    } else {
                                        ParamValue::from_param_value_field(param.param_value, kind)
                                    };
                                    // A set or read waiting for this parameter is answered by it,
                                    // before the table takes the name. The requests stay locked
                                    // until the table has the value, so a caller that sees the
                                    // request finished finds the table already agreeing with it.
                                    // Lock order: requests, then parameters.
                                    let mut held = shared.requests.lock();
                                    if let Ok(held) = held.as_mut() {
                                        for request in held.values_mut() {
                                            request.on_param_value(
                                                id,
                                                &name,
                                                param.param_index,
                                                value,
                                            );
                                        }
                                    }
                                    if let Ok(mut table) = shared.params.lock() {
                                        let table = table.entry(id).or_default();
                                        table.insert(
                                            name,
                                            value,
                                            param.param_index,
                                            param.param_count,
                                        );
                                        // The low-airspeed warning reads these from the
                                        // vehicle's `MAV.param`, which every PARAM_VALUE
                                        // updates as it passes.
                                        // C#: MAVLinkInterface.cs:5766-5796;
                                        // CurrentState.cs:3858-3880
                                        if airspeed && let Some(state) = registry.working_mut(id) {
                                            let [min, fbw_min] = AIRSPEED_MIN_PARAMS.map(|name| {
                                                table.get(name).map(ParamValue::as_f64)
                                            });
                                            state.set_airspeed_min_params(min, fbw_min);
                                        }
                                    }
                                    drop(held);
                                }
                            }
                        }
                    });
                }
            }
            Err(_) => break,
        }

        // What the screens wrote into a vehicle's state since the last pass.
        if let Ok(mut writes) = shared.state_writes.lock() {
            for (id, write) in writes.drain(..) {
                let Some(state) = registry.working_mut(id) else {
                    continue;
                };
                match write {
                    StateWrite::StreamRates(rates) => state.rates = rates,
                    StateWrite::AltOffsetHome(offset) => state.alt_offset_home = offset,
                }
            }
        }

        // `UpdateCurrentSettings` on every vehicle listed, after each read, at most every 50 ms
        // each; and inside it, the telemetry streams asked for at each vehicle's own rates -
        // never while a recording is played, which the C# plays with its port closed. See
        // `current_settings`. C#: MainV2.cs:3058-3069; CurrentState.cs:4580-4663
        let now = Instant::now();
        let recording = matches!(transport.read_time(), ReadTime::Recorded(_));
        let port_open = transport.is_open() && !recording;
        for (id, clocks) in &mut known {
            if !clocks.due(now) {
                continue;
            }
            let Some(state) = registry.working_mut(*id) else {
                continue;
            };
            // `BaseStream != null && !BaseStream.IsOpen && !logreadmode`.
            state.update_current_settings(!transport.is_open() && !recording);
            let rates = state.rates;
            if config.stream_rate_hz == 0
                || !port_open
                || !clocks.streams_due(now, config.timeouts.stream_rerequest)
            {
                continue;
            }
            for (stream, hz) in current_settings::stream_requests(rates) {
                let Some(request) = current_settings::request_datastream(*id, stream, hz) else {
                    continue;
                };
                // `getDatastream` sends each one twice. C#: MAVLinkInterface.cs:3262-3263
                for _ in 0..2 {
                    send_message(
                        transport.as_mut(),
                        recorder.as_mut(),
                        &mut stats,
                        &config,
                        &mut tx_seq,
                        &request,
                    );
                }
            }
        }

        // MAVFTP: let each client's wait run out, and send what the clients want sent - this
        // pass's replies' answers included.
        ftp::tick(shared, Instant::now(), &mut ftp_sends);
        for (id, payload) in ftp_sends.drain(..) {
            send_message(
                transport.as_mut(),
                recorder.as_mut(),
                &mut stats,
                &config,
                &mut tx_seq,
                &ftp::ftp_message(id, &payload),
            );
        }

        // Pick up transfers the caller queued, and start them.
        if let Ok(mut queued) = shared.mission_requests.lock() {
            for transfer in queued.drain(..) {
                let transfer = transfer.with_timeouts(config.timeouts);
                let id = transfer.target;
                let kind = transfer.mission_type;
                let first = transfer.begin();
                if let Ok(mut transfers) = shared.missions.lock() {
                    transfers.insert((id, kind), transfer);
                }
                pending_actions.push((id, kind, first));
            }
        }

        // Retry whatever step is outstanding.
        if let Ok(mut transfers) = shared.missions.lock() {
            for ((id, kind), transfer) in transfers.iter_mut() {
                let action = transfer.on_tick();
                if action != Action::Nothing {
                    pending_actions.push((*id, *kind, action));
                }
            }
        }

        // Send whatever the state machines decided, outside their lock.
        for (id, kind, action) in pending_actions.drain(..) {
            let message = match action {
                Action::RequestList => commands::request_mission_list(id, kind),
                Action::RequestItem(seq) => commands::request_mission_item(id, seq, kind),
                Action::SendCount(count) => commands::send_mission_count(id, count, kind),
                Action::SendItem(item) => commands::send_mission_item(id, &item, kind),
                Action::SendAck => {
                    commands::send_mission_ack(id, mission_transfer::MISSION_ACCEPTED, kind)
                }
                Action::SendPartialList { start, end } => {
                    MavMessage::MissionWritePartialList(MissionWritePartialList {
                        start_index: i16::try_from(start).unwrap_or(i16::MAX),
                        end_index: i16::try_from(end).unwrap_or(i16::MAX),
                        target_system: id.sysid,
                        target_component: id.compid,
                        mission_type: kind,
                    })
                }
                Action::Nothing => continue,
            };
            send_message(
                transport.as_mut(),
                recorder.as_mut(),
                &mut stats,
                &config,
                &mut tx_seq,
                &message,
            );
        }

        // Parameter downloads: chase the holes a lossy stream leaves, as getParamListAsync does.
        // Only a download the caller started is chased; see `Shared::param_downloads`.
        if let Ok(mut downloads) = shared.param_downloads.lock() {
            let now = Instant::now();
            for (id, download) in downloads.iter_mut() {
                match download.on_tick(now) {
                    ParamAction::Nothing => {}
                    action => param_actions.push((*id, action)),
                }
            }
        }
        for (id, action) in param_actions.drain(..) {
            match action {
                ParamAction::Nothing => {}
                ParamAction::RequestList => {
                    send_message(
                        transport.as_mut(),
                        recorder.as_mut(),
                        &mut stats,
                        &config,
                        &mut tx_seq,
                        &commands::request_param_list(id),
                    );
                }
                // Ten at a time rather than every hole at once: asking for hundreds floods a
                // 57,600 baud radio and the replies collide with the telemetry stream.
                ParamAction::RequestIndices(burst) => {
                    for index in burst.as_slice() {
                        send_message(
                            transport.as_mut(),
                            recorder.as_mut(),
                            &mut stats,
                            &config,
                            &mut tx_seq,
                            &commands::request_param_by_index(id, *index),
                        );
                    }
                }
            }
        }

        // Requests: pick up what the caller queued, send it, and retry what is outstanding.
        //
        // The pick-up happens under the table's lock. A caller looking a request up reads the
        // table and then the queue, and between the queue being drained and the table taking
        // the request it was in neither: `Link::request` said `None` for a request that was
        // alive, and a caller that took `None` for "forgotten" gave up on a write the vehicle
        // then accepted. Lock order, as at the PARAM_VALUE arm: requests, then parameters.
        if let Ok(mut held) = shared.requests.lock() {
            if let Ok(mut queue) = shared.request_queue.lock() {
                picked_up.append(&mut queue);
            }
            if !picked_up.is_empty() {
                let now = Instant::now();
                for (_, request) in &mut picked_up {
                    let ardupilot = registry
                        .working(request.target)
                        .is_some_and(|state| state.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA);
                    let send = match shared.params.lock() {
                        Ok(tables) => request.begin(
                            &config.timeouts,
                            tables.get(&request.target),
                            ardupilot,
                            now,
                        ),
                        Err(_) => request.begin(&config.timeouts, None, ardupilot, now),
                    };
                    request_sends.push(send);
                }
                held.extend(picked_up.drain(..));
                forget_finished_requests(&mut held);
            }
        }
        if let Ok(mut held) = shared.requests.lock() {
            let now = Instant::now();
            for request in held.values_mut() {
                match request.on_tick(now) {
                    requests::Outgoing::Nothing => {}
                    send => request_sends.push(send),
                }
            }
        }
        for send in request_sends.drain(..) {
            let (message, times) = match send {
                requests::Outgoing::Nothing => continue,
                requests::Outgoing::Once(message) => (message, 1),
                requests::Outgoing::Twice(message) => (message, 2),
            };
            for _ in 0..times {
                send_message(
                    transport.as_mut(),
                    recorder.as_mut(),
                    &mut stats,
                    &config,
                    &mut tx_seq,
                    &message,
                );
            }
        }

        // Publish snapshots on a cadence rather than per packet: no display can show more than
        // one state per frame, so per-packet publishing is pure overhead.
        if last_publish.elapsed() >= config.publish_interval {
            registry.publish_all();
            if let Ok(mut description) = shared.description.lock() {
                // Borrowed, so asking is free; the text is copied only when it changed - a UDP
                // link learning its peer - and then into the buffer the old text had.
                let current = transport.description();
                if description.as_str() != current {
                    description.clear();
                    description.push_str(current);
                }
            }
            stats.publishes += 1;
            last_publish = Instant::now();

            // Expose handles for any newly discovered vehicle.
            expose_handles(shared, &registry, known.keys());
        }

        // Heartbeat, so the vehicle does not declare GCS failsafe.
        if config.send_heartbeat && last_heartbeat.elapsed() >= config.heartbeat_interval {
            let hb = Heartbeat {
                custom_mode: 0,
                r#type: MAV_TYPE_GCS,
                autopilot: MAV_AUTOPILOT_INVALID,
                base_mode: 0,
                system_status: 0,
                mavlink_version: 3,
            };
            let mut payload = [0u8; Heartbeat::LEN];
            hb.encode(&mut payload);
            let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
            if let Ok(n) = encode_v2(
                &mut frame,
                tx_seq,
                config.sysid,
                config.compid,
                Heartbeat::ID,
                &payload,
                Heartbeat::CRC_EXTRA,
                0,
            ) {
                tx_seq = tx_seq.wrapping_add(1);
                if let Some(bytes) = frame.get(..n) {
                    send_frame(transport.as_mut(), recorder.as_mut(), &mut stats, bytes);
                }
            }
            last_heartbeat = Instant::now();
        }

        // Outbound queue. Re-stamp the sequence number here, where it is owned.
        while let Ok(mut bytes) = outbound.try_recv() {
            if let Some(seq) = bytes.get_mut(4) {
                *seq = tx_seq;
                tx_seq = tx_seq.wrapping_add(1);
            }
            // The checksum covers the sequence byte, so it must be recomputed after re-stamping.
            if let Some(fixed) = restamp_checksum(&bytes) {
                bytes = fixed;
            }
            send_frame(transport.as_mut(), recorder.as_mut(), &mut stats, &bytes);
        }

        // Flush the recording periodically so a crash costs seconds, not the whole flight.
        if let Some(writer) = recorder.as_mut()
            && last_flush.elapsed() >= Duration::from_secs(1)
        {
            let _ = writer.flush();
            last_flush = Instant::now();
        }

        stats.decode = *decoder.stats();
        if let Ok(mut shared_stats) = shared.stats.lock() {
            *shared_stats = stats;
        }
    }

    if let Some(writer) = recorder.as_mut() {
        let _ = writer.flush();
    }
    decoder.flush(&DIALECT, |_| {});
    registry.publish_all();
    // And every vehicle's handle, or a link that ended before its first publish - a short
    // recording played unpaced - would leave the vehicles it heard unreachable.
    expose_handles(shared, &registry, known.keys());
    stats.decode = *decoder.stats();
    if let Ok(mut shared_stats) = shared.stats.lock() {
        *shared_stats = stats;
    }
    shared.running.store(false, Ordering::Release);
}

/// Makes a reader handle reachable through [`Link::vehicle`] for each of `ids` that has none yet.
fn expose_handles<'a>(
    shared: &Shared,
    registry: &VehicleRegistry,
    ids: impl Iterator<Item = &'a VehicleId>,
) {
    if let Ok(mut handles) = shared.handles.lock() {
        for id in ids {
            if !handles.contains_key(id)
                && let Some(handle) = registry.handle(*id)
            {
                handles.insert(*id, handle);
            }
        }
    }
}

/// What this link's own fence upload does to the vehicle's `fencepoints`, read before the transfer
/// moves on: the vehicle's first request, for item 0 or 1, clears it (`setWPTotalAsync`, either
/// request message), and a `MISSION_REQUEST` for the item after the one last sent - or any
/// `MISSION_ACK` - files that one (`setWPAsync`). Only answers addressed to this ground station
/// count, as the C# checks.
///
/// A `MISSION_REQUEST_INT` files nothing, as in the C#: its `setWPAsync` for an `_INT` item has
/// no branch for that message, and the one for a float item files only mission items from it
/// (`:4183-4215`). Against a vehicle that asks with it, only the last item - acknowledged - is
/// filed, until the fence is read back.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3801-3830, 3832-3861, 4098-4133,
/// 4134-4182, 4273-4309, 4310-4346`
fn file_fence_upload(shared: &Arc<Shared>, id: VehicleId, gcs: VehicleId, msg: &MavMessage) {
    const FENCE: u8 = mp_mission::fence::MISSION_TYPE_FENCE;
    // (the item asked for, whether a request may file, to whom it is addressed)
    let (asked, files, target) = match msg {
        MavMessage::MissionRequestInt(m) if m.mission_type == FENCE => {
            (Some(m.seq), false, (m.target_system, m.target_component))
        }
        MavMessage::MissionRequest(m) if m.mission_type == FENCE => {
            (Some(m.seq), true, (m.target_system, m.target_component))
        }
        MavMessage::MissionAck(m) if m.mission_type == FENCE => {
            (None, true, (m.target_system, m.target_component))
        }
        _ => return,
    };
    if target != (gcs.sysid, gcs.compid) {
        return;
    }
    let Ok(transfers) = shared.missions.lock() else {
        return;
    };
    let Some(transfer) = transfers.get(&(id, FENCE)) else {
        return;
    };
    let mission_transfer::TransferState::Uploading { last_requested } = *transfer.state() else {
        return;
    };
    let filed = match (last_requested, asked) {
        // setWPTotalAsync: the first request answers the count.
        (None, Some(0 | 1)) => {
            if let Ok(mut held) = shared.fence_points.lock() {
                held.clear(id);
            }
            return;
        }
        // setWPAsync: on to the next item, or the ack, files the one sent.
        (Some(sent), Some(next)) if files && u32::from(next) == u32::from(sent) + 1 => sent,
        (Some(sent), None) => sent,
        _ => return,
    };
    if let Some(item) = transfer.items().get(usize::from(filed))
        && let Ok(mut held) = shared.fence_points.lock()
    {
        held.store(id, item.seq, fence_points::uploaded(item));
    }
}

/// Hands a message to the transfer it belongs to, and returns what that transfer wants to send.
///
/// `MISSION_ITEM_INT` is the awkward one: it carries no `mission_type` of its own, so it belongs
/// to whichever of this vehicle's transfers is currently waiting for an item. There is only ever
/// one, because each transfer is lock-step and asks for the next item only after the last arrived.
fn route_transfer(shared: &Arc<Shared>, id: VehicleId, msg: &MavMessage) -> Option<(u8, Action)> {
    let declared = match msg {
        MavMessage::MissionCount(m) => Some(m.mission_type),
        MavMessage::MissionRequestInt(m) => Some(m.mission_type),
        MavMessage::MissionRequest(m) => Some(m.mission_type),
        MavMessage::MissionAck(m) => Some(m.mission_type),
        MavMessage::MissionItemInt(_) => None,
        // Not part of a transfer at all.
        _ => return None,
    };

    let mut transfers = shared.missions.lock().ok()?;

    let kind = match declared {
        Some(kind) => kind,
        None => *transfers
            .iter()
            .find(|((target, _), transfer)| *target == id && transfer.expects_item())
            .map(|((_, kind), _)| kind)?,
    };

    let transfer = transfers.get_mut(&(id, kind))?;
    let action = match msg {
        MavMessage::MissionCount(m) => transfer.on_count(m.count),
        MavMessage::MissionItemInt(m) => transfer.on_item(&WireItem {
            seq: m.seq,
            frame: m.frame,
            command: m.command,
            current: m.current,
            autocontinue: m.autocontinue,
            param1: m.param1,
            param2: m.param2,
            param3: m.param3,
            param4: m.param4,
            x: m.x,
            y: m.y,
            z: m.z,
        }),
        // ArduPilot answers a MISSION_COUNT with the request variant matching the protocol the
        // GCS appears to be speaking, and it defaults to the older MISSION_REQUEST (id 40) rather
        // than MISSION_REQUEST_INT (id 51). Handling only the _INT form makes an upload stall at
        // zero percent with no error - the vehicle is waiting for us and we are waiting for it.
        MavMessage::MissionRequestInt(m) => transfer.on_request(m.seq),
        MavMessage::MissionRequest(m) => transfer.on_request(m.seq),
        MavMessage::MissionAck(m) => transfer.on_ack(m.r#type),
        _ => Action::Nothing,
    };
    Some((kind, action))
}

/// Recomputes a v2 frame's checksum after its sequence byte was re-stamped.
fn restamp_checksum(frame: &[u8]) -> Option<Vec<u8>> {
    let payload_len = usize::from(*frame.get(1)?);
    let msgid = u32::from_le_bytes([*frame.get(7)?, *frame.get(8)?, *frame.get(9)?, 0]);
    let crc_extra = mp_mavlink::Dialect::crc_extra(&DIALECT, msgid)?;
    let payload_end = 10 + payload_len;
    let checksum = mp_mavlink::crc::checksum(frame.get(1..payload_end)?, crc_extra);

    let mut out = frame.to_vec();
    out.get_mut(payload_end..payload_end + 2)?
        .copy_from_slice(&checksum.to_le_bytes());
    Some(out)
}
