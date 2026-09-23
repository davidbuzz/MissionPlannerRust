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

#![forbid(unsafe_code)]

pub mod commands;
pub mod messages;
pub mod mission_transfer;
pub mod params;

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mission_transfer::{Action, MissionTransfer};
use mp_mavlink::{DecodeStats, FrameDecoder, Message as _, encode_v2};
use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavCmd, MavMessage, RequestDataStream};
use mp_mission::{MISSION_TYPE_MISSION, MissionItem, WireItem};
use mp_transport::{OpenError, Transport};
use mp_vehicle::{StateHandle, VehicleId, VehicleRegistry};
use params::{ParamTable, ParamType, ParamValue, decode_param_id};

/// MAVLink component id for a ground control station.
pub const MAV_COMP_ID_MISSIONPLANNER: u8 = 190;
/// `MAV_TYPE_GCS`.
const MAV_TYPE_GCS: u8 = 6;
/// `MAV_AUTOPILOT_INVALID`, which is what a GCS reports.
const MAV_AUTOPILOT_INVALID: u8 = 8;

/// `MAV_AUTOPILOT_ARDUPILOTMEGA`. ArduPilot encodes parameters differently from the
/// specification, so which autopilot is on the other end is not a cosmetic detail.
const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;

/// How long the parameter stream must be quiet before gaps are re-requested.
const PARAM_GAP_TIMEOUT: Duration = Duration::from_millis(1500);

/// How many missing parameters to re-request at once.
const PARAM_RETRY_BURST: usize = 10;

/// Minimum time the I/O loop spends per iteration when there is nothing to read.
const IDLE_POLL: Duration = Duration::from_millis(1);

/// The `MAV_DATA_STREAM` ids Mission Planner turns on when it connects: raw sensors, extended
/// status, RC channels, position and the three "extra" groups that carry attitude and VFR data.
const STREAMS: &[u8] = &[1, 2, 3, 6, 10, 11, 12];

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
    /// Telemetry rate to request from each vehicle, in hertz. Zero disables the request.
    ///
    /// ArduPilot streams almost nothing until a GCS asks: a fresh SITL sends only heartbeats.
    /// Mission Planner sends `REQUEST_DATA_STREAM` on connect for exactly this reason, and a
    /// port that omits it looks like a broken link rather than a quiet vehicle.
    pub stream_rate_hz: u16,
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
        }
    }
}

/// Errors from running a link.
#[derive(Debug, thiserror::Error)]
pub enum LinkError {
    /// The transport could not be opened.
    #[error(transparent)]
    Open(#[from] OpenError),
    /// The link thread could not be started.
    #[error("could not start link thread: {0}")]
    Thread(String),
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
    /// Parameters received per vehicle, and whether a download is in progress.
    params: Mutex<BTreeMap<VehicleId, ParamTable>>,
    /// Vehicles whose parameter download the caller has asked for.
    param_downloads: Mutex<BTreeMap<VehicleId, bool>>,
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
    stats: Mutex<LinkStats>,
    running: AtomicBool,
    frames_received: AtomicU64,
}

/// A running link.
#[derive(Debug)]
pub struct Link {
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
        let description = transport.description();
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
    /// Progress is observable through [`Link::params`]; the link thread re-requests any gaps, so
    /// the caller does not need to implement retry logic.
    pub fn download_params(&self, target: VehicleId) -> bool {
        if let Ok(mut downloads) = self.shared.param_downloads.lock() {
            downloads.insert(target, true);
        }
        self.send(&commands::request_param_list(target))
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

    /// A snapshot of a vehicle's parameters.
    #[must_use]
    pub fn params(&self, target: VehicleId) -> Option<ParamTable> {
        self.shared.params.lock().ok()?.get(&target).cloned()
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
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let Some(payload) = payload.get(..len) else {
            return false;
        };

        // Sequence numbering is the link thread's job, so it is applied there; zero here.
        let Ok(n) = encode_v2(
            &mut frame,
            0,
            self.config.sysid,
            self.config.compid,
            message.id(),
            payload,
            message.crc_extra(),
            0,
        ) else {
            return false;
        };
        frame
            .get(..n)
            .is_some_and(|bytes| self.outbound.send(bytes.to_vec()).is_ok())
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
        mp_log::TlogWriter::create(path)
            .inspect_err(|e| eprintln!("recording disabled: {e}"))
            .ok()
    });
    let mut last_flush = Instant::now();
    let mut buf = [0u8; 4096];
    let mut stats = LinkStats::default();
    let mut tx_seq: u8 = 0;

    let mut newly_seen: Vec<VehicleId> = Vec::new();
    let mut last_param = Instant::now();
    let mut pending_actions: Vec<(VehicleId, u8, Action)> = Vec::new();
    let mut last_publish = Instant::now();
    let mut last_heartbeat = Instant::now() - config.heartbeat_interval;
    let mut known: BTreeMap<VehicleId, ()> = BTreeMap::new();

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
                if let Some(chunk) = buf.get(..n) {
                    decoder.push_and_drain(chunk, &DIALECT, |frame| {
                        shared.frames_received.fetch_add(1, Ordering::Relaxed);
                        if let Some(writer) = recorder.as_mut() {
                            // Record the frame exactly as received, before any interpretation:
                            // a recording must not depend on our decoder understanding it.
                            let _ = writer.write_frame(frame.raw);
                        }
                        if let Some(msg) = MavMessage::decode(frame.msgid, frame.payload) {
                            let id = registry.apply(frame.sysid, frame.compid, frame.seq, &msg);
                            if known.insert(id, ()).is_none() {
                                newly_seen.push(id);
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
                                MavMessage::CommandAck(ack) => {
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
                                if let (Some(kind), Ok(mut table)) =
                                    (ParamType::from_wire(param.param_type), shared.params.lock())
                                {
                                    let value = if is_ardupilot {
                                        ParamValue::from_ardupilot(param.param_value, kind)
                                    } else {
                                        ParamValue::from_param_value_field(param.param_value, kind)
                                    };
                                    table.entry(id).or_default().insert(
                                        name,
                                        value,
                                        param.param_index,
                                        param.param_count,
                                    );
                                    last_param = Instant::now();
                                }
                            }
                        }
                    });
                }
            }
            Err(_) => break,
        }

        // Ask a newly discovered vehicle to start streaming telemetry.
        for id in newly_seen.drain(..) {
            if config.stream_rate_hz == 0 || id.compid != 1 {
                continue;
            }
            for stream_id in STREAMS {
                let req = RequestDataStream {
                    req_message_rate: config.stream_rate_hz,
                    target_system: id.sysid,
                    target_component: id.compid,
                    req_stream_id: *stream_id,
                    start_stop: 1,
                };
                let mut payload = [0u8; RequestDataStream::LEN];
                req.encode(&mut payload);
                let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
                if let Ok(n) = encode_v2(
                    &mut frame,
                    tx_seq,
                    config.sysid,
                    config.compid,
                    RequestDataStream::ID,
                    &payload,
                    RequestDataStream::CRC_EXTRA,
                    0,
                ) {
                    tx_seq = tx_seq.wrapping_add(1);
                    if let Some(bytes) = frame.get(..n)
                        && transport.write_all(bytes).is_ok()
                    {
                        stats.bytes_written += bytes.len() as u64;
                        stats.frames_sent += 1;
                    }
                }
            }
        }

        // Pick up transfers the caller queued, and start them.
        if let Ok(mut queued) = shared.mission_requests.lock() {
            for transfer in queued.drain(..) {
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
                Action::RequestList => Some(commands::request_mission_list(id, kind)),
                Action::RequestItem(seq) => Some(commands::request_mission_item(id, seq, kind)),
                Action::SendCount(count) => Some(commands::send_mission_count(id, count, kind)),
                Action::SendItem(item) => Some(commands::send_mission_item(id, &item, kind)),
                Action::SendAck => Some(commands::send_mission_ack(
                    id,
                    mission_transfer::MISSION_ACCEPTED,
                    kind,
                )),
                Action::Nothing => None,
            };
            let Some(message) = message else { continue };

            let mut payload = [0u8; 255];
            let len = message.encode(&mut payload);
            let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
            if let Some(body) = payload.get(..len)
                && let Ok(n) = encode_v2(
                    &mut frame,
                    tx_seq,
                    config.sysid,
                    config.compid,
                    message.id(),
                    body,
                    message.crc_extra(),
                    0,
                )
            {
                tx_seq = tx_seq.wrapping_add(1);
                if let Some(bytes) = frame.get(..n)
                    && transport.write_all(bytes).is_ok()
                {
                    stats.bytes_written += bytes.len() as u64;
                    stats.frames_sent += 1;
                }
            }
        }

        // Parameter gap recovery. PARAM_REQUEST_LIST streams once with no retransmission, so a
        // single lost packet on a telemetry link leaves a hole that never fills by itself. When
        // the stream goes quiet with gaps outstanding, ask for them individually.
        if last_param.elapsed() >= PARAM_GAP_TIMEOUT {
            let outstanding: Vec<(VehicleId, Vec<u16>)> = shared
                .params
                .lock()
                .map(|tables| {
                    tables
                        .iter()
                        .filter(|(_, table)| !table.is_complete() && table.expected().is_some())
                        .map(|(id, table)| (*id, table.missing()))
                        .collect()
                })
                .unwrap_or_default();

            for (id, missing) in outstanding {
                // A burst rather than the whole list: asking for hundreds at once floods a
                // 57,600 baud radio and the replies collide with the telemetry stream.
                for index in missing.into_iter().take(PARAM_RETRY_BURST) {
                    let request = commands::request_param_by_index(id, index);
                    let mut payload = [0u8; 255];
                    let len = request.encode(&mut payload);
                    let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
                    if let Some(body) = payload.get(..len)
                        && let Ok(n) = encode_v2(
                            &mut frame,
                            tx_seq,
                            config.sysid,
                            config.compid,
                            request.id(),
                            body,
                            request.crc_extra(),
                            0,
                        )
                    {
                        tx_seq = tx_seq.wrapping_add(1);
                        if let Some(bytes) = frame.get(..n)
                            && transport.write_all(bytes).is_ok()
                        {
                            stats.bytes_written += bytes.len() as u64;
                            stats.frames_sent += 1;
                        }
                    }
                }
            }
            last_param = Instant::now();
        }

        // Publish snapshots on a cadence rather than per packet: no display can show more than
        // one state per frame, so per-packet publishing is pure overhead.
        if last_publish.elapsed() >= config.publish_interval {
            registry.publish_all();
            if let Ok(mut description) = shared.description.lock() {
                let current = transport.description();
                if *description != current {
                    *description = current;
                }
            }
            stats.publishes += 1;
            last_publish = Instant::now();

            // Expose handles for any newly discovered vehicle.
            if let Ok(mut handles) = shared.handles.lock() {
                for id in known.keys() {
                    if !handles.contains_key(id)
                        && let Some(handle) = registry.handle(*id)
                    {
                        handles.insert(*id, handle);
                    }
                }
            }
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
                if let Some(bytes) = frame.get(..n)
                    && transport.write_all(bytes).is_ok()
                {
                    stats.bytes_written += bytes.len() as u64;
                    stats.frames_sent += 1;
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
            if transport.write_all(&bytes).is_ok() {
                stats.bytes_written += bytes.len() as u64;
                stats.frames_sent += 1;
            }
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
    stats.decode = *decoder.stats();
    if let Ok(mut shared_stats) = shared.stats.lock() {
        *shared_stats = stats;
    }
    shared.running.store(false, Ordering::Release);
}

/// Recomputes a v2 frame's checksum after its sequence byte was re-stamped.
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
