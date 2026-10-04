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

//! Counts what the real link thread allocates per packet (DELIVERABLES.md Deliverable 5, PLAN.md §8.2).
//!
//! `mp-vehicle/tests/no_alloc_ingest.rs` proves the stages from bytes to a published
//! `VehicleState` allocate nothing. This test runs the code that strings those stages together -
//! `run_link`, the private loop behind [`Link`] - with everything it does around them: the tlog
//! recording every GCS session writes, routing to mission transfers, the message log, parameter
//! tables, heartbeats, and publishing snapshots and handles. Nothing is reimplemented here, so
//! nothing can drift from what ships.
//!
//! # How a link thread is measured from outside it
//!
//! The link thread is private, but it calls the transport, and the transport is ours. Each
//! `read` returns exactly one recorded frame, so everything the thread does between one `read` and
//! the next is the cost of that one packet. The transport counts this thread's allocations at each
//! `read` and switches counting off while it does its own bookkeeping. Allocation counting is
//! per thread, so the test thread, the harness and other tests are never charged.
//!
//! The link runs with the configuration the GUI uses (`LinkConfig::default()` plus a recording)
//! except for two timers: it publishes and sends its heartbeat on every loop iteration, instead of
//! every 20 ms and every second. That makes both part of every packet's cost, which is the
//! strictest reading of "per packet", and it does not depend on how fast the machine replays.
//!
//! # Steady state
//!
//! Each recording is served twice through the same link and only the second pass is measured. The
//! first pass grows everything that grows once: a registry entry, handle and stream request per
//! vehicle, the parameter tables (one entry per name), and the message log's ring. Frames are
//! served without the tlog's timestamps, as a live serial or UDP link delivers them; timestamp
//! resync is covered by the other two allocation tests, which feed the raw file.
//!
//! # What is allowed to allocate, and why
//!
//! Three message types allocate by design, in [`EXCLUDED`] with the reason for each. They are
//! events a person reads, not telemetry: a parameter arriving, the vehicle saying something, the
//! vehicle answering a command. Each is bounded per frame, and the test fails if one stops
//! allocating, so the list cannot outlive its reasons.
//!
//! Nothing else is allowed, publishing included. The link asks the transport for its description
//! on every publish, to notice a UDP link learning its peer; `Transport::description` lends its
//! text, so asking costs nothing, and the cost is charged to the packet like everything else the
//! loop does. [`an_allocating_description_is_caught`] proves a description that allocates again
//! would fail this test. One further cost is not per packet and is reported separately: parameter
//! gap recovery, which runs on a timer - and only inside a download the caller started, which
//! this test never does, so here it must not run at all.

// A global allocator is an `unsafe impl` by definition: `GlobalAlloc`'s contract cannot be
// stated in safe Rust. This is a test binary; `mp-link`'s own source forbids `unsafe`.
#![allow(unsafe_code)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

use mp_os::Lock as _;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::BTreeMap;
use std::hint::black_box;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::DIALECT;
use mp_transport::{ReplayTransport, Transport};
use mp_vehicle::VehicleId;

thread_local! {
    /// Whether this thread's allocations are being counted. Only the link thread ever sets it,
    /// from inside the transport.
    static WATCHING: Cell<bool> = const { Cell::new(false) };
    /// `alloc`, `alloc_zeroed` and `realloc` calls made on this thread while watching.
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
}

/// Counts an allocation if this thread is watching.
///
/// `try_with` rather than `with`, because an allocator can run during thread teardown and must
/// never panic. The cells are const-initialised and have no destructor, so touching them needs no
/// lazy setup and cannot recurse into the allocator.
fn note_allocation() {
    let watching = WATCHING.try_with(Cell::get).unwrap_or(false);
    if watching {
        let _ = ALLOCATIONS.try_with(|n| n.set(n.get() + 1));
    }
}

struct CountingAllocator;

// SAFETY: every method forwards to `System` with its arguments unchanged, so this allocator makes
// exactly the guarantees `System` makes. The only addition is a thread-local counter increment,
// which neither allocates nor unwinds.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        note_allocation();
        // SAFETY: the caller upholds `GlobalAlloc::alloc`'s contract, which is all `System` needs.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        note_allocation();
        // SAFETY: as for `alloc`; the contract is the same.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` was returned by this allocator, which means by `System`, for `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        note_allocation();
        // SAFETY: `ptr` and `layout` came from `System` via this allocator, and the caller upholds
        // `realloc`'s contract for `new_size`.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn allocations_so_far() -> u64 {
    ALLOCATIONS.with(Cell::get)
}

/// A message type that allocates on the link thread by design.
struct Excluded {
    msgid: u32,
    name: &'static str,
    /// The most one frame of this type may allocate in steady state.
    max_per_frame: u64,
    /// Where, and why that is acceptable.
    why: &'static str,
}

/// Everything allowed to allocate per packet: these three message types, and nothing else - not
/// the publish, and not the transport's description the link asks for on each one. Kept short and
/// named on purpose; a new entry needs a reason as good as these.
const EXCLUDED: &[Excluded] = &[
    Excluded {
        msgid: 22,
        name: "PARAM_VALUE",
        // `decode_param_id` builds the name (mp-params/src/lib.rs:295) and `ParamTable::insert`
        // clones it for the index map (lib.rs:353); the table already holds that name, so the
        // key passed to `values.insert` (lib.rs:355) is dropped, not stored.
        max_per_frame: 2,
        why: "the parameter table is keyed by name, as Mission Planner's is \
              (C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5770 builds a string per \
              PARAM_VALUE). A parameter arrives when one is read or set, not as telemetry.",
    },
    Excluded {
        msgid: 77,
        name: "COMMAND_ACK",
        // In `run_link`'s COMMAND_ACK arm: the command's name (mp-link/src/lib.rs:1120, `to_owned`
        // or `format!`) and the logged line (lib.rs:1126, `format!`), which starts with no capacity
        // because its format string begins with an argument, so it allocates and then grows.
        max_per_frame: 3,
        why: "the message log holds text an operator reads; an ack answers a command the \
              operator sent, so it arrives at the rate of button presses, not telemetry.",
    },
    Excluded {
        msgid: 253,
        name: "STATUSTEXT",
        // The text (mp-link/src/messages.rs:215, `to_owned`), plus the log ring growing
        // (messages.rs:164) until it holds `messages::CAPACITY` lines - a recording with fewer
        // than that across both passes is still growing it when measured.
        max_per_frame: 2,
        why: "the message log holds the vehicle's own words as owned text, as Mission Planner \
              does (C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5380-5386 adds a string \
              to cs.messages per STATUSTEXT).",
    },
];

/// `PARAM_REQUEST_READ`, which only parameter gap recovery sends.
const PARAM_REQUEST_READ: u32 = 20;
/// `HEARTBEAT`, which the link sends on its own timer.
const HEARTBEAT: u32 = 0;

/// What one message type cost in the measured pass.
#[derive(Debug, Default, Clone, Copy)]
struct Cost {
    frames: u64,
    allocations: u64,
    max_per_frame: u64,
}

/// What the transport saw, shared with the test thread.
#[derive(Debug, Default)]
struct Probe {
    /// Per message id, allocations on the link thread while handling that frame, excluding the
    /// iterations below.
    per_msgid: BTreeMap<u32, Cost>,
    /// Iterations in which the link sent `PARAM_REQUEST_READ`: parameter gap recovery ran. It runs
    /// on a timer, so its cost lands on whichever frame happened to be in hand. Per message id of
    /// that frame: (iterations, allocations).
    gap_recovery: BTreeMap<u32, (u64, u64)>,
    /// Message ids the link wrote during the measured pass.
    written: BTreeMap<u32, u64>,
    /// Publishes in the measured pass, counted as the link's calls to `Transport::description`,
    /// which it makes once per publish; and what those calls allocated.
    publishes: u64,
    description_allocations: u64,
    measured_frames: u64,
    /// Wall time the measured pass took, which bounds how often a timer can have fired in it.
    measured_elapsed: Duration,
}

/// One recorded frame.
struct Recorded {
    msgid: u32,
    bytes: Vec<u8>,
}

/// Serves a recording one frame per `read`, and measures the link thread between reads.
struct FramePerRead {
    frames: Vec<Recorded>,
    passes: usize,
    pass: usize,
    next: usize,
    /// The production `file:` transport over the same recording, used only for `description`, so
    /// the per-publish cost measured is the one a replayed log really pays.
    replay: ReplayTransport,
    probe: Arc<Mutex<Probe>>,
    /// The frame whose handling is being measured, if it belongs to the measured pass.
    in_flight: Option<u32>,
    /// `ALLOCATIONS` when counting last resumed.
    mark: u64,
    /// What `description` allocated since the last read. `description` takes `&self`.
    description_cost: Cell<u64>,
    description_calls: Cell<u64>,
    /// Makes `description` allocate as it did when the trait returned a `String`, to prove this
    /// test notices.
    allocating_description: bool,
    wrote_param_request: bool,
    /// When the first frame of the measured pass was served.
    measured_from: Option<Instant>,
    open: bool,
}

impl FramePerRead {
    /// Closes the books on the iteration that just ended. Called with counting off.
    fn settle(&mut self, spent: u64) {
        let description = self.description_cost.replace(0);
        let publishes = self.description_calls.replace(0);
        let wrote_param_request = std::mem::take(&mut self.wrote_param_request);
        let Some(msgid) = self.in_flight.take() else {
            return;
        };
        let mut probe = self.probe.os_lock().unwrap();
        probe.measured_frames += 1;
        probe.publishes += publishes;
        probe.description_allocations += description;
        // The description's cost stays in: asking for it is part of publishing, and publishing is
        // part of every packet here.
        let packet = spent;
        if wrote_param_request {
            let entry = probe.gap_recovery.entry(msgid).or_default();
            entry.0 += 1;
            entry.1 += packet;
            return;
        }
        let cost = probe.per_msgid.entry(msgid).or_default();
        cost.frames += 1;
        cost.allocations += packet;
        cost.max_per_frame = cost.max_per_frame.max(packet);
    }
}

impl Transport for FramePerRead {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        WATCHING.with(|w| w.set(false));
        let spent = allocations_so_far() - self.mark;
        self.settle(spent);

        if self.next == self.frames.len() {
            self.next = 0;
            self.pass += 1;
        }
        if self.pass == self.passes {
            if let Some(from) = self.measured_from.take() {
                self.probe.os_lock().unwrap().measured_elapsed = from.elapsed();
            }
            // Counting stays off: the link's shutdown is not a packet.
            self.open = false;
            return Ok(0);
        }
        let frame = &self.frames[self.next];
        self.next += 1;
        buf[..frame.bytes.len()].copy_from_slice(&frame.bytes);
        if self.pass + 1 == self.passes {
            self.in_flight = Some(frame.msgid);
            self.measured_from.get_or_insert_with(Instant::now);
        }

        self.mark = allocations_so_far();
        WATCHING.with(|w| w.set(true));
        Ok(frame.bytes.len())
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        // Bookkeeping runs uncounted, so it is not charged to the link.
        let watching = WATCHING.with(|w| w.replace(false));
        // Every frame the link sends is v2 and built by the link itself.
        let msgid = u32::from_le_bytes([buf[7], buf[8], buf[9], 0]);
        if msgid == PARAM_REQUEST_READ {
            self.wrote_param_request = true;
        }
        if self.in_flight.is_some() {
            *self
                .probe
                .os_lock()
                .unwrap()
                .written
                .entry(msgid)
                .or_default() += 1;
        }
        WATCHING.with(|w| w.set(watching));
        Ok(())
    }

    fn description(&self) -> &str {
        // Counted, deliberately: this is the production transport's cost, paid on the link
        // thread each time the link calls it.
        let before = allocations_so_far();
        if self.allocating_description {
            // What asking cost when the trait returned an owned `String`.
            black_box(self.replay.description().to_owned());
        }
        let text = self.replay.description();
        self.description_cost
            .set(self.description_cost.get() + allocations_so_far() - before);
        self.description_calls.set(self.description_calls.get() + 1);
        text
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }
}

/// Every `.tlog` in `testdata/mavlink`, by name, in a stable order.
fn fixture_tlogs() -> Vec<(String, Vec<u8>)> {
    let dir = Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink"
    ));
    let mut out: Vec<(String, Vec<u8>)> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("reading {}: {e}", dir.display()))
        .map(|entry| entry.expect("directory entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "tlog"))
        .map(|path| {
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {name}: {e}"));
            (name, bytes)
        })
        .collect();
    out.sort();
    assert!(
        !out.is_empty(),
        "no .tlog fixtures in {}: an empty corpus would prove nothing",
        dir.display()
    );
    out
}

/// The frames in a recording, as the link's own decoder finds them.
fn frames_of(bytes: &[u8]) -> Vec<Recorded> {
    let mut decoder = FrameDecoder::new();
    let mut out = Vec::new();
    let mut keep = |frame: &mp_mavlink::Frame<'_>| {
        out.push(Recorded {
            msgid: frame.msgid,
            bytes: frame.raw.to_vec(),
        });
    };
    decoder.push_and_drain(bytes, &DIALECT, &mut keep);
    decoder.flush(&DIALECT, &mut keep);
    out
}

/// A recording path nobody else is using. `TlogWriter` refuses to overwrite.
fn recording_path(name: &str) -> PathBuf {
    let path = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("no_alloc_ingest-{}-{name}", mp_os::process_id()));
    let _ = std::fs::remove_file(&path);
    path
}

/// Runs one recording through a real link, twice, and returns what the second pass cost.
fn run(name: &str, bytes: Vec<u8>, allocating_description: bool) -> (Probe, usize) {
    let frames = frames_of(&bytes);
    let count = frames.len();
    let probe = Arc::new(Mutex::new(Probe::default()));
    let transport = FramePerRead {
        frames,
        passes: 2,
        pass: 0,
        next: 0,
        replay: ReplayTransport::from_bytes(name, bytes),
        probe: Arc::clone(&probe),
        in_flight: None,
        mark: 0,
        description_cost: Cell::new(0),
        description_calls: Cell::new(0),
        allocating_description,
        wrote_param_request: false,
        measured_from: None,
        open: true,
    };

    let recording = recording_path(name);
    let config = LinkConfig {
        record_path: Some(recording.clone()),
        publish_interval: Duration::ZERO,
        heartbeat_interval: Duration::ZERO,
        ..LinkConfig::default()
    };
    let mut link = Link::from_transport(Box::new(transport), config);
    let deadline = Instant::now() + Duration::from_secs(300);
    while link.is_running() {
        assert!(Instant::now() < deadline, "{name}: the link never finished");
        wasm_thread::sleep(Duration::from_millis(10));
    }

    // Both passes were received, recorded and published, or the numbers below describe a link
    // that did not do its job.
    assert_eq!(
        link.frames_received(),
        2 * count as u64,
        "{name}: frames served twice"
    );
    let stats = link.stats();
    assert!(stats.publishes >= count as u64, "{name}: {stats:?}");
    let recorded = std::fs::metadata(&recording).map(|m| m.len()).unwrap_or(0);
    assert!(
        recorded > 0,
        "{name}: the link recorded nothing to {}",
        recording.display()
    );
    let primary = VehicleId::new(1, 1);
    let snapshot = link
        .vehicle(primary)
        .unwrap_or_else(|| panic!("{name}: no handle for {primary}"))
        .load();
    assert!(
        snapshot.messages_applied > 0,
        "{name}: a reader saw no state"
    );
    link.close();
    let _ = std::fs::remove_file(&recording);

    let probe = std::mem::take(&mut *probe.os_lock().unwrap());
    (probe, count)
}

#[test]
fn the_real_link_thread_allocates_nothing_per_telemetry_packet() {
    assert!(
        EXCLUDED.len() <= 3,
        "the exclusion list has grown to {}; each entry needs a reason",
        EXCLUDED.len()
    );

    let mut frames_checked = 0u64;
    // Across every recording: the most one frame of each excluded type allocated.
    let mut excluded_seen: BTreeMap<u32, (u64, u64)> = BTreeMap::new();

    for (name, bytes) in fixture_tlogs() {
        let (probe, count) = run(&name, bytes, false);
        assert_eq!(
            probe.measured_frames, count as u64,
            "{name}: every served frame of the measured pass must be accounted for"
        );
        assert!(
            count >= 10_000,
            "{name}: only {count} frames; is the fixture truncated?"
        );

        // Every packet paid for a publish and a heartbeat, and each publish asked the transport
        // how it describes itself (`run_link`, mp-link/src/lib.rs), to notice a UDP link learning
        // its peer. Asking is free. Checked first and by name, so a regression says where it is;
        // the rule below would catch it anyway, on every message type.
        assert!(
            probe.publishes >= count as u64,
            "{name}: the link must publish, and ask for the description, every iteration in this \
             test"
        );
        assert!(
            probe.written.get(&HEARTBEAT).copied().unwrap_or(0) >= count as u64,
            "{name}: the link must send a heartbeat every iteration in this test: {:?}",
            probe.written
        );
        assert_eq!(
            probe.description_allocations, 0,
            "{name}: Transport::description allocated {} times over {} publishes",
            probe.description_allocations, probe.publishes
        );

        // The rule: zero, for everything not excluded by name.
        let offenders: Vec<(u32, Cost)> = probe
            .per_msgid
            .iter()
            .filter(|(msgid, cost)| {
                cost.allocations > 0 && !EXCLUDED.iter().any(|e| e.msgid == **msgid)
            })
            .map(|(msgid, cost)| (*msgid, *cost))
            .collect();
        assert!(
            offenders.is_empty(),
            "{name}: message types that allocate per packet on the link thread: {offenders:?}"
        );

        for excluded in EXCLUDED {
            let Some(cost) = probe.per_msgid.get(&excluded.msgid) else {
                continue;
            };
            assert!(
                cost.max_per_frame <= excluded.max_per_frame,
                "{name}: {} allocated {} in one frame, more than the {} allowed ({})",
                excluded.name,
                cost.max_per_frame,
                excluded.max_per_frame,
                excluded.why
            );
            let seen = excluded_seen.entry(excluded.msgid).or_default();
            seen.0 += cost.frames;
            seen.1 = seen.1.max(cost.max_per_frame);
        }

        // Parameter gap recovery (`ParamDownload` in mp-link/src/param_download.rs) runs only
        // inside a download the caller started with `Link::download_params`, as the C#'s does
        // inside `getParamListAsync`. This test starts none, so the recorded PARAM_VALUEs - each
        // carrying the vehicle's full count - must not make the link ask for anything. Before
        // the download became a machine of its own they did: the old timer chased every table
        // with holes, ten requests every 1.5 s.
        for msgid in probe.written.keys() {
            assert!(
                *msgid == HEARTBEAT,
                "{name}: the link sent msgid {msgid} in steady state; only heartbeats are \
                 expected: {:?}",
                probe.written
            );
        }
        let recoveries: u64 = probe.gap_recovery.values().map(|(n, _)| n).sum();
        assert_eq!(
            recoveries, 0,
            "{name}: parameters were asked for in {:?} with no download running",
            probe.measured_elapsed
        );

        let clean: u64 = probe
            .per_msgid
            .iter()
            .filter(|(_, cost)| cost.allocations == 0)
            .map(|(_, cost)| cost.frames)
            .sum();
        eprintln!(
            "{name}: {count} frames measured, {clean} with zero allocations; excluded {:?}; \
             gap recovery {:?}; sent {:?}; {} publishes cost {} allocations in \
             Transport::description",
            EXCLUDED
                .iter()
                .filter_map(|e| probe.per_msgid.get(&e.msgid).map(|c| (e.name, *c)))
                .collect::<Vec<_>>(),
            probe.gap_recovery,
            probe.written,
            probe.publishes,
            probe.description_allocations,
        );
        frames_checked += count as u64;
    }

    // Each exclusion must still be earning its place: seen in the corpus, and still allocating.
    // When one stops allocating, delete it here rather than leaving a tolerance nothing needs.
    for excluded in EXCLUDED {
        let (frames, max) = excluded_seen
            .get(&excluded.msgid)
            .copied()
            .unwrap_or_default();
        assert!(
            frames > 0,
            "{}: excluded but never seen in the corpus",
            excluded.name
        );
        assert!(
            max > 0,
            "{}: no longer allocates; remove it from EXCLUDED",
            excluded.name
        );
    }

    assert!(
        frames_checked >= 20_000,
        "checked only {frames_checked} frames"
    );
}

/// The rule above is only as good as its measurement, so prove it catches a description that
/// allocates: the same link over the same recording, with `description` made to allocate once per
/// ask. When the trait returned a `String` it cost two per publish; one is enough to be caught.
#[test]
fn an_allocating_description_is_caught() {
    let (name, bytes) = fixture_tlogs().into_iter().next().unwrap();
    let name = format!("allocating-description-{name}");
    let (probe, count) = run(&name, bytes, true);

    assert!(
        probe.publishes >= count as u64,
        "{name}: {}",
        probe.publishes
    );
    assert!(
        probe.description_allocations >= probe.publishes,
        "{name}: {} publishes, but only {} allocations counted in Transport::description",
        probe.publishes,
        probe.description_allocations
    );
    // Charged to the packet it was published after, so the per-packet rule fails for every
    // message type, not only the three excluded ones.
    let unnoticed: Vec<(u32, Cost)> = probe
        .per_msgid
        .iter()
        .filter(|(_, cost)| cost.allocations < cost.frames)
        .map(|(msgid, cost)| (*msgid, *cost))
        .collect();
    assert!(
        unnoticed.is_empty(),
        "{name}: message types whose packets were not charged for the description: {unnoticed:?}"
    );
}
