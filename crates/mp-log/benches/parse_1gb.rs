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

//! Deliverable 14's parse budget: a 1 GB dataflash log opened to its first plot in under two seconds.
//!
//! `DELIVERABLES.md` Deliverable 14 asks for "a 1 GB dataflash log in < 2 s". What the log browser does to
//! open a log and plot its first field (`LogBrowse::open`, then `LogBrowse::graph`) is, on
//! [`LogFile`]: read and index the file; the field inventory, the units, the chart's labels, the
//! cursor's position records, the start of the time axis and the map's routes; and one field's
//! samples, `ATT.Roll`. That whole sequence is `open_to_first_plot`, and it is what the gate
//! holds to two seconds.
//!
//! **The log** is `testdata/dataflash.bin` - a healthy 525 kB copter log - repeated 2,045 times,
//! 1,074,205,780 bytes and 23,392,755 records, written once to `<target>/mp-log-bench/` and
//! reused. A log of many flights joined end to end declares its formats again with each flight;
//! one long flight declares them once; both are one walk's work, and the pieces the walk is cut
//! into for its threads are held to the one walk's answer by `index.rs`'s tests either way.
//!
//! The parts, each a criterion bench on the same log:
//! - `open` - [`LogFile::open`]: the file read by eight threads, each walking what it has just
//!   read, and the pieces checked against one walk and joined into the index;
//! - `index_in_memory` - [`RecordIndex::build`] of the bytes already read: the walk alone;
//! - `plottable`, `units`, `overlays`, `positions`, `time_origin`, `routes` - the products
//!   `LogBrowse::open` makes, each read through the index;
//! - `first_extract` - `ATT.Roll`'s 372,190 samples, `LogBrowse::graph`;
//! - `open_to_first_plot` - all of it, in that order, from the file.
//!
//! After criterion, the bench runs `open_to_first_plot` seven times and fails if the median is
//! over two seconds. Skipped in unoptimised builds (`cargo test --benches`), which instead run
//! it once on the 525 kB fixture to show it runs. `PARSE_1GB_BEFORE=1` also times the slice
//! functions `LogBrowse::open` called before [`LogFile`] - five minutes of work.
//!
//! Measured on the development machine, an i7-10875H (8 cores, 16 threads, 32 GB, swap full),
//! release profile, page cache warm, with other agents' builds running beside it throughout: a
//! load average of 15 to 20 on its 16 threads for the "after" column below, falling to 9 by the
//! gate, and of about 20 for the "before" column, which got 64% of one core. Timings on this
//! machine move by a factor of two or more with that load; the ratios do not. "Before" is one
//! run, wall clock; "after" is criterion's median. Peak memory is `/usr/bin/time -v`'s maximum
//! resident set for the whole sequence:
//!
//! | part                          | before (slice functions) | after ([`LogFile`])           |
//! |-------------------------------|--------------------------|-------------------------------|
//! | read the file                 | 1.31 s                   | } `open` 686 ms, eight        |
//! | index (`RecordIndex::build`)  | 8.96 s                   | } threads (the walk: 273 ms)  |
//! | plottable                     | 202.1 s                  | 81.8 ms                       |
//! | units                         | 38.0 s                   | 21.5 ms                       |
//! | overlays                      | 11.2 s                   | 54.0 ms                       |
//! | positions                     | 9.10 s                   | 59.8 ms                       |
//! | time_origin                   | 2.5 ms                   | 0.014 ms                      |
//! | routes                        | 46.2 s                   | 72.4 ms                       |
//! | read the file again (`graph`) | 3.77 s                   | -                             |
//! | extract `ATT.Roll`            | 6.04 s                   | 30.6 ms                       |
//! | **open to first plot**        | **326.8 s**              | **975 ms**                    |
//! | peak memory                   | 1.29 GB                  | 1.62 GB                       |
//!
//! Gate: median 611 ms, fastest 562 ms, slowest 623 ms, against 2 s. Run by hand three times at a
//! load of 16 to 20 the whole sequence took 1.21 s to 1.22 s, and at a load of 6, 0.63 s to 1.06
//! s. What is left is mostly the reading: a gigabyte copied out of the page cache into memory
//! the kernel must first zero, which no amount of parsing speed removes; `std::fs::read` of the
//! file alone takes 0.5 s here.
//!
//! The before column is `LogBrowse::open` and `graph` as they stood: `std::fs::read`, then each
//! slice function walking every record of the gigabyte and decoding most of them, `plottable` a
//! `BTreeMap` entry and two `String`s per field of every record; `graph` read the file again. The
//! extra memory after is the index, thirteen bytes a record, kept while the log is open, and the
//! pieces' tables while they are joined.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(missing_docs)] // criterion_group! expands to an undocumented pub fn

use std::hint::black_box;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use criterion::{BatchSize, Criterion, criterion_group};
use mp_log::index::RecordIndex;
use mp_log::logfile::LogFile;
use mp_log::overlay::{Firmware, Overlays, Positions};
use mp_log::plot::{PlottableField, Point, UnitTable};
use mp_log::track::Routes;

/// Deliverable 14's budget.
const BUDGET: Duration = Duration::from_secs(2);

/// How big the log is made: a gibibyte, in whole copies of the fixture.
const SIZE: usize = 1 << 30;

/// The log, made once.
static LOG: OnceLock<PathBuf> = OnceLock::new();

fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin")
}

/// Where the build's output goes: `CARGO_TARGET_DIR` if it is set, else the workspace's
/// `target`. Never the repository.
fn target_dir() -> PathBuf {
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    match std::env::var_os("CARGO_TARGET_DIR").map(PathBuf::from) {
        Some(dir) if dir.is_absolute() => dir,
        Some(dir) => workspace.join(dir),
        None => workspace.join("target"),
    }
}

/// The gigabyte log, written if it is not already there at its full size.
fn big_log() -> &'static Path {
    LOG.get_or_init(|| {
        let one = std::fs::read(fixture()).expect("the fixture");
        let copies = SIZE.div_ceil(one.len());
        let want = (copies * one.len()) as u64;
        let dir = target_dir().join("mp-log-bench");
        let path = dir.join("parse_1gb.bin");
        if std::fs::metadata(&path).is_ok_and(|metadata| metadata.len() == want) {
            return path;
        }
        std::fs::create_dir_all(&dir).expect("the bench's directory");
        let partial = dir.join("parse_1gb.bin.partial");
        let mut out = std::io::BufWriter::new(std::fs::File::create(&partial).expect("create"));
        for _ in 0..copies {
            out.write_all(&one).expect("write");
        }
        out.flush().expect("flush");
        drop(out);
        std::fs::rename(&partial, &path).expect("rename");
        println!("parse_1gb: wrote {} ({want} bytes)", path.display());
        path
    })
}

/// What the log browser names modes with (`logbrowse.rs`'s `flight_mode_name`).
fn mode_name(firmware: Firmware, mode: u64) -> Option<String> {
    mp_log::convert::flight_mode_name(firmware, u8::try_from(mode).ok()?)
}

/// Everything the log browser has once a log is open and its first field plotted.
#[allow(dead_code)] // held to be dropped outside the timing
struct Opened {
    log: LogFile,
    fields: Vec<PlottableField>,
    units: UnitTable,
    overlays: Overlays,
    positions: Positions,
    origin: Option<f64>,
    routes: Routes,
    points: Vec<Point>,
}

/// `LogBrowse::open`, then `LogBrowse::graph` of the first field it would plot, on
/// [`LogFile`]: what Deliverable 14's two seconds are for.
fn open_to_first_plot(path: &Path) -> Opened {
    let log = LogFile::open(path).expect("the log");
    let fields = log.plottable();
    let units = log.units();
    let overlays = log.overlays(mode_name);
    let positions = log.positions();
    let origin = log.time_origin();
    let routes = log.routes();
    let points = log.extract_instance("ATT", None, "Roll");
    Opened {
        log,
        fields,
        units,
        overlays,
        positions,
        origin,
        routes,
        points,
    }
}

fn parse_1gb(c: &mut Criterion) {
    let path = big_log();
    let mut group = c.benchmark_group("parse_1gb");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(8));
    // Each iteration's gigabyte and a half is dropped, outside the timing, before the next is
    // made: a batch of them held together would be more memory than a laptop has to spare.
    group.bench_function("open", |b| {
        b.iter_batched(
            || black_box(path),
            |path| LogFile::open(path).expect("the log"),
            BatchSize::PerIteration,
        );
    });
    let bytes = std::fs::read(path).expect("the log");
    group.bench_function("index_in_memory", |b| {
        b.iter_batched(
            || black_box(bytes.as_slice()),
            RecordIndex::build,
            BatchSize::PerIteration,
        );
    });
    drop(bytes);
    let log = LogFile::open(path).expect("the log");
    group.bench_function("plottable", |b| b.iter(|| log.plottable()));
    group.bench_function("units", |b| b.iter(|| log.units()));
    group.bench_function("overlays", |b| b.iter(|| log.overlays(mode_name)));
    group.bench_function("positions", |b| b.iter(|| log.positions()));
    group.bench_function("time_origin", |b| b.iter(|| log.time_origin()));
    group.bench_function("routes", |b| b.iter(|| log.routes()));
    group.bench_function("first_extract", |b| {
        b.iter(|| log.extract_instance("ATT", None, "Roll"));
    });
    drop(log);
    group.measurement_time(Duration::from_secs(15));
    group.bench_function("open_to_first_plot", |b| {
        b.iter_batched(
            || black_box(path),
            open_to_first_plot,
            BatchSize::PerIteration,
        );
    });
    group.finish();
}

/// `LogBrowse::open` and `graph` as they were before [`LogFile`]: the file read, each slice
/// function walking all of it, and the file read again for the plot. Run only when asked.
fn before(path: &Path) {
    let mut parts: Vec<(&str, Duration)> = Vec::new();
    let mut time = |name, started: Instant| parts.push((name, started.elapsed()));
    let whole = Instant::now();
    let started = Instant::now();
    let data = std::fs::read(path).expect("the log");
    time("read", started);
    let started = Instant::now();
    black_box(mp_log::plot::plottable(&data));
    time("plottable", started);
    let started = Instant::now();
    black_box(mp_log::plot::units(&data));
    time("units", started);
    let started = Instant::now();
    black_box(mp_log::overlay::overlays(&data, mode_name));
    time("overlays", started);
    let started = Instant::now();
    black_box(Positions::read(&data));
    time("positions", started);
    let started = Instant::now();
    black_box(mp_log::plot::time_origin(&data));
    time("time_origin", started);
    let started = Instant::now();
    black_box(mp_log::track::routes(&data));
    time("routes", started);
    let started = Instant::now();
    black_box(RecordIndex::build(&data));
    time("index", started);
    drop(data);
    let started = Instant::now();
    let data = std::fs::read(path).expect("the log");
    time("read again", started);
    let started = Instant::now();
    black_box(mp_log::plot::extract_instance(&data, "ATT", None, "Roll"));
    time("extract ATT.Roll", started);
    time("open to first plot", whole);
    for (name, took) in parts {
        println!("parse_1gb before: {name:<20} {took:?}");
    }
}

/// Opens the log to its first plot seven times and fails if the median is over two seconds.
fn gate(path: &Path) {
    black_box(open_to_first_plot(path));
    let mut times: Vec<Duration> = (0..7)
        .map(|_| {
            let started = Instant::now();
            let opened = open_to_first_plot(black_box(path));
            let took = started.elapsed();
            drop(opened);
            took
        })
        .collect();
    times.sort_unstable();
    let median = times[times.len() / 2];
    println!(
        "parse_1gb gate: open to first plot median {median:?}, fastest {:?}, slowest {:?}, \
         budget {BUDGET:?}",
        times[0],
        times[times.len() - 1]
    );
    assert!(
        median <= BUDGET,
        "a 1 GB log takes {median:?} to open to its first plot, over Deliverable 14's {BUDGET:?}"
    );
}

criterion_group!(benches, parse_1gb);

fn main() {
    if cfg!(debug_assertions) {
        // Unoptimised, a gigabyte means nothing and takes minutes; show that it runs.
        let opened = open_to_first_plot(&fixture());
        assert_eq!(opened.points.len(), 182, "ATT.Roll of the fixture");
        println!("parse_1gb: skipped in an unoptimised build; the fixture opens to its first plot");
        return;
    }
    if std::env::var_os("PARSE_1GB_BEFORE").is_some() {
        before(big_log());
    }
    benches();
    Criterion::default().configure_from_args().final_summary();
    gate(big_log());
}
