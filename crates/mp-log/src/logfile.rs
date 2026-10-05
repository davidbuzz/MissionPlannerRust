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

//! A dataflash log opened once and read by type: what `DFLogBuffer` is to the log browser.
//!
//! The functions in [`crate::plot`], [`crate::overlay`] and [`crate::track`] each take the log's
//! bytes and walk every record in it, decoding every record of the types they want, and the log
//! browser calls seven of them to open one log - fine for a flight of a few minutes, and minutes
//! of work for a gigabyte (`DELIVERABLES.md` Deliverable 14 budgets two seconds). `DFLogBuffer` does it the
//! other way round: `setlinecount` walks the file **once**, recording where each record starts
//! and, per message type, the lines of its records (`messageindex`, `messageindexline`), and
//! everything afterwards - `GetEnumeratorType(types)` - reads only the records of the types asked
//! for. [`LogFile`] is that: one walk into a [`RecordIndex`], and every product read through it.
//!
//! **Columnar where it counts.** Reading one field of ten million records does not need ten
//! million decoded records: the field's place in its format is found once, and those bytes of each
//! record are read (`crate::dataflash::Column`). [`LogFile::plottable`], [`LogFile::time_origin`],
//! [`LogFile::extract_instance`], [`LogFile::positions`], [`LogFile::routes`] and the `GPS`
//! records of [`LogFile::overlays`] read that way; the rest decode the few records they need
//! whole, as the C# does. The file itself is read and walked by several threads at once
//! ([`RecordIndex::read_file`]), each checked against what one walk would have found.
//!
//! **The same answers.** Each method returns what the slice function of the same name returns
//! for the same bytes - `tests/logfile.rs` holds them to it over the fixtures - including where a
//! log declares one type twice, differently: a record is read with the declaration in force
//! where it was logged, as the walk reads it. A "line" is a record's place among every record the
//! walk yields, `FMT` records included: the grid's row, and `DFItem.lineno`.
//!
//! **Not memory-mapped.** Deliverable 14 says "memory-mapped", and this crate forbids `unsafe`, which a
//! mapping needs (`memmap2::Mmap::map` is `unsafe`: another process may change the file under
//! it). The file is read into memory whole, once, and the index is about thirteen bytes a record
//! beside it.
//! `// C#: ExtLibs/Utilities/DFLogBuffer.cs:43-126, 206-330, 686-760`

use std::collections::BTreeMap;
use std::path::Path;

use crate::dataflash::{Column, LogMessage, MessageFormat, PerFormat, Value, decode_as, label_of};
use crate::index::{RecordIndex, Row, Rows, Segment};
use crate::overlay::{
    FIRMWARE_LINES, Firmware, OVERLAY_TYPES, OverlayWalk, Overlays, PLACE_FIELDS, PositionRecord,
    Positions, family_of, firmware_named, minute_field, place_in,
};
use crate::plot::{
    PlottableField, Point, UnitTable, instance_labels, seconds_since, text_of, unit_id, unit_table,
};
use crate::track::{DRAWMAP_TYPES, Routes, route_record};

/// The field that places a record on the time axis.
const TIME_FIELD: &str = "TimeUS";

/// A dataflash log in memory, with its records indexed by line and by type.
pub struct LogFile {
    /// The log's bytes.
    data: Vec<u8>,
    /// Where every record is, and each type's lines.
    index: RecordIndex,
}

impl std::fmt::Debug for LogFile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LogFile")
            .field("bytes", &self.data.len())
            .field("records", &self.index.len())
            .finish_non_exhaustive()
    }
}

impl LogFile {
    /// Reads a log and indexes it: `new DFLogBuffer(filename)`.
    ///
    /// # Errors
    ///
    /// Whatever reading the file does. A file that is not a dataflash log is not an error: it is
    /// a log of no records.
    pub fn open(path: impl AsRef<Path>) -> std::io::Result<Self> {
        let (data, index) = RecordIndex::read_file(path.as_ref())?;
        Ok(Self { data, index })
    }

    /// Indexes a log already in memory.
    #[must_use]
    pub fn from_bytes(data: Vec<u8>) -> Self {
        let index = RecordIndex::build(&data);
        Self { data, index }
    }

    /// The log's bytes.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.data
    }

    /// The index: every record's offset and type, each type's lines, the formats.
    #[must_use]
    pub const fn index(&self) -> &RecordIndex {
        &self.index
    }

    /// Records in the log, `FMT` records included: the grid's rows.
    #[must_use]
    pub fn len(&self) -> usize {
        self.index.len()
    }

    /// Whether the log holds no records at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    /// One row, decoded: `DFLogBuffer[index]`, what the grid shows on that row.
    ///
    /// Decoded against the whole log's formats, as [`RecordIndex::decode`] and the C#'s indexer
    /// decode a row; see [`Self::messages`] for the one kind of log where that differs from the
    /// format the record was logged under.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:560-610`
    #[must_use]
    pub fn record(&self, row: usize) -> Option<LogMessage> {
        let offset = usize::try_from(self.index.offset(row)?).ok()?;
        self.index.decode(self.data.get(offset..)?)
    }

    /// Every record of the named types, with its line, in log order:
    /// `GetEnumeratorType(string[])`.
    ///
    /// A record is named, and decoded, by the format in force where it was logged - what
    /// [`crate::DataflashReader::next_message`] gives it. The C# names and decodes by the whole
    /// log's formats, and the two differ only for a log that declares one type twice,
    /// differently: the C# gives every record of that type to either name and decodes them all
    /// with the last declaration, which misreads the ones logged under the first. `FMT` records
    /// are included when `FMT` is asked for.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:701-760`
    #[must_use]
    pub fn messages(&self, names: &[&str]) -> Messages<'_> {
        Messages {
            data: &self.data,
            rows: self.index.rows(|name| names.contains(&name), false),
        }
    }

    /// Which field carries each instanced message's instance number, by message name: what
    /// `plot::instance_fields` reads from the `FMTU`s.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:230-250`
    #[must_use]
    pub fn instance_fields(&self) -> BTreeMap<String, String> {
        instance_labels(self.index.instance_positions(), self.index.formats())
    }

    /// The `TimeUS` of the first record that has one: [`crate::plot::time_origin`].
    ///
    /// The first record of each type that has a numeric `TimeUS`, and the earliest of those; one
    /// field of one record is read.
    #[must_use]
    pub fn time_origin(&self) -> Option<f64> {
        let mut first: Option<(u32, Column)> = None;
        for segment in self.index.data_segments() {
            let Some(format) = segment.format else {
                continue;
            };
            let Some(column) = Column::named(format, TIME_FIELD).filter(|column| column.numeric())
            else {
                continue;
            };
            let Some(&line) = segment.lines.first() else {
                continue;
            };
            if first.is_none_or(|(earliest, _)| line < earliest) {
                first = Some((line, column));
            }
        }
        let (line, column) = first?;
        column.read_f64(&self.data, self.index.start_of(line)?)
    }

    /// Everything that can be plotted against time: [`crate::plot::plottable`].
    ///
    /// A field of a timed type has a sample in every record of it, so the count is the type's
    /// record count - split by instance for an instanced type, which is the one field of each
    /// record that has to be read.
    #[must_use]
    pub fn plottable(&self) -> Vec<PlottableField> {
        let instances = self.instance_fields();
        let segments = self.index.data_segments();
        // A type with no `TimeUS` has nothing plottable, and nothing of it is read.
        fn timed<'a>(segment: &Segment<'a>) -> Option<&'a MessageFormat> {
            segment
                .format
                .filter(|format| Column::named(format, TIME_FIELD).is_some())
        }
        let selectors: Vec<Option<Column>> = segments
            .iter()
            .map(|segment| {
                let format = timed(segment)?;
                let label = instances.get(&format.name)?;
                Column::named(format, label).filter(|column| column.numeric())
            })
            .collect();
        let by_instance = self.instance_counts(&segments, &selectors);
        let mut counts: BTreeMap<(String, Option<i64>, String), usize> = BTreeMap::new();
        for (segment, by_instance) in segments.iter().zip(by_instance) {
            let Some(format) = timed(segment) else {
                continue;
            };
            let instance_label = instances.get(&format.name);
            for position in 0..format.format.len() {
                // `decode_fields` stops at the first field it cannot size; so do the counts.
                let Some(column) = Column::nth(format, position) else {
                    break;
                };
                let label = label_of(format, position);
                // The instance field is the selector, not a series, and `TimeUS` is the axis.
                if label == TIME_FIELD || Some(&label) == instance_label || !column.numeric() {
                    continue;
                }
                for (instance, samples) in &by_instance {
                    *counts
                        .entry((format.name.clone(), *instance, label.clone()))
                        .or_default() += samples;
                }
            }
        }
        counts
            .into_iter()
            .map(|((message, instance, field), samples)| PlottableField {
                message,
                instance,
                field,
                samples,
            })
            .collect()
    }

    /// How many records of each stretch carry each instance number, reading only the field that
    /// says: `(instance, records)` per stretch, in the order first met. A stretch with no such
    /// field is all one instance, `None`, and none of its records is read.
    ///
    /// The records are read in log order rather than a type at a time - one pass forward
    /// through the file, which is what memory reads fastest - and the pass is split between
    /// threads by line, their counts added at the end.
    fn instance_counts(
        &self,
        segments: &[Segment<'_>],
        selectors: &[Option<Column>],
    ) -> Vec<Counts> {
        let mut counts: Vec<Counts> = segments
            .iter()
            .zip(selectors)
            .map(|(segment, selector)| match selector {
                Some(_) => Vec::new(),
                None => vec![(None, segment.lines.len())],
            })
            .collect();
        if selectors.iter().all(Option::is_none) {
            return counts;
        }
        // For each type, where each of its stretches starts and what to read in it - every
        // stretch, so that a record is never taken for one of the stretch before it.
        let mut plans: Vec<Vec<(usize, usize, Option<Column>)>> = vec![Vec::new(); 256];
        for (position, (segment, selector)) in segments.iter().zip(selectors).enumerate() {
            if let (Some(first), Some(plan)) = (
                segment.lines.first(),
                plans.get_mut(usize::from(segment.msg_type)),
            ) {
                plan.push((*first as usize, position, *selector));
            }
        }
        // Lines past `u32::MAX` are in no stretch; see `RecordIndex::lines_of`.
        let lines = self
            .index
            .len()
            .min(usize::try_from(u64::from(u32::MAX) + 1).unwrap_or(usize::MAX));
        // One thread where this one may not wait for others - a web page's main thread, where
        // a scope may not even be made (mp_os::may_block) - and then no scope at all.
        let threads = if lines < 1 << 20 || !mp_os::may_block() {
            1
        } else {
            wasm_thread::available_parallelism().map_or(1, |threads| threads.get().min(8))
        };
        if threads == 1 {
            let found = self.count_instances(&plans, segments.len(), 0..lines);
            for (total, found) in counts.iter_mut().zip(found) {
                for (instance, records) in found {
                    tally(total, instance, records);
                }
            }
            return counts;
        }
        let per_thread = lines.div_ceil(threads);
        let parts: Vec<Vec<Counts>> = wasm_thread::scope(|scope| {
            let handles: Vec<_> = (0..threads)
                .map(|part| {
                    let plans = &plans;
                    let from = part * per_thread;
                    let to = (from + per_thread).min(lines);
                    scope.spawn(move || self.count_instances(plans, segments.len(), from..to))
                })
                .collect();
            handles
                .into_iter()
                .map(|handle| {
                    handle
                        .join()
                        .unwrap_or_else(|panic| std::panic::resume_unwind(panic))
                })
                .collect()
        });
        for part in parts {
            for (total, found) in counts.iter_mut().zip(part) {
                for (instance, records) in found {
                    tally(total, instance, records);
                }
            }
        }
        counts
    }

    /// [`Self::instance_counts`] over some lines.
    fn count_instances(
        &self,
        plans: &[Vec<(usize, usize, Option<Column>)>],
        stretches: usize,
        lines: std::ops::Range<usize>,
    ) -> Vec<Counts> {
        let mut counts: Vec<Counts> = vec![Vec::new(); stretches];
        // Each type's stretch in force: the last that starts at or before the line.
        let mut at: Vec<usize> = plans
            .iter()
            .map(|plan| {
                plan.partition_point(|(first, _, _)| *first <= lines.start)
                    .saturating_sub(1)
            })
            .collect();
        let types = self.index.types();
        let offsets = self.index.offsets();
        for line in lines {
            let Some(msg_type) = types.get(line) else {
                break;
            };
            let slot = usize::from(*msg_type);
            let (Some(plan), Some(at)) = (plans.get(slot), at.get_mut(slot)) else {
                continue;
            };
            if plan.is_empty() {
                continue;
            }
            while plan
                .get(*at + 1)
                .is_some_and(|(first, _, _)| *first <= line)
            {
                *at += 1;
            }
            let Some(&(first, stretch, Some(selector))) = plan.get(*at) else {
                continue;
            };
            if first > line {
                continue;
            }
            let Some(offset) = offsets
                .get(line)
                .and_then(|offset| usize::try_from(*offset).ok())
            else {
                continue;
            };
            #[allow(clippy::cast_possible_truncation)] // an instance number is a small integer
            let instance = selector
                .read_f64(&self.data, offset)
                .map(|value| value as i64);
            if let Some(found) = counts.get_mut(stretch) {
                tally(found, instance, 1);
            }
        }
        counts
    }

    /// One field's samples: [`crate::plot::extract`].
    #[must_use]
    pub fn extract(&self, message: &str, field: &str) -> Vec<Point> {
        self.extract_instance(message, None, field)
    }

    /// One instance's samples: [`crate::plot::extract_instance`].
    ///
    /// The records of the one type, and of each only its `TimeUS`, the field and the instance.
    #[must_use]
    pub fn extract_instance(
        &self,
        message: &str,
        instance: Option<i64>,
        field: &str,
    ) -> Vec<Point> {
        let instance_label = instance
            .is_some()
            .then(|| self.instance_fields().get(message).cloned())
            .flatten();
        let Some(origin) = self.time_origin() else {
            return Vec::new();
        };
        let mut points = Vec::new();
        let mut stretches = 0usize;
        for segment in self.index.data_segments() {
            if segment.name != message {
                continue;
            }
            let Some(format) = segment.format else {
                continue;
            };
            let (Some(time), Some(value)) = (
                Column::named(format, TIME_FIELD),
                Column::named(format, field),
            ) else {
                continue;
            };
            // A record whose format has no instance field has no instance, which is not the one
            // asked for.
            let selector = match instance_label.as_deref() {
                Some(label) => match Column::named(format, label) {
                    Some(column) => Some(column),
                    None => continue,
                },
                None => None,
            };
            stretches += 1;
            for line in segment.lines {
                let Some(offset) = self.index.start_of(*line) else {
                    continue;
                };
                if let Some(selector) = selector {
                    #[allow(clippy::cast_possible_truncation)] // instance numbers are small
                    let found = selector
                        .read_f64(&self.data, offset)
                        .map(|value| value as i64);
                    if found != instance {
                        continue;
                    }
                }
                let (Some(time), Some(value)) = (
                    time.read_f64(&self.data, offset),
                    value.read_f64(&self.data, offset),
                ) else {
                    continue;
                };
                if value.is_finite() {
                    points.push(Point {
                        seconds: seconds_since(origin, time),
                        line: *line as usize,
                        value,
                    });
                }
            }
        }
        if stretches > 1 {
            points.sort_by_key(|point| point.line);
        }
        points
    }

    /// Every field's unit: [`crate::plot::units`].
    ///
    /// The last `FMTU` of each format, `UNIT` of each id and `MULT` of each id is the one that
    /// counts, so each is read from the end of the log back, and a record whose format or id is
    /// already settled is passed over after reading its key.
    #[must_use]
    pub fn units(&self) -> UnitTable {
        let data = self.data.as_slice();
        let mut fmtu: BTreeMap<u8, (String, String)> = BTreeMap::new();
        let mut columns = PerFormat::default();
        for row in self.index.rows(|name| name == "FMTU", true).rev() {
            let Some(format) = row.format else {
                continue;
            };
            let found = columns.get(format, |format| {
                let fmt_type = Column::named(format, "FmtType")?;
                let unit_ids = Column::named(format, "UnitIds")?;
                let mult_ids = Column::named(format, "MultIds")?;
                (fmt_type.numeric() && unit_ids.text() && mult_ids.text())
                    .then_some((fmt_type, unit_ids, mult_ids))
            });
            let Some((fmt_type, unit_ids, mult_ids)) = found else {
                continue;
            };
            let Some(fmt_type) = fmt_type.read_f64(data, row.offset) else {
                continue;
            };
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            // a format type is a byte, cast as `units` casts it
            let key = fmt_type as u8;
            if fmtu.contains_key(&key) {
                continue;
            }
            let (Some(unit_ids), Some(mult_ids)) = (
                unit_ids.read(data, row.offset),
                mult_ids.read(data, row.offset),
            ) else {
                continue;
            };
            let (Some(unit_ids), Some(mult_ids)) = (unit_ids.as_text(), mult_ids.as_text()) else {
                continue;
            };
            fmtu.insert(
                key,
                (unit_ids.trim().to_owned(), mult_ids.trim().to_owned()),
            );
        }

        let mut unit_labels: BTreeMap<char, String> = BTreeMap::new();
        let mut columns = PerFormat::default();
        for row in self.index.rows(|name| name == "UNIT", true).rev() {
            let Some(format) = row.format else {
                continue;
            };
            let found = columns.get(format, |format| {
                Column::named(format, "Id").zip(Column::named(format, "Label"))
            });
            let Some((id, label)) = found else {
                continue;
            };
            let Some(id) = id.read_f64(data, row.offset).and_then(unit_id) else {
                continue;
            };
            if unit_labels.contains_key(&id) {
                continue;
            }
            if let Some(label) = label.read(data, row.offset).as_ref().and_then(text_of) {
                unit_labels.insert(id, label);
            }
        }

        let mut multipliers: BTreeMap<char, f64> = BTreeMap::new();
        let mut columns = PerFormat::default();
        for row in self.index.rows(|name| name == "MULT", true).rev() {
            let Some(format) = row.format else {
                continue;
            };
            let found = columns.get(format, |format| {
                Column::named(format, "Id").zip(Column::named(format, "Mult"))
            });
            let Some((id, mult)) = found else {
                continue;
            };
            let Some(id) = id.read_f64(data, row.offset).and_then(unit_id) else {
                continue;
            };
            if multipliers.contains_key(&id) {
                continue;
            }
            if let Some(mult) = mult.read_f64(data, row.offset) {
                multipliers.insert(id, mult);
            }
        }
        unit_table(&fmtu, &unit_labels, &multipliers, self.index.formats())
    }

    /// Everything the log browser draws over its chart: [`crate::overlay::overlays`].
    ///
    /// The records of the types it labels, in log order; a `GPS` record gives the minute labels
    /// two numbers, read without decoding it. The firmware guess - the last of the first hundred
    /// thousand and one `MSG` and `PARM` lines that names a vehicle - is found from that line
    /// back: in a log that names its vehicle in its first messages, a few lines' reading rather
    /// than a hundred thousand.
    #[must_use]
    pub fn overlays(&self, mode_name: impl Fn(Firmware, u64) -> Option<String>) -> Overlays {
        let data = self.data.as_slice();
        let mut walk = OverlayWalk::default();
        let mut gps = PerFormat::default();
        let labelled = |name: &str| name != "PARM" && OVERLAY_TYPES.contains(&name);
        for row in self.index.rows(labelled, false) {
            let Some(format) = row.format else {
                continue;
            };
            if row.name == "GPS" {
                let (time, minute) = gps.get(format, |format| {
                    (
                        Column::named(format, TIME_FIELD),
                        minute_field(format).and_then(|(position, micro)| {
                            Some((Column::nth(format, position)?, micro))
                        }),
                    )
                });
                let time_us = time.and_then(|time| time.read_f64(data, row.offset));
                let minute = minute
                    .and_then(|(column, micro)| Some((micro, column.read_f64(data, row.offset)?)));
                walk.gps(row.line, time_us, minute);
            } else if let Some(message) = data
                .get(row.offset..)
                .and_then(|bytes| decode_as(Some(format), bytes))
            {
                walk.record(row.line, format, &message);
            }
        }
        let lines: Vec<Row<'_>> = self
            .index
            .rows(|name| name == "MSG" || name == "PARM", false)
            .take(FIRMWARE_LINES + 1)
            .collect();
        walk.firmware(lines.iter().rev().find_map(|row| {
            let message = decode_as(row.format, data.get(row.offset..)?)?;
            firmware_named(&message)
        }));
        walk.finish(mode_name)
    }

    /// The records a double click on the chart can land on: [`crate::overlay::Positions::read`].
    ///
    /// Every record whose name starts `GPS` or `POS`, reading its `TimeUS` and the three fields
    /// `GetGPSFromRow` finds by their place in the first `GPS` or `POS` format declared by then.
    #[must_use]
    pub fn positions(&self) -> Positions {
        let data = self.data.as_slice();
        let mut gps_family = Timeline::new(self.index.first_named_over_time("GPS"));
        let mut pos_family = Timeline::new(self.index.first_named_over_time("POS"));
        let mut time = PerFormat::default();
        // The place fields' columns, for a record format and the family format it is read by.
        let mut places: Option<(&MessageFormat, &MessageFormat, [Option<Column>; 3])> = None;
        let mut records = Vec::new();
        for row in self.index.rows(
            |name| name.starts_with("GPS") || name.starts_with("POS"),
            false,
        ) {
            let Some(format) = row.format else {
                continue;
            };
            let family = family_of(row.name);
            let family_format = if family == "GPS" {
                gps_family.at(row.line)
            } else {
                pos_family.at(row.line)
            };
            let place = family_format.and_then(|family_format| {
                let columns = match places {
                    Some((seen_family, seen, columns))
                        if std::ptr::eq(seen_family, family_format)
                            && std::ptr::eq(seen, format) =>
                    {
                        columns
                    }
                    _ => {
                        let columns = PLACE_FIELDS.map(|name| {
                            let position = family_format
                                .labels
                                .iter()
                                .position(|label| label == name)?;
                            Column::nth(format, position)
                        });
                        places = Some((family_format, format, columns));
                        columns
                    }
                };
                place_in(family, |name| {
                    let slot = PLACE_FIELDS.iter().position(|field| *field == name)?;
                    columns
                        .get(slot)
                        .copied()
                        .flatten()?
                        .read_f64(data, row.offset)
                })
            });
            let time_us = time
                .get(format, |format| Column::named(format, TIME_FIELD))
                .and_then(|column| column.read_f64(data, row.offset));
            records.push(PositionRecord {
                line: row.line,
                name: row.name.to_owned(),
                time_us,
                place,
            });
        }
        let declared = self
            .index
            .formats()
            .values()
            .any(|format| format.name == "GPS" || format.name == "POS");
        Positions::from_parts(records, self.index.len(), declared)
    }

    /// Every route the log holds: [`crate::track::routes`], from the records of
    /// [`DRAWMAP_TYPES`] alone, and of each only the fields the map reads - [`ROUTE_FIELDS`]
    /// and the `GPS` instance field.
    ///
    /// Each record is handed to [`crate::track::route_record`] as the walk hands it one, but
    /// as one message per format whose values are overwritten record by record: a `GPS`
    /// record a second for an hour is a great many messages to make and throw away.
    #[must_use]
    pub fn routes(&self) -> Routes {
        let data = self.data.as_slice();
        let instances = self.instance_fields();
        let gps_instance = instances.get("GPS").map(String::as_str);
        let mut routes = Routes::default();
        // The format last read, its message, and where each of its fields comes from.
        let mut current: Option<(&MessageFormat, LogMessage, Vec<Column>)> = None;
        for row in self.index.rows(|name| DRAWMAP_TYPES.contains(&name), true) {
            let Some(format) = row.format else {
                continue;
            };
            if !current
                .as_ref()
                .is_some_and(|(seen, _, _)| std::ptr::eq(*seen, format))
            {
                current = Some(route_message(format, gps_instance));
            }
            let Some((_, message, columns)) = current.as_mut() else {
                continue;
            };
            let mut whole = true;
            for ((_, value), column) in message.fields.iter_mut().zip(columns.iter()) {
                match column.read(data, row.offset) {
                    Some(read) => *value = read,
                    None => whole = false,
                }
            }
            if whole {
                route_record(&mut routes, message, row.line, gps_instance);
            }
        }
        routes
    }
}

/// A message of the fields of `format` that [`LogFile::routes`] reads, in the format's order,
/// the first of each label as [`LogMessage::field`] would find it, with where each is read from.
fn route_message<'a>(
    format: &'a MessageFormat,
    gps_instance: Option<&str>,
) -> (&'a MessageFormat, LogMessage, Vec<Column>) {
    let mut positions: Vec<usize> = ROUTE_FIELDS
        .iter()
        .copied()
        .chain(gps_instance)
        .filter_map(|label| {
            (0..format.format.len()).find(|position| label_of(format, *position) == label)
        })
        .collect();
    positions.sort_unstable();
    positions.dedup();
    let (fields, columns) = positions
        .into_iter()
        .filter_map(|position| {
            let column = Column::nth(format, position)?;
            Some(((label_of(format, position), Value::Uint(0)), column))
        })
        .unzip();
    (
        format,
        LogMessage {
            name: format.name.clone(),
            fields,
        },
        columns,
    )
}

/// The fields [`crate::track::route_record`] reads of a record, by label: all a record gives the
/// map, which is `DrawMap`'s `getPointLatLng` and its `CMD` branch. [`LogFile::routes`] decodes
/// these and the `GPS` instance field and no other, so a field `route_record` comes to read
/// belongs here too; `tests/logfile.rs` holds the routes to the walk's, which decodes every
/// field, over logs that have every route.
/// `// C#: Log/LogBrowse.cs:2242-2433, 2541-2784`
pub(crate) const ROUTE_FIELDS: [&str; 14] = [
    "TimeUS", "Status", "Lat", "Lng", "GCrs", "CNum", "CTot", "CId", "Prm1", "Prm2", "Prm3",
    "Prm4", "Alt", "Frame",
];

/// How many records of a stretch carry each instance number, in the order first met.
type Counts = Vec<(Option<i64>, usize)>;

/// Adds records of an instance to a stretch's counts.
fn tally(counts: &mut Counts, instance: Option<i64>, records: usize) {
    match counts.iter_mut().find(|(seen, _)| *seen == instance) {
        Some((_, count)) => *count += records,
        None => counts.push((instance, records)),
    }
}

/// The records of some types, decoded, with their lines, in log order: [`LogFile::messages`].
#[derive(Debug, Clone)]
pub struct Messages<'a> {
    /// The log's bytes.
    data: &'a [u8],
    /// The records still to come.
    rows: Rows<'a>,
}

impl Iterator for Messages<'_> {
    type Item = (usize, LogMessage);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let row = self.rows.next()?;
            let Some(bytes) = self.data.get(row.offset..) else {
                continue;
            };
            if let Some(message) = decode_as(row.format, bytes) {
                return Some((row.line, message));
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.rows.size_hint()
    }
}

/// Which format a name meant at each point of the log, looked up at lines that only go forward.
struct Timeline<'a> {
    /// `(line, format)` wherever it changes, from line 0.
    changes: Vec<(usize, Option<&'a MessageFormat>)>,
    /// The change in force at the last line asked about.
    at: usize,
}

impl<'a> Timeline<'a> {
    /// A timeline from [`RecordIndex::first_named_over_time`].
    const fn new(changes: Vec<(usize, Option<&'a MessageFormat>)>) -> Self {
        Self { changes, at: 0 }
    }

    /// The format in force at a line no earlier than the last one asked about.
    fn at(&mut self, line: usize) -> Option<&'a MessageFormat> {
        while self
            .changes
            .get(self.at + 1)
            .is_some_and(|(from, _)| *from <= line)
        {
            self.at += 1;
        }
        self.changes.get(self.at).and_then(|(_, format)| *format)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{fixed, fmt, record};

    const FMTU: u8 = 129;
    const GPS: u8 = 140;
    const GPS2: u8 = 141;
    const GPSB: u8 = 142;
    const POS: u8 = 143;
    const CMD: u8 = 144;
    const CAM: u8 = 145;
    const TRIG: u8 = 146;
    const MODE: u8 = 147;
    const MSG: u8 = 148;
    const EV: u8 = 149;

    fn degrees(value: f64) -> [u8; 4] {
        #[allow(clippy::cast_possible_truncation)]
        let scaled = (value * 1e7).round() as i32;
        scaled.to_le_bytes()
    }

    /// A `GPS` record in the first layout: `TimeUS,I,Status,GMS,GWk,Lat,Lng,GCrs`.
    fn gps(time_us: u64, instance: u8, status: u8, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend([instance, status]);
        payload.extend(345_600_000u32.to_le_bytes());
        payload.extend(2_300u16.to_le_bytes());
        payload.extend(degrees(lat));
        payload.extend(degrees(lng));
        payload.extend(90.5f32.to_le_bytes());
        record(GPS, &payload)
    }

    /// A `GPS` record in the second layout, declared part way through the log:
    /// `TimeUS,I,Lng,Lat,Status` - every field but the instance somewhere else. The instance
    /// stays where `FMTU` marks it, as the whole log's last format is what names it.
    fn gps_again(time_us: u64, instance: u8, status: u8, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(instance);
        payload.extend(degrees(lng));
        payload.extend(degrees(lat));
        payload.push(status);
        record(GPS, &payload)
    }

    fn fix(msg_type: u8, time_us: u64, status: u8, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(status);
        payload.extend(degrees(lat));
        payload.extend(degrees(lng));
        payload.extend(12.0f32.to_le_bytes());
        record(msg_type, &payload)
    }

    fn place(msg_type: u8, time_us: u64, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend(degrees(lat));
        payload.extend(degrees(lng));
        record(msg_type, &payload)
    }

    fn cmd(total: u16, number: u16, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = 7u64.to_le_bytes().to_vec();
        payload.extend(total.to_le_bytes());
        payload.extend(number.to_le_bytes());
        payload.extend(16u16.to_le_bytes());
        for prm in [1.0f32, 2.0, 3.0, 4.0] {
            payload.extend(prm.to_le_bytes());
        }
        payload.extend(degrees(lat));
        payload.extend(degrees(lng));
        payload.extend(25.0f32.to_le_bytes());
        payload.push(3);
        record(CMD, &payload)
    }

    fn timed(msg_type: u8, time_us: u64, rest: &[u8]) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend_from_slice(rest);
        record(msg_type, &payload)
    }

    /// Every route `DrawMap` draws, every label `LogBrowse` writes over its chart, and a `GPS`
    /// whose layout changes part way through the log.
    fn everything() -> Vec<u8> {
        let mut log = fmt(FMTU, 44, "FMTU", "QBNN", "TimeUS,FmtType,UnitIds,MultIds");
        log.extend(fmt(
            GPS,
            31,
            "GPS",
            "QBBIHLLf",
            "TimeUS,I,Status,GMS,GWk,Lat,Lng,GCrs",
        ));
        log.extend(fmt(GPS2, 24, "GPS2", "QBLLf", "TimeUS,Status,Lat,Lng,GCrs"));
        log.extend(fmt(GPSB, 24, "GPSB", "QBLLf", "TimeUS,Status,Lat,Lng,GCrs"));
        log.extend(fmt(POS, 19, "POS", "QLL", "TimeUS,Lat,Lng"));
        log.extend(fmt(
            CMD,
            46,
            "CMD",
            "QHHHffffLLfB",
            "TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,Frame",
        ));
        log.extend(fmt(CAM, 19, "CAM", "QLL", "TimeUS,Lat,Lng"));
        log.extend(fmt(TRIG, 19, "TRIG", "QLL", "TimeUS,Lat,Lng"));
        log.extend(fmt(MODE, 13, "MODE", "QMB", "TimeUS,Mode,ModeNum"));
        log.extend(fmt(MSG, 75, "MSG", "QZ", "TimeUS,Message"));
        log.extend(fmt(EV, 12, "EV", "QB", "TimeUS,Id"));
        let mut fmtu = 0u64.to_le_bytes().to_vec();
        fmtu.push(GPS);
        fmtu.extend(fixed("s#-----", 16));
        fmtu.extend(fixed("F------", 16));
        log.extend(record(FMTU, &fmtu));
        log.extend(timed(MSG, 1_000, &fixed("ArduCopter V4.5.7", 64)));
        log.extend(timed(MODE, 2_000, &[5, 5]));
        for step in 0..200u32 {
            let time = 3_000_000 + u64::from(step) * 1_000_000;
            let offset = f64::from(step) * 1e-4;
            #[allow(clippy::cast_possible_truncation)]
            let instance = (step % 4) as u8;
            let status = if step % 5 == 0 { 1 } else { 3 };
            log.extend(gps(time, instance, status, -35.0 - offset, 149.0 + offset));
            if step % 7 == 0 {
                log.extend(fix(GPS2, time, 3, -35.5, 149.5 + offset));
                log.extend(fix(GPSB, time, 3, -35.6, 149.6 + offset));
                log.extend(place(POS, time, -35.7, 149.7 + offset));
            }
            if step % 50 == 0 {
                log.extend(timed(EV, time, &[10]));
                log.extend(place(CAM, time, -35.8, 149.8));
                log.extend(place(CAM, time, 0.0, 149.8));
                log.extend(place(TRIG, time, -35.9, 149.9));
                log.extend(timed(MODE, time, &[3, 3]));
            }
        }
        for (number, lat) in [(0u16, 0.0), (1, -35.1), (2, -35.2), (1, -35.1), (3, -35.3)] {
            let lng = if lat == 0.0 { 0.0 } else { 149.0 };
            log.extend(cmd(4, number, lat, lng));
        }
        // The `GPS` layout changes: the fields are somewhere else, and the record is shorter.
        log.extend(fmt(GPS, 21, "GPS", "QBLLB", "TimeUS,I,Lng,Lat,Status"));
        for step in 0..40u64 {
            #[allow(clippy::cast_possible_truncation)]
            let instance = (step % 3) as u8;
            log.extend(gps_again(
                250_000_000 + step * 2_000_000,
                instance,
                3,
                -34.0,
                150.0,
            ));
        }
        log.extend(timed(MSG, 400_000_000, &fixed("ArduPlane V4.5.7", 64)));
        log
    }

    fn mode_name(firmware: Firmware, mode: u64) -> Option<String> {
        crate::convert::flight_mode_name(firmware, u8::try_from(mode).ok()?)
    }

    /// Every product of a log that has everything is the walk's, where the columnar reads have
    /// to find each field in two layouts of one type.
    #[test]
    fn a_log_with_everything_reads_as_the_walk_reads_it() {
        let data = everything();
        let log = LogFile::from_bytes(data.clone());
        let routes = log.routes();
        assert_eq!(routes, crate::track::routes(&data));
        assert!(
            !routes.gps.is_empty()
                && !routes.gps2.is_empty()
                && !routes.gpsb.is_empty()
                && !routes.pos.is_empty()
                && !routes.cameras.is_empty()
                && !routes.repeats.is_empty(),
            "every route has something on it: {routes:?}"
        );
        assert_eq!(routes.commands.len(), 3, "home is at 0,0");
        assert_eq!(
            routes.commands.first().and_then(|first| first.frame),
            Some(3)
        );
        let overlays = log.overlays(mode_name);
        assert_eq!(overlays, crate::overlay::overlays(&data, mode_name));
        assert_eq!(
            overlays.firmware,
            Some(Firmware::Plane),
            "the last line decides"
        );
        assert!(overlays.minutes.len() > 3, "{:?}", overlays.minutes);
        assert!(!overlays.events.is_empty() && overlays.modes.len() > 2);
        assert_eq!(log.positions(), crate::overlay::Positions::read(&data));
        assert_eq!(log.plottable(), crate::plot::plottable(&data));
        assert_eq!(log.units(), crate::plot::units(&data));
        assert_eq!(log.time_origin(), crate::plot::time_origin(&data));
        assert_eq!(log.instance_fields(), crate::plot::instance_fields(&data));
        for instance in [None, Some(0), Some(1), Some(2), Some(3)] {
            for field in ["Lat", "Lng", "Status", "GCrs", "I"] {
                assert_eq!(
                    log.extract_instance("GPS", instance, field),
                    crate::plot::extract_instance(&data, "GPS", instance, field),
                    "GPS[{instance:?}].{field}"
                );
            }
        }
    }

    /// A type declared three times, the middle time with no `TimeUS` and no instance field:
    /// the middle stretch's records are no one's samples, and above all not the samples of the
    /// stretch before it, read with that stretch's layout.
    #[test]
    fn a_stretch_with_nothing_to_read_is_not_counted_as_the_one_before() {
        const TST: u8 = 150;
        let mut log = fmt(FMTU, 44, "FMTU", "QBNN", "TimeUS,FmtType,UnitIds,MultIds");
        let mut fmtu = 0u64.to_le_bytes().to_vec();
        fmtu.push(TST);
        fmtu.extend(fixed("s#-", 16));
        fmtu.extend(fixed("F--", 16));
        log.extend(record(FMTU, &fmtu));
        let tst = |time: u64, instance: u8, value: f32| {
            let mut payload = time.to_le_bytes().to_vec();
            payload.push(instance);
            payload.extend(value.to_le_bytes());
            record(TST, &payload)
        };
        log.extend(fmt(TST, 16, "TST", "QBf", "TimeUS,I,V"));
        for step in 0..5u8 {
            log.extend(tst(u64::from(step), step % 2, 1.0));
        }
        log.extend(fmt(TST, 16, "TST", "QBf", "TimeUX,J,V"));
        for step in 0..7u8 {
            log.extend(tst(u64::from(step), 7, 2.0));
        }
        log.extend(fmt(TST, 16, "TST", "QBf", "TimeUS,I,V"));
        for step in 0..3u8 {
            log.extend(tst(u64::from(step), 1, 3.0));
        }
        let file = LogFile::from_bytes(log.clone());
        let fields = file.plottable();
        assert_eq!(fields, crate::plot::plottable(&log));
        let samples: Vec<(Option<i64>, usize)> = fields
            .iter()
            .filter(|field| field.field == "V")
            .map(|field| (field.instance, field.samples))
            .collect();
        assert_eq!(samples, vec![(Some(0), 3), (Some(1), 5)]);
    }

    /// The instance fields are `plot::instance_fields`', on the fixtures as on the hand-made log.
    #[test]
    fn the_instance_fields_are_the_walks() {
        for name in [
            "dataflash.bin",
            "dataflash_damaged.bin",
            "dataflash/edge.bin",
        ] {
            let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata")
                .join(name);
            let data = mp_os::fs::read(&path).unwrap_or_default();
            assert!(!data.is_empty(), "{name}");
            let log = LogFile::from_bytes(data.clone());
            assert_eq!(
                log.instance_fields(),
                crate::plot::instance_fields(&data),
                "{name}"
            );
        }
    }

    /// A column read of a number is the decoded field's number, for every type that is one and
    /// every byte pattern tried - `NaN` included, bit for bit.
    #[test]
    fn a_column_reads_a_number_as_decoding_the_field_does() {
        let bytes: Vec<u8> = (0..=255u8)
            .chain((0..=255u8).rev())
            .cycle()
            .take(4096)
            .collect();
        for code in b"bBMhHiIqQfdgcCeELnNZa" {
            let format = MessageFormat {
                msg_type: 1,
                length: 0,
                name: "N".to_owned(),
                format: char::from(*code).to_string(),
                labels: vec!["V".to_owned()],
            };
            let column = Column::named(&format, "V");
            assert!(column.is_some(), "{}", char::from(*code));
            for at in 0..bytes.len() - 70 {
                let decoded = column
                    .and_then(|column| column.read(&bytes, at))
                    .and_then(|value| value.as_f64());
                let read = column.and_then(|column| column.read_f64(&bytes, at));
                assert_eq!(
                    decoded.map(f64::to_bits),
                    read.map(f64::to_bits),
                    "{} at {at}",
                    char::from(*code)
                );
            }
        }
    }
}
