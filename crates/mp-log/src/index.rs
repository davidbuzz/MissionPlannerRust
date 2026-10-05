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

//! Random access to a dataflash log's records, for a grid that shows them a screenful at a time.
//!
//! Mission Planner's log browser puts every record of a log in `dataGridView1`, and a real log
//! holds hundreds of thousands. It does not hold them: `DFLogBuffer` walks the file once for the
//! byte offset each record starts at, and the grid's `CellValueNeeded` decodes a row only when
//! the grid asks to paint it. This is that walk, and the few things the grid needs from the whole
//! log before it can show any one row - the formats, which field of a message is its instance, and
//! where GPS time puts the start of the log's clock.
//!
//! The same walk keeps, per message type, the lines its records are on: `messageindexline`, which
//! is what lets `GetEnumeratorType` read the records of the types it is asked for and seek past
//! every other one. [`crate::logfile::LogFile`] reads everything the log browser shows that way.
//!
//! **What it costs.** Thirteen bytes a record: an eight-byte offset, a one-byte type and a
//! four-byte line in its type's list. The C# keeps three eight-byte lists per record
//! (`linestartoffset`, `messageindex`, `messageindexline`), so this is about half of what the
//! original spends, and nothing here grows with the size of a record.
//! `// C#: ExtLibs/Utilities/DFLogBuffer.cs:28-40, 98-126`

use std::collections::BTreeMap;

use crate::dataflash::{
    Column, FMT_PAYLOAD_LEN, FMT_TYPE, LogMessage, MessageFormat, PerFormat, Step, Value, Walk,
    decode_record, parse_fmt,
};

/// Every record's place in a log, and what a row needs from the rest of it.
#[derive(Debug, Clone, Default)]
pub struct RecordIndex {
    /// Where each record starts, in log order. A row number is an index into this.
    offsets: Vec<u64>,
    /// Each record's message type, parallel to `offsets`, so a filter by type needs no reads.
    types: Vec<u8>,
    /// The lines of each type's records, in log order, by type byte: `messageindexline`.
    ///
    /// A line is held as a `u32`, four bytes rather than eight: a log with four billion records
    /// would be a file of tens of gigabytes. The records of one past that are left out of these
    /// lists rather than wrapped round, as [`Self::rows_named`] always has.
    lines: Vec<Vec<u32>>,
    /// Every `FMT` record that changed a declaration, in log order: the line it is on and the
    /// format it declared. A healthy log declares each type once; a log that is several logs
    /// joined declares them again, identically, and only a declaration that differs is kept.
    changes: Vec<Declaration>,
    /// The formats, as the whole log declares them.
    formats: BTreeMap<u8, MessageFormat>,
    /// Which field of a message type carries its instance number, from the `#` in its `FMTU`.
    instance_fields: BTreeMap<u8, usize>,
    /// The same, keyed as `plot::instance_fields` keys it before it looks the format up.
    instance_positions: BTreeMap<i64, usize>,
    /// How many records of each type the log holds: `SeenMessageTypes`, with counts.
    counts: BTreeMap<u8, usize>,
    /// Where GPS time puts the start of the log's clock, if a GPS ever had a fix.
    gps_start: Option<GpsStart>,
}

/// The records one stretch of a log holds, as a walk from some state finds them.
///
/// The whole log is one piece when it is walked from start to end. A log big enough to be worth
/// it is cut into a piece per thread, each walked at once from its first byte on the guess that
/// the log declares nothing its first few megabytes did not - so of a log of many flights of
/// one vehicle, and of one long flight once the types it declares the first time it logs one
/// have all been logged. [`RecordIndex::check`] keeps a piece only where the walk from the start
/// would have found the same records, and walks again where it would not.
struct Piece {
    /// Where each record starts, in the whole log's bytes.
    offsets: Vec<u64>,
    /// Each record's type.
    types: Vec<u8>,
    /// Each type's records, by their place in this piece.
    lines: Vec<Vec<u32>>,
    /// Records of each type.
    counts: [usize; 256],
    /// The walk, where it stopped.
    walk: Walk,
}

impl Piece {
    fn new(walk: Walk) -> Self {
        Self {
            offsets: Vec::new(),
            types: Vec::new(),
            lines: vec![Vec::new(); 256],
            counts: [0; 256],
            walk,
        }
    }

    /// Indexes every record the walk finds in `data` that starts before `limit`, adding `base`
    /// to each offset: `data` may be a piece of the log that starts `base` bytes in. See
    /// [`Walk::step`] for `complete`.
    fn walk(&mut self, data: &[u8], complete: bool, limit: usize, base: usize) -> Step {
        loop {
            match self.walk.step_until(data, complete, limit) {
                Step::Record(record) => {
                    let line = self.offsets.len();
                    // usize to u64 is lossless on every target this builds for; the offset is
                    // stored wide because a log is a file, and files outgrow a 32-bit address
                    // space.
                    self.offsets.push((record.offset + base) as u64);
                    self.types.push(record.msg_type);
                    let slot = usize::from(record.msg_type);
                    if let Some(count) = self.counts.get_mut(slot) {
                        *count += 1;
                    }
                    if let (Ok(line), Some(list)) = (u32::try_from(line), self.lines.get_mut(slot))
                    {
                        list.push(line);
                    }
                }
                other => return other,
            }
        }
    }
}

/// What [`RecordIndex::check`] made of the pieces.
///
/// The counts are what the tests hold the check to: that a log which declares nothing new part
/// way through is walked once, in parallel, and one that does costs a round and not a walk in
/// one thread.
#[cfg_attr(not(test), allow(dead_code))]
struct Checked {
    /// Each piece as kept, with how many of its first records to leave out.
    pieces: Vec<(Piece, usize)>,
    /// How many times the pieces left were walked again at once, knowing more formats.
    rounds: usize,
    /// How many pieces were walked again from the true walk's position, in one thread.
    alone: usize,
}

/// How the work of indexing a log is shared out.
#[derive(Debug, Clone, Copy)]
struct Split {
    /// Pieces, each walked by a thread of its own.
    pieces: usize,
    /// Bytes of the log's start walked first, for the formats every other piece starts with.
    prefix: usize,
    /// Bytes a thread reads before walking them, when it reads its piece from a file.
    read: usize,
}

impl Split {
    /// A log smaller than this is walked in one piece: the threads would cost more than they
    /// save.
    const PARALLEL_FROM: usize = 32 << 20;

    /// A file bigger than this is walked in one piece, so that no line of the whole log can
    /// pass `u32::MAX` while its pieces are joined: a record is at least three bytes.
    const PARALLEL_TO: usize = 8 << 30;

    /// How a log of `len` bytes is split on this machine.
    fn of(len: usize) -> Self {
        let threads =
            wasm_thread::available_parallelism().map_or(1, |threads| threads.get().min(8));
        Self {
            pieces: if (Self::PARALLEL_FROM..Self::PARALLEL_TO).contains(&len) {
                threads
            } else {
                1
            },
            prefix: 4 << 20,
            // Eight threads' megabyte each, just read, fits in the processor's cache together.
            read: 1 << 20,
        }
    }

    /// Where each piece of `len` bytes starts, and where the next one does: the last runs to the
    /// end of the log, whatever that turns out to be.
    fn bounds(self, len: usize) -> Vec<(usize, usize)> {
        let per = len.div_ceil(self.pieces.max(1)).max(1);
        let mut bounds: Vec<(usize, usize)> = (0..self.pieces.max(1))
            .map(|piece| (piece * per, (piece + 1) * per))
            .filter(|(start, _)| *start < len.max(1))
            .collect();
        if let Some(last) = bounds.last_mut() {
            last.1 = usize::MAX;
        }
        bounds
    }

    /// The formats the log's first `prefix` bytes declare: what every piece but the first is
    /// walked with.
    fn guess(self, data: &[u8]) -> Walk {
        let mut walk = Walk::default();
        while let Step::Record(_) = walk.step(data.get(..self.prefix).unwrap_or(data), false) {}
        walk
    }
}

/// One `FMT` record that changed what a type means.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Declaration {
    /// The line of the `FMT` record.
    line: usize,
    /// What it declared.
    format: MessageFormat,
}

impl RecordIndex {
    /// Walks a log once and indexes every record in it, format declarations included.
    ///
    /// `setlinecount`: one pass for each record's offset and type, and each type's lines. Only
    /// then are the few records the grid needs before its first row read, through those lists -
    /// the `FMTU` records for the instance fields and the `GPS` records up to the first fix - as
    /// the C# reads them afterwards with `GetEnumeratorType`.
    ///
    /// A log of more than a few tens of megabytes is walked in pieces, a thread each; see
    /// [`Self::check`] for why that finds exactly the records one walk would.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:98-126, 206-330`
    #[must_use]
    pub fn build(data: &[u8]) -> Self {
        Self::build_split(data, Split::of(data.len()))
    }

    /// [`Self::build`], split as `split` says.
    fn build_split(data: &[u8], split: Split) -> Self {
        let bounds = split.bounds(data.len());
        if bounds.len() < 2 {
            let mut piece = Piece::new(Walk::default());
            piece.walk(data, true, usize::MAX, 0);
            return Self::assemble(data, vec![(piece, 0)]);
        }
        let guess = split.guess(data);
        let pieces = walk_pieces(data, &bounds, &guess);
        let checked = Self::check(data, pieces, &bounds, &guess);
        Self::assemble(data, checked.pieces)
    }

    /// Reads a log from `source` and indexes it as it arrives: [`Self::build`] of everything
    /// `source` holds, and those bytes.
    ///
    /// The walk takes each few megabytes as they are read, while they are still in the
    /// processor's cache, rather than reading a whole file into memory and then walking it back
    /// out: for a gigabyte, the difference between touching every byte once and twice.
    /// `size_hint` is how many bytes to make room for at the start - a file's length - and
    /// nothing goes wrong if it is not.
    ///
    /// # Errors
    ///
    /// Whatever reading `source` does.
    pub fn read(
        mut source: impl std::io::Read,
        size_hint: usize,
    ) -> std::io::Result<(Vec<u8>, Self)> {
        use std::io::Read as _;
        let piece_size = Split::of(size_hint).read as u64;
        let mut data = Vec::with_capacity(size_hint);
        let mut piece = Piece::new(Walk::default());
        loop {
            let read = (&mut source).take(piece_size).read_to_end(&mut data)?;
            let complete = read == 0;
            piece.walk(&data, complete, usize::MAX, 0);
            if complete {
                break;
            }
        }
        let index = Self::assemble(&data, vec![(piece, 0)]);
        Ok((data, index))
    }

    /// Reads a log file and indexes it: [`Self::read`], with the file cut into pieces that
    /// threads of their own read and walk at once, each walking what it has just read.
    ///
    /// Each thread opens the file for itself, so no two share a position in it. A file too
    /// small to be worth it, or one that turns out shorter than its length said, is read by
    /// [`Self::read`] instead.
    ///
    /// # Errors
    ///
    /// Whatever opening or reading the file does.
    pub fn read_file(path: &std::path::Path) -> std::io::Result<(Vec<u8>, Self)> {
        let file = mp_os::fs::File::open(path)?;
        // A file whose length does not fit in memory's address space cannot be read into it
        // whatever is done; it is read until that fails.
        let len = usize::try_from(file.metadata()?.len()).unwrap_or(0);
        Self::read_file_split(path, file, len, Split::of(len))
    }

    /// [`Self::read_file`], split as `split` says.
    fn read_file_split(
        path: &std::path::Path,
        mut file: mp_os::fs::File,
        len: usize,
        split: Split,
    ) -> std::io::Result<(Vec<u8>, Self)> {
        use std::io::Read as _;
        let bounds = split.bounds(len);
        if bounds.len() < 2 {
            return Self::read(file, len);
        }
        let mut data = vec![0u8; len];
        // The start of the log, for the formats every other piece is walked with; the first
        // piece's thread reads these bytes again, and walks them itself.
        let prefix = split.prefix.min(len);
        if let Some(start) = data.get_mut(..prefix) {
            file.read_exact(start)?;
        }
        let guess = split.guess(&data);
        let per = bounds.get(1).map_or(len, |second| second.0);
        let read: std::io::Result<Vec<Piece>> = wasm_thread::scope(|scope| {
            let handles: Vec<_> = data
                .chunks_mut(per)
                .zip(&bounds)
                .map(|(bytes, &(start, _))| {
                    let walk = if start == 0 {
                        Walk::default()
                    } else {
                        Walk::resumed_at(0, &guess)
                    };
                    scope.spawn(move || read_piece(path, start, bytes, walk, split.read))
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
        let Ok(mut pieces) = read else {
            // The file changed under us; read it as it is now, in one piece.
            let file = mp_os::fs::File::open(path)?;
            let len = usize::try_from(file.metadata()?.len()).unwrap_or(0);
            return Self::read(file, len);
        };
        // Each piece stopped where its own bytes ran out, part way into a record that runs on
        // into the next piece's; with the whole log read, each finishes its last records.
        for (piece, &(start, end)) in pieces.iter_mut().zip(&bounds) {
            piece.walk.rebase(start);
            piece.walk(&data, true, end, 0);
        }
        let checked = Self::check(&data, pieces, &bounds, &guess);
        let index = Self::assemble(&data, checked.pieces);
        Ok((data, index))
    }

    /// Keeps each piece that found what one walk of the whole log would, and walks again the
    /// ones that did not: the pieces, each with how many of its first records to leave out.
    ///
    /// A walk is its position and its formats, and nothing else: two walks at the same place
    /// knowing the same formats find the same records from there on. So a piece is right from
    /// the first record the true walk - carried on from the end of the piece before - finds in
    /// it, if the piece found that record too, knowing the same formats: the guess it started
    /// with, and no `FMT` among the records before, which were misreadings of the middle of a
    /// record the piece before had not finished.
    ///
    /// Where the true walk knows formats the guess did not - a log that declares a type the
    /// first time it logs one, part way through - every piece from there on was walked knowing
    /// too little, and they are all walked again at once, knowing what the true walk knows.
    /// Where a piece still does not agree - garbage that no two walks read alike - it is walked
    /// again from the true walk's position, in one thread, as it would have been.
    fn check(data: &[u8], pieces: Vec<Piece>, bounds: &[(usize, usize)], guess: &Walk) -> Checked {
        let mut guess = guess.clone();
        let mut rounds = 0usize;
        let mut alone = 0usize;
        let mut pieces: Vec<Option<Piece>> = pieces.into_iter().map(Some).collect();
        let mut checked: Vec<(Piece, usize)> = Vec::with_capacity(pieces.len());
        for (at, &(_, end)) in bounds.iter().enumerate() {
            let Some(mut piece) = pieces.get_mut(at).and_then(Option::take) else {
                continue;
            };
            let Some(truth) = checked.last().map(|(before, _)| before.walk.clone()) else {
                // The first piece is walked from the start of the log: it is the true walk.
                checked.push((piece, 0));
                continue;
            };
            if truth.formats() != guess.formats() {
                rounds += 1;
                guess = Walk::resumed_at(0, &truth);
                let rest = bounds.get(at..).unwrap_or_default();
                let mut again = walk_pieces(data, rest, &guess).into_iter();
                if let Some(first) = again.next() {
                    piece = first;
                }
                for (slot, fresh) in pieces.iter_mut().skip(at + 1).zip(again) {
                    *slot = Some(fresh);
                }
            }
            match agreement(data, &truth, &piece, end) {
                Some(skip) => checked.push((piece, skip)),
                None => {
                    alone += 1;
                    let mut again = Piece::new(truth);
                    again.walk(data, true, end, 0);
                    checked.push((again, 0));
                }
            }
        }
        Checked {
            pieces: checked,
            rounds,
            alone,
        }
    }

    /// The index the pieces make, less the first records of each that [`Self::check`] left
    /// out; then the few records the grid needs before its first row.
    fn assemble(data: &[u8], pieces: Vec<(Piece, usize)>) -> Self {
        let mut index = Self {
            formats: pieces
                .last()
                .map(|(piece, _)| piece.walk.formats().clone())
                .unwrap_or_default(),
            ..Self::default()
        };
        let mut counts = [0usize; 256];
        if let [(_, 0)] = pieces.as_slice() {
            // One walk of the whole log: its tables are the index's as they are.
            for (piece, _) in pieces {
                index.offsets = piece.offsets;
                index.types = piece.types;
                index.lines = piece.lines;
                counts = piece.counts;
            }
        } else {
            let joined = join(pieces);
            index.offsets = joined.offsets;
            index.types = joined.types;
            index.lines = joined.lines;
            counts = joined.counts;
        }
        // Pushing doubles a vector's capacity as it goes, which would leave up to half of each
        // table unused for as long as the log is open.
        index.offsets.shrink_to_fit();
        index.types.shrink_to_fit();
        for list in &mut index.lines {
            list.shrink_to_fit();
        }
        index.counts = (0u8..=u8::MAX)
            .zip(counts)
            .filter(|(_, count)| *count > 0)
            .collect();
        index.read_declarations(data);
        index.read_instances(data);
        index.gps_start = index.first_fix(data);
        index
    }

    /// Every `FMT` record that changed a declaration, from the `FMT` records' own bytes: a log
    /// that is a thousand copies of one log declares every type a thousand times, and comparing
    /// the 86 bytes of two declarations settles nearly all of them without reading either.
    fn read_declarations(&mut self, data: &[u8]) {
        let body = |offset: usize| data.get(offset + 3..offset + 3 + FMT_PAYLOAD_LEN);
        let mut last: [Option<usize>; 256] = [None; 256];
        let mut changes: Vec<Declaration> = Vec::new();
        for line in self.lines_of(FMT_TYPE) {
            let line = *line as usize;
            let Some(offset) = self
                .offsets
                .get(line)
                .and_then(|offset| usize::try_from(*offset).ok())
            else {
                continue;
            };
            let raw = body(offset);
            let Some(&declared) = raw.and_then(<[u8]>::first) else {
                continue;
            };
            let Some(previous) = last.get_mut(usize::from(declared)) else {
                continue;
            };
            if previous.is_some_and(|previous| body(previous) == raw) {
                continue;
            }
            *previous = Some(offset);
            let Some(format) = raw.and_then(parse_fmt) else {
                continue;
            };
            let current = changes
                .iter()
                .rev()
                .find(|change| change.format.msg_type == declared);
            if current.is_none_or(|current| current.format != format) {
                changes.push(Declaration { line, format });
            }
        }
        self.changes = changes;
    }

    /// The instance fields, from the `#` in each `FMTU`'s `UnitIds`: the last `FMTU` of a type
    /// that marks one decides it, so the records are read from the end and a type already
    /// decided is not read again.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:230-245`
    fn read_instances(&mut self, data: &[u8]) {
        let mut by_type = BTreeMap::new();
        let mut by_position = BTreeMap::new();
        let mut columns = PerFormat::default();
        for row in self.rows(|name| name == "FMTU", false).rev() {
            let Some(format) = row.format else {
                continue;
            };
            let resolved = columns.get(format, |format| {
                Column::named(format, "FmtType")
                    .zip(Column::named(format, "UnitIds"))
                    .filter(|(fmt_type, unit_ids)| fmt_type.numeric() && unit_ids.text())
            });
            let Some((fmt_type, unit_ids)) = resolved else {
                continue;
            };
            let Some(value) = fmt_type.read_f64(data, row.offset) else {
                continue;
            };
            // `RecordIndex`'s key is range-checked; `plot::instance_fields` casts, and only a
            // data record counts there, as `next_message` never returns an `FMT`.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // range-checked
            let by_type_key = (0.0..=f64::from(u8::MAX))
                .contains(&value)
                .then_some(value as u8)
                .filter(|key| !by_type.contains_key(key));
            #[allow(clippy::cast_possible_truncation)] // as plot::instance_fields casts it
            let by_position_key = (row.msg_type != FMT_TYPE)
                .then_some(value as i64)
                .filter(|key| !by_position.contains_key(key));
            if by_type_key.is_none() && by_position_key.is_none() {
                continue;
            }
            // Most `FMTU`s mark no instance; a `UnitIds` with no `#` in its bytes cannot have one
            // once it is text, and is passed over without being made into any.
            if unit_ids
                .raw(data, row.offset)
                .is_some_and(|raw| !raw.split(|b| *b == 0).next().unwrap_or(raw).contains(&b'#'))
            {
                continue;
            }
            let Some(position) = unit_ids
                .read(data, row.offset)
                .as_ref()
                .and_then(Value::as_text)
                .and_then(|units| units.trim().find('#'))
            else {
                continue;
            };
            if let Some(key) = by_type_key {
                by_type.insert(key, position);
            }
            if let Some(key) = by_position_key {
                by_position.insert(key, position);
            }
        }
        self.instance_fields = by_type;
        self.instance_positions = by_position;
    }

    /// The first `GPS`-named record with a fix, read through the index: `DFItem` sets
    /// `gpsstarttime` from the first message whose type starts with GPS and that has a fix, and
    /// after that never looks again.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:163-208; ExtLibs/Utilities/DFLogBuffer.cs:311-329`
    fn first_fix(&self, data: &[u8]) -> Option<GpsStart> {
        const LABELS: [&str; 7] = ["Status", "TimeMS", "GMS", "Week", "GWk", "TimeUS", "T"];
        let mut columns = PerFormat::default();
        for row in self.rows(|name| name.starts_with("GPS"), false) {
            let Some(format) = row.format else {
                continue;
            };
            let found = columns.get(format, |format| {
                LABELS.map(|label| Column::named(format, label))
            });
            let start = GpsStart::from_fields(|label| {
                LABELS
                    .iter()
                    .position(|known| *known == label)
                    .and_then(|position| found.get(position).copied().flatten())
                    .and_then(|column| column.read(data, row.offset))
            });
            if start.is_some() {
                return start;
            }
        }
        None
    }

    /// Records in the log.
    #[must_use]
    pub fn len(&self) -> usize {
        self.offsets.len()
    }

    /// Whether the log holds no records at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.offsets.is_empty()
    }

    /// Where a record starts.
    #[must_use]
    pub fn offset(&self, row: usize) -> Option<u64> {
        self.offsets.get(row).copied()
    }

    /// A record's message type.
    #[must_use]
    pub fn msg_type(&self, row: usize) -> Option<u8> {
        self.types.get(row).copied()
    }

    /// The lines of one type's records, in log order: `messageindexline[type]`.
    #[must_use]
    pub fn lines_of(&self, msg_type: u8) -> &[u32] {
        self.lines
            .get(usize::from(msg_type))
            .map_or(&[], Vec::as_slice)
    }

    /// The format a type had where a line is: the last declaration of it at or before the line.
    ///
    /// What the walk decoded that line's record with. It differs from [`Self::formats`], which is
    /// the whole log's last word, only in a log that declares one type twice, differently.
    #[must_use]
    pub fn format_at(&self, msg_type: u8, line: usize) -> Option<&MessageFormat> {
        self.changes
            .iter()
            .take_while(|change| change.line <= line)
            .filter(|change| change.format.msg_type == msg_type)
            .last()
            .map(|change| &change.format)
    }

    /// The formats the log declares.
    #[must_use]
    pub const fn formats(&self) -> &BTreeMap<u8, MessageFormat> {
        &self.formats
    }

    /// A format by message name: `dflog.logformat[name]`.
    #[must_use]
    pub fn format_named(&self, name: &str) -> Option<&MessageFormat> {
        self.formats.values().find(|format| format.name == name)
    }

    /// Which field of a message type is its instance number, if it has one.
    #[must_use]
    pub fn instance_field(&self, msg_type: u8) -> Option<usize> {
        self.instance_fields.get(&msg_type).copied()
    }

    /// `plot::instance_fields`'s reading of the `FMTU`s: format type to the position of its `#`.
    pub(crate) const fn instance_positions(&self) -> &BTreeMap<i64, usize> {
        &self.instance_positions
    }

    /// The most fields any declared message has.
    #[must_use]
    pub fn widest(&self) -> usize {
        self.formats
            .values()
            .map(|format| format.labels.len().max(format.format.len()))
            .max()
            .unwrap_or(0)
    }

    /// The message types the log holds records of, by name, sorted: `SeenMessageTypes`.
    #[must_use]
    pub fn seen(&self) -> Vec<String> {
        let mut names: Vec<String> = self
            .counts
            .keys()
            .filter_map(|msg_type| self.formats.get(msg_type))
            .map(|format| format.name.clone())
            .collect();
        names.sort();
        names.dedup();
        names
    }

    /// The rows holding a message of one name, in log order.
    ///
    /// A type is named by the whole log's formats. Row numbers are held as `u32`: four bytes a
    /// matching row rather than eight, and a log with four billion records of one type would be
    /// a file of a terabyte. One that somehow had more is cut short rather than wrapped round.
    #[must_use]
    pub fn rows_named(&self, name: &str) -> Vec<u32> {
        let wanted: Vec<u8> = self
            .formats
            .iter()
            .filter(|(_, format)| format.name == name)
            .map(|(msg_type, _)| *msg_type)
            .collect();
        let mut rows: Vec<u32> = wanted
            .iter()
            .flat_map(|msg_type| self.lines_of(*msg_type).iter().copied())
            .collect();
        if wanted.len() > 1 {
            rows.sort_unstable();
        }
        rows
    }

    /// Decodes a record from bytes that start at its header.
    #[must_use]
    pub fn decode(&self, bytes: &[u8]) -> Option<LogMessage> {
        decode_record(&self.formats, bytes)
    }

    /// Where GPS time puts the start of the log's clock.
    #[must_use]
    pub const fn gps_start(&self) -> Option<GpsStart> {
        self.gps_start
    }

    /// Bytes the index itself holds on the heap for its per-record tables.
    ///
    /// The number that scales with the log. Formats and counts are bounded by the 256 message
    /// types a log can declare, and are left out.
    #[must_use]
    pub fn record_bytes(&self) -> usize {
        self.offsets.capacity() * std::mem::size_of::<u64>()
            + self.types.capacity()
            + self
                .lines
                .iter()
                .map(|list| list.capacity() * std::mem::size_of::<u32>())
                .sum::<usize>()
    }

    /// One type's records, split where the log declares the type again, differently: each
    /// stretch with the format it was logged under and the name that gives it.
    ///
    /// `FMT` records logged before the log declares `FMT` itself are a stretch with no format,
    /// named `FMT`: they are read by the fixed layout, as [`decode_record`] reads them.
    pub(crate) fn segments(&self, msg_type: u8) -> Vec<Segment<'_>> {
        let lines = self.lines_of(msg_type);
        let declared: Vec<&Declaration> = self
            .changes
            .iter()
            .filter(|change| change.format.msg_type == msg_type)
            .collect();
        let split = |line: usize| lines.partition_point(|at| (*at as usize) < line);
        let mut segments = Vec::new();
        if msg_type == FMT_TYPE {
            let end = split(declared.first().map_or(usize::MAX, |first| first.line));
            if let Some(before) = lines.get(..end).filter(|before| !before.is_empty()) {
                segments.push(Segment {
                    msg_type,
                    format: None,
                    name: "FMT",
                    lines: before,
                });
            }
        }
        for (position, declaration) in declared.iter().enumerate() {
            let start = split(declaration.line);
            let end = declared
                .get(position + 1)
                .map_or(lines.len(), |next| split(next.line));
            if let Some(stretch) = lines.get(start..end).filter(|stretch| !stretch.is_empty()) {
                segments.push(Segment {
                    msg_type,
                    format: Some(&declaration.format),
                    name: &declaration.format.name,
                    lines: stretch,
                });
            }
        }
        segments
    }

    /// The records whose name, where each was logged, passes `wanted`, in log order: what
    /// `GetEnumeratorType(string[])` walks, merging the types' line lists.
    ///
    /// `data_only` leaves out `FMT` records whatever they are named, as `next_message` does.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:701-760`
    pub(crate) fn rows(&self, wanted: impl Fn(&str) -> bool, data_only: bool) -> Rows<'_> {
        let cursors = (0u8..=u8::MAX)
            .filter(|msg_type| !(data_only && *msg_type == FMT_TYPE))
            .filter(|msg_type| !self.lines_of(*msg_type).is_empty())
            .flat_map(|msg_type| self.segments(msg_type))
            .filter(|segment| wanted(segment.name))
            .collect();
        Rows {
            offsets: &self.offsets,
            cursors,
        }
    }

    /// Every stretch of data records, each type and each declaration of it: what a reader that
    /// wants every type goes through, one stretch at a time.
    pub(crate) fn data_segments(&self) -> Vec<Segment<'_>> {
        (0u8..=u8::MAX)
            .filter(|msg_type| *msg_type != FMT_TYPE)
            .filter(|msg_type| !self.lines_of(*msg_type).is_empty())
            .flat_map(|msg_type| self.segments(msg_type))
            .collect()
    }

    /// Each record's type, by line.
    pub(crate) fn types(&self) -> &[u8] {
        &self.types
    }

    /// Where each record starts, by line.
    pub(crate) fn offsets(&self) -> &[u64] {
        &self.offsets
    }

    /// Where the record on a line starts, as an index into the log's bytes.
    pub(crate) fn start_of(&self, line: u32) -> Option<usize> {
        self.offsets
            .get(line as usize)
            .and_then(|offset| usize::try_from(*offset).ok())
    }

    /// The first format of a name, by type number, among those declared at each point of the
    /// log: `(line, format)` wherever that changes, in log order.
    ///
    /// `GetGPSFromRow` finds a field's position with `FindMessageOffset("GPS", ...)`, which asks
    /// the formats for the first named `GPS`; a record is read against the formats the walk had
    /// when it reached it.
    pub(crate) fn first_named_over_time(&self, name: &str) -> Vec<(usize, Option<&MessageFormat>)> {
        let mut table: Vec<Option<&MessageFormat>> = vec![None; 256];
        let mut timeline: Vec<(usize, Option<&MessageFormat>)> = vec![(0, None)];
        for change in &self.changes {
            if let Some(slot) = table.get_mut(usize::from(change.format.msg_type)) {
                *slot = Some(&change.format);
            }
            let first = table
                .iter()
                .flatten()
                .copied()
                .find(|format| format.name == name);
            let same = timeline
                .last()
                .is_some_and(|(_, last)| match (last, first) {
                    (Some(last), Some(first)) => std::ptr::eq(*last, first),
                    (None, None) => true,
                    _ => false,
                });
            if !same {
                timeline.push((change.line, first));
            }
        }
        timeline
    }
}

/// One type's records under one declaration of it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Segment<'a> {
    /// The type.
    pub(crate) msg_type: u8,
    /// The format they were logged under; `None` only for `FMT` records logged before the log
    /// declares `FMT`.
    pub(crate) format: Option<&'a MessageFormat>,
    /// The name that format gives them.
    pub(crate) name: &'a str,
    /// Their lines, in log order.
    pub(crate) lines: &'a [u32],
}

/// One record found through the index.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Row<'a> {
    /// Its line.
    pub(crate) line: usize,
    /// Where it starts in the log's bytes.
    pub(crate) offset: usize,
    /// Its type.
    pub(crate) msg_type: u8,
    /// The format it was logged under: see [`Segment::format`].
    pub(crate) format: Option<&'a MessageFormat>,
    /// The name that format gives it.
    pub(crate) name: &'a str,
}

/// The records of several stretches, merged into log order, from either end.
#[derive(Debug, Clone)]
pub(crate) struct Rows<'a> {
    /// The log's record offsets.
    offsets: &'a [u64],
    /// What is left of each stretch.
    cursors: Vec<Segment<'a>>,
}

impl<'a> Rows<'a> {
    /// The record on a line of a stretch.
    fn row(&self, segment: &Segment<'a>, line: u32) -> Option<Row<'a>> {
        Some(Row {
            line: line as usize,
            offset: usize::try_from(*self.offsets.get(line as usize)?).ok()?,
            msg_type: segment.msg_type,
            format: segment.format,
            name: segment.name,
        })
    }
}

impl<'a> Iterator for Rows<'a> {
    type Item = Row<'a>;

    fn next(&mut self) -> Option<Row<'a>> {
        loop {
            let (position, line) = self
                .cursors
                .iter()
                .enumerate()
                .filter_map(|(position, cursor)| Some((position, *cursor.lines.first()?)))
                .min_by_key(|(_, line)| *line)?;
            let cursor = self.cursors.get_mut(position)?;
            cursor.lines = cursor.lines.get(1..).unwrap_or_default();
            let segment = *cursor;
            if let Some(row) = self.row(&segment, line) {
                return Some(row);
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let left = self.cursors.iter().map(|cursor| cursor.lines.len()).sum();
        (0, Some(left))
    }
}

impl DoubleEndedIterator for Rows<'_> {
    fn next_back(&mut self) -> Option<Self::Item> {
        loop {
            let (position, line) = self
                .cursors
                .iter()
                .enumerate()
                .filter_map(|(position, cursor)| Some((position, *cursor.lines.last()?)))
                .max_by_key(|(_, line)| *line)?;
            let cursor = self.cursors.get_mut(position)?;
            cursor.lines = cursor
                .lines
                .get(..cursor.lines.len().saturating_sub(1))
                .unwrap_or_default();
            let segment = *cursor;
            if let Some(row) = self.row(&segment, line) {
                return Some(row);
            }
        }
    }
}

/// The instant GPS time gives the log's clock: `DFLog.gpsstarttime`, with `msoffset`.
///
/// Mission Planner writes each row's time as the GPS time of the first fix plus how long after
/// that fix the row was logged. Before any fix the log has no wall-clock time at all, and the C#
/// counts from the year 1 - see [`boot_ms`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsStart {
    /// The GPS week of the fix.
    pub week: u32,
    /// Milliseconds into that week.
    pub week_ms: u64,
    /// The vehicle's clock at the fix, in whole milliseconds since boot.
    pub boot_ms: i64,
}

/// Unix time of the GPS epoch, 1980-01-06, in milliseconds.
const GPS_EPOCH_UNIX_MS: i64 = 315_964_800_000;

/// Milliseconds in a GPS week.
const WEEK_MS: i64 = 7 * 24 * 60 * 60 * 1000;

impl GpsStart {
    /// Reads the GPS time out of a `GPS` message, if it has a fix and a plausible time.
    ///
    /// `GetTimeGPS`: a status of 0, 1 or 2 is no 3D fix and no time; the time of week is
    /// `TimeMS` in old logs and `GMS` in new ones, the week `Week` or `GWk`; a week past 5000 or a
    /// time past the end of the week is garbage. The boot-clock offset is `TimeUS` in
    /// milliseconds, or the old `T` field.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:163-208, 646-700`
    #[must_use]
    pub fn from_message(message: &LogMessage) -> Option<Self> {
        Self::from_fields(|label| message.field(label).cloned())
    }

    /// [`Self::from_message`] with each field fetched by label, as [`LogMessage::field`] finds
    /// it: how the index reads the few fields it needs of a `GPS` record without decoding it.
    pub(crate) fn from_fields(field: impl Fn(&str) -> Option<Value>) -> Option<Self> {
        let number = |label: &str| field(label).as_ref().and_then(Value::as_f64);
        if let Some(status) = number("Status")
            && status < 3.0
        {
            return None;
        }
        let week_ms = field("TimeMS")
            .or_else(|| field("GMS"))
            .as_ref()
            .and_then(Value::as_f64)?;
        let week = field("Week")
            .or_else(|| field("GWk"))
            .as_ref()
            .and_then(Value::as_f64)?;
        if !(0.0..=5000.0).contains(&week) || !(0.0..=WEEK_MS as f64).contains(&week_ms) {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // both range-checked just above
        let (week, week_ms) = (week as u32, week_ms as u64);
        // `msoffset = long.Parse(TimeUS) / 1000`: integer milliseconds, truncated.
        #[allow(clippy::cast_possible_truncation)] // microseconds since boot fit an i64
        let boot_ms = number("TimeUS")
            .map(|time_us| (time_us as i64) / 1000)
            .or_else(|| number("T").map(|t| t as i64))
            .unwrap_or(0);
        Some(Self {
            week,
            week_ms,
            boot_ms,
        })
    }

    /// The fix's time as Unix milliseconds, UTC.
    ///
    /// `gpsTimeToTime`: the GPS epoch, plus the weeks, plus the time of week, less the leap
    /// seconds GPS time has gained on UTC. The C# asks for the leap seconds of *today's* date
    /// rather than the log's, so a log from 2016 read now is a second out; the caller decides
    /// which date to ask about, and [`leap_seconds_gps`] answers.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:702-711`
    #[must_use]
    pub fn unix_ms(&self, leap_seconds: i64) -> i64 {
        GPS_EPOCH_UNIX_MS
            + i64::from(self.week) * WEEK_MS
            + i64::try_from(self.week_ms).unwrap_or(0)
            - leap_seconds * 1000
    }
}

/// Seconds GPS time is ahead of UTC in a given month: `LeapSecondsGPS`.
///
/// The TAI-UTC table less the nineteen seconds GPS time was already behind TAI at its epoch. The
/// C#'s table stops at 2017, as the leap seconds have so far.
/// `// C#: ExtLibs/Utilities/rtcm3.cs:715-735`
#[must_use]
pub fn leap_seconds_gps(year: i32, month: u32) -> i64 {
    let yyyymm = i64::from(year) * 100 + i64::from(month);
    let tai = match yyyymm {
        201_701.. => 37,
        201_507.. => 36,
        201_207.. => 35,
        200_901.. => 34,
        200_601.. => 33,
        199_901.. => 32,
        199_707.. => 31,
        199_601.. => 30,
        _ => 0,
    };
    tai - 19
}

/// A record's time on the vehicle's clock, in milliseconds since boot: `DFItem.timems`.
///
/// `TimeMS` if the message has one, else `TimeUS` in milliseconds, else the old `T`. A message
/// with none of them - `FMT`, say - has no time, and the C# then uses zero.
/// `// C#: ExtLibs/Utilities/DFLog.cs:98-131`
#[must_use]
pub fn boot_ms(message: &LogMessage) -> Option<f64> {
    if let Some(ms) = message.field("TimeMS").and_then(Value::as_f64) {
        return Some(ms);
    }
    if let Some(us) = message.field("TimeUS").and_then(Value::as_f64) {
        return Some(us / 1000.0);
    }
    message.field("T").and_then(Value::as_f64)
}

/// The pieces' tables, joined into the whole log's.
struct Joined {
    offsets: Vec<u64>,
    types: Vec<u8>,
    lines: Vec<Vec<u32>>,
    counts: [usize; 256],
}

/// Where one piece's records go in the joined tables: its offsets, its types, and its lines of
/// each type.
type Place<'a> = (&'a mut [u64], &'a mut [u8], Vec<&'a mut [u32]>);

/// Joins the pieces' tables, less the first records of each that [`RecordIndex::check`] left
/// out, each piece copying its own into its place at once: for a gigabyte, three hundred
/// megabytes of tables that one thread copying them would spend a fifth of a second on.
fn join(pieces: Vec<(Piece, usize)>) -> Joined {
    let mut counts = [0usize; 256];
    // Where each piece's records go, and its lines of each type.
    let mut bases = Vec::with_capacity(pieces.len());
    let mut firsts: Vec<[usize; 256]> = Vec::with_capacity(pieces.len());
    let mut total = 0usize;
    let mut totals = [0usize; 256];
    for (piece, skip) in &pieces {
        bases.push(total);
        total += piece.offsets.len().saturating_sub(*skip);
        let left_out = piece.types.get(..*skip).unwrap_or_default();
        let mut first = [0usize; 256];
        for (slot, ((count, found), (from, lines))) in counts
            .iter_mut()
            .zip(piece.counts)
            .zip(first.iter_mut().zip(&piece.lines))
            .enumerate()
        {
            let skipped = left_out
                .iter()
                .filter(|msg_type| usize::from(**msg_type) == slot)
                .count();
            *count += found - skipped;
            *from = lines.partition_point(|line| (*line as usize) < *skip);
            if let Some(total) = totals.get_mut(slot) {
                *total += lines.len() - *from;
            }
        }
        firsts.push(first);
    }
    let mut offsets = vec![0u64; total];
    let mut types = vec![0u8; total];
    let mut lines: Vec<Vec<u32>> = totals.iter().map(|total| vec![0u32; *total]).collect();
    // Each piece's place in each table.
    let mut places: Vec<Place<'_>> = Vec::with_capacity(pieces.len());
    let (mut offsets_left, mut types_left) = (offsets.as_mut_slice(), types.as_mut_slice());
    let mut lines_left: Vec<&mut [u32]> = lines.iter_mut().map(Vec::as_mut_slice).collect();
    for ((piece, skip), first) in pieces.iter().zip(&firsts) {
        let kept = piece.offsets.len().saturating_sub(*skip);
        let (offsets_here, rest) = std::mem::take(&mut offsets_left)
            .split_at_mut_checked(kept)
            .unwrap_or_default();
        offsets_left = rest;
        let (types_here, rest) = std::mem::take(&mut types_left)
            .split_at_mut_checked(kept)
            .unwrap_or_default();
        types_left = rest;
        let lines_here = lines_left
            .iter_mut()
            .zip(first.iter().zip(&piece.lines))
            .map(|(left, (from, found))| {
                let (here, rest) = std::mem::take(left)
                    .split_at_mut_checked(found.len() - from)
                    .unwrap_or_default();
                *left = rest;
                here
            })
            .collect();
        places.push((offsets_here, types_here, lines_here));
    }
    wasm_thread::scope(|scope| {
        for (((piece, skip), (offsets, types, lines)), (base, first)) in pieces
            .into_iter()
            .zip(places)
            .zip(bases.into_iter().zip(&firsts))
        {
            scope.spawn(move || {
                offsets.copy_from_slice(piece.offsets.get(skip..).unwrap_or_default());
                types.copy_from_slice(piece.types.get(skip..).unwrap_or_default());
                for ((into, found), from) in lines.into_iter().zip(&piece.lines).zip(first) {
                    let found = found.get(*from..).unwrap_or_default();
                    for (into, line) in into.iter_mut().zip(found) {
                        // Below `u32::MAX`: a log is walked in pieces only when it is too small
                        // to hold that many records.
                        *into = u32::try_from(*line as usize - skip + base).unwrap_or(u32::MAX);
                    }
                }
            });
        }
    });
    Joined {
        offsets,
        types,
        lines,
        counts,
    }
}

/// Walks each piece of `data` in a thread of its own: the first from the start of the log, the
/// others from their first byte knowing the formats `guess` knows.
fn walk_pieces(data: &[u8], bounds: &[(usize, usize)], guess: &Walk) -> Vec<Piece> {
    wasm_thread::scope(|scope| {
        let handles: Vec<_> = bounds
            .iter()
            .map(|&(start, end)| {
                let walk = if start == 0 {
                    Walk::default()
                } else {
                    Walk::resumed_at(start, guess)
                };
                scope.spawn(move || {
                    let mut piece = Piece::new(walk);
                    piece.walk(data, true, end, 0);
                    piece
                })
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
    })
}

/// How many of a piece's first records to leave out, if from there on it found what the true
/// walk finds: see [`RecordIndex::check`]. `truth` is the true walk where the piece before it
/// stopped; the piece was walked knowing the formats `truth` knows.
fn agreement(data: &[u8], truth: &Walk, piece: &Piece, end: usize) -> Option<usize> {
    let mut probe = truth.clone();
    let Step::Record(first) = probe.step_until(data, true, end) else {
        return None;
    };
    let at = first.offset as u64;
    let skip = piece.offsets.partition_point(|offset| *offset < at);
    let found = piece.offsets.get(skip) == Some(&at);
    let misread_formats = piece
        .types
        .get(..skip)
        .is_some_and(|types| types.contains(&FMT_TYPE));
    (found && !misread_formats).then_some(skip)
}

/// Reads one piece of a log file into `bytes` a few megabytes at a time, walking each few as it
/// arrives, while it is still in the cache. The walk stops short of a record that runs on past
/// the piece, which [`RecordIndex::read_file`] finishes once every piece is read.
fn read_piece(
    path: &std::path::Path,
    start: usize,
    bytes: &mut [u8],
    walk: Walk,
    chunk: usize,
) -> std::io::Result<Piece> {
    use std::io::{Read as _, Seek as _};
    let mut file = mp_os::fs::File::open(path)?;
    file.seek(std::io::SeekFrom::Start(start as u64))?;
    let mut piece = Piece::new(walk);
    let limit = bytes.len();
    let mut filled = 0;
    while filled < limit {
        let next = (filled + chunk.max(1)).min(limit);
        if let Some(target) = bytes.get_mut(filled..next) {
            file.read_exact(target)?;
        }
        filled = next;
        piece.walk(bytes.get(..filled).unwrap_or_default(), false, limit, start);
    }
    Ok(piece)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dataflash::DataflashReader;
    use crate::testlog::{fmt, record};

    fn fixture() -> Vec<u8> {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin");
        mp_os::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    /// Every record is indexed, the format declarations among them, and in log order.
    ///
    /// The number is what a byte-level walk of the fixture finds: 169 `FMT` records and 11,270
    /// data records. `next_message` sees the second figure; the grid shows both, as the C# grid
    /// does, because an `FMT` is a line of the log like any other.
    #[test]
    fn the_real_log_indexes_every_record_format_declarations_included() {
        let data = fixture();
        let index = RecordIndex::build(&data);
        assert_eq!(index.len(), 11_439);

        let mut messages = 0usize;
        let mut reader = DataflashReader::new(&data);
        while reader.next_message().is_some() {
            messages += 1;
        }
        assert_eq!(
            index.len() - messages,
            169,
            "the difference is exactly the FMTs"
        );
        assert!(
            index.offsets.windows(2).all(|pair| pair[0] < pair[1]),
            "offsets run forward"
        );
        assert_eq!(index.offset(0), Some(0), "a healthy log starts at byte 0");
    }

    /// A row is decoded from its own bytes and nothing else, against the whole log's formats.
    #[test]
    fn a_row_decodes_from_its_offset_alone() {
        let data = fixture();
        let index = RecordIndex::build(&data);

        // Row 0 is the log declaring FMT itself, decoded with that declaration.
        let first = index
            .decode(&data[usize::try_from(index.offset(0).unwrap()).unwrap()..])
            .unwrap();
        assert_eq!(first.name, "FMT");
        assert_eq!(first.field("Name"), Some(&Value::Text("FMT".to_owned())));

        // Every ATT the reader finds in order, the index finds by offset.
        let mut reader = DataflashReader::new(&data);
        let walked: Vec<LogMessage> = std::iter::from_fn(|| reader.next_message())
            .filter(|message| message.name == "ATT")
            .collect();
        let rows = index.rows_named("ATT");
        assert_eq!(rows.len(), walked.len());
        assert_eq!(rows.len(), 182);
        for (row, expected) in rows.iter().zip(&walked) {
            let offset = usize::try_from(index.offset(*row as usize).unwrap()).unwrap();
            assert_eq!(index.decode(&data[offset..]).as_ref(), Some(expected));
        }
    }

    /// The instance field comes from the `#` in `FMTU`, as a position in the message.
    #[test]
    fn the_instance_field_is_the_hash_in_fmtu() {
        let index = RecordIndex::build(&fixture());
        let vibe = index.format_named("VIBE").unwrap();
        let field = index.instance_field(vibe.msg_type).unwrap();
        assert_eq!(vibe.labels[field], "IMU");
        let att = index.format_named("ATT").unwrap();
        assert_eq!(
            index.instance_field(att.msg_type),
            None,
            "ATT has one instance"
        );
    }

    /// The seen types are the ones with records, by name, sorted - not every format declared.
    #[test]
    fn the_seen_types_are_the_ones_with_records() {
        let index = RecordIndex::build(&fixture());
        let seen = index.seen();
        assert!(seen.windows(2).all(|pair| pair[0] < pair[1]), "{seen:?}");
        for name in ["ATT", "FMT", "GPS", "VIBE", "CMD"] {
            assert!(seen.iter().any(|seen| seen == name), "{name} in {seen:?}");
        }
        // GPA is declared and has records; UBX1 is declared and has none.
        assert!(!seen.iter().any(|seen| seen == "UBX1"), "{seen:?}");
        assert_eq!(index.widest(), 16);
    }

    /// Thirteen bytes a record, and nothing that grows with a record's size.
    #[test]
    fn the_index_costs_thirteen_bytes_a_record() {
        let index = RecordIndex::build(&fixture());
        // `shrink_to_fit` may leave a little over, by the allocator's leave; not a doubling.
        assert!(index.record_bytes() >= 13 * index.len());
        assert!(
            index.record_bytes() <= 13 * index.len() + 64,
            "{} bytes for {} records",
            index.record_bytes(),
            index.len()
        );
    }

    /// The fixture's GPS never gets a fix, so there is no GPS time to anchor the clock.
    #[test]
    fn a_log_with_no_fix_has_no_gps_start() {
        assert_eq!(RecordIndex::build(&fixture()).gps_start(), None);
    }

    /// Arbitrary bytes index without panicking, and the index never outnumbers the bytes.
    #[test]
    fn garbage_indexes_to_nothing_and_does_not_panic() {
        for pattern in [
            vec![0xA3u8; 4096],
            [0xA3u8, 0x95, 0x80].repeat(1024),
            [0xA3u8, 0x95, 0xFF].repeat(1024),
            (0..=255u8).cycle().take(10_000).collect(),
        ] {
            let index = RecordIndex::build(&pattern);
            assert!(index.len() <= pattern.len());
            for row in 0..index.len() {
                let offset = usize::try_from(index.offset(row).unwrap()).unwrap();
                let _ = index.decode(&pattern[offset..]);
            }
        }
    }

    fn testdata(name: &str) -> Vec<u8> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name);
        mp_os::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    /// Logs whose pieces a split cuts in awkward places: the fixtures, logs joined end to end -
    /// one of which declares its types three times over, differently - and garbage.
    fn awkward_logs() -> Vec<(&'static str, Vec<u8>)> {
        let healthy = fixture();
        let damaged = testdata("dataflash_damaged.bin");
        let edge = testdata("dataflash/edge.bin");
        vec![
            ("healthy", healthy.clone()),
            ("damaged", damaged.clone()),
            ("edge", edge.clone()),
            (
                "healthy x4",
                [&healthy[..], &healthy, &healthy, &healthy].concat(),
            ),
            (
                "healthy, edge, healthy",
                [&healthy[..], &edge, &healthy].concat(),
            ),
            ("damaged, healthy", [&damaged[..], &healthy].concat()),
            ("edge, damaged, edge", [&edge[..], &damaged, &edge].concat()),
            ("garbage", (0..=255u8).cycle().take(50_000).collect()),
            ("headers", [0xA3u8, 0x95, 0x80].repeat(5_000)),
            ("empty", Vec::new()),
        ]
    }

    /// A split for logs of a few hundred kilobytes: pieces of tens of kilobytes, formats guessed
    /// from the first 4 kB, read a kilobyte at a time.
    const fn small(pieces: usize) -> Split {
        Split {
            pieces,
            prefix: 4096,
            read: 1024,
        }
    }

    /// Two indexes are the same, table for table.
    fn assert_same(found: &RecordIndex, expected: &RecordIndex, what: &str) {
        assert_eq!(found.offsets, expected.offsets, "{what}: offsets");
        assert_eq!(found.types, expected.types, "{what}: types");
        assert_eq!(found.lines, expected.lines, "{what}: lines");
        assert_eq!(found.changes, expected.changes, "{what}: declarations");
        assert_eq!(found.formats, expected.formats, "{what}: formats");
        assert_eq!(found.counts, expected.counts, "{what}: counts");
        assert_eq!(found.instance_fields, expected.instance_fields, "{what}");
        assert_eq!(
            found.instance_positions, expected.instance_positions,
            "{what}"
        );
        assert_eq!(found.gps_start, expected.gps_start, "{what}: gps start");
    }

    /// Walked in pieces, a thread each, a log is indexed exactly as one walk indexes it: where
    /// the pieces agree with the walk and where they have to be walked again.
    #[test]
    fn a_log_walked_in_pieces_is_indexed_as_one_walk_indexes_it() {
        for (name, log) in awkward_logs() {
            let whole = RecordIndex::build_split(&log, small(1));
            for pieces in [2, 3, 7, 16] {
                let split = RecordIndex::build_split(&log, small(pieces));
                assert_same(&split, &whole, &format!("{name} in {pieces}"));
            }
        }
    }

    /// Read from a file in pieces, a thread each walking what it has just read, a log is its
    /// bytes and the index one walk makes of them.
    #[test]
    fn a_log_read_in_pieces_is_indexed_as_one_walk_indexes_it() {
        let dir = mp_os::temp_dir();
        for (name, log) in awkward_logs() {
            let path = dir.join(format!(
                "mp-log-read-in-pieces-{}-{}.bin",
                mp_os::process_id(),
                name.replace([' ', ','], "-")
            ));
            mp_os::fs::write(&path, &log).unwrap();
            let whole = RecordIndex::build_split(&log, small(1));
            for pieces in [1, 2, 5, 16] {
                let file = mp_os::fs::File::open(&path).unwrap();
                let (data, index) =
                    RecordIndex::read_file_split(&path, file, log.len(), small(pieces)).unwrap();
                assert_eq!(data, log, "{name} in {pieces}");
                assert_same(&index, &whole, &format!("{name} read in {pieces}"));
            }
            let (data, index) = RecordIndex::read_file(&path).unwrap();
            assert_eq!(data, log, "{name}");
            assert_same(&index, &whole, name);
            mp_os::fs::remove_file(&path).unwrap();
        }
    }

    /// Where the log declares nothing new part way through, the pieces are kept as their
    /// threads walked them - only the misreadings at the start of each are left out - and none
    /// is walked again. The parallel walk is worth something only if that is so.
    #[test]
    fn pieces_of_a_log_that_declares_nothing_new_are_kept() {
        let healthy = fixture();
        let log = [&healthy[..], &healthy, &healthy, &healthy].concat();
        // The fixture declares its last type 297,888 bytes in: the guess is from all of it.
        let split = Split {
            prefix: healthy.len(),
            ..small(4)
        };
        let bounds = split.bounds(log.len());
        let guess = split.guess(&log);
        let pieces: Vec<Piece> = bounds
            .iter()
            .map(|&(start, end)| {
                let walk = if start == 0 {
                    Walk::default()
                } else {
                    Walk::resumed_at(start, &guess)
                };
                let mut piece = Piece::new(walk);
                piece.walk(&log, true, end, 0);
                piece
            })
            .collect();
        let walked: Vec<*const u64> = pieces.iter().map(|piece| piece.offsets.as_ptr()).collect();
        let checked = RecordIndex::check(&log, pieces, &bounds, &guess);
        let kept: Vec<*const u64> = checked
            .pieces
            .iter()
            .map(|(piece, _)| piece.offsets.as_ptr())
            .collect();
        assert_eq!(kept, walked, "every piece kept as walked");
        assert_eq!((checked.rounds, checked.alone), (0, 0));
    }

    /// A guess from too little of the log - here a few kilobytes, short of the types the
    /// fixture declares only when it first logs them - costs a second round of pieces walked at
    /// once, knowing what the true walk knows, and not a walk of the rest in one thread: each
    /// piece found again is kept as that round walked it.
    #[test]
    fn a_guess_that_knows_too_little_is_walked_again_at_once() {
        let healthy = fixture();
        let log = [&healthy[..], &healthy, &healthy, &healthy].concat();
        let split = small(4);
        let bounds = split.bounds(log.len());
        let guess = split.guess(&log);
        let pieces = walk_pieces(&log, &bounds, &guess);
        let checked = RecordIndex::check(&log, pieces, &bounds, &guess);
        assert_eq!(
            (checked.rounds, checked.alone),
            (1, 0),
            "one more round, and no piece walked alone"
        );
        assert_same(
            &RecordIndex::assemble(&log, checked.pieces),
            &RecordIndex::build_split(&log, small(1)),
            "after a second round",
        );
    }

    /// A split shares a log out only when it is big enough to be worth it, and bounds cover it.
    #[test]
    fn a_split_covers_the_log() {
        assert_eq!(Split::of(1 << 20).pieces, 1, "a megabyte is one piece");
        let split = small(3);
        let bounds = split.bounds(10);
        assert_eq!(bounds, vec![(0, 4), (4, 8), (8, usize::MAX)]);
        assert_eq!(small(16).bounds(3), vec![(0, 1), (1, 2), (2, usize::MAX)]);
        assert_eq!(small(4).bounds(0), vec![(0, usize::MAX)]);
    }

    const GPS: u8 = 140;

    /// A GPS record: `QBBIHBcLLeffffB`, the layout of the fixture's GPS.
    fn gps(time_us: u64, status: u8, week_ms: u32, week: u16, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(0); // I
        payload.push(status);
        payload.extend(week_ms.to_le_bytes());
        payload.extend(week.to_le_bytes());
        payload.push(10); // NSats
        payload.extend(90i16.to_le_bytes()); // HDop, hundredths
        #[allow(clippy::cast_possible_truncation)]
        payload.extend(((lat * 1e7) as i32).to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        payload.extend(((lng * 1e7) as i32).to_le_bytes());
        payload.extend(58_400i32.to_le_bytes()); // Alt, hundredths
        for _ in 0..4 {
            payload.extend(0f32.to_le_bytes());
        }
        payload.push(1); // U
        record(GPS, &payload)
    }

    fn gps_log(records: &[Vec<u8>]) -> Vec<u8> {
        let mut log = fmt(
            GPS,
            51,
            "GPS",
            "QBBIHBcLLeffffB",
            "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U",
        );
        for bytes in records {
            log.extend_from_slice(bytes);
        }
        log
    }

    /// The first 3D fix sets the clock; a GPS with no fix before it does not.
    #[test]
    fn the_first_fix_anchors_the_clock() {
        let log = gps_log(&[
            gps(1_000_000, 1, 0, 0, 0.0, 0.0),
            gps(2_500_700, 3, 345_600_000, 2_300, -35.363, 149.165),
            gps(3_000_000, 3, 345_600_500, 2_300, -35.363, 149.165),
        ]);
        let start = RecordIndex::build(&log).gps_start().unwrap();
        assert_eq!(
            start,
            GpsStart {
                week: 2_300,
                week_ms: 345_600_000,
                boot_ms: 2_500,
            },
            "TimeUS 2,500,700 is 2,500 whole milliseconds"
        );
        // Week 2300 began 2024-02-04; four days in is 2024-02-08, less 18 leap seconds.
        assert_eq!(start.unix_ms(18), 1_707_350_400_000 - 18_000);
    }

    /// A week or a time of week outside what GPS can say is not a time.
    #[test]
    fn an_impossible_gps_time_is_refused() {
        let message = |week: u16, week_ms: u32| {
            let log = gps_log(&[gps(0, 3, week_ms, week, 0.0, 0.0)]);
            RecordIndex::build(&log).gps_start()
        };
        assert!(message(2_300, 1_000).is_some());
        assert_eq!(message(6_000, 1_000), None, "week past 5000");
        assert_eq!(
            message(2_300, 700_000_000),
            None,
            "past the end of the week"
        );
    }

    /// The leap second table, as the C# has it.
    #[test]
    fn leap_seconds_follow_the_table() {
        assert_eq!(leap_seconds_gps(2026, 9), 18);
        assert_eq!(leap_seconds_gps(2017, 1), 18);
        assert_eq!(leap_seconds_gps(2016, 12), 17);
        assert_eq!(leap_seconds_gps(2012, 7), 16);
        assert_eq!(
            leap_seconds_gps(1990, 1),
            -19,
            "the C# has no entry before 1996"
        );
    }

    /// A message's boot time prefers `TimeMS`, then `TimeUS`, then `T`.
    #[test]
    fn boot_time_reads_whichever_field_the_message_has() {
        let message = |fields: &[(&str, Value)]| LogMessage {
            name: "X".to_owned(),
            fields: fields
                .iter()
                .map(|(label, value)| ((*label).to_owned(), value.clone()))
                .collect(),
        };
        assert_eq!(
            boot_ms(&message(&[("TimeUS", Value::Uint(52_351_828))])),
            Some(52_351.828)
        );
        assert_eq!(
            boot_ms(&message(&[("TimeMS", Value::Uint(1_500))])),
            Some(1_500.0)
        );
        assert_eq!(boot_ms(&message(&[("T", Value::Uint(9))])), Some(9.0));
        assert_eq!(boot_ms(&message(&[("Type", Value::Uint(9))])), None);
    }
}
