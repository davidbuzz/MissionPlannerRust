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

//! `DataflashLog.py`: the analyzer's own reading of a dataflash log's text - the `.log` Mission
//! Planner converts a `.bin` to before it runs the analyzer - into formats, parameters, messages,
//! mode changes and a channel per field, line numbers kept, and the helpers the checks share
//! (`DataflashLogHelper`, `LogIterator`).
//!
//! Ported as it is, quirks included: a line the reader cannot place is a "BAD LINE" and skipped
//! (`runner.exe` is run with `-s`), a second `FMT` for a name is ignored, a mode number a copter
//! does not name - or a mode the converter has already named - is kept as it came, the firmware
//! line is read from the first `MSG` that names a known vehicle and the `MODE` lines before it
//! are patched in then, and the log's size counts each line a byte longer than it is, as the
//! Windows runner reads `\r\n` as one byte and adds one for each line.
//!
//! `read_binary` is not ported: Mission Planner always converts a `.bin` first, so the runner is
//! never given one (`GCSViews/FlightData.cs:1319-1336`).
//! `// LogAnalyzer/py2exe/DataflashLog.py; VehicleType.py`

use std::collections::HashMap;
use std::rc::Rc;

use super::pyval::{Op, PyError, PyResult, Value, floor_div, np_mean, py_max, py_min, py_str};

/// `VehicleType`: the three the analyzer knows. `// VehicleType.py:1-4`
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VehicleType {
    /// `Plane = 17`.
    Plane,
    /// `Copter = 23`.
    Copter,
    /// `Rover = 37`.
    Rover,
}

impl VehicleType {
    /// `VehicleTypeString[vehicleType]`. `// VehicleType.py:8-12`
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::Plane => "ArduPlane",
            Self::Copter => "ArduCopter",
            Self::Rover => "ArduRover",
        }
    }

    /// `msg_vehicle_to_vehicle_map`: the first word of a vehicle's firmware line.
    /// `// DataflashLog.py:528-533`
    fn from_message(word: &str) -> Option<Self> {
        match word {
            "ArduCopter" | "APM:Copter" => Some(Self::Copter),
            "ArduPlane" => Some(Self::Plane),
            "ArduRover" => Some(Self::Rover),
            _ => None,
        }
    }
}

/// A dictionary keyed by name, kept in first-seen order. Python 2's dicts iterate in hash
/// order, which differs between the Windows runner and any other build; first-seen order is
/// this port's, and the checks that list a dict's entries say so.
#[derive(Clone, Debug)]
pub(crate) struct Ordered<V> {
    entries: Vec<(String, V)>,
    index: HashMap<String, usize>,
}

impl<V> Default for Ordered<V> {
    fn default() -> Self {
        Self {
            entries: Vec::new(),
            index: HashMap::new(),
        }
    }
}

impl<V> Ordered<V> {
    /// The entry called `key`.
    pub(crate) fn get(&self, key: &str) -> Option<&V> {
        self.index
            .get(key)
            .and_then(|&i| self.entries.get(i))
            .map(|(_, v)| v)
    }

    /// The entry called `key`, to change.
    pub(crate) fn get_mut(&mut self, key: &str) -> Option<&mut V> {
        let i = *self.index.get(key)?;
        self.entries.get_mut(i).map(|(_, v)| v)
    }

    /// `key in dict`.
    pub(crate) fn contains_key(&self, key: &str) -> bool {
        self.index.contains_key(key)
    }

    /// `dict[key] = value`: replaced in place, or added at the end.
    pub(crate) fn insert(&mut self, key: &str, value: V) {
        if let Some(slot) = self.get_mut(key) {
            *slot = value;
        } else {
            self.index.insert(key.to_owned(), self.entries.len());
            self.entries.push((key.to_owned(), value));
        }
    }

    /// The entries, first seen first.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&str, &V)> {
        self.entries.iter().map(|(k, v)| (k.as_str(), v))
    }

    /// `len(dict)`.
    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }
}

/// `Format`: a message type as its `FMT` line declares it - the name, the format characters and
/// the labels. (`msgType` and `msgLen` are kept by the Python and read by nothing.)
/// `// DataflashLog.py:17-25`
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Format {
    /// `name`.
    pub(crate) name: String,
    /// `types`, one character a field.
    pub(crate) types: String,
    /// `labels`, the column names.
    pub(crate) labels: Vec<String>,
}

/// A data line read by its format's class (`Format.to_class`): the labels' values in order,
/// each cast by its format character - a label past the end of the format characters keeps its
/// text - and a label named twice holding the later value for both.
#[derive(Clone, Debug)]
pub(crate) struct Record {
    /// The line's format.
    pub(crate) format: Rc<Format>,
    /// One value a label.
    pub(crate) values: Vec<Value>,
}

impl Record {
    /// `e.<label>`: the value of the last label so named, or `None` for an `AttributeError`.
    pub(crate) fn get(&self, label: &str) -> Option<&Value> {
        let at = self.format.labels.iter().rposition(|l| l == label)?;
        self.values.get(at)
    }

    /// `hasattr(e, label)`.
    pub(crate) fn has(&self, label: &str) -> bool {
        self.format.labels.iter().any(|l| l == label)
    }

    /// `e.<label>` or the `AttributeError` the Python raises.
    fn attr(&self, label: &str) -> PyResult<&Value> {
        self.get(label)
            .ok_or_else(|| PyError::attribute(&format!("Log__{}", self.format.name), label))
    }
}

/// `Channel`: one field's samples, `(line, value)` in line order (`listData`); `dictData` -
/// one value a line, the last written - is a view of the same list.
/// `// DataflashLog.py:204-272`
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Channel {
    /// `listData`.
    pub(crate) list: Vec<(usize, Value)>,
}

impl Channel {
    fn push(&mut self, line: usize, value: Value) {
        self.list.push((line, value));
    }

    /// `dictData.items()`: one value a line, the last written for it.
    pub(crate) fn dict(&self) -> impl Iterator<Item = (usize, &Value)> + '_ {
        self.list
            .iter()
            .enumerate()
            .filter_map(move |(i, (line, value))| {
                let same_next = self.list.get(i + 1).is_some_and(|(next, _)| next == line);
                (!same_next).then_some((*line, value))
            })
    }

    /// `dictData.values()`.
    pub(crate) fn values(&self) -> impl Iterator<Item = &Value> + '_ {
        self.dict().map(|(_, v)| v)
    }

    /// `not channel.dictData`.
    pub(crate) fn dict_is_empty(&self) -> bool {
        self.list.is_empty()
    }

    /// `getSegment(startLine, endLine)`: the samples from `start` to `end` inclusive, as a new
    /// channel (of `dictData` only, in the Python; nothing asks a segment for its list).
    pub(crate) fn segment(&self, start: usize, end: usize) -> Self {
        Self {
            list: self
                .dict()
                .filter(|(line, _)| *line >= start && *line <= end)
                .map(|(line, value)| (line, value.clone()))
                .collect(),
        }
    }

    /// `min()`.
    pub(crate) fn min(&self) -> PyResult<Value> {
        py_min(self.values())
    }

    /// `max()`.
    pub(crate) fn max(&self) -> PyResult<Value> {
        py_max(self.values())
    }

    /// `avg()`: `numpy.mean` of the values.
    pub(crate) fn avg(&self) -> PyResult<f64> {
        np_mean(self.values())
    }

    /// `bisect.bisect_left(listData, (lineNumber, -99999))`: the first sample at or after `line`.
    fn bisect_left(&self, line: usize) -> usize {
        self.list.partition_point(|(l, _)| *l < line)
    }

    /// `getNearestValueFwd(lineNumber)`: `(value, line)` of the first sample at or after `line`.
    pub(crate) fn nearest_fwd(&self, line: usize) -> PyResult<(Value, usize)> {
        self.list
            .get(self.bisect_left(line)..)
            .and_then(|rest| rest.iter().find(|(l, _)| *l >= line))
            .map(|(l, v)| (v.clone(), *l))
            .ok_or_else(|| PyError::other(format!("Error finding nearest value for line {line}")))
    }

    /// `getNearestValueBack(lineNumber)`: `(value, line)` of the last sample at or before
    /// `line`.
    pub(crate) fn nearest_back(&self, line: usize) -> PyResult<(Value, usize)> {
        self.list
            .get(..self.bisect_left(line))
            .and_then(|before| before.iter().rev().find(|(l, _)| *l <= line))
            .map(|(l, v)| (v.clone(), *l))
            .ok_or_else(|| PyError::other(format!("Error finding nearest value for line {line}")))
    }

    /// `getNearestValue(lineNumber, lookForwards)`: forwards then back, or back then forwards.
    pub(crate) fn nearest(&self, line: usize, forwards: bool) -> PyResult<(Value, usize)> {
        if forwards {
            self.nearest_fwd(line).or_else(|_| self.nearest_back(line))
        } else {
            self.nearest_back(line).or_else(|_| self.nearest_fwd(line))
        }
    }

    /// `getIndexOf(lineNumber)`: the index of the first sample at `line`.
    pub(crate) fn index_of(&self, line: usize) -> PyResult<usize> {
        let at = self.bisect_left(line);
        match self.list.get(at) {
            None => Err(PyError::index()),
            Some((l, _)) if *l == line => Ok(at),
            Some(_) => Err(PyError::other(format!(
                "Error finding index for line {line}"
            ))),
        }
    }
}

/// `DataflashLog`: what the analyzer reads a log into. `// DataflashLog.py:408-436`
#[derive(Clone, Debug, Default)]
pub(crate) struct DataflashLog {
    /// `filename`, as the runner was given it.
    pub(crate) filename: String,
    /// `vehicleType`, from the header or the first `MSG` naming one.
    pub(crate) vehicle_type: Option<VehicleType>,
    /// `firmwareVersion`.
    pub(crate) firmware_version: String,
    /// `firmwareHash`.
    pub(crate) firmware_hash: String,
    /// `freeRAM`.
    pub(crate) free_ram: i64,
    /// `hardwareType`.
    pub(crate) hardware_type: String,
    /// `formats`, `FMT` itself among them.
    pub(crate) formats: Ordered<Rc<Format>>,
    /// `parameters`.
    pub(crate) parameters: Ordered<Value>,
    /// `messages`: the `MSG` lines after the firmware's, by line.
    pub(crate) messages: Vec<(usize, String)>,
    /// `modeChanges`: by line, `(mode, value)` - the mode's name or its number as it came, and
    /// `ThrCrs` or `ModeNum`.
    pub(crate) mode_changes: Vec<(usize, (Value, Value))>,
    /// `channels`: a group of channels a message type, a channel a label.
    pub(crate) channels: Ordered<Ordered<Channel>>,
    /// `filesizeKB`.
    pub(crate) filesize_kb: f64,
    /// `durationSecs`.
    pub(crate) duration_secs: i64,
    /// `lineCount`.
    pub(crate) line_count: usize,
    /// `skippedLines`: header lines that were nothing known.
    pub(crate) skipped_lines: i64,
    /// `backpatch_these_modechanges`: the `MODE` lines before the vehicle was known.
    backpatch: Vec<(usize, Record)>,
    /// `frame`, from a `MSG` whose first word has "Frame" in it.
    pub(crate) frame: Option<String>,
}

/// The copter's mode numbers as `handleModeChange` names them. `// DataflashLog.py:547-561`
fn copter_mode_name(number: i64) -> Option<&'static str> {
    Some(match number {
        0 => "STABILIZE",
        1 => "ACRO",
        2 => "ALT_HOLD",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        7 => "CIRCLE",
        9 => "LAND",
        10 => "OF_LOITER",
        11 => "DRIFT",
        13 => "SPORT",
        14 => "FLIP",
        15 => "AUTOTUNE",
        16 => "HYBRID",
        _ => return None,
    })
}

/// `tokens2[0].isdigit()`.
fn is_digits(text: &str) -> bool {
    !text.is_empty() && text.bytes().all(|b| b.is_ascii_digit())
}

/// `text[1:-1]`.
fn inner(text: &str) -> String {
    let mut chars = text.chars();
    chars.next();
    chars.next_back();
    chars.as_str().to_owned()
}

/// `self.modeChanges[lineNumber] = change`, the list kept in line order.
fn set_mode_change(
    changes: &mut Vec<(usize, (Value, Value))>,
    line: usize,
    change: (Value, Value),
) {
    match changes.binary_search_by_key(&line, |(l, _)| *l) {
        Ok(at) => {
            if let Some(slot) = changes.get_mut(at) {
                slot.1 = change;
            }
        }
        Err(at) => changes.insert(at, (line, change)),
    }
}

/// `handleModeChange(lineNumber, e)`: a copter's mode number named from the table, with
/// `ThrCrs` if the line has it and `ModeNum` otherwise; anything the table or the line has not
/// got keeps the mode as it came; a plane or rover takes `Mode` and `ModeNum`; no vehicle is an
/// error. `// DataflashLog.py:544-578`
fn handle_mode_change(
    vehicle: Option<VehicleType>,
    changes: &mut Vec<(usize, (Value, Value))>,
    line: usize,
    record: &Record,
) -> PyResult<()> {
    match vehicle {
        Some(VehicleType::Copter) => {
            let mode = record.attr("Mode")?;
            let other = if record.has("ThrCrs") {
                "ThrCrs"
            } else {
                "ModeNum"
            };
            let value = record.attr(other)?.clone();
            let named = mode.to_int().ok().and_then(copter_mode_name);
            let mode = named.map_or_else(|| mode.clone(), Value::text);
            set_mode_change(changes, line, (mode, value));
            Ok(())
        }
        Some(VehicleType::Plane | VehicleType::Rover) => {
            let mode = record.attr("Mode")?.clone();
            let number = record.attr("ModeNum")?.clone();
            set_mode_change(changes, line, (mode, number));
            Ok(())
        }
        None => Err(PyError::other(format!(
            "Unknown log type for MODE line vehicletype=(None) line=({})",
            record.format.name
        ))),
    }
}

impl DataflashLog {
    /// `DataflashLog(logfile, format="auto", ignoreBadlines)` on a text log: reads it, then the
    /// line count, the size and the GPS duration.
    ///
    /// # Errors
    ///
    /// A line that will not read when `ignore_bad_lines` is off, or a `GPS` channel without a
    /// time label or with one that is not a number - the Python raises out of `read`, and the
    /// runner writes nothing. `// DataflashLog.py:477-523`
    pub(crate) fn read(text: &str, filename: &str, ignore_bad_lines: bool) -> PyResult<Self> {
        let mut log = Self {
            filename: filename.to_owned(),
            ..Self::default()
        };
        let (num_bytes, line_number) = log.read_text(text, ignore_bad_lines)?;
        log.line_count = line_number;
        log.filesize_kb = super::pyval::usize_to_float(num_bytes) / 1024.0;
        if let Some(gps) = log.channels.get("GPS") {
            let label = ["TimeMS", "TimeUS", "Time"]
                .into_iter()
                .find(|l| gps.contains_key(l));
            let channel = label
                .and_then(|l| gps.get(l))
                .ok_or_else(PyError::key_none)?;
            let first = channel
                .list
                .first()
                .ok_or_else(PyError::index)?
                .1
                .to_int()?;
            let last = channel.list.last().ok_or_else(PyError::index)?.1.to_int()?;
            let (first, last) = if label == Some("TimeUS") {
                (floor_div(first, 1000)?, floor_div(last, 1000)?)
            } else {
                (first, last)
            };
            log.duration_secs = floor_div(last.wrapping_sub(first), 1000)?;
        }
        Ok(log)
    }

    /// `read_text(f, ignoreBadlines)`: a line at a time, counting each as the Windows runner
    /// does - `len(line) + 1`, a `\r\n` already one byte - and skipping what will not read.
    /// `// DataflashLog.py:636-687`
    fn read_text(&mut self, text: &str, ignore_bad_lines: bool) -> PyResult<(usize, usize)> {
        self.formats.insert(
            "FMT",
            Rc::new(Format {
                name: "FMT".to_owned(),
                types: String::new(),
                labels: Vec::new(),
            }),
        );
        let mut line_number = 0usize;
        let mut num_bytes = 0usize;
        for piece in text.split_inclusive('\n') {
            line_number += 1;
            let (content, had_newline) = piece
                .strip_suffix('\n')
                .map_or((piece, false), |c| (c, true));
            let py_len = if had_newline && content.ends_with('\r') {
                content.len()
            } else {
                content.len() + usize::from(had_newline)
            };
            num_bytes += py_len + 1;
            let line = content.trim_matches(['\n', '\r']);
            if let Err(bad) = self.read_line(line_number, line, ignore_bad_lines) {
                // "BAD LINE: ..." on stderr, which Mission Planner only logs.
                if !ignore_bad_lines {
                    return Err(PyError::other(format!(
                        "Error parsing line {line_number} of log file {} - {}",
                        self.filename, bad.text
                    )));
                }
            }
        }
        Ok((num_bytes, line_number))
    }

    /// One line: the header lines first, then a `FMT`, then a data line by its format. An error
    /// is the Python's exception, which makes the line a "BAD LINE".
    fn read_line(
        &mut self,
        line_number: usize,
        line: &str,
        ignore_bad_lines: bool,
    ) -> PyResult<()> {
        let tokens: Vec<&str> = line.split(", ").collect();
        if line == " Ready to drive." || line == " Ready to FLY." {
            return Ok(());
        }
        if line == "----------------------------------------" {
            return Err(PyError::other(
                "Log file seems to be in the older format (prior to self-describing logs), which isn't supported",
            ));
        }
        if tokens.len() == 1 {
            let tokens2: Vec<&str> = line.split(' ').collect();
            let first = tokens2.first().copied().unwrap_or("");
            if line.is_empty() {
                // nothing
            } else if tokens2.len() == 1 && is_digits(first) {
                // the log index
            } else if tokens2.len() == 3 && first == "Free" && tokens2.get(1) == Some(&"RAM:") {
                let text = tokens2.get(2).copied().unwrap_or("");
                self.free_ram = Value::text(text).to_int()?;
            } else if ["APM", "PX4", "MPNG"].contains(&first) {
                self.hardware_type = line.to_owned();
            } else if (tokens2.len() == 2 || tokens2.len() == 3)
                && tokens2
                    .get(1)
                    .and_then(|t| t.chars().next())
                    .ok_or_else(|| PyError::other("string index out of range"))?
                    .eq_ignore_ascii_case(&'v')
            {
                // e.g. "ArduCopter V3.1 (5c6503e2)"
                if let Some(vehicle) = VehicleType::from_message(first) {
                    self.vehicle_type = Some(vehicle);
                }
                self.firmware_version = tokens2.get(1).copied().unwrap_or("").to_owned();
                if let Some(hash) = tokens2.get(2).filter(|_| tokens2.len() == 3) {
                    self.firmware_hash = inner(hash);
                }
            } else if ignore_bad_lines {
                // "Error parsing line N of log file ... (skipping line)"
                self.skipped_lines += 1;
            } else {
                return Err(PyError::other(""));
            }
            return Ok(());
        }
        let name = tokens.first().copied().unwrap_or("");
        if name == "FMT" {
            // `Format(*tokens[1:])`: five arguments, no more and no fewer.
            let (Some(fmt_name), Some(types), Some(labels), 6) = (
                tokens.get(3).copied(),
                tokens.get(4).copied(),
                tokens.get(5).copied(),
                tokens.len(),
            ) else {
                return Err(PyError::other(format!(
                    "__init__() takes exactly 6 arguments ({} given)",
                    tokens.len()
                )));
            };
            if !self.formats.contains_key(fmt_name) {
                self.formats.insert(
                    fmt_name,
                    Rc::new(Format {
                        name: fmt_name.to_owned(),
                        types: types.to_owned(),
                        labels: labels.split(',').map(str::to_owned).collect(),
                    }),
                );
            }
            return Ok(());
        }
        let Some(format) = self.formats.get(name).map(Rc::clone) else {
            return Err(PyError::other(format!("Unknown Format {name}")));
        };
        let fields = tokens.get(1..).unwrap_or(&[]);
        if fields.len() != format.labels.len() {
            return Err(PyError::other("Invalid Length"));
        }
        let values = fields
            .iter()
            .enumerate()
            .map(|(i, text)| {
                format
                    .types
                    .chars()
                    .nth(i)
                    .map_or_else(|| Value::text(text), |kind| Value::trycast(text, kind))
            })
            .collect();
        self.process(line_number, Record { format, values })
    }

    /// `process(lineNumber, e)` for a data line: a parameter, a message, a mode change, or
    /// samples into the channels. `// DataflashLog.py:587-633`
    fn process(&mut self, line: usize, record: Record) -> PyResult<()> {
        match record.format.name.as_str() {
            "PARM" => {
                let name = py_str(record.attr("Name")?);
                let value = record.attr("Value")?.clone();
                self.parameters.insert(&name, value);
            }
            "MSG" => {
                let message = match record.attr("Message")? {
                    Value::Str(text) => text.to_string(),
                    other => return Err(PyError::attribute(other.type_name(), "split")),
                };
                let tokens: Vec<&str> = message.split(' ').collect();
                let first = tokens.first().copied().unwrap_or("");
                if self.frame.as_deref().is_none_or(str::is_empty) && first.contains("Frame") {
                    self.frame = Some(tokens.get(1).ok_or_else(PyError::index)?.to_string());
                }
                if self.vehicle_type.is_none() {
                    if let Some(vehicle) = VehicleType::from_message(first) {
                        self.vehicle_type = Some(vehicle);
                    }
                    self.back_patch_mode_changes()?;
                    self.firmware_version = tokens.get(1).ok_or_else(PyError::index)?.to_string();
                    if let Some(hash) = tokens.get(2).filter(|_| tokens.len() == 3) {
                        self.firmware_hash = inner(hash);
                    }
                } else {
                    self.messages.push((line, message));
                }
            }
            "MODE" => {
                if self.vehicle_type.is_none() {
                    self.backpatch.push((line, record));
                } else {
                    handle_mode_change(self.vehicle_type, &mut self.mode_changes, line, &record)?;
                }
            }
            name => {
                // first time seeing this type of log line, create the channel storage
                if !self.channels.contains_key(name) {
                    let mut group = Ordered::default();
                    for label in &record.format.labels {
                        if !group.contains_key(label) {
                            group.insert(label, Channel::default());
                        }
                    }
                    self.channels.insert(name, group);
                }
                let group = self
                    .channels
                    .get_mut(name)
                    .ok_or_else(|| PyError::key(name))?;
                for label in &record.format.labels {
                    let value = record.attr(label)?.clone();
                    group
                        .get_mut(label)
                        .ok_or_else(|| PyError::key(label))?
                        .push(line, value);
                }
            }
        }
        Ok(())
    }

    /// `backPatchModeChanges`: the `MODE` lines kept while no vehicle was known, each handled
    /// now; the list is never emptied, and is only replayed while the vehicle stays unknown.
    /// `// DataflashLog.py:580-582`
    fn back_patch_mode_changes(&mut self) -> PyResult<()> {
        for (line, record) in &self.backpatch {
            handle_mode_change(self.vehicle_type, &mut self.mode_changes, *line, record)?;
        }
        Ok(())
    }

    /// `getCopterType()`: `tradheli`, `quad`, `hex` or `octo` from the `MOT` format's labels;
    /// `None` when there is no such name, or the vehicle is not a copter (the Python's `None`
    /// or `""`, both false). `// DataflashLog.py:442-458`
    pub(crate) fn copter_type(&self) -> Option<&'static str> {
        if self.vehicle_type != Some(VehicleType::Copter) {
            return None;
        }
        let labels = self
            .formats
            .get("MOT")
            .map(|f| f.labels.as_slice())
            .unwrap_or(&[]);
        if labels.iter().any(|l| l == "GGain") {
            Some("tradheli")
        } else {
            match labels.len() {
                4 => Some("quad"),
                6 => Some("hex"),
                8 => Some("octo"),
                _ => None,
            }
        }
    }

    /// `num_motor_channels()`: by the frame's name; a frame the table has not got is a
    /// `KeyError`. `// DataflashLog.py:460-475`
    pub(crate) fn num_motor_channels(&self) -> PyResult<i64> {
        let frame = self.frame.as_deref().unwrap_or("");
        Ok(match frame {
            "QUAD" => 4,
            "HEXA" | "Y6" => 6,
            "OCTA" | "OCTA_QUAD" => 8,
            "TRI" => 3,
            "SINGLE" | "TAILSITTER" => 1,
            "COAX" => 2,
            "DODECA_HEXA" => 12,
            _ => return Err(PyError::key(frame)),
        })
    }

    /// `logdata.channels[group][label]`, with the `KeyError` for either.
    pub(crate) fn channel(&self, group: &str, label: &str) -> PyResult<&Channel> {
        self.channels
            .get(group)
            .ok_or_else(|| PyError::key(group))?
            .get(label)
            .ok_or_else(|| PyError::key(label))
    }

    /// `logdata.channels[group]`, with its `KeyError`.
    pub(crate) fn group(&self, group: &str) -> PyResult<&Ordered<Channel>> {
        self.channels.get(group).ok_or_else(|| PyError::key(group))
    }

    /// `logdata.parameters[name]`, with its `KeyError`.
    pub(crate) fn parameter(&self, name: &str) -> PyResult<&Value> {
        self.parameters.get(name).ok_or_else(|| PyError::key(name))
    }
}

/// `DataflashLogHelper.getTimeAtLine(logdata, lineNumber)`: the GPS time at the first GPS line
/// at or after `line`, or the latest GPS time when there is none.
/// `// DataflashLog.py:333-353`
pub(crate) fn time_at_line(log: &DataflashLog, line: usize) -> PyResult<Value> {
    let gps = log
        .channels
        .get("GPS")
        .ok_or_else(|| PyError::other("no GPS log data found"))?;
    let label = ["TimeMS", "Time", "TimeUS"]
        .into_iter()
        .find(|l| gps.contains_key(l))
        .ok_or_else(|| PyError::other("Unable to get time label"))?;
    let channel = gps.get(label).ok_or_else(|| PyError::key(label))?;
    if let Some((at_line, _)) = channel.dict().find(|(l, _)| *l >= line)
        && at_line <= log.line_count
    {
        // `dictData[lineNumber]`: the value last written for that line.
        if let Some(value) = channel.dict().find(|(l, _)| *l == at_line).map(|(_, v)| v) {
            return Ok(value.clone());
        }
    }
    // "didn't find GPS data for N - using maxtime" on stderr.
    channel.max()
}

/// `DataflashLogHelper.findLoiterChunks(logdata, minLengthSeconds, noRCInputs)`: the stretches
/// of the log in a mode named exactly `LOITER`, `(startLine, endLine)`, each longer than
/// `min_length_seconds` by the GPS clock in milliseconds (a `TimeUS` clock is not scaled),
/// longest first. `noRCInputs` is not implemented by the Python. `// DataflashLog.py:355-386`
pub(crate) fn find_loiter_chunks(
    log: &DataflashLog,
    min_length_seconds: f64,
) -> PyResult<Vec<(usize, usize)>> {
    let mut ordered = log.mode_changes.clone();
    ordered.sort_by_key(|(line, _)| *line);
    let mut chunks = Vec::new();
    let loiter = Value::text("LOITER");
    for (i, (start, (mode, _))) in ordered.iter().enumerate() {
        if !mode.eq(&loiter) {
            continue;
        }
        let end = match ordered.get(i + 1) {
            None => log.line_count,
            Some((next, _)) => next.saturating_sub(1),
        };
        let seconds = time_at_line(log, end)?
            .arith(&time_at_line(log, *start)?, Op::Sub)?
            .arith(&Value::Int(1), Op::Add)?
            .arith(&Value::Float(1000.0), Op::Div)?;
        if seconds.gt(&Value::Float(min_length_seconds)) {
            chunks.push((*start, end));
        }
    }
    chunks.sort_by_key(|(start, end)| std::cmp::Reverse(end.saturating_sub(*start)));
    Ok(chunks)
}

/// `DataflashLogHelper.isLogEmpty(logdata)`: "Throttle never above 20%" when `CTUN.ThrOut`
/// never passes 200 on a copter or 20 on anything else - or `CTUN.ThO`, which runs 0 to 1,
/// never passes 0.2. `// DataflashLog.py:388-405`
pub(crate) fn is_log_empty(log: &DataflashLog) -> PyResult<Option<&'static str>> {
    let mut threshold = if log.vehicle_type == Some(VehicleType::Copter) {
        Value::Int(200)
    } else {
        Value::Int(20)
    };
    if let Some(ctun) = log.channels.get("CTUN") {
        let max_throttle = match ctun.get("ThrOut") {
            Some(channel) => channel.max()?,
            None => {
                let max = ctun.get("ThO").ok_or_else(|| PyError::key("ThO"))?.max()?;
                threshold = Value::Float(0.2);
                max
            }
        };
        if max_throttle.lt(&threshold) {
            return Ok(Some("Throttle never above 20%"));
        }
    }
    Ok(None)
}

/// `LogIterator`: walks the log a line at a time, an index into each channel group's samples
/// moving to the first sample at or after the current line. `// DataflashLog.py:274-327`
pub(crate) struct LogIterator<'a> {
    log: &'a DataflashLog,
    /// `currentLine`.
    pub(crate) current_line: usize,
    /// `iterators`: a group, its index into the group's first channel and that sample's line.
    iterators: Vec<(&'a str, usize, usize)>,
}

impl<'a> LogIterator<'a> {
    /// `LogIterator(logdata, lineNumber)`: every format that has a channel group, jumped to
    /// `line`.
    pub(crate) fn new(log: &'a DataflashLog, line: usize) -> PyResult<Self> {
        let mut iterators = Vec::new();
        for (name, _) in log.formats.iter() {
            if log.channels.contains_key(name) {
                let channel = Self::first_channel(log, name)?;
                let (_, at_line) = channel.nearest(line, true)?;
                iterators.push((name, channel.index_of(at_line)?, at_line));
            }
        }
        Ok(Self {
            log,
            current_line: line,
            iterators,
        })
    }

    /// The group's channel of its format's first label.
    fn first_channel(log: &'a DataflashLog, group: &str) -> PyResult<&'a Channel> {
        let format = log.formats.get(group).ok_or_else(|| PyError::key(group))?;
        let label = format.labels.first().ok_or_else(PyError::index)?;
        log.channel(group, label)
    }

    /// `lit[group][label]`: the sample of `label` at the group's current index.
    pub(crate) fn get(&self, group: &str, label: &str) -> PyResult<Value> {
        let (_, index, _) = self
            .iterators
            .iter()
            .find(|(name, _, _)| *name == group)
            .ok_or_else(|| PyError::key(group))?;
        self.log
            .channel(group, label)?
            .list
            .get(*index)
            .map(|(_, v)| v.clone())
            .ok_or_else(PyError::index)
    }

    /// `next(lit)`: the next line, each group's index moving on by one when the line has passed
    /// its sample and there is a sample after it.
    pub(crate) fn advance(&mut self) -> PyResult<()> {
        self.current_line += 1;
        if self.current_line > self.log.line_count {
            return Ok(());
        }
        for (name, index, at_line) in &mut self.iterators {
            let channel = Self::first_channel(self.log, name)?;
            if self.current_line > *at_line && *index + 1 < channel.list.len() {
                *index += 1;
                *at_line = channel.list.get(*index).map_or(*at_line, |(l, _)| *l);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n\
                        FMT, 32, 35, PARM, QNff, TimeUS,Name,Value,Default\n\
                        FMT, 123, 14, MODE, QMBB, TimeUS,Mode,ModeNum,Rsn\n\
                        FMT, 95, 20, GPS, QBBIf, TimeUS,I,Status,GMS,HDop\n\
                        FMT, 10, 10, MSG, QZ, TimeUS,Message\n";

    fn read(text: &str) -> DataflashLog {
        DataflashLog::read(text, "t.log", true).unwrap()
    }

    #[test]
    fn the_header_lines_are_read_as_the_python_reads_them() {
        let text = "1\r\n\r\nArduCopter V3.0.1 (5c6503e2)\r\nFree RAM: 1331\r\nAPM 2\r\n\
                    junk line here\r\n Ready to FLY.\r\n";
        let log = read(text);
        assert_eq!(log.vehicle_type, Some(VehicleType::Copter));
        assert_eq!(log.firmware_version, "V3.0.1");
        assert_eq!(log.firmware_hash, "5c6503e2");
        assert_eq!(log.free_ram, 1331);
        assert_eq!(log.hardware_type, "APM 2");
        assert_eq!(log.skipped_lines, 1);
        assert_eq!(log.line_count, 7);
        // Each line a byte longer than its text, the "\r\n" one byte: 1+2, 0+2, 28+2, ...
        let bytes = [1, 0, 28, 14, 5, 14, 14]
            .iter()
            .map(|n| n + 2)
            .sum::<usize>();
        assert!(
            (log.filesize_kb - super::super::pyval::usize_to_float(bytes) / 1024.0).abs() < 1e-12
        );
    }

    #[test]
    fn formats_parameters_and_channels_are_filled_and_bad_lines_skipped() {
        let text = format!(
            "{HEAD}PARM, 1, RC3_MIN, 1100, 0\nGPS, 100, 0, 3, 5, 1.5\nGPS, 200, 0, 3, 7, nan\n\
             GPS, 300, 0\nXYZ, 1, 2\nFMT, 95, 20, GPS, QB, TimeUS,I\nGPS, 400, 0, 3, 9, 0.9\n"
        );
        let log = read(&text);
        assert_eq!(log.formats.len(), 5);
        assert_eq!(log.parameters.get("RC3_MIN"), Some(&Value::Float(1100.0)));
        let hdop = log.channel("GPS", "HDop").unwrap();
        assert_eq!(hdop.list.len(), 3);
        assert_eq!(hdop.list.first(), Some(&(7, Value::Float(1.5))));
        assert!(
            hdop.list
                .get(1)
                .is_some_and(|(line, v)| *line == 8 && v.is_nan())
        );
        assert_eq!(hdop.list.get(2), Some(&(12, Value::Float(0.9))));
        assert_eq!(
            log.channel("GPS", "GMS").unwrap().max().unwrap(),
            Value::Int(9)
        );
        assert_eq!(log.skipped_lines, 0);
        assert_eq!(log.line_count, 12);
        // TimeUS: microseconds to milliseconds, then to whole seconds, floored.
        assert_eq!(log.duration_secs, 0);
        assert!(
            log.channel("GPS", "Nope")
                .is_err_and(|e| e.key_error && e.text == "'Nope'")
        );
        assert_eq!(log.channel("ZZZ", "x").unwrap_err().text, "'ZZZ'");
    }

    #[test]
    fn the_vehicle_comes_from_the_first_known_msg_and_modes_are_patched_in() {
        let text = format!(
            "{HEAD}MODE, 1, Loiter, 5, 1\nMSG, 2, Frame: QUAD/X\nMODE, 3, 8, 8, 1\n\
             MSG, 4, ArduCopter V4.5.7 (1a2b3c4d)\nMODE, 5, 3, 3, 2\nMSG, 6, hello world\n"
        );
        let log = read(&text);
        assert_eq!(log.frame.as_deref(), Some("QUAD/X"));
        // The Frame line was not a vehicle: it set the firmware version until the next MSG did.
        assert_eq!(log.vehicle_type, Some(VehicleType::Copter));
        assert_eq!(log.firmware_version, "V4.5.7");
        assert_eq!(log.firmware_hash, "1a2b3c4d");
        assert_eq!(log.messages, vec![(11, "hello world".to_owned())]);
        assert_eq!(
            log.mode_changes,
            vec![
                (6, (Value::text("Loiter"), Value::Int(5))),
                (8, (Value::Int(8), Value::Int(8))),
                (10, (Value::text("AUTO"), Value::Int(3))),
            ]
        );
        assert_eq!(log.num_motor_channels().unwrap_err().text, "'QUAD/X'");
        assert_eq!(log.copter_type(), None);
    }

    #[test]
    fn a_frame_message_before_any_vehicle_fails_when_modes_wait() {
        // The Frame MSG tries to patch the MODE line in with no vehicle, raises, and is skipped;
        // then the real firmware line patches it in.
        let text = format!(
            "{HEAD}MODE, 1, Loiter, 5, 1\nMSG, 2, Frame: QUAD\nMSG, 4, ArduPlane V4.5.7 (1a2b3c4d)\n"
        );
        let log = read(&text);
        assert_eq!(log.frame.as_deref(), Some("QUAD"));
        assert_eq!(log.vehicle_type, Some(VehicleType::Plane));
        assert_eq!(log.firmware_version, "V4.5.7");
        assert_eq!(
            log.mode_changes,
            vec![(6, (Value::text("Loiter"), Value::Int(5)))]
        );
        assert_eq!(log.num_motor_channels().unwrap(), 4);
    }

    #[test]
    fn nearest_values_and_segments_follow_bisect() {
        let channel = Channel {
            list: vec![
                (3, Value::Int(1)),
                (7, Value::Int(2)),
                (7, Value::Int(3)),
                (9, Value::Int(4)),
            ],
        };
        assert_eq!(channel.nearest_fwd(4).unwrap(), (Value::Int(2), 7));
        assert_eq!(channel.nearest_back(8).unwrap(), (Value::Int(3), 7));
        assert_eq!(channel.nearest(1, false).unwrap(), (Value::Int(1), 3));
        assert_eq!(channel.nearest(10, true).unwrap(), (Value::Int(4), 9));
        assert_eq!(channel.index_of(7).unwrap(), 1);
        assert_eq!(
            channel.index_of(8).unwrap_err().text,
            "Error finding index for line 8"
        );
        assert_eq!(
            channel.index_of(10).unwrap_err().text,
            "list index out of range"
        );
        let dict: Vec<_> = channel.dict().map(|(l, v)| (l, v.clone())).collect();
        assert_eq!(
            dict,
            vec![(3, Value::Int(1)), (7, Value::Int(3)), (9, Value::Int(4))]
        );
        assert_eq!(channel.segment(4, 8).list, vec![(7, Value::Int(3))]);
        assert_eq!(channel.min().unwrap(), Value::Int(1));
        assert_eq!(channel.max().unwrap(), Value::Int(4));
        assert!((channel.avg().unwrap() - 8.0 / 3.0).abs() < 1e-12);
        assert!(Channel::default().nearest(1, true).is_err());
    }

    #[test]
    fn loiter_chunks_and_emptiness_are_the_helpers() {
        let text = format!(
            "{HEAD}FMT, 1, 1, CTUN, Qf, TimeUS,ThO\nMSG, 1, ArduCopter V4.5.7 (1a2b3c4d)\n\
             GPS, 1000000, 0, 3, 1, 1.5\nMODE, 2, 5, 5, 1\nGPS, 2000000, 0, 3, 5, 1.5\n\
             GPS, 2500000, 0, 3, 7, 1.5\nMODE, 3, 0, 0, 1\nMODE, 4, 5, 5, 1\n\
             GPS, 3000000, 0, 3, 50, 1.5\nCTUN, 5, 0.1\nGPS, 9000000, 0, 3, 60, 1.5\n"
        );
        let log = read(&text);
        // Two LOITER stretches, lines 9-11 and 13-16, the longer first; each is timed by the
        // first GPS line at or after its ends, and the TimeUS clock is taken as milliseconds.
        assert_eq!(
            find_loiter_chunks(&log, 10.0).unwrap(),
            vec![(13, 16), (9, 11)]
        );
        assert!(find_loiter_chunks(&log, 1000.0).unwrap() == vec![(13, 16)]);
        assert_eq!(time_at_line(&log, 13).unwrap(), Value::Int(3_000_000));
        assert_eq!(time_at_line(&log, 20).unwrap(), Value::Int(9_000_000));
        assert_eq!(
            is_log_empty(&log).unwrap(),
            Some("Throttle never above 20%")
        );
        let mut lit = LogIterator::new(&log, 9).unwrap();
        assert_eq!(lit.get("GPS", "GMS").unwrap(), Value::Int(5));
        for _ in 9..13 {
            lit.advance().unwrap();
        }
        assert_eq!(lit.current_line, 13);
        assert_eq!(lit.get("GPS", "GMS").unwrap(), Value::Int(50));
        assert_eq!(lit.get("CTUN", "ThO").unwrap(), Value::Float(0.1));
        assert_eq!(lit.get("ZZZ", "x").unwrap_err().text, "'ZZZ'");
        let no_gps = read(&format!(
            "{HEAD}MSG, 1, ArduCopter V4.5.7 (1a2b3c4d)\nMODE, 2, 5, 5, 1\n"
        ));
        assert_eq!(
            find_loiter_chunks(&no_gps, 10.0).unwrap_err().text,
            "no GPS log data found"
        );
    }

    /// `UnitTest.py`'s assertions over the analyzer's example log (`examples/
    /// robert_lefebvre_octo_PM.log` in ArduPilot's tree, a CRLF file): what the Python read of
    /// it. Its `int(filesizeKB) == 307` counted three bytes a line end, a CRLF file read on
    /// Linux; read as the Windows runner reads it, the log is 302 KB, as the C# tree's
    /// `example_output.xml` says.
    #[test]
    fn the_example_log_reads_as_unittest_py_says() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/dataflash/loganalyzer/robert_lefebvre_octo_PM.log");
        let text = mp_os::fs::read_to_string(path).unwrap();
        let log = DataflashLog::read(&text, "examples/robert_lefebvre_octo_PM.log", false).unwrap();
        assert_eq!(log.vehicle_type, Some(VehicleType::Copter));
        assert_eq!(log.firmware_version, "V3.0.1");
        assert_eq!(log.firmware_hash, "5c6503e2");
        assert_eq!(log.free_ram, 1331);
        assert_eq!(log.hardware_type, "APM 2");
        assert_eq!(log.formats.len(), 27);
        assert_eq!(
            log.formats.get("GPS").unwrap().labels,
            [
                "Status", "Time", "NSats", "HDop", "Lat", "Lng", "RelAlt", "Alt", "Spd", "GCrs"
            ]
        );
        assert_eq!(
            log.formats.get("ATT").unwrap().labels,
            [
                "RollIn", "Roll", "PitchIn", "Pitch", "YawIn", "Yaw", "NavYaw"
            ]
        );
        assert_eq!(log.parameters.len(), 272);
        assert_eq!(
            log.parameters.get("INS_ACCSCAL_X"),
            Some(&Value::Float(0.992_788))
        );
        assert_eq!(
            log.parameters.get("COMPASS_OFS_X"),
            Some(&Value::Float(-20.490_345))
        );
        assert!(log.messages.is_empty());
        assert_eq!(
            log.mode_changes,
            vec![
                (644, (Value::text("ALT_HOLD"), Value::Int(269))),
                (2204, (Value::text("LOITER"), Value::Int(269))),
                (4404, (Value::text("ALT_HOLD"), Value::Int(269))),
                (4594, (Value::text("STABILIZE"), Value::Int(269))),
            ]
        );
        let nsats = log.channel("GPS", "NSats").unwrap();
        assert_eq!(nsats.min().unwrap(), Value::Int(6));
        assert_eq!(nsats.max().unwrap(), Value::Int(8));
        let hdop = &log.channel("GPS", "HDop").unwrap().list;
        assert_eq!(hdop.first(), Some(&(552, Value::Float(4.68))));
        assert_eq!(hdop.get(44), Some(&(768, Value::Float(4.67))));
        assert_eq!(hdop.get(157), Some(&(1288, Value::Float(2.28))));
        let throut = &log.channel("CTUN", "ThrOut").unwrap().list;
        assert_eq!(throut.get(5), Some(&(321, Value::Int(139))));
        assert_eq!(throut.get(45), Some(&(409, Value::Int(242))));
        assert_eq!(throut.get(125), Some(&(589, Value::Int(266))));
        let crate_ = &log.channel("CTUN", "CRate").unwrap().list;
        assert_eq!(crate_.get(3), Some(&(317, Value::Int(35))));
        assert_eq!(crate_.get(51), Some(&(421, Value::Int(31))));
        assert_eq!(crate_.get(115), Some(&(563, Value::Int(-8))));
        assert_eq!(log.line_count, 4750);
        assert_eq!(log.duration_secs, 155);
        assert_eq!(log.filesize_kb, 302.754_882_812_5);
        assert_eq!(log.copter_type(), Some("octo"));
    }

    #[test]
    fn a_gps_channel_without_a_time_is_a_failed_read() {
        let text = "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n\
                    FMT, 95, 20, GPS, Bf, Status,HDop\nGPS, 3, 1.5\n";
        let failed = DataflashLog::read(text, "t.log", true).unwrap_err();
        assert_eq!(failed.text, "None");
        let strict = DataflashLog::read("FMT, 1\n", "t.log", false).unwrap_err();
        assert!(
            strict
                .text
                .starts_with("Error parsing line 1 of log file t.log - ")
        );
    }
}
