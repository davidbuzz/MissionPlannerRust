//! `Plugins/AnonymizeBinlogPlugin.cs`, "Anonymize Binlog", one of the C#'s four real plugins:
//! "Anonymize Bin Log..." on the flight screen's map menu moves every coordinate in an ArduPilot
//! `.bin` log by a latitude and longitude offset, so the log can be shared without saying where
//! it was flown.
//!
//! Ported whole, with two differences the world makes:
//!
//! * the C# asks where to save before it asks for the offsets and writes the file itself; here
//!   the save dialog comes after the offsets, since `save-file` takes the anonymised bytes and
//!   writes them where the user chooses;
//! * a blank offset is random in the C# (`new Random()`, seeded from the clock). A plugin has no
//!   clock and no entropy of its own, so the seed is a hash of the log's own bytes: different for
//!   every log, and not recoverable from the anonymised one, whose bytes differ.

use std::sync::{Mutex, PoisonError};

use mp_plugins::Guest;
use mp_plugins::host::{self, MapMenu, MessageButtons};

/// The menu entry's id.
static ENTRY: Mutex<Option<u32>> = Mutex::new(None);

const HEAD1: u8 = 0xA3;
const HEAD2: u8 = 0x95;
const FMT_MSG_ID: u8 = 128;
const FMT_MSG_LEN: usize = 89;
const UNIT_LATITUDE: u8 = b'D';
const UNIT_LONGITUDE: u8 = b'U';

/// The columns taken for a latitude when the log has no `FMTU`.
/// `// C#: Plugins/AnonymizeBinlogPlugin.cs:134-138`
const FALLBACK_LAT_NAMES: &[&str] = &[
    "lat", "hlat", "dlat", "oalat", "dlt", "olt", "elat", "olat", "clat", "trlat", "wplat", "rlat",
    "tp_lat",
];

/// And for a longitude. `// C#: Plugins/AnonymizeBinlogPlugin.cs:140-144`
const FALLBACK_LNG_NAMES: &[&str] = &[
    "lng", "lon", "hlon", "hlng", "dlng", "oalng", "dlg", "olg", "elng", "olng", "clng", "trlng",
    "wplng", "rlng", "tp_lng",
];

/// `DfFormatSize`: a format character's size in bytes. `// C#: Plugins/AnonymizeBinlogPlugin.cs:146-153`
const fn format_size(fc: u8) -> Option<usize> {
    Some(match fc {
        b'b' | b'B' | b'M' => 1,
        b'c' | b'C' | b'h' | b'H' => 2,
        b'e' | b'E' | b'f' | b'i' | b'I' | b'L' | b'n' => 4,
        b'd' | b'q' | b'Q' => 8,
        b'N' => 16,
        b'a' | b'Z' => 64,
        b'A' => 128,
        _ => return None,
    })
}

/// `StringFormats`. `// C#: Plugins/AnonymizeBinlogPlugin.cs:155`
const fn is_string_format(fc: u8) -> bool {
    matches!(fc, b'a' | b'n' | b'N' | b'Z' | b'A')
}

/// `FmtDef`.
#[derive(Debug, Clone)]
struct FmtDef {
    name: String,
    length: usize,
    format: Vec<u8>,
    columns: Vec<String>,
}

/// `CoordPatch`: where a coordinate is in its message.
#[derive(Debug, Clone, Copy)]
struct CoordPatch {
    offset: usize,
    fc: u8,
    size: usize,
    lat: bool,
}

/// The format definitions, keyed by type, in the order they were first seen (a C#
/// `Dictionary`'s order).
#[derive(Debug, Default)]
struct Formats {
    defs: Vec<(u8, FmtDef)>,
}

impl Formats {
    fn get(&self, id: u8) -> Option<&FmtDef> {
        self.defs
            .iter()
            .find(|(key, _)| *key == id)
            .map(|(_, def)| def)
    }

    fn set(&mut self, id: u8, def: FmtDef) {
        match self.defs.iter_mut().find(|(key, _)| *key == id) {
            Some(slot) => slot.1 = def,
            None => self.defs.push((id, def)),
        }
    }
}

/// `ReadAscii`: up to `length` bytes, stopping at a NUL. `// C#: Plugins/AnonymizeBinlogPlugin.cs:488-495`
fn read_ascii(data: &[u8], offset: usize, length: usize) -> String {
    let end = (offset + length).min(data.len());
    let bytes = data.get(offset..end).unwrap_or_default();
    let bytes = bytes
        .iter()
        .position(|&b| b == 0)
        .map_or(bytes, |nul| bytes.get(..nul).unwrap_or_default());
    bytes.iter().map(|&b| char::from(b)).collect()
}

/// The walk every pass makes: a message at each header, its length from its format.
fn at(data: &[u8], pos: usize, offset: usize) -> u8 {
    data.get(pos + offset).copied().unwrap_or(0)
}

/// `ParseFormats`: every `FMT` message. `// C#: Plugins/AnonymizeBinlogPlugin.cs:187-227`
fn parse_formats(data: &[u8]) -> Formats {
    let mut formats = Formats::default();
    let mut pos = 0;
    let end = data.len().saturating_sub(2);
    while pos < end {
        if at(data, pos, 0) == HEAD1 && at(data, pos, 1) == HEAD2 {
            let id = at(data, pos, 2);
            if id == FMT_MSG_ID {
                if pos + FMT_MSG_LEN <= data.len() {
                    let type_id = at(data, pos, 3);
                    let length = usize::from(at(data, pos, 4));
                    let name = read_ascii(data, pos + 5, 4);
                    let format = read_ascii(data, pos + 9, 16).into_bytes();
                    let columns = read_ascii(data, pos + 25, 64)
                        .split(',')
                        .map(str::to_owned)
                        .collect();
                    formats.set(
                        type_id,
                        FmtDef {
                            name,
                            length,
                            format,
                            columns,
                        },
                    );
                }
                pos += FMT_MSG_LEN;
            } else if let Some(def) = formats.get(id) {
                // A zero length would stand still; the C# spins there, this moves on.
                pos += def.length.max(1);
            } else {
                pos += 1;
            }
        } else {
            pos += 1;
        }
    }
    formats
}

/// `ParseFmtu`: each type's unit characters, from the `FMTU` messages.
/// `// C#: Plugins/AnonymizeBinlogPlugin.cs:229-277`
fn parse_fmtu(data: &[u8], formats: &Formats) -> Vec<(u8, String)> {
    let Some((fmtu, fmtu_def)) = formats.defs.iter().find(|(_, def)| def.name == "FMTU") else {
        return Vec::new();
    };
    let mut map: Vec<(u8, String)> = Vec::new();
    let mut pos = 0;
    let end = data.len().saturating_sub(2);
    while pos < end {
        if at(data, pos, 0) == HEAD1 && at(data, pos, 1) == HEAD2 {
            let id = at(data, pos, 2);
            if id == *fmtu && pos + fmtu_def.length <= data.len() {
                let format_type = at(data, pos, 11);
                let units = read_ascii(data, pos + 12, 16);
                match map.iter_mut().find(|(key, _)| *key == format_type) {
                    Some(slot) => slot.1 = units,
                    None => map.push((format_type, units)),
                }
                pos += fmtu_def.length.max(1);
            } else if let Some(def) = formats.get(id) {
                pos += def.length.max(1);
            } else {
                pos += 1;
            }
        } else {
            pos += 1;
        }
    }
    map
}

/// `ComputeFieldOffsets`: each field's offset after the three header bytes, up to the first
/// character the table does not know. `// C#: Plugins/AnonymizeBinlogPlugin.cs:279-291`
fn field_offsets(format: &[u8]) -> Vec<(usize, u8, usize)> {
    let mut offset = 3;
    let mut out = Vec::new();
    for &fc in format {
        let Some(size) = format_size(fc) else { break };
        out.push((offset, fc, size));
        offset += size;
    }
    out
}

/// `IdentifyCoordFields`: by the `FMTU` units when the log has them, else by the column names.
/// `// C#: Plugins/AnonymizeBinlogPlugin.cs:293-360`
fn coord_fields(formats: &Formats, fmtu: &[(u8, String)]) -> Vec<(u8, Vec<CoordPatch>)> {
    let use_fmtu = !fmtu.is_empty();
    let mut out = Vec::new();
    for (type_id, def) in &formats.defs {
        if matches!(def.name.as_str(), "FMT" | "FMTU" | "MULT" | "UNIT") {
            continue;
        }
        let fields = field_offsets(&def.format);
        let count = fields.len().min(def.columns.len());
        let mut patches = Vec::new();
        let units = fmtu
            .iter()
            .find(|(key, _)| key == type_id)
            .map(|(_, units)| units);
        match units {
            Some(units) if use_fmtu => {
                for (unit, &(offset, fc, size)) in units.bytes().zip(&fields).take(count) {
                    let lat = match unit {
                        UNIT_LATITUDE => true,
                        UNIT_LONGITUDE => false,
                        _ => continue,
                    };
                    patches.push(CoordPatch {
                        offset,
                        fc,
                        size,
                        lat,
                    });
                }
            }
            _ => {
                for (column, &(offset, fc, size)) in def.columns.iter().zip(&fields).take(count) {
                    let cl = column.to_lowercase();
                    let lat = if FALLBACK_LAT_NAMES.contains(&cl.as_str()) {
                        true
                    } else if FALLBACK_LNG_NAMES.contains(&cl.as_str()) {
                        false
                    } else if fc == b'L' {
                        if cl.contains("lat") || cl.contains("lt") {
                            true
                        } else if cl.contains("lng") || cl.contains("lon") || cl.contains("lg") {
                            false
                        } else {
                            continue;
                        }
                    } else {
                        continue;
                    };
                    patches.push(CoordPatch {
                        offset,
                        fc,
                        size,
                        lat,
                    });
                }
            }
        }
        if !patches.is_empty() {
            out.push((*type_id, patches));
        }
    }
    out
}

/// The `N` bytes at `offset`.
fn bytes<const N: usize>(data: &[u8], offset: usize) -> Option<[u8; N]> {
    data.get(offset..offset + N)?.try_into().ok()
}

/// `ReadValue`. `// C#: Plugins/AnonymizeBinlogPlugin.cs:362-378`
#[allow(clippy::cast_precision_loss)]
fn read_value(data: &[u8], offset: usize, fc: u8) -> Option<f64> {
    Some(match fc {
        b'b' => f64::from(i8::from_le_bytes(bytes(data, offset)?)),
        b'B' | b'M' => f64::from(*data.get(offset)?),
        b'c' | b'h' => f64::from(i16::from_le_bytes(bytes(data, offset)?)),
        b'C' | b'H' => f64::from(u16::from_le_bytes(bytes(data, offset)?)),
        b'e' | b'i' | b'L' => f64::from(i32::from_le_bytes(bytes(data, offset)?)),
        b'E' | b'I' => f64::from(u32::from_le_bytes(bytes(data, offset)?)),
        b'f' => f64::from(f32::from_le_bytes(bytes(data, offset)?)),
        b'd' => f64::from_le_bytes(bytes(data, offset)?),
        b'q' => i64::from_le_bytes(bytes(data, offset)?) as f64,
        b'Q' => u64::from_le_bytes(bytes(data, offset)?) as f64,
        _ => 0.0,
    })
}

/// `WriteValue`: the C#'s casts, which truncate toward zero.
/// `// C#: Plugins/AnonymizeBinlogPlugin.cs:380-398`
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]
fn write_value(data: &mut [u8], offset: usize, fc: u8, value: f64) {
    let bytes: Vec<u8> = match fc {
        b'b' => vec![(value as i8) as u8],
        b'B' | b'M' => vec![value as u8],
        b'c' | b'h' => (value as i16).to_le_bytes().to_vec(),
        b'C' | b'H' => (value as u16).to_le_bytes().to_vec(),
        b'e' | b'i' | b'L' => (value as i32).to_le_bytes().to_vec(),
        b'E' | b'I' => (value as u32).to_le_bytes().to_vec(),
        b'f' => (value as f32).to_le_bytes().to_vec(),
        b'd' => value.to_le_bytes().to_vec(),
        b'q' => (value as i64).to_le_bytes().to_vec(),
        b'Q' => (value as u64).to_le_bytes().to_vec(),
        _ => return,
    };
    if let Some(slot) = data.get_mut(offset..offset + bytes.len()) {
        slot.copy_from_slice(&bytes);
    }
}

/// `OffsetValue`: integers are degrees times 1e7; a float over 1000 is taken for that too.
/// `// C#: Plugins/AnonymizeBinlogPlugin.cs:400-415`
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn offset_value(old: f64, fc: u8, offset: f64) -> f64 {
    match fc {
        b'L' | b'i' => f64::from((old + offset * 1e7) as i32),
        b'I' => f64::from((old + offset * 1e7) as i32 as u32),
        b'f' if old.abs() > 1000.0 => old + offset * 1e7,
        b'f' | b'd' => old + offset,
        _ => old,
    }
}

/// `Anonymizer.Anonymize`, on the log's bytes: the anonymised copy, or why not.
/// `// C#: Plugins/AnonymizeBinlogPlugin.cs:417-486`
fn anonymize(data: &[u8], offset_lat: f64, offset_lng: f64) -> Result<Vec<u8>, String> {
    let formats = parse_formats(data);
    let fmtu = parse_fmtu(data, &formats);
    let coords = coord_fields(&formats, &fmtu);
    if coords.is_empty() {
        return Err("No coordinate fields found in log file.".to_owned());
    }
    let mut out = data.to_vec();
    let mut pos = 0;
    let end = data.len().saturating_sub(2);
    while pos < end {
        if at(data, pos, 0) != HEAD1 || at(data, pos, 1) != HEAD2 {
            pos += 1;
            continue;
        }
        let id = at(data, pos, 2);
        if id == FMT_MSG_ID {
            pos += FMT_MSG_LEN;
            continue;
        }
        let Some(def) = formats.get(id) else {
            pos += 1;
            continue;
        };
        let length = def.length;
        if let Some((_, patches)) = coords.iter().find(|(key, _)| *key == id)
            && pos + length <= data.len()
        {
            for patch in patches {
                let offset = pos + patch.offset;
                if offset + patch.size > data.len() || is_string_format(patch.fc) {
                    continue;
                }
                let Some(old) = read_value(data, offset, patch.fc) else {
                    continue;
                };
                if old == 0.0 {
                    continue;
                }
                let degrees = if patch.lat { offset_lat } else { offset_lng };
                write_value(
                    &mut out,
                    offset,
                    patch.fc,
                    offset_value(old, patch.fc, degrees),
                );
            }
        }
        pos += length.max(1);
    }
    Ok(out)
}

/// FNV-1a over the log: the seed of its random offsets.
fn seed(data: &[u8]) -> u64 {
    data.iter().fold(0xcbf2_9ce4_8422_2325, |hash, &b| {
        (hash ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
    })
}

/// A number in [0, 1) from the seed, the seed moved on (xorshift64*).
#[allow(clippy::cast_precision_loss)]
fn next_double(state: &mut u64) -> f64 {
    let mut x = *state | 1;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    (x.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
}

/// `RandomOffset`: 0.5 to 2 degrees either way. `// C#: Plugins/AnonymizeBinlogPlugin.cs:101-106`
fn random_offset(state: &mut u64) -> f64 {
    let magnitude = next_double(state) * 1.5 + 0.5;
    let sign = if next_double(state) < 0.5 { -1.0 } else { 1.0 };
    magnitude * sign
}

/// `InputBox`, the plugin's own: a caption of "Anonymize Bin Log".
fn ask(prompt: &str) -> Option<String> {
    host::input_box("Anonymize Bin Log", prompt, "0")
}

/// The message box every outcome ends in.
fn tell(text: &str) {
    let _ = host::message_box(text, "Anonymize Bin Log", MessageButtons::Ok);
}

/// The input file's name with `_anon` before its extension.
fn anon_name(name: &str) -> String {
    match name.rfind('.') {
        Some(dot) if dot > 0 => format!("{}_anon{}", &name[..dot], &name[dot..]),
        _ => format!("{name}_anon"),
    }
}

struct AnonymizeBinlog;

impl Guest for AnonymizeBinlog {
    fn name() -> String {
        "Anonymize Binlog".to_owned()
    }

    fn version() -> String {
        "1.0".to_owned()
    }

    fn author() -> String {
        "MissionPlanner".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    fn init() -> bool {
        true
    }

    // `// C#: Plugins/AnonymizeBinlogPlugin.cs:28-32`
    fn loaded() -> bool {
        let id = host::menu_add(MapMenu::FlightData, None, "Anonymize Bin Log...");
        *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) = Some(id);
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    // `// C#: Plugins/AnonymizeBinlogPlugin.cs:44-99`
    fn menu_click(id: u32, _lat: f64, _lng: f64) {
        if *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) != Some(id) {
            return;
        }
        let Some(input) = host::open_file(
            "Select ArduPilot .bin log to anonymize",
            "BIN logs (*.bin)|*.bin|All files (*.*)|*.*",
        ) else {
            return;
        };
        let Some(lat_text) = ask("Latitude Offset (degrees, blank = random)") else {
            return;
        };
        let Some(lng_text) = ask("Longitude Offset (degrees, blank = random)") else {
            return;
        };
        let mut state = seed(&input.data);
        let mut offset = |text: &str| -> Result<f64, String> {
            if text.trim().is_empty() {
                Ok(random_offset(&mut state))
            } else {
                text.trim()
                    .parse::<f64>()
                    .map_err(|_| "Input string was not in a correct format.".to_owned())
            }
        };
        let offsets = offset(&lat_text).and_then(|lat| Ok((lat, offset(&lng_text)?)));
        let (offset_lat, offset_lng) = match offsets {
            Ok(offsets) => offsets,
            Err(err) => return tell(&format!("Error: {err}")),
        };
        let data = match anonymize(&input.data, offset_lat, offset_lng) {
            Ok(data) => data,
            Err(err) => return tell(&format!("Error: {err}")),
        };
        let Some(output) = host::save_file(
            "Save anonymized log",
            "BIN logs (*.bin)|*.bin|All files (*.*)|*.*",
            &anon_name(&input.name),
            &data,
        ) else {
            return;
        };
        tell(&format!(
            "Anonymization complete.\n\nOutput: {output}\n\nLat offset: {offset_lat:+.6}\u{b0}\nLon offset: {offset_lng:+.6}\u{b0}"
        ));
    }

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(AnonymizeBinlog);
