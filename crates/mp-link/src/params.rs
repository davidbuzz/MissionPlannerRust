//! Parameter values and the download protocol.
//!
//! Replaces `ExtLibs/Mavlink/MAVLinkParam.cs` and the parameter half of `MAVLinkInterface`.
//!
//! # The encoding trap
//!
//! `PARAM_VALUE.param_value` is declared `float` on the wire, but it does not always *contain* a
//! float. It carries four bytes whose interpretation is given by `param_type`: an `INT32`
//! parameter with the value 5 arrives as the four bytes of the integer 5, which read as a float is
//! 7e-45. Treating the field as a number rather than as bytes silently corrupts every integer
//! parameter on the vehicle - and those include flight-mode numbers and failsafe actions.
//!
//! # The rounding trap
//!
//! A `REAL32` parameter of 0.8 widens to `0.800000011920929` as an `f64`. Mission Planner rounds
//! to seven significant digits before showing or comparing, which is why its parameter editor
//! shows `0.8`. We do the same, so that a parameter read back after a write compares equal.

use std::collections::BTreeMap;

/// MAVLink `MAV_PARAM_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParamType {
    /// 8-bit unsigned.
    Uint8,
    /// 8-bit signed.
    Int8,
    /// 16-bit unsigned.
    Uint16,
    /// 16-bit signed.
    Int16,
    /// 32-bit unsigned.
    Uint32,
    /// 32-bit signed.
    Int32,
    /// 32-bit float.
    Real32,
}

impl ParamType {
    /// From the wire value of `MAV_PARAM_TYPE`.
    #[must_use]
    pub const fn from_wire(value: u8) -> Option<Self> {
        Some(match value {
            1 => Self::Uint8,
            2 => Self::Int8,
            3 => Self::Uint16,
            4 => Self::Int16,
            5 => Self::Uint32,
            6 => Self::Int32,
            9 => Self::Real32,
            // 7 and 8 are 64-bit integers and 10 is a double. ArduPilot never sends them, and
            // accepting them would mean silently truncating to four bytes.
            _ => return None,
        })
    }

    /// The wire value.
    #[must_use]
    pub const fn to_wire(self) -> u8 {
        match self {
            Self::Uint8 => 1,
            Self::Int8 => 2,
            Self::Uint16 => 3,
            Self::Int16 => 4,
            Self::Uint32 => 5,
            Self::Int32 => 6,
            Self::Real32 => 9,
        }
    }

    /// Whether values of this type are whole numbers.
    #[must_use]
    pub const fn is_integer(self) -> bool {
        !matches!(self, Self::Real32)
    }
}

/// A parameter value: four wire bytes plus the type that says how to read them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamValue {
    raw: [u8; 4],
    param_type: ParamType,
}

/// Significant digits kept for float parameters, matching Mission Planner's display and
/// comparison behaviour.
const FLOAT_SIGNIFICANT_DIGITS: i32 = 7;

impl ParamValue {
    /// Wraps the raw wire bytes of a `PARAM_VALUE`.
    #[must_use]
    pub const fn from_wire_bytes(raw: [u8; 4], param_type: ParamType) -> Self {
        Self { raw, param_type }
    }

    /// Interprets `PARAM_VALUE.param_value` as delivered by a generated message struct.
    ///
    /// The `f32` is treated as a carrier for four bytes, not as a number.
    #[must_use]
    pub fn from_param_value_field(value: f32, param_type: ParamType) -> Self {
        Self {
            raw: value.to_le_bytes(),
            param_type,
        }
    }

    /// The bytes to put back into `PARAM_VALUE.param_value` or `PARAM_SET.param_value`.
    #[must_use]
    pub fn to_param_value_field(self) -> f32 {
        f32::from_le_bytes(self.raw)
    }

    /// The raw four bytes.
    #[must_use]
    pub const fn raw(self) -> [u8; 4] {
        self.raw
    }

    /// The type.
    #[must_use]
    pub const fn param_type(self) -> ParamType {
        self.param_type
    }

    /// The numeric value, rounded for floats the way Mission Planner rounds.
    #[must_use]
    pub fn as_f64(self) -> f64 {
        match self.param_type {
            ParamType::Uint8 => f64::from(self.raw[0]),
            ParamType::Int8 => f64::from(self.raw[0] as i8),
            ParamType::Uint16 => f64::from(u16::from_le_bytes([self.raw[0], self.raw[1]])),
            ParamType::Int16 => f64::from(i16::from_le_bytes([self.raw[0], self.raw[1]])),
            ParamType::Uint32 => f64::from(u32::from_le_bytes(self.raw)),
            ParamType::Int32 => f64::from(i32::from_le_bytes(self.raw)),
            ParamType::Real32 => round_to_significant_digits(
                f64::from(f32::from_le_bytes(self.raw)),
                FLOAT_SIGNIFICANT_DIGITS,
            ),
        }
    }

    /// Builds a value of a given type from a number, saturating rather than wrapping.
    ///
    /// Saturation matters: a user typing 300 into a `UINT8` parameter should get 255 and a visible
    /// disagreement, not 44.
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn from_f64(value: f64, param_type: ParamType) -> Self {
        let raw = match param_type {
            ParamType::Uint8 => [value.clamp(0.0, 255.0).round() as u8, 0, 0, 0],
            ParamType::Int8 => [(value.clamp(-128.0, 127.0).round() as i8) as u8, 0, 0, 0],
            ParamType::Uint16 => {
                let v = (value.clamp(0.0, 65_535.0).round() as u16).to_le_bytes();
                [v[0], v[1], 0, 0]
            }
            ParamType::Int16 => {
                let v = (value.clamp(-32_768.0, 32_767.0).round() as i16).to_le_bytes();
                [v[0], v[1], 0, 0]
            }
            ParamType::Uint32 => {
                (value.clamp(0.0, f64::from(u32::MAX)).round() as u32).to_le_bytes()
            }
            ParamType::Int32 => (value
                .clamp(f64::from(i32::MIN), f64::from(i32::MAX))
                .round() as i32)
                .to_le_bytes(),
            ParamType::Real32 => (value as f32).to_le_bytes(),
        };
        Self { raw, param_type }
    }
}

/// Rounds to a number of significant digits, as `MAVLinkParam.RoundToSignificantDigits` does.
#[must_use]
pub fn round_to_significant_digits(value: f64, digits: i32) -> f64 {
    if value == 0.0 || !value.is_finite() {
        return value;
    }
    let magnitude = value.abs().log10().floor() + 1.0;
    let power = f64::from(digits) - magnitude;
    let scale = 10f64.powf(power);
    (value * scale).round() / scale
}

/// Decodes a `param_id`, which is a fixed 16-byte field that is **not** null-terminated when the
/// name uses all 16 bytes.
///
/// Truncating at the first NUL is right; assuming one exists is not, and the names that fill the
/// field are real - `SERVO16_FUNCTION` is exactly 16 characters.
#[must_use]
pub fn decode_param_id(param_id: &[u8; 16]) -> String {
    let end = param_id
        .iter()
        .position(|b| *b == 0)
        .unwrap_or(param_id.len());
    param_id.get(..end).map_or_else(String::new, |bytes| {
        String::from_utf8_lossy(bytes).into_owned()
    })
}

/// Encodes a name into the fixed 16-byte field.
#[must_use]
pub fn encode_param_id(name: &str) -> [u8; 16] {
    let mut out = [0u8; 16];
    let bytes = name.as_bytes();
    let n = bytes.len().min(16);
    if let (Some(dst), Some(src)) = (out.get_mut(..n), bytes.get(..n)) {
        dst.copy_from_slice(src);
    }
    out
}

/// A vehicle's parameters, and what is still missing.
#[derive(Debug, Default, Clone)]
pub struct ParamTable {
    values: BTreeMap<String, ParamValue>,
    indices: BTreeMap<u16, String>,
    expected: Option<u16>,
    /// Whether the vehicle ever revised its own total upward.
    count_revised: bool,
}

impl ParamTable {
    /// An empty table.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Records a received `PARAM_VALUE`.
    pub fn insert(&mut self, name: String, value: ParamValue, index: u16, count: u16) {
        // Take the largest count the vehicle has ever claimed, not the first.
        //
        // ArduPilot revises this upward while it finishes enumerating: observed reporting 1395
        // early in a download that ended at 1408. Trusting the first value declares the download
        // complete with parameters still missing - and a parameter set silently short by thirteen
        // entries is the kind of thing nobody notices until a config screen shows a blank field.
        if count > 0 {
            match self.expected {
                None => self.expected = Some(count),
                Some(previous) if count > previous => {
                    self.expected = Some(count);
                    self.count_revised = true;
                }
                Some(_) => {}
            }
        }
        // Index 65535 is the "not part of the list" marker used by replies to a single read.
        if index != u16::MAX {
            self.indices.insert(index, name.clone());
        }
        self.values.insert(name, value);
    }

    /// Looks a parameter up by name.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<ParamValue> {
        self.values.get(name).copied()
    }

    /// How many parameters have been received.
    #[must_use]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Whether nothing has been received.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// How many the vehicle says it has, once it has said. This is the largest figure it has
    /// claimed, because ArduPilot revises it upward mid-download.
    #[must_use]
    pub const fn expected(&self) -> Option<u16> {
        self.expected
    }

    /// Whether the vehicle revised its total upward during the download.
    #[must_use]
    pub const fn count_was_revised(&self) -> bool {
        self.count_revised
    }

    /// Whether every parameter the vehicle claims has arrived.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.expected
            .is_some_and(|expected| self.indices.len() >= usize::from(expected))
    }

    /// Indices that have not arrived yet.
    ///
    /// Requesting these individually is the only reliable way to finish a parameter download:
    /// `PARAM_REQUEST_LIST` streams once with no retransmission, so a single dropped packet on a
    /// telemetry link leaves a permanent hole.
    #[must_use]
    pub fn missing(&self) -> Vec<u16> {
        let Some(expected) = self.expected else {
            return Vec::new();
        };
        (0..expected)
            .filter(|index| !self.indices.contains_key(index))
            .collect()
    }

    /// Every parameter, sorted by name.
    pub fn iter(&self) -> impl Iterator<Item = (&String, &ParamValue)> {
        self.values.iter()
    }

    /// How many distinct list indices have been seen.
    ///
    /// This is the number that decides completeness, and it can differ from [`ParamTable::len`]:
    /// ArduPilot answers a request-by-name with index 65535, and a vehicle can report a count that
    /// does not match the number of distinct names it actually sends. Keeping the two separate
    /// means the difference is visible rather than surfacing as a download that never finishes.
    #[must_use]
    pub fn indexed_len(&self) -> usize {
        self.indices.len()
    }

    /// Names received that were never given a place in the list.
    #[must_use]
    pub fn unindexed(&self) -> Vec<&String> {
        let placed: std::collections::BTreeSet<&String> = self.indices.values().collect();
        self.values
            .keys()
            .filter(|name| !placed.contains(name))
            .collect()
    }

    /// Progress as a fraction, for a progress bar.
    #[must_use]
    pub fn progress(&self) -> f32 {
        match self.expected {
            Some(expected) if expected > 0 => {
                f32::from(u16::try_from(self.indices.len()).unwrap_or(u16::MAX))
                    / f32::from(expected)
            }
            _ => 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integer_parameters_reinterpret_the_bytes_rather_than_the_number() {
        // The trap this whole module exists for. ArduPilot sends FLTMODE1 = 5 as an INT32: the
        // four bytes of the integer 5, carried in a field declared float.
        let wire = f32::from_le_bytes(5i32.to_le_bytes());
        assert!(
            wire < 1e-44,
            "the carrier float is a denormal, not 5.0: {wire}"
        );

        let value = ParamValue::from_param_value_field(wire, ParamType::Int32);
        assert!(
            (value.as_f64() - 5.0).abs() < f64::EPSILON,
            "expected 5, got {}",
            value.as_f64()
        );

        // Reading the same bytes as a float - the naive implementation - gives nonsense.
        let wrong = ParamValue::from_param_value_field(wire, ParamType::Real32);
        assert!(
            wrong.as_f64() < 1e-40,
            "the naive reading is {}",
            wrong.as_f64()
        );
    }

    #[test]
    fn every_type_round_trips_through_the_wire_field() {
        let cases: &[(ParamType, f64)] = &[
            (ParamType::Uint8, 200.0),
            (ParamType::Int8, -100.0),
            (ParamType::Uint16, 50_000.0),
            (ParamType::Int16, -20_000.0),
            (ParamType::Uint32, 3_000_000_000.0),
            (ParamType::Int32, -1_500_000_000.0),
            (ParamType::Real32, 0.8),
        ];
        for (param_type, number) in cases {
            let built = ParamValue::from_f64(*number, *param_type);
            let wire = built.to_param_value_field();
            let back = ParamValue::from_param_value_field(wire, *param_type);
            assert!(
                (back.as_f64() - number).abs() < 1e-6,
                "{param_type:?}: {number} came back as {}",
                back.as_f64()
            );
        }
    }

    #[test]
    fn float_parameters_read_back_the_way_a_user_typed_them() {
        // 0.8 as f32 widened to f64 is 0.800000011920929. A parameter editor showing that, or a
        // read-back comparing unequal to the value just written, is a bug users have reported
        // against other ground stations.
        let value = ParamValue::from_f64(0.8, ParamType::Real32);
        assert!(
            (value.as_f64() - 0.8).abs() < f64::EPSILON,
            "got {}",
            value.as_f64()
        );

        for typed in [0.1, 0.25, 1.5, 12.34, 0.001, 45_000.0, -3.75] {
            let stored = ParamValue::from_f64(typed, ParamType::Real32);
            assert!(
                (stored.as_f64() - typed).abs() < typed.abs() * 1e-6 + 1e-9,
                "{typed} read back as {}",
                stored.as_f64()
            );
        }
    }

    #[test]
    fn out_of_range_values_saturate_rather_than_wrap() {
        // 300 into a UINT8 must become 255, not 44. A wrapped value is a wrong setting that looks
        // deliberate.
        assert!((ParamValue::from_f64(300.0, ParamType::Uint8).as_f64() - 255.0).abs() < 1e-9);
        assert!((ParamValue::from_f64(-5.0, ParamType::Uint8).as_f64() - 0.0).abs() < 1e-9);
        assert!((ParamValue::from_f64(-200.0, ParamType::Int8).as_f64() + 128.0).abs() < 1e-9);
        assert!(
            (ParamValue::from_f64(1e12, ParamType::Int32).as_f64() - f64::from(i32::MAX)).abs()
                < 1.0
        );
    }

    #[test]
    fn parameter_names_that_fill_the_field_are_not_truncated() {
        // SERVO16_FUNCTION is exactly 16 characters, so there is no terminating NUL to find.
        let full = encode_param_id("SERVO16_FUNCTION");
        assert_eq!(decode_param_id(&full), "SERVO16_FUNCTION");

        let short = encode_param_id("ARMING_CHECK");
        assert_eq!(decode_param_id(&short), "ARMING_CHECK");
        assert_eq!(short[12], 0, "shorter names are NUL padded");

        // Over-long names are truncated at the field width rather than overflowing.
        let long = encode_param_id("THIS_NAME_IS_MUCH_TOO_LONG");
        assert_eq!(decode_param_id(&long).len(), 16);
    }

    #[test]
    fn the_table_knows_what_is_still_missing() {
        let mut table = ParamTable::new();
        assert!(!table.is_complete(), "an empty table is not complete");
        assert!(
            table.missing().is_empty(),
            "with no count there is nothing known to be missing"
        );

        let value = ParamValue::from_f64(1.0, ParamType::Real32);
        table.insert("A".to_owned(), value, 0, 5);
        table.insert("C".to_owned(), value, 2, 5);
        table.insert("E".to_owned(), value, 4, 5);

        assert_eq!(table.expected(), Some(5));
        assert_eq!(table.len(), 3);
        assert_eq!(
            table.missing(),
            vec![1, 3],
            "gaps must be identified for re-request"
        );
        assert!(!table.is_complete());
        assert!((table.progress() - 0.6).abs() < 1e-6);

        table.insert("B".to_owned(), value, 1, 5);
        table.insert("D".to_owned(), value, 3, 5);
        assert!(table.is_complete());
        assert!(table.missing().is_empty());
        assert!((table.progress() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn a_revised_upward_count_is_believed() {
        // Observed against real SITL: the vehicle reported 1395 early in a download that ended at
        // 1408. Believing the first figure declares completeness with parameters still missing.
        let mut table = ParamTable::new();
        let value = ParamValue::from_f64(1.0, ParamType::Real32);

        table.insert("A".to_owned(), value, 0, 2);
        table.insert("B".to_owned(), value, 1, 2);
        assert!(
            table.is_complete(),
            "complete against the count known so far"
        );

        // The vehicle now admits to more.
        table.insert("C".to_owned(), value, 2, 4);
        assert_eq!(table.expected(), Some(4));
        assert!(table.count_was_revised());
        assert!(
            !table.is_complete(),
            "must reopen when the vehicle raises its count"
        );
        assert_eq!(table.missing(), vec![3]);

        table.insert("D".to_owned(), value, 3, 4);
        assert!(table.is_complete());
    }

    #[test]
    fn a_downward_revision_is_ignored() {
        // The opposite case must not shrink the target, or a late stray message could truncate a
        // download that was proceeding correctly.
        let mut table = ParamTable::new();
        let value = ParamValue::from_f64(1.0, ParamType::Real32);
        table.insert("A".to_owned(), value, 0, 10);
        table.insert("B".to_owned(), value, 1, 3);
        assert_eq!(table.expected(), Some(10), "the larger claim stands");
        assert!(
            !table.count_was_revised(),
            "a downward claim is not a revision"
        );
    }

    #[test]
    fn a_single_read_reply_does_not_disturb_the_index_map() {
        // A reply to PARAM_REQUEST_READ by name carries index 65535, meaning "not telling you
        // where this sits in the list". Recording that as an index would corrupt the gap analysis.
        let mut table = ParamTable::new();
        let value = ParamValue::from_f64(1.0, ParamType::Real32);
        table.insert("A".to_owned(), value, 0, 3);
        table.insert("LATER".to_owned(), value, u16::MAX, 3);

        assert_eq!(table.len(), 2, "both values are stored");
        assert_eq!(
            table.missing(),
            vec![1, 2],
            "but only real indices count toward completeness"
        );
    }

    #[test]
    fn rounding_matches_the_reference_behaviour() {
        assert!((round_to_significant_digits(0.800_000_011_920_929, 7) - 0.8).abs() < 1e-12);
        assert!((round_to_significant_digits(1_234.567_89, 7) - 1_234.568).abs() < 1e-9);
        assert!((round_to_significant_digits(0.0, 7) - 0.0).abs() < f64::EPSILON);
        assert!(round_to_significant_digits(f64::NAN, 7).is_nan());
        assert!(round_to_significant_digits(f64::INFINITY, 7).is_infinite());
    }
}
