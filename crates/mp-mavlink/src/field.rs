//! Dynamic field access for generated messages.
//!
//! Generated code is where a port quietly goes wrong: a field can land at the right offset with
//! the wrong name, or the right name with the wrong sign. Byte-level round-tripping cannot see
//! that. Naming every field and its value lets the differential harness compare our decode with
//! the C# original field by field, across every message type, without hand-written comparisons.

/// A decoded field value.
#[derive(Debug, Clone, PartialEq)]
pub enum FieldValue {
    /// Unsigned integer field.
    Unsigned(u64),
    /// Signed integer field.
    Signed(i64),
    /// Floating point field.
    Float(f64),
    /// Array of unsigned integers.
    UnsignedArray(Vec<u64>),
    /// Array of signed integers.
    SignedArray(Vec<i64>),
    /// Array of floats.
    FloatArray(Vec<f64>),
}

impl FieldValue {
    /// Renders the value the way the reference dumper does, for differential comparison.
    #[must_use]
    pub fn to_compare_string(&self) -> String {
        match self {
            Self::Unsigned(v) => v.to_string(),
            Self::Signed(v) => v.to_string(),
            Self::Float(v) => format_float(*v),
            Self::UnsignedArray(v) => v
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(" "),
            Self::SignedArray(v) => v
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(" "),
            Self::FloatArray(v) => v
                .iter()
                .map(|f| format_float(*f))
                .collect::<Vec<_>>()
                .join(" "),
        }
    }
}

fn format_float(v: f64) -> String {
    if v.is_nan() {
        "NaN".to_owned()
    } else if v.is_infinite() {
        if v > 0.0 {
            "Infinity".to_owned()
        } else {
            "-Infinity".to_owned()
        }
    } else {
        format!("{v}")
    }
}
