//! The number formats the output files use beyond `mp_log::netfmt`'s: the KML formatter's, and
//! the round-trip forms re-exported from there (they moved there for `mp-kml`'s sake).

use mp_log::netfmt;

/// `Environment.NewLine`, which `StreamWriter.WriteLine` and the XML writers end lines with.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

pub use mp_log::netfmt::{double_roundtrip, single_roundtrip};

/// `value.ToString("#0.##############")` in the invariant culture, what SharpKml's `KmlFormatter`
/// writes every `double` and `float` as: the value at 15 significant digits, then rounded half up
/// to 14 decimals, trailing zeros and a bare point dropped, and no minus sign on what rounds to
/// zero.
/// `// C#: ExtLibs/SharpKml/Base/KmlFormatter.cs:25-32`
#[must_use]
pub fn kml_double(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
    }
    let (negative, digits, scale) = netfmt::general_digits(value, 15);
    // Digits at places 10^(scale-1) down to 10^-14.
    let keep = usize::try_from((scale + 14).max(0)).unwrap_or(0);
    let mut kept: Vec<u8> = digits.iter().copied().take(keep).collect();
    let mut scale = scale;
    if digits.get(keep).is_some_and(|&d| d >= 5) {
        // Round up through the kept digits.
        let mut carry = true;
        for d in kept.iter_mut().rev() {
            if *d == 9 {
                *d = 0;
            } else {
                *d += 1;
                carry = false;
                break;
            }
        }
        if carry {
            kept.insert(0, 1);
            scale += 1;
        }
    }
    let mut integer = String::new();
    let mut fraction = String::new();
    for i in 0..usize::try_from(scale.max(0)).unwrap_or(0) {
        integer.push(char::from(b'0' + kept.get(i).copied().unwrap_or(0)));
    }
    if integer.is_empty() {
        integer.push('0');
    }
    let leading_zeros = usize::try_from((-scale).max(0)).unwrap_or(0);
    for _ in 0..leading_zeros.min(14) {
        fraction.push('0');
    }
    let from = usize::try_from(scale.max(0)).unwrap_or(0);
    for &d in kept.iter().skip(from) {
        fraction.push(char::from(b'0' + d));
    }
    let fraction = fraction.get(..fraction.len().min(14)).unwrap_or_default();
    let fraction = fraction.trim_end_matches('0');
    let zero = integer.bytes().all(|b| b == b'0') && fraction.is_empty();
    let mut out = String::new();
    if negative && !zero {
        out.push('-');
    }
    out.push_str(&integer);
    if !fraction.is_empty() {
        out.push('.');
        out.push_str(fraction);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn what_mono_printed() {
        // What mono printed for `ToString("#0.##############", CultureInfo.InvariantCulture)`.
        assert_eq!(kml_double(-27.469_820_022_583_008), "-27.469820022583");
        assert_eq!(kml_double(153.025_123_456_789_12), "153.025123456789");
        assert_eq!(kml_double(0.000_000_000_000_001), "0");
        assert_eq!(kml_double(-0.000_000_000_000_001), "0");
        assert_eq!(kml_double(1e20), "100000000000000000000");
        assert_eq!(kml_double(0.5), "0.5");
        assert_eq!(kml_double(2.000_000_000_000_05), "2.00000000000005");
        assert_eq!(
            kml_double(netfmt::parse_double("12345.6789012345678").unwrap()),
            "12345.6789012346"
        );
        assert_eq!(kml_double(25.1), "25.1");
        assert_eq!(kml_double(-27.4698), "-27.4698");
        assert_eq!(kml_double(0.0), "0");
        assert_eq!(kml_double(-180.268_989_562_988_28), "-180.268989562988");
    }
}
