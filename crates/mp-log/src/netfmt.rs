//! Numbers and text the way .NET writes and reads them, for the files Mission Planner's log
//! converters produce.
//!
//! Every converter on the DataFlash Logs page goes through text: `BinaryLog` turns each field into
//! `((IConvertible)value).ToString(CultureInfo.InvariantCulture)` and joins them
//! (`ExtLibs/Utilities/BinaryLog.cs:157-176`), and `LogOutput` and `MatLab` split that text again
//! and `double.Parse` the pieces. So what a `.log`, a `.kml` or a `.mat` holds is decided by .NET's
//! formatting and parsing rules, not by the numbers in the log, and those rules are ported here.
//!
//! **Formatting** is .NET Framework's, which is what Mission Planner runs on (`net472`,
//! `MissionPlanner.csproj:3`): the default `ToString()` of a `float` is the general format with 7
//! significant digits and of a `double` with 15 - not the shortest round-trip form .NET Core
//! switched to - so `3.14159274f` is written `3.141593`. Digits are rounded half away from zero
//! from the value's exact binary expansion, the exponent form starts when the decimal exponent is
//! at least the precision or below -4, and it is written `E+XX` with at least two digits. Mono
//! formats the same way, which `tests/convert.rs` holds against the oracle's output byte for byte.
//!
//! **Parsing** is `Number.ParseNumber` for the invariant culture, with the styles the C# asks for.

/// `NumberFormatInfo.InvariantInfo.NaNSymbol`.
const NAN: &str = "NaN";

/// `double.NaN` as .NET has it: `0.0 / 0.0` on x86, the negative quiet NaN, which is what
/// `double.Parse("NaN")` returns and so what a `.mat` stores for one.
pub const DOTNET_NAN: f64 = f64::from_bits(0xFFF8_0000_0000_0000);
/// `NumberFormatInfo.InvariantInfo.PositiveInfinitySymbol`.
const POSITIVE_INFINITY: &str = "Infinity";
/// `NumberFormatInfo.InvariantInfo.NegativeInfinitySymbol`.
const NEGATIVE_INFINITY: &str = "-Infinity";

/// `float.ToString(CultureInfo.InvariantCulture)`: general format, 7 significant digits.
#[must_use]
pub fn single(value: f32) -> String {
    general(f64::from(value), 7)
}

/// `double.ToString(CultureInfo.InvariantCulture)`: general format, 15 significant digits.
#[must_use]
pub fn double(value: f64) -> String {
    general(value, 15)
}

/// A number as .NET's general format with `precision` digits has it: sign, digits (trailing zeros
/// dropped), and the power of ten of the first digit's place plus one (`0.d1d2... x 10^scale`).
#[must_use]
pub fn general_digits(value: f64, precision: i32) -> (bool, Vec<u8>, i32) {
    let text = general(value, usize::try_from(precision).unwrap_or(15));
    let negative = text.starts_with('-');
    let body = text.trim_start_matches('-');
    let (mantissa, exponent) = match body.split_once('E') {
        Some((m, e)) => (m, e.parse::<i32>().unwrap_or(0)),
        None => (body, 0),
    };
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let mut digits: Vec<u8> = whole
        .bytes()
        .chain(fraction.bytes())
        .map(|b| b - b'0')
        .collect();
    let mut scale = i32::try_from(whole.len()).unwrap_or(0) + exponent;
    while digits.first() == Some(&0) && digits.len() > 1 {
        digits.remove(0);
        scale -= 1;
    }
    if digits.iter().all(|&d| d == 0) {
        return (negative, Vec::new(), 0);
    }
    (negative, digits, scale)
}

/// `DoubleToNumber(value, precision)` read back through `NumberToDouble`: the magnitude of the
/// value's first `precision` significant digits, zeros kept.
fn reread(value: f64, precision: i32) -> f64 {
    let (_, mut digits, scale) = general_digits(value, precision);
    digits.resize(usize::try_from(precision).unwrap_or(0).max(digits.len()), 0);
    number_to_double_digits(&digits, i64::from(scale))
}

/// `double.ToString("R")` as .NET Framework (and mono, the oracle) writes it, which is what
/// `XmlConvert.ToString(double)` and so `XmlSerializer` write: 15 digits when `NumberToDouble` of
/// those 15 digits gives the value back, else 17 - and that test is not "does the text parse back",
/// because the digits it reads keep their trailing zeros (see `number_to_double`), so
/// `-27.4692539`, whose 15 digits end `…539000000`, comes out `-27.469253899999998`.
/// `// C#: clr/src/classlibnative/bcltype/number.cpp FormatDouble, case 'R'`
#[must_use]
pub fn double_roundtrip(value: f64) -> String {
    if !value.is_finite() || value == 0.0 {
        return general(value, 15);
    }
    if reread(value, 15) == value.abs() {
        general(value, 15)
    } else {
        general(value, 17)
    }
}

/// `float.ToString("R")`: 7 digits when `NumberToDouble` of those 7, narrowed, is the value, else 9.
/// `// C#: clr/src/classlibnative/bcltype/number.cpp FormatSingle, case 'R'`
#[must_use]
pub fn single_roundtrip(value: f32) -> String {
    let wide = f64::from(value);
    if !value.is_finite() || value == 0.0 {
        return general(wide, 7);
    }
    #[allow(clippy::cast_possible_truncation)]
    let back = reread(wide, 7) as f32;
    if back == value.abs() {
        general(wide, 7)
    } else {
        general(wide, 9)
    }
}

/// The general ("G") format with `precision` significant digits, as .NET Framework's
/// `Number.FormatDouble` writes it for the invariant culture.
///
/// Negative zero is written `0`: .NET Framework drops the sign of a zero.
#[must_use]
pub fn general(value: f64, precision: usize) -> String {
    if value.is_nan() {
        return NAN.to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 {
            POSITIVE_INFINITY.to_owned()
        } else {
            NEGATIVE_INFINITY.to_owned()
        };
    }
    if value == 0.0 {
        return "0".to_owned();
    }
    let precision = precision.max(1);
    let (digits, scale) = significant_digits(value.abs(), precision);
    let mut out = String::new();
    if value < 0.0 {
        out.push('-');
    }
    // `value = 0.d1d2d3... x 10^scale`. The exponent form is used when the decimal point would sit
    // more than `precision` places right of the first digit, or more than three zeros left of it.
    let scientific = scale > i32::try_from(precision).unwrap_or(i32::MAX) || scale < -3;
    if scientific {
        let mut rest = digits.iter();
        if let Some(&first) = rest.next() {
            push_digit(&mut out, first);
        }
        if digits.len() > 1 {
            out.push('.');
            for &d in rest {
                push_digit(&mut out, d);
            }
        }
        let exponent = scale - 1;
        out.push('E');
        out.push(if exponent < 0 { '-' } else { '+' });
        let magnitude = exponent.unsigned_abs();
        if magnitude < 10 {
            out.push('0');
        }
        out.push_str(&magnitude.to_string());
    } else if scale <= 0 {
        out.push_str("0.");
        for _ in 0..scale.unsigned_abs() {
            out.push('0');
        }
        for &d in &digits {
            push_digit(&mut out, d);
        }
    } else {
        let whole = usize::try_from(scale).unwrap_or(0);
        for index in 0..whole.max(digits.len()) {
            if index == whole {
                out.push('.');
            }
            push_digit(&mut out, digits.get(index).copied().unwrap_or(0));
        }
    }
    out
}

fn push_digit(out: &mut String, digit: u8) {
    out.push(char::from(b'0' + digit));
}

/// The first `precision` significant digits of a positive finite `value`, rounded half away from
/// zero from its exact expansion, trailing zeros dropped; and the scale `s` with
/// `value ~ 0.d1d2... x 10^s`.
fn significant_digits(value: f64, precision: usize) -> (Vec<u8>, i32) {
    // Three guard digits decide almost every case: Rust rounds them correctly, so unless they read
    // exactly 500 the exact expansion is known to be above or below the half.
    let (mut digits, exponent) = expansion(value, precision + 3);
    let at = |index: usize| u32::from(digits.get(index).copied().unwrap_or(0));
    let guard = at(precision) * 100 + at(precision + 1) * 10 + at(precision + 2);
    let round_up = if guard == 500 {
        // Within half a unit of the third guard digit of a tie. Enough further digits tell a true
        // tie (which rounds up) from a value a hair either side of one: a double's distance from
        // a decimal tie, when it is not on it, grows no smaller than about 10^-(precision + |e|).
        let magnitude = value.log10().abs().min(400.0);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let extra = 40 + 2 * magnitude as usize;
        let (exact, _) = expansion(value, precision + extra);
        exact.get(precision).is_some_and(|&d| d >= 5)
    } else {
        guard > 500
    };
    digits.truncate(precision);
    let mut scale = exponent + 1;
    if round_up {
        let mut index = precision;
        loop {
            if index == 0 {
                digits.insert(0, 1);
                digits.truncate(precision);
                scale += 1;
                break;
            }
            index -= 1;
            match digits.get_mut(index) {
                Some(digit) if *digit == 9 => *digit = 0,
                Some(digit) => {
                    *digit += 1;
                    break;
                }
                None => break,
            }
        }
    }
    while digits.len() > 1 && digits.last() == Some(&0) {
        digits.pop();
    }
    (digits, scale)
}

/// `count` significant digits of `value`, correctly rounded, and the decimal exponent of the first.
fn expansion(value: f64, count: usize) -> (Vec<u8>, i32) {
    let text = format!("{:.*e}", count.saturating_sub(1), value);
    let (mantissa, exponent) = text.split_once('e').unwrap_or((&text, "0"));
    let digits = mantissa
        .bytes()
        .filter(u8::is_ascii_digit)
        .map(|b| b - b'0')
        .collect();
    (digits, exponent.parse().unwrap_or(0))
}

/// `Encoding.ASCII.GetString`: every byte above 0x7F becomes `?`.
#[must_use]
pub fn ascii(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|&b| if b < 0x80 { char::from(b) } else { '?' })
        .collect()
}

/// `string.Trim('\0')`: NULs off both ends.
#[must_use]
pub fn trim_nul(text: &str) -> &str {
    text.trim_matches('\0')
}

/// `Char.IsWhiteSpace` as `Number.ParseNumber` tests it: space and `\t` to `\r`.
const fn is_number_white(c: char) -> bool {
    c == ' ' || matches!(c, '\t'..='\r')
}

/// `string.Trim()` for ASCII text: .NET trims every Unicode white space character, which for the
/// ASCII this module sees is space, `\t` to `\r`, and the separators `\x1C` to `\x1F`.
#[must_use]
pub fn trim(text: &str) -> &str {
    text.trim_matches(|c: char| c.is_whitespace() || matches!(c, '\u{1c}'..='\u{1f}'))
}

/// The `NumberStyles` a parse allows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Styles {
    leading_white: bool,
    trailing_white: bool,
    leading_sign: bool,
    trailing_sign: bool,
    parentheses: bool,
    decimal_point: bool,
    thousands: bool,
    exponent: bool,
}

/// `NumberStyles.Float | NumberStyles.AllowThousands`: what `double.Parse(s)` and
/// `double.TryParse(s, out d)` use.
const FLOAT_THOUSANDS: Styles = Styles {
    leading_white: true,
    trailing_white: true,
    leading_sign: true,
    trailing_sign: false,
    parentheses: false,
    decimal_point: true,
    thousands: true,
    exponent: true,
};

/// `NumberStyles.Any`.
const ANY: Styles = Styles {
    leading_white: true,
    trailing_white: true,
    leading_sign: true,
    trailing_sign: true,
    parentheses: true,
    decimal_point: true,
    thousands: true,
    exponent: true,
};

/// `NumberStyles.Integer`: what `int.Parse(s)` and `long.Parse(s)` use.
const INTEGER: Styles = Styles {
    leading_white: true,
    trailing_white: true,
    leading_sign: true,
    trailing_sign: false,
    parentheses: false,
    decimal_point: false,
    thousands: false,
    exponent: false,
};

/// A number `ParseNumber` accepted: its sign, its significant digits and where the point goes.
struct Parsed {
    negative: bool,
    digits: String,
    /// Power of ten the digits are multiplied by.
    exponent: i64,
}

/// `Number.ParseNumber` for the invariant culture: `+`/`-` signs, `.` decimal point, `,` group
/// separator, and nothing after the number but white space and NULs.
fn parse_number(text: &str, styles: Styles) -> Option<Parsed> {
    let chars: Vec<char> = text.chars().collect();
    let mut at = 0usize;
    let mut negative = false;
    let mut signed = false;
    let mut parens = false;
    // Leading white space, then one sign or opening parenthesis. White space after the sign is not
    // allowed: the invariant culture's NumberNegativePattern is 1.
    while let Some(&c) = chars.get(at) {
        if styles.leading_white && is_number_white(c) && !signed {
            at += 1;
        } else if styles.leading_sign && !signed && (c == '+' || c == '-') {
            signed = true;
            negative = c == '-';
            at += 1;
        } else if styles.parentheses && !signed && c == '(' {
            signed = true;
            parens = true;
            negative = true;
            at += 1;
        } else {
            break;
        }
    }
    let mut digits = String::new();
    let mut seen_digit = false;
    let mut seen_point = false;
    let mut exponent: i64 = 0;
    while let Some(&c) = chars.get(at) {
        if c.is_ascii_digit() {
            seen_digit = true;
            // Leading zeros carry no information; the rest is kept whole and handed to Rust's
            // correctly rounded conversion.
            if !(digits.is_empty() && c == '0') {
                digits.push(c);
                if seen_point {
                    exponent -= 1;
                }
            } else if seen_point {
                exponent -= 1;
            }
            at += 1;
        } else if styles.decimal_point && !seen_point && c == '.' {
            seen_point = true;
            at += 1;
        } else if styles.thousands && seen_digit && !seen_point && c == ',' {
            at += 1;
        } else {
            break;
        }
    }
    if !seen_digit {
        return None;
    }
    if styles.exponent && matches!(chars.get(at), Some('e' | 'E')) {
        let mut probe = at + 1;
        let mut exponent_negative = false;
        if let Some(&c) = chars.get(probe)
            && (c == '+' || c == '-')
        {
            exponent_negative = c == '-';
            probe += 1;
        }
        let start = probe;
        let mut value: i64 = 0;
        while let Some(&c) = chars.get(probe)
            && let Some(d) = c.to_digit(10)
        {
            value = value
                .saturating_mul(10)
                .saturating_add(i64::from(d))
                .min(1_000_000);
            probe += 1;
        }
        // An `E` with no digits after it is not part of the number, and whatever follows decides.
        if probe > start {
            exponent += if exponent_negative { -value } else { value };
            at = probe;
        }
    }
    // Trailing white space, a trailing sign, the closing parenthesis.
    while let Some(&c) = chars.get(at) {
        if styles.trailing_white && is_number_white(c) {
            at += 1;
        } else if styles.trailing_sign && !signed && (c == '+' || c == '-') {
            signed = true;
            negative = c == '-';
            at += 1;
        } else if parens && c == ')' {
            parens = false;
            at += 1;
        } else {
            break;
        }
    }
    if parens {
        return None;
    }
    // `TrailingZeros`: NULs may follow the number.
    if chars.iter().skip(at).any(|&c| c != '\0') {
        return None;
    }
    Some(Parsed {
        negative,
        digits,
        exponent,
    })
}

/// `rgval64Power10`: 10^1 to 10^15, then 10^-1 to 10^-15, as normalised 64-bit mantissas. Read
/// out of mono's `mscorlib.dll`, whose `double.Parse` is .NET Framework's `NumberToDouble`.
const POWER10: [u64; 30] = [
    0xa000_0000_0000_0000,
    0xc800_0000_0000_0000,
    0xfa00_0000_0000_0000,
    0x9c40_0000_0000_0000,
    0xc350_0000_0000_0000,
    0xf424_0000_0000_0000,
    0x9896_8000_0000_0000,
    0xbebc_2000_0000_0000,
    0xee6b_2800_0000_0000,
    0x9502_f900_0000_0000,
    0xba43_b740_0000_0000,
    0xe8d4_a510_0000_0000,
    0x9184_e72a_0000_0000,
    0xb5e6_20f4_8000_0000,
    0xe35f_a931_a000_0000,
    0xcccc_cccc_cccc_cccd,
    0xa3d7_0a3d_70a3_d70b,
    0x8312_6e97_8d4f_df3c,
    0xd1b7_1758_e219_652e,
    0xa7c5_ac47_1b47_8425,
    0x8637_bd05_af6c_69b7,
    0xd6bf_94d5_e57a_42be,
    0xabcc_7711_8461_ceff,
    0x8970_5f41_36b4_a599,
    0xdbe6_fece_bded_d5c2,
    0xafeb_ff0b_cb24_ab02,
    0x8cbc_cc09_6f50_88cf,
    0xe12e_1342_4bb4_0e18,
    0xb424_dc35_095c_d813,
    0x901d_7cf7_3ab0_acdc,
];

/// `rgexp64Power10`: the binary exponents of [`POWER10`], shared by the powers and their inverses.
const POWER10_EXPONENT: [i32; 15] = [4, 7, 10, 14, 17, 20, 24, 27, 30, 34, 37, 40, 44, 47, 50];

/// `rgval64Power10By16`: 10^16 to 10^336 in steps of 10^16, then their inverses.
const POWER10_BY16: [u64; 42] = [
    0x8e1b_c9bf_0400_0000,
    0x9dc5_ada8_2b70_b59e,
    0xaf29_8d05_0e43_95d6,
    0xc278_1f49_ffcf_a6d4,
    0xd7e7_7a8f_87da_f7fa,
    0xefb3_ab16_c59b_14a0,
    0x850f_adc0_9923_329c,
    0x93ba_47c9_80e9_8cde,
    0xa402_b9c5_a8d3_a6e6,
    0xb616_a12b_7fe6_17a8,
    0xca28_a291_859b_bf90,
    0xe070_f78d_3927_5566,
    0xf92e_0c35_3782_6140,
    0x8a52_96ff_e33c_c92c,
    0x9991_a6f3_d6bf_1762,
    0xaa7e_ebfb_9df9_de8a,
    0xbd49_d14a_a79d_bc7e,
    0xd226_fc19_5c6a_2f88,
    0xe950_df20_247c_83f8,
    0x8184_2f29_f2cc_e373,
    0x8fca_c257_558e_e4e2,
    0xe695_94be_c44d_e160,
    0xcfb1_1ead_4539_94c3,
    0xbb12_7c53_b17e_c165,
    0xa87f_ea27_a539_e9b3,
    0x97c5_60ba_6b09_19b5,
    0x88b4_02f7_fd75_53ab,
    0xf643_35bc_f065_d3a0,
    0xddd0_467c_64bc_e4c4,
    0xc7ca_ba6e_7c53_82ed,
    0xb3f4_e093_db73_a0b7,
    0xa217_27db_38cb_0053,
    0x91ff_8377_5423_cc29,
    0x8380_dea9_3da4_bc82,
    0xece5_3cec_4a31_4f00,
    0xd560_5fcd_cf32_e217,
    0xc031_4325_637a_1978,
    0xad1c_8eab_5ee4_3ba2,
    0x9bec_ce62_836a_c5b0,
    0x8c71_dcd9_ba0b_495c,
    0xfd00_b897_4782_3938,
    0xe3e2_7a44_4d8d_991a,
];

/// `rgexp64Power10By16`: the binary exponents of [`POWER10_BY16`].
const POWER10_BY16_EXPONENT: [i32; 21] = [
    54, 107, 160, 213, 266, 319, 373, 426, 479, 532, 585, 638, 691, 745, 798, 851, 904, 957, 1010,
    1064, 1117,
];

/// `Mul64Lossy`: the top 64 bits of a 128-bit product, roughly, renormalised.
fn mul64_lossy(a: u64, b: u64, exponent: &mut i32) -> u64 {
    let mul = |x: u64, y: u64| (x & 0xffff_ffff) * (y & 0xffff_ffff);
    let mut value = mul(a >> 32, b >> 32)
        .wrapping_add(mul(a >> 32, b) >> 32)
        .wrapping_add(mul(a, b >> 32) >> 32);
    if value & 0x8000_0000_0000_0000 == 0 {
        value <<= 1;
        *exponent -= 1;
    }
    value
}

/// .NET Framework's `NumberToDouble`: the magnitude of `0.d1d2d3... x 10^scale`.
///
/// **Not correctly rounded, on purpose.** It reads at most 18 digits, multiplies by at most two
/// table powers of ten with a truncating 64-bit product, and rounds once - so `-0.0007147721`
/// reads a unit in the last place away from the nearest double. Mission Planner parses every number
/// of a log's text this way, and a `.mat` stores what it gets.
fn number_to_double(digits: &str, scale: i64) -> f64 {
    // `ParseNumber`'s `NUMBER` holds 50 digits and drops the trailing zeros of those (`digEnd`).
    let stored: Vec<u8> = digits
        .trim_start_matches('0')
        .bytes()
        .take(50)
        .map(|b| b - b'0')
        .collect();
    let significant = stored.iter().rposition(|&d| d != 0).map_or(0, |at| at + 1);
    number_to_double_digits(stored.get(..significant).unwrap_or_default(), scale)
}

/// `NumberToDouble` over a `NUMBER` whose digits are given as-is - **trailing zeros included**,
/// which is how `DoubleToNumber` fills one for "R"'s round-trip test. The zeros change how many
/// digits go into the 64-bit mantissa and which power of ten it is scaled by, and so, for about
/// one number in a thousand, the rounding: `-27.4692539`'s 15 digits `274692539000000` read back a
/// unit in the last place away, so "R" writes it with 17.
/// `// C#: clr/src/classlibnative/bcltype/number.cpp NumberToDouble`
fn number_to_double_digits(digits: &[u8], scale: i64) -> f64 {
    let digits: Vec<u64> = digits
        .iter()
        .skip_while(|&&d| d == 0)
        .map(|&d| u64::from(d))
        .collect();
    let total = digits.len();
    if total == 0 {
        return 0.0;
    }
    let to_int = |part: &[u64]| part.iter().fold(0u64, |acc, &d| acc * 10 + d);
    let mut remaining = total;
    let count = remaining.min(9);
    remaining -= count;
    let mut value = to_int(digits.get(..count).unwrap_or_default());
    if remaining > 0 {
        let count = remaining.min(9);
        remaining -= count;
        let power = POWER10.get(count - 1).copied().unwrap_or(0);
        let shift = POWER10_EXPONENT.get(count - 1).copied().unwrap_or(0);
        let multiplier = power >> (64 - shift);
        value = value * multiplier + to_int(digits.get(9..9 + count).unwrap_or_default());
    }
    let consumed = i64::try_from(total - remaining).unwrap_or(0);
    let scale = scale.saturating_sub(consumed);
    let magnitude = scale.unsigned_abs();
    if magnitude >= 22 * 16 {
        return if scale > 0 { f64::INFINITY } else { 0.0 };
    }
    let magnitude = usize::try_from(magnitude).unwrap_or(0);

    let mut exponent: i32 = 64;
    for (mask, shift) in [
        (0xFFFF_FFFF_0000_0000u64, 32),
        (0xFFFF_0000_0000_0000, 16),
        (0xFF00_0000_0000_0000, 8),
        (0xF000_0000_0000_0000, 4),
        (0xC000_0000_0000_0000, 2),
        (0x8000_0000_0000_0000, 1),
    ] {
        if value & mask == 0 {
            value <<= shift;
            exponent -= shift;
        }
    }

    let inverse = scale < 0;
    let index = magnitude & 15;
    if index > 0 {
        let multiplier_exponent = POWER10_EXPONENT.get(index - 1).copied().unwrap_or(0);
        exponent += if inverse {
            -multiplier_exponent + 1
        } else {
            multiplier_exponent
        };
        let multiplier = POWER10
            .get(index + if inverse { 15 } else { 0 } - 1)
            .copied()
            .unwrap_or(0);
        value = mul64_lossy(value, multiplier, &mut exponent);
    }
    let index = magnitude >> 4;
    if index > 0 {
        let multiplier_exponent = POWER10_BY16_EXPONENT.get(index - 1).copied().unwrap_or(0);
        exponent += if inverse {
            -multiplier_exponent + 1
        } else {
            multiplier_exponent
        };
        let multiplier = POWER10_BY16
            .get(index + if inverse { 21 } else { 0 } - 1)
            .copied()
            .unwrap_or(0);
        value = mul64_lossy(value, multiplier, &mut exponent);
    }

    // Round to 53 bits, half to even, on bit 10.
    if value & (1 << 10) != 0 {
        let mut rounded = value
            .wrapping_add((1 << 10) - 1)
            .wrapping_add((value >> 11) & 1);
        if rounded < value {
            rounded = (rounded >> 1) | 0x8000_0000_0000_0000;
            exponent += 1;
        }
        value = rounded;
    }

    exponent += 0x3FE;
    let bits = if exponent <= 0 {
        if exponent == -52 && value >= 0x8000_0000_0000_0058 {
            1
        } else if exponent <= -52 {
            0
        } else {
            value >> (-exponent + 11)
        }
    } else if exponent >= 0x7FF {
        0x7FF0_0000_0000_0000
    } else {
        (u64::try_from(exponent).unwrap_or(0) << 52) + ((value >> 11) & 0x000F_FFFF_FFFF_FFFF)
    };
    f64::from_bits(bits)
}

fn parse_double_with(text: &str, styles: Styles) -> Option<f64> {
    if let Some(parsed) = parse_number(text, styles) {
        let length = i64::try_from(parsed.digits.len()).unwrap_or(i64::MAX);
        let magnitude = number_to_double(&parsed.digits, length.saturating_add(parsed.exponent));
        // .NET Framework refuses a number too large for a double rather than calling it infinity.
        if magnitude.is_infinite() {
            return None;
        }
        return Some(if parsed.negative {
            -magnitude
        } else {
            magnitude
        });
    }
    // What `Double.TryParse` tries when the number does not parse: the culture's own symbols.
    match trim(text) {
        POSITIVE_INFINITY => Some(f64::INFINITY),
        NEGATIVE_INFINITY => Some(f64::NEG_INFINITY),
        NAN => Some(DOTNET_NAN),
        _ => None,
    }
}

/// `double.Parse(s, CultureInfo.InvariantCulture)` and `double.TryParse(s, out d)` under the
/// invariant culture: `NumberStyles.Float | NumberStyles.AllowThousands`.
#[must_use]
pub fn parse_double(text: &str) -> Option<f64> {
    parse_double_with(text, FLOAT_THOUSANDS)
}

/// `double.TryParse(s, NumberStyles.Any, CultureInfo.InvariantCulture, out d)`.
#[must_use]
pub fn parse_double_any(text: &str) -> Option<f64> {
    parse_double_with(text, ANY)
}

/// `long.Parse(s)`: `NumberStyles.Integer`, and an overflow is a failure.
#[must_use]
pub fn parse_i64(text: &str) -> Option<i64> {
    let parsed = parse_number(text, INTEGER)?;
    let magnitude: i128 = if parsed.digits.is_empty() {
        0
    } else {
        parsed.digits.parse().ok()?
    };
    i64::try_from(if parsed.negative {
        -magnitude
    } else {
        magnitude
    })
    .ok()
}

/// `int.Parse(s)`: `NumberStyles.Integer`, and an overflow is a failure.
#[must_use]
pub fn parse_i32(text: &str) -> Option<i32> {
    i32::try_from(parse_i64(text)?).ok()
}

/// `Comparer<string>.Default` under the invariant culture, which is what orders a
/// `SortedDictionary<string, _>`: a culture-aware comparison, not an ordinal one.
///
/// Only the characters parameter names are made of are placed as the culture places them -
/// punctuation (`_`) before digits before letters, letters compared without case first and lower
/// case first after that, a prefix before the longer string - which is what Mission Planner's own
/// `.param` output holds (`testdata/dataflash/golden/kml/*.param`: `ATC_ANG_YAW_P` before
/// `ATC_ANGLE_BOOST`, `INS_ACC_ID` before `INS_ACC1_CALTEMP` before `INS_ACCEL_FILTER`).
/// Anything else falls back to its code point, which the full collation tables would not always
/// agree with.
#[must_use]
pub fn culture_compare(a: &str, b: &str) -> std::cmp::Ordering {
    fn primary(c: char) -> (u8, u32) {
        match c {
            '0'..='9' => (1, c as u32),
            'a'..='z' | 'A'..='Z' => (2, c.to_ascii_lowercase() as u32),
            c if c.is_ascii() => (0, c as u32),
            c => (3, c as u32),
        }
    }
    let by_primary = a.chars().map(primary).cmp(b.chars().map(primary));
    by_primary
        .then_with(|| {
            // Lower case sorts before upper case at the tertiary level.
            let tertiary = |c: char| u8::from(c.is_ascii_uppercase());
            a.chars().map(tertiary).cmp(b.chars().map(tertiary))
        })
        .then_with(|| a.cmp(b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_keeps_the_trailing_zeros_it_reads() {
        // What mono printed for ToString("R") (and XmlConvert.ToString) of double.Parse(text).
        for (text, r) in [
            ("-27.4692539", "-27.469253899999998"),
            ("-27.4698", "-27.4698"),
            ("153.0251011", "153.0251011"),
            ("65.18", "65.18"),
            ("25.1", "25.1"),
            ("-13.1813034", "-13.181303399999999"),
            ("41.3394838", "41.339483799999996"),
        ] {
            let v = parse_double(text).unwrap();
            assert_eq!(double_roundtrip(v), r, "{text}");
        }
        assert_eq!(double_roundtrip(0.0), "0");
        assert_eq!(single_roundtrip(0.073_479_59), "0.07347959");
    }

    /// Every value here was printed by mono 6.12 (`((IConvertible)f).ToString(Invariant)`), which
    /// formats as .NET Framework does.
    #[test]
    fn single_matches_the_runtime() {
        let cases: &[(f32, &str)] = &[
            (0.1, "0.1"),
            (1e-5, "1E-05"),
            (123_456_789.0, "1.234568E+08"),
            (1_234_567.0, "1234567"),
            (12_345_678.0, "1.234568E+07"),
            (0.0001, "0.0001"),
            (0.000_012_345_67, "1.234567E-05"),
            (1.5e-45, "1.401298E-45"),
            (f32::MAX, "3.402823E+38"),
            (-0.0, "0"),
            (9.999_999_9, "10"),
            (2.718_281_7, "2.718282"),
            (16_777_216.0, "1.677722E+07"),
            (1e7, "1E+07"),
            (9_999_999.0, "9999999"),
            (100_000.0, "100000"),
            (0.999_999_97, "0.9999999"),
            (1.000_000_1, "1"),
            // Exact ties round away from zero.
            (1_234_568.5, "1234569"),
            (1_234_567.5, "1234568"),
            (-1_234_568.5, "-1234569"),
            (123_456.75, "123456.8"),
            (3.051_757_8e-5, "3.051758E-05"),
        ];
        for &(value, text) in cases {
            assert_eq!(single(value), text, "{value:e}");
        }
        assert_eq!(single(f32::NAN), "NaN");
        assert_eq!(single(f32::INFINITY), "Infinity");
        assert_eq!(single(f32::NEG_INFINITY), "-Infinity");
    }

    #[test]
    fn double_matches_the_runtime() {
        let cases: &[(f64, &str)] = &[
            (0.1 + 0.2, "0.3"),
            (1.0 / 3.0, "0.333333333333333"),
            (35.363_262_1, "35.3632621"),
            (1e15, "1E+15"),
            (1e16, "1E+16"),
            (123_456_789_012_345_680.0, "1.23456789012346E+17"),
            (1e-5, "1E-05"),
            (5e-324, "4.94065645841247E-324"),
            (f64::MAX, "1.79769313486232E+308"),
            (-35.363_262_1, "-35.3632621"),
            (1_234_567_890_123_456.0, "1.23456789012346E+15"),
            (999_999_999_999_999.9, "1E+15"),
            (100_000_000_000_000.5, "100000000000001"),
            (100_000_000_000_001.5, "100000000000002"),
            (100_000_000_000_000.25, "100000000000000"),
            (0.123_456_789_012_345_5, "0.123456789012345"),
            (-100_000_000_000_000.5, "-100000000000001"),
            (1e21, "1E+21"),
            (1e-7, "1E-07"),
            (123_456_789_012_345.0, "123456789012345"),
            (12.34, "12.34"),
        ];
        for &(value, text) in cases {
            assert_eq!(double(value), text, "{value:e}");
        }
    }

    #[test]
    fn roundtrip_takes_seventeen_digits_only_when_fifteen_lose_the_value() {
        assert_eq!(double_roundtrip(0.1 + 0.2), "0.30000000000000004");
        assert_eq!(double_roundtrip(1.0 / 3.0), "0.33333333333333331");
        assert_eq!(
            double_roundtrip(999_999_999_999_999.9),
            "999999999999999.88"
        );
        assert_eq!(double_roundtrip(-27.513_363_8), "-27.5133638");
        assert_eq!(single_roundtrip(0.999_999_97), "0.99999994");
        assert_eq!(single_roundtrip(2.0), "2");
        assert_eq!(single_roundtrip(16_777_216.0), "16777216");
    }

    /// Every value here was read by mono's `double.Parse`, which is .NET Framework's: the first
    /// four a unit in the last place from the nearest double, as the golden `.mat` files hold them.
    #[test]
    fn parsing_rounds_as_the_runtime_rounds() {
        let cases: &[(&str, u64)] = &[
            ("-0.0007147721", (-0.000_714_772_100_000_000_1f64).to_bits()),
            ("0.0001759681", 0.000_175_968_100_000_000_01f64.to_bits()),
            ("0.2019788", 0.201_978_800_000_000_01f64.to_bits()),
            ("-9.793323", (-9.793_323_000_000_001f64).to_bits()),
            ("35.3632621", 35.363_262_1f64.to_bits()),
            ("0.1", 0.1f64.to_bits()),
            ("1E-05", 1e-5f64.to_bits()),
            ("123456789012345678", 123_456_789_012_345_680f64.to_bits()),
        ];
        for &(text, bits) in cases {
            assert_eq!(parse_double(text).map(f64::to_bits), Some(bits), "{text}");
        }
    }

    #[test]
    fn parsing_follows_number_styles() {
        assert_eq!(parse_double(" 12.5\r\n"), Some(12.5));
        assert_eq!(parse_double("-1E-05"), Some(-1e-5));
        assert_eq!(parse_double("+.5"), Some(0.5));
        assert_eq!(parse_double("5."), Some(5.0));
        assert_eq!(parse_double("1,000.5"), Some(1000.5));
        assert_eq!(parse_double("(5)"), None);
        assert_eq!(parse_double_any("(5)"), Some(-5.0));
        assert_eq!(parse_double_any("5-"), Some(-5.0));
        assert_eq!(parse_double("5-"), None);
        assert_eq!(parse_double("- 5"), None);
        assert_eq!(parse_double("1E"), None);
        assert_eq!(parse_double("12\0\0"), Some(12.0));
        assert_eq!(parse_double("Loiter"), None);
        assert_eq!(parse_double(""), None);
        assert_eq!(parse_double(" Infinity "), Some(f64::INFINITY));
        assert!(parse_double("NaN").is_some_and(f64::is_nan));
        assert_eq!(parse_double("1e400"), None);
        assert_eq!(parse_i32(" 42\r\n"), Some(42));
        assert_eq!(parse_i32("-7"), Some(-7));
        assert_eq!(parse_i32("4.0"), None);
        assert_eq!(parse_i32("3000000000"), None);
        assert_eq!(parse_i64("3000000000"), Some(3_000_000_000));
    }

    #[test]
    fn parameter_names_sort_as_the_invariant_culture_sorts_them() {
        let mut names = [
            "BATTERY",
            "ATC_ANGLE_BOOST",
            "BATT2_MONITOR",
            "ATC_ANG_YAW_P",
            "BATT_MONITOR",
            "RC1_DZ",
            "RC10_DZ",
            "RC_OPTIONS",
            "INS_ACC2OFFS_X",
            "INS_ACC2_ID",
            "INS_ACCEL_FILTER",
        ];
        names.sort_by(|a, b| culture_compare(a, b));
        assert_eq!(
            names,
            [
                "ATC_ANG_YAW_P",
                "ATC_ANGLE_BOOST",
                "BATT_MONITOR",
                "BATT2_MONITOR",
                "BATTERY",
                "INS_ACC2_ID",
                "INS_ACC2OFFS_X",
                "INS_ACCEL_FILTER",
                "RC_OPTIONS",
                "RC1_DZ",
                "RC10_DZ",
            ]
        );
    }

    #[test]
    fn ascii_decoding_replaces_high_bytes() {
        assert_eq!(ascii(&[0x41, 0x80, 0xFF, 0x42]), "A??B");
        assert_eq!(trim_nul("\0ab\0c\0\0"), "ab\0c");
    }
}
