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

//! .NET's numbers as the Survey (Grid) dialog uses them.
//!
//! `Grid/GridUI.cs` keeps every setting in a `NumericUpDown`, whose `Value` is a `System.Decimal`,
//! and moves it between `decimal`, `double`, `float` and `int` at every use:
//! `(double)NUM_Distance.Value` into the grid, `(decimal)((1 - overlap / 100.0f) * viewheight)`
//! back out of `doCalc`,
//! `(float)NUM_spacing.Value` into a `DO_SET_CAM_TRIGG_DIST`. Each conversion rounds its own way,
//! and the Stats labels are .NET's custom format strings (`"0.##"`, `"#.#"`, `"00"`), which round
//! the value's fifteen significant digits rather than its binary value. This module is those
//! conversions and formats, transliterated from the runtime's own code, so the dialog's numbers are
//! the C#'s to the bit; `crates/mp-mission/tests/gridui_vectors.rs` holds them to the dialog run
//! under mono.
//!
//! The runtime code is .NET's `DecCalc` (`VarDecFromR8`, `VarDecFromR4`, `VarR8FromDec`, the same
//! algorithms as the .NET Framework's `oleaut32` ones) and `Number.Formatting` (`RoundNumber`,
//! `NumberToStringFormat`, `FormatGeneral`), which the .NET Framework and mono share.

use std::cmp::Ordering;

/// `DEC_SCALE_MAX`: a decimal has at most 28 digits after the point.
const SCALE_MAX: u32 = 28;
/// The largest mantissa a decimal holds: 96 bits.
const MANTISSA_MAX: u128 = (1 << 96) - 1;
/// `s_doublePowers10`: the doubles `1e0` to `1e28`, as the C# compiler writes the literals.
const DOUBLE_POWERS10: [f64; 29] = [
    1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16,
    1e17, 1e18, 1e19, 1e20, 1e21, 1e22, 1e23, 1e24, 1e25, 1e26, 1e27, 1e28,
];

/// `10^n` as an integer, for `n` up to 38.
const fn pow10(n: u32) -> u128 {
    let mut value: u128 = 1;
    let mut i = 0;
    while i < n {
        value *= 10;
        i += 1;
    }
    value
}

/// The double `10^power` from [`DOUBLE_POWERS10`], for a power known to be in range.
fn double_power10(power: i32) -> f64 {
    usize::try_from(power)
        .ok()
        .and_then(|index| DOUBLE_POWERS10.get(index))
        .copied()
        .unwrap_or(f64::NAN)
}

/// A `System.Decimal`: a sign, a 96-bit magnitude and a power of ten to divide it by.
///
/// The scale is kept as the C# keeps it, trailing zeros and all, because `ToString()` shows it:
/// `50m` prints "50" and `50.0m` prints "50.0", and `savesettings` writes what `ToString()` prints.
#[derive(Debug, Clone, Copy, Default)]
pub struct Decimal {
    negative: bool,
    magnitude: u128,
    scale: u32,
}

impl Decimal {
    /// Zero, scale 0.
    pub const ZERO: Self = Self {
        negative: false,
        magnitude: 0,
        scale: 0,
    };

    /// `mantissa / 10^scale`. The mantissa must fit in 96 bits and the scale be at most 28, as a
    /// `decimal`'s must; `new decimal(int[])` in a Designer file is one of these.
    #[must_use]
    pub const fn new(mantissa: i128, scale: u32) -> Self {
        Self {
            negative: mantissa < 0,
            magnitude: mantissa.unsigned_abs(),
            scale,
        }
    }

    /// A whole number.
    #[must_use]
    pub const fn from_int(value: i64) -> Self {
        Self::new(value as i128, 0)
    }

    /// The signed mantissa.
    #[must_use]
    pub const fn mantissa(self) -> i128 {
        // The magnitude never exceeds 96 bits, so it fits an i128 either way round.
        #[allow(clippy::cast_possible_wrap)]
        let magnitude = self.magnitude as i128;
        if self.negative { -magnitude } else { magnitude }
    }

    /// The number of digits after the point.
    #[must_use]
    pub const fn scale(self) -> u32 {
        self.scale
    }

    /// Whether the value is zero, whatever its scale and sign.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.magnitude == 0
    }

    /// `(decimal)double`: `DecCalc.VarDecFromR8`. The double is rounded to fifteen significant
    /// digits - "the R8 format has only 15 digits of precision, and we want to keep garbage digits
    /// out of the Decimal" - by scaling it in double arithmetic and rounding half to even, then as
    /// many trailing zeros are taken off as the scaling put on. `None` where the C# throws
    /// `OverflowException`: a NaN, an infinity, or a magnitude past a decimal's.
    #[must_use]
    pub fn from_f64(input: f64) -> Option<Self> {
        // C#: DecCalc.VarDecFromR8. DBLBIAS is 1022.
        let biased = i32::try_from((input.to_bits() >> 52) & 0x7FF).unwrap_or(0);
        let exp = biased - 1022;
        if exp < -94 {
            return Some(Self::ZERO);
        }
        if exp > 96 {
            return None;
        }
        let negative = input < 0.0;
        let mut dbl = input.abs();
        let mut power = 14 - ((exp * 19728) >> 16);
        if power >= 0 {
            // We have less than 15 digits, scale input up.
            if power > 28 {
                power = 28;
            }
            dbl *= double_power10(power);
        } else if power != -1 || dbl >= 1e15 {
            dbl /= double_power10(-power);
        } else {
            power = 0; // didn't scale it
        }
        if dbl < 1e14 && power < 28 {
            dbl *= 10.0;
            power += 1;
        }
        // Round to int64: truncate, then half to even on what is left.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mut mant = dbl as i64 as u64;
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_wrap)]
        let rest = dbl - (mant as i64) as f64;
        if rest > 0.5 || (rest == 0.5 && mant & 1 != 0) {
            mant += 1;
        }
        Self::from_rounded(negative, u128::from(mant), power, 14)
    }

    /// `(decimal)float`: `DecCalc.VarDecFromR4`, the same with seven digits.
    #[must_use]
    pub fn from_f32(input: f32) -> Option<Self> {
        // C#: DecCalc.VarDecFromR4. SNGBIAS is 126.
        let biased = i32::try_from((input.to_bits() >> 23) & 0xFF).unwrap_or(0);
        let exp = biased - 126;
        if exp < -94 {
            return Some(Self::ZERO);
        }
        if exp > 96 {
            return None;
        }
        let negative = input < 0.0;
        let mut dbl = f64::from(input.abs());
        let mut power = 6 - ((exp * 19728) >> 16);
        if power >= 0 {
            if power > 28 {
                power = 28;
            }
            dbl *= double_power10(power);
        } else if power != -1 || dbl >= 1e7 {
            dbl /= double_power10(-power);
        } else {
            power = 0;
        }
        if dbl < 1e6 && power < 28 {
            dbl *= 10.0;
            power += 1;
        }
        // Round to integer.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mut mant = dbl as i32 as u32;
        let rest = dbl - f64::from(mant);
        if rest > 0.5 || (rest == 0.5 && mant & 1 != 0) {
            mant += 1;
        }
        Self::from_rounded(negative, u128::from(mant), power, 6)
    }

    /// The end of `VarDecFromR8` and `VarDecFromR4`: a rounded integer mantissa at `power`, with
    /// up to `most` trailing zeros factored out - never more than the power put there.
    fn from_rounded(negative: bool, mant: u128, power: i32, most: u32) -> Option<Self> {
        if mant == 0 {
            return Some(Self::ZERO);
        }
        if power < 0 {
            let magnitude = mant.checked_mul(pow10(power.unsigned_abs()))?;
            if magnitude > MANTISSA_MAX {
                return None;
            }
            return Some(Self {
                negative,
                magnitude,
                scale: 0,
            });
        }
        let mut power = power.unsigned_abs();
        let mut lmax = power.min(most);
        let mut magnitude = mant;
        // The C# tries 8, 4, 2 and 1 in turn; that takes off every trailing zero up to `lmax`.
        while lmax > 0 && magnitude.is_multiple_of(10) {
            magnitude /= 10;
            power -= 1;
            lmax -= 1;
        }
        Some(Self {
            negative,
            magnitude,
            scale: power,
        })
    }

    /// `(double)decimal`: `DecCalc.VarR8FromDec`, the low 64 bits and the high 32 as doubles,
    /// divided by the double power of ten.
    #[must_use]
    pub fn to_f64(self) -> f64 {
        // C#: ((double)value.Low64 + (double)value.High * ds2to64) / s_doublePowers10[value.Scale]
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
        let low = (self.magnitude & u128::from(u64::MAX)) as u64 as f64;
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
        let high = (self.magnitude >> 64) as u64 as f64;
        let value = (low + high * 18_446_744_073_709_551_616.0)
            / double_power10(i32::try_from(self.scale).unwrap_or(0));
        if self.negative { -value } else { value }
    }

    /// `(float)decimal`: `DecCalc.VarR4FromDec`, which is `(float)VarR8FromDec`.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn to_f32(self) -> f32 {
        self.to_f64() as f32
    }

    /// `(int)decimal`: truncated toward zero. `None` where the C# throws `OverflowException`.
    #[must_use]
    pub fn to_i32(self) -> Option<i32> {
        let whole = self.magnitude / pow10(self.scale);
        let whole = i64::try_from(whole).ok()?;
        i32::try_from(if self.negative { -whole } else { whole }).ok()
    }

    /// `Math.Round(decimal)` and `decimal.Round`: to a whole number, a half to the even one.
    #[must_use]
    pub fn round_even(self) -> Self {
        let divisor = pow10(self.scale);
        let mut whole = self.magnitude / divisor;
        let rest = self.magnitude % divisor;
        match (rest * 2).cmp(&divisor) {
            Ordering::Greater => whole += 1,
            Ordering::Equal if !whole.is_multiple_of(2) => whole += 1,
            _ => {}
        }
        Self {
            negative: self.negative && whole != 0,
            magnitude: whole,
            scale: 0,
        }
    }

    /// `decimal + decimal`, at the larger scale. `None` where the C# throws `OverflowException`
    /// (`UpButton` catches it and takes the maximum).
    #[must_use]
    pub fn checked_add(self, other: Self) -> Option<Self> {
        let scale = self.scale.max(other.scale);
        let a = i128::try_from(self.magnitude.checked_mul(pow10(scale - self.scale))?).ok()?;
        let b = i128::try_from(other.magnitude.checked_mul(pow10(scale - other.scale))?).ok()?;
        let a = if self.negative { -a } else { a };
        let b = if other.negative { -b } else { b };
        let sum = a.checked_add(b)?;
        if sum.unsigned_abs() > MANTISSA_MAX {
            return None;
        }
        Some(Self::new(sum, scale))
    }

    /// `decimal - decimal`.
    #[must_use]
    pub fn checked_sub(self, other: Self) -> Option<Self> {
        self.checked_add(Self {
            negative: !other.negative,
            ..other
        })
    }

    /// `decimal.Parse(text, NumberStyles.Number)` in an invariant-style culture, which is what
    /// `NumericUpDown.ParseEditText` and `loadsetting` call: white space either side, a sign in
    /// front or behind, thousands separators in the whole part, a decimal point, no exponent.
    /// `None` where the C# throws.
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        let mut text = text.trim();
        let mut negative = false;
        if let Some(rest) = text.strip_prefix('-') {
            negative = true;
            text = rest;
        } else if let Some(rest) = text.strip_prefix('+') {
            text = rest;
        } else if let Some(rest) = text.strip_suffix('-') {
            negative = true;
            text = rest;
        } else if let Some(rest) = text.strip_suffix('+') {
            text = rest;
        }
        let (whole, fraction) = text.split_once('.').unwrap_or((text, ""));
        let whole: String = whole.chars().filter(|c| *c != ',').collect();
        if whole.is_empty() && fraction.is_empty() {
            return None;
        }
        if !whole.chars().all(|c| c.is_ascii_digit())
            || !fraction.chars().all(|c| c.is_ascii_digit())
        {
            return None;
        }
        // Past 28 places the C# rounds; a typed number that long is not one this dialog meets.
        let fraction = fraction.get(..fraction.len().min(SCALE_MAX as usize))?;
        let mut magnitude: u128 = 0;
        for digit in whole.bytes().chain(fraction.bytes()) {
            magnitude = magnitude
                .checked_mul(10)?
                .checked_add(u128::from(digit - b'0'))?;
            if magnitude > MANTISSA_MAX {
                return None;
            }
        }
        Some(Self {
            negative: negative && magnitude != 0,
            magnitude,
            scale: u32::try_from(fraction.len()).ok()?,
        })
    }

    /// The value's exact digits, for formatting.
    fn number(self) -> Number {
        let digits = self.magnitude.to_string().into_bytes();
        let scale =
            i32::try_from(digits.len()).unwrap_or(0) - i32::try_from(self.scale).unwrap_or(0);
        Number::new(self.negative, digits, scale)
    }

    /// `decimal.ToString()`: every digit, the scale's trailing zeros included.
    #[must_use]
    pub fn to_text(self) -> String {
        let digits = self.magnitude.to_string();
        let scale = self.scale as usize;
        let padded = if digits.len() <= scale {
            format!("{}{digits}", "0".repeat(scale + 1 - digits.len()))
        } else {
            digits
        };
        let (whole, fraction) = padded.split_at(padded.len() - scale);
        let sign = if self.negative && self.magnitude != 0 {
            "-"
        } else {
            ""
        };
        if fraction.is_empty() {
            format!("{sign}{whole}")
        } else {
            format!("{sign}{whole}.{fraction}")
        }
    }

    /// `decimal.ToString("F<places>")`: what a `NumericUpDown` shows at `DecimalPlaces`.
    #[must_use]
    pub fn to_fixed(self, places: u32) -> String {
        let pattern = if places == 0 {
            "0".to_owned()
        } else {
            format!("0.{}", "0".repeat(places as usize))
        };
        self.format(&pattern)
    }

    /// `decimal.ToString(format)` for a custom format of the shapes the dialog uses.
    #[must_use]
    pub fn format(self, pattern: &str) -> String {
        custom_format(self.number(), pattern)
    }
}

impl PartialEq for Decimal {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Decimal {}

impl PartialOrd for Decimal {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Decimal {
    /// By value: `50m == 50.0m`, as the C#'s operators compare.
    fn cmp(&self, other: &Self) -> Ordering {
        let sign = |d: &Self| {
            if d.magnitude == 0 {
                0
            } else if d.negative {
                -1
            } else {
                1
            }
        };
        let (a_sign, b_sign) = (sign(self), sign(other));
        if a_sign != b_sign || a_sign == 0 {
            return a_sign.cmp(&b_sign);
        }
        let scale = self.scale.max(other.scale);
        let magnitudes = self
            .magnitude
            .checked_mul(pow10(scale - self.scale))
            .zip(other.magnitude.checked_mul(pow10(scale - other.scale)));
        let by_magnitude = magnitudes.map_or_else(
            || {
                self.to_f64()
                    .abs()
                    .partial_cmp(&other.to_f64().abs())
                    .unwrap_or(Ordering::Equal)
            },
            |(a, b)| a.cmp(&b),
        );
        if a_sign < 0 {
            by_magnitude.reverse()
        } else {
            by_magnitude
        }
    }
}

/// .NET's `NUMBER`: a sign, significant digits, and where the point goes - the value is
/// `0.d1d2d3... * 10^scale`.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Number {
    negative: bool,
    /// ASCII digits, without trailing zeros; empty for zero.
    digits: Vec<u8>,
    scale: i32,
}

impl Number {
    fn new(negative: bool, mut digits: Vec<u8>, scale: i32) -> Self {
        // Leading zeros move the point; trailing ones are dropped, as the runtime drops them.
        let leading = digits.iter().take_while(|d| **d == b'0').count();
        digits.drain(..leading);
        while digits.last() == Some(&b'0') {
            digits.pop();
        }
        let scale = if digits.is_empty() {
            0
        } else {
            scale - i32::try_from(leading).unwrap_or(0)
        };
        Self {
            negative: negative && !digits.is_empty(),
            digits,
            scale,
        }
    }

    /// `DoubleToNumber` / `FloatToNumber`: the value's first `precision` significant digits,
    /// correctly rounded. `None` for a NaN or an infinity, which are formatted by name.
    fn from_float(value: f64, precision: usize) -> Option<Self> {
        if !value.is_finite() {
            return None;
        }
        if value == 0.0 {
            return Some(Self::new(false, Vec::new(), 0));
        }
        let text = format!("{:.*e}", precision - 1, value.abs());
        let (mantissa, exponent) = text.split_once('e')?;
        let exponent: i32 = exponent.parse().ok()?;
        let digits: Vec<u8> = mantissa.bytes().filter(u8::is_ascii_digit).collect();
        Some(Self::new(value < 0.0, digits, exponent + 1))
    }

    /// `RoundNumber(ref number, pos)`: keep `pos` digits, rounding up on a fifth digit, and drop
    /// the trailing zeros; nothing left is zero, unsigned.
    fn round(&mut self, pos: i32) {
        // A position before the first digit rounds nothing up: the C#'s `i == pos` never holds.
        let Ok(keep) = usize::try_from(pos) else {
            self.digits.clear();
            self.scale = 0;
            self.negative = false;
            return;
        };
        if keep >= self.digits.len() {
            return;
        }
        let round_up = self.digits.get(keep).is_some_and(|d| *d >= b'5');
        self.digits.truncate(keep);
        if round_up {
            // Carry from the last kept digit; past the first one it becomes a new leading 1.
            let mut i = keep;
            loop {
                if i == 0 {
                    self.digits.insert(0, b'1');
                    self.scale += 1;
                    break;
                }
                i -= 1;
                if let Some(digit) = self.digits.get_mut(i) {
                    if *digit == b'9' {
                        *digit = b'0';
                    } else {
                        *digit += 1;
                        break;
                    }
                }
            }
        }
        while self.digits.last() == Some(&b'0') {
            self.digits.pop();
        }
        if self.digits.is_empty() {
            self.scale = 0;
            self.negative = false;
        }
    }

    /// The digit at `index`, or `'0'` past the end.
    fn digit(&self, index: i32) -> u8 {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.digits.get(i))
            .copied()
            .unwrap_or(b'0')
    }
}

/// `NumberToStringFormat` for one section of `0`, `#`, one `.` and literal text after the last
/// placeholder - `"#"`, `"0.##"`, `"#.#"`, `"00"`, `"0.00 cm"` - which is every format the dialog
/// uses.
fn custom_format(mut number: Number, pattern: &str) -> String {
    let body_end = pattern
        .find(|c: char| c != '0' && c != '#' && c != '.')
        .unwrap_or(pattern.len());
    let (body, literal) = pattern.split_at(body_end);
    let (whole, fraction) = body.split_once('.').unwrap_or((body, ""));
    // The fewest whole digits: from the first '0' to the point.
    let min_whole = whole.find('0').map_or(0, |first| whole.len() - first);
    // The fewest fraction digits: up to the last '0'.
    let min_fraction = fraction.rfind('0').map_or(0, |last| last + 1);
    let max_fraction = i32::try_from(fraction.len()).unwrap_or(0);

    number.round(number.scale + max_fraction);

    let mut out = String::new();
    if number.negative {
        out.push('-');
    }
    let whole_digits = number.scale.max(i32::try_from(min_whole).unwrap_or(0));
    for position in 0..whole_digits {
        // Leading placeholders beyond the value's own digits are zeros.
        let index = number.scale - whole_digits + position;
        out.push(if index < 0 {
            '0'
        } else {
            char::from(number.digit(index))
        });
    }
    let mut fraction_text = String::new();
    for position in 0..max_fraction {
        fraction_text.push(char::from(number.digit(number.scale + position)));
    }
    while fraction_text.len() > min_fraction && fraction_text.ends_with('0') {
        fraction_text.pop();
    }
    if !fraction_text.is_empty() {
        out.push('.');
        out.push_str(&fraction_text);
    }
    out.push_str(literal);
    out
}

/// `double.ToString(format)` for a custom format: fifteen significant digits, then the format.
#[must_use]
pub fn format_f64(value: f64, pattern: &str) -> String {
    Number::from_float(value, 15).map_or_else(|| non_finite(value), |n| custom_format(n, pattern))
}

/// `float.ToString(format)` for a custom format: seven significant digits, then the format.
#[must_use]
pub fn format_f32(value: f32, pattern: &str) -> String {
    Number::from_float(f64::from(value), 7).map_or_else(
        || non_finite(f64::from(value)),
        |n| custom_format(n, pattern),
    )
}

/// What .NET prints for a value that is not a number, whatever the format.
fn non_finite(value: f64) -> String {
    if value.is_nan() {
        "NaN".to_owned()
    } else if value > 0.0 {
        "Infinity".to_owned()
    } else {
        "-Infinity".to_owned()
    }
}

/// `double.ToString()`: `"G"`, fifteen significant digits.
#[must_use]
pub fn general_f64(value: f64) -> String {
    Number::from_float(value, 15).map_or_else(|| non_finite(value), |n| general(n, 15))
}

/// `float.ToString()`: `"G"`, seven significant digits.
#[must_use]
pub fn general_f32(value: f32) -> String {
    Number::from_float(f64::from(value), 7)
        .map_or_else(|| non_finite(f64::from(value)), |n| general(n, 7))
}

/// `FormatGeneral`: the digits with the point where it falls, or scientific notation when the
/// exponent is past the precision or below -4.
fn general(number: Number, precision: i32) -> String {
    let mut out = String::new();
    if number.negative {
        out.push('-');
    }
    if number.digits.is_empty() {
        out.push('0');
        return out;
    }
    let scientific = number.scale > precision || number.scale < -3;
    let point = if scientific { 1 } else { number.scale };
    if point > 0 {
        for index in 0..point {
            out.push(char::from(number.digit(index)));
        }
    } else {
        out.push('0');
    }
    let len = i32::try_from(number.digits.len()).unwrap_or(0);
    if len > point {
        out.push('.');
        for _ in point..0 {
            out.push('0');
        }
        for index in point.max(0)..len {
            out.push(char::from(number.digit(index)));
        }
    }
    if scientific {
        let exponent = number.scale - 1;
        out.push('E');
        out.push(if exponent < 0 { '-' } else { '+' });
        out.push_str(&format!("{:02}", exponent.abs()));
    }
    out
}

/// `(int)double` on the x86 and x64 runtimes: truncated, and `int.MinValue` for a NaN or a value
/// out of range (`cvttsd2si`'s answer) where Rust's `as` would saturate.
#[must_use]
pub fn to_int(value: f64) -> i32 {
    if value.is_nan() || value >= 2_147_483_648.0 || value <= -2_147_483_649.0 {
        i32::MIN
    } else {
        #[allow(clippy::cast_possible_truncation)]
        let truncated = value as i32;
        truncated
    }
}

/// `double.Parse(text)` in an invariant-style culture: white space either side, a sign, thousands
/// separators before the point, a point, an exponent. `None` where the C# throws.
#[must_use]
pub fn parse_f64(text: &str) -> Option<f64> {
    let text = text.trim();
    if text.is_empty() {
        return None;
    }
    let (whole, rest) = match text.find(['.', 'e', 'E']) {
        Some(at) => text.split_at(at),
        None => (text, ""),
    };
    let whole: String = whole.chars().filter(|c| *c != ',').collect();
    let joined = format!("{whole}{rest}");
    // Rust's parser takes "inf", "nan" and "infinity" in any case, which .NET does not.
    if joined
        .chars()
        .any(|c| !(c.is_ascii_digit() || matches!(c, '.' | 'e' | 'E' | '+' | '-')))
    {
        return None;
    }
    joined.parse::<f64>().ok()
}

/// `float.Parse(text)`: .NET parses the double and narrows it.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn parse_f32(text: &str) -> Option<f32> {
    let value = parse_f64(text)? as f32;
    value.is_finite().then_some(value)
}

/// `int.Parse(text)`: white space either side, a sign, digits.
#[must_use]
pub fn parse_i32(text: &str) -> Option<i32> {
    let text = text.trim();
    let digits = text.strip_prefix(['-', '+']).unwrap_or(text);
    if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    text.parse().ok()
}

/// `bool.Parse(text)`: "True" or "False" in any case, with white space either side.
#[must_use]
pub fn parse_bool(text: &str) -> Option<bool> {
    let text = text.trim();
    if text.eq_ignore_ascii_case("true") {
        Some(true)
    } else if text.eq_ignore_ascii_case("false") {
        Some(false)
    } else {
        None
    }
}

/// `bool.ToString()`.
#[must_use]
pub const fn bool_text(value: bool) -> &'static str {
    if value { "True" } else { "False" }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dec(text: &str) -> Decimal {
        Decimal::parse(text).expect("a decimal")
    }

    #[test]
    fn a_decimal_prints_its_scale() {
        assert_eq!(Decimal::new(500, 1).to_text(), "50.0");
        assert_eq!(Decimal::from_int(50).to_text(), "50");
        assert_eq!(Decimal::new(5, 3).to_text(), "0.005");
        assert_eq!(Decimal::new(-125, 2).to_text(), "-1.25");
        assert_eq!(dec("  1,000.50 ").to_text(), "1000.50");
        assert_eq!(dec("12-").to_text(), "-12");
        assert!(Decimal::parse("1e3").is_none());
        assert!(Decimal::parse("").is_none());
        assert!(Decimal::parse(".").is_none());
        assert_eq!(dec(".5").to_text(), "0.5");
    }

    #[test]
    fn decimals_compare_by_value() {
        assert_eq!(dec("50"), dec("50.00"));
        assert!(dec("-1") < dec("0"));
        assert!(dec("-2") < dec("-1.5"));
        assert!(dec("0.3") > dec("0.29"));
        assert_eq!(Decimal::ZERO, dec("-0.0"));
    }

    #[test]
    fn double_to_decimal_keeps_fifteen_digits() {
        // (decimal)(double)0.1 is 0.1: the garbage digits past fifteen are rounded away.
        assert_eq!(Decimal::from_f64(0.1).expect("0.1").to_text(), "0.1");
        assert_eq!(
            Decimal::from_f64(179.999_999_999_999_97)
                .expect("a bearing")
                .to_text(),
            "180"
        );
        assert_eq!(
            Decimal::from_f64(46.199_999_999_999_996)
                .expect("a spacing")
                .to_text(),
            "46.2"
        );
        assert_eq!(
            Decimal::from_f64(std::f64::consts::PI)
                .expect("pi")
                .to_text(),
            "3.14159265358979"
        );
        assert_eq!(
            Decimal::from_f64(1e20).expect("big").to_text(),
            "100000000000000000000"
        );
        assert!(Decimal::from_f64(f64::NAN).is_none());
        assert!(Decimal::from_f64(1e30).is_none());
        assert_eq!(Decimal::from_f64(1e-30).expect("tiny"), Decimal::ZERO);
    }

    #[test]
    fn float_to_decimal_keeps_seven_digits() {
        assert_eq!(Decimal::from_f32(6.3).expect("6.3").to_text(), "6.3");
        assert_eq!(Decimal::from_f32(12.0).expect("12").to_text(), "12");
        assert_eq!(Decimal::from_f32(0.1).expect("0.1").to_text(), "0.1");
    }

    #[test]
    fn decimal_to_double_divides_once() {
        assert_eq!(dec("46.2").to_f64(), 46.2);
        assert_eq!(dec("0.3").to_f64(), 0.3);
        assert_eq!(dec("-999").to_f64(), -999.0);
    }

    #[test]
    fn decimal_rounding_and_truncation() {
        assert_eq!(dec("44.5").round_even().to_text(), "44");
        assert_eq!(dec("45.5").round_even().to_text(), "46");
        assert_eq!(dec("-0.4").round_even().to_text(), "0");
        assert_eq!(dec("2.9").to_i32(), Some(2));
        assert_eq!(dec("-2.9").to_i32(), Some(-2));
        assert_eq!(dec("3000000000").to_i32(), None);
        assert_eq!(dec("44.5").to_fixed(0), "45");
        assert_eq!(dec("50").to_fixed(1), "50.0");
        assert_eq!(dec("46.2").format("0.#"), "46.2");
        assert_eq!(dec("0").format("0.#"), "0");
        assert_eq!(
            dec("1.5").checked_add(dec("0.25")).map(Decimal::to_text),
            Some("1.75".to_owned())
        );
    }

    #[test]
    fn custom_formats_round_fifteen_digits_half_away() {
        assert_eq!(format_f64(0.0, "#"), "");
        assert_eq!(format_f64(0.4, "#"), "");
        assert_eq!(format_f64(998_376.4, "#"), "998376");
        assert_eq!(format_f64(0.5, "#.#"), ".5");
        assert_eq!(format_f64(22.194_9, "0.##"), "22.19");
        assert_eq!(format_f64(2.0, "0.##"), "2");
        assert_eq!(format_f64(2.345, "0.00"), "2.35");
        assert_eq!(format_f64(59.7, "00"), "60");
        assert_eq!(format_f64(5.3, "00"), "05");
        assert_eq!(format_f64(1.155, "0.00 cm"), "1.16 cm");
        assert_eq!(format_f64(-0.4, "0"), "0");
        assert_eq!(format_f64(0.0006, "0.##"), "0");
        assert_eq!(format_f64(0.006, "0.##"), "0.01");
        assert_eq!(format_f64(9.996, "0.##"), "10");
        assert_eq!(format_f64(f64::NAN, "0.00"), "NaN");
        assert_eq!(format_f32(5.099_9, "0"), "5");
    }

    #[test]
    fn general_formats() {
        assert_eq!(general_f64(63.266_139_984_130_86), "63.2661399841309");
        assert_eq!(general_f64(1082.0), "1082");
        assert_eq!(general_f64(0.0), "0");
        assert_eq!(general_f64(1e20), "1E+20");
        assert_eq!(general_f64(0.000_12), "0.00012");
        assert_eq!(general_f64(0.000_012), "1.2E-05");
        assert_eq!(general_f32(6.16), "6.16");
        assert_eq!(general_f32(4608.0), "4608");
    }

    #[test]
    fn casts_and_parses() {
        assert_eq!(to_int(f64::NAN), i32::MIN);
        assert_eq!(to_int(f64::INFINITY), i32::MIN);
        assert_eq!(to_int(-2.9), -2);
        assert_eq!(parse_f64(" 6.16 "), Some(6.16));
        assert_eq!(parse_f64("1,000.5"), Some(1000.5));
        assert_eq!(parse_f64("inf"), None);
        assert_eq!(parse_f64(""), None);
        assert_eq!(parse_i32(" 4608 "), Some(4608));
        assert_eq!(parse_i32("4608.5"), None);
        assert_eq!(parse_bool("true"), Some(true));
        assert_eq!(parse_bool("yes"), None);
    }
}
