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

//! Python 2's values and their text, as `DataflashLog.py` and the checks see them.
//!
//! `Format.trycastToFormatType` leaves a field an `int`, a `float` or, when the cast fails, the
//! text itself (`DataflashLog.py:31-43`); the checks then compare, add and divide those with
//! Python 2's rules - an `int` divided by an `int` is floored, a number is below any text, a
//! text in arithmetic raises - and print them with `%s` (`str`: twelve significant digits),
//! `repr` (the shortest round trip), `%.2f` and `%d`. A check that goes wrong raises, and
//! `TestSuite.run` shows `str(e)` as an UNKNOWN test's message (`LogAnalyzer.py:85-92`);
//! [`PyError`] carries that text.

/// `str(e)` of the exception a check raised, and whether it was a `KeyError` - the one some
/// checks catch themselves (`except KeyError as e`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct PyError {
    /// A `KeyError`, as against any other exception.
    pub(crate) key_error: bool,
    /// `str(e)`: a `KeyError`'s is its key's `repr`.
    pub(crate) text: String,
}

impl PyError {
    /// `KeyError(key)`: `str(e)` is `repr(key)`, quotes included.
    pub(crate) fn key(key: &str) -> Self {
        Self {
            key_error: true,
            text: repr_str(key),
        }
    }

    /// `KeyError(None)`, a lookup by a name that was never found.
    pub(crate) fn key_none() -> Self {
        Self {
            key_error: true,
            text: "None".to_owned(),
        }
    }

    /// Any other exception, with its message.
    pub(crate) fn other(text: impl Into<String>) -> Self {
        Self {
            key_error: false,
            text: text.into(),
        }
    }

    /// `IndexError: list index out of range`.
    pub(crate) fn index() -> Self {
        Self::other("list index out of range")
    }

    /// `AttributeError: 'T' object has no attribute 'name'`.
    pub(crate) fn attribute(type_name: &str, name: &str) -> Self {
        Self::other(format!("'{type_name}' object has no attribute '{name}'"))
    }
}

/// What a check returns, or the exception it raised.
pub(crate) type PyResult<T> = Result<T, PyError>;

/// A field's or a parameter's value as `trycastToFormatType` leaves it: an `int`, a `float`, or
/// the text when the type is a text type or the cast failed.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Value {
    /// `int` (or `long`).
    Int(i64),
    /// `float`.
    Float(f64),
    /// `str`.
    Str(Box<str>),
}

/// An arithmetic operator, named as Python names it in a `TypeError`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Op {
    /// `+`
    Add,
    /// `-`
    Sub,
    /// `*`
    Mul,
    /// `/`
    Div,
}

impl Op {
    const fn symbol(self) -> &'static str {
        match self {
            Self::Add => "+",
            Self::Sub => "-",
            Self::Mul => "*",
            Self::Div => "/",
        }
    }
}

impl Value {
    /// A `str`.
    pub(crate) fn text(text: &str) -> Self {
        Self::Str(text.into())
    }

    /// `Format.trycastToFormatType(value, valueType)`: the format characters of
    /// `libraries/DataFlash/DataFlash.h` cast to `float`, `int` or `str`; a cast that fails leaves
    /// the text. `// DataflashLog.py:31-43`
    pub(crate) fn trycast(text: &str, kind: char) -> Self {
        if "fcCeELd".contains(kind) {
            parse_float(text).map_or_else(|| Self::text(text), Self::Float)
        } else if "bBhHiIMQq".contains(kind) {
            parse_int(text).map_or_else(|| Self::text(text), Self::Int)
        } else {
            Self::text(text)
        }
    }

    /// `type(x).__name__`.
    pub(crate) const fn type_name(&self) -> &'static str {
        match self {
            Self::Int(_) => "int",
            Self::Float(_) => "float",
            Self::Str(_) => "str",
        }
    }

    /// `isinstance(x, float) and math.isnan(x)`.
    pub(crate) fn is_nan(&self) -> bool {
        matches!(self, Self::Float(f) if f.is_nan())
    }

    /// `math.isnan(x)`: a text is "a float is required".
    pub(crate) fn isnan(&self) -> PyResult<bool> {
        match self {
            Self::Int(_) => Ok(false),
            Self::Float(f) => Ok(f.is_nan()),
            Self::Str(_) => Err(PyError::other("a float is required")),
        }
    }

    /// `float(x)`.
    pub(crate) fn to_float(&self) -> PyResult<f64> {
        match self {
            Self::Int(i) => Ok(int_to_float(*i)),
            Self::Float(f) => Ok(*f),
            Self::Str(s) => parse_float(s)
                .ok_or_else(|| PyError::other(format!("could not convert string to float: {s}"))),
        }
    }

    /// `int(x)`: a float truncated, a text read in base 10.
    pub(crate) fn to_int(&self) -> PyResult<i64> {
        match self {
            Self::Int(i) => Ok(*i),
            Self::Float(f) => float_to_int(*f),
            Self::Str(s) => parse_int(s).ok_or_else(|| {
                PyError::other(format!("invalid literal for int() with base 10: '{s}'"))
            }),
        }
    }

    /// The number a text is not, for `math.sqrt` and the like: "a float is required".
    pub(crate) fn number(&self) -> PyResult<f64> {
        match self {
            Self::Int(i) => Ok(int_to_float(*i)),
            Self::Float(f) => Ok(*f),
            Self::Str(_) => Err(PyError::other("a float is required")),
        }
    }

    /// `x <op> y` with Python 2's rules: two `int`s stay an `int` (`/` floors), a `float` makes a
    /// `float`, a `str` raises, and division by zero raises.
    pub(crate) fn arith(&self, other: &Self, op: Op) -> PyResult<Self> {
        match (self, other) {
            (Self::Str(_), _) | (_, Self::Str(_)) => Err(PyError::other(format!(
                "unsupported operand type(s) for {}: '{}' and '{}'",
                op.symbol(),
                self.type_name(),
                other.type_name()
            ))),
            (Self::Int(a), Self::Int(b)) => Ok(Self::Int(match op {
                Op::Add => a.wrapping_add(*b),
                Op::Sub => a.wrapping_sub(*b),
                Op::Mul => a.wrapping_mul(*b),
                Op::Div => floor_div(*a, *b)?,
            })),
            _ => {
                let (a, b) = (self.number()?, other.number()?);
                Ok(Self::Float(match op {
                    Op::Add => a + b,
                    Op::Sub => a - b,
                    Op::Mul => a * b,
                    Op::Div => {
                        if b == 0.0 {
                            return Err(PyError::other("float division by zero"));
                        }
                        a / b
                    }
                }))
            }
        }
    }

    /// `x ** 2`.
    pub(crate) fn squared(&self) -> PyResult<Self> {
        match self {
            Self::Str(_) => Err(PyError::other(
                "unsupported operand type(s) for ** or pow(): 'str' and 'int'",
            )),
            _ => self.arith(self, Op::Mul),
        }
    }

    /// `abs(x)`.
    pub(crate) fn abs(&self) -> PyResult<Self> {
        match self {
            Self::Int(i) => Ok(Self::Int(i.wrapping_abs())),
            Self::Float(f) => Ok(Self::Float(f.abs())),
            Self::Str(_) => Err(PyError::other("bad operand type for abs(): 'str'")),
        }
    }

    /// `x == y`: numbers by value, texts by text, a number and a text never.
    pub(crate) fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Int(a), Self::Int(b)) => a == b,
            (Self::Str(a), Self::Str(b)) => a == b,
            (Self::Str(_), _) | (_, Self::Str(_)) => false,
            _ => self.number().ok() == other.number().ok(),
        }
    }

    /// `x < y`: numbers by value (a NaN is below nothing), texts by text, and every number is
    /// below every text, as Python 2 orders unlike types.
    pub(crate) fn lt(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Int(a), Self::Int(b)) => a < b,
            (Self::Str(a), Self::Str(b)) => a < b,
            (Self::Str(_), _) => false,
            (_, Self::Str(_)) => true,
            _ => matches!(
                (self.number(), other.number()),
                (Ok(a), Ok(b)) if a < b
            ),
        }
    }

    /// `x > y`.
    pub(crate) fn gt(&self, other: &Self) -> bool {
        other.lt(self)
    }

    /// `x <= y`.
    pub(crate) fn le(&self, other: &Self) -> bool {
        self.lt(other) || self.eq(other)
    }

    /// `x >= y`.
    pub(crate) fn ge(&self, other: &Self) -> bool {
        self.gt(other) || self.eq(other)
    }
}

/// `min(values)`: the first of the smallest, by `<`; an empty sequence raises.
pub(crate) fn py_min<'a>(values: impl IntoIterator<Item = &'a Value>) -> PyResult<Value> {
    let mut best: Option<&Value> = None;
    for value in values {
        if best.is_none_or(|b| value.lt(b)) {
            best = Some(value);
        }
    }
    best.cloned()
        .ok_or_else(|| PyError::other("min() arg is an empty sequence"))
}

/// `max(values)`: the first of the largest, by `>`; an empty sequence raises.
pub(crate) fn py_max<'a>(values: impl IntoIterator<Item = &'a Value>) -> PyResult<Value> {
    let mut best: Option<&Value> = None;
    for value in values {
        if best.is_none_or(|b| value.gt(b)) {
            best = Some(value);
        }
    }
    best.cloned()
        .ok_or_else(|| PyError::other("max() arg is an empty sequence"))
}

/// `sum(values)`, from `0`.
pub(crate) fn py_sum<'a>(values: impl IntoIterator<Item = &'a Value>) -> PyResult<Value> {
    let mut total = Value::Int(0);
    for value in values {
        total = total.arith(value, Op::Add)?;
    }
    Ok(total)
}

/// `numpy.mean(values)`: a `float64`, NaN of nothing; a text in the list cannot be reduced.
pub(crate) fn np_mean<'a>(values: impl IntoIterator<Item = &'a Value>) -> PyResult<f64> {
    let mut sum = 0.0;
    let mut count = 0usize;
    for value in values {
        if matches!(value, Value::Str(_)) {
            return Err(PyError::other("cannot perform reduce with flexible type"));
        }
        sum += value.number()?;
        count += 1;
    }
    if count == 0 {
        return Ok(f64::NAN);
    }
    Ok(sum / usize_to_float(count))
}

/// `numpy.std(values)`: the population standard deviation, as a `float64`.
pub(crate) fn np_std<'a>(values: impl IntoIterator<Item = &'a Value> + Clone) -> PyResult<f64> {
    let mean = np_mean(values.clone())?;
    let mut sum = 0.0;
    let mut count = 0usize;
    for value in values {
        let d = value.number()? - mean;
        sum += d * d;
        count += 1;
    }
    if count == 0 {
        return Ok(f64::NAN);
    }
    Ok((sum / usize_to_float(count)).sqrt())
}

/// `a // b` (what `/` is on two `int`s in Python 2): floored, and zero raises.
pub(crate) fn floor_div(a: i64, b: i64) -> PyResult<i64> {
    if b == 0 {
        return Err(PyError::other("integer division or modulo by zero"));
    }
    let q = a.wrapping_div(b);
    if a.wrapping_rem(b) != 0 && ((a < 0) != (b < 0)) {
        Ok(q - 1)
    } else {
        Ok(q)
    }
}

/// `float(text)`: Python 2 strips white space and reads a decimal, `inf`, `infinity` or `nan`
/// with an optional sign.
pub(crate) fn parse_float(text: &str) -> Option<f64> {
    let text = text.trim();
    if text.is_empty() {
        return None;
    }
    text.parse().ok()
}

/// `int(text)`: Python 2 strips white space and reads a signed decimal.
pub(crate) fn parse_int(text: &str) -> Option<i64> {
    text.trim().parse().ok()
}

/// `int(f)`: truncated towards zero; NaN and the infinities raise.
pub(crate) fn float_to_int(f: f64) -> PyResult<i64> {
    if f.is_nan() {
        return Err(PyError::other("cannot convert float NaN to integer"));
    }
    if f.is_infinite() {
        return Err(PyError::other("cannot convert float infinity to integer"));
    }
    // Saturating, where Python would make a long.
    #[allow(clippy::cast_possible_truncation)]
    Ok(f.trunc() as i64)
}

/// `float(i)`.
#[allow(clippy::cast_precision_loss)]
pub(crate) const fn int_to_float(i: i64) -> f64 {
    i as f64
}

/// `float(n)` of a count.
#[allow(clippy::cast_precision_loss)]
pub(crate) const fn usize_to_float(n: usize) -> f64 {
    n as f64
}

/// `round(f)`: Python 2 rounds half away from zero.
pub(crate) fn py_round(f: f64) -> f64 {
    f.round()
}

/// `str(x)`.
pub(crate) fn py_str(value: &Value) -> String {
    match value {
        Value::Int(i) => i.to_string(),
        Value::Float(f) => str_float(*f),
        Value::Str(s) => s.to_string(),
    }
}

/// `repr(x)`.
pub(crate) fn py_repr(value: &Value) -> String {
    match value {
        Value::Int(i) => i.to_string(),
        Value::Float(f) => repr_float(*f),
        Value::Str(s) => repr_str(s),
    }
}

/// `str(list)`: each element's `repr`, between brackets.
pub(crate) fn list_str(values: &[Value]) -> String {
    let inner: Vec<String> = values.iter().map(py_repr).collect();
    format!("[{}]", inner.join(", "))
}

/// `"%.<places>f" % x`.
pub(crate) fn fixed(value: &Value, places: usize) -> PyResult<String> {
    match value {
        Value::Str(_) => Err(PyError::other("float argument required, not str")),
        _ => Ok(fixed_f(value.number()?, places)),
    }
}

/// `"%.<places>f" % f`: `nan` and `inf` as Python writes them.
pub(crate) fn fixed_f(f: f64, places: usize) -> String {
    if f.is_nan() {
        return "nan".to_owned();
    }
    if f.is_infinite() {
        return if f < 0.0 { "-inf" } else { "inf" }.to_owned();
    }
    format!("{f:.places$}")
}

/// `"%d" % x`: a float truncated; a text is not a number.
pub(crate) fn decimal(value: &Value) -> PyResult<String> {
    match value {
        Value::Int(i) => Ok(i.to_string()),
        Value::Float(f) => decimal_f(*f),
        Value::Str(_) => Err(PyError::other("%d format: a number is required, not str")),
    }
}

/// `"%d" % f`.
pub(crate) fn decimal_f(f: f64) -> PyResult<String> {
    Ok(float_to_int(f)?.to_string())
}

/// `str(f)`: twelve significant digits, a point always, an exponent below 1e-4 or from 1e12.
pub(crate) fn str_float(f: f64) -> String {
    float_text(f, Some(12), 12)
}

/// `repr(f)`: the shortest digits that read back, a point always, an exponent below 1e-4 or
/// from 1e16.
pub(crate) fn repr_float(f: f64) -> String {
    float_text(f, None, 16)
}

/// The digits of `f` - the shortest that read back, or the first `sig` rounded - without
/// trailing zeros, and the decimal point's position: `|f| = 0.d1d2... × 10^decpt`.
fn digits(f: f64, sig: Option<usize>) -> (String, i32) {
    let text = match sig {
        None => format!("{:e}", f.abs()),
        Some(n) => format!("{:.*e}", n.saturating_sub(1), f.abs()),
    };
    let (mantissa, exponent) = text.split_once('e').unwrap_or((text.as_str(), "0"));
    let exp: i32 = exponent.parse().unwrap_or(0);
    let mut digits: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    while digits.len() > 1 && digits.ends_with('0') {
        digits.pop();
    }
    if digits.is_empty() {
        digits.push('0');
    }
    (digits, exp + 1)
}

/// `PyOS_double_to_string` in the `r` or `g` style with `Py_DTSF_ADD_DOT_0`: the exponent form
/// when the point falls at or before the fourth leading zero or past `exp_above` digits.
fn float_text(f: f64, sig: Option<usize>, exp_above: i32) -> String {
    if f.is_nan() {
        return "nan".to_owned();
    }
    if f.is_infinite() {
        return if f < 0.0 { "-inf" } else { "inf" }.to_owned();
    }
    let (digits, decpt) = digits(f, sig);
    let count = i32::try_from(digits.len()).unwrap_or(i32::MAX);
    let mut out = String::new();
    if f.is_sign_negative() {
        out.push('-');
    }
    if decpt <= -4 || decpt > exp_above {
        let (head, tail) = digits.split_at(1);
        out.push_str(head);
        if !tail.is_empty() {
            out.push('.');
            out.push_str(tail);
        }
        let e = decpt - 1;
        out.push('e');
        out.push(if e < 0 { '-' } else { '+' });
        out.push_str(&format!("{:02}", e.abs()));
    } else if decpt <= 0 {
        out.push_str("0.");
        for _ in 0..(-decpt) {
            out.push('0');
        }
        out.push_str(&digits);
    } else if decpt >= count {
        out.push_str(&digits);
        for _ in 0..(decpt - count) {
            out.push('0');
        }
        out.push_str(".0");
    } else {
        let (head, tail) = digits.split_at(usize::try_from(decpt).unwrap_or(0));
        out.push_str(head);
        out.push('.');
        out.push_str(tail);
    }
    out
}

/// `repr(s)` of a Python 2 `str`: single quotes unless the text has one and no double quote,
/// the backslash and the quote escaped, tabs and newlines as `\t`, `\n` and `\r`, any other byte
/// outside printable ASCII as `\xhh`.
pub(crate) fn repr_str(s: &str) -> String {
    let quote = if s.contains('\'') && !s.contains('"') {
        b'"'
    } else {
        b'\''
    };
    let mut out = String::new();
    out.push(char::from(quote));
    for byte in s.bytes() {
        match byte {
            b if b == quote => {
                out.push('\\');
                out.push(char::from(b));
            }
            b'\\' => out.push_str("\\\\"),
            b'\t' => out.push_str("\\t"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            0x20..=0x7e => out.push(char::from(byte)),
            _ => out.push_str(&format!("\\x{byte:02x}")),
        }
    }
    out.push(char::from(quote));
    out
}

/// `str(datetime.timedelta(seconds=n))` for a whole number of seconds: `H:MM:SS`, with
/// `N day(s), ` before it when there are days.
pub(crate) fn timedelta(seconds: i64) -> String {
    let days = seconds.div_euclid(86_400);
    let rest = seconds.rem_euclid(86_400);
    let (h, m, s) = (rest / 3600, (rest % 3600) / 60, rest % 60);
    let mut out = String::new();
    if days != 0 {
        out.push_str(&format!(
            "{days} day{}, ",
            if days.abs() == 1 { "" } else { "s" }
        ));
    }
    out.push_str(&format!("{h}:{m:02}:{s:02}"));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trycast_follows_the_format_characters() {
        assert_eq!(Value::trycast("4.68", 'f'), Value::Float(4.68));
        assert_eq!(Value::trycast(" 12 ", 'B'), Value::Int(12));
        assert_eq!(Value::trycast("Loiter", 'M'), Value::text("Loiter"));
        assert_eq!(Value::trycast("1.0", 'h'), Value::text("1.0"));
        assert_eq!(Value::trycast("12", 'N'), Value::text("12"));
        assert_eq!(Value::trycast("7", 'a'), Value::text("7"));
        assert!(Value::trycast("nan", 'f').is_nan());
        assert_eq!(Value::trycast("", 'f'), Value::text(""));
    }

    #[test]
    fn str_and_repr_of_floats_are_pythons() {
        assert_eq!(repr_float(302.754_882_812_5), "302.7548828125");
        assert_eq!(repr_float(1.0), "1.0");
        assert_eq!(repr_float(0.0), "0.0");
        assert_eq!(repr_float(-0.0), "-0.0");
        assert_eq!(repr_float(1e16), "1e+16");
        assert_eq!(repr_float(1e15), "1000000000000000.0");
        assert_eq!(repr_float(0.0001), "0.0001");
        assert_eq!(repr_float(0.00001), "1e-05");
        assert_eq!(repr_float(1.5e-7), "1.5e-07");
        assert_eq!(repr_float(-20.490_345), "-20.490345");
        assert_eq!(repr_float(0.1 + 0.2), "0.30000000000000004");
        assert_eq!(repr_float(f64::NAN), "nan");
        assert_eq!(repr_float(f64::NEG_INFINITY), "-inf");
        assert_eq!(str_float(4.68), "4.68");
        assert_eq!(str_float(0.15), "0.15");
        assert_eq!(str_float(100.0), "100.0");
        assert_eq!(str_float(0.1 + 0.2), "0.3");
        assert_eq!(str_float(123_456_789_012.0), "123456789012.0");
        assert_eq!(str_float(1_234_567_890_123.0), "1.23456789012e+12");
        assert_eq!(str_float(1e16), "1e+16");
        assert_eq!(str_float(0.000_012_345_678_901_234), "1.23456789012e-05");
    }

    #[test]
    fn percent_formats_truncate_and_name_their_errors() {
        assert_eq!(fixed(&Value::Int(3), 2).unwrap(), "3.00");
        assert_eq!(fixed(&Value::Float(0.125), 2).unwrap(), "0.12");
        assert_eq!(fixed_f(f64::NAN, 2), "nan");
        assert_eq!(decimal(&Value::Float(3.7)).unwrap(), "3");
        assert_eq!(decimal(&Value::Float(-3.7)).unwrap(), "-3");
        assert_eq!(decimal(&Value::Float(-0.2)).unwrap(), "0");
        assert_eq!(
            decimal(&Value::text("x")).unwrap_err().text,
            "%d format: a number is required, not str"
        );
        assert_eq!(
            fixed(&Value::text("x"), 2).unwrap_err().text,
            "float argument required, not str"
        );
        assert_eq!(
            decimal_f(f64::NAN).unwrap_err().text,
            "cannot convert float NaN to integer"
        );
    }

    #[test]
    fn repr_of_text_quotes_as_python_does() {
        assert_eq!(repr_str("COMPASS_OFS_X"), "'COMPASS_OFS_X'");
        assert_eq!(repr_str("it's"), "\"it's\"");
        assert_eq!(repr_str("a'b\"c"), "'a\\'b\"c'");
        assert_eq!(repr_str("tab\there\n\\"), "'tab\\there\\n\\\\'");
        assert_eq!(repr_str("\u{1}é"), "'\\x01\\xc3\\xa9'");
        assert_eq!(PyError::key("GPS").text, "'GPS'");
        assert_eq!(
            list_str(&[Value::Int(1500), Value::Float(2.5)]),
            "[1500, 2.5]"
        );
    }

    #[test]
    fn arithmetic_is_python_twos() {
        let (i, f, s) = (Value::Int(7), Value::Float(2.0), Value::text("x"));
        assert_eq!(i.arith(&Value::Int(2), Op::Div).unwrap(), Value::Int(3));
        assert_eq!(
            Value::Int(-7).arith(&Value::Int(2), Op::Div).unwrap(),
            Value::Int(-4)
        );
        assert_eq!(i.arith(&f, Op::Div).unwrap(), Value::Float(3.5));
        assert_eq!(
            i.arith(&Value::Int(0), Op::Div).unwrap_err().text,
            "integer division or modulo by zero"
        );
        assert_eq!(
            f.arith(&Value::Float(0.0), Op::Div).unwrap_err().text,
            "float division by zero"
        );
        assert_eq!(
            s.arith(&f, Op::Sub).unwrap_err().text,
            "unsupported operand type(s) for -: 'str' and 'float'"
        );
        assert_eq!(
            s.squared().unwrap_err().text,
            "unsupported operand type(s) for ** or pow(): 'str' and 'int'"
        );
        assert!(i.gt(&f) && f.lt(&i) && i.lt(&s) && !s.lt(&i));
        assert!(Value::Int(6).eq(&Value::Float(6.0)));
        assert!(!Value::Float(f64::NAN).lt(&Value::Float(1.0)));
        assert!(!Value::Float(f64::NAN).eq(&Value::Float(f64::NAN)));
        assert_eq!(
            py_min([&Value::Float(6.0), &Value::Int(6), &Value::Int(8)]).unwrap(),
            Value::Float(6.0)
        );
        assert_eq!(py_max([&i, &f]).unwrap(), i);
        assert_eq!(
            py_min(std::iter::empty()).unwrap_err().text,
            "min() arg is an empty sequence"
        );
        assert_eq!(py_sum([&i, &Value::Int(3)]).unwrap(), Value::Int(10));
        assert_eq!(np_mean([&i, &Value::Int(3)]).unwrap(), 5.0);
        assert!(np_mean(std::iter::empty()).unwrap().is_nan());
        assert_eq!(
            np_mean([&s]).unwrap_err().text,
            "cannot perform reduce with flexible type"
        );
        let two = [Value::Int(2), Value::Int(4)];
        assert_eq!(np_std(two.iter()).unwrap(), 1.0);
        assert_eq!(
            Value::text("4.5").to_int().unwrap_err().text,
            "invalid literal for int() with base 10: '4.5'"
        );
        assert_eq!(Value::text(" 45 ").to_int().unwrap(), 45);
        assert_eq!(Value::Float(-4.9).to_int().unwrap(), -4);
        assert_eq!(
            Value::text("abc").to_float().unwrap_err().text,
            "could not convert string to float: abc"
        );
    }

    #[test]
    fn timedelta_text_is_pythons() {
        assert_eq!(timedelta(155), "0:02:35");
        assert_eq!(timedelta(0), "0:00:00");
        assert_eq!(timedelta(86_400 + 3661), "1 day, 1:01:01");
        assert_eq!(timedelta(2 * 86_400), "2 days, 0:00:00");
        assert_eq!(timedelta(-5), "-1 day, 23:59:55");
    }
}
