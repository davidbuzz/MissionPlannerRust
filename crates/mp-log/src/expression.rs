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

//! The expressions a preselected graph plots.
//!
//! `CMB_preselect_SelectedIndexChanged` graphs every piece of a preselected graph through
//! `GraphItem`'s expression path, which runs the piece as Python (`TestPython`: IronPython, with
//! `math` and MAVProxy's `mavextra` imported) over the records of the message types it names, in
//! log order, and plots a point for each record once every name in it has a value. This is that
//! evaluation for the part of Python the shipped graphs use - numbers, `TYPE.Field` and
//! `TYPE[n].Field`, `+ - * / % **`, parentheses, and the functions of `math` and `mavextra` that
//! work on numbers - without an interpreter. A piece that needs anything else - a whole message
//! handed to a function, `expected_mag_yaw(GPS,ATT,MAG)` - is refused by name, where the C# would
//! run it.
//!
//! **Numbers** follow Python 2, which IronPython is: a field logged as an integer is an `int`,
//! and `int / int` is floor division.
//!
//! **Three deliberate differences**, each where the C#'s behaviour is an accident of the
//! machinery rather than something a graph file could want:
//!
//! - the script keeps one entry per message type, `{instance: record}` for an instanced one, so
//!   an expression naming two instances of a type (`GPS[0].Spd-GPS[1].Spd`) raises `KeyError`
//!   and plots nothing; here each `TYPE[n]` keeps its own latest record;
//! - the script gives up after 200 records on which a name has no value yet; here the records
//!   before every name has one are skipped however many there are;
//! - `sqrt(-1)` raises `ValueError` and ends the script, losing the whole curve; here that one
//!   point is left out.
//!
//! `// C#: Log/LogBrowse.cs:1270-1298, 1304-1455; mavextra.py:187-320, 732-745, 1363-1368`

use std::collections::BTreeMap;

use crate::dataflash::{LogMessage, Value};

/// A number as Python holds it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Number {
    /// `int` (Python 2's `long` too).
    Int(i64),
    /// `float`.
    Float(f64),
}

impl Number {
    #[allow(clippy::cast_precision_loss)] // Python does the same widening
    const fn float(self) -> f64 {
        match self {
            Self::Int(value) => value as f64,
            Self::Float(value) => value,
        }
    }
}

/// A field of a message, as an expression names it.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Reference {
    /// The message type, `GPS`.
    pub message: String,
    /// The instance, for `GPS[1]`.
    pub instance: Option<i64>,
    /// The field, `Spd`.
    pub field: String,
}

/// What an expression is made of.
#[derive(Debug, Clone, PartialEq)]
enum Node {
    Number(Number),
    Text(String),
    Field(Reference),
    Negate(Box<Node>),
    Binary(Operator, Box<Node>, Box<Node>),
    Call(Function, Vec<Node>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Operator {
    Add,
    Subtract,
    Multiply,
    Divide,
    Remainder,
    Power,
    BitAnd,
    BitOr,
    BitXor,
    ShiftLeft,
    ShiftRight,
}

/// The functions an expression can call: `math`'s, Python's `abs`, `min` and `max`, and the
/// numeric ones of `mavextra`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Function {
    Sqrt,
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Atan2,
    Degrees,
    Radians,
    Fabs,
    Abs,
    Pow,
    Exp,
    Log,
    Log10,
    Floor,
    Ceil,
    Hypot,
    Min,
    Max,
    Wrap180,
    Wrap360,
    AngleDiff,
    Constrain,
    Kmh,
    Lowpass,
    Diff,
    Delta,
    DeltaAngle,
    Average,
}

impl Function {
    fn named(name: &str) -> Option<Self> {
        Some(match name {
            "sqrt" => Self::Sqrt,
            "sin" => Self::Sin,
            "cos" => Self::Cos,
            "tan" => Self::Tan,
            "asin" => Self::Asin,
            "acos" => Self::Acos,
            "atan" => Self::Atan,
            "atan2" => Self::Atan2,
            "degrees" => Self::Degrees,
            "radians" => Self::Radians,
            "fabs" => Self::Fabs,
            "abs" => Self::Abs,
            "pow" => Self::Pow,
            "exp" => Self::Exp,
            "log" => Self::Log,
            "log10" => Self::Log10,
            "floor" => Self::Floor,
            "ceil" => Self::Ceil,
            "hypot" => Self::Hypot,
            "min" => Self::Min,
            "max" => Self::Max,
            "wrap_180" => Self::Wrap180,
            "wrap_360" => Self::Wrap360,
            "angle_diff" => Self::AngleDiff,
            "constrain" => Self::Constrain,
            "kmh" => Self::Kmh,
            "lowpass" => Self::Lowpass,
            "diff" => Self::Diff,
            "delta" => Self::Delta,
            "delta_angle" => Self::DeltaAngle,
            "average" => Self::Average,
            _ => return None,
        })
    }
}

/// Why an expression cannot be plotted here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExpressionError {
    /// It calls a function this evaluator does not have: one that takes whole messages, or one
    /// of `mavextra`'s that reads the log's clock through `mavutil`.
    Unsupported(String),
    /// It is not an expression this evaluator can read.
    Syntax(String),
}

impl std::fmt::Display for ExpressionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unsupported(name) => write!(f, "{name} is not available without Python"),
            Self::Syntax(why) => write!(f, "cannot read the expression: {why}"),
        }
    }
}

/// A parsed expression.
#[derive(Debug, Clone, PartialEq)]
pub struct Expression {
    root: Node,
}

/// One point of an evaluated expression.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// The line of the record that produced it.
    pub line: usize,
    /// That record's `TimeUS`, if it has one.
    pub time_us: Option<f64>,
    /// The expression's value.
    pub value: f64,
}

impl Expression {
    /// Reads an expression; a trailing `:2`, which only says which axis, is dropped as
    /// `TestPython` drops it.
    ///
    /// # Errors
    /// [`ExpressionError::Syntax`] for what cannot be read, and
    /// [`ExpressionError::Unsupported`] for a call to a function this evaluator lacks.
    pub fn parse(text: &str) -> Result<Self, ExpressionError> {
        let text = text.replace(":2", "");
        // MAVProxy's `EXPR{CONDITION}`, which Python cannot read either: the C# plots nothing.
        if text.contains('{') {
            return Err(ExpressionError::Unsupported("{condition}".to_owned()));
        }
        let tokens = tokenize(&text)?;
        let mut parser = Parser { tokens, at: 0 };
        let root = parser.expression()?;
        if parser.at != parser.tokens.len() {
            return Err(ExpressionError::Syntax(format!(
                "unexpected {:?}",
                parser.tokens.get(parser.at)
            )));
        }
        Ok(Self { root })
    }

    /// Every field the expression names, once each.
    #[must_use]
    pub fn references(&self) -> Vec<Reference> {
        let mut found = Vec::new();
        collect(&self.root, &mut found);
        found.sort();
        found.dedup();
        found
    }

    /// The message types the expression reads records of: `GetEnumeratorType`'s list.
    #[must_use]
    pub fn messages(&self) -> Vec<String> {
        let mut names: Vec<String> = self
            .references()
            .into_iter()
            .map(|reference| reference.message)
            .collect();
        names.dedup();
        names
    }

    /// Evaluates the expression over records in log order.
    ///
    /// `records` gives each record's line, its decoded message and its instance (from the
    /// message type's `FMTU`); only records of the types named matter, and others are passed
    /// over. After each record of a named type the expression is evaluated with the latest record
    /// for each reference, and a point is made once every reference has one. A division by zero,
    /// or a value that is not a number, makes no point, as `evaluate_expression` returns `None`
    /// for `ZeroDivisionError`.
    /// `// C#: Log/LogBrowse.cs:1388-1427`
    #[must_use]
    pub fn evaluate<'a>(
        &self,
        records: impl IntoIterator<Item = (usize, &'a LogMessage, Option<i64>)>,
    ) -> Vec<Sample> {
        let references = self.references();
        let mut latest: BTreeMap<Reference, Number> = BTreeMap::new();
        let mut state = State::default();
        let mut samples = Vec::new();
        for (line, message, instance) in records {
            let mut touched = false;
            for reference in &references {
                if reference.message != message.name
                    || (reference.instance.is_some() && reference.instance != instance)
                {
                    continue;
                }
                touched = true;
                match message.field(&reference.field).and_then(number_of) {
                    Some(value) => {
                        latest.insert(reference.clone(), value);
                    }
                    None => {
                        latest.remove(reference);
                    }
                }
            }
            if !touched || latest.len() < references.len() {
                continue;
            }
            if let Some(value) = eval(&self.root, &latest, &mut state)
                .map(Number::float)
                .filter(|value| value.is_finite())
            {
                samples.push(Sample {
                    line,
                    time_us: message.field("TimeUS").and_then(Value::as_f64),
                    value,
                });
            }
        }
        samples
    }
}

/// [`records_of`] through a log's index: only the named types' records are read.
#[must_use]
pub fn records_in(
    log: &crate::logfile::LogFile,
    names: &[String],
) -> Vec<(usize, LogMessage, Option<i64>)> {
    let instance_fields = log.instance_fields();
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    log.messages(&names)
        .filter(|(_, message)| message.name != "FMT")
        .map(|(line, message)| {
            #[allow(clippy::cast_possible_truncation)] // an instance number is a small integer
            let instance = instance_fields
                .get(&message.name)
                .and_then(|label| message.field(label))
                .and_then(Value::as_f64)
                .map(|value| value as i64);
            (line, message, instance)
        })
        .collect()
}

/// The records of some message types, in log order, each with its line and its instance:
/// `GetEnumeratorType(types)` for [`Expression::evaluate`].
///
/// The instance is the value of the field the type's `FMTU` marks with `#`, as `DFItem.instance`
/// reads it; a type without one has none.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:701-773`
#[must_use]
pub fn records_of(data: &[u8], names: &[String]) -> Vec<(usize, LogMessage, Option<i64>)> {
    let instance_fields = crate::plot::instance_fields(data);
    let mut reader = crate::dataflash::DataflashReader::new(data);
    let mut records = Vec::new();
    let mut line = 0usize;
    while let Some(record) = reader.next_record() {
        let this_line = line;
        line += 1;
        let wanted = reader
            .formats()
            .get(&record.msg_type)
            .is_some_and(|format| names.contains(&format.name));
        if !wanted || record.msg_type == crate::dataflash::FMT_TYPE {
            continue;
        }
        let Some(message) = data
            .get(record.offset..)
            .and_then(|bytes| crate::dataflash::decode_record(reader.formats(), bytes))
        else {
            continue;
        };
        #[allow(clippy::cast_possible_truncation)] // an instance number is a small integer
        let instance = instance_fields
            .get(&message.name)
            .and_then(|label| message.field(label))
            .and_then(Value::as_f64)
            .map(|value| value as i64);
        records.push((this_line, message, instance));
    }
    records
}

/// A field's value as the script sees it: `ToDictionary`'s raw value.
fn number_of(value: &Value) -> Option<Number> {
    match value {
        Value::Int(value) => Some(Number::Int(*value)),
        Value::Uint(value) => i64::try_from(*value).ok().map(Number::Int),
        Value::Float(value) => Some(Number::Float(*value)),
        _ => None,
    }
}

fn collect(node: &Node, found: &mut Vec<Reference>) {
    match node {
        Node::Field(reference) => found.push(reference.clone()),
        Node::Negate(inner) => collect(inner, found),
        Node::Binary(_, left, right) => {
            collect(left, found);
            collect(right, found);
        }
        Node::Call(_, arguments) => {
            for argument in arguments {
                collect(argument, found);
            }
        }
        Node::Number(_) | Node::Text(_) => {}
    }
}

/// `mavextra`'s module-level dictionaries, which live as long as one script run.
#[derive(Debug, Default)]
struct State {
    lowpass: BTreeMap<String, f64>,
    diff: BTreeMap<String, f64>,
    delta: BTreeMap<String, (f64, f64, f64)>,
    average: BTreeMap<String, Vec<f64>>,
}

/// A stateful function's key, whatever was passed: `'x'`, `"Lat"`, `0`.
fn key_of(node: &Node, latest: &BTreeMap<Reference, Number>, state: &mut State) -> Option<String> {
    match node {
        Node::Text(text) => Some(text.clone()),
        other => eval(other, latest, state).map(|number| match number {
            Number::Int(value) => value.to_string(),
            Number::Float(value) => value.to_string(),
        }),
    }
}

#[allow(clippy::too_many_lines)] // one arm per function, each short
fn eval(node: &Node, latest: &BTreeMap<Reference, Number>, state: &mut State) -> Option<Number> {
    use Number::{Float, Int};
    match node {
        Node::Number(number) => Some(*number),
        Node::Text(_) => None,
        Node::Field(reference) => latest.get(reference).copied(),
        Node::Negate(inner) => Some(match eval(inner, latest, state)? {
            Int(value) => Int(value.checked_neg()?),
            Float(value) => Float(-value),
        }),
        Node::Binary(operator, left, right) => {
            let (a, b) = (eval(left, latest, state)?, eval(right, latest, state)?);
            binary(*operator, a, b)
        }
        Node::Call(function, arguments) => {
            let mut value =
                |index: usize| -> Option<Number> { eval(arguments.get(index)?, latest, state) };
            let one = |value: Option<Number>| value.map(Number::float);
            Some(match function {
                Function::Sqrt => Float(one(value(0))?.sqrt()),
                Function::Sin => Float(one(value(0))?.sin()),
                Function::Cos => Float(one(value(0))?.cos()),
                Function::Tan => Float(one(value(0))?.tan()),
                Function::Asin => Float(one(value(0))?.asin()),
                Function::Acos => Float(one(value(0))?.acos()),
                Function::Atan => Float(one(value(0))?.atan()),
                Function::Atan2 => {
                    let y = one(value(0))?;
                    Float(y.atan2(one(value(1))?))
                }
                Function::Degrees => Float(one(value(0))?.to_degrees()),
                Function::Radians => Float(one(value(0))?.to_radians()),
                Function::Fabs => Float(one(value(0))?.abs()),
                Function::Abs => match value(0)? {
                    Int(value) => Int(value.checked_abs()?),
                    Float(value) => Float(value.abs()),
                },
                Function::Pow => {
                    let base = one(value(0))?;
                    Float(base.powf(one(value(1))?))
                }
                Function::Exp => Float(one(value(0))?.exp()),
                Function::Log => {
                    let x = one(value(0))?;
                    match arguments.len() {
                        1 => Float(x.ln()),
                        _ => Float(x.ln() / one(value(1))?.ln()),
                    }
                }
                Function::Log10 => Float(one(value(0))?.log10()),
                Function::Floor => Float(one(value(0))?.floor()),
                Function::Ceil => Float(one(value(0))?.ceil()),
                Function::Hypot => {
                    let x = one(value(0))?;
                    Float(x.hypot(one(value(1))?))
                }
                Function::Min | Function::Max => {
                    let mut best: Option<Number> = None;
                    for index in 0..arguments.len() {
                        let candidate = value(index)?;
                        best = Some(match best {
                            None => candidate,
                            Some(current) => {
                                let better = if *function == Function::Min {
                                    candidate.float() < current.float()
                                } else {
                                    candidate.float() > current.float()
                                };
                                if better { candidate } else { current }
                            }
                        });
                    }
                    best?
                }
                Function::Wrap180 => {
                    let mut angle = value(0)?.float();
                    if angle > 180.0 {
                        angle -= 360.0;
                    }
                    if angle < -180.0 {
                        angle += 360.0;
                    }
                    Float(angle)
                }
                Function::Wrap360 => {
                    let mut angle = value(0)?.float();
                    if angle > 360.0 {
                        angle -= 360.0;
                    }
                    if angle < 0.0 {
                        angle += 360.0;
                    }
                    Float(angle)
                }
                Function::AngleDiff => {
                    let first = value(0)?.float();
                    let mut ret = first - value(1)?.float();
                    if ret > 180.0 {
                        ret -= 360.0;
                    }
                    if ret < -180.0 {
                        ret += 360.0;
                    }
                    Float(ret)
                }
                Function::Constrain => {
                    let mut v = value(0)?.float();
                    let (low, high) = (value(1)?.float(), value(2)?.float());
                    if v < low {
                        v = low;
                    }
                    if v > high {
                        v = high;
                    }
                    Float(v)
                }
                Function::Kmh => Float(value(0)?.float() * 3.6),
                Function::Lowpass => {
                    let var = value(0)?.float();
                    let factor = value(2)?.float();
                    let key = key_of(arguments.get(1)?, latest, state)?;
                    let entry = state
                        .lowpass
                        .entry(key)
                        .and_modify(|held| *held = factor * *held + (1.0 - factor) * var)
                        .or_insert(var);
                    Float(*entry)
                }
                Function::Diff => {
                    let var = value(0)?.float();
                    let key = key_of(arguments.get(1)?, latest, state)?;
                    match state.diff.insert(key, var) {
                        None => Int(0),
                        Some(last) => Float(var - last),
                    }
                }
                Function::Delta | Function::DeltaAngle => {
                    let var = value(0)?.float();
                    // Without `tusec` the C# reads `mavutil.mavfile_global.timestamp`, which the
                    // log browser's script never sets up.
                    let tnow = value(2)?.float() * 1.0e-6;
                    let key = key_of(arguments.get(1)?, latest, state)?;
                    let mut ret = 0.0;
                    if let Some(&(last_v, last_t, last_ret)) = state.delta.get(&key) {
                        if last_t == tnow {
                            return Some(Float(last_ret));
                        }
                        let mut dv = var - last_v;
                        if *function == Function::DeltaAngle {
                            if dv > 180.0 {
                                dv -= 360.0;
                            }
                            if dv < -180.0 {
                                dv += 360.0;
                            }
                        }
                        ret = dv / (tnow - last_t);
                    }
                    state.delta.insert(key, (var, tnow, ret));
                    Float(ret)
                }
                Function::Average => {
                    let var = value(0)?.float();
                    let count = value(2)?.float();
                    let key = key_of(arguments.get(1)?, latest, state)?;
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let n = count.max(0.0) as usize;
                    match state.average.get_mut(&key) {
                        None => {
                            state.average.insert(key, vec![var; n]);
                            Float(var)
                        }
                        Some(window) => {
                            if !window.is_empty() {
                                window.remove(0);
                            }
                            window.push(var);
                            Float(window.iter().sum::<f64>() / count)
                        }
                    }
                }
            })
        }
    }
}

/// Python 2's arithmetic: `int` with `int` stays `int`, `/` and `%` floor, anything with a
/// `float` is a `float`; dividing by zero is `ZeroDivisionError`, a point with no value.
fn binary(operator: Operator, a: Number, b: Number) -> Option<Number> {
    use Number::{Float, Int};
    if let (Int(x), Int(y)) = (a, b) {
        return match operator {
            Operator::Add => x.checked_add(y).map(Int),
            Operator::Subtract => x.checked_sub(y).map(Int),
            Operator::Multiply => x.checked_mul(y).map(Int),
            Operator::Divide => {
                let quotient = x.checked_div(y)?;
                let floored = x % y != 0 && (x < 0) != (y < 0);
                Some(Int(quotient - i64::from(floored)))
            }
            Operator::Remainder => {
                let r = x.checked_rem(y)?;
                Some(Int(if r != 0 && (r < 0) != (y < 0) {
                    r + y
                } else {
                    r
                }))
            }
            Operator::Power => match u32::try_from(y) {
                Ok(exponent) => x.checked_pow(exponent).map(Int),
                #[allow(clippy::cast_precision_loss)]
                Err(_) => Some(Float((x as f64).powf(y as f64))),
            },
            Operator::BitAnd => Some(Int(x & y)),
            Operator::BitOr => Some(Int(x | y)),
            Operator::BitXor => Some(Int(x ^ y)),
            Operator::ShiftLeft => u32::try_from(y)
                .ok()
                .and_then(|y| x.checked_shl(y))
                .map(Int),
            Operator::ShiftRight => u32::try_from(y)
                .ok()
                .map(|y| Int(x.checked_shr(y).unwrap_or(if x < 0 { -1 } else { 0 }))),
        };
    }
    let (x, y) = (a.float(), b.float());
    Some(Float(match operator {
        // `TypeError` on a float: no value.
        Operator::BitAnd
        | Operator::BitOr
        | Operator::BitXor
        | Operator::ShiftLeft
        | Operator::ShiftRight => return None,
        Operator::Add => x + y,
        Operator::Subtract => x - y,
        Operator::Multiply => x * y,
        Operator::Divide => {
            if y == 0.0 {
                return None;
            }
            x / y
        }
        Operator::Remainder => {
            if y == 0.0 {
                return None;
            }
            let r = x % y;
            if r != 0.0 && (r < 0.0) != (y < 0.0) {
                r + y
            } else {
                r
            }
        }
        Operator::Power => x.powf(y),
    }))
}

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Number(Number),
    Name(String),
    Text(String),
    Symbol(&'static str),
}

fn tokenize(text: &str) -> Result<Vec<Token>, ExpressionError> {
    let chars: Vec<char> = text.chars().collect();
    let mut tokens = Vec::new();
    let mut at = 0;
    while let Some(&c) = chars.get(at) {
        if c.is_whitespace() {
            at += 1;
        } else if c.is_ascii_digit()
            || (c == '.' && chars.get(at + 1).is_some_and(char::is_ascii_digit))
        {
            let start = at;
            let mut float = false;
            while let Some(&d) = chars.get(at) {
                if d.is_ascii_digit() {
                    at += 1;
                } else if d == '.' && !float {
                    float = true;
                    at += 1;
                } else if matches!(d, 'e' | 'E')
                    && chars
                        .get(at + 1)
                        .is_some_and(|n| n.is_ascii_digit() || matches!(n, '+' | '-'))
                {
                    float = true;
                    at += 2;
                } else {
                    break;
                }
            }
            let literal: String = chars.get(start..at).unwrap_or_default().iter().collect();
            let number = if float {
                literal.parse().ok().map(Number::Float)
            } else {
                literal.parse().ok().map(Number::Int)
            };
            tokens.push(Token::Number(number.ok_or_else(|| {
                ExpressionError::Syntax(format!("bad number {literal}"))
            })?));
        } else if c.is_ascii_alphabetic() || c == '_' {
            let start = at;
            while chars
                .get(at)
                .is_some_and(|d| d.is_ascii_alphanumeric() || *d == '_')
            {
                at += 1;
            }
            tokens.push(Token::Name(
                chars.get(start..at).unwrap_or_default().iter().collect(),
            ));
        } else if c == '\'' || c == '"' {
            let start = at + 1;
            let end = chars
                .iter()
                .skip(start)
                .position(|d| *d == c)
                .map(|offset| start + offset)
                .ok_or_else(|| ExpressionError::Syntax("unterminated string".to_owned()))?;
            tokens.push(Token::Text(
                chars.get(start..end).unwrap_or_default().iter().collect(),
            ));
            at = end + 1;
        } else {
            let two: String = chars.get(at..at + 2).unwrap_or_default().iter().collect();
            let symbol = match two.as_str() {
                "**" => "**",
                "<<" => "<<",
                ">>" => ">>",
                _ => match c {
                    '&' => "&",
                    '|' => "|",
                    '^' => "^",
                    '~' => "~",
                    '+' => "+",
                    '-' => "-",
                    '*' => "*",
                    '/' => "/",
                    '%' => "%",
                    '(' => "(",
                    ')' => ")",
                    '[' => "[",
                    ']' => "]",
                    '.' => ".",
                    ',' => ",",
                    other => {
                        return Err(ExpressionError::Syntax(format!("unexpected {other}")));
                    }
                },
            };
            at += symbol.len();
            tokens.push(Token::Symbol(symbol));
        }
    }
    Ok(tokens)
}

struct Parser {
    tokens: Vec<Token>,
    at: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.at)
    }

    fn eat(&mut self, symbol: &str) -> bool {
        if matches!(self.peek(), Some(Token::Symbol(s)) if *s == symbol) {
            self.at += 1;
            true
        } else {
            false
        }
    }

    fn expect(&mut self, symbol: &str) -> Result<(), ExpressionError> {
        if self.eat(symbol) {
            Ok(())
        } else {
            Err(ExpressionError::Syntax(format!(
                "expected {symbol}, found {:?}",
                self.peek()
            )))
        }
    }

    /// Python's precedence, loosest first: `|`, `^`, `&`, shifts, then `+ -`.
    fn expression(&mut self) -> Result<Node, ExpressionError> {
        self.binary_level(0)
    }

    fn binary_level(&mut self, level: usize) -> Result<Node, ExpressionError> {
        const LEVELS: [&[(&str, Operator)]; 4] = [
            &[("|", Operator::BitOr)],
            &[("^", Operator::BitXor)],
            &[("&", Operator::BitAnd)],
            &[("<<", Operator::ShiftLeft), (">>", Operator::ShiftRight)],
        ];
        let Some(operators) = LEVELS.get(level) else {
            return self.arith();
        };
        let mut left = self.binary_level(level + 1)?;
        'outer: loop {
            for (symbol, operator) in *operators {
                if self.eat(symbol) {
                    let right = self.binary_level(level + 1)?;
                    left = Node::Binary(*operator, Box::new(left), Box::new(right));
                    continue 'outer;
                }
            }
            return Ok(left);
        }
    }

    fn arith(&mut self) -> Result<Node, ExpressionError> {
        let mut left = self.term()?;
        loop {
            let operator = if self.eat("+") {
                Operator::Add
            } else if self.eat("-") {
                Operator::Subtract
            } else {
                return Ok(left);
            };
            let right = self.term()?;
            left = Node::Binary(operator, Box::new(left), Box::new(right));
        }
    }

    fn term(&mut self) -> Result<Node, ExpressionError> {
        let mut left = self.factor()?;
        loop {
            let operator = if self.eat("*") {
                Operator::Multiply
            } else if self.eat("/") {
                Operator::Divide
            } else if self.eat("%") {
                Operator::Remainder
            } else {
                return Ok(left);
            };
            let right = self.factor()?;
            left = Node::Binary(operator, Box::new(left), Box::new(right));
        }
    }

    /// Unary signs bind looser than `**`: `-x**2` is `-(x**2)`.
    fn factor(&mut self) -> Result<Node, ExpressionError> {
        if self.eat("-") {
            return Ok(Node::Negate(Box::new(self.factor()?)));
        }
        if self.eat("~") {
            // `~x` is `-x - 1` for an int.
            let inner = self.factor()?;
            return Ok(Node::Binary(
                Operator::Subtract,
                Box::new(Node::Negate(Box::new(inner))),
                Box::new(Node::Number(Number::Int(1))),
            ));
        }
        if self.eat("+") {
            return self.factor();
        }
        let base = self.atom()?;
        if self.eat("**") {
            let exponent = self.factor()?;
            return Ok(Node::Binary(
                Operator::Power,
                Box::new(base),
                Box::new(exponent),
            ));
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<Node, ExpressionError> {
        let token = self
            .peek()
            .cloned()
            .ok_or_else(|| ExpressionError::Syntax("unexpected end".to_owned()))?;
        self.at += 1;
        match token {
            Token::Number(number) => Ok(Node::Number(number)),
            Token::Text(text) => Ok(Node::Text(text)),
            Token::Symbol("(") => {
                let inner = self.expression()?;
                self.expect(")")?;
                Ok(inner)
            }
            Token::Name(name) => self.named(name),
            Token::Symbol(other) => Err(ExpressionError::Syntax(format!("unexpected {other}"))),
        }
    }

    /// A call, or a field: `NAME(...)`, `NAME.Field`, `NAME[n].Field`.
    fn named(&mut self, name: String) -> Result<Node, ExpressionError> {
        if self.eat("(") {
            let function =
                Function::named(&name).ok_or_else(|| ExpressionError::Unsupported(name.clone()))?;
            let mut arguments = Vec::new();
            if !self.eat(")") {
                loop {
                    arguments.push(self.expression()?);
                    if self.eat(")") {
                        break;
                    }
                    self.expect(",")?;
                }
            }
            return Ok(Node::Call(function, arguments));
        }
        let mut instance = None;
        if self.eat("[") {
            match self.peek().cloned() {
                Some(Token::Number(Number::Int(value))) => {
                    self.at += 1;
                    instance = Some(value);
                }
                other => {
                    return Err(ExpressionError::Syntax(format!(
                        "expected an instance, found {other:?}"
                    )));
                }
            }
            self.expect("]")?;
        }
        if self.eat(".")
            && let Some(Token::Name(field)) = self.peek().cloned()
        {
            self.at += 1;
            // `RFND.Dist[0]`: a number indexed, a `TypeError` in Python too.
            if matches!(self.peek(), Some(Token::Symbol("["))) {
                return Err(ExpressionError::Unsupported(format!("{name}.{field}[]")));
            }
            return Ok(Node::Field(Reference {
                message: name,
                instance,
                field,
            }));
        }
        // A name on its own is a whole message, which only a function that takes messages uses.
        Err(ExpressionError::Unsupported(name))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message(name: &str, fields: &[(&str, Value)]) -> LogMessage {
        LogMessage {
            name: name.to_owned(),
            fields: fields
                .iter()
                .map(|(label, value)| ((*label).to_owned(), value.clone()))
                .collect(),
        }
    }

    fn values(samples: &[Sample]) -> Vec<f64> {
        samples.iter().map(|sample| sample.value).collect()
    }

    /// A field is the field; arithmetic is Python's.
    #[test]
    fn arithmetic_over_one_message() {
        let records = [
            message(
                "ATT",
                &[
                    ("TimeUS", Value::Uint(10)),
                    ("Roll", Value::Float(3.0)),
                    ("Pitch", Value::Float(4.0)),
                ],
            ),
            message(
                "ATT",
                &[
                    ("TimeUS", Value::Uint(20)),
                    ("Roll", Value::Float(-6.0)),
                    ("Pitch", Value::Float(8.0)),
                ],
            ),
        ];
        let over = |text: &str| {
            let expression = Expression::parse(text).expect("parses");
            values(
                &expression.evaluate(
                    records
                        .iter()
                        .enumerate()
                        .map(|(line, message)| (line, message, None)),
                ),
            )
        };
        assert_eq!(over("ATT.Roll"), vec![3.0, -6.0]);
        assert_eq!(over("sqrt(ATT.Roll**2+ATT.Pitch**2)"), vec![5.0, 10.0]);
        assert_eq!(over("-ATT.Roll**2"), vec![-9.0, -36.0]);
        assert_eq!(over("ATT.Roll*2+1:2"), vec![7.0, -11.0]);
        assert_eq!(over("abs(ATT.Roll)"), vec![3.0, 6.0]);
        assert_eq!(over("max(2,ATT.Roll)"), vec![3.0, 2.0]);
        assert_eq!(over("wrap_360(ATT.Roll)"), vec![3.0, 354.0]);
        assert_eq!(over("lowpass(ATT.Roll,'r',0.5)"), vec![3.0, -1.5]);
        assert_eq!(over("diff(ATT.Roll,\"d\")"), vec![0.0, -9.0]);
        assert_eq!(
            over("delta(ATT.Roll,0,ATT.TimeUS)"),
            vec![0.0, -9.0 / (20.0 * 1.0e-6 - 10.0 * 1.0e-6)]
        );
        let degrees = over("degrees(atan2(ATT.Pitch,ATT.Roll))");
        assert!((degrees[0] - 53.130_102).abs() < 1e-5, "{degrees:?}");
    }

    /// Python 2: integer fields divide by flooring, and dividing by zero makes no point.
    #[test]
    fn integers_are_python_2_integers() {
        let records = [
            message("BAT", &[("Cnt", Value::Int(7)), ("Z", Value::Int(0))]),
            message("BAT", &[("Cnt", Value::Int(-7)), ("Z", Value::Int(0))]),
        ];
        let over = |text: &str| {
            values(
                &Expression::parse(text)
                    .expect("parses")
                    .evaluate(records.iter().map(|message| (0, message, None))),
            )
        };
        assert_eq!(over("BAT.Cnt/2"), vec![3.0, -4.0]);
        assert_eq!(over("BAT.Cnt/2.0"), vec![3.5, -3.5]);
        assert_eq!(over("BAT.Cnt%3"), vec![1.0, 2.0]);
        assert!(over("BAT.Cnt/BAT.Z").is_empty());
        assert_eq!(over("BAT.Cnt&(1<<2)"), vec![4.0, 0.0]);
        assert_eq!(over("BAT.Cnt|8^1"), vec![15.0, -7.0]);
        assert!(over("BAT.Cnt/2.0&1").is_empty(), "a float has no bits");
    }

    /// Two messages: a point per record of either, once both have been seen, with the latest of
    /// each - and an instance keeps its own latest.
    #[test]
    fn several_messages_join_on_their_latest_records() {
        let records = [
            (message("GPS", &[("Spd", Value::Float(1.0))]), Some(0)),
            (message("GPS", &[("Spd", Value::Float(5.0))]), Some(1)),
            (message("CTUN", &[("As", Value::Float(10.0))]), None),
            (message("GPS", &[("Spd", Value::Float(2.0))]), Some(0)),
            (message("GPS", &[("Spd", Value::Float(7.0))]), Some(1)),
        ];
        let over = |text: &str| {
            values(
                &Expression::parse(text).expect("parses").evaluate(
                    records
                        .iter()
                        .enumerate()
                        .map(|(line, (message, instance))| (line, message, *instance)),
                ),
            )
        };
        assert_eq!(over("GPS[0].Spd-CTUN.As"), vec![-9.0, -8.0]);
        assert_eq!(over("GPS[1].Spd-GPS[0].Spd"), vec![4.0, 3.0, 5.0]);
        assert_eq!(over("GPS[0].Spd"), vec![1.0, 2.0]);
        assert_eq!(over("GPS.Spd"), vec![1.0, 5.0, 2.0, 7.0]);
    }

    /// Through the index or by walking the bytes, the same records with the same instances.
    #[test]
    fn records_through_the_index_are_the_walks() {
        for name in ["dataflash.bin", "dataflash_damaged.bin"] {
            let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata")
                .join(name);
            let data = std::fs::read(&path).expect("fixture");
            let names = vec!["ATT".to_owned(), "GPS".to_owned(), "VIBE".to_owned()];
            let walked = records_of(&data, &names);
            let log = crate::logfile::LogFile::from_bytes(data);
            let indexed = records_in(&log, &names);
            assert!(!walked.is_empty());
            assert_eq!(walked, indexed, "{name}");
        }
    }

    /// A function over whole messages is refused by name; so is what cannot be read.
    #[test]
    fn what_needs_python_is_refused() {
        assert_eq!(
            Expression::parse("expected_mag_yaw(GPS,ATT,MAG)"),
            Err(ExpressionError::Unsupported("expected_mag_yaw".to_owned()))
        );
        assert_eq!(
            Expression::parse("sqrt(ATTITUDE)"),
            Err(ExpressionError::Unsupported("ATTITUDE".to_owned()))
        );
        assert!(matches!(
            Expression::parse("ATT.Roll >"),
            Err(ExpressionError::Syntax(_))
        ));
        let parsed = Expression::parse("sqrt(IMU[0].AccX**2+IMU[0].AccY**2)").expect("parses");
        assert_eq!(parsed.messages(), vec!["IMU".to_owned()]);
        assert_eq!(parsed.references().len(), 2);
        assert_eq!(parsed.references()[0].instance, Some(0));
    }

    /// Every piece of the shipped graphs either reads, or is refused for a named function, or is
    /// one Python cannot read either: the files have pieces whose brackets do not balance.
    #[test]
    fn every_shipped_piece_reads_or_names_what_it_lacks() {
        let mut read = 0;
        let mut refused = 0;
        let mut unreadable = Vec::new();
        for graph in crate::mavgraph::graphs() {
            for item in graph.items.iter().flatten() {
                let piece = item.graphed();
                match Expression::parse(&piece) {
                    Ok(_) => read += 1,
                    Err(ExpressionError::Unsupported(_)) => refused += 1,
                    Err(ExpressionError::Syntax(why)) => {
                        let opened = piece.matches('(').count();
                        let closed = piece.matches(')').count();
                        assert_ne!(opened, closed, "{}: {piece} - {why}", graph.name);
                        unreadable.push(piece);
                    }
                }
            }
        }
        eprintln!(
            "shipped pieces: {read} read, {refused} refused, {} unbalanced: {unreadable:?}",
            unreadable.len()
        );
        assert!(read > refused * 3, "{read} read, {refused} refused");
        assert!(unreadable.len() < 10);
    }
}
