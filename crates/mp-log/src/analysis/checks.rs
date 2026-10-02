//! The analyzer's checks: `LogAnalyzer/py2exe/tests/*.py`, one function a file, run in the order
//! the Windows runner loads them - its `tests` folder listed by name - and named as each
//! `Test.name` names itself. Each is ported line for line, Python 2's arithmetic and wording
//! included, and returns what its `TestResult` holds or the exception it raised, which
//! `TestSuite.run` turns into an UNKNOWN with the exception's text.
//!
//! `TestDualGyroDrift` sets `enable = False` and is neither run nor written. `verbose` is never
//! set by `runner.exe`, so the verbose-only passages (Autotune's `ATUN` lines, Compass's
//! min/max lines) are not here.
//!
//! Where Python 2 iterates a dict or a set - the channels and fields a NaN is reported in, the
//! parameters, the names of the errors found - its order is its hash table's, which differs
//! between the Windows runner and any other build; this port lists them in first-seen or
//! declared order and says so at each site.

use super::logdata::{
    Channel, DataflashLog, LogIterator, Ordered, VehicleType, find_loiter_chunks, is_log_empty,
};
use super::pyval::{
    Op, PyError, PyResult, Value, decimal, decimal_f, fixed, fixed_f, float_to_int, floor_div,
    list_str, np_std, py_max, py_min, py_repr, py_round, py_str, py_sum, repr_float,
    usize_to_float,
};

/// `TestResult.StatusType`: NA is not applicable to this log (a copter's check on a plane),
/// UNKNOWN is data the check needed and had not got. `// LogAnalyzer.py:37-43`
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Status {
    /// `GOOD`
    Good,
    /// `FAIL`
    Fail,
    /// `WARN`
    Warn,
    /// `UNKNOWN`
    Unknown,
    /// `NA`
    Na,
}

impl Status {
    /// The status as `outputXML` writes it.
    pub(crate) const fn text(self) -> &'static str {
        match self {
            Self::Good => "GOOD",
            Self::Fail => "FAIL",
            Self::Warn => "WARN",
            Self::Unknown => "UNKNOWN",
            Self::Na => "NA",
        }
    }
}

/// `TestResult`: a status and a message, which may run to several lines.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Outcome {
    /// `status`.
    pub(crate) status: Status,
    /// `statusMessage`.
    pub(crate) message: String,
}

impl Outcome {
    fn good() -> Self {
        Self::new(Status::Good, "")
    }

    /// A status with its message.
    pub(crate) fn new(status: Status, message: impl Into<String>) -> Self {
        Self {
            status,
            message: message.into(),
        }
    }

    /// `FAIL()`.
    fn fail(&mut self) {
        self.status = Status::Fail;
    }

    /// `WARN()`: a FAIL is never softened.
    fn warn(&mut self) {
        if self.status != Status::Fail {
            self.status = Status::Warn;
        }
    }
}

/// One check: `Test.run(logdata, verbose=False)`.
pub(crate) type Check = fn(&DataflashLog) -> PyResult<Outcome>;

/// The enabled checks with their names, in the runner's order.
pub(crate) const CHECKS: [(&str, Check); 17] = [
    ("Autotune", autotune),
    ("Brownout", brownout),
    ("Compass", compass),
    ("Dupe Log Data", dupe_log_data),
    ("Empty", empty),
    ("Event/Failsafe", events),
    ("GPS", gps_glitch),
    ("IMU Mismatch", imu_match),
    ("Motor Balance", motor_balance),
    ("NaNs", nan),
    ("OpticalFlow", opt_flow),
    ("Parameters", params),
    ("PM", performance),
    ("Pitch/Roll", pitch_roll_coupling),
    ("Thrust", thrust),
    ("VCC", vcc),
    ("Vibration", vibration),
];

/// `x == n` for a sample and a small whole number.
fn is(value: &Value, n: i64) -> bool {
    value.eq(&Value::Int(n))
}

/// `if x:` of a sample: zero and the empty text are false.
fn truthy(value: &Value) -> bool {
    match value {
        Value::Int(i) => *i != 0,
        Value::Float(f) => *f != 0.0,
        Value::Str(s) => !s.is_empty(),
    }
}

/// `len(list)` as Python counts it.
fn count(n: usize) -> i64 {
    i64::try_from(n).unwrap_or(i64::MAX)
}

/// `TestAutotune`: the autotune sessions a copter's `EV` events record, each GOOD on success,
/// FAIL on failure and UNKNOWN otherwise, the last deciding. The guard for missing data resets
/// its flag on every name, so only a missing `ATUN` returns early; a missing `EV` then raises
/// its `KeyError`. `// tests/TestAutotune.py`
fn autotune(log: &DataflashLog) -> PyResult<Outcome> {
    const INITIALISED: i64 = 30;
    const SUCCESS: i64 = 33;
    const FAILED: i64 = 34;
    const EVENTS: [i64; 8] = [30, 31, 32, 33, 34, 35, 36, 37];
    let mut out = Outcome::good();
    if log.vehicle_type != Some(VehicleType::Copter) {
        return Ok(Outcome::new(Status::Na, ""));
    }
    let mut missing_last = false;
    for group in ["EV", "ATDE", "ATUN"] {
        missing_last = !log.channels.contains_key(group);
        if missing_last {
            out.status = Status::Unknown;
            out.message = format!("No {group} log data");
        }
    }
    if missing_last {
        return Ok(out);
    }
    let events: Vec<&(usize, Value)> = log
        .channel("EV", "Id")?
        .list
        .iter()
        .filter(|(_, id)| EVENTS.iter().any(|e| is(id, *e)))
        .collect();
    let mut attempts: Vec<&[&(usize, Value)]> = Vec::new();
    let mut session_start: Option<usize> = None;
    for (i, (_, id)) in events.iter().enumerate() {
        if is(id, INITIALISED) {
            if let Some(start) = session_start {
                attempts.push(events.get(start..i).unwrap_or(&[]));
            }
            session_start = Some(i);
        }
    }
    if let Some(start) = session_start {
        attempts.push(events.get(start..).unwrap_or(&[]));
    }
    for attempt in attempts {
        let has = |code: i64| attempt.iter().any(|(_, id)| is(id, code));
        // last wins
        let mark = if has(SUCCESS) {
            out.status = Status::Good;
            "[+]"
        } else if has(FAILED) {
            out.status = Status::Fail;
            "[-]"
        } else {
            out.status = Status::Unknown;
            "[?]"
        };
        let start = attempt.first().map_or(0, |(line, _)| *line);
        let stop = attempt.last().map_or(0, |(line, _)| *line);
        out.message
            .push_str(&format!("{mark} Autotune {start}-{stop}\n"));
    }
    Ok(out)
}

/// `TestBrownout`: a log that ends armed (by the last arm/disarm event) above three metres of
/// barometric altitude was cut short. `// tests/TestBrownout.py`
fn brownout(log: &DataflashLog) -> PyResult<Outcome> {
    let mut out = Outcome::good();
    let mut is_armed = false;
    if log.channels.contains_key("EV") {
        for (_, id) in &log.channel("EV", "Id")?.list {
            if is(id, 10) {
                is_armed = true;
            } else if is(id, 11) {
                is_armed = false;
            }
        }
    }
    let Some(ctun) = log.channels.get("CTUN") else {
        return Ok(Outcome::new(Status::Unknown, "No CTUN log data"));
    };
    if let Some(bar_alt) = ctun.get("BarAlt") {
        let (final_alt, _) = bar_alt.nearest(log.line_count, false)?;
        if is_armed && final_alt.gt(&Value::Float(3.0)) {
            out = Outcome::new(
                Status::Fail,
                format!(
                    "Truncated Log? Ends while armed at altitude {}m",
                    fixed(&final_alt, 2)?
                ),
            );
        }
    }
    Ok(out)
}

/// `math.sqrt(x**2 + y**2 + z**2)`.
fn vec_len(x: &Value, y: &Value, z: &Value) -> PyResult<f64> {
    Ok(x.squared()?
        .arith(&y.squared()?, Op::Add)?
        .arith(&z.squared()?, Op::Add)?
        .number()?
        .sqrt())
}

/// `TestCompass`: the compass offset parameters and the largest offsets logged in `MAG`, both
/// warned above 300 and failed above 500; the change in the field's length over the log, warned
/// above 25% and failed above 35%, with notes outside 120..550; all-zero samples. A `KeyError`
/// is a FAIL with the key "not found". `// tests/TestCompass.py`
fn compass(log: &DataflashLog) -> PyResult<Outcome> {
    const WARN_OFFSET: f64 = 300.0;
    const FAIL_OFFSET: f64 = 500.0;
    let mut out = Outcome::good();
    let body = |out: &mut Outcome| -> PyResult<()> {
        let ox = log.parameter("COMPASS_OFS_X")?;
        let oy = log.parameter("COMPASS_OFS_Y")?;
        let oz = log.parameter("COMPASS_OFS_Z")?;
        let length = vec_len(ox, oy, oz)?;
        if length > FAIL_OFFSET {
            out.fail();
            out.message = format!(
                "FAIL: Large compass offset params (X:{}, Y:{}, Z:{})\n",
                fixed(ox, 2)?,
                fixed(oy, 2)?,
                fixed(oz, 2)?
            );
        } else if length > WARN_OFFSET {
            out.warn();
            out.message = format!(
                "WARN: Large compass offset params (X:{}, Y:{}, Z:{})\n",
                fixed(ox, 2)?,
                fixed(oy, 2)?,
                fixed(oz, 2)?
            );
        }
        if log.channels.contains_key("MAG") {
            let xs = &log.channel("MAG", "OfsX")?.list;
            let ys = &log.channel("MAG", "OfsY")?.list;
            let zs = &log.channel("MAG", "OfsZ")?.list;
            // `reduce` over `zip`: as many rows as the shortest list, the later of two equal
            // lengths kept.
            let mut largest: Option<(&Value, &Value, &Value)> = None;
            for ((_, x), ((_, y), (_, z))) in xs.iter().zip(ys.iter().zip(zs.iter())) {
                largest = Some(match largest {
                    None => (x, y, z),
                    Some(best) => {
                        if vec_len(best.0, best.1, best.2)? > vec_len(x, y, z)? {
                            best
                        } else {
                            (x, y, z)
                        }
                    }
                });
            }
            let (x, y, z) = largest.ok_or_else(|| {
                PyError::other("reduce() of empty sequence with no initial value")
            })?;
            let length = vec_len(x, y, z)?;
            if length > FAIL_OFFSET {
                out.fail();
                out.message.push_str(&format!(
                    "FAIL: Large compass offset in MAG data (X:{}, Y:{}, Z:{})\n",
                    fixed(x, 2)?,
                    fixed(y, 2)?,
                    fixed(z, 2)?
                ));
            } else if length > WARN_OFFSET {
                out.warn();
                out.message.push_str(&format!(
                    "WARN: Large compass offset in MAG data (X:{}, Y:{}, Z:{})\n",
                    fixed(x, 2)?,
                    fixed(y, 2)?,
                    fixed(z, 2)?
                ));
            }
        }
        // check for mag field length change, and length outside of recommended range
        if log.channels.contains_key("MAG") {
            const WARN_DIFF: f64 = 0.25;
            const FAIL_DIFF: f64 = 0.35;
            const MIN_FIELD: f64 = 120.0;
            const MAX_FIELD: f64 = 550.0;
            let mag_x = &log.channel("MAG", "MagX")?.list;
            let zero = Value::Int(0);
            let (mut min_field, mut max_field): (Option<f64>, Option<f64>) = (None, None);
            let mut zeros_found = false;
            for index in 0..mag_x.len() {
                let x = &mag_x.get(index).ok_or_else(PyError::index)?.1;
                let y = &log
                    .channel("MAG", "MagY")?
                    .list
                    .get(index)
                    .ok_or_else(PyError::index)?
                    .1;
                let z = &log
                    .channel("MAG", "MagZ")?
                    .list
                    .get(index)
                    .ok_or_else(PyError::index)?
                    .1;
                if x.eq(&zero) && y.eq(&zero) && z.eq(&zero) {
                    // sometimes they're zero, not sure why
                    zeros_found = true;
                } else {
                    let field = x
                        .arith(x, Op::Mul)?
                        .arith(&y.arith(y, Op::Mul)?, Op::Add)?
                        .arith(&z.arith(z, Op::Mul)?, Op::Add)?
                        .number()?
                        .sqrt();
                    // `mf < None` is never true and `mf > None` always is: the minimum is only
                    // ever set by the first sample, so a first sample of zeros leaves it unset.
                    if min_field.is_some_and(|m| field < m) {
                        min_field = Some(field);
                    }
                    if max_field.is_none_or(|m| field > m) {
                        max_field = Some(field);
                    }
                    if index == 0 {
                        min_field = Some(field);
                        max_field = Some(field);
                    }
                }
            }
            match min_field {
                None => {
                    out.fail();
                    out.message.push_str("No valid mag data found\n");
                }
                Some(min) => {
                    let max = max_field.unwrap_or(min);
                    if min == 0.0 {
                        return Err(PyError::other("float division by zero"));
                    }
                    let percent = (max - min) / min;
                    if percent > FAIL_DIFF {
                        out.fail();
                        out.message.push_str(&format!(
                            "Large change in mag_field ({}%)\n",
                            fixed_f(percent * 100.0, 2)
                        ));
                    } else if percent > WARN_DIFF {
                        out.warn();
                        out.message.push_str(&format!(
                            "Moderate change in mag_field ({}%)\n",
                            fixed_f(percent * 100.0, 2)
                        ));
                    } else {
                        out.message.push_str(&format!(
                            "mag_field interference within limits ({}%)\n",
                            fixed_f(percent * 100.0, 2)
                        ));
                    }
                    if min < MIN_FIELD {
                        out.message.push_str(&format!(
                            "Min mag field length ({}) < recommended ({})\n",
                            fixed_f(min, 2),
                            fixed_f(MIN_FIELD, 2)
                        ));
                    }
                    if max > MAX_FIELD {
                        out.message.push_str(&format!(
                            "Max mag field length ({}) > recommended ({})\n",
                            fixed_f(max, 2),
                            fixed_f(MAX_FIELD, 2)
                        ));
                    }
                }
            }
            if zeros_found {
                if out.status == Status::Good {
                    out.warn();
                }
                out.message
                    .push_str("All zeros found in MAG X/Y/Z log data\n");
            }
        } else {
            out.message
                .push_str("No MAG data, unable to test mag_field interference\n");
        }
        Ok(())
    };
    match body(&mut out) {
        Ok(()) => Ok(out),
        Err(e) if e.key_error => {
            out.status = Status::Fail;
            out.message = format!("{} not found", e.text);
            Ok(out)
        }
        Err(e) => Err(e),
    }
}

/// `range(start, stop, step)` with a step that is not zero.
fn py_range(start: i64, stop: i64, step: i64) -> Vec<i64> {
    let mut out = Vec::new();
    let mut x = start;
    while (step > 0 && x < stop) || (step < 0 && x > stop) {
        out.push(x);
        x += step;
    }
    out
}

/// `list[start:stop]` with Python's bounds, negatives counted from the end.
fn py_slice<T>(list: &[T], start: i64, stop: i64) -> &[T] {
    let n = count(list.len());
    let clamp = |x: i64| if x < 0 { (x + n).max(0) } else { x.min(n) };
    let (s, e) = (clamp(start), clamp(stop));
    match (usize::try_from(s), usize::try_from(e)) {
        (Ok(s), Ok(e)) if s < e => list.get(s..e).unwrap_or(&[]),
        _ => &[],
    }
}

/// `TestDupeLogData`: twenty `ATT.Pitch` samples from each of ten points through the log, looked
/// for again later in the log; a match is data logged twice. Fewer than eleven samples make
/// `range` a step of zero. `// tests/TestDupeLogData.py`
fn dupe_log_data(log: &DataflashLog) -> PyResult<Outcome> {
    if !log.channels.contains_key("ATT") {
        return Ok(Outcome::new(Status::Unknown, "No ATT log data"));
    }
    let data = &log.channel("ATT", "Pitch")?.list;
    let len = count(data.len());
    let att_end = len - 1;
    let step = floor_div(att_end, 11)?;
    if step == 0 {
        return Err(PyError::other("range() step argument must not be zero"));
    }
    let indices = py_range(step, att_end - step, step);
    let first = *indices.first().ok_or_else(PyError::index)?;
    let match_sample = |sample: &[(usize, Value)], start: i64| -> PyResult<Option<usize>> {
        // ignore if all data in sample is the same value
        let same = sample
            .iter()
            .filter(|(_, v)| sample.first().is_some_and(|(_, f)| v.eq(f)))
            .count();
        if same == 20 {
            return Ok(None);
        }
        let mut i = start;
        while i < len {
            if i != start {
                let mut j = 0usize;
                while j < 20 {
                    let Some((_, d)) = usize::try_from(i).ok().and_then(|i| data.get(i + j)) else {
                        break;
                    };
                    let (_, s) = sample.get(j).ok_or_else(PyError::index)?;
                    if !d.eq(s) {
                        break;
                    }
                    j += 1;
                }
                if j == 20 {
                    return Ok(usize::try_from(i)
                        .ok()
                        .and_then(|i| data.get(i))
                        .map(|(line, _)| *line));
                }
            }
            i += 1;
        }
        Ok(None)
    };
    let mut sample_index = 0usize;
    let mut i = first;
    while i < len {
        if indices.get(sample_index).copied() == Some(i) {
            let sample = py_slice(data, i, i + 20);
            // `if matchedLine:` - a line of zero would be false, but lines start at one.
            if let Some(matched) = match_sample(sample, i)?.filter(|line| *line != 0) {
                let from = sample.first().map_or(0, |(line, _)| *line);
                return Ok(Outcome::new(
                    Status::Fail,
                    format!("Duplicate data chunks found in log ({from} and {matched})"),
                ));
            }
            sample_index += 1;
            if sample_index >= indices.len() {
                break;
            }
        }
        i += 1;
    }
    Ok(Outcome::good())
}

/// `TestEmpty`: `DataflashLogHelper.isLogEmpty`. `// tests/TestEmpty.py`
fn empty(log: &DataflashLog) -> PyResult<Outcome> {
    Ok(match is_log_empty(log)? {
        Some(why) => Outcome::new(Status::Fail, format!("Empty log? {why}")),
        None => Outcome::good(),
    })
}

/// `ERR.Subsys` and `ERR.ECode` side by side, as the checks read them (both with the Python's
/// `assert` on their lengths).
fn err_codes(log: &DataflashLog) -> PyResult<Vec<(&Value, &Value)>> {
    let subsys = &log.channel("ERR", "Subsys")?.list;
    let ecode = &log.channel("ERR", "ECode")?.list;
    if subsys.len() != ecode.len() {
        // AssertionError, with no text.
        return Err(PyError::other(""));
    }
    Ok(subsys
        .iter()
        .zip(ecode.iter())
        .map(|((_, s), (_, e))| (s, e))
        .collect())
}

/// `TestEvents`: the `ERR` subsystems and codes that are failures or failsafes; a fence breach
/// alone is a WARN, anything else a FAIL naming them. The Python lists them from a set, in its
/// hash order; here they come in the order the code names them. `// tests/TestEvents.py`
fn events(log: &DataflashLog) -> PyResult<Outcome> {
    const NAMES: [&str; 10] = [
        "PPM",
        "COMPASS",
        "FS_THR",
        "FS_BATT",
        "GPS",
        "GCS",
        "FENCE",
        "FLT_MODE",
        "GPS_GLITCH",
        "CRASH",
    ];
    let mut out = Outcome::good();
    let mut found = [false; 10];
    if log.channels.contains_key("ERR") {
        for (subsys, ecode) in err_codes(log)? {
            let which = if is(subsys, 2) && is(ecode, 1) {
                Some(0)
            } else if is(subsys, 3) && (is(ecode, 1) || is(ecode, 2)) {
                Some(1)
            } else if is(subsys, 5) && is(ecode, 1) {
                Some(2)
            } else if is(subsys, 6) && is(ecode, 1) {
                Some(3)
            } else if is(subsys, 7) && is(ecode, 1) {
                Some(4)
            } else if is(subsys, 8) && is(ecode, 1) {
                Some(5)
            } else if is(subsys, 9) && (is(ecode, 1) || is(ecode, 2)) {
                Some(6)
            } else if is(subsys, 10) {
                Some(7)
            } else if is(subsys, 11) && is(ecode, 2) {
                Some(8)
            } else if is(subsys, 12) && is(ecode, 1) {
                Some(9)
            } else {
                None
            };
            if let Some(slot) = which.and_then(|w| found.get_mut(w)) {
                *slot = true;
            }
        }
    }
    let errors: Vec<&str> = NAMES
        .iter()
        .zip(found.iter())
        .filter(|(_, f)| **f)
        .map(|(n, _)| *n)
        .collect();
    if !errors.is_empty() {
        if errors == ["FENCE"] {
            out.status = Status::Warn;
        } else {
            out.status = Status::Fail;
            out.message = if errors.len() == 1 {
                "ERR found: ".to_owned()
            } else {
                "ERRs found: ".to_owned()
            };
        }
        for err in errors {
            out.message.push_str(err);
            out.message.push(' ');
        }
    }
    Ok(out)
}

/// `TestGPSGlitch`: GPS glitch errors, then the satellite count and HDOP against their limits.
/// With glitches the Python joins two messages with `"\n".join(a, b)`, which raises, so that case
/// is always an UNKNOWN with the `TypeError`'s text. `// tests/TestGPSGlitch.py`
fn gps_glitch(log: &DataflashLog) -> PyResult<Outcome> {
    if !log.channels.contains_key("GPS") {
        return Ok(Outcome::new(Status::Unknown, "No GPS log data"));
    }
    let mut glitches = 0i64;
    if log.channels.contains_key("ERR") {
        for (subsys, ecode) in err_codes(log)? {
            if is(subsys, 11) && is(ecode, 2) {
                glitches += 1;
            }
        }
    }
    let mut out = Outcome::good();
    if glitches > 0 {
        out = Outcome::new(
            Status::Fail,
            format!("GPS glitch errors found ({glitches})"),
        );
    }
    let gps = log.group("GPS")?;
    let sats = ["NSats", "NSat", "numSV"]
        .into_iter()
        .find_map(|name| gps.get(name));
    let hdop = ["HDop", "HDp", "EPH"]
        .into_iter()
        .find_map(|name| gps.get(name));
    let sats_min = sats
        .ok_or_else(|| PyError::attribute("NoneType", "min"))?
        .min()?;
    let hdop_max = hdop
        .ok_or_else(|| PyError::attribute("NoneType", "max"))?
        .max()?;
    let bad_sats_warn = sats_min.lt(&Value::Int(6));
    let bad_hdop_warn = hdop_max.gt(&Value::Float(3.0));
    let bad_sats_fail = sats_min.lt(&Value::Int(5));
    let bad_hdop_fail = hdop_max.gt(&Value::Float(10.0));
    let sats_msg = format!(
        "Min satellites: {}, Max HDop: {}",
        py_str(&sats_min),
        py_str(&hdop_max)
    );
    if glitches > 0 {
        return Err(PyError::other(
            "join() takes exactly one argument (2 given)",
        ));
    }
    if bad_sats_fail || bad_hdop_fail {
        out = Outcome::new(Status::Fail, sats_msg);
    } else if bad_sats_warn || bad_hdop_warn {
        out = Outcome::new(Status::Warn, sats_msg);
    }
    Ok(out)
}

/// An accelerometer sample with its time in seconds.
struct Accel {
    t: f64,
    x: Value,
    y: Value,
    z: Value,
}

/// `TestIMUMatch`: the two IMUs' accelerations, each `IMU` sample against the nearest `IMU2`
/// sample in time, the differences filtered with a five-second time constant, the largest
/// against 0.75 (WARN) and 1.5 (FAIL). The Python's inner loop reuses the outer loop's
/// variable, so the `IMU` sample it subtracts from is the one at the `IMU2` index it stopped
/// at; that is kept. `// tests/TestIMUMatch.py`
fn imu_match(log: &DataflashLog) -> PyResult<Outcome> {
    const WARN: f64 = 0.75;
    const FAIL: f64 = 1.5;
    const FILTER_TC: f64 = 5.0;
    let has_imu = log.channels.contains_key("IMU");
    let has_imu2 = log.channels.contains_key("IMU2");
    if has_imu && !has_imu2 {
        return Ok(Outcome::new(Status::Na, "No IMU2"));
    }
    if !has_imu || !has_imu2 {
        return Ok(Outcome::new(Status::Unknown, "No IMU log data"));
    }
    let gps = log.group("GPS")?;
    let time_label = ["TimeMS", "TimeUS", "Time"]
        .into_iter()
        .find(|l| gps.contains_key(l));
    /// `imu[timeLabel]`: a `KeyError` of `None` when no time label was found.
    fn field<'a>(group: &'a Ordered<Channel>, label: Option<&str>) -> PyResult<&'a Channel> {
        match label {
            None => Err(PyError::key_none()),
            Some(label) => group.get(label).ok_or_else(|| PyError::key(label)),
        }
    }
    let imu1 = log.group("IMU")?;
    let imu2 = log.group("IMU2")?;
    let t1 = field(imu1, time_label)?;
    let (x1, y1, z1) = (
        field(imu1, Some("AccX"))?,
        field(imu1, Some("AccY"))?,
        field(imu1, Some("AccZ"))?,
    );
    let t2 = field(imu2, time_label)?;
    let (x2, y2, z2) = (
        field(imu2, Some("AccX"))?,
        field(imu2, Some("AccY"))?,
        field(imu2, Some("AccZ"))?,
    );
    let multiplier = Value::Float(if time_label == Some("TimeUS") {
        1.0e-6
    } else {
        1.0e-3
    });
    let samples = |t: &Channel, x: &Channel, y: &Channel, z: &Channel| -> PyResult<Vec<Accel>> {
        let mut out = Vec::with_capacity(t.list.len());
        for (i, (_, time)) in t.list.iter().enumerate() {
            let at = |c: &Channel| {
                c.list
                    .get(i)
                    .map(|(_, v)| v.clone())
                    .ok_or_else(PyError::index)
            };
            out.push(Accel {
                t: time.arith(&multiplier, Op::Mul)?.number()?,
                x: at(x)?,
                y: at(y)?,
                z: at(z)?,
            });
        }
        out.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
        Ok(out)
    };
    let imu1 = samples(t1, x1, y1, z1)?;
    let imu2 = samples(t2, x2, y2, z2)?;
    let mut imu2_index = 0usize;
    let mut last_t: Option<f64> = None;
    let (mut xf, mut yf, mut zf, mut max_diff) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for outer in 0..imu1.len() {
        let t = imu1.get(outer).map_or(0.0, |s| s.t);
        let dt = last_t.map_or(0.0, |l| t - l);
        let dt = if 0.1 < dt { 0.1 } else { dt };
        // find closest imu2 value
        let mut i = outer;
        let mut next: Option<&Accel> = None;
        // `range(imu2_index, len(imu2))`, its bounds fixed as the loop starts.
        let (from, to) = (imu2_index, imu2.len());
        for k in from..to {
            next = imu2.get(k);
            imu2_index = k;
            i = k;
            if next.is_some_and(|n| n.t >= t) {
                break;
            }
        }
        let prev = if imu2_index == 0 {
            imu2.last()
        } else {
            imu2.get(imu2_index - 1)
        }
        .ok_or_else(PyError::index)?;
        let next = next.ok_or_else(|| PyError::attribute("NoneType", "__getitem__"))?;
        let closest = if (next.t - t).abs() < (prev.t - t).abs() {
            next
        } else {
            prev
        };
        let sample = imu1.get(i).ok_or_else(PyError::index)?;
        let xdiff = sample.x.arith(&closest.x, Op::Sub)?.number()?;
        let ydiff = sample.y.arith(&closest.y, Op::Sub)?.number()?;
        let zdiff = sample.z.arith(&closest.z, Op::Sub)?.number()?;
        xf += (xdiff - xf) * dt / FILTER_TC;
        yf += (ydiff - yf) * dt / FILTER_TC;
        zf += (zdiff - zf) * dt / FILTER_TC;
        let diff = (xf * xf + yf * yf + zf * zf).sqrt();
        if diff > max_diff {
            max_diff = diff;
        }
        last_t = Some(t);
    }
    let numbers = format!(
        "(Mismatch: {}, WARN: {}, FAIL: {})",
        fixed_f(max_diff, 2),
        fixed_f(WARN, 2),
        fixed_f(FAIL, 2)
    );
    Ok(if max_diff > FAIL {
        Outcome::new(
            Status::Fail,
            format!("Check vibration or accelerometer calibration. {numbers}"),
        )
    } else if max_diff > WARN {
        Outcome::new(
            Status::Warn,
            format!("Check vibration or accelerometer calibration. {numbers}"),
        )
    } else {
        Outcome::new(Status::Good, numbers)
    })
}

/// `TestBalanceTwist` ("Motor Balance"): the average of each motor's output while the average of
/// all is above the minimum throttle, the spread between the highest and lowest motor warned
/// above 75 and failed above 150. UNKNOWN without `RCOU`, with fewer than two channels or with
/// nothing above the minimum. Sums of whole numbers divide as whole numbers.
/// `// tests/TestMotorBalance.py`
fn motor_balance(log: &DataflashLog) -> PyResult<Outcome> {
    if log.vehicle_type != Some(VehicleType::Copter) {
        return Ok(Outcome::new(Status::Na, ""));
    }
    let mut out = Outcome::new(Status::Unknown, "");
    let Some(rcou) = log.channels.get("RCOU") else {
        return Ok(out);
    };
    let mut columns: Vec<&Vec<(usize, Value)>> = Vec::new();
    for i in 1..=8 {
        for prefix in ["Chan", "Ch", "C"] {
            if let Some(channel) = rcou.get(&format!("{prefix}{i}")) {
                columns.push(&channel.list);
            }
        }
    }
    // `zip(*ch)`: a row a sample, as many as the shortest column.
    let rows_count = columns.iter().map(|c| c.len()).min().unwrap_or(0);
    let mut rows: Vec<Vec<&Value>> = (0..rows_count)
        .map(|r| {
            columns
                .iter()
                .filter_map(|c| c.get(r).map(|(_, v)| v))
                .collect()
        })
        .collect();
    let mut num_channels = 0usize;
    for row in &mut rows {
        row.retain(|v| v.gt(&Value::Int(0)) && v.lt(&Value::Int(3000)));
        num_channels = num_channels.max(row.len());
    }
    if log.frame.as_deref().is_some_and(|f| !f.is_empty()) {
        num_channels = usize::try_from(log.num_motor_channels()?).unwrap_or(0);
    }
    if num_channels < 2 {
        return Ok(out);
    }
    let by_rc3 = || -> PyResult<Value> {
        let rc3_min = log.parameter("RC3_MIN")?;
        let thr_min = log.parameter("THR_MIN")?;
        let rc3_max = log.parameter("RC3_MAX")?;
        rc3_min.arith(
            &thr_min
                .arith(&rc3_max.arith(rc3_min, Op::Sub)?, Op::Div)?
                .arith(&Value::Float(1000.0), Op::Div)?,
            Op::Add,
        )
    };
    let min_throttle = match by_rc3() {
        Ok(value) => value,
        Err(e) if e.key_error => {
            let pwm_min = log.parameter("MOT_PWM_MIN")?;
            let pwm_max = log.parameter("MOT_PWM_MAX")?;
            let rc3_min = log.parameter("RC3_MIN")?;
            pwm_min
                .arith(&pwm_max.arith(rc3_min, Op::Sub)?, Op::Div)?
                .arith(&Value::Float(1000.0), Op::Div)?
        }
        Err(e) => return Err(e),
    };
    let channels = Value::Int(count(num_channels));
    let mut kept: Vec<Vec<&Value>> = Vec::new();
    for row in rows {
        if py_sum(row.iter().copied())?
            .arith(&channels, Op::Div)?
            .gt(&min_throttle)
        {
            kept.push(row);
        }
    }
    if kept.is_empty() {
        return Ok(out);
    }
    let mut avg_sum = Value::Int(0);
    let mut averages = Vec::with_capacity(num_channels);
    for i in 0..num_channels {
        let column = kept
            .iter()
            .map(|row| row.get(i).copied().ok_or_else(PyError::index))
            .collect::<PyResult<Vec<&Value>>>()?;
        let avg =
            py_sum(column.iter().copied())?.arith(&Value::Int(count(column.len())), Op::Div)?;
        avg_sum = avg_sum.arith(&avg, Op::Add)?;
        averages.push(avg);
    }
    let avg_all = avg_sum.arith(&channels, Op::Div)?;
    let spread = py_min(averages.iter())?
        .arith(&py_max(averages.iter())?, Op::Sub)?
        .abs()?;
    out.message = format!(
        "Motor channel averages = {}\nAverage motor output = {}\nDifference between min and max motor averages = {}",
        list_str(&averages),
        fixed(&avg_all, 0)?,
        fixed(&spread, 0)?
    );
    out.status = Status::Good;
    if spread.gt(&Value::Int(75)) {
        out.status = Status::Warn;
    }
    if spread.gt(&Value::Int(150)) {
        out.status = Status::Fail;
    }
    Ok(out)
}

/// `TestNaN`: every channel of every group with a NaN in it, once each. The Python walks its
/// dicts in hash order; this walks them as the log declared them. `// tests/TestNaN.py`
fn nan(log: &DataflashLog) -> PyResult<Outcome> {
    let mut out = Outcome::good();
    for (group, channels) in log.channels.iter() {
        for (field, channel) in channels.iter() {
            if channel.list.iter().any(|(_, v)| v.is_nan()) {
                out.fail();
                out.message
                    .push_str(&format!("Found NaN in {group}.{field}\n"));
            }
        }
    }
    Ok(out)
}

/// `numpy`'s `IndexError` for an array read past its end.
fn np_index(index: usize, size: usize) -> PyError {
    PyError::other(format!(
        "index {index} is out of bounds for axis 0 with size {size}"
    ))
}

/// `numpy.polyfit(x, y, 1, cov=True)` as numpy 1.11.2, the runner's, computes it: the slope and
/// intercept of the least-squares line, and the variance of the slope - `inv(XᵀX)[0][0]` scaled
/// by the residual over `n - 4` (that numpy's "extra -2.0 factor"), which is infinite for four
/// points and negative for three. An empty `x` raises as numpy's check does; a singular `XᵀX`
/// (every `x` the same) is numpy's `LinAlgError`; two points or fewer leave numpy's `lstsq`
/// without residuals, which then fail to broadcast.
fn polyfit(x: &[f64], y: &[f64]) -> PyResult<((f64, f64), f64)> {
    let n = x.len();
    if n == 0 {
        return Err(PyError::other("expected non-empty vector for x"));
    }
    if n <= 2 {
        return Err(PyError::other(
            "operands could not be broadcast together with shapes (2,2) (0,) ",
        ));
    }
    let nf = usize_to_float(n);
    let sum_x: f64 = x.iter().sum();
    let sum_xx: f64 = x.iter().map(|v| v * v).sum();
    let det = nf * sum_xx - sum_x * sum_x;
    if det == 0.0 {
        return Err(PyError::other("Singular matrix"));
    }
    let mean_x = sum_x / nf;
    let mean_y = y.iter().sum::<f64>() / nf;
    let sxx: f64 = x.iter().map(|v| (v - mean_x) * (v - mean_x)).sum();
    let sxy: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(a, b)| (a - mean_x) * (b - mean_y))
        .sum();
    let slope = sxy / sxx;
    let intercept = mean_y - slope * mean_x;
    let resid: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(a, b)| {
            let e = b - (slope * a + intercept);
            e * e
        })
        .sum();
    // fac = resids / (len(x) - order - 2.0), numpy's division: infinite or NaN over zero.
    let fac = resid / (nf - 4.0);
    Ok(((slope, intercept), (nf / det) * fac))
}

/// `math.sqrt(x)`: a negative raises.
fn py_sqrt(x: f64) -> PyResult<f64> {
    if x < 0.0 {
        return Err(PyError::other("math domain error"));
    }
    Ok(x.sqrt())
}

/// `TestFlow` ("OpticalFlow"): the optical flow sensor's scale calibration - the flow rate
/// against the body rate over the stretch of the log the vehicle was rocked past 15°, a line
/// fitted to each axis, the scale factors that would make its slope one, FAILs for a poor fit
/// or excessive factors, and the recommendation. A `KeyError` is a FAIL with the key "not
/// found". The `flow_calibration.param` the Python writes beside the runner is not written:
/// nothing reads it. `// tests/TestOptFlow.py`
fn opt_flow(log: &DataflashLog) -> PyResult<Outcome> {
    const TILT_THRESHOLD: f64 = 15.0;
    const QUALITY_THRESHOLD: f64 = 124.0;
    const MIN_RATE: f64 = 0.0;
    const MAX_RATE: f64 = 2.0;
    const PARAM_STD_THRESHOLD: f64 = 5.0;
    const PARAM_ABS_THRESHOLD: i64 = 200;
    const MIN_NUM_POINTS: i64 = 100;
    let mut out = Outcome::good();
    let body = |out: &mut Outcome| -> PyResult<()> {
        let fx_scaler = log.parameter("FLOW_FXSCALER")?;
        let fy_scaler = log.parameter("FLOW_FYSCALER")?;
        let column = |group: &Ordered<Channel>, name: &str| -> PyResult<Vec<f64>> {
            group
                .get(name)
                .ok_or_else(|| PyError::key(name))?
                .list
                .iter()
                .map(|(_, v)| v.to_float())
                .collect()
        };
        let Some(of) = log.channels.get("OF") else {
            out.fail();
            out.message = "FAIL: no optical flow data\n".to_owned();
            return Ok(());
        };
        let flow_x = column(of, "flowX")?;
        let body_x = column(of, "bodyX")?;
        let flow_y = column(of, "flowY")?;
        let body_y = column(of, "bodyY")?;
        let flow_time = column(of, "TimeUS")?;
        let flow_qual = column(of, "Qual")?;
        let Some(att) = log.channels.get("ATT") else {
            out.fail();
            out.message = "FAIL: no attitude data\n".to_owned();
            return Ok(());
        };
        let roll = column(att, "Roll")?;
        let pitch = column(att, "Pitch")?;
        let att_time = column(att, "TimeUS")?;
        let at = |values: &[f64], i: usize| -> PyResult<f64> {
            values
                .get(i)
                .copied()
                .ok_or_else(|| np_index(i, values.len()))
        };
        // The stretch between the first and last tilt past the threshold, in flow samples.
        let window = |angle: &[f64]| -> PyResult<(usize, usize)> {
            let mut start_time = 0.0;
            for (i, a) in angle.iter().enumerate() {
                if a.abs() > TILT_THRESHOLD {
                    start_time = at(&att_time, i)?;
                    break;
                }
            }
            let start = flow_time.iter().position(|t| *t > start_time).unwrap_or(0);
            let mut end_time = 0.0;
            for (i, a) in angle.iter().enumerate().rev() {
                if a.abs() > TILT_THRESHOLD {
                    end_time = at(&att_time, i)?;
                    break;
                }
            }
            let end = flow_time.iter().rposition(|t| *t < end_time).unwrap_or(0);
            Ok((start, end))
        };
        // Resampled: the window's samples with a body rate inside the limits and good quality.
        let resample = |(start, end): (usize, usize),
                        flow: &[f64],
                        body: &[f64]|
         -> PyResult<(Vec<f64>, Vec<f64>)> {
            let mut flows = Vec::new();
            let mut bodies = Vec::new();
            for i in 0..roll.len() {
                if i >= start
                    && i <= end
                    && at(body, i)?.abs() > MIN_RATE
                    && at(body, i)?.abs() < MAX_RATE
                    && at(&flow_qual, i)? > QUALITY_THRESHOLD
                {
                    flows.push(at(flow, i)?);
                    bodies.push(at(body, i)?);
                }
            }
            Ok((flows, bodies))
        };
        let roll_window = window(&roll)?;
        if count(roll_window.1) - count(roll_window.0) <= MIN_NUM_POINTS {
            out.fail();
            out.message = "FAIL: insufficient roll data pointsa\n".to_owned();
            return Ok(());
        }
        let (flow_x_res, body_x_res) = resample(roll_window, &flow_x, &body_x)?;
        let pitch_window = window(&pitch)?;
        if count(pitch_window.1) - count(pitch_window.0) <= MIN_NUM_POINTS {
            out.fail();
            out.message = "FAIL: insufficient pitch data pointsa\n".to_owned();
            return Ok(());
        }
        let (flow_y_res, body_y_res) = resample(pitch_window, &flow_y, &body_y)?;
        let ((slope_x, _), cov_x) = polyfit(&body_x_res, &flow_x_res)?;
        let ((slope_y, _), cov_y) = polyfit(&body_y_res, &flow_y_res)?;
        // numpy's division: a slope of zero makes an infinity, which int() refuses.
        let fx_new =
            float_to_int(1000.0 * ((1.0 + 0.001 * fx_scaler.to_float()?) / slope_x - 1.0))?;
        let fy_new =
            float_to_int(1000.0 * ((1.0 + 0.001 * fy_scaler.to_float()?) / slope_y - 1.0))?;
        let std_x = py_sqrt(cov_x)?;
        let std_y = py_sqrt(cov_y)?;
        if std_x > PARAM_STD_THRESHOLD || std_y > PARAM_STD_THRESHOLD {
            out.fail();
            out.message = format!(
                "FAIL: inaccurate fit - poor quality or insufficient data\nFLOW_FXSCALER 1STD = {}\nFLOW_FYSCALER 1STD = {}\n",
                decimal_f(py_round(1000.0 * std_x))?,
                decimal_f(py_round(1000.0 * std_y))?
            );
        }
        if fx_new.abs() > PARAM_ABS_THRESHOLD || fy_new.abs() > PARAM_ABS_THRESHOLD {
            out.fail();
            out.message = format!(
                "FAIL: required scale factors are excessive\nFLOW_FXSCALER={}\nFLOW_FYSCALER={}\n",
                decimal(fx_scaler)?,
                decimal(fy_scaler)?
            );
        }
        out.message = format!(
            "Set FLOW_FXSCALER to {fx_new}\nSet FLOW_FYSCALER to {fy_new}\n\nCal plots saved to flow_calibration.pdf\nCal parameters saved to flow_calibration.param\n\nFLOW_FXSCALER 1STD = {}\nFLOW_FYSCALER 1STD = {}\n",
            decimal_f(py_round(1000.0 * std_x))?,
            decimal_f(py_round(1000.0 * std_y))?
        );
        Ok(())
    };
    match body(&mut out) {
        Ok(()) => Ok(out),
        Err(e) if e.key_error => {
            out.status = Status::Fail;
            out.message = format!("{} not found", e.text);
            Ok(out)
        }
        Err(e) => Err(e),
    }
}

/// `TestParams`: parameters that are NaN, then a copter's `MAG_ENABLE`, `THR_MIN` and `THR_MID`
/// against their limits; a `KeyError` is a FAIL with the key "not found", replacing the message.
/// The parameters come in the order the log declared them, not the Python dict's.
/// `// tests/TestParams.py`
fn params(log: &DataflashLog) -> PyResult<Outcome> {
    let mut out = Outcome::good();
    for (name, value) in log.parameters.iter() {
        if value.isnan()? {
            out.status = Status::Fail;
            out.message.push_str(&format!("{name} is NaN\n"));
        }
    }
    let body = |out: &mut Outcome| -> PyResult<()> {
        if log.vehicle_type == Some(VehicleType::Copter) {
            let mut check = |name: &str,
                             expected: i64,
                             bad: fn(&Value, &Value) -> bool,
                             wording: &str|
             -> PyResult<()> {
                let value = log.parameter(name)?;
                let expected = Value::Int(expected);
                if bad(value, &expected) {
                    out.status = Status::Fail;
                    out.message.push_str(&format!(
                        "{name} set to {}, expecting {wording}{}\n",
                        py_repr(value),
                        py_repr(&expected)
                    ));
                }
                Ok(())
            };
            check("MAG_ENABLE", 1, |v, e| !v.eq(e), "")?;
            check("THR_MIN", 200, Value::ge, "less than ")?;
            check("THR_MID", 701, Value::ge, "less than ")?;
            // The Python's "more than" check says "less than" too.
            check("THR_MID", 299, Value::le, "less than ")?;
        }
        if out.status == Status::Fail {
            out.message = format!("Bad parameters found:\n{}", out.message);
        }
        Ok(())
    };
    match body(&mut out) {
        Ok(()) => Ok(out),
        Err(e) if e.key_error => {
            out.status = Status::Fail;
            out.message = format!("{} not found", e.text);
            Ok(out)
        }
        Err(e) => Err(e),
    }
}

/// `TestPerformance` ("PM"): a copter's slow loops, `NLon` over `NLoop` above 6% a line; more
/// than six such lines or one above 10% a FAIL, any a WARN. `// tests/TestPerformance.py`
fn performance(log: &DataflashLog) -> PyResult<Outcome> {
    if log.vehicle_type != Some(VehicleType::Copter) {
        return Ok(Outcome::new(Status::Na, ""));
    }
    if !log.channels.contains_key("PM") {
        return Ok(Outcome::new(Status::Unknown, "No PM log data"));
    }
    let n_lon = &log.channel("PM", "NLon")?.list;
    let (mut max_slow, mut max_slow_line, mut slow_lines) = (0.0f64, 0usize, 0i64);
    for i in 0..n_lon.len() {
        let (line, lon) = n_lon.get(i).ok_or_else(PyError::index)?;
        let (_, n_loop) = log
            .channel("PM", "NLoop")?
            .list
            .get(i)
            .ok_or_else(PyError::index)?;
        let _max_t = log
            .channel("PM", "MaxT")?
            .list
            .get(i)
            .ok_or_else(PyError::index)?;
        let percent_slow = lon
            .arith(&Value::Float(n_loop.to_float()?), Op::Div)?
            .arith(&Value::Float(100.0), Op::Mul)?
            .number()?;
        if percent_slow > 6.0 {
            slow_lines += 1;
            if percent_slow > max_slow {
                max_slow = percent_slow;
                max_slow_line = *line;
            }
        }
    }
    let message = || {
        format!(
            "{slow_lines} slow loop lines found, max {}% on line {max_slow_line}",
            fixed_f(max_slow, 2)
        )
    };
    Ok(if max_slow > 10.0 || slow_lines > 6 {
        Outcome::new(Status::Fail, message())
    } else if max_slow > 6.0 {
        Outcome::new(Status::Warn, message())
    } else {
        Outcome::good()
    })
}

/// `TestPitchRollCoupling` ("Pitch/Roll"): a copter's roll and pitch beyond the maximum lean
/// angle (`ANGLE_MAX`, or 45°) plus ten, over its auto and manual flight segments and above two
/// metres; a mode the Python does not list raises. `// tests/TestPitchRollCoupling.py`
fn pitch_roll_coupling(log: &DataflashLog) -> PyResult<Outcome> {
    const AUTO_MODES: [&str; 8] = [
        "RTL",
        "AUTO",
        "LAND",
        "LOITER",
        "GUIDED",
        "CIRCLE",
        "OF_LOITER",
        "HYBRID",
    ];
    const MANUAL_MODES: [&str; 5] = ["STABILIZE", "DRIFT", "ALTHOLD", "ALT_HOLD", "POSHOLD"];
    const IGNORE_MODES: [&str; 5] = ["ACRO", "SPORT", "FLIP", "AUTOTUNE", ""];
    if log.vehicle_type != Some(VehicleType::Copter) {
        return Ok(Outcome::new(Status::Na, ""));
    }
    if !log.channels.contains_key("ATT") {
        return Ok(Outcome::new(Status::Unknown, "No ATT log data"));
    }
    let mut ordered = log.mode_changes.clone();
    ordered.sort_by_key(|(line, _)| *line);
    let mut auto_segments: Vec<(usize, usize)> = Vec::new();
    let mut manual_segments: Vec<(usize, usize)> = Vec::new();
    let mut is_auto = false; // we always start in a manual control mode
    let mut prev_line = 0usize;
    let mut mode = String::new();
    for (line, (name, _)) in &ordered {
        mode = match name {
            Value::Str(text) => text.to_ascii_uppercase(),
            other => return Err(PyError::attribute(other.type_name(), "upper")),
        };
        if prev_line == 0 {
            prev_line = *line;
        }
        let before = line.saturating_sub(1);
        if AUTO_MODES.contains(&mode.as_str()) {
            if !is_auto {
                manual_segments.push((prev_line, before));
                prev_line = *line;
            }
            is_auto = true;
        } else if MANUAL_MODES.contains(&mode.as_str()) {
            if is_auto {
                auto_segments.push((prev_line, before));
                prev_line = *line;
            }
            is_auto = false;
        } else if IGNORE_MODES.contains(&mode.as_str()) {
            if is_auto {
                auto_segments.push((prev_line, before));
            } else {
                manual_segments.push((prev_line, before));
            }
            prev_line = 0;
        } else {
            return Err(PyError::other(format!(
                "Unknown mode in TestPitchRollCoupling: {mode}"
            )));
        }
    }
    // and handle the last segment, which doesn't have an ending
    if AUTO_MODES.contains(&mode.as_str()) {
        auto_segments.push((prev_line, log.line_count));
    } else if MANUAL_MODES.contains(&mode.as_str()) {
        manual_segments.push((prev_line, log.line_count));
    }
    let mut max_lean_angle = 45.0;
    if let Some(angle_max) = log.parameters.get("ANGLE_MAX") {
        max_lean_angle = angle_max.arith(&Value::Float(100.0), Op::Div)?.number()?;
    }
    let limit = Value::Float(max_lean_angle + 10.0);
    let min_alt = Value::Float(2.0);
    let (mut max_roll, mut max_roll_line) = (Value::Float(0.0), 0usize);
    let (mut max_pitch, mut max_pitch_line) = (Value::Float(0.0), 0usize);
    for (start, end) in manual_segments.iter().chain(auto_segments.iter()) {
        let roll_segment = log.channel("ATT", "Roll")?.segment(*start, *end);
        let pitch_segment = log.channel("ATT", "Pitch")?.segment(*start, *end);
        if roll_segment.dict_is_empty() && pitch_segment.dict_is_empty() {
            continue;
        }
        let roll = py_max([&roll_segment.min()?.abs()?, &roll_segment.max()?.abs()?])?;
        let pitch = py_max([&pitch_segment.min()?.abs()?, &pitch_segment.max()?.abs()?])?;
        if (roll.gt(&limit) && roll.abs()?.gt(&max_roll.abs()?))
            || (pitch.gt(&limit) && pitch.abs()?.gt(&max_pitch.abs()?))
        {
            let mut lit = LogIterator::new(log, *start)?;
            if lit.current_line != *start {
                return Err(PyError::other(""));
            }
            while lit.current_line <= *end {
                let relative_alt = lit.get("CTUN", "BarAlt")?;
                if relative_alt.gt(&min_alt) {
                    let roll = lit.get("ATT", "Roll")?;
                    let pitch = lit.get("ATT", "Pitch")?;
                    if roll.abs()?.gt(&limit) && roll.abs()?.gt(&max_roll.abs()?) {
                        max_roll = roll;
                        max_roll_line = lit.current_line;
                    }
                    if pitch.abs()?.gt(&limit) && pitch.abs()?.gt(&max_pitch.abs()?) {
                        max_pitch = pitch;
                        max_pitch_line = lit.current_line;
                    }
                }
                lit.advance()?;
            }
        }
    }
    if truthy(&max_roll) && max_roll.abs()?.gt(&max_pitch.abs()?) {
        return Ok(Outcome::new(
            Status::Fail,
            format!(
                "Roll ({}, line {max_roll_line}) > maximum lean angle ({})",
                fixed(&max_roll, 2)?,
                fixed_f(max_lean_angle, 2)
            ),
        ));
    }
    if truthy(&max_pitch) {
        return Ok(Outcome::new(
            Status::Fail,
            format!(
                "Pitch ({}, line {max_pitch_line}) > maximum lean angle ({})",
                fixed(&max_pitch, 2)?,
                fixed_f(max_lean_angle, 2)
            ),
        ));
    }
    Ok(Outcome::good())
}

/// `TestThrust`: a copter's stretches of throttle above 700 while level, longer than fifty
/// samples, whose average climb rate is below 50 cm/s (FAIL) or 100 cm/s (WARN).
/// `// tests/TestThrust.py`
fn thrust(log: &DataflashLog) -> PyResult<Outcome> {
    const HIGH_THROTTLE: i64 = 700;
    const TILT: i64 = 20;
    const CLIMB_WARN: f64 = 100.0;
    const CLIMB_FAIL: f64 = 50.0;
    const MIN_SAMPLES: usize = 50;
    if log.vehicle_type != Some(VehicleType::Copter) {
        return Ok(Outcome::new(Status::Na, ""));
    }
    if !log.channels.contains_key("CTUN") {
        return Ok(Outcome::new(Status::Unknown, "No CTUN log data"));
    }
    if !log.channels.contains_key("ATT") {
        return Ok(Outcome::new(Status::Unknown, "No ATT log data"));
    }
    let ctun = log.group("CTUN")?;
    let Some(throttle_key) = ["ThO", "ThrOut"].into_iter().find(|k| ctun.contains_key(k)) else {
        return Ok(Outcome::new(
            Status::Unknown,
            "Could not find throttle out column",
        ));
    };
    let data = &log.channel("CTUN", throttle_key)?.list;
    let high = Value::Int(HIGH_THROTTLE);
    let tilt = Value::Int(TILT);
    let mut segments: Vec<(usize, usize)> = Vec::new();
    let mut start: Option<usize> = None;
    for (i, (line, value)) in data.iter().enumerate() {
        let mut below_tilt = true;
        if value.gt(&high) {
            let (roll, _) = log.channel("ATT", "Roll")?.nearest(*line, true)?;
            let (pitch, _) = log.channel("ATT", "Pitch")?.nearest(*line, true)?;
            if roll.abs()?.gt(&tilt) || pitch.abs()?.gt(&tilt) {
                below_tilt = false;
            }
        }
        if value.gt(&high) && below_tilt {
            if start.is_none() {
                start = Some(i);
            }
        } else if let Some(from) = start {
            if i - from > MIN_SAMPLES {
                segments.push((from, i));
            }
            start = None;
        }
    }
    let climb_rate = if ctun.contains_key("CRate") {
        "CRate"
    } else {
        "CRt"
    };
    let mut out = Outcome::good();
    for (from, to) in segments {
        let start_line = data.get(from).map_or(0, |(line, _)| *line);
        let end_line = data.get(to).map_or(0, |(line, _)| *line);
        let avg_climb = log
            .channel("CTUN", climb_rate)?
            .segment(start_line, end_line)
            .avg()?;
        let avg_throttle = log
            .channel("CTUN", throttle_key)?
            .segment(start_line, end_line)
            .avg()?;
        if avg_climb < CLIMB_FAIL {
            return Ok(Outcome::new(
                Status::Fail,
                format!(
                    "Avg climb rate {} cm/s for throttle avg {}",
                    fixed_f(avg_climb, 2),
                    decimal_f(avg_throttle)?
                ),
            ));
        }
        if avg_climb < CLIMB_WARN {
            out = Outcome::new(
                Status::Warn,
                format!(
                    "Avg climb rate {}  cm/s for throttle avg {}",
                    fixed_f(avg_climb, 2),
                    decimal_f(avg_throttle)?
                ),
            );
        }
    }
    Ok(out)
}

/// `TestVCC`: the board's supply from `CURR.Vcc` in millivolts (or `POWR.Vcc` in volts, scaled),
/// a WARN when it swings more than 0.3 V and a FAIL below 4.6 V. `// tests/TestVCC.py`
fn vcc(log: &DataflashLog) -> PyResult<Outcome> {
    if !log.channels.contains_key("CURR") {
        return Ok(Outcome::new(Status::Unknown, "No CURR log data"));
    }
    let from_curr = || -> PyResult<(Value, Value)> {
        let channel = log.channel("CURR", "Vcc")?;
        Ok((channel.min()?, channel.max()?))
    };
    let (vcc_min, vcc_max) = match from_curr() {
        Ok(both) => both,
        Err(e) if e.key_error => {
            let channel = log.channel("POWR", "Vcc")?;
            let (min, max) = (channel.min()?, channel.max()?);
            (
                min.arith(&Value::Int(1000), Op::Mul)?,
                max.arith(&Value::Int(1000), Op::Mul)?,
            )
        }
        Err(e) => return Err(e),
    };
    let diff = vcc_max.arith(&vcc_min, Op::Sub)?;
    let min_threshold = Value::Float(4.6 * 1000.0);
    let max_diff = Value::Float(0.3 * 1000.0);
    let thousand = Value::Float(1000.0);
    Ok(if diff.gt(&max_diff) {
        Outcome::new(
            Status::Warn,
            format!(
                "VCC min/max diff {}v, should be <{}v",
                py_str(&diff.arith(&thousand, Op::Div)?),
                py_str(&max_diff.arith(&thousand, Op::Div)?)
            ),
        )
    } else if vcc_min.lt(&min_threshold) {
        Outcome::new(
            Status::Fail,
            format!(
                "VCC below minimum of {}v ({}v)",
                repr_float(4.6),
                py_repr(&vcc_min.arith(&thousand, Op::Div)?)
            ),
        )
    } else {
        Outcome::good()
    })
}

/// `TestVibration`: twice the standard deviation of each accelerometer axis over the longest
/// stretch of LOITER longer than ten seconds, against 1.5/3.0 g sideways and 2.0/5.0 g
/// vertically. `// tests/TestVibration.py`
fn vibration(log: &DataflashLog) -> PyResult<Outcome> {
    const WARN_XY: f64 = 1.5;
    const FAIL_XY: f64 = 3.0;
    const WARN_Z: f64 = 2.0;
    const FAIL_Z: f64 = 5.0;
    if log.vehicle_type != Some(VehicleType::Copter) {
        return Ok(Outcome::new(Status::Na, ""));
    }
    if !log.channels.contains_key("IMU") {
        return Ok(Outcome::new(Status::Unknown, "No IMU log data"));
    }
    // find some stable LOITER data to analyze, at least 10 seconds
    let chunks = find_loiter_chunks(log, 10.0)?;
    let Some((start, end)) = chunks.first() else {
        return Ok(Outcome::new(
            Status::Unknown,
            "No stable LOITER log data found",
        ));
    };
    let std_dev = |axis: &str| -> PyResult<f64> {
        let segment = log.channel("IMU", axis)?.segment(*start, *end);
        let values: Vec<&Value> = segment.values().collect();
        np_std(values.iter().copied())
    };
    // use 2x standard deviations as the metric, so if 95% of samples lie within the aim range
    let x = (2.0 * std_dev("AccX")?).abs();
    let y = (2.0 * std_dev("AccY")?).abs();
    let z = (2.0 * std_dev("AccZ")?).abs();
    let numbers = format!(
        "(X:{}g, Y:{}g, Z:{}g)",
        fixed_f(x, 2),
        fixed_f(y, 2),
        fixed_f(z, 2)
    );
    Ok(if x > FAIL_XY || y > FAIL_XY || z > FAIL_Z {
        Outcome::new(Status::Fail, format!("Vibration too high {numbers}"))
    } else if x > WARN_XY || y > WARN_XY || z > WARN_Z {
        Outcome::new(Status::Warn, format!("Vibration slightly high {numbers}"))
    } else {
        Outcome::new(Status::Good, format!("Good vibration values {numbers}"))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n\
                        FMT, 32, 35, PARM, QNff, TimeUS,Name,Value,Default\n\
                        FMT, 123, 14, MODE, QMBB, TimeUS,Mode,ModeNum,Rsn\n\
                        FMT, 10, 10, MSG, QZ, TimeUS,Message\n\
                        FMT, 1, 1, EV, QB, TimeUS,Id\n\
                        FMT, 2, 2, ERR, QBB, TimeUS,Subsys,ECode\n\
                        FMT, 3, 3, CTUN, Qfff, TimeUS,ThrOut,BarAlt,CRate\n\
                        FMT, 4, 4, ATT, Qff, TimeUS,Roll,Pitch\n\
                        FMT, 5, 5, GPS, QBf, TimeUS,NSats,HDop\n\
                        FMT, 6, 6, MAG, Qffffff, TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ\n\
                        FMT, 7, 7, CURR, Qf, TimeUS,Vcc\n\
                        FMT, 8, 8, PM, QHHI, TimeUS,NLon,NLoop,MaxT\n\
                        FMT, 9, 9, RCOU, QHHHH, TimeUS,C1,C2,C3,C4\n\
                        FMT, 11, 11, IMU, Qfff, TimeUS,AccX,AccY,AccZ\n\
                        FMT, 12, 12, IMU2, Qfff, TimeUS,AccX,AccY,AccZ\n\
                        MSG, 1, ArduCopter V4.5.7 (1a2b3c4d)\n";

    fn copter(body: &str) -> DataflashLog {
        DataflashLog::read(&format!("{HEAD}{body}"), "t.log", true).unwrap()
    }

    fn run(check: Check, body: &str) -> Outcome {
        check(&copter(body)).unwrap()
    }

    fn fails(check: Check, body: &str) -> String {
        check(&copter(body)).unwrap_err().text
    }

    /// The copter log with one of the head's formats declared differently.
    fn reformatted(format: &str, instead: &str, body: &str) -> DataflashLog {
        let head = HEAD.replace(format, instead);
        assert_ne!(head, HEAD);
        DataflashLog::read(&format!("{head}{body}"), "t.log", true).unwrap()
    }

    #[test]
    fn autotune_sessions_are_read_last_first_and_the_guard_resets() {
        assert_eq!(
            run(autotune, ""),
            Outcome::new(Status::Unknown, "No ATUN log data")
        );
        let atun = "FMT, 20, 20, ATUN, QB, TimeUS,Axis\nFMT, 21, 21, ATDE, QB, TimeUS,Angle\nATUN, 1, 0\nATDE, 1, 0\n";
        // EV missing, but ATUN present: the flag was reset, and the KeyError follows.
        assert_eq!(fails(autotune, atun), "'EV'");
        let events = "EV, 1, 30\nEV, 2, 34\nEV, 3, 30\nEV, 4, 10\nEV, 5, 33\n";
        let out = run(autotune, &format!("{atun}{events}"));
        assert_eq!(out.status, Status::Good);
        assert_eq!(out.message, "[-] Autotune 21-22\n[+] Autotune 23-25\n");
        let plane =
            DataflashLog::read(&HEAD.replace("ArduCopter", "ArduPlane"), "t.log", true).unwrap();
        assert_eq!(autotune(&plane).unwrap(), Outcome::new(Status::Na, ""));
    }

    #[test]
    fn brownout_is_a_log_ending_armed_and_high() {
        assert_eq!(
            run(brownout, ""),
            Outcome::new(Status::Unknown, "No CTUN log data")
        );
        let good = "EV, 1, 10\nCTUN, 2, 500, 25.5, 0\nEV, 3, 11\n";
        assert_eq!(run(brownout, good), Outcome::good());
        let cut = "EV, 1, 10\nCTUN, 2, 500, 25.5, 0\n";
        assert_eq!(
            run(brownout, cut),
            Outcome::new(
                Status::Fail,
                "Truncated Log? Ends while armed at altitude 25.50m"
            )
        );
    }

    #[test]
    fn compass_reports_offsets_field_change_and_missing_keys() {
        assert_eq!(
            run(compass, ""),
            Outcome::new(Status::Fail, "'COMPASS_OFS_X' not found")
        );
        let params = "PARM, 1, COMPASS_OFS_X, 184.96, 0\nPARM, 1, COMPASS_OFS_Y, -25.32, 0\nPARM, 1, COMPASS_OFS_Z, 250.07, 0\n";
        let out = run(compass, params);
        assert_eq!(out.status, Status::Warn);
        assert_eq!(
            out.message,
            "WARN: Large compass offset params (X:184.96, Y:-25.32, Z:250.07)\nNo MAG data, unable to test mag_field interference\n"
        );
        let mag = "MAG, 1, 300, 0, 0, 184, -25, 250\nMAG, 2, 0, 0, 0, 184, -25, 250\nMAG, 3, 0, 420, 0, 184, -25, 250\n";
        let out = run(compass, &format!("{params}{mag}"));
        assert_eq!(out.status, Status::Fail);
        assert_eq!(
            out.message,
            "WARN: Large compass offset params (X:184.96, Y:-25.32, Z:250.07)\n\
             WARN: Large compass offset in MAG data (X:184.00, Y:-25.00, Z:250.00)\n\
             Large change in mag_field (40.00%)\n\
             All zeros found in MAG X/Y/Z log data\n"
        );
        // A first sample of zeros leaves the minimum unset: the Python's `mf < None`.
        let zero_first = "MAG, 1, 0, 0, 0, 1, 1, 1\nMAG, 2, 300, 0, 0, 1, 1, 1\n";
        let out = run(compass, &format!("{params}{zero_first}"));
        assert!(
            out.message.contains("No valid mag data found\n"),
            "{}",
            out.message
        );
        let no_offsets = reformatted(
            "FMT, 6, 6, MAG, Qffffff, TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ\n",
            "FMT, 6, 6, MAG, Qf, TimeUS,MagX\n",
            &format!("{params}MAG, 1, 5\n"),
        );
        assert_eq!(
            compass(&no_offsets).unwrap(),
            Outcome::new(Status::Fail, "'OfsX' not found")
        );
    }

    #[test]
    fn dupe_log_data_finds_a_repeated_chunk() {
        assert_eq!(
            run(dupe_log_data, ""),
            Outcome::new(Status::Unknown, "No ATT log data")
        );
        assert_eq!(
            fails(dupe_log_data, "ATT, 1, 1, 2\n"),
            "range() step argument must not be zero"
        );
        let mut varied = String::new();
        for i in 0..300 {
            varied.push_str(&format!("ATT, {i}, 0, {i}\n"));
        }
        assert_eq!(run(dupe_log_data, &varied), Outcome::good());
        // The same 40 samples again a hundred later: the sample taken at index 108 (line 125) is
        // found again at index 208 (line 225).
        let mut duped = String::new();
        for i in 0..300 {
            let v = if (100..140).contains(&i) {
                (i - 100) * 3
            } else if (200..240).contains(&i) {
                (i - 200) * 3
            } else {
                i
            };
            duped.push_str(&format!("ATT, {i}, 0, {v}\n"));
        }
        assert_eq!(
            run(dupe_log_data, &duped),
            Outcome::new(
                Status::Fail,
                "Duplicate data chunks found in log (125 and 225)"
            )
        );
    }

    #[test]
    fn empty_events_and_gps_follow_the_tables() {
        assert_eq!(
            run(empty, "CTUN, 1, 100, 0, 0\n"),
            Outcome::new(Status::Fail, "Empty log? Throttle never above 20%")
        );
        assert_eq!(run(empty, "CTUN, 1, 300, 0, 0\n"), Outcome::good());
        assert_eq!(
            run(events, "ERR, 1, 9, 1\n"),
            Outcome::new(Status::Warn, "FENCE ")
        );
        assert_eq!(
            run(events, "ERR, 1, 7, 1\n"),
            Outcome::new(Status::Fail, "ERR found: GPS ")
        );
        assert_eq!(
            run(events, "ERR, 1, 7, 1\nERR, 2, 11, 2\nERR, 3, 7, 1\n"),
            Outcome::new(Status::Fail, "ERRs found: GPS GPS_GLITCH ")
        );
        assert_eq!(
            run(gps_glitch, ""),
            Outcome::new(Status::Unknown, "No GPS log data")
        );
        assert_eq!(
            run(gps_glitch, "GPS, 1, 6, 4.68\nGPS, 2, 8, 2.1\n"),
            Outcome::new(Status::Warn, "Min satellites: 6, Max HDop: 4.68")
        );
        assert_eq!(
            run(gps_glitch, "GPS, 1, 0, 99.68\n"),
            Outcome::new(Status::Fail, "Min satellites: 0, Max HDop: 99.68")
        );
        assert_eq!(run(gps_glitch, "GPS, 1, 9, 1.2\n"), Outcome::good());
        assert_eq!(
            fails(gps_glitch, "GPS, 1, 9, 1.2\nERR, 2, 11, 2\n"),
            "join() takes exactly one argument (2 given)"
        );
        let no_sats = reformatted(
            "FMT, 5, 5, GPS, QBf, TimeUS,NSats,HDop\n",
            "FMT, 5, 5, GPS, Qf, TimeUS,HDop\n",
            "GPS, 1, 1.2\n",
        );
        assert_eq!(
            gps_glitch(&no_sats).unwrap_err().text,
            "'NoneType' object has no attribute 'min'"
        );
    }

    #[test]
    fn imu_match_filters_the_difference_between_the_imus() {
        assert_eq!(
            run(imu_match, ""),
            Outcome::new(Status::Unknown, "No IMU log data")
        );
        assert_eq!(
            run(imu_match, "IMU, 1, 0, 0, -9.8\n"),
            Outcome::new(Status::Na, "No IMU2")
        );
        assert_eq!(
            fails(imu_match, "IMU, 1, 0, 0, -9.8\nIMU2, 1, 0, 0, -9.8\n"),
            "'GPS'"
        );
        let mut body = String::from("GPS, 1, 9, 1.2\n");
        for i in 0..200 {
            body.push_str(&format!(
                "IMU, {}, 0, 0, -9.8\nIMU2, {}, 3, 0, -9.8\n",
                i * 100_000,
                i * 100_000
            ));
        }
        let out = run(imu_match, &body);
        assert_eq!(out.status, Status::Fail);
        assert_eq!(
            out.message,
            "Check vibration or accelerometer calibration. (Mismatch: 2.95, WARN: 0.75, FAIL: 1.50)"
        );
        let same = body.replace(", 3, 0, -9.8", ", 0, 0, -9.8");
        assert_eq!(
            run(imu_match, &same),
            Outcome::new(Status::Good, "(Mismatch: 0.00, WARN: 0.75, FAIL: 1.50)")
        );
    }

    #[test]
    fn motor_balance_averages_whole_numbers_as_whole_numbers() {
        assert_eq!(run(motor_balance, ""), Outcome::new(Status::Unknown, ""));
        let params =
            "PARM, 1, RC3_MIN, 1100, 0\nPARM, 1, THR_MIN, 130, 0\nPARM, 1, RC3_MAX, 1900, 0\n";
        let mut body = params.to_owned();
        for i in 0..10 {
            body.push_str(&format!("RCOU, {i}, 1500, 1510, 1700, 1500\n"));
        }
        let out = run(motor_balance, &body);
        assert_eq!(out.status, Status::Fail);
        assert_eq!(
            out.message,
            "Motor channel averages = [1500, 1510, 1700, 1500]\nAverage motor output = 1552\nDifference between min and max motor averages = 200"
        );
        let frame = format!("MSG, 1, Frame: QUAD/X\n{body}");
        assert_eq!(fails(motor_balance, &frame), "'QUAD/X'");
        let no_params = body.replace(params, "");
        assert_eq!(fails(motor_balance, &no_params), "'MOT_PWM_MIN'");
    }

    #[test]
    fn nans_params_and_performance() {
        assert_eq!(
            run(nan, "CTUN, 1, 300, nan, 0\nATT, 2, nan, 1\n"),
            Outcome::new(
                Status::Fail,
                "Found NaN in CTUN.BarAlt\nFound NaN in ATT.Roll\n"
            )
        );
        assert_eq!(
            run(params, ""),
            Outcome::new(Status::Fail, "'MAG_ENABLE' not found")
        );
        let good =
            "PARM, 1, MAG_ENABLE, 1, 0\nPARM, 1, THR_MIN, 130, 0\nPARM, 1, THR_MID, 500, 0\n";
        assert_eq!(run(params, good), Outcome::good());
        let bad = "PARM, 1, X, nan, 0\nPARM, 1, MAG_ENABLE, 0, 0\nPARM, 1, THR_MIN, 250, 0\nPARM, 1, THR_MID, 200, 0\n";
        assert_eq!(
            run(params, bad),
            Outcome::new(
                Status::Fail,
                "Bad parameters found:\nX is NaN\nMAG_ENABLE set to 0.0, expecting 1\nTHR_MIN set to 250.0, expecting less than 200\nTHR_MID set to 200.0, expecting less than 299\n"
            )
        );
        assert_eq!(
            run(performance, ""),
            Outcome::new(Status::Unknown, "No PM log data")
        );
        assert_eq!(
            run(performance, "PM, 1, 1, 100, 5\nPM, 2, 19, 100, 5\n"),
            Outcome::new(
                Status::Fail,
                "1 slow loop lines found, max 19.00% on line 18"
            )
        );
        assert_eq!(
            run(performance, "PM, 1, 7, 100, 5\n"),
            Outcome::new(
                Status::Warn,
                "1 slow loop lines found, max 7.00% on line 17"
            )
        );
        let no_nloop = reformatted(
            "FMT, 8, 8, PM, QHHI, TimeUS,NLon,NLoop,MaxT\n",
            "FMT, 8, 8, PM, QH, TimeUS,NLon\n",
            "PM, 1, 7\n",
        );
        assert_eq!(performance(&no_nloop).unwrap_err().text, "'NLoop'");
    }

    #[test]
    fn pitch_roll_coupling_walks_the_segments() {
        assert_eq!(
            run(pitch_roll_coupling, ""),
            Outcome::new(Status::Unknown, "No ATT log data")
        );
        assert_eq!(
            fails(pitch_roll_coupling, "MODE, 1, Brake, 17, 1\nATT, 2, 0, 0\n"),
            "Unknown mode in TestPitchRollCoupling: BRAKE"
        );
        assert_eq!(
            fails(pitch_roll_coupling, "MODE, 1, 8, 8, 1\nATT, 2, 0, 0\n"),
            "'int' object has no attribute 'upper'"
        );
        let level = "MODE, 1, Loiter, 5, 1\nATT, 2, 10, -5\nCTUN, 3, 500, 20, 0\nATT, 4, 20, 5\n";
        assert_eq!(run(pitch_roll_coupling, level), Outcome::good());
        // The line reported is the iterator's current line, where the segment starts: the
        // iterator indexes the next sample at or after it, not the sample's own line.
        let steep = "MODE, 1, Loiter, 5, 1\nCTUN, 2, 500, 20, 0\nATT, 3, 70, -5\nATT, 4, 20, 5\n";
        assert_eq!(
            run(pitch_roll_coupling, steep),
            Outcome::new(
                Status::Fail,
                "Roll (70.00, line 17) > maximum lean angle (45.00)"
            )
        );
        let angle_max = format!("PARM, 1, ANGLE_MAX, 3000, 0\n{steep}");
        assert_eq!(
            run(pitch_roll_coupling, &angle_max),
            Outcome::new(
                Status::Fail,
                "Roll (70.00, line 18) > maximum lean angle (30.00)"
            )
        );
    }

    #[test]
    fn thrust_vcc_and_vibration() {
        assert_eq!(
            run(thrust, ""),
            Outcome::new(Status::Unknown, "No CTUN log data")
        );
        let mut body = String::from("ATT, 1, 0, 0\n");
        for i in 0..60 {
            body.push_str(&format!("CTUN, {i}, 800, 10, 20\n"));
        }
        body.push_str("CTUN, 99, 100, 10, 20\n");
        // The segment runs to the first sample below the threshold, which is averaged in too.
        assert_eq!(
            run(thrust, &body),
            Outcome::new(
                Status::Fail,
                "Avg climb rate 20.00 cm/s for throttle avg 788"
            )
        );
        let slow = body.replace(", 800, 10, 20", ", 800, 10, 75");
        assert_eq!(
            run(thrust, &slow),
            Outcome::new(
                Status::Warn,
                "Avg climb rate 74.10  cm/s for throttle avg 788"
            )
        );
        assert_eq!(
            run(vcc, ""),
            Outcome::new(Status::Unknown, "No CURR log data")
        );
        assert_eq!(
            run(vcc, "CURR, 1, 4700\nCURR, 2, 5050\n"),
            Outcome::new(Status::Warn, "VCC min/max diff 0.35v, should be <0.3v")
        );
        assert_eq!(
            run(vcc, "CURR, 1, 4512\nCURR, 2, 4600\n"),
            Outcome::new(Status::Fail, "VCC below minimum of 4.6v (4.512v)")
        );
        assert_eq!(run(vcc, "CURR, 1, 5000\n"), Outcome::good());
        let no_vcc = reformatted(
            "FMT, 7, 7, CURR, Qf, TimeUS,Vcc\n",
            "FMT, 7, 7, CURR, Qf, TimeUS,Volt\n",
            "CURR, 1, 5\n",
        );
        assert_eq!(vcc(&no_vcc).unwrap_err().text, "'POWR'");
        assert_eq!(
            run(vibration, ""),
            Outcome::new(Status::Unknown, "No IMU log data")
        );
        // The converter names modes, so "LOITER" in capitals is never seen in a converted log.
        assert_eq!(
            run(
                vibration,
                "GPS, 1, 9, 1.2\nMODE, 2, Loiter, 5, 1\nIMU, 3, 0, 0, -9.8\n"
            ),
            Outcome::new(Status::Unknown, "No stable LOITER log data found")
        );
        // A LOITER stretch timed by the GPS lines at or after its ends: 498 seconds.
        let loiter = "MODE, 2, LOITER, 5, 1\nGPS, 2000, 9, 1.2\nIMU, 3, 0, 1, -9.8\nIMU, 4, 0, -1, -9.8\nGPS, 500000, 9, 1.2\n";
        assert_eq!(
            run(vibration, loiter),
            Outcome::new(
                Status::Warn,
                "Vibration slightly high (X:0.00g, Y:2.00g, Z:0.00g)"
            )
        );
    }

    #[test]
    fn opt_flow_fits_the_line_as_numpy_did() {
        assert_eq!(
            run(opt_flow, ""),
            Outcome::new(Status::Fail, "'FLOW_FXSCALER' not found")
        );
        let scalers = "PARM, 1, FLOW_FXSCALER, 0, 0\nPARM, 1, FLOW_FYSCALER, 0, 0\n";
        assert_eq!(
            run(opt_flow, scalers),
            Outcome::new(Status::Fail, "FAIL: no optical flow data\n")
        );
        let formats = "FMT, 30, 30, OF, QBffffff, TimeUS,Qual,flowX,flowY,bodyX,bodyY,x,y\nFMT, 31, 31, ATT, Qff, TimeUS,Roll,Pitch\n";
        let mut body = format!("{scalers}{formats}");
        // 300 samples: the vehicle rocked in roll then pitch, flow 1.1 times the body rate.
        for i in 0..300 {
            let t = i * 10_000;
            let phase = usize_to_float(i) / 20.0;
            let (roll, pitch) = if i < 150 {
                (30.0 * phase.sin(), 0.0)
            } else {
                (0.0, 30.0 * phase.sin())
            };
            let (bx, by) = (
                1.5 * phase.cos() * if i < 150 { 1.0 } else { 0.1 },
                1.5 * phase.cos() * if i < 150 { 0.1 } else { 1.0 },
            );
            body.push_str(&format!(
                "OF, {t}, 200, {}, {}, {bx}, {by}, 0, 0\nATT, {t}, {roll}, {pitch}\n",
                1.1 * bx,
                1.1 * by
            ));
        }
        let out = run(opt_flow, &body);
        assert_eq!(out.status, Status::Good, "{}", out.message);
        assert_eq!(
            out.message,
            "Set FLOW_FXSCALER to -90\nSet FLOW_FYSCALER to -90\n\nCal plots saved to flow_calibration.pdf\nCal parameters saved to flow_calibration.param\n\nFLOW_FXSCALER 1STD = 0\nFLOW_FYSCALER 1STD = 0\n"
        );
        let x = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let y = [1.0, 3.1, 4.9, 7.2, 8.8, 11.1];
        let ((slope, intercept), var) = polyfit(&x, &y).unwrap();
        assert!(
            (slope - 2.0).abs() < 0.01 && (intercept - 1.0).abs() < 0.05,
            "{slope} {intercept}"
        );
        // numpy 1.11: Vbase[0][0] * resid / (n - 4).
        let resid: f64 = x
            .iter()
            .zip(y.iter())
            .map(|(a, b)| (b - (slope * a + intercept)).powi(2))
            .sum();
        assert!((var - (6.0 / (6.0 * 55.0 - 225.0)) * resid / 2.0).abs() < 1e-12);
        assert_eq!(
            polyfit(&[], &[]).unwrap_err().text,
            "expected non-empty vector for x"
        );
        assert_eq!(
            polyfit(&[1.0, 1.0, 1.0, 1.0, 1.0], &[1.0, 2.0, 3.0, 4.0, 5.0])
                .unwrap_err()
                .text,
            "Singular matrix"
        );
        assert_eq!(
            polyfit(&[1.0, 2.0], &[1.0, 2.0]).unwrap_err().text,
            "operands could not be broadcast together with shapes (2,2) (0,) "
        );
    }

    #[test]
    fn python_ranges_and_slices() {
        assert_eq!(py_range(2, 9, 3), vec![2, 5, 8]);
        assert!(py_range(-1, 0, -1).is_empty());
        assert_eq!(py_range(3, 0, -1), vec![3, 2, 1]);
        assert!(py_range(5, 2, 1).is_empty());
        let list = [1, 2, 3, 4, 5];
        assert_eq!(py_slice(&list, 1, 3), &[2, 3]);
        assert_eq!(py_slice(&list, 3, 20), &[4, 5]);
        assert_eq!(py_slice(&list, -1, 19), &[5]);
        assert!(py_slice(&list, 4, 2).is_empty());
    }
}
