//! `headless-planner log fft <log> <field> [size] [--mag] [--start <hz>] [--peaks <count>]`: one field's
//! spectrum as Mission Planner's FFT window (`Controls/fftui.cs`) computes it, without the
//! window.
//!
//! The field is named as `headless-planner fields` lists it - `ACC1.AccX`, or `ACC[0].AccX` for an instanced
//! message. `size` is the FFT size, a power of two, 1,024 by default; the window's Bins box takes
//! its power of two instead (10). `--mag` is the window's Magnitude box (amplitudes rather than
//! decibels) and `--start` its Start Freq box (5 Hz). The window draws the averaged spectrum as a
//! graph and says what a point is only as its tooltip, `"{0} hz/{1} rpm"`; this prints the
//! graph's title and the highest peaks, each as that tooltip reads, with its amplitude.
//! `// C#: Controls/fftui.cs:357-481, 596-599`

#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::path::Path;
use std::process::ExitCode;

use mp_log::fft;

/// How many peaks are printed unless `--peaks` says otherwise.
const DEFAULT_PEAKS: usize = 10;

/// The usage line.
pub(crate) const USAGE: &str = "usage: headless-planner log fft <log.bin> <MSG.Field|MSG[instance].Field> [size] [--mag] [--start <hz>] [--peaks <count>]";

/// What the command line asked for.
#[derive(Debug, Clone, PartialEq)]
struct Request<'a> {
    file: &'a str,
    message: &'a str,
    instance: Option<i64>,
    field: &'a str,
    bins: u32,
    in_db: bool,
    start_freq: f64,
    peaks: usize,
}

/// `MSG.Field` or `MSG[instance].Field`.
fn parse_selector(selector: &str) -> Option<(&str, Option<i64>, &str)> {
    let (message, field) = selector.rsplit_once('.')?;
    if field.is_empty() || message.is_empty() {
        return None;
    }
    match message.split_once('[') {
        Some((name, rest)) => {
            let instance = rest.strip_suffix(']')?.parse().ok()?;
            (!name.is_empty()).then_some((name, Some(instance), field))
        }
        None => Some((message, None, field)),
    }
}

fn parse(args: &[String]) -> Result<Request<'_>, String> {
    let mut positional = Vec::new();
    let mut in_db = true;
    let mut start_freq = fft::DEFAULT_START_FREQ;
    let mut peaks = DEFAULT_PEAKS;
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        match arg.as_str() {
            "--mag" => in_db = false,
            "--start" => {
                start_freq = rest
                    .next()
                    .and_then(|value| value.parse().ok())
                    .ok_or("--start takes a frequency in hertz")?;
            }
            "--peaks" => {
                peaks = rest
                    .next()
                    .and_then(|value| value.parse().ok())
                    .ok_or("--peaks takes a count")?;
            }
            _ => positional.push(arg.as_str()),
        }
    }
    let (Some(file), Some(selector)) = (positional.first(), positional.get(1)) else {
        return Err(USAGE.to_owned());
    };
    let (message, instance, field) = parse_selector(selector)
        .ok_or_else(|| format!("{selector}: name a field as MSG.Field or MSG[instance].Field"))?;
    let bins = match positional.get(2) {
        Some(size) => {
            let size: usize = size
                .parse()
                .map_err(|_| format!("{size}: the size is a number of samples"))?;
            if size < 2 || !size.is_power_of_two() {
                return Err(format!("{size}: the size must be a power of two"));
            }
            size.trailing_zeros()
        }
        None => fft::DEFAULT_BINS,
    };
    if positional.len() > 3 {
        return Err(USAGE.to_owned());
    }
    Ok(Request {
        file,
        message,
        instance,
        field,
        bins,
        in_db,
        start_freq,
        peaks,
    })
}

/// Runs `headless-planner log fft` on the arguments after `fft`.
#[must_use]
pub(crate) fn run(args: &[String]) -> ExitCode {
    let request = match parse(args) {
        Ok(request) => request,
        Err(message) => {
            eprintln!("{message}");
            return ExitCode::from(2);
        }
    };
    match spectrum(&request) {
        Ok(text) => {
            print!("{text}");
            ExitCode::SUCCESS
        }
        Err(message) => {
            eprintln!("{message}");
            ExitCode::FAILURE
        }
    }
}

/// The report: the graph's title, then the peaks.
fn spectrum(request: &Request<'_>) -> Result<String, String> {
    let path = Path::new(request.file);
    let log =
        mp_log::logfile::LogFile::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let series = fft::read_series(&log, request.message, request.instance, request.field);
    let n = 1usize << request.bins;
    let named = match request.instance {
        Some(instance) => format!("{}[{instance}].{}", request.message, request.field),
        None => format!("{}.{}", request.message, request.field),
    };
    let spectrum = fft::field_spectrum(&series, request.bins, request.start_freq, request.in_db)
        .ok_or_else(|| {
            format!(
                "{named}: {} samples, and a {n}-point FFT needs more than {n}",
                series.values.len()
            )
        })?;
    let file_name = path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut out = format!(
        "{}\n{named}: {n}-point FFT, {} slices of which {} averaged, {} from {} Hz\n",
        fft::title(request.message, &file_name, spectrum.sample_rate),
        spectrum.slices,
        spectrum.slices.saturating_sub(1),
        if request.in_db {
            "amplitude in dB"
        } else {
            "amplitude"
        },
        mp_log::netfmt::double(request.start_freq),
    );
    for (hz, amplitude) in fft::peaks(&spectrum, request.start_freq, request.peaks) {
        out.push_str(&format!(
            "  {:<24} {}\n",
            fft::point_value(hz),
            mp_log::netfmt::double(amplitude)
        ));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|arg| (*arg).to_owned()).collect()
    }

    #[test]
    fn a_field_is_named_with_or_without_an_instance() {
        assert_eq!(parse_selector("ACC1.AccX"), Some(("ACC1", None, "AccX")));
        assert_eq!(
            parse_selector("ACC[2].AccZ"),
            Some(("ACC", Some(2), "AccZ"))
        );
        assert_eq!(parse_selector("ACC[x].AccZ"), None);
        assert_eq!(parse_selector("AccX"), None);
        assert_eq!(parse_selector(".AccX"), None);
    }

    #[test]
    fn the_size_is_a_power_of_two_and_defaults_to_the_windows_bins() {
        let defaults = args(&["a.bin", "IMU.AccX"]);
        let request = parse(&defaults).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(request.bins, 10);
        assert!(request.in_db);
        assert!((request.start_freq - 5.0).abs() < f64::EPSILON);

        let sized = args(&["a.bin", "IMU.AccX", "256", "--mag", "--start", "20"]);
        let request = parse(&sized).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(request.bins, 8);
        assert!(!request.in_db);
        assert!((request.start_freq - 20.0).abs() < f64::EPSILON);

        assert!(parse(&args(&["a.bin", "IMU.AccX", "1000"])).is_err());
        assert!(parse(&args(&["a.bin"])).is_err());
    }
}
