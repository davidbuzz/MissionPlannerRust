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

//! The FFT window's arithmetic: a log field's spectrum, as `Controls/fftui.cs` draws it.
//!
//! **Which transform.** DELIVERABLES.md D14 names `ExtLibs/Exocortex.DSP`, but nothing in Mission
//! Planner calls it: the only users of an FFT are `fftui` (SETUP's FFT Setup page and the
//! advanced config's FFT button open it) and `Spectrogram` (MainV2's hotkey), and both use
//! `MissionPlanner.Utilities.FFT2` in `ExtLibs/Utilities/fft.cs` - Gerald Beauregard's in-place
//! radix-2 transform. `fft3.cs`, the alglib sliding DFT PLAN.md counts as an alglib call site, has
//! no callers either. `FFT2` is therefore the spec, and `Exocortex.DSP` and `fft3` are dead code.
//!
//! What `FFT2.rin` computes, for `N = 1 << bins` samples:
//! - a periodic Hann window with a gain of `4/N`, `w[i] = (4/N) * 0.5 * (1 - cos(2 pi i / N))`,
//!   so that a sine of amplitude `A` exactly on a bin reads `A` in that bin (the window's mean is
//!   `2/N` and a real sine puts half its energy in each of the two mirrored bins);
//! - the unnormalised forward DFT, `X[k] = sum x[n] e^(-2 pi i k n / N)` - `run` with
//!   `inverse = false` scales by 1 and reorders the bit-reversed result into natural order;
//! - the first `N/2` magnitudes, and with `outputLog` (the window's default, its Magnitude box
//!   unticked) `20 / ln 10 * ln(m + double.Epsilon)`: decibels, `double.Epsilon` being .NET's
//!   smallest subnormal, not machine epsilon.
//!
//! `FreqTable` labels bin `i` with `i * samplerate / N` in **integer** arithmetic, so the axis is
//! whole hertz, truncated. The window averages whole `N`-sample slices of a series and plots
//! amplitude against those frequencies, x titled "Freq Hz" and y "Amplitude".
//!
//! The transform is `rustfft`'s rather than a transcription of `FFT2.run`: the two compute the
//! same DFT, and `tests/fft.rs` holds them together - a line-for-line port of `run` kept in the
//! test as the oracle - to 1e-12 of the spectrum's peak, and holds the scaling to synthetic sines
//! whose amplitudes are known.
//!
//! **Where the GUI calls in.** Mission Planner has no FFT button on the log browser: `fftui` is
//! opened from SETUP > Optional Hardware > FFT Setup (`ConfigFFT.but_fft_Click`) and from the
//! advanced config (`ConfigAdvanced`), and picks its own log. When that window is ported it calls
//! [`field_spectrum`] per series, as `headless-planner log fft` does.
//! `// C#: ExtLibs/Utilities/fft.cs:63-158; Controls/fftui.cs:357-600, 976`

use std::f64::consts::PI;

use rustfft::FftPlanner;
use rustfft::num_complex::Complex;

/// The window's Bins box: `NUM_bins.Value`, the FFT size's power of two. 1,024 samples.
/// `// C#: Controls/fftui.Designer.cs:88-92`
pub const DEFAULT_BINS: u32 = 10;

/// The window's Start Freq box: `NUM_startfreq.Value`, below which a bin is left out of the
/// average. 5 Hz.
/// `// C#: Controls/fftui.Designer.cs:121-125`
pub const DEFAULT_START_FREQ: f64 = 5.0;

/// `20 / Math.Log(10)`: natural log to decibels.
/// `// C#: ExtLibs/Utilities/fft.cs:117`
const SCALE: f64 = 20.0 / std::f64::consts::LN_10;

/// .NET's `double.Epsilon`: the smallest positive subnormal, 4.94e-324. Added before the log so
/// an empty bin is a very large negative number rather than minus infinity.
/// `// C#: ExtLibs/Utilities/fft.cs:148`
const DOUBLE_EPSILON: f64 = f64::from_bits(1);

/// The Hann analysis window `rin` builds, gain included: `(4.0/N)*0.5*(1 - Math.Cos(2*Math.PI*i/N))`.
/// `// C#: ExtLibs/Utilities/fft.cs:127-130`
#[must_use]
pub fn hanning(n: usize) -> Vec<f64> {
    #[allow(clippy::cast_precision_loss)] // an FFT size is far below 2^53
    let size = n as f64;
    (0..n)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            let i = i as f64;
            (4.0 / size) * 0.5 * (1.0 - (2.0 * PI * i / size).cos())
        })
        .collect()
}

/// `FFT2.rin`: the windowed spectrum of `data`, `data.len() / 2` bins, in decibels when `in_db`
/// (the window's default) and as magnitudes when not.
///
/// `data.len()` is the FFT size and must be a power of two, as `init(bins)` makes it in the C#,
/// whose every caller passes exactly `1 << bins` samples; any other length is refused rather
/// than transformed at a size the C# never uses.
/// `// C#: ExtLibs/Utilities/fft.cs:115-158`
#[must_use]
pub fn rin(data: &[f64], in_db: bool) -> Option<Vec<f64>> {
    let n = data.len();
    if n < 2 || !n.is_power_of_two() {
        return None;
    }
    // Hanning analysis window, and the real input windowed.
    let mut buffer: Vec<Complex<f64>> = hanning(n)
        .iter()
        .zip(data)
        .map(|(w, x)| Complex::new(w * x, 0.0))
        .collect();
    // `run(xRe, xIm)`: forward, unscaled, in natural order.
    FftPlanner::<f64>::new()
        .plan_fft_forward(n)
        .process(&mut buffer);
    Some(
        buffer
            .iter()
            .take(n / 2)
            .map(|bin| {
                // `Math.Sqrt(re*re + im*im)`, rounded as the C# rounds it: no fused multiply-add.
                let magnitude = (bin.re * bin.re + bin.im * bin.im).sqrt();
                if in_db {
                    SCALE * (magnitude + DOUBLE_EPSILON).ln()
                } else {
                    magnitude
                }
            })
            .collect(),
    )
}

/// `FFT2.FreqTable`: the frequency of each of the `N/2` bins, `i*samplerate/N` in C#'s `int`
/// arithmetic - truncated to whole hertz, and wrapping where `i*samplerate` passes `int.MaxValue`
/// as unchecked C# does.
/// `// C#: ExtLibs/Utilities/fft.cs:94-107`
#[must_use]
pub fn freq_table(sample_count: i32, sample_rate: i32) -> Vec<f64> {
    if sample_count <= 0 {
        return Vec::new();
    }
    (0..sample_count / 2)
        .map(|i| f64::from(i.wrapping_mul(sample_rate) / sample_count))
        .collect()
}

/// `Math.Round(value, 1)`: to one decimal place, a midpoint to the even neighbour - .NET's
/// default `MidpointRounding.ToEven` - computed as .NET computes it, scaled by ten and back.
/// `// C#: Controls/fftui.cs:456`
#[must_use]
pub fn round_1(value: f64) -> f64 {
    (value * 10.0).round_ties_even() / 10.0
}

/// `FFT2.datastate`'s clock: the sample interval as the window estimates it, an exponential
/// average of the gaps between timestamps, in milliseconds.
///
/// It starts at zero with `lasttime` at zero, so the first gap it averages is the first sample's
/// whole time since boot; a series long enough to transform has decayed that away. A sample
/// earlier than the last is refused, and a repeat of the last time leaves the average alone.
/// `// C#: Controls/fftui.cs:411-420`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SampleClock {
    /// `timedelta`: the averaged gap, milliseconds.
    pub timedelta: f64,
    /// `lasttime`: the last accepted time, milliseconds.
    pub lasttime: f64,
}

impl SampleClock {
    /// Offers a sample's time, milliseconds (`TimeUS / 1000.0`); whether the sample is kept.
    pub fn accept(&mut self, time_ms: f64) -> bool {
        if time_ms < self.lasttime {
            return false;
        }
        #[allow(clippy::float_cmp)] // the C#'s `time != lasttime`, exactly
        if time_ms != self.lasttime {
            // Unfused, as the C# rounds it.
            self.timedelta = self.timedelta * 0.99 + (time_ms - self.lasttime) * 0.01;
        }
        self.lasttime = time_ms;
        true
    }

    /// `Math.Round(1000 / timedelta, 1)`: the sample rate in hertz the window titles its plot with.
    #[must_use]
    pub fn sample_rate(&self) -> f64 {
        round_1(1000.0 / self.timedelta)
    }
}

/// One series' averaged spectrum, as one of `fftui`'s graphs plots it.
#[derive(Debug, Clone, PartialEq)]
pub struct Spectrum {
    /// The sample rate the clock estimated, hertz, one decimal place.
    pub sample_rate: f64,
    /// The FFT size.
    pub n: usize,
    /// How many whole `N`-sample slices the series held, `totalsamples / N`: every one but the
    /// last is averaged, each weighted `1 / slices`.
    pub slices: usize,
    /// Each bin's frequency, from [`freq_table`]: the x of each point.
    pub freqs: Vec<f64>,
    /// Each bin's averaged amplitude, zero below the start frequency: the y of each point.
    pub amplitudes: Vec<f64>,
}

/// The average the "Run all imus" buttons plot for one series: `N`-sample slices from the
/// start, the last whole slice skipped (`while (count > 1) // skip last part`), each slice's
/// spectrum added in divided by the whole number of slices, and bins below `start_freq` left at
/// zero. `None` for a series of `N` samples or fewer, which the C# skips.
///
/// `done + count` in the C#'s divisor is the same number every pass - the slice count - so with
/// three slices the average is two spectra, each divided by three.
/// `// C#: Controls/fftui.cs:449-481`
#[must_use]
pub fn average(
    data: &[f64],
    n: usize,
    sample_rate: f64,
    start_freq: f64,
    in_db: bool,
) -> Option<Spectrum> {
    if n < 2 || !n.is_power_of_two() || data.len() <= n {
        return None;
    }
    // `(int)samplerate`: towards zero; `fft.FreqTable(N, ...)`.
    #[allow(clippy::cast_possible_truncation)]
    let rate = sample_rate as i32;
    let freqs = freq_table(i32::try_from(n).ok()?, rate);
    let slices = data.len() / n;
    #[allow(clippy::cast_precision_loss)] // a count of slices
    let divisor = slices as f64;
    let mut amplitudes = vec![0.0; n / 2];
    for slice in data.chunks_exact(n).take(slices - 1) {
        let spectrum = rin(slice, in_db)?;
        for ((sum, bin), freq) in amplitudes.iter_mut().zip(&spectrum).zip(&freqs) {
            if *freq < start_freq {
                continue;
            }
            *sum += bin / divisor;
        }
    }
    Some(Spectrum {
        sample_rate,
        n,
        slices,
        freqs,
        amplitudes,
    })
}

/// A series read from a log for the window: each sample's value, the clock over its times.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FieldSeries {
    /// The values, in log order, of the samples the clock accepted.
    pub values: Vec<f64>,
    /// The clock, after every sample.
    pub clock: SampleClock,
}

impl FieldSeries {
    /// Adds a sample: its `TimeUS` and value, kept if its time is not earlier than the last.
    /// `// C#: Controls/fftui.cs:409-424`
    pub fn push(&mut self, time_us: f64, value: f64) {
        if self.clock.accept(time_us / 1000.0) {
            self.values.push(value);
        }
    }
}

/// One series' samples from a log, gathered as the "Run all imus" buttons gather a sensor's:
/// every record of `message` - of `instance`, where one is given, by the field the log's `FMTU`
/// marks as the instance - in log order, its `TimeUS` offered to the clock and `field` kept
/// where the clock accepts it. A record without a numeric `TimeUS` or `field` is passed over,
/// where the C#'s `Convert.ToDouble` would throw.
/// `// C#: Controls/fftui.cs:394-424`
#[must_use]
pub fn read_series(
    log: &crate::logfile::LogFile,
    message: &str,
    instance: Option<i64>,
    field: &str,
) -> FieldSeries {
    let instance_label = instance
        .is_some()
        .then(|| log.instance_fields().get(message).cloned())
        .flatten();
    let mut series = FieldSeries::default();
    for (_, record) in log.messages(&[message]) {
        if let Some(label) = instance_label.as_deref() {
            #[allow(clippy::cast_possible_truncation)] // instance numbers are small
            let found = record
                .field(label)
                .and_then(crate::Value::as_f64)
                .map(|value| value as i64);
            if found != instance {
                continue;
            }
        }
        let (Some(time_us), Some(value)) = (
            record.field("TimeUS").and_then(crate::Value::as_f64),
            record.field(field).and_then(crate::Value::as_f64),
        ) else {
            continue;
        };
        series.push(time_us, value);
    }
    series
}

/// One series of a log, through the window: the samples gathered as `BUT_accgyrall_Click`
/// gathers them, the rate the clock estimates, and the averaged spectrum at `bins`
/// (`N = 1 << bins`).
/// `// C#: Controls/fftui.cs:357-481`
#[must_use]
pub fn field_spectrum(
    series: &FieldSeries,
    bins: u32,
    start_freq: f64,
    in_db: bool,
) -> Option<Spectrum> {
    let n = 1usize.checked_shl(bins)?;
    average(
        &series.values,
        n,
        series.clock.sample_rate(),
        start_freq,
        in_db,
    )
}

/// The graph's title: `"FFT " + type + " - " + file name + " - " + samplerate + "hz input"`.
/// `// C#: Controls/fftui.cs:497-499`
#[must_use]
pub fn title(sensor: &str, file_name: &str, sample_rate: f64) -> String {
    format!(
        "FFT {sensor} - {file_name} - {}hz input",
        crate::netfmt::double(sample_rate)
    )
}

/// What the graph shows for a point under the pointer: `"{0} hz/{1} rpm"`, the frequency and it
/// times sixty.
/// `// C#: Controls/fftui.cs:596-599`
#[must_use]
pub fn point_value(hz: f64) -> String {
    format!(
        "{} hz/{} rpm",
        crate::netfmt::double(hz),
        crate::netfmt::double(hz * 60.0)
    )
}

/// The spectrum's peaks: bins at or above the start frequency higher than both neighbours
/// (an end bin than its one neighbour), highest first, at most `count`. What a reader of
/// `fftui`'s graph looks for, one tooltip at a time.
#[must_use]
pub fn peaks(spectrum: &Spectrum, start_freq: f64, count: usize) -> Vec<(f64, f64)> {
    let values = &spectrum.amplitudes;
    let mut found: Vec<(f64, f64)> = Vec::new();
    for (index, (freq, value)) in spectrum.freqs.iter().zip(values).enumerate() {
        if *freq < start_freq {
            continue;
        }
        let left = index
            .checked_sub(1)
            .and_then(|left| values.get(left))
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        let right = values.get(index + 1).copied().unwrap_or(f64::NEG_INFINITY);
        if *value > left && *value > right {
            found.push((*freq, *value));
        }
    }
    found.sort_by(|a, b| b.1.total_cmp(&a.1));
    found.truncate(count);
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whole-hertz bins: `i*samplerate/N` in `int`.
    #[test]
    fn the_frequency_table_is_whole_hertz_truncated() {
        let freqs = freq_table(1024, 1000);
        assert_eq!(freqs.len(), 512);
        assert_eq!(freqs.first(), Some(&0.0));
        // 3 * 1000 / 1024 = 2.93, truncated.
        assert_eq!(freqs.get(3), Some(&2.0));
        assert_eq!(freqs.get(511), Some(&499.0));
    }

    /// `Math.Round(x, 1)` rounds a midpoint to even, where Rust's `round` goes away from zero.
    #[test]
    fn rounding_is_to_even_as_dotnet_rounds() {
        assert!((round_1(0.25) - 0.2).abs() < 1e-12);
        assert!((round_1(0.35) - 0.4).abs() < 1e-12);
        assert!((round_1(999.96) - 1000.0).abs() < 1e-9);
    }

    /// The clock's first gap is the first time itself, and a sample back in time is refused.
    #[test]
    fn the_clock_averages_gaps_from_zero_and_refuses_going_back() {
        let mut clock = SampleClock::default();
        assert!(clock.accept(100.0));
        assert!((clock.timedelta - 1.0).abs() < 1e-12);
        assert!(!clock.accept(50.0));
        assert!(clock.accept(100.0));
        assert!(
            (clock.timedelta - 1.0).abs() < 1e-12,
            "a repeat changes nothing"
        );
        assert!(clock.accept(101.0));
        assert!((clock.timedelta - 1.0).abs() < 1e-12, "0.99 * 1 + 1 * 0.01");
    }

    /// A size the C# never uses is refused.
    #[test]
    fn only_a_power_of_two_is_transformed() {
        assert!(rin(&[0.0; 100], true).is_none());
        assert!(rin(&[], true).is_none());
        assert_eq!(rin(&[0.0; 8], false).map(|bins| bins.len()), Some(4));
    }

    /// An empty bin in decibels is the log of `double.Epsilon`, not minus infinity.
    #[test]
    fn an_empty_bin_in_decibels_is_finite() {
        let bins = rin(&[0.0; 16], true).unwrap_or_default();
        assert!(bins.iter().all(|bin| bin.is_finite() && *bin < -6000.0));
    }

    /// The point value's text.
    #[test]
    fn the_point_value_reads_hertz_and_rpm() {
        assert_eq!(point_value(12.0), "12 hz/720 rpm");
        assert_eq!(
            title("ACC1", "a.bin", 400.0),
            "FFT ACC1 - a.bin - 400hz input"
        );
    }
}
