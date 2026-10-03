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

//! Deliverable 14's FFT, held to what Mission Planner's FFT window computes.
//!
//! The C# cannot run here, so the golden is a sample set whose spectrum is known exactly:
//! synthetic sines at known frequencies and amplitudes, and a DC level. `FFT2.rin`'s window has a
//! gain of `4/N`, which is chosen so that a sine of amplitude `A` exactly on bin `k` reads `A` in
//! bin `k` and, the window being a periodic Hann, `A/2` in bins `k-1` and `k+1` and nothing
//! anywhere else; a constant `c` reads `2c` in bin 0 and `c` in bin 1. Those are the numbers
//! asserted, to 1e-6 relative, with the peak required at exactly the expected bin.
//!
//! The transform itself is held to a line-for-line port of `FFT2.run` - the butterflies, the
//! twiddle recurrence and the bit-reversed copy-out - kept here as the oracle, on random data and
//! on the sines, to 1e-12 of the spectrum's peak.
//! `// C#: ExtLibs/Utilities/fft.cs:115-291`

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]

use std::f64::consts::PI;

use mp_log::fft::{self, FieldSeries, Spectrum};

/// `FFT2.run(xRe, xIm)`, forward, transcribed: the C#'s linked list is an array walked in the
/// same order, and everything else is as written.
/// `// C#: ExtLibs/Utilities/fft.cs:168-270`
fn fft2_run(x_re: &mut [f64], x_im: &mut [f64]) {
    let n = x_re.len();
    let log_n = n.trailing_zeros();
    let mut re = x_re.to_vec();
    let mut im = x_im.to_vec();
    let mut num_flies = n >> 1;
    let mut span = n >> 1;
    let mut spacing = n;
    let mut w_index_step = 1usize;
    for _stage in 0..log_n {
        let mut w_angle_inc = w_index_step as f64 * 2.0 * PI / n as f64;
        w_angle_inc *= -1.0;
        let w_mul_re = w_angle_inc.cos();
        let w_mul_im = w_angle_inc.sin();
        let mut start = 0;
        while start < n {
            let mut w_re = 1.0f64;
            let mut w_im = 0.0f64;
            // `xTop`/`xBot` walk the list from `start` and `start + span`.
            for fly in 0..num_flies {
                let (top, bot) = (start + fly, start + span + fly);
                let (top_re, top_im, bot_re, bot_im) = (re[top], im[top], re[bot], im[bot]);
                re[top] = top_re + bot_re;
                im[top] = top_im + bot_im;
                let bot_re = top_re - bot_re;
                let bot_im = top_im - bot_im;
                re[bot] = bot_re * w_re - bot_im * w_im;
                im[bot] = bot_re * w_im + bot_im * w_re;
                let t_re = w_re;
                w_re = w_re * w_mul_re - w_im * w_mul_im;
                w_im = t_re * w_mul_im + w_im * w_mul_re;
            }
            start += spacing;
        }
        num_flies >>= 1;
        span >>= 1;
        spacing >>= 1;
        w_index_step <<= 1;
    }
    for k in 0..n {
        let target = (k as u32).reverse_bits() >> (32 - log_n);
        x_re[target as usize] = re[k];
        x_im[target as usize] = im[k];
    }
}

/// `FFT2.rin` over the transcribed `run`: the oracle for [`fft::rin`].
/// `// C#: ExtLibs/Utilities/fft.cs:115-158`
fn fft2_rin(data: &[f64], output_log: bool) -> Vec<f64> {
    let scale = 20.0 / 10f64.ln();
    let n = data.len();
    let mut x_re = vec![0.0; n];
    let mut x_im = vec![0.0; n];
    for i in 0..n {
        let window = (4.0 / n as f64) * 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
        x_re[i] = window * data[i];
    }
    fft2_run(&mut x_re, &mut x_im);
    (0..n / 2)
        .map(|i| {
            let magnitude = (x_re[i] * x_re[i] + x_im[i] * x_im[i]).sqrt();
            if output_log {
                scale * (magnitude + f64::from_bits(1)).ln()
            } else {
                magnitude
            }
        })
        .collect()
}

/// A sum of sines, `(bin, amplitude, phase)`, sampled `n` times: bin `k` is `k` cycles per `n`
/// samples, so exactly on bin `k` whatever the sample rate.
fn sines(n: usize, parts: &[(usize, f64, f64)]) -> Vec<f64> {
    (0..n)
        .map(|i| {
            parts
                .iter()
                .map(|(bin, amplitude, phase)| {
                    amplitude * (2.0 * PI * *bin as f64 * i as f64 / n as f64 + phase).sin()
                })
                .sum()
        })
        .collect()
}

fn relative(actual: f64, expected: f64) -> f64 {
    ((actual - expected) / expected).abs()
}

/// A deterministic pseudo-random sequence (xorshift), so the test needs no dependency.
fn noise(n: usize, seed: u64) -> Vec<f64> {
    let mut state = seed;
    (0..n)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        })
        .collect()
}

/// The largest bin: its index and value.
fn peak(bins: &[f64]) -> (usize, f64) {
    bins.iter()
        .copied()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap()
}

#[test]
fn one_sine_reads_its_amplitude_in_its_bin_and_half_either_side() {
    for (n, bin, amplitude, phase) in [
        (1024, 100, 1.0, 0.0),
        (1024, 37, 9.81, 0.7),
        (256, 5, 0.003, 2.0),
        (4096, 1500, 250.0, -1.1),
    ] {
        let bins = fft::rin(&sines(n, &[(bin, amplitude, phase)]), false).unwrap();
        assert_eq!(bins.len(), n / 2);
        let (at, value) = peak(&bins);
        assert_eq!(at, bin, "N={n}: the peak is on the sine's bin");
        assert!(
            relative(value, amplitude) < 1e-6,
            "N={n} bin {bin}: {value} for amplitude {amplitude}"
        );
        for side in [bin - 1, bin + 1] {
            assert!(
                relative(bins[side], amplitude / 2.0) < 1e-6,
                "N={n}: bin {side} is {} for half of {amplitude}",
                bins[side]
            );
        }
        for (index, other) in bins.iter().enumerate() {
            if index + 1 < bin || index > bin + 1 {
                assert!(
                    *other < amplitude * 1e-9,
                    "N={n}: bin {index} leaks {other}"
                );
            }
        }
    }
}

#[test]
fn three_sines_read_their_three_amplitudes() {
    let n = 1024;
    let parts = [(40, 3.0, 0.1), (200, 0.5, 1.3), (400, 12.0, 2.9)];
    let bins = fft::rin(&sines(n, &parts), false).unwrap();
    for (bin, amplitude, _) in parts {
        assert!(
            relative(bins[bin], amplitude) < 1e-6,
            "bin {bin}: {} for {amplitude}",
            bins[bin]
        );
    }
}

#[test]
fn a_constant_reads_twice_itself_at_zero_hertz() {
    let bins = fft::rin(&vec![3.5; 512], false).unwrap();
    assert!(relative(bins[0], 7.0) < 1e-12, "{}", bins[0]);
    assert!(relative(bins[1], 3.5) < 1e-12, "{}", bins[1]);
    assert!(bins[2..].iter().all(|bin| *bin < 1e-9));
}

#[test]
fn decibels_are_twenty_log_ten_of_the_amplitude() {
    let bins = fft::rin(&sines(1024, &[(64, 10.0, 0.0)]), true).unwrap();
    assert!((bins[64] - 20.0).abs() < 1e-6, "{}", bins[64]);
    assert!(
        (bins[63] - 20.0 * 5f64.log10()).abs() < 1e-6,
        "{}",
        bins[63]
    );
}

#[test]
fn the_transform_matches_fft2_run_transcribed() {
    for (n, data) in [
        (8, noise(8, 1)),
        (1024, noise(1024, 7)),
        (4096, noise(4096, 99)),
        (1024, sines(1024, &[(17, 2.0, 0.3), (300, 0.25, 1.0)])),
    ] {
        for in_db in [false, true] {
            let ours = fft::rin(&data, in_db).unwrap();
            let theirs = fft2_rin(&data, in_db);
            assert_eq!(ours.len(), theirs.len());
            let top = theirs.iter().fold(0.0f64, |top, bin| top.max(bin.abs()));
            for (index, (a, b)) in ours.iter().zip(&theirs).enumerate() {
                if in_db {
                    // Decibels of a near-empty bin magnify rounding; compare where the bin holds
                    // more than a millionth of the peak.
                    let magnitude = 10f64.powf(b / 20.0);
                    if magnitude > 1e-6 * 10f64.powf(top / 20.0) {
                        assert!((a - b).abs() < 1e-6, "N={n} dB bin {index}: {a} vs {b}");
                    }
                } else {
                    assert!(
                        (a - b).abs() <= 1e-12 * top,
                        "N={n} bin {index}: {a} vs {b}"
                    );
                }
            }
        }
    }
}

#[test]
fn the_average_skips_the_last_slice_and_divides_by_all_of_them() {
    let n = 256;
    // Four whole slices and a remainder: three are averaged, each divided by four.
    let one = sines(n, &[(32, 8.0, 0.0)]);
    let mut data: Vec<f64> = one.iter().copied().cycle().take(4 * n + 17).collect();
    // The last whole slice is different, and must not show.
    for value in &mut data[3 * n..4 * n] {
        *value *= 100.0;
    }
    let spectrum: Spectrum = fft::average(&data, n, 1000.0, 5.0, false).unwrap();
    assert_eq!(spectrum.slices, 4);
    assert!(relative(spectrum.amplitudes[32], 8.0 * 3.0 / 4.0) < 1e-6);
    // 1000 Hz over 256 bins: bin 1 is 3 Hz and bin 2 is 7 Hz, so bins 0 and 1 are below 5 Hz.
    assert_eq!(spectrum.freqs[1], 3.0);
    assert_eq!(spectrum.freqs[2], 7.0);
    assert_eq!(spectrum.amplitudes[0], 0.0);
    assert_eq!(spectrum.amplitudes[1], 0.0);
    // A series of N samples or fewer is skipped.
    assert!(fft::average(&data[..n], n, 1000.0, 5.0, false).is_none());
}

#[test]
fn a_field_series_through_the_window_finds_its_sine() {
    // 1 kHz for ten seconds, a 50 Hz sine of amplitude 2 plus a 1 g offset: an accelerometer
    // axis on a vibrating frame.
    let mut series = FieldSeries::default();
    for i in 0..10_000u32 {
        let time_us = 5_000_000.0 + f64::from(i) * 1000.0;
        let t = f64::from(i) / 1000.0;
        series.push(time_us, 9.81 + 2.0 * (2.0 * PI * 50.0 * t).sin());
    }
    // A sample back in time is refused.
    series.push(1.0, 1e9);
    assert_eq!(series.values.len(), 10_000);
    assert_eq!(series.clock.sample_rate(), 1000.0);

    let spectrum = fft::field_spectrum(&series, 10, 5.0, false).unwrap();
    assert_eq!(spectrum.n, 1024);
    assert_eq!(spectrum.slices, 9);
    // 50 Hz at 1 kHz over 1024 points sits between bins 51 and 52 (49.8 and 50.8 Hz, labelled
    // 49 and 50 as `FreqTable` truncates); the peak is the nearer, 51.
    let peaks = fft::peaks(&spectrum, 5.0, 3);
    assert_eq!(peaks[0].0, 49.0, "{peaks:?}");
    assert_eq!(spectrum.freqs[51], 49.0);
    // The DC is left out: bin 0 is below the start frequency.
    assert_eq!(spectrum.amplitudes[0], 0.0);
}
