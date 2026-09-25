//! `MagCalib.cs`'s fit held to PLAN.md §7.2's class D.
//!
//! The C# cannot be run here (PLAN.md §7.1: `MagCalib.cs` does not build under mono), so the
//! golden is the sample sets `headless-planner magcal` extracts from the logs under `testdata` - committed as
//! `testdata/magcal/*.txt` by `mp-cli`'s ignored `regenerate_magcal_fixtures` - fitted by the port
//! and held to what any correct run of the C#'s fit must satisfy:
//!
//! - **residual**: the port's sum of squares is no more than 1.001 × that of an independent sphere
//!   fit (Levenberg-Marquardt on the analytic Jacobian, from the C#'s own starting point), which
//!   stands in for `residual_csharp`;
//! - **offsets** within 1% of the fitted radius of that fit's;
//! - **convergence**: both stop because nothing more can be gained, not on a failure;
//! - **invariants** of a least-squares sphere, true at any minimum: the radius is the samples'
//!   mean distance from the centre, and the gradient in the offsets is nil;
//! - each ellipsoid stage ends no worse than it starts, its diagonals of length √3, the offsets
//!   the sphere's.
//!
//! And synthetic sets with known offsets, and with a known scale, are recovered to 1%.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::needless_range_loop
)]

use std::path::Path;

use mp_calibration::CalibrationError;
use mp_calibration::magcalib::{
    Sample, TERMINATION_MAXITS, TERMINATION_NAN, TERMINATION_STEP, TERMINATION_STRINGENT,
    calc_radius, fit_dataflash, fit_tlog, least_sq, list_capacity, remove_outliers, sum_of_squares,
};
use nalgebra::{DMatrix, DVector};

/// A fixture: its path (`tlog` or `dataflash`) and its samples.
fn fixture(name: &str) -> (String, Vec<Sample>) {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/magcal")
        .join(format!("{name}.txt"));
    let text = std::fs::read_to_string(&path).unwrap();
    let mut kind = String::new();
    let mut samples = Vec::new();
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("# path: ") {
            kind = rest.to_owned();
        } else if !line.starts_with('#') {
            let v: Vec<f32> = line.split(' ').map(|s| s.parse().unwrap()).collect();
            samples.push([v[0], v[1], v[2]]);
        }
    }
    (kind, samples)
}

/// Sphere residuals and the analytic Jacobian at `x`.
fn sphere(data: &[Sample], x: &DVector<f64>) -> (DVector<f64>, DMatrix<f64>) {
    let mut f = DVector::zeros(data.len());
    let mut j = DMatrix::zeros(data.len(), 4);
    for (i, s) in data.iter().enumerate() {
        let d: Vec<f64> = (0..3).map(|k| f64::from(s[k]) + x[k]).collect();
        let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        f[i] = x[3] - len;
        for k in 0..3 {
            j[(i, k)] = -d[k] / len;
        }
        j[(i, 3)] = 1.0;
    }
    (f, j)
}

/// The reference: Marquardt's damping on the exact Jacobian, from where the C# starts, until a
/// step gains nothing. Returns the answer, its sum of squares and whether it converged.
fn reference_sphere(data: &[Sample]) -> (Vec<f64>, f64, bool) {
    let mut x = DVector::from_vec(vec![0.0, 0.0, 0.0, calc_radius(data)]);
    let (mut f, mut j) = sphere(data, &x);
    let mut cost = f.norm_squared();
    let mut lambda = 1e-3;
    for _ in 0..10_000 {
        let a = j.transpose() * &j;
        let g = j.transpose() * &f;
        let mut damped = a.clone();
        for k in 0..4 {
            damped[(k, k)] += lambda * a[(k, k)].max(1e-300);
        }
        let Some(chol) = damped.cholesky() else {
            lambda *= 10.0;
            continue;
        };
        let step = chol.solve(&(-g));
        let trial = &x + step;
        let (tf, tj) = sphere(data, &trial);
        let trial_cost = tf.norm_squared();
        if trial_cost < cost {
            let gain = cost - trial_cost;
            x = trial;
            f = tf;
            j = tj;
            cost = trial_cost;
            lambda = (lambda / 10.0).max(1e-15);
            if gain <= 1e-15 * cost {
                return (x.as_slice().to_vec(), cost, true);
            }
        } else {
            lambda *= 10.0;
            if lambda > 1e20 {
                return (x.as_slice().to_vec(), cost, true);
            }
        }
    }
    (x.as_slice().to_vec(), cost, false)
}

/// The class D gate and the sphere's invariants, for one sample set; prints what it found.
fn class_d(name: &str, data: &[Sample]) -> (Vec<f64>, f64) {
    let fit = least_sq(data, false).unwrap();
    let stage = &fit.stages[0];
    let (reference, reference_residual, converged) = reference_sphere(data);
    let radius = reference[3].abs();
    println!(
        "{name}: {} samples, offsets {:.3} {:.3} {:.3}, radius {:.3}, residual {:.6} \
         (reference {:.3} {:.3} {:.3} r {:.3} residual {:.6}), termination {} ({}), {} steps",
        data.len(),
        fit.x[0],
        fit.x[1],
        fit.x[2],
        fit.x[3],
        stage.residual,
        reference[0],
        reference[1],
        reference[2],
        reference[3],
        reference_residual,
        stage.termination,
        stage.reason,
        stage.iterations,
    );
    assert!(converged, "{name}: the reference did not converge");
    assert!(
        stage.residual <= reference_residual * 1.001,
        "{name}: residual {} against {}",
        stage.residual,
        reference_residual
    );
    for k in 0..3 {
        assert!(
            (fit.x[k] - reference[k]).abs() <= 0.01 * radius,
            "{name}: offset {k} {} against {}",
            fit.x[k],
            reference[k]
        );
    }
    assert!(
        [TERMINATION_STEP, TERMINATION_STRINGENT].contains(&stage.termination),
        "{name}: {}",
        stage.reason
    );
    // At a minimum the radius is the mean distance, and the offsets' gradient is nil.
    let x = DVector::from_vec(fit.x.clone());
    let (f, j) = sphere(data, &x);
    let mean: f64 = f.iter().sum::<f64>() / data.len() as f64;
    assert!(mean.abs() <= 1e-6 * radius, "{name}: mean residual {mean}");
    let g = j.transpose() * &f;
    let scale = j.norm() * f.norm().max(1e-12);
    assert!(g.norm() <= 1e-6 * scale, "{name}: gradient {}", g.norm());
    (fit.x, stage.residual)
}

/// Every ellipsoid stage ends no worse than it starts, the next starts where it ended, and the
/// answer has diagonals of length √3 and the sphere's offsets.
fn ellipsoid_invariants(name: &str, data: &[Sample]) -> Vec<f64> {
    let fit = least_sq(data, true).unwrap();
    let sphere = least_sq(data, false).unwrap();
    assert_eq!(fit.stages.len(), 3);
    for stage in &fit.stages {
        assert!(stage.residual <= stage.start_residual, "{name}: {stage:?}");
        assert_ne!(
            stage.termination, TERMINATION_NAN,
            "{name}: {}",
            stage.reason
        );
    }
    let six = &fit.stages[1];
    let nine = &fit.stages[2];
    assert!((nine.start_residual - six.residual).abs() <= 1e-9 * six.residual.max(1.0));
    assert_eq!(fit.x[..3], sphere.x[..3], "{name}");
    let diag = (fit.x[3] * fit.x[3] + fit.x[4] * fit.x[4] + fit.x[5] * fit.x[5]).sqrt();
    assert!((diag - 3.0_f64.sqrt()).abs() < 1e-12, "{name}: {diag}");
    println!(
        "{name} ellipsoid: di {:.4} {:.4} {:.4} odi {:.4} {:.4} {:.4}, rad {:.3}, residual {:.3} \
         -> {:.3} -> {:.3}, terminations {} {}",
        fit.x[3],
        fit.x[4],
        fit.x[5],
        fit.x[6],
        fit.x[7],
        fit.x[8],
        fit.rad,
        six.start_residual,
        six.residual,
        nine.residual,
        six.termination,
        nine.termination,
    );
    fit.x
}

#[test]
fn a_still_vehicle_in_a_telemetry_log_is_not_enough_data() {
    for name in ["autotest", "multisystem"] {
        let (kind, data) = fixture(name);
        assert_eq!(kind, "tlog");
        let error = fit_tlog(data.clone()).unwrap_err();
        assert!(
            matches!(error, CalibrationError::NotEnoughData { samples } if samples == data.len()),
            "{name}"
        );
        assert_eq!(error.to_string(), "Log does not contain enough data");
    }
}

#[test]
fn the_dataflash_log_meets_class_d() {
    let (kind, data) = fixture("dataflash");
    assert_eq!(kind, "dataflash");
    assert_eq!(data.len(), 364);
    let (offsets, residual) = class_d("dataflash.bin", &data);
    ellipsoid_invariants("dataflash.bin", &data);
    // `getOffsetsLog`'s answer is the ellipsoid's offsets, which are the sphere's.
    let fit = fit_dataflash(&data).unwrap();
    assert_eq!(fit.offsets, [offsets[0], offsets[1], offsets[2]]);
    assert_eq!(fit.samples, 364);
    // Pinned: what this port found, held to class D's own tolerance on the next run.
    let pinned = PINNED_DATAFLASH;
    for k in 0..3 {
        assert!(
            (offsets[k] - pinned[k]).abs() <= 0.01 * pinned[3],
            "{offsets:?}"
        );
    }
    assert!(residual <= pinned[4] * 1.001, "{residual}");
}

/// The dataflash fixture's sphere: offsets, radius and sum of squares, as found 2026-09-25.
///
/// The log is SITL's, standing still: its two compasses (every `MAG` line is compass 1 to the C#,
/// whatever its instance) are two tight clusters about 645 from the origin, so the sphere is
/// barely determined and the fit walks from the C#'s start to a small sphere through both - the
/// same one the independent reference reaches. Nonsense as a calibration, and what the C# is
/// asked to fit.
const PINNED_DATAFLASH: [f64; 5] = [
    -292.546_834_244_562_43,
    122.079_074_127_069_66,
    552.419_038_671_528,
    250.636_689_665_215_07,
    17_261.940_589,
];

/// Points spread evenly over the unit sphere (a Fibonacci lattice).
fn lattice(count: usize) -> Vec<[f64; 3]> {
    let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
    (0..count)
        .map(|i| {
            let y = 1.0 - 2.0 * (i as f64 + 0.5) / count as f64;
            let r = (1.0 - y * y).sqrt();
            let theta = golden * i as f64;
            [r * theta.cos(), y, r * theta.sin()]
        })
        .collect()
}

/// A repeatable noise in [-1, 1].
fn noise(i: usize, k: usize) -> f64 {
    let v = ((i * 7919 + k * 104_729) % 2003) as f64;
    v / 1001.5 - 1.0
}

#[allow(clippy::cast_possible_truncation)]
fn sample(v: [f64; 3]) -> Sample {
    [v[0] as f32, v[1] as f32, v[2] as f32]
}

#[test]
fn known_offsets_are_recovered_to_one_percent() {
    // A compass whose samples sit 420 from (-150, 60, 210), with a few units of noise, as a
    // vehicle turned through every attitude would give. The offsets are minus the centre.
    let centre = [-150.0, 60.0, 210.0];
    let data: Vec<Sample> = lattice(600)
        .into_iter()
        .enumerate()
        .map(|(i, u)| sample([0, 1, 2].map(|k| centre[k] + 420.0 * u[k] + 3.0 * noise(i, k))))
        .collect();
    let (offsets, _) = class_d("synthetic offsets", &data);
    for k in 0..3 {
        assert!(
            (offsets[k] + centre[k]).abs() <= 0.01 * 420.0,
            "{offsets:?}"
        );
    }
    assert!((offsets[3] - 420.0).abs() <= 0.01 * 420.0);
    // Through the telemetry path: the farthest sixteenth dropped, then the same fit.
    let fit = fit_tlog(data.clone()).unwrap();
    assert_eq!(fit.samples, 600 - 600 / 16);
    for k in 0..3 {
        assert!(
            (fit.offsets[k] + centre[k]).abs() <= 0.01 * 420.0,
            "{:?}",
            fit.offsets
        );
    }
    ellipsoid_invariants("synthetic offsets", &data);
}

#[test]
fn a_known_scale_is_recovered_to_one_percent() {
    // An ellipsoid about the origin: the samples are a sphere divided by the scale, so the
    // matrix that makes them round again is the scale. The C# fits the ellipsoid about the origin
    // whatever the offsets are (`MagCalib.cs:1444`), so this is the case it can recover.
    let scale = [1.08, 0.95, 1.0];
    let data: Vec<Sample> = lattice(600)
        .into_iter()
        .map(|u| sample([0, 1, 2].map(|k| 400.0 * u[k] / scale[k])))
        .collect();
    let x = ellipsoid_invariants("synthetic scale", &data);
    let length = (scale[0] * scale[0] + scale[1] * scale[1] + scale[2] * scale[2]).sqrt();
    for k in 0..3 {
        let expected = scale[k] / length * 3.0_f64.sqrt();
        assert!((x[3 + k] - expected).abs() <= 0.01 * expected, "{x:?}");
    }
    for k in 0..3 {
        assert!(x[6 + k].abs() <= 0.01, "{x:?}");
    }
}

#[test]
fn outliers_go_by_distance_from_the_origin() {
    let (_, mut data) = fixture("dataflash");
    let capacity = list_capacity(data.len());
    remove_outliers(&mut data, capacity);
    assert_eq!(data.len(), 364 - 364 / 16);
    let norm = |s: &Sample| f64::from(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt();
    assert!(data.windows(2).all(|w| norm(&w[0]) <= norm(&w[1])));
}

#[test]
fn the_sum_of_squares_is_alglibs_objective() {
    assert_eq!(sum_of_squares(&[3.0, 4.0]), 25.0);
    // The unused codes are named for what the solver can report.
    assert_ne!(TERMINATION_MAXITS, TERMINATION_STRINGENT);
}
