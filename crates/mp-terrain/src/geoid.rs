//! The EGM96 geoid undulation: `ExtLibs/GeoidHeightsDotNet/GeoidHeights/GeoidHeights.cs`, the
//! 360-degree spherical harmonic model over its `Coef.cs` - `cc`, `cs`, `hc` and `hs`, 65,342
//! doubles each, carried as `assets/geoid/egm96.bin` (little-endian `f64`, the four arrays in
//! that order, written from the C# file) - as the NMEA output's GGA sentence reads it
//! (`Controls/SerialOutputNMEA.cs:143`). A transliteration: the same sums in the same order, the
//! C#'s 1-based arrays kept with an unused element 0.
//! `// C#: ExtLibs/GeoidHeightsDotNet/GeoidHeights/GeoidHeights.cs:1-188; Coef.cs`

// A transliteration: the C#'s indexed arrays, each sized above its highest index here (`p` to
// `l_value`, the Legendre and trigonometric tables to 361, the roots to 721), so the indices are
// the algorithm's, as in the C#.
#![allow(clippy::indexing_slicing)]

use std::sync::OnceLock;

/// `l_value`: the harmonic terms up to degree and order 360.
const L_VALUE: usize = 65341;
/// `nmax`.
const NMAX: usize = 360;
/// The four arrays, `cc`, `cs`, `hc`, `hs`, each `L_VALUE + 1` doubles.
static COEF: &[u8] = include_bytes!("../../../assets/geoid/egm96.bin");
const ARRAY: usize = L_VALUE + 1;

/// `Coef.cc[k]`, `cs`, `hc`, `hs`: array `which` (0 to 3) at `k`.
fn coef(which: usize, k: usize) -> f64 {
    let at = (which * ARRAY + k) * 8;
    let bytes: [u8; 8] = COEF[at..at + 8].try_into().unwrap_or([0; 8]);
    f64::from_le_bytes(bytes)
}

/// The static constructor's `drts[n] = sqrt(n)` and `dirt[n] = 1 / drts[n]` for `n` to
/// `2 * nmax + 1`.
/// `// C#: GeoidHeights.cs:14-23`
struct Roots {
    drts: Vec<f64>,
    dirt: Vec<f64>,
}

fn roots() -> &'static Roots {
    static ROOTS: OnceLock<Roots> = OnceLock::new();
    ROOTS.get_or_init(|| {
        let mut drts = vec![0.0; 1301];
        let mut dirt = vec![0.0; 1301];
        for n in 1..=(2 * NMAX + 1) {
            #[allow(clippy::cast_precision_loss)] // n to 721
            let root = (n as f64).sqrt();
            drts[n] = root;
            dirt[n] = 1.0 / root;
        }
        Roots { drts, dirt }
    })
}

/// `undulation(degLat, degLon)`: the geoid's height above the WGS84 ellipsoid, metres.
/// `// C#: GeoidHeights.cs:25-45`
#[must_use]
pub fn undulation(deg_lat: f64, deg_lon: f64) -> f64 {
    let lat = deg_lat * std::f64::consts::PI / 180.0;
    let lon = deg_lon * std::f64::consts::PI / 180.0;
    let k = NMAX + 1;
    let mut p = vec![0.0; L_VALUE + 1];
    let mut sinml = vec![0.0; 362];
    let mut cosml = vec![0.0; 362];
    let (rlat, gr, re) = radgra(lat, lon);
    let rlat = std::f64::consts::PI / 2.0 - rlat;
    let mut rleg = vec![0.0; 362];
    for j in 1..=k {
        let m = j - 1;
        legfdn(m, rlat, &mut rleg);
        for i in j..=k {
            p[(i - 1) * i / 2 + m + 1] = rleg[i];
        }
    }
    dscml(lon, &mut sinml, &mut cosml);
    hundu(&p, &sinml, &cosml, gr, re)
}

/// `radgra`: the geocentric distance, the geocentric latitude and the normal gravity at the
/// point, from WGS84 (g873).
/// `// C#: GeoidHeights.cs:47-70`
fn radgra(lat: f64, lon: f64) -> (f64, f64, f64) {
    const A: f64 = 6_378_137.0;
    const E2: f64 = 0.006_694_379_990_13;
    const GEQT: f64 = 9.780_325_335_9;
    const K: f64 = 0.001_931_852_652_46;
    let t1 = lat.sin() * lat.sin();
    let n = A / (1.0 - E2 * t1).sqrt();
    let t2 = n * lat.cos();
    let x = t2 * lon.cos();
    let y = t2 * lon.sin();
    let z = (n * (1.0 - E2)) * lat.sin();
    let re = (x * x + y * y + z * z).sqrt();
    let rlat = (z / (x * x + y * y).sqrt()).atan();
    let gr = GEQT * (1.0 + K * t1) / (1.0 - E2 * t1).sqrt();
    (rlat, gr, re)
}

/// `legfdn`: the normalised Legendre functions of order `m` at colatitude `theta` into `rleg`.
/// `// C#: GeoidHeights.cs:72-123`
fn legfdn(m: usize, theta: f64, rleg: &mut [f64]) {
    let Roots { drts, dirt } = roots();
    let mut rlnn = vec![0.0; 362];
    let nmx1 = NMAX + 1;
    let m1 = m + 1;
    let m2 = m + 2;
    let m3 = m + 3;
    let cothet = theta.cos();
    let sithet = theta.sin();
    rlnn[1] = 1.0;
    rlnn[2] = sithet * drts[3];
    for n1 in 3..=m1 {
        let n = n1 - 1;
        let n2 = 2 * n;
        rlnn[n1] = drts[n2 + 1] * dirt[n2] * sithet * rlnn[n];
    }
    match m {
        1 => {
            rleg[2] = rlnn[2];
            rleg[3] = drts[5] * cothet * rleg[2];
        }
        0 => {
            rleg[1] = 1.0;
            rleg[2] = cothet * drts[3];
        }
        _ => {}
    }
    rleg[m1] = rlnn[m1];
    if m2 <= nmx1 {
        rleg[m2] = drts[m1 * 2 + 1] * cothet * rleg[m1];
        if m3 <= nmx1 {
            for n1 in m3..=nmx1 {
                let n = n1 - 1;
                if (m == 0 && n < 2) || (m == 1 && n < 3) {
                    continue;
                }
                let n2 = 2 * n;
                rleg[n1] = drts[n2 + 1]
                    * dirt[n + m]
                    * dirt[n - m]
                    * (drts[n2 - 1] * cothet * rleg[n1 - 1]
                        - drts[n + m - 1] * drts[n - m - 1] * dirt[n2 - 3] * rleg[n1 - 2]);
            }
        }
    }
}

/// `hundu`: the undulation from the harmonic sums - the height anomaly over the ellipsoid, then
/// `ac / 100` and `-0.53` to refer it to WGS84.
/// `// C#: GeoidHeights.cs:125-157`
fn hundu(p: &[f64], sinml: &[f64], cosml: &[f64], gr: f64, re: f64) -> f64 {
    const GM: f64 = 0.398_600_441_8e15;
    const AE: f64 = 6_378_137.0;
    let (cc, cs, hc, hs) = (0, 1, 2, 3);
    let ar = AE / re;
    let mut arn = ar;
    let mut ac = 0.0;
    let mut a = 0.0;
    let mut k = 3;
    for n in 2..=NMAX {
        arn *= ar;
        k += 1;
        let mut sum = p[k] * coef(hc, k);
        let mut sumc = p[k] * coef(cc, k);
        for m in 1..=n {
            k += 1;
            let tempc = coef(cc, k) * cosml[m] + coef(cs, k) * sinml[m];
            let temp = coef(hc, k) * cosml[m] + coef(hs, k) * sinml[m];
            sumc += p[k] * tempc;
            sum += p[k] * temp;
        }
        ac += sumc;
        a += sum * arn;
    }
    ac +=
        coef(cc, 1) + p[2] * coef(cc, 2) + p[3] * (coef(cc, 3) * cosml[1] + coef(cs, 3) * sinml[1]);
    a * GM / (gr * re) + ac / 100.0 - 0.53
}

/// `dscml`: `sin(m * lon)` and `cos(m * lon)` for `m` to `nmax`, by recurrence.
/// `// C#: GeoidHeights.cs:159-175`
fn dscml(rlon: f64, sinml: &mut [f64], cosml: &mut [f64]) {
    let a = rlon.sin();
    let b = rlon.cos();
    sinml[1] = a;
    cosml[1] = b;
    sinml[2] = 2.0 * b * a;
    cosml[2] = 2.0 * b * b - 1.0;
    for m in 3..=NMAX {
        sinml[m] = 2.0 * b * sinml[m - 1] - sinml[m - 2];
        cosml[m] = 2.0 * b * cosml[m - 1] - cosml[m - 2];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The asset is the C#'s four arrays: their lengths, and the first and last values of
    /// `Coef.cs`.
    #[test]
    fn the_coefficients_are_the_csharps() {
        assert_eq!(COEF.len(), 4 * ARRAY * 8);
        assert_eq!(coef(0, 0), 0.0);
        assert_eq!(coef(0, 1), -5.027_452_699_772_63);
        assert_eq!(coef(2, 4), 1.403_247_522_032_47e-9);
        assert_eq!(coef(3, L_VALUE), -8.302_249_455_25e-11);
    }

    /// Ten points against the C# library itself, compiled and run under Mono on 2026-10-02
    /// (`undulation` printed with "R"): the same to a micrometre, the libm sines being the
    /// only difference.
    #[test]
    #[allow(clippy::excessive_precision)] // the C#'s "R" digits, as it printed them
    fn undulations_are_the_csharps() {
        let golden = [
            (-35.363_261, 149.165_23, 19.448_113_054_557_41),
            (0.0, 0.0, 17.161_578_498_270_735),
            (51.5, -0.12, 45.947_264_357_018_7),
            (-33.86, 151.21, 22.500_355_469_332_842),
            (40.7, -74.0, -32.775_216_637_513_296),
            (-89.9, 10.0, -29.484_136_745_889_511),
            (89.9, -170.0, 13.566_342_164_862_958),
            (27.9881, 86.925, -28.741_321_231_584_479),
            (-12.0, -77.0, 24.029_073_855_871_736),
            (35.68, 139.69, 36.642_461_905_187_126),
        ];
        for (lat, lon, expected) in golden {
            let ours = undulation(lat, lon);
            assert!(
                (ours - expected).abs() < 1e-6,
                "{lat} {lon}: {ours} against the C#'s {expected}"
            );
        }
    }
}
