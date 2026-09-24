//! GeoUtility's UTM to WGS 84 transformation, which the planner's Enter UTM Coord goes through.
//!
//! `enterUTMCoordToolStripMenuItem_Click` (`GCSViews/FlightPlanner.cs:3258-3286` @ efb0801) builds
//! a `GeoUtility.GeoSystem.UTM` and casts it to `Geographic`, which is `Transform.UTMWGS`
//! (`ExtLibs/GeoUtility/Transformation/UTMWGS.cs:44-116`, constants in `Definition.cs:38-58`): a
//! series in the eccentricity for the footpoint latitude, then the usual transverse Mercator
//! inverse to sixth order. It is not ProjNet's, which the survey grid's `utm.rs` carries, and
//! the two differ in the ninth decimal of a degree - so this one is ported for the one handler
//! that uses it.
//!
//! The constructor the handler calls (`Geocentric(zone, east, north, hem)`, `GeoSystem/Base/
//! Geocentric.cs:84-98`) sets the band to "N" for the north and "A" for the south before the
//! transform reads it, and `UTMWGS` takes a band below 'N' with a positive northing as the
//! southern hemisphere, subtracting ten million metres. That is all the band does here.

use std::f64::consts::PI;

/// `WGS84_HALBACHSE`, the semi-major axis.
const HALBACHSE: f64 = 6_378_137.000;
/// `WGS84_ABPLATTUNG`, the flattening.
const ABPLATTUNG: f64 = 3.352_810_664_747_48E-03;
/// `WGS84_POL`: the polar radius of curvature.
const POL: f64 = HALBACHSE / (1.0 - ABPLATTUNG);
/// `WGS84_EXZENT2`: the second eccentricity squared.
const EXZENT2: f64 =
    ((2.0 * ABPLATTUNG) - (ABPLATTUNG * ABPLATTUNG)) / ((1.0 - ABPLATTUNG) * (1.0 - ABPLATTUNG));
const EXZENT4: f64 = EXZENT2 * EXZENT2;
const EXZENT6: f64 = EXZENT4 * EXZENT2;
const EXZENT8: f64 = EXZENT4 * EXZENT4;
/// `UTM_FAKTOR`, the scale on the central meridian.
const FAKTOR: f64 = 0.9996;
/// `UTM_FALSE_EASTING`.
const FALSE_EASTING: f64 = 500_000.0;

/// `Transform.UTMWGS`: (latitude, longitude) in degrees from a zone, an easting and a northing,
/// `south` standing for the band letter below 'N' the constructor gives the southern hemisphere.
/// `// C#: ExtLibs/GeoUtility/Transformation/UTMWGS.cs:44-116`
#[must_use]
#[allow(clippy::many_single_char_names, clippy::similar_names)]
pub fn utm_to_wgs84(zone: i32, south: bool, east: f64, north: f64) -> (f64, f64) {
    let koeff0 = POL
        * (PI / 180.0)
        * (1.0 - 3.0 * EXZENT2 / 4.0 + 45.0 * EXZENT4 / 64.0 - 175.0 * EXZENT6 / 256.0
            + 11025.0 * EXZENT8 / 16384.0);
    let koeff2 = (180.0 / PI)
        * (3.0 * EXZENT2 / 8.0 - 3.0 * EXZENT4 / 16.0 + 213.0 * EXZENT6 / 2048.0
            - 255.0 * EXZENT8 / 4096.0);
    let koeff4 =
        (180.0 / PI) * (21.0 * EXZENT4 / 256.0 - 21.0 * EXZENT6 / 256.0 + 533.0 * EXZENT8 / 8192.0);
    let koeff6 = (180.0 / PI) * (151.0 * EXZENT6 / 6144.0 - 453.0 * EXZENT8 / 12288.0);
    // `if (b < 'N' && band != "" && north > 0) north = north - 10E+06;`
    let north = if south && north > 0.0 {
        north - 10E+06
    } else {
        north
    };
    let sig = (north / FAKTOR) / koeff0;
    let sig_rad = sig * PI / 180.0;
    let fbreite = sig
        + koeff2 * (2.0 * sig_rad).sin()
        + koeff4 * (4.0 * sig_rad).sin()
        + koeff6 * (6.0 * sig_rad).sin();
    let breite_rad = fbreite * PI / 180.0;
    let tangens1 = breite_rad.tan();
    let tangens2 = tangens1 * tangens1;
    let tangens4 = tangens2 * tangens2;
    let cosinus1 = breite_rad.cos();
    let cosinus2 = cosinus1 * cosinus1;
    let eta = EXZENT2 * cosinus2;
    let qkhm1 = POL / (1.0 + eta).sqrt();
    let qkhm2 = qkhm1.powi(2);
    let qkhm3 = qkhm1.powi(3);
    let qkhm4 = qkhm1.powi(4);
    let qkhm5 = qkhm1.powi(5);
    let qkhm6 = qkhm1.powi(6);
    let merid = f64::from((zone - 30) * 6 - 3);
    let dlaenge1 = (east - FALSE_EASTING) / FAKTOR;
    let dlaenge2 = dlaenge1.powi(2);
    let dlaenge3 = dlaenge1.powi(3);
    let dlaenge4 = dlaenge1.powi(4);
    let dlaenge5 = dlaenge1.powi(5);
    let dlaenge6 = dlaenge1.powi(6);
    let bfakt2 = -tangens1 * (1.0 + eta) / (2.0 * qkhm2);
    let bfakt4 = tangens1 * (5.0 + 3.0 * tangens2 + 6.0 * eta * (1.0 - tangens2)) / (24.0 * qkhm4);
    let bfakt6 = -tangens1 * (61.0 + 90.0 * tangens2 + 45.0 * tangens4) / (720.0 * qkhm6);
    let lfakt1 = 1.0 / (qkhm1 * cosinus1);
    let lfakt3 = -(1.0 + 2.0 * tangens2 + eta) / (6.0 * qkhm3 * cosinus1);
    let lfakt5 = (5.0 + 28.0 * tangens2 + 24.0 * tangens4) / (120.0 * qkhm5 * cosinus1);
    let breite =
        fbreite + (180.0 / PI) * (bfakt2 * dlaenge2 + bfakt4 * dlaenge4 + bfakt6 * dlaenge6);
    let laenge = merid + (180.0 / PI) * (lfakt1 * dlaenge1 + lfakt3 * dlaenge3 + lfakt5 * dlaenge5);
    (breite, laenge)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The formula transcribed independently (Python, `UTMWGS.cs` line by line) over three
    /// points: the handler's own defaults in zone 50 south, the committed field's corner in 55
    /// south, and GeoUtility's documentation example in 32 north. Not an oracle run of the C#:
    /// GeoUtility is not built by the mono harness yet.
    #[test]
    fn the_transform_matches_its_transcription() {
        let (lat, lng) = utm_to_wgs84(50, true, 578_994.0, 6_126_244.0);
        assert!((lat - -35.003_341_850_105_35).abs() < 1e-12, "{lat}");
        assert!((lng - 117.865_695_813_446_24).abs() < 1e-12, "{lng}");
        let (lat, lng) = utm_to_wgs84(55, true, 695_400.0, 6_084_100.0);
        assert!((lat - -35.367_300_939_431_25).abs() < 1e-12, "{lat}");
        assert!((lng - 149.150_818_946_518_8).abs() < 1e-12, "{lng}");
        let (lat, lng) = utm_to_wgs84(32, false, 412_345.0, 5_567_890.0);
        assert!((lat - 50.256_648_573_733_31).abs() < 1e-12, "{lat}");
        assert!((lng - 7.770_338_421_434_999_5).abs() < 1e-12, "{lng}");
    }

    /// GeoUtility and ProjNet (the grid's `utm.rs`) agree to the eighth decimal and no further:
    /// the reason this transform is carried separately.
    #[test]
    fn it_is_not_projnets() {
        let (lat, lng) = utm_to_wgs84(55, true, 695_400.0, 6_084_100.0);
        let projnet = crate::shapefile::position(
            Some(crate::shapefile::Projection::Utm {
                zone: 55,
                north: false,
            }),
            695_400.0,
            6_084_100.0,
        )
        .expect("a position");
        assert!((lat - projnet.latitude()).abs() < 1e-8);
        assert!((lng - projnet.longitude()).abs() < 1e-8);
        assert!((lat - projnet.latitude()).abs() > 1e-11);
    }
}
