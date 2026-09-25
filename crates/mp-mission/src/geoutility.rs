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

/// `UTM_BAND`: the latitude bands from 80 south, eight degrees each, `X` twice at the top.
const UTM_BAND: &[u8] = b"CDEFGHJKLMNPQRSTUVWXX";
/// `MGRS_EAST`: the 100 km column letters, three sets of eight by zone.
const MGRS_EAST: &[u8] = b"ABCDEFGHJKLMNPQRSTUVWXYZ";
/// `MGRS_NORTH1`: the 100 km row letters, twenty of them.
const MGRS_NORTH1: &[u8] = b"ABCDEFGHJKLMNPQRSTUV";

/// A UTM position as GeoUtility holds one: zone, band, easting and northing.
#[derive(Debug, Clone, PartialEq)]
pub struct Utm {
    /// 1 to 60.
    pub zone: i32,
    /// The band letter, `C` to `X`.
    pub band: char,
    /// Metres east of the false easting.
    pub east: f64,
    /// Metres north, ten million added in the south.
    pub north: f64,
}

impl Utm {
    /// `UTM.ToString()`: `Zoneband + " " + East + " " + North`, each number as `.ToString()`
    /// writes a double (its shortest form here, .NET's 15 digits; the same for the three
    /// decimals the transform rounds to).
    /// `// C#: ExtLibs/GeoUtility/GeoSystem/UTM.cs:251-255`
    #[must_use]
    pub fn text(&self) -> String {
        format!(
            "{}{} {} {}",
            self.zone,
            self.band,
            crate::dotnet::general_f64(self.east),
            crate::dotnet::general_f64(self.north)
        )
    }
}

/// An MGRS reference as GeoUtility holds one.
#[derive(Debug, Clone, PartialEq)]
pub struct Mgrs {
    /// The UTM zone and band.
    pub zone: i32,
    /// The band letter.
    pub band: char,
    /// The 100 km square's two letters.
    pub grid: String,
    /// Metres east within the square, 0 to 99,999.
    pub east: f64,
    /// Metres north within the square.
    pub north: f64,
}

impl Mgrs {
    /// `MGRS.ToString()` at `Precision = 5`: `Zoneband + Grid + EastString + NorthString`, the
    /// two numbers rounded and padded to five digits, then trailing zeros dropped from both
    /// together while both end in one.
    /// `// C#: ExtLibs/GeoUtility/GeoSystem/MGRS.cs:369-388`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // rounded metres within a 100 km square
    pub fn text(&self) -> String {
        let mut e = format!("{:05}", self.east.round() as i64);
        let mut n = format!("{:05}", self.north.round() as i64);
        while e.ends_with('0') && n.ends_with('0') {
            e.pop();
            n.pop();
        }
        format!("{}{}{}{}{}", self.zone, self.band, self.grid, e, n)
    }
}

/// Where GeoUtility refuses.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeoError {
    /// `ERROR_GEO2UTM`: outside 180 W to 180 E or 80 S to 84 N.
    OutsideUtm,
    /// `ERROR_UTM_ZONE`: a zone or band MGRS has no letters for.
    OutsideMgrs,
}

impl std::fmt::Display for GeoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OutsideUtm => f.write_str("GeoUtility.ErrorProvider.GeoException: ERROR_GEO2UTM"),
            Self::OutsideMgrs => {
                f.write_str("GeoUtility.ErrorProvider.GeoException: ERROR_UTM_ZONE")
            }
        }
    }
}

impl std::error::Error for GeoError {}

/// `Transform.WGSUTM`: latitude and longitude in degrees to UTM, the zone with Norway's and
/// Svalbard's exceptions, the band from [`UTM_BAND`], easting and northing rounded to three
/// decimals as GeoUtility rounds them.
///
/// # Errors
///
/// Outside 180 W to 180 E (the west edge excluded) or 80 S to 84 N.
/// `// C#: ExtLibs/GeoUtility/Transformation/WGSUTM.cs:44-131`
#[allow(clippy::many_single_char_names, clippy::similar_names)]
pub fn wgs84_to_utm(lat: f64, lng: f64) -> Result<Utm, GeoError> {
    let (laenge, breite) = (lng, lat);
    if laenge <= -180.0 || laenge > 180.0 || !(-80.0..=84.0).contains(&breite) {
        return Err(GeoError::OutsideUtm);
    }
    let koeff0 = POL
        * (PI / 180.0)
        * (1.0 - 3.0 * EXZENT2 / 4.0 + 45.0 * EXZENT4 / 64.0 - 175.0 * EXZENT6 / 256.0
            + 11025.0 * EXZENT8 / 16384.0);
    let koeff2 = POL
        * (-3.0 * EXZENT2 / 8.0 + 15.0 * EXZENT4 / 32.0 - 525.0 * EXZENT6 / 1024.0
            + 2205.0 * EXZENT8 / 4096.0);
    let koeff4 =
        POL * (15.0 * EXZENT4 / 256.0 - 105.0 * EXZENT6 / 1024.0 + 2205.0 * EXZENT8 / 16384.0);
    let koeff6 = POL * (-35.0 * EXZENT6 / 3072.0 + 315.0 * EXZENT8 / 12288.0);
    #[allow(clippy::cast_possible_truncation)] // `(int)`
    let mut zone = ((laenge + 180.0) / 6.0) as i32 + 1;
    if (56.0..64.0).contains(&breite) && (3.0..12.0).contains(&laenge) {
        zone = 32;
    } else if breite >= 72.0 {
        if (0.0..9.0).contains(&laenge) {
            zone = 31;
        } else if (9.0..21.0).contains(&laenge) {
            zone = 33;
        } else if (21.0..33.0).contains(&laenge) {
            zone = 35;
        } else if (33.0..42.0).contains(&laenge) {
            zone = 37;
        }
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // `(int)(1 + ...)`, 1..=21
    let band_index = (1.0 + (breite + 80.0) / 8.0) as usize;
    let band = UTM_BAND
        .get(band_index.saturating_sub(1))
        .copied()
        .map_or('X', char::from);
    let breite_rad = breite * PI / 180.0;
    let tangens1 = breite_rad.tan();
    let tangens2 = tangens1.powi(2);
    let tangens4 = tangens1.powi(4);
    let cosinus1 = breite_rad.cos();
    let cosinus2 = cosinus1.powi(2);
    let cosinus3 = cosinus1.powi(3);
    let cosinus4 = cosinus1.powi(4);
    let cosinus5 = cosinus1.powi(5);
    let eta = EXZENT2 * cosinus2;
    let qkhm = POL / (1.0 + eta).sqrt();
    let lmbog = (koeff0 * breite)
        + (koeff2 * (2.0 * breite_rad).sin())
        + (koeff4 * (4.0 * breite_rad).sin())
        + (koeff6 * (6.0 * breite_rad).sin());
    let merid = f64::from((zone - 30) * 6 - 3);
    let dlaenge1 = (laenge - merid) * PI / 180.0;
    let dlaenge2 = dlaenge1.powi(2);
    let dlaenge3 = dlaenge1.powi(3);
    let dlaenge4 = dlaenge1.powi(4);
    let dlaenge5 = dlaenge1.powi(5);
    let northing = FAKTOR
        * (lmbog
            + qkhm * cosinus2 * tangens1 * dlaenge2 / 2.0
            + qkhm * cosinus4 * tangens1 * (5.0 - tangens2 + 9.0 * eta) * dlaenge4 / 24.0);
    let north = if breite < 0.0 {
        10E+06 + northing
    } else {
        northing
    };
    let east = FAKTOR
        * (qkhm * cosinus1 * dlaenge1
            + qkhm * cosinus3 * (1.0 - tangens2 + eta) * dlaenge3 / 6.0
            + qkhm * cosinus5 * (5.0 - 18.0 * tangens2 + tangens4) * dlaenge5 / 120.0)
        + FALSE_EASTING;
    Ok(Utm {
        zone,
        band,
        east: (east * 1000.0).round() / 1000.0,
        north: (north * 1000.0).round() / 1000.0,
    })
}

/// `Transform.UTMMGR`: the 100 km square's letters from the zone and the leading digits of the
/// easting and northing, and the metres within the square.
///
/// # Errors
///
/// A zone outside 1 to 60 or a band outside `C` to `X`.
/// `// C#: ExtLibs/GeoUtility/Transformation/UTMMGR.cs:44-105`
#[allow(clippy::cast_possible_truncation)] // rounded metres, seven digits at most
pub fn utm_to_mgrs(utm: &Utm) -> Result<Mgrs, GeoError> {
    if !(1..=60).contains(&utm.zone) || !('C'..='X').contains(&utm.band) {
        return Err(GeoError::OutsideMgrs);
    }
    // The first digit of the easting and the first two of the northing, from the text of the
    // numbers before their decimal point.
    let east_whole = format!("{}", utm.east.trunc());
    let north_whole = format!("{}", utm.north.trunc());
    let east_plan: i32 = east_whole
        .get(..1)
        .and_then(|d| d.parse().ok())
        .unwrap_or(0);
    let north_plan: i32 = north_whole
        .get(..2)
        .and_then(|d| d.parse().ok())
        .unwrap_or(0);
    let mut east = format!("{}", utm.east.round() as i64);
    if east.len() > 2 {
        east.remove(0);
    }
    let mut north = format!("{}", utm.north.round() as i64);
    if north.len() > 5 {
        north = north.split_off(north.len() - 5);
    }
    if east.len() < north.len() {
        east = format!("{east:0>width$}", width = north.len());
    } else if north.len() < east.len() {
        north = format!("{north:0>width$}", width = east.len());
    }
    let eastgrid = match utm.zone % 3 {
        1 => east_plan - 1,
        2 => east_plan + 7,
        _ => east_plan + 15,
    };
    let mut northgrid = if utm.zone % 2 == 1 { 0 } else { 5 };
    let mut i = north_plan;
    while i - 20 >= 0 {
        i -= 20;
    }
    northgrid += i;
    if northgrid > 19 {
        northgrid -= 20;
    }
    let letter = |table: &[u8], index: i32| {
        usize::try_from(index)
            .ok()
            .and_then(|index| table.get(index))
            .copied()
            .map(char::from)
    };
    let (Some(column), Some(row)) = (letter(MGRS_EAST, eastgrid), letter(MGRS_NORTH1, northgrid))
    else {
        return Err(GeoError::OutsideMgrs);
    };
    Ok(Mgrs {
        zone: utm.zone,
        band: utm.band,
        grid: format!("{column}{row}"),
        east: east.parse().unwrap_or(0.0),
        north: north.parse().unwrap_or(0.0),
    })
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

    /// The forward transform undoes the inverse: a point to UTM and back lands within a tenth of
    /// a millimetre, in both hemispheres and at Norway's widened zone 32.
    #[test]
    fn wgs84_to_utm_round_trips_through_utm_to_wgs84() {
        for (lat, lng, zone, band) in [
            (-35.363_262_1, 149.165_237_4, 55, 'H'),
            (50.256_648_573_733_31, 7.770_338_421_434_999_5, 32, 'U'),
            (60.0, 5.0, 32, 'V'),
            (-0.5, 100.0, 47, 'M'),
        ] {
            let utm = wgs84_to_utm(lat, lng).expect("inside UTM");
            assert_eq!((utm.zone, utm.band), (zone, band), "{lat},{lng}");
            // The forward transform rounds to a millimetre and both series stop at the sixth
            // order, so the way back is good to a centimetre - four degrees off the meridian in
            // Norway's widened zone 32 is the worst of these.
            let (back_lat, back_lng) = utm_to_wgs84(utm.zone, lat < 0.0, utm.east, utm.north);
            assert!((back_lat - lat).abs() < 1e-7, "{lat} -> {back_lat}");
            assert!((back_lng - lng).abs() < 1e-7, "{lng} -> {back_lng}");
        }
    }

    /// GeoUtility's documentation example: 50.256648..., 7.770338... is zone 32U, 412345 east,
    /// 5567890 north, the numbers rounded to three decimals as `WGSUTM` rounds them.
    #[test]
    fn the_forward_transform_matches_the_inverses_fixture() {
        let utm = wgs84_to_utm(50.256_648_573_733_31, 7.770_338_421_434_999_5).expect("32U");
        assert!((utm.east - 412_345.0).abs() < 1e-3, "{}", utm.east);
        assert!((utm.north - 5_567_890.0).abs() < 1e-3, "{}", utm.north);
        assert_eq!(utm.text(), "32U 412345 5567890");
        let south = wgs84_to_utm(-35.367_300_939_431_25, 149.150_818_946_518_8).expect("55H");
        assert!((south.east - 695_400.0).abs() < 1e-3, "{}", south.east);
        assert!((south.north - 6_084_100.0).abs() < 1e-3, "{}", south.north);
    }

    #[test]
    fn outside_the_utm_bands_is_error_geo2utm() {
        assert_eq!(wgs84_to_utm(85.0, 0.0), Err(GeoError::OutsideUtm));
        assert_eq!(wgs84_to_utm(-80.5, 0.0), Err(GeoError::OutsideUtm));
        assert_eq!(wgs84_to_utm(0.0, -180.0), Err(GeoError::OutsideUtm));
        assert!(wgs84_to_utm(0.0, 180.0).is_ok());
        assert_eq!(
            GeoError::OutsideUtm.to_string(),
            "GeoUtility.ErrorProvider.GeoException: ERROR_GEO2UTM"
        );
    }

    /// MGRS from UTM: the 100 km square's column letter is the easting's leading digit into the
    /// zone's third of the alphabet, the row letter the northing's leading two digits modulo 20,
    /// offset five for an even zone; the digits are the metres within the square.
    #[test]
    fn mgrs_letters_and_digits_follow_the_scheme() {
        // 55H, 695400 E: column 6 in set 1 (zone 55 % 3 == 1) is F; 6084100 N: 60 - 3*20 = 0,
        // zone odd so no offset: A.
        let utm = Utm {
            zone: 55,
            band: 'H',
            east: 695_400.0,
            north: 6_084_100.0,
        };
        let mgrs = utm_to_mgrs(&utm).expect("55HFA");
        assert_eq!(mgrs.grid, "FA");
        assert!((mgrs.east - 95_400.0).abs() < f64::EPSILON);
        assert!((mgrs.north - 84_100.0).abs() < f64::EPSILON);
        // Trailing zeros dropped from both together while both end in one: 95400 / 84100 ->
        // 954 / 841.
        assert_eq!(mgrs.text(), "55HFA954841");
        // 32U, 412345 E: column 4 in set 2 (32 % 3 == 2) is 4 + 7 = 11 -> M; 5567890 N: 55 - 40
        // = 15, zone even so + 5 = 20 -> wraps to 0 -> A.
        let utm = Utm {
            zone: 32,
            band: 'U',
            east: 412_345.0,
            north: 5_567_890.0,
        };
        let mgrs = utm_to_mgrs(&utm).expect("32UMA");
        assert_eq!(mgrs.text(), "32UMA1234567890");
        assert_eq!(
            utm_to_mgrs(&Utm {
                zone: 61,
                band: 'U',
                east: 0.0,
                north: 0.0
            }),
            Err(GeoError::OutsideMgrs)
        );
    }

    #[test]
    fn mgrs_pads_to_five_digits_each() {
        let utm = Utm {
            zone: 55,
            band: 'H',
            east: 600_007.0,
            north: 6_000_003.0,
        };
        let mgrs = utm_to_mgrs(&utm).expect("mgrs");
        assert!((mgrs.east - 7.0).abs() < f64::EPSILON);
        assert!((mgrs.north - 3.0).abs() < f64::EPSILON);
        assert_eq!(mgrs.text(), "55HFA0000700003");
    }
}
