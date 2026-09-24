//! The files `CreateReportFiles` writes beside the photos - `location.txt`, `location.csv`,
//! `location.tel`, `location.geo`, `location.jxl`, `location.gpx`, `location.kml`, and an empty
//! `loglocation.csv` - and the positions `doworkGPSOFFSET` writes beside the log.
//!
//! Every number is written as the C# writes it: `ToString(CultureInfo.InvariantCulture)` or
//! string concatenation - .NET Framework's 15 significant digits for a `double`, 7 for a `float`
//! (`mp_log::netfmt`) - with the current culture taken to be the invariant one; the `.jxl`'s IDs
//! as `ToString("0000000")`; the KML's through SharpKml's formatter ([`crate::numfmt`]).
//! Text files are UTF-8 without a byte order mark, lines ended by `Environment.NewLine`; the
//! `.jxl` and `.gpx` are ASCII.
//! `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1089-1125, 1235-1677`

use std::io::Write as _;
use std::path::Path;

use mp_log::netfmt;

use crate::georef::{DEG2RAD, GeoRefImageBase, Output};
use crate::kml;
use crate::location::{Location, OrderedMap, PictureInformation};
use crate::numfmt::NEWLINE;
use crate::photos::{self, SEPARATOR};
use crate::projection::{self, Terrain};
use crate::xml::TextWriter;

/// `JXL_ID_OFFSET`: the first record number of the `.jxl`'s stations.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:37, 1411-1412`
pub const JXL_ID_OFFSET: i32 = 10;

/// The camera settings the form passes to `CreateReportFiles`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReportSettings {
    /// `offset`: the seconds offset, written into `location.tel`'s header.
    pub offset: f32,
    /// `num_camerarotation`: added to each photo's yaw for its ground overlay.
    pub camera_rotation: f64,
    /// `num_hfov`.
    pub hfov: f64,
    /// `num_vfov`.
    pub vfov: f64,
    /// `usegpsalt`: the GPS altitude rather than the AMSL or relative one.
    pub usegpsalt: bool,
}

impl Default for ReportSettings {
    /// The form's defaults: rotation 90, fields of view 200 and 130
    /// (`Georefimage.Designer.cs:199-252`), no offset, no GPS altitude.
    fn default() -> Self {
        Self {
            offset: 0.0,
            camera_rotation: 90.0,
            hfov: 200.0,
            vfov: 130.0,
            usegpsalt: false,
        }
    }
}

/// Every file `CreateReportFiles` writes, by name, in the order it opens them.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ReportFiles {
    /// `(file name, contents)`.
    pub files: Vec<(String, Vec<u8>)>,
    /// The KML, which the form also hands its web server (`georefimage.cs:252-256`).
    pub kml: String,
}

fn double(value: f64) -> String {
    netfmt::double(value)
}

fn single(value: f32) -> String {
    netfmt::single(value)
}

/// `int.ToString("0000000")`.
fn id(value: i32) -> String {
    if value < 0 {
        format!("-{:07}", value.unsigned_abs())
    } else {
        format!("{value:07}")
    }
}

/// `Encoding.ASCII`: every character past 0x7F a `?`.
fn ascii(text: &str) -> Vec<u8> {
    text.chars()
        .map(|c| u8::try_from(c).ok().filter(u8::is_ascii).unwrap_or(b'?'))
        .collect()
}

/// The raw `FieldBook` records, as the C# source holds them - with that file's CRLF line ends,
/// whatever the platform. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1330-1371`
const CAMERA_RECORDS: &str = "   <CameraDesignRecord ID='00000001'>\r
                                      <Type>GoPro   </Type>\r
                                      <HeightPixels>2400</HeightPixels>\r
                                      <WidthPixels>3200</WidthPixels>\r
                                      <PixelSize>0.0000022</PixelSize>\r
                                      <LensModel>Rectilinear</LensModel>\r
                                      <NominalFocalLength>0.002</NominalFocalLength>\r
                                    </CameraDesignRecord>\r
                                    <CameraRecord2 ID='00000002'>\r
                                      <CameraDesignID>00000001</CameraDesignID>\r
                                      <CameraPosition>01</CameraPosition>\r
                                      <Optics>\r
                                        <IdealAngularMagnification>1.0</IdealAngularMagnification>\r
                                        <AngleSymmetricDistortion>\r
                                          <Order3>-0.35</Order3>\r
                                          <Order5>0.15</Order5>\r
                                          <Order7>-0.033</Order7>\r
                                          <Order9> 0</Order9>\r
                                        </AngleSymmetricDistortion>\r
                                        <AngleDecenteringDistortion>\r
                                          <Column>0</Column>\r
                                          <Row>0</Row>\r
                                        </AngleDecenteringDistortion>\r
                                      </Optics>\r
                                      <Geometry>\r
                                        <PerspectiveCenterPixels>\r
                                          <PrincipalPointColumn>-1615.5</PrincipalPointColumn>\r
                                          <PrincipalPointRow>-1187.5</PrincipalPointRow>\r
                                          <PrincipalDistance>-2102</PrincipalDistance>\r
                                        </PerspectiveCenterPixels>\r
                                        <VectorOffset>\r
                                          <X>0</X>\r
                                          <Y>0</Y>\r
                                          <Z>0</Z>\r
                                        </VectorOffset>\r
                                        <BiVectorAngle>\r
                                          <XX>0</XX>\r
                                          <YY>0</YY>\r
                                          <ZZ>-1.5707963268</ZZ>\r
                                        </BiVectorAngle>\r
                                      </Geometry>\r
                                    </CameraRecord2>";

/// The `.jxl`'s header: the job, its coordinate system, the camera records, the instrument and
/// the atmosphere. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1269-1397`
fn jxl_header(w: &mut TextWriter) {
    w.declaration("<?xml version=\"1.0\" encoding=\"us-ascii\" standalone=\"no\"?>");
    w.start_element("JOBFile");
    w.attribute("jobName", "MPGeoRef");
    w.attribute("product", "Gatewing");
    w.attribute("productVersion", "1.0");
    w.attribute("version", "5.6");
    w.start_element("Environment");
    w.start_element("CoordinateSystem");
    w.element_string("SystemName", "Default");
    w.element_string("ZoneName", "Default");
    w.element_string("DatumName", "WGS 1984");
    w.start_element("Ellipsoid");
    w.element_string("EarthRadius", "6378137");
    w.element_string("Flattening", "0.00335281067183");
    w.end_element();
    w.start_element("Projection");
    w.element_string("Type", "NoProjection");
    w.element_string("Scale", "1");
    w.element_string("GridOrientation", "IncreasingNorthEast");
    w.element_string("SouthAzimuth", "false");
    w.element_string("ApplySeaLevelCorrection", "true");
    w.end_element();
    w.start_element("LocalSite");
    w.element_string("Type", "Grid");
    w.element_string("ProjectLocationLatitude", "");
    w.element_string("ProjectLocationLongitude", "");
    w.element_string("ProjectLocationHeight", "");
    w.end_element();
    w.start_element("Datum");
    w.element_string("Type", "ThreeParameter");
    w.element_string("GridName", "WGS 1984");
    w.element_string("Direction", "WGS84ToLocal");
    w.element_string("EarthRadius", "6378137");
    w.element_string("Flattening", "0.00335281067183");
    w.element_string("TranslationX", "0");
    w.element_string("TranslationY", "0");
    w.element_string("TranslationZ", "0");
    w.end_element();
    w.start_element("HorizontalAdjustment");
    w.element_string("Type", "NoAdjustment");
    w.end_element();
    w.start_element("VerticalAdjustment");
    w.element_string("Type", "NoAdjustment");
    w.end_element();
    w.start_element("CombinedScaleFactor");
    w.start_element("Location");
    w.element_string("Latitude", "");
    w.element_string("Longitude", "");
    w.element_string("Height", "");
    w.end_element();
    w.element_string("Scale", "");
    w.end_element();
    w.end_element();
    w.end_element();

    w.start_element("FieldBook");
    w.raw(CAMERA_RECORDS);
    w.start_element("PhotoInstrumentRecord");
    w.attribute("ID", "0000000E");
    w.element_string("Type", "Aerial");
    w.element_string("Model", "X100");
    w.element_string("Serial", "000-000");
    w.element_string("FirmwareVersion", "v0.0");
    w.element_string("UserDefinedName", "Prototype");
    w.end_element();
    w.start_element("AtmosphereRecord");
    w.attribute("ID", "0000000F");
    w.element_string("Pressure", "");
    w.element_string("Temperature", "");
    w.element_string("PPM", "");
    w.element_string("ApplyEarthCurvatureCorrection", "false");
    w.element_string("ApplyRefractionCorrection", "false");
    w.element_string("RefractionCoefficient", "0");
    w.element_string("PressureInputMethod", "ReadFromInstrument");
    w.end_element();
}

/// `GenPhotoStationRecord`: a photo's station, point and image records, numbered from
/// `last_record_n`; the next free number is returned. The roll and pitch passed are always 0 and
/// the yaw goes in as `-yaw` in radians.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1578-1677`
#[allow(clippy::too_many_arguments)]
fn gen_photo_station_record(
    w: &mut TextWriter,
    station_ids: &mut Vec<i32>,
    imgname: &str,
    lat: f64,
    lng: f64,
    alt: f64,
    yaw: f64,
    imgwidth: i32,
    imgheight: i32,
    mut last_record_n: i32,
) -> i32 {
    let (roll, pitch) = (0.0f64, 0.0f64);
    let photo_station_id = last_record_n;
    let point_record_id = last_record_n + 1;
    let image_record_id = last_record_n + 2;
    last_record_n += 3;
    station_ids.push(photo_station_id);
    let yaw = -yaw * DEG2RAD;

    w.start_element("PhotoStationRecord");
    w.attribute("ID", &id(photo_station_id));
    w.element_string("StationName", &photos::file_stem(imgname));
    w.element_string("InstrumentHeight", "");
    w.start_element("RawInstrumentHeight");
    w.element_string("MeasurementMethod", "TrueHeight");
    w.element_string("MeasuredHeight", "0");
    w.element_string("HorizontalOffset", "0");
    w.element_string("VerticalOffset", "0");
    w.end_element();
    w.element_string("InstrumentID", "0000000E");
    w.element_string("AtmosphereID", "0000000F");
    w.element_string("StationType", "RawSensorValues");
    w.start_element("DeviceAxisOrientationData");
    w.start_element("DeviceAxisOrientation");
    w.start_element("BiVector");
    w.element_string("XX", &double(roll));
    w.element_string("YY", &double(pitch));
    w.element_string("ZZ", &double(yaw));
    w.end_element();
    w.end_element();
    w.end_element();
    w.end_element();

    w.start_element("PointRecord");
    w.attribute("ID", &id(point_record_id));
    w.element_string("Name", &photos::file_stem(imgname));
    w.element_string("Code", "");
    w.element_string("Method", "Coordinates");
    w.element_string("SurveyMethod", "Autonomous");
    w.element_string("Classification", "Normal");
    w.element_string("Deleted", "false");
    w.start_element("WGS84");
    w.element_string("Latitude", &double(lat));
    w.element_string("Longitude", &double(lng));
    w.element_string("Height", &double(alt));
    w.end_element();
    w.end_element();

    w.start_element("ImageRecord");
    w.attribute("ID", &id(image_record_id));
    w.element_string("StationID", &id(photo_station_id));
    w.element_string("BackBearingID", "");
    w.element_string("CameraID", "00000002");
    w.element_string("PointRecordID", "");
    w.element_string("FileName", &photos::file_name(imgname));
    w.element_string("HorizontalAngle", "");
    w.element_string("VerticalAngle", "");
    w.element_string("Width", &imgwidth.to_string());
    w.element_string("Height", &imgheight.to_string());
    w.element_string("SourceX", "0");
    w.element_string("SourceY", "0");
    w.element_string("SourceWidth", &imgwidth.to_string());
    w.element_string("SourceHeight", &imgheight.to_string());
    w.end_element();
    last_record_n
}

/// `GenFlightMission`. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1553-1576`
fn gen_flight_mission(w: &mut TextWriter, station_ids: &[i32], last_record_n: i32) {
    w.start_element("FlightMissionRecord");
    w.attribute("ID", &id(last_record_n));
    w.element_string("Name", "MP");
    w.start_element("FlightBlock");
    w.start_element("FlightPlan");
    w.attribute("height", "100");
    w.attribute("percentForwardOverlap", "75");
    w.attribute("percentLateralOverlap", "75");
    w.end_element();
    w.start_element("StationList");
    for station in station_ids {
        w.element_string("StationID", &id(*station));
    }
    w.end_element();
    w.end_element();
    w.end_element();
}

/// `writeGPX`: a track point per photo, its relative altitude as the elevation and its yaw as
/// both course and compass; no declaration, no formatting.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1089-1125`
fn gpx(pictures: &OrderedMap<String, PictureInformation>) -> Vec<u8> {
    let mut w = TextWriter::new(false);
    w.start_element("gpx");
    w.start_element("trk");
    w.start_element("trkseg");
    for p in pictures.values() {
        let l = &p.location;
        w.start_element("trkpt");
        w.attribute("lat", &double(l.lat));
        w.attribute("lon", &double(l.lon));
        w.element_string("time", &l.time.format_gpx());
        w.element_string("ele", &double(l.rel_alt));
        w.element_string("course", &single(l.yaw));
        w.element_string("compass", &single(l.yaw));
        w.end_element();
    }
    ascii(&w.finish())
}

impl GeoRefImageBase {
    /// `CreateReportFiles`: every report file for the matched photos, returned in the order the
    /// C# opens them (the KML also on its own); [`write_report_files`] puts them in the folder.
    /// The path in the KML is the vehicle's positions at the AMSL or relative altitude
    /// `useAMSLAlt` picks, while every photo's own altitude there is AMSL (or GPS); the
    /// `location.*` files use the one `useAMSLAlt` picks.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1235-1551`
    #[must_use]
    pub fn create_report_files(
        &mut self,
        pictures: &OrderedMap<String, PictureInformation>,
        settings: &ReportSettings,
        append_text: Output<'_>,
        terrain: &dyn Terrain,
    ) -> ReportFiles {
        let amsl = self.use_amsl_alt;
        let gpsalt = settings.usegpsalt;
        self.jxl_station_ids.clear();

        let mut jxl = TextWriter::new(true);
        jxl_header(&mut jxl);

        let mut tel = String::new();
        tel.push_str(&format!("version=1{NEWLINE}"));
        tel.push_str(&format!(
            "#seconds offset - {}{NEWLINE}",
            single(settings.offset)
        ));
        tel.push_str(&format!("#longitude and latitude - in degrees{NEWLINE}"));
        tel.push_str(&format!("#name\tutc\tlongitude\tlatitude\theight{NEWLINE}"));
        let mut txt = format!("#name latitude/Y longitude/X height/Z yaw pitch roll SAlt{NEWLINE}");
        let mut geo = format!("EPSG:4326{NEWLINE}");
        let mut csv = String::new();

        append_text("Start Processing\n");
        let mut last_record_n = JXL_ID_OFFSET;

        let path: Vec<(f64, f64, f64)> = self
            .vehicle_locations
            .values()
            .map(|item| (item.lon, item.lat, item.get_altitude(amsl, gpsalt)))
            .collect();

        let mut kml_photos = Vec::new();
        for pic in pictures.values() {
            let l: &Location = &pic.location;
            let filename = photos::file_name(&pic.path);
            let name = photos::file_stem(&pic.path);
            let alpha = f64::from(l.yaw) + settings.camera_rotation;
            let rect = projection::get_bounding_box(
                l.lat,
                l.lon,
                l.get_altitude(true, gpsalt),
                alpha,
                settings.hfov,
                settings.vfov,
                terrain,
            );
            kml_photos.push(kml::Photo {
                name: name.clone(),
                file_lower: filename.to_lowercase(),
                time: l.time,
                lon: l.lon,
                lat: l.lat,
                alt: l.get_altitude(true, gpsalt),
                north: f64::from(rect.bottom()),
                south: f64::from(rect.top()),
                east: f64::from(rect.right()),
                west: f64::from(rect.left()),
                rotation: -alpha % 360.0,
            });

            let alt = l.get_altitude(amsl, gpsalt);
            txt.push_str(&format!(
                "{filename} {} {} {} {} {} {} {}{NEWLINE}",
                double(l.lat),
                double(l.lon),
                double(alt),
                single(l.yaw),
                single(l.pitch),
                single(l.roll),
                double(l.s_alt)
            ));
            csv.push_str(&format!(
                "{filename},{},{},{},{},{},{}{NEWLINE}",
                double(l.lat),
                double(l.lon),
                double(alt),
                single(l.yaw),
                single(l.pitch),
                single(l.roll)
            ));
            tel.push_str(&format!(
                "{filename}\t{}\t{}\t{}\t{}{NEWLINE}",
                l.time.format_exif(),
                double(l.lon),
                double(l.lat),
                double(alt)
            ));
            geo.push_str(&format!(
                "{filename} {} {} {} 0 0 0 10 10{NEWLINE}",
                double(l.lon),
                double(l.lat),
                double(alt)
            ));
            last_record_n = gen_photo_station_record(
                &mut jxl,
                &mut self.jxl_station_ids,
                &pic.path,
                l.lat,
                l.lon,
                alt,
                f64::from(l.yaw),
                pic.width,
                pic.height,
                last_record_n,
            );
        }

        let kml_text = kml::document(&path, &kml_photos);
        let gpx_bytes = gpx(pictures);
        gen_flight_mission(&mut jxl, &self.jxl_station_ids, last_record_n);
        let jxl_bytes = ascii(&jxl.finish());
        append_text("Done \n\n");

        ReportFiles {
            files: vec![
                ("loglocation.csv".to_owned(), Vec::new()),
                ("location.csv".to_owned(), csv.into_bytes()),
                ("location.kml".to_owned(), kml_text.clone().into_bytes()),
                ("location.txt".to_owned(), txt.into_bytes()),
                ("location.geo".to_owned(), geo.into_bytes()),
                ("location.tel".to_owned(), tel.into_bytes()),
                ("location.jxl".to_owned(), jxl_bytes),
                ("location.gpx".to_owned(), gpx_bytes),
            ],
            kml: kml_text,
        }
    }
}

/// Writes each report file into `dir`, replacing what was there.
///
/// # Errors
///
/// Where a file cannot be written.
pub fn write_report_files(dir: &str, files: &ReportFiles) -> std::io::Result<()> {
    for (name, contents) in &files.files {
        std::fs::write(format!("{dir}{SEPARATOR}{name}"), contents)?;
    }
    Ok(())
}

/// `XmlSerializer.Serialize` of `List<VehicleLocation>`: the positions as `XmlTextWriter`
/// writes them indented by two, each property in declaration order, doubles and floats in their
/// round-trip form and times in their round-trip-kind form. The namespace declarations come in
/// mono's order, `xsd` before `xsi`, which is what the oracle runs (.NET Framework is believed to
/// write `xsi` first; not verified).
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:644-653`
#[must_use]
pub fn locations_xml(locations: &OrderedMap<i64, Location>) -> String {
    let mut w = TextWriter::new(true);
    w.declaration("<?xml version=\"1.0\"?>");
    w.start_element("ArrayOfVehicleLocation");
    w.attribute("xmlns:xsd", "http://www.w3.org/2001/XMLSchema");
    w.attribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    for l in locations.values() {
        w.start_element("VehicleLocation");
        w.element_string("Time", &l.time.format_xml_roundtrip());
        w.element_string("Lat", &crate::numfmt::double_roundtrip(l.lat));
        w.element_string("Lon", &crate::numfmt::double_roundtrip(l.lon));
        w.element_string("AltAMSL", &crate::numfmt::double_roundtrip(l.alt_amsl));
        w.element_string("RelAlt", &crate::numfmt::double_roundtrip(l.rel_alt));
        w.element_string("GPSAlt", &crate::numfmt::double_roundtrip(l.gps_alt));
        w.element_string("SAlt", &crate::numfmt::double_roundtrip(l.s_alt));
        w.element_string("Roll", &crate::numfmt::single_roundtrip(l.roll));
        w.element_string("Pitch", &crate::numfmt::single_roundtrip(l.pitch));
        w.element_string("Yaw", &crate::numfmt::single_roundtrip(l.yaw));
        w.end_element();
    }
    w.finish()
}

/// `File.OpenWrite(path)` and the serializer: the file is overwritten from its start and **not
/// truncated**, so a shorter document leaves the tail of a longer one after it.
///
/// # Errors
///
/// Where the file cannot be opened or written (which the C#'s `catch {}` swallows).
pub fn write_locations_xml(
    path: &str,
    locations: &OrderedMap<i64, Location>,
) -> std::io::Result<()> {
    let text = locations_xml(locations);
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(false)
        .open(Path::new(path))?;
    file.write_all(text.as_bytes())
}
