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

//! The Survey (Grid) dialog's `.grid` file: `Grid/GridData.cs` as `XmlSerializer` writes and
//! reads it.
//!
//! `GridUI.SaveGrid` gathers the dialog's settings into a `GridData` (`savegriddata`) and
//! serialises it with `new XmlSerializer(typeof(GridData))` into a `StreamWriter`; `LoadGrid`
//! deserialises one from a `StreamReader` and sets the controls from it (`loadgriddata`,
//! [`crate::gridui::Dialog::load_grid_data`]) (`Grid/GridUI.cs:154-323`). The file is whatever
//! .NET's `XmlSerializer` makes of that struct, and this module writes and reads the same bytes, so
//! a file Mission Planner saved loads here and one saved here loads there.
//!
//! - **The writer** is `XmlSerializer.Serialize(TextWriter, object)` under the .NET Framework 4.7.2
//!   Mission Planner is built for (`MissionPlanner.csproj`, `net472`): an `XmlTextWriter` indented
//!   two spaces; the XML declaration with the `StreamWriter`'s encoding, `utf-8`, and no byte order
//!   mark; the root named after the type, with `XmlSerializer`'s default namespace declarations,
//!   `xsi` then `xsd`; then every public field of `GridData` in declaration order as an element of
//!   its own name - the struct has no attribute that renames or skips one, and no property.
//!   `poly`, a `List<PointLatLngAlt>`, is an element of `PointLatLngAlt` elements, each its public
//!   read-write properties in their order: `Lat`, `Lng`, `Alt`, `Tag`, `Tag2`, and `color`, a
//!   `System.Drawing.Color` whose properties are all read-only, so an element with nothing in it.
//!   Each value is `XmlConvert.ToString`'s: a `decimal` with the scale it holds ("5.0" stays
//!   "5.0"), a `bool` `true` or `false`, a `double` `ToString("R")` in the invariant culture (the
//!   .NET Framework's: fifteen significant digits when they read back as the same double, else
//!   seventeen; `INF`, `-INF`, `NaN` and `-0` by name), a string escaped as `XmlTextWriter`
//!   escapes text, an empty one an element closed short (`<camera />`). Lines end in
//!   [`NEW_LINE`], `Environment.NewLine`, the last with none.
//! - **The reader** is `XmlSerializer.Deserialize`: a document that is not well formed, or whose
//!   root is not `GridData` in no namespace, is "There is an error in XML document"; the members
//!   are found by name in any order, an element of no member's name - or a member's second - is
//!   passed over, and a member missing keeps `default(GridData)`'s value, 0, false or nothing.
//!   `XmlConvert` reads each value, and one it refuses fails the whole file.
//!
//! The reader is this module's own, as `cameras.rs`'s is: the crate has no XML dependency, and
//! `XmlSerializer` needs no more of XML than elements, text, the predefined entities, character
//! references and CDATA, with comments, processing instructions and a document type passed over.
//!
//! The tests hold both halves to a file made by hand from `GridData.cs` and these rules,
//! `testdata/grid/file/square.grid`, until the mono oracle writes one (`testdata/grid/README.md`).

use std::fmt::Write as _;
use std::path::Path;

use crate::cameras::NEW_LINE;
use crate::dotnet::{Decimal, general_f64, parse_f64};

/// `ofd.Filter` and `sfd.Filter`: both file dialogs list `.grid` files.
/// `// C#: Grid/GridUI.cs:160, 188`
pub const FILTER: &str = "*.grid|*.grid";

/// The extension [`FILTER`] names: `SaveFileDialog`'s `AddExtension` puts it after a name typed
/// without one.
pub const EXTENSION: &str = "grid";

/// The root element: `XmlSerializer` names it after the type.
const ROOT: &str = "GridData";

/// `poly`'s items, named after their type.
const POINT: &str = "PointLatLngAlt";

/// The namespaces `XmlSerializer` declares on the root when it is given none, in the .NET
/// Framework's order.
const NAMESPACES: &str = " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
                          xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\"";

/// How many members `GridData` has after `poly`.
const MEMBER_COUNT: usize = 42;

/// `GridData`'s members after `poly`, as `GridData.cs` declares them: the order `XmlSerializer`
/// writes them in, and the names it reads them by.
/// `// C#: Grid/GridData.cs:8-64`
const MEMBERS: [&str; MEMBER_COUNT] = [
    "camera",
    "alt",
    "angle",
    "camdir",
    "speed",
    "usespeed",
    "autotakeoff",
    "autotakeoff_RTL",
    "splitmission",
    "internals",
    "footprints",
    "advanced",
    "dist",
    "overshoot1",
    "overshoot2",
    "leadin",
    "leadin2",
    "startfrom",
    "overlap",
    "sidelap",
    "spacing",
    "crossgrid",
    "spiral",
    "copter_delay",
    "copter_headinghold_chk",
    "copter_headinghold",
    "copter_spline",
    "alternateLanes",
    "minlaneseparation",
    "clockwiseLaps",
    "laps",
    "matchPerimeter",
    "trigdist",
    "digicam",
    "repeatservo",
    "breaktrigdist",
    "repeatservo_no",
    "repeatservo_pwm",
    "repeatservo_cycle",
    "setservo_no",
    "setservo_low",
    "setservo_high",
];

/// Deeper than this the reader stops: a `GridData` is four elements deep, and a file nested past
/// this is no grid file.
const DEPTH_MAX: usize = 256;

/// `FormatException`'s message, for a number `XmlConvert` cannot read.
const FORMAT: &str = "Input string was not in a correct format.";

/// The reader's words where the document is not well formed: `XmlException`'s, near enough.
const UNEXPECTED_END: &str = "Unexpected end of file has occurred.";
/// See [`UNEXPECTED_END`].
const ROOT_LEVEL: &str = "Data at the root level is invalid.";
/// See [`UNEXPECTED_END`].
const NAME_START: &str = "Name cannot begin with this character.";
/// `ReadElementString` on a member with elements in it.
const SIMPLE_CONTENT: &str =
    "ReadElementString could only be called on an element with simple or empty content.";

/// A corner of `poly`: `PointLatLngAlt`'s public read-write properties but `color`, which
/// `XmlSerializer` writes as an empty element and reads as `Color.Empty`.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:19-30`
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointLatLngAlt {
    /// `Lat`, degrees.
    pub lat: f64,
    /// `Lng`, degrees.
    pub lng: f64,
    /// `Alt`, metres.
    pub alt: f64,
    /// `Tag`, `""` unless set.
    pub tag: String,
    /// `Tag2`, likewise.
    pub tag2: String,
}

impl PointLatLngAlt {
    /// A corner of the dialog's polygon: the conversion from GMap's `PointLatLng`, which the
    /// drawn polygon's corners are - no altitude, no tags.
    /// `// C#: Grid/GridUI.cs:94-95; ExtLibs/Utilities/PointLatLngAlt.cs:48-52, 97-100`
    #[must_use]
    pub fn new(lat: f64, lng: f64) -> Self {
        Self {
            lat,
            lng,
            ..Self::default()
        }
    }
}

/// `GridData`: what `savegriddata` gathers from the dialog and `loadgriddata` sets it from, member
/// by member as `GridData.cs` declares them. `Default` is `default(GridData)`, what a member a file
/// lacks keeps.
/// `// C#: Grid/GridData.cs:6-67; Grid/GridUI.cs:201-323`
#[derive(Debug, Clone, PartialEq, Default)]
#[allow(clippy::struct_excessive_bools)] // GridData.cs's fields, one for one
pub struct GridData {
    /// `poly`: the polygon, the dialog's `list`.
    pub poly: Vec<PointLatLngAlt>,
    /// `camera`: `CMB_camera.Text`.
    pub camera: String,
    /// `alt`: `NUM_altitude`.
    pub alt: Decimal,
    /// `angle`: `NUM_angle`.
    pub angle: Decimal,
    /// `camdir`: `CHK_camdirection`.
    pub camdir: bool,
    /// `speed`: `NUM_UpDownFlySpeed`.
    pub speed: Decimal,
    /// `usespeed`: `CHK_usespeed`.
    pub usespeed: bool,
    /// `autotakeoff`: `CHK_toandland`.
    pub autotakeoff: bool,
    /// `autotakeoff_RTL`: `CHK_toandland_RTL`.
    pub autotakeoff_rtl: bool,
    /// `splitmission`: `NUM_split`.
    pub splitmission: Decimal,
    /// `internals`: `CHK_internals`.
    pub internals: bool,
    /// `footprints`: `CHK_footprints`.
    pub footprints: bool,
    /// `advanced`: `CHK_advanced`.
    pub advanced: bool,
    /// `dist`: `NUM_Distance`.
    pub dist: Decimal,
    /// `overshoot1`: `NUM_overshoot`.
    pub overshoot1: Decimal,
    /// `overshoot2`: `NUM_overshoot2`.
    pub overshoot2: Decimal,
    /// `leadin`: `NUM_leadin`.
    pub leadin: Decimal,
    /// `leadin2`: `NUM_leadin2`.
    pub leadin2: Decimal,
    /// `startfrom`: `CMB_startfrom.Text`.
    pub startfrom: String,
    /// `overlap`: `num_overlap`.
    pub overlap: Decimal,
    /// `sidelap`: `num_sidelap`.
    pub sidelap: Decimal,
    /// `spacing`: `NUM_spacing`.
    pub spacing: Decimal,
    /// `crossgrid`: `chk_crossgrid`.
    pub crossgrid: bool,
    /// `spiral`: `chk_spiral`.
    pub spiral: bool,
    /// `copter_delay`: `NUM_copter_delay`.
    pub copter_delay: Decimal,
    /// `copter_headinghold_chk`: `CHK_copter_headinghold`.
    pub copter_headinghold_chk: bool,
    /// `copter_headinghold`: `decimal.Parse(TXT_headinghold.Text)`.
    pub copter_headinghold: Decimal,
    /// `copter_spline`: `chk_spline`.
    pub copter_spline: bool,
    /// `alternateLanes`: nothing sets it or reads it, so false.
    pub alternate_lanes: bool,
    /// `minlaneseparation`: `NUM_Lane_Dist`.
    pub minlaneseparation: Decimal,
    /// `clockwiseLaps`: `NUM_clockwise_laps`.
    pub clockwise_laps: Decimal,
    /// `laps`: `NUM_laps`.
    pub laps: Decimal,
    /// `matchPerimeter`: `CHK_match_spiral_perimeter`.
    pub match_perimeter: bool,
    /// `trigdist`: `rad_trigdist`.
    pub trigdist: bool,
    /// `digicam`: `rad_digicam`.
    pub digicam: bool,
    /// `repeatservo`: `rad_repeatservo`.
    pub repeatservo: bool,
    /// `breaktrigdist`: `chk_stopstart`.
    pub breaktrigdist: bool,
    /// `repeatservo_no`: `NUM_reptservo`.
    pub repeatservo_no: Decimal,
    /// `repeatservo_pwm`: `num_reptpwm`.
    pub repeatservo_pwm: Decimal,
    /// `repeatservo_cycle`: `NUM_repttime`.
    pub repeatservo_cycle: Decimal,
    /// `setservo_no`: `num_setservono`.
    pub setservo_no: Decimal,
    /// `setservo_low`: `num_setservolow`.
    pub setservo_low: Decimal,
    /// `setservo_high`: `num_setservohigh`.
    pub setservo_high: Decimal,
}

/// Why a `.grid` file was not read or written, or not all of it loaded.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum GridFileError {
    /// `XmlSerializer.Deserialize`'s `InvalidOperationException`: the file is not well-formed
    /// XML, its root is not a `GridData`, or `XmlConvert` refused a value - its message and,
    /// after it, its inner exception's.
    #[error("There is an error in XML document ({line}, {column}). {reason}")]
    Document {
        /// The line, from 1.
        line: usize,
        /// The column, from 1.
        column: usize,
        /// What the inner exception says.
        reason: String,
    },
    /// A corner no position can be made of - this application's positions are held to the
    /// globe's ranges, which the C#'s doubles are not - so nothing is loaded.
    #[error("corner {corner}: {why}")]
    Corner {
        /// Which, from 1.
        corner: usize,
        /// Why not.
        why: mp_units::PositionError,
    },
    /// `NumericUpDown.Value`'s `ArgumentOutOfRangeException` in `loadgriddata`: a value outside
    /// the control's range, which ends the load there.
    #[error(
        "{control}: Value of '{value}' is not valid for 'Value'. 'Value' should be between \
         'Minimum' and 'Maximum'."
    )]
    OutOfRange {
        /// The control's Designer name.
        control: &'static str,
        /// The value, as `ToString` writes it.
        value: String,
    },
    /// The file could not be opened, read or written: the error's words.
    #[error("{0}")]
    Io(String),
}

impl GridData {
    /// `writer.Serialize(sw, griddata)`: the file `SaveGrid` writes, byte for byte as the module's
    /// header sets out.
    /// `// C#: Grid/GridUI.cs:179, 193-196`
    #[must_use]
    pub fn to_xml(&self) -> String {
        let mut out = String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
        write_line(&mut out, 0, &format!("<{ROOT}{NAMESPACES}>"));
        if self.poly.is_empty() {
            write_line(&mut out, 1, "<poly />");
        } else {
            write_line(&mut out, 1, "<poly>");
            for point in &self.poly {
                write_line(&mut out, 2, &format!("<{POINT}>"));
                write_line(&mut out, 3, &text_element("Lat", &xml_double_text(point.lat)));
                write_line(&mut out, 3, &text_element("Lng", &xml_double_text(point.lng)));
                write_line(&mut out, 3, &text_element("Alt", &xml_double_text(point.alt)));
                write_line(&mut out, 3, &text_element("Tag", &point.tag));
                write_line(&mut out, 3, &text_element("Tag2", &point.tag2));
                write_line(&mut out, 3, "<color />");
                write_line(&mut out, 2, &format!("</{POINT}>"));
            }
            write_line(&mut out, 1, "</poly>");
        }
        for (name, value) in self.members() {
            write_line(&mut out, 1, &text_element(name, &value));
        }
        write_line(&mut out, 0, &format!("</{ROOT}>"));
        out
    }

    /// `(GridData)reader.Deserialize(sr)`: the members a file names, by name, as the module's
    /// header sets out.
    /// `// C#: Grid/GridUI.cs:156, 167`
    ///
    /// # Errors
    /// The `InvalidOperationException`'s message, "There is an error in XML document (line,
    /// column).", and what is wrong there.
    pub fn from_xml(xml: &str) -> Result<Self, GridFileError> {
        // A byte order mark the text kept is no character of the document. Then the end-of-line
        // handling every XML reader does first: CR LF, and a CR alone, are a LF.
        let xml = xml.strip_prefix('\u{feff}').unwrap_or(xml);
        let text = xml.replace("\r\n", "\n").replace('\r', "\n");
        let root = parse(&text)?;
        if root.name != ROOT || root.in_namespace() {
            let reason = format!(
                "<{} xmlns='{}'> was not expected.",
                root.name,
                root.namespace.as_deref().unwrap_or("")
            );
            return Err(document_error(&text, root.at, reason));
        }
        let mut data = Self::default();
        // `paramsRead`: `poly`, then the members in order.
        let mut read = [false; MEMBER_COUNT + 1];
        for member in root.elements() {
            if member.foreign() {
                continue;
            }
            let name = member.name.as_str();
            let index = if name == "poly" {
                0
            } else {
                match MEMBERS.iter().position(|known| *known == name) {
                    Some(index) => index + 1,
                    None => continue,
                }
            };
            match read.get_mut(index) {
                Some(seen) if !*seen => *seen = true,
                _ => continue,
            }
            if index == 0 {
                for point in member.elements() {
                    if point.name == POINT && !point.foreign() {
                        data.poly.push(read_point(&text, point)?);
                    }
                }
                continue;
            }
            let value = member
                .text()
                .ok_or_else(|| document_error(&text, member.at, SIMPLE_CONTENT))?;
            data.set_member(name, &value)
                .map_err(|reason| document_error(&text, member.at, reason))?;
        }
        Ok(data)
    }

    /// `LoadGrid`'s reading: `new StreamReader(ofd.FileName)` - UTF-8, or UTF-16 after its byte
    /// order mark - and [`Self::from_xml`].
    /// `// C#: Grid/GridUI.cs:165-167`
    ///
    /// # Errors
    /// The file cannot be read, or is not a `GridData`.
    pub fn read(path: &Path) -> Result<Self, GridFileError> {
        let bytes = mp_os::fs::read(path).map_err(|error| GridFileError::Io(error.to_string()))?;
        Self::from_xml(&decode(&bytes))
    }

    /// `SaveGrid`'s writing: `new StreamWriter(sfd.FileName)` - UTF-8 without a byte order mark,
    /// over any file of that name - and [`Self::to_xml`].
    /// `// C#: Grid/GridUI.cs:193-196`
    ///
    /// # Errors
    /// The file cannot be written.
    pub fn write(&self, path: &Path) -> Result<(), GridFileError> {
        mp_os::fs::write(path, self.to_xml()).map_err(|error| GridFileError::Io(error.to_string()))
    }

    /// The members after `poly`, each with its value as `XmlConvert.ToString` writes it, in
    /// [`MEMBERS`]' order.
    fn members(&self) -> [(&'static str, String); MEMBER_COUNT] {
        let number = |value: Decimal| value.to_text();
        let flag = |value: bool| xml_bool_text(value).to_owned();
        [
            ("camera", self.camera.clone()),
            ("alt", number(self.alt)),
            ("angle", number(self.angle)),
            ("camdir", flag(self.camdir)),
            ("speed", number(self.speed)),
            ("usespeed", flag(self.usespeed)),
            ("autotakeoff", flag(self.autotakeoff)),
            ("autotakeoff_RTL", flag(self.autotakeoff_rtl)),
            ("splitmission", number(self.splitmission)),
            ("internals", flag(self.internals)),
            ("footprints", flag(self.footprints)),
            ("advanced", flag(self.advanced)),
            ("dist", number(self.dist)),
            ("overshoot1", number(self.overshoot1)),
            ("overshoot2", number(self.overshoot2)),
            ("leadin", number(self.leadin)),
            ("leadin2", number(self.leadin2)),
            ("startfrom", self.startfrom.clone()),
            ("overlap", number(self.overlap)),
            ("sidelap", number(self.sidelap)),
            ("spacing", number(self.spacing)),
            ("crossgrid", flag(self.crossgrid)),
            ("spiral", flag(self.spiral)),
            ("copter_delay", number(self.copter_delay)),
            ("copter_headinghold_chk", flag(self.copter_headinghold_chk)),
            ("copter_headinghold", number(self.copter_headinghold)),
            ("copter_spline", flag(self.copter_spline)),
            ("alternateLanes", flag(self.alternate_lanes)),
            ("minlaneseparation", number(self.minlaneseparation)),
            ("clockwiseLaps", number(self.clockwise_laps)),
            ("laps", number(self.laps)),
            ("matchPerimeter", flag(self.match_perimeter)),
            ("trigdist", flag(self.trigdist)),
            ("digicam", flag(self.digicam)),
            ("repeatservo", flag(self.repeatservo)),
            ("breaktrigdist", flag(self.breaktrigdist)),
            ("repeatservo_no", number(self.repeatservo_no)),
            ("repeatservo_pwm", number(self.repeatservo_pwm)),
            ("repeatservo_cycle", number(self.repeatservo_cycle)),
            ("setservo_no", number(self.setservo_no)),
            ("setservo_low", number(self.setservo_low)),
            ("setservo_high", number(self.setservo_high)),
        ]
    }

    /// One member after `poly` from its element's text: a string as it is, a `decimal` through
    /// `XmlConvert.ToDecimal`, a `bool` through `XmlConvert.ToBoolean`. Err is why the value is
    /// refused.
    fn set_member(&mut self, name: &str, value: &str) -> Result<(), String> {
        match name {
            "camera" => value.clone_into(&mut self.camera),
            "startfrom" => value.clone_into(&mut self.startfrom),
            "alt" => self.alt = read_decimal(value)?,
            "angle" => self.angle = read_decimal(value)?,
            "speed" => self.speed = read_decimal(value)?,
            "splitmission" => self.splitmission = read_decimal(value)?,
            "dist" => self.dist = read_decimal(value)?,
            "overshoot1" => self.overshoot1 = read_decimal(value)?,
            "overshoot2" => self.overshoot2 = read_decimal(value)?,
            "leadin" => self.leadin = read_decimal(value)?,
            "leadin2" => self.leadin2 = read_decimal(value)?,
            "overlap" => self.overlap = read_decimal(value)?,
            "sidelap" => self.sidelap = read_decimal(value)?,
            "spacing" => self.spacing = read_decimal(value)?,
            "copter_delay" => self.copter_delay = read_decimal(value)?,
            "copter_headinghold" => self.copter_headinghold = read_decimal(value)?,
            "minlaneseparation" => self.minlaneseparation = read_decimal(value)?,
            "clockwiseLaps" => self.clockwise_laps = read_decimal(value)?,
            "laps" => self.laps = read_decimal(value)?,
            "repeatservo_no" => self.repeatservo_no = read_decimal(value)?,
            "repeatservo_pwm" => self.repeatservo_pwm = read_decimal(value)?,
            "repeatservo_cycle" => self.repeatservo_cycle = read_decimal(value)?,
            "setservo_no" => self.setservo_no = read_decimal(value)?,
            "setservo_low" => self.setservo_low = read_decimal(value)?,
            "setservo_high" => self.setservo_high = read_decimal(value)?,
            "camdir" => self.camdir = read_bool(value)?,
            "usespeed" => self.usespeed = read_bool(value)?,
            "autotakeoff" => self.autotakeoff = read_bool(value)?,
            "autotakeoff_RTL" => self.autotakeoff_rtl = read_bool(value)?,
            "internals" => self.internals = read_bool(value)?,
            "footprints" => self.footprints = read_bool(value)?,
            "advanced" => self.advanced = read_bool(value)?,
            "crossgrid" => self.crossgrid = read_bool(value)?,
            "spiral" => self.spiral = read_bool(value)?,
            "copter_headinghold_chk" => self.copter_headinghold_chk = read_bool(value)?,
            "copter_spline" => self.copter_spline = read_bool(value)?,
            "alternateLanes" => self.alternate_lanes = read_bool(value)?,
            "matchPerimeter" => self.match_perimeter = read_bool(value)?,
            "trigdist" => self.trigdist = read_bool(value)?,
            "digicam" => self.digicam = read_bool(value)?,
            "repeatservo" => self.repeatservo = read_bool(value)?,
            "breaktrigdist" => self.breaktrigdist = read_bool(value)?,
            // Not a member: `from_xml` passes such an element over before it gets here.
            _ => {}
        }
        Ok(())
    }
}

/// A corner of `poly`: `Lat`, `Lng` and `Alt` through `XmlConvert.ToDouble`, `Tag` and `Tag2` as
/// they are; `color` and anything else passed over, and a member's second.
fn read_point(text: &str, element: &Element) -> Result<PointLatLngAlt, GridFileError> {
    let mut point = PointLatLngAlt::default();
    let mut read = [false; 5];
    for member in element.elements() {
        if member.foreign() {
            continue;
        }
        let index = match member.name.as_str() {
            "Lat" => 0,
            "Lng" => 1,
            "Alt" => 2,
            "Tag" => 3,
            "Tag2" => 4,
            _ => continue,
        };
        match read.get_mut(index) {
            Some(seen) if !*seen => *seen = true,
            _ => continue,
        }
        let value = member
            .text()
            .ok_or_else(|| document_error(text, member.at, SIMPLE_CONTENT))?;
        match index {
            3 => point.tag = value,
            4 => point.tag2 = value,
            _ => {
                let number =
                    xml_double(&value).ok_or_else(|| document_error(text, member.at, FORMAT))?;
                match index {
                    0 => point.lat = number,
                    1 => point.lng = number,
                    _ => point.alt = number,
                }
            }
        }
    }
    Ok(point)
}

/// `XmlConvert.ToDecimal`, with `FormatException`'s message for what it refuses.
fn read_decimal(value: &str) -> Result<Decimal, String> {
    xml_decimal(value).ok_or_else(|| FORMAT.to_owned())
}

/// `XmlConvert.ToBoolean`, with `FormatException`'s message for what it refuses.
fn read_bool(value: &str) -> Result<bool, String> {
    xml_bool(value).ok_or_else(|| format!("The string '{value}' is not a valid Boolean value."))
}

// ---------------------------------------------------------------------------------------------
// XmlConvert.
// ---------------------------------------------------------------------------------------------

/// XML's white space: space, tab, line feed, carriage return (`XmlConvert.TrimString`).
const fn is_space(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\n' | '\r')
}

/// The white space .NET's number parsing allows either side of a number: space, and tab to
/// carriage return.
const fn is_number_white(c: char) -> bool {
    matches!(c, ' ' | '\t'..='\r')
}

/// `XmlConvert.ToString(bool)`.
const fn xml_bool_text(value: bool) -> &'static str {
    if value { "true" } else { "false" }
}

/// `XmlConvert.ToBoolean`: `true` or `1`, `false` or `0`, white space either side; in that case
/// only, so "True" is refused.
fn xml_bool(text: &str) -> Option<bool> {
    match text.trim_matches(is_space) {
        "true" | "1" => Some(true),
        "false" | "0" => Some(false),
        _ => None,
    }
}

/// `XmlConvert.ToDecimal`: `decimal.Parse` in the invariant culture allowing a leading sign, a
/// point and white space either side - no thousands separator, trailing sign or exponent - the
/// scale kept as written.
fn xml_decimal(text: &str) -> Option<Decimal> {
    let number = text.trim_matches(is_number_white);
    let digits = number.strip_prefix(['-', '+']).unwrap_or(number);
    if !digits.chars().all(|c| c.is_ascii_digit() || c == '.') {
        return None;
    }
    Decimal::parse(number)
}

/// `XmlConvert.ToDouble`: `INF` and `-INF` by name; else `double.Parse` in the invariant culture
/// allowing a leading sign, a point, an exponent and white space either side, which takes
/// `Infinity`, `-Infinity` and `NaN` too.
fn xml_double(text: &str) -> Option<f64> {
    let number = text.trim_matches(is_space);
    match number {
        "INF" | "Infinity" => return Some(f64::INFINITY),
        "-INF" | "-Infinity" => return Some(f64::NEG_INFINITY),
        "NaN" => return Some(f64::NAN),
        _ => {}
    }
    let number = number.trim_matches(is_number_white);
    // Rust's parser takes "inf", "nan" and "infinity" in any case, which .NET does not.
    if number.is_empty()
        || !number
            .chars()
            .all(|c| c.is_ascii_digit() || matches!(c, '.' | 'e' | 'E' | '+' | '-'))
    {
        return None;
    }
    number.parse().ok()
}

/// `XmlConvert.ToString(double)`: `-INF`, `INF` and `-0` by name, else `ToString("R")` in the
/// invariant culture as the .NET Framework formats it - the fifteen significant digits of `"G15"`
/// when they read back as the same double, else the seventeen of `"G17"` - and `NaN` for a NaN.
fn xml_double_text(value: f64) -> String {
    let named = if value.is_nan() {
        Some("NaN")
    } else if value.is_infinite() {
        Some(if value > 0.0 { "INF" } else { "-INF" })
    } else if value == 0.0 {
        Some(if value.is_sign_negative() { "-0" } else { "0" })
    } else {
        None
    };
    if let Some(named) = named {
        return named.to_owned();
    }
    let fifteen = general_f64(value);
    if parse_f64(&fifteen) == Some(value) {
        return fifteen;
    }
    general_17(value)
}

/// `"G17"`, `"R"`'s second try: seventeen significant digits correctly rounded, trailing zeros
/// dropped, in scientific notation past seventeen whole digits or below 0.0001, as
/// `FormatGeneral` writes them (`crate::dotnet`'s `"G"`, at seventeen).
fn general_17(value: f64) -> String {
    let text = format!("{:.16e}", value.abs());
    let (mantissa, exponent) = text.split_once('e').unwrap_or((text.as_str(), "0"));
    // Where the point goes: the value is 0.d1d2d3... * 10^scale.
    let scale = exponent.parse::<i32>().unwrap_or(0) + 1;
    let mut digits: Vec<char> = mantissa.chars().filter(char::is_ascii_digit).collect();
    while digits.len() > 1 && digits.last() == Some(&'0') {
        digits.pop();
    }
    let scientific = !(-3..=17).contains(&scale);
    let point = if scientific { 1 } else { scale };
    let count = i32::try_from(digits.len()).unwrap_or(17);
    let digit = |index: i32| {
        usize::try_from(index)
            .ok()
            .and_then(|index| digits.get(index))
            .copied()
            .unwrap_or('0')
    };
    let mut out = String::new();
    if value < 0.0 {
        out.push('-');
    }
    if point > 0 {
        for index in 0..point {
            out.push(digit(index));
        }
    } else {
        out.push('0');
    }
    if count > point {
        out.push('.');
        for _ in point..0 {
            out.push('0');
        }
        for index in point.max(0)..count {
            out.push(digit(index));
        }
    }
    if scientific {
        let exponent = scale - 1;
        let sign = if exponent < 0 { '-' } else { '+' };
        let _ = write!(out, "E{sign}{:02}", exponent.abs());
    }
    out
}

// ---------------------------------------------------------------------------------------------
// The writer.
// ---------------------------------------------------------------------------------------------

/// A line of the indented `XmlTextWriter`: [`NEW_LINE`], two spaces a level, the markup.
fn write_line(out: &mut String, depth: usize, markup: &str) {
    out.push_str(NEW_LINE);
    for _ in 0..depth {
        out.push_str("  ");
    }
    out.push_str(markup);
}

/// An element holding text, as `WriteElementString` writes one: closed short when the text is
/// empty.
fn text_element(name: &str, value: &str) -> String {
    if value.is_empty() {
        format!("<{name} />")
    } else {
        format!("<{name}>{}</{name}>", xml_text(value))
    }
}

/// `XmlTextWriter.WriteString` in an element: `<`, `>` and `&` as entities, a control character
/// other than tab, line feed and carriage return - and U+FFFE and U+FFFF - as a character
/// reference (`XmlTextEncoder.Write`), anything else as it is, in UTF-8.
fn xml_text(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '&' => out.push_str("&amp;"),
            '\t' | '\n' | '\r' => out.push(c),
            c if c < ' ' || matches!(c, '\u{FFFE}' | '\u{FFFF}') => {
                let _ = write!(out, "&#x{:X};", u32::from(c));
            }
            c => out.push(c),
        }
    }
    out
}

// ---------------------------------------------------------------------------------------------
// The reader.
// ---------------------------------------------------------------------------------------------

/// `new StreamReader(path)`'s text: UTF-16 after its byte order mark, little or big endian, else
/// UTF-8 with its mark dropped; what cannot be decoded is U+FFFD.
fn decode(bytes: &[u8]) -> String {
    if let Some(rest) = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]) {
        return String::from_utf8_lossy(rest).into_owned();
    }
    if let Some(rest) = bytes.strip_prefix(&[0xFF, 0xFE]) {
        return utf16(rest, u16::from_le_bytes);
    }
    if let Some(rest) = bytes.strip_prefix(&[0xFE, 0xFF]) {
        return utf16(rest, u16::from_be_bytes);
    }
    String::from_utf8_lossy(bytes).into_owned()
}

/// UTF-16 text, its units read by `unit`.
fn utf16(bytes: &[u8], unit: fn([u8; 2]) -> u16) -> String {
    let units = bytes
        .chunks_exact(2)
        .map(|pair| <[u8; 2]>::try_from(pair).map_or(0xFFFD, unit));
    char::decode_utf16(units)
        .map(|c| c.unwrap_or(char::REPLACEMENT_CHARACTER))
        .collect()
}

/// An element as the reader keeps it: its name, the default namespace it declares, what is in it,
/// and where its start tag is.
#[derive(Debug)]
struct Element {
    name: String,
    /// `xmlns="..."`, when declared.
    namespace: Option<String>,
    content: Vec<Node>,
    /// The start tag's offset in the text, for an error's line and column.
    at: usize,
}

/// What an element holds.
#[derive(Debug)]
enum Node {
    Element(Element),
    /// Text and CDATA, the entities decoded.
    Text(String),
}

impl Element {
    /// The elements in it, in order.
    fn elements(&self) -> impl Iterator<Item = &Self> {
        self.content.iter().filter_map(|node| match node {
            Node::Element(element) => Some(element),
            Node::Text(_) => None,
        })
    }

    /// Whether it declares a default namespace other than none.
    fn in_namespace(&self) -> bool {
        self.namespace.as_deref().is_some_and(|ns| !ns.is_empty())
    }

    /// Whether `XmlSerializer` would take it for no member: in a namespace of its own, or named
    /// with a prefix.
    fn foreign(&self) -> bool {
        self.in_namespace() || self.name.contains(':')
    }

    /// `ReadElementString`: its text, or `None` when there are elements in it. White space alone
    /// is no text, as the serializer's reader passes over insignificant white space.
    fn text(&self) -> Option<String> {
        let mut text = String::new();
        for node in &self.content {
            match node {
                Node::Text(more) => text.push_str(more),
                Node::Element(_) => return None,
            }
        }
        if text.chars().all(is_space) {
            text.clear();
        }
        Some(text)
    }
}

/// "There is an error in XML document": `at`'s line and column in `text`, and why.
fn document_error(text: &str, at: usize, reason: impl Into<String>) -> GridFileError {
    let before = text.get(..at).unwrap_or(text);
    let line = before.matches('\n').count() + 1;
    let column = before
        .rsplit('\n')
        .next()
        .map_or(0, |last| last.chars().count())
        + 1;
    GridFileError::Document {
        line,
        column,
        reason: reason.into(),
    }
}

/// The document's root element, read as `XmlTextReader` reads it - comments, processing
/// instructions and a document type passed over, CDATA as text, the predefined entities and
/// character references decoded - or where it is not well formed. Its lines already end in LF.
fn parse(text: &str) -> Result<Element, GridFileError> {
    let mut at = 0;
    // The elements opened and not yet closed, the innermost last.
    let mut open: Vec<Element> = Vec::new();
    let mut root: Option<Element> = None;
    loop {
        let mut rest = text.get(at..).unwrap_or("");
        if open.is_empty() {
            // Outside the root, white space is nothing.
            let trimmed = rest.trim_start_matches(is_space);
            at += rest.len() - trimmed.len();
            rest = trimmed;
        }
        if rest.is_empty() {
            break;
        }
        if let Some(after) = rest.strip_prefix("<!--") {
            let end = after
                .find("-->")
                .ok_or_else(|| document_error(text, at, UNEXPECTED_END))?;
            at += "<!--".len() + end + "-->".len();
        } else if let Some(after) = rest.strip_prefix("<![CDATA[") {
            let end = after
                .find("]]>")
                .ok_or_else(|| document_error(text, at, UNEXPECTED_END))?;
            let Some(parent) = open.last_mut() else {
                return Err(document_error(text, at, ROOT_LEVEL));
            };
            let data = after.get(..end).unwrap_or("").to_owned();
            parent.content.push(Node::Text(data));
            at += "<![CDATA[".len() + end + "]]>".len();
        } else if let Some(after) = rest.strip_prefix("<?") {
            let end = after
                .find("?>")
                .ok_or_else(|| document_error(text, at, UNEXPECTED_END))?;
            at += "<?".len() + end + "?>".len();
        } else if let Some(after) = rest.strip_prefix("<!") {
            // A document type, which only the prolog may hold: passed over, internal subset and
            // all.
            if !open.is_empty() || root.is_some() {
                return Err(document_error(text, at, "Unexpected DTD declaration."));
            }
            let end = declaration_end(after)
                .ok_or_else(|| document_error(text, at, UNEXPECTED_END))?;
            at += "<!".len() + end + ">".len();
        } else if let Some(after) = rest.strip_prefix("</") {
            let (name, length) = name_at(after);
            let tail = after.get(length..).unwrap_or("");
            let closed = tail.trim_start_matches(is_space);
            if !closed.starts_with('>') {
                return Err(document_error(text, at, UNEXPECTED_END));
            }
            let Some(element) = open.pop() else {
                return Err(document_error(text, at, ROOT_LEVEL));
            };
            if element.name != name {
                let reason = format!(
                    "The '{}' start tag does not match the end tag of '{name}'.",
                    element.name
                );
                return Err(document_error(text, at, reason));
            }
            at += "</".len() + length + (tail.len() - closed.len()) + ">".len();
            attach(&mut open, &mut root, element);
        } else if let Some(after) = rest.strip_prefix('<') {
            if open.is_empty() && root.is_some() {
                return Err(document_error(
                    text,
                    at,
                    "There are multiple root elements.",
                ));
            }
            if open.len() >= DEPTH_MAX {
                return Err(document_error(
                    text,
                    at,
                    "The document's elements are nested too deeply.",
                ));
            }
            let (element, empty, length) = start_tag(after, at)
                .map_err(|(offset, reason)| document_error(text, at + 1 + offset, reason))?;
            at += "<".len() + length;
            if empty {
                attach(&mut open, &mut root, element);
            } else {
                open.push(element);
            }
        } else {
            let Some(parent) = open.last_mut() else {
                return Err(document_error(text, at, ROOT_LEVEL));
            };
            let end = rest.find('<').unwrap_or(rest.len());
            let raw = rest.get(..end).unwrap_or("");
            let decoded = unescape(raw).map_err(|reason| document_error(text, at, reason))?;
            parent.content.push(Node::Text(decoded));
            at += end;
        }
    }
    if let Some(element) = open.last() {
        let reason = format!(
            "{UNEXPECTED_END} The following elements are not closed: {}.",
            element.name
        );
        return Err(document_error(text, text.len(), reason));
    }
    root.ok_or_else(|| document_error(text, at, "Root element is missing."))
}

/// An element closed: into its parent, or the document's root.
fn attach(open: &mut [Element], root: &mut Option<Element>, element: Element) {
    match open.last_mut() {
        Some(parent) => parent.content.push(Node::Element(element)),
        None => *root = Some(element),
    }
}

/// The name at the start of `tag`, and its length in bytes: up to white space or `/`, `>`, `=`,
/// `<` or a quote.
fn name_at(tag: &str) -> (&str, usize) {
    let length = tag
        .find(|c: char| is_space(c) || matches!(c, '/' | '>' | '=' | '<' | '"' | '\''))
        .unwrap_or(tag.len());
    (tag.get(..length).unwrap_or(""), length)
}

/// A start tag after its `<`, the `<` at `at`: the element it opens, whether it closes itself
/// (`<name />`), and the tag's length; or where in it, and why, it is not well formed. Of its
/// attributes only a default namespace declaration is kept.
fn start_tag(tag: &str, at: usize) -> Result<(Element, bool, usize), (usize, String)> {
    let (name, mut used) = name_at(tag);
    if name.is_empty() {
        return Err((0, NAME_START.to_owned()));
    }
    let mut element = Element {
        name: name.to_owned(),
        namespace: None,
        content: Vec::new(),
        at,
    };
    loop {
        let rest = tag.get(used..).unwrap_or("");
        let attribute = rest.trim_start_matches(is_space);
        used += rest.len() - attribute.len();
        if attribute.starts_with("/>") {
            return Ok((element, true, used + "/>".len()));
        }
        if attribute.starts_with('>') {
            return Ok((element, false, used + ">".len()));
        }
        if attribute.is_empty() {
            return Err((used, UNEXPECTED_END.to_owned()));
        }
        let (key, length) = name_at(attribute);
        if key.is_empty() {
            return Err((used, NAME_START.to_owned()));
        }
        let after_key = attribute.get(length..).unwrap_or("");
        let Some(after_equals) = after_key.trim_start_matches(is_space).strip_prefix('=') else {
            return Err((used + length, "'=' is expected.".to_owned()));
        };
        let quoted = after_equals.trim_start_matches(is_space);
        let Some(quote) = quoted.chars().next().filter(|c| matches!(c, '"' | '\'')) else {
            return Err((used + length, "A quote is expected.".to_owned()));
        };
        let inner = quoted.get(1..).unwrap_or("");
        let Some(end) = inner.find(quote) else {
            return Err((used, UNEXPECTED_END.to_owned()));
        };
        let raw = inner.get(..end).unwrap_or("");
        if raw.contains('<') {
            return Err((used, "'<' is an invalid attribute character.".to_owned()));
        }
        let value = unescape(raw).map_err(|reason| (used, reason))?;
        if key == "xmlns" {
            element.namespace = Some(value);
        }
        // The attribute, through its closing quote.
        used += (attribute.len() - quoted.len()) + 1 + end + 1;
    }
}

/// Where a declaration after `<!` ends: its `>`, past an internal subset's brackets.
fn declaration_end(declaration: &str) -> Option<usize> {
    let mut depth = 0_usize;
    for (index, c) in declaration.char_indices() {
        match c {
            '[' => depth += 1,
            ']' => depth = depth.saturating_sub(1),
            '>' if depth == 0 => return Some(index),
            _ => {}
        }
    }
    None
}

/// Text with its entities decoded: the five predefined and character references, decimal or
/// hexadecimal. Any other is an entity no document type here declares.
fn unescape(raw: &str) -> Result<String, String> {
    let mut out = String::with_capacity(raw.len());
    let mut rest = raw;
    while let Some(amp) = rest.find('&') {
        out.push_str(rest.get(..amp).unwrap_or(""));
        let after = rest.get(amp + 1..).unwrap_or("");
        let Some(semicolon) = after.find(';') else {
            return Err("An error occurred while parsing EntityName.".to_owned());
        };
        let entity = after.get(..semicolon).unwrap_or("");
        let decoded = match entity {
            "lt" => Some('<'),
            "gt" => Some('>'),
            "amp" => Some('&'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ => entity
                .strip_prefix("#x")
                .and_then(|hex| u32::from_str_radix(hex, 16).ok())
                .or_else(|| {
                    entity
                        .strip_prefix('#')
                        .filter(|digits| digits.chars().all(|c| c.is_ascii_digit()))
                        .and_then(|digits| digits.parse().ok())
                })
                .and_then(char::from_u32),
        };
        let Some(c) = decoded else {
            return Err(format!("Reference to undeclared entity '{entity}'."));
        };
        out.push(c);
        rest = after.get(semicolon + 1..).unwrap_or("");
    }
    out.push_str(rest);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A file as Mission Planner writes one, made by hand from `GridData.cs` and `XmlSerializer`'s
    /// rules until the oracle writes one: CR LF line ends, as the .NET Framework writes them on
    /// Windows.
    const FIXTURE: &str = include_str!("../../../testdata/grid/file/square.grid");

    fn dec(text: &str) -> Decimal {
        Decimal::parse(text).expect("a decimal")
    }

    /// The fixture, member by member.
    #[test]
    fn the_fixture_reads_member_by_member() {
        let data = GridData::from_xml(FIXTURE).expect("the fixture");
        assert_eq!(
            data.poly,
            [
                PointLatLngAlt::new(-35.36, 149.16),
                PointLatLngAlt::new(-35.36, 149.171),
                PointLatLngAlt::new(-35.369, 149.171),
                PointLatLngAlt::new(-35.369, 149.16),
            ]
        );
        assert_eq!(data.camera, "Canon SX230 HS");
        assert_eq!(data.startfrom, "BottomLeft");
        let numbers = [
            ("alt", data.alt, "120"),
            ("angle", data.angle, "45"),
            ("speed", data.speed, "8.5"),
            ("splitmission", data.splitmission, "1"),
            ("dist", data.dist, "34.56"),
            ("overshoot1", data.overshoot1, "5"),
            ("overshoot2", data.overshoot2, "0"),
            ("leadin", data.leadin, "0"),
            ("leadin2", data.leadin2, "10"),
            ("overlap", data.overlap, "70"),
            ("sidelap", data.sidelap, "65.5"),
            ("spacing", data.spacing, "12.8"),
            ("copter_delay", data.copter_delay, "0.5"),
            ("copter_headinghold", data.copter_headinghold, "45"),
            ("minlaneseparation", data.minlaneseparation, "0"),
            ("clockwiseLaps", data.clockwise_laps, "1"),
            ("laps", data.laps, "200"),
            ("repeatservo_no", data.repeatservo_no, "7"),
            ("repeatservo_pwm", data.repeatservo_pwm, "1500"),
            ("repeatservo_cycle", data.repeatservo_cycle, "3"),
            ("setservo_no", data.setservo_no, "9"),
            ("setservo_low", data.setservo_low, "1000"),
            ("setservo_high", data.setservo_high, "2000"),
        ];
        for (name, value, text) in numbers {
            assert_eq!(value.to_text(), text, "{name}");
        }
        let flags = [
            ("camdir", data.camdir, true),
            ("usespeed", data.usespeed, true),
            ("autotakeoff", data.autotakeoff, true),
            ("autotakeoff_RTL", data.autotakeoff_rtl, false),
            ("internals", data.internals, false),
            ("footprints", data.footprints, true),
            ("advanced", data.advanced, true),
            ("crossgrid", data.crossgrid, false),
            ("spiral", data.spiral, false),
            ("copter_headinghold_chk", data.copter_headinghold_chk, true),
            ("copter_spline", data.copter_spline, true),
            ("alternateLanes", data.alternate_lanes, false),
            ("matchPerimeter", data.match_perimeter, false),
            ("trigdist", data.trigdist, false),
            ("digicam", data.digicam, true),
            ("repeatservo", data.repeatservo, false),
            ("breaktrigdist", data.breaktrigdist, true),
        ];
        for (name, value, wanted) in flags {
            assert_eq!(value, wanted, "{name}");
        }
        // Every member checked: the two strings, the numbers and the flags.
        assert_eq!(numbers.len() + flags.len() + 2, MEMBER_COUNT);
    }

    /// What this writes of the fixture's values is the fixture, byte for byte, but for the line
    /// ends, which are the platform's (`Environment.NewLine`).
    #[test]
    fn the_fixture_is_what_this_writes() {
        let data = GridData::from_xml(FIXTURE).expect("the fixture");
        let expected = FIXTURE.replace("\r\n", "\n").replace('\n', NEW_LINE);
        assert_eq!(data.to_xml(), expected);
        assert!(FIXTURE.contains("\r\n"), "the .NET Framework's line ends");
        assert!(!FIXTURE.ends_with('\n'), "XmlSerializer ends on the root's end tag");
    }

    /// The writer's member names are `GridData.cs`'s, in its order, and the reader takes each of
    /// them: a file of every member set reads back to the same data.
    #[test]
    fn every_member_is_written_and_read_by_its_name() {
        let names: Vec<&str> = GridData::default()
            .members()
            .iter()
            .map(|(name, _)| *name)
            .collect();
        assert_eq!(names, MEMBERS);
        let data = GridData {
            poly: vec![
                PointLatLngAlt {
                    lat: -35.36 - 1.0 / 7.0,
                    lng: 149.16 + 2.0 / 9.0,
                    alt: 3.5,
                    tag: "S".to_owned(),
                    tag2: "x & y".to_owned(),
                },
                PointLatLngAlt::new(1.0 / 3.0, 0.1 + 0.2),
            ],
            camera: "Mine <A & B>".to_owned(),
            alt: dec("1"),
            angle: dec("2.5"),
            camdir: true,
            speed: dec("3.0"),
            usespeed: true,
            autotakeoff: true,
            autotakeoff_rtl: true,
            splitmission: dec("4"),
            internals: true,
            footprints: true,
            advanced: true,
            dist: dec("5.25"),
            overshoot1: dec("-6"),
            overshoot2: dec("7"),
            leadin: dec("-8"),
            leadin2: dec("9"),
            startfrom: "TopRight".to_owned(),
            overlap: dec("10.5"),
            sidelap: dec("11"),
            spacing: dec("12"),
            crossgrid: true,
            spiral: true,
            copter_delay: dec("13.1"),
            copter_headinghold_chk: true,
            copter_headinghold: dec("14"),
            copter_spline: true,
            alternate_lanes: true,
            minlaneseparation: dec("15"),
            clockwise_laps: dec("-1"),
            laps: dec("17"),
            match_perimeter: true,
            trigdist: true,
            digicam: true,
            repeatservo: true,
            breaktrigdist: true,
            repeatservo_no: dec("18"),
            repeatservo_pwm: dec("19"),
            repeatservo_cycle: dec("20"),
            setservo_no: dec("21"),
            setservo_low: dec("22"),
            setservo_high: dec("23"),
        };
        let xml = data.to_xml();
        let read = GridData::from_xml(&xml).expect("its own file");
        assert_eq!(read, data);
        assert_eq!(read.to_xml(), xml, "the scales kept: 3.0 stays 3.0");
        for written in [
            "<speed>3.0</speed>",
            "<camera>Mine &lt;A &amp; B&gt;</camera>",
            "<Tag>S</Tag>",
            "<Tag2>x &amp; y</Tag2>",
            "<Alt>3.5</Alt>",
            "<Lat>0.33333333333333331</Lat>",
            "<Lng>0.30000000000000004</Lng>",
            "<autotakeoff_RTL>true</autotakeoff_RTL>",
            "<clockwiseLaps>-1</clockwiseLaps>",
        ] {
            assert!(xml.contains(written), "{written}");
        }
    }

    /// `default(GridData)`: an empty list closed short, an empty string likewise, zeros and
    /// falses - and the root with `XmlSerializer`'s two namespaces.
    #[test]
    fn nothing_set_is_written_as_the_serializer_writes_it() {
        let xml = GridData::default().to_xml();
        let expected_start = [
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>",
            "<GridData xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
             xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">",
            "  <poly />",
            "  <camera />",
            "  <alt>0</alt>",
            "  <angle>0</angle>",
            "  <camdir>false</camdir>",
        ]
        .join(NEW_LINE);
        assert!(xml.starts_with(&expected_start), "{xml}");
        assert!(xml.ends_with(&format!("{NEW_LINE}</GridData>")));
        assert_eq!(GridData::from_xml(&xml), Ok(GridData::default()));
    }

    /// `XmlConvert.ToString(double)`: the .NET Framework's `"R"` - fifteen digits when they read
    /// back, seventeen when they do not (where the shortest would be sixteen: 1/3) - and the
    /// names; whatever it writes reads back as the same double.
    #[test]
    fn a_double_is_written_round_trip_as_the_framework_writes_it() {
        assert_eq!(xml_double_text(-35.36), "-35.36");
        assert_eq!(xml_double_text(149.171), "149.171");
        assert_eq!(xml_double_text(0.1 + 0.2), "0.30000000000000004");
        assert_eq!(xml_double_text(1.0 / 3.0), "0.33333333333333331");
        assert_eq!(xml_double_text(0.0001), "0.0001");
        assert_eq!(xml_double_text(0.00001), "1E-05");
        assert_eq!(xml_double_text(1e20), "1E+20");
        assert_eq!(xml_double_text(0.0), "0");
        assert_eq!(xml_double_text(-0.0), "-0");
        assert_eq!(xml_double_text(f64::INFINITY), "INF");
        assert_eq!(xml_double_text(f64::NEG_INFINITY), "-INF");
        assert_eq!(xml_double_text(f64::NAN), "NaN");
        for value in [
            -35.36 - 1.0 / 7.0,
            149.16 + 2.0 / 9.0,
            1.0 / 3.0,
            2.0 / 3.0,
            1e-7,
            123_456.789,
            -1e-5 / 3.0,
            1e22 / 3.0,
        ] {
            let text = xml_double_text(value);
            assert_eq!(xml_double(&text), Some(value), "{text}");
        }
    }

    /// `XmlConvert`'s readers: white space either side; `ToDecimal` keeps the scale and takes no
    /// separator, trailing sign or exponent; `ToBoolean` is lower case or a digit; `ToDouble`
    /// takes an exponent and the names.
    #[test]
    fn values_are_read_as_xml_convert_reads_them() {
        let decimal = |text: &str| xml_decimal(text).map(Decimal::to_text);
        assert_eq!(decimal(" 12.50 ").as_deref(), Some("12.50"));
        assert_eq!(decimal("+3").as_deref(), Some("3"));
        assert_eq!(decimal("-0.5").as_deref(), Some("-0.5"));
        for refused in ["1e3", "1,000", "5-", "", ".", "ten", "1.2.3"] {
            assert_eq!(decimal(refused), None, "{refused:?}");
        }
        assert_eq!(xml_bool(" true "), Some(true));
        assert_eq!(xml_bool("1"), Some(true));
        assert_eq!(xml_bool("0"), Some(false));
        assert_eq!(xml_bool("false"), Some(false));
        assert_eq!(xml_bool("True"), None, "bool.ToString()'s case is not XML's");
        assert_eq!(xml_bool("yes"), None);
        assert_eq!(xml_double(" -35.36 "), Some(-35.36));
        assert_eq!(xml_double("1E-05"), Some(0.00001));
        assert_eq!(xml_double("INF"), Some(f64::INFINITY));
        assert_eq!(xml_double("-INF"), Some(f64::NEG_INFINITY));
        assert!(xml_double("NaN").is_some_and(f64::is_nan));
        for refused in ["inf", "1,5", "", "0x10", "1.5f"] {
            assert_eq!(xml_double(refused), None, "{refused:?}");
        }
    }

    /// The serializer's reader: members in any order; a name it does not know, with whatever is
    /// in it, passed over; a member's second passed over; one missing at its default; a corner's
    /// own members likewise, `color` with anything in it; comments, CDATA and references.
    #[test]
    fn the_reader_takes_members_by_name() {
        let xml = concat!(
            "\u{feff}<?xml version=\"1.0\"?>\n<!-- a grid -->\n<GridData>",
            "<laps>5</laps><unknown><deep>1</deep></unknown>",
            "<alt> 7 </alt><alt>8</alt><camdir>1</camdir>",
            "<camera>a&#x41;&#66;&lt;<![CDATA[<x>]]></camera>",
            "<poly><PointLatLngAlt><Lng>2</Lng><Lat>1</Lat><color><R>9</R></color><Extra/>",
            "</PointLatLngAlt><Other/></poly></GridData>",
        );
        let data = GridData::from_xml(xml).expect("a grid");
        assert_eq!(data.laps.to_text(), "5");
        assert_eq!(data.alt.to_text(), "7", "the first of two");
        assert!(data.camdir);
        assert_eq!(data.camera, "aAB<<x>");
        assert_eq!(data.poly, [PointLatLngAlt::new(1.0, 2.0)]);
        assert_eq!(data.speed.to_text(), "0", "missing: default(decimal)");
        assert_eq!(data.startfrom, "");
        assert!(!data.trigdist);
    }

    /// What `Deserialize` throws for: not well formed, another root, a root in a namespace, a
    /// value `XmlConvert` refuses - each with its line and column.
    #[test]
    fn what_is_not_a_grid_says_where() {
        let error = |xml: &str| GridData::from_xml(xml).expect_err("not a grid");
        assert!(matches!(
            error("<Grid></Grid>"),
            GridFileError::Document { line: 1, column: 1, reason }
                if reason == "<Grid xmlns=''> was not expected."
        ));
        assert!(matches!(
            error("<GridData xmlns=\"urn:x\"/>"),
            GridFileError::Document { reason, .. } if reason.contains("was not expected")
        ));
        assert!(matches!(
            error("<GridData>\n  <alt>ten</alt>\n</GridData>"),
            GridFileError::Document { line: 2, column: 3, reason } if reason == FORMAT
        ));
        assert!(matches!(
            error("<GridData>\r\n  <camdir>True</camdir>\r\n</GridData>"),
            GridFileError::Document { line: 2, column: 3, .. }
        ));
        assert!(matches!(
            error(concat!(
                "<GridData><poly><PointLatLngAlt><Lat>north</Lat>",
                "</PointLatLngAlt></poly></GridData>"
            )),
            GridFileError::Document { reason, .. } if reason == FORMAT
        ));
        assert!(matches!(
            error("<GridData><alt><b>1</b></alt></GridData>"),
            GridFileError::Document { reason, .. } if reason == SIMPLE_CONTENT
        ));
        for broken in [
            "",
            "hello",
            "<GridData>",
            "<GridData><alt>1</GridData>",
            "<GridData/><GridData/>",
            "<GridData><camera>&nbsp;</camera></GridData>",
            "<GridData attribute></GridData>",
        ] {
            assert!(
                matches!(
                    GridData::from_xml(broken),
                    Err(GridFileError::Document { .. })
                ),
                "{broken:?}"
            );
        }
        let deep = format!(
            "<GridData>{}{}</GridData>",
            "<a>".repeat(DEPTH_MAX * 4),
            "</a>".repeat(DEPTH_MAX * 4)
        );
        assert!(matches!(
            GridData::from_xml(&deep),
            Err(GridFileError::Document { .. })
        ));
    }

    /// `StreamReader`'s byte order marks: UTF-8's dropped, UTF-16's read in either order.
    #[test]
    fn a_byte_order_mark_picks_the_encoding() {
        let xml = "<GridData><camera>\u{dc}</camera></GridData>";
        let mut utf8 = vec![0xEF, 0xBB, 0xBF];
        utf8.extend_from_slice(xml.as_bytes());
        let mut little = vec![0xFF, 0xFE];
        let mut big = vec![0xFE, 0xFF];
        for unit in xml.encode_utf16() {
            little.extend_from_slice(&unit.to_le_bytes());
            big.extend_from_slice(&unit.to_be_bytes());
        }
        for bytes in [xml.as_bytes().to_vec(), utf8, little, big] {
            let data = GridData::from_xml(&decode(&bytes)).expect("a grid");
            assert_eq!(data.camera, "\u{dc}");
        }
    }

    /// `SaveGrid`'s file read back by `LoadGrid`, on disk.
    #[test]
    fn a_file_written_reads_back() {
        let dir = mp_os::temp_dir().join(format!("mp-mission-grid-file-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("square.grid");
        let data = GridData::from_xml(FIXTURE).expect("the fixture");
        data.write(&path).expect("written");
        let bytes = mp_os::fs::read(&path).unwrap();
        assert!(bytes.starts_with(b"<?xml"), "no byte order mark");
        assert_eq!(GridData::read(&path), Ok(data));
        assert!(matches!(
            GridData::read(&dir.join("missing.grid")),
            Err(GridFileError::Io(_))
        ));
        let _ = mp_os::fs::remove_dir_all(&dir);
    }
}
