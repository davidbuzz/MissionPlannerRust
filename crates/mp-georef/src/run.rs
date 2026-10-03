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

//! What the form's two buttons do with `GeoRefImageBase`: "Process" (`BUT_doit_Click`) and
//! "GeoTag Images" (`BUT_Geotagimages_Click`), with the controls they read as settings. The form
//! itself - its map, picture box, browse dialogs - is not here; it will call these.
//! `// C#: GeoRef/georefimage.cs:140-319`

use crate::exif_write;
use crate::georef::{GeoRefImageBase, GeorefError, Output, ProcessingMode};
use crate::photos;
use crate::projection::Terrain;
use crate::report::{self, ReportFiles, ReportSettings};

/// The form's controls a run reads.
#[derive(Debug, Clone, PartialEq)]
pub struct FormSettings {
    /// `RDIO_CAMMsgSynchro`, `RDIO_trigmsg` or the time-offset radio button: CAM by default
    /// (`georefimage.cs:35`).
    pub mode: ProcessingMode,
    /// `TXT_offsetseconds`, parsed only in time-offset mode.
    pub offset_text: String,
    /// `chk_usegps2`: `GPS2` rather than `GPS` lines.
    pub use_gps2: bool,
    /// `chk_cammsg`: in time-offset mode, the `CAM` messages as the positions.
    pub use_cam_messages: bool,
    /// `num_dropfromstart`.
    pub drop_start: i32,
    /// `num_dropend`.
    pub drop_end: i32,
    /// `num_camerarotation`.
    pub camera_rotation: f64,
    /// `num_hfov`.
    pub hfov: f64,
    /// `num_vfov`.
    pub vfov: f64,
    /// `chk_camusegpsalt`: in CAM mode, the GPS altitude.
    pub cam_use_gps_alt: bool,
    /// `chk_trigusergpsalt`: in TRIG mode, the GPS altitude.
    pub trig_use_gps_alt: bool,
    /// `txt_basealt`: added to the AMSL altitude written into geotagged photos.
    pub base_alt_text: String,
}

impl Default for FormSettings {
    /// The form's defaults (`Georefimage.Designer.cs`, `georefimage.resx`).
    fn default() -> Self {
        Self {
            mode: ProcessingMode::CamMsg,
            offset_text: "0".to_owned(),
            use_gps2: false,
            use_cam_messages: false,
            drop_start: 0,
            drop_end: 0,
            camera_rotation: 90.0,
            hfov: 200.0,
            vfov: 130.0,
            cam_use_gps_alt: false,
            trig_use_gps_alt: false,
            base_alt_text: "0".to_owned(),
        }
    }
}

impl FormSettings {
    /// `UseGpsorGPS2`. `// C#: GeoRef/georefimage.cs:64-70`
    #[must_use]
    pub const fn gps(&self) -> &'static str {
        if self.use_gps2 { "GPS2" } else { "GPS" }
    }
}

/// `float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture)`.
#[must_use]
pub fn parse_offset(text: &str) -> Option<f32> {
    let value = mp_log::netfmt::parse_double(text)?;
    // NumberStyles.Float has no thousands separator.
    if text.contains(',') {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)]
    let single = value as f32;
    (single.is_finite() || !value.is_finite()).then_some(single)
}

/// The shutter lag the form takes from its text box: `int.TryParse` in the invariant culture,
/// and 0 - with the box reset to "0" - for anything else.
/// `// C#: GeoRef/georefimage.cs:360-367`
#[must_use]
pub fn parse_shutter_lag(text: &str) -> i32 {
    mp_log::netfmt::parse_i32(text).unwrap_or(0)
}

/// What choosing a photo folder puts in the offset box: the digits after the first
/// `seconds_offset: ` followed by a digit in that folder's `location.txt` (the regular expression
/// `seconds_offset: ([0-9]+)`), if the file is there. Nothing this port or the C# writes has that
/// text - `location.tel` says `#seconds offset - ` - so it answers only for files from elsewhere.
/// `// C#: GeoRef/georefimage.cs:111-137`
#[must_use]
pub fn offset_from_location_txt(dir: &str) -> Option<String> {
    let text = std::fs::read_to_string(format!("{dir}{}location.txt", photos::SEPARATOR)).ok()?;
    const KEY: &str = "seconds_offset: ";
    let mut rest = text.as_str();
    while let Some(at) = rest.find(KEY) {
        let after = rest.get(at + KEY.len()..).unwrap_or_default();
        let digits: String = after.chars().take_while(char::is_ascii_digit).collect();
        if !digits.is_empty() {
            return Some(digits);
        }
        rest = rest.get(at + 1..).unwrap_or_default();
    }
    None
}

impl GeoRefImageBase {
    /// What the form clears when the log changes (the log box's `TextChanged`: the positions and
    /// the pictures) or the positions it wants do (the GPS2 and CAM check boxes'
    /// `CheckedChanged`: the positions), so the next run reads them again.
    /// `// C#: GeoRef/georefimage.cs:327-335, 376-386`
    pub fn forget_positions(&mut self, and_pictures: bool) {
        self.vehicle_locations.clear();
        if and_pictures && let Some(pictures) = self.pictures_info.as_mut() {
            pictures.clear();
        }
    }

    /// "Process": the mode's matching, then every report file into `dir_pictures`. Where the C#
    /// throws, the form prints `"Error " + ex` and carries on to draw what it has; so does this.
    /// `picturesInfo` keeps its old value when the matching throws, and is `None` (`null`) when
    /// it returns nothing.
    /// `// C#: GeoRef/georefimage.cs:140-200`
    pub fn process(
        &mut self,
        log_file_path: &str,
        dir_pictures: &str,
        settings: &FormSettings,
        append_text: Output<'_>,
        terrain: &dyn Terrain,
    ) -> Option<ReportFiles> {
        if !std::path::Path::new(log_file_path).is_file()
            || !std::path::Path::new(dir_pictures).is_dir()
        {
            return None;
        }
        let mut seconds = 0.0f32;
        if settings.mode == ProcessingMode::TimeOffset {
            match parse_offset(&settings.offset_text) {
                Some(s) => seconds = s,
                None => {
                    append_text(
                        "Offset number not in correct format. Use . as decimal separator\n",
                    );
                    return None;
                }
            }
        }
        let gps = settings.gps();
        let result: Result<Option<ReportFiles>, GeorefError> = (|| {
            let (pictures, usegpsalt) = match settings.mode {
                ProcessingMode::TimeOffset => (
                    self.dowork_gps_offset(
                        log_file_path,
                        dir_pictures,
                        seconds,
                        gps,
                        settings.use_cam_messages,
                        &mut *append_text,
                    )?,
                    false,
                ),
                ProcessingMode::CamMsg => (
                    self.dowork_cam(
                        log_file_path,
                        dir_pictures,
                        gps,
                        &mut *append_text,
                        settings.drop_start,
                        settings.drop_end,
                    )?,
                    settings.cam_use_gps_alt,
                ),
                ProcessingMode::Trig => (
                    self.dowork_trig(
                        log_file_path,
                        dir_pictures,
                        gps,
                        &mut *append_text,
                        settings.drop_start,
                        settings.drop_end,
                    )?,
                    settings.trig_use_gps_alt,
                ),
            };
            self.pictures_info = pictures;
            let Some(pictures) = self.pictures_info.clone() else {
                return Ok(None);
            };
            let files = self.create_report_files(
                &pictures,
                &ReportSettings {
                    offset: seconds,
                    camera_rotation: settings.camera_rotation,
                    hfov: settings.hfov,
                    vfov: settings.vfov,
                    usegpsalt,
                },
                &mut *append_text,
                terrain,
            );
            report::write_report_files(dir_pictures, &files).map_err(|e| GeorefError::Io {
                path: dir_pictures.to_owned(),
                message: e.to_string(),
            })?;
            Ok(Some(files))
        })();
        match result {
            Ok(files) => files,
            Err(e) => {
                append_text(&format!("Error {e}"));
                None
            }
        }
    }

    /// "GeoTag Images": the `geotagged` folder emptied, then each matched photo's copy with the
    /// altitude the options pick - the GPS one in CAM or TRIG mode when asked, else AMSL plus the
    /// base altitude, else relative.
    /// `// C#: GeoRef/georefimage.cs:268-319`
    ///
    /// # Errors
    ///
    /// Where the C# throws out of the handler: the folder cannot be made, or the base altitude is
    /// not a number (`double.Parse`, uncaught).
    pub fn geotag_images(
        &self,
        root_folder: &str,
        settings: &FormSettings,
        append_text: Output<'_>,
    ) -> Result<(), GeorefError> {
        let folder = photos::geotag_folder(root_folder);
        let io = |e: std::io::Error| GeorefError::Io {
            path: folder.to_string_lossy().into_owned(),
            message: e.to_string(),
        };
        if folder.exists() {
            std::fs::remove_dir_all(&folder).map_err(io)?;
        }
        std::fs::create_dir_all(&folder).map_err(io)?;
        let Some(pictures) = &self.pictures_info else {
            append_text("no valid matchs");
            return Ok(());
        };
        for pic in pictures.values() {
            let l = &pic.location;
            let gps = (settings.cam_use_gps_alt && settings.mode == ProcessingMode::CamMsg)
                || (settings.trig_use_gps_alt && settings.mode == ProcessingMode::Trig);
            let alt = if gps {
                l.gps_alt
            } else if self.use_amsl_alt {
                let base =
                    mp_log::netfmt::parse_double(&settings.base_alt_text).ok_or_else(|| {
                        GeorefError::BadLog("Input string was not in a correct format.".to_owned())
                    })?;
                base + l.alt_amsl
            } else {
                l.rel_alt
            };
            exif_write::write_coordinates_to_image(
                &pic.path,
                l.lat,
                l.lon,
                alt,
                root_folder,
                &mut *append_text,
            );
        }
        append_text("GeoTagging FINISHED \n\n");
        Ok(())
    }
}
