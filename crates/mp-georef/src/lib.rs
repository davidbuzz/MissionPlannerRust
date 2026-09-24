//! Geo Reference Images: Mission Planner's `GeoRefImageBase` (`ExtLibs/Utilities/
//! GeoRefImageBase.cs`, GPL-3.0-or-later) and the logic of the "Geo Reference Images" form's
//! handlers (`GeoRef/georefimage.cs`), without the form.
//!
//! - [`dataflash`], [`tlog`]: the log readers the C# uses (`DFLogBuffer` with its `gpsstarttime`
//!   side effects, `MavlinkParse.ReadPacket`).
//! - [`exif_read`]: the photo times (`MetadataExtractor` 2.4.0's first APP1, `GetDate`).
//! - [`georef`]: the three modes (`doworkGPSOFFSET`, `doworkCAM`, `doworkTRIG`), `EstimateOffset`,
//!   the log readers for GPS, CAM, TRIG, ATT and `CAMERA_FEEDBACK`.
//! - [`report`], [`kml`], [`xml`], [`numfmt`]: every file "Process" writes, byte for byte.
//! - [`exif_write`]: `WriteCoordinatesToImage` over the `ExifLibrary` (ExifLibNet 2.1.3) JPEG
//!   reader and writer it uses, written here; no EXIF crate is in the lock file.
//! - [`projection`]: `ImageProjection`'s footprint over terrain.
//! - [`run`]: the form's buttons and text boxes as functions.
//!
//! What a form over this needs: a log file box and a folder box (with the offset
//! [`run::offset_from_location_txt`] offers when a folder is chosen), the mode radio buttons and
//! the controls [`FormSettings`] holds, a text box the runs append to, the buttons calling
//! [`GeoRefImageBase::estimate_offset`], [`GeoRefImageBase::process`] and
//! [`GeoRefImageBase::geotag_images`], [`GeoRefImageBase::forget_positions`] on the log box's
//! and the GPS2 and CAM check boxes' changes, and a map drawing [`ReportFiles::kml`] and the
//! footprints. The progress bar and picture box have no logic here.

#![forbid(unsafe_code)]

pub mod dataflash;
pub mod exif_read;
pub mod exif_write;
pub mod georef;
pub mod kml;
pub mod location;
pub mod numfmt;
pub mod photos;
pub mod projection;
pub mod report;
pub mod run;
pub mod time;
pub mod tlog;
pub mod xml;

pub use georef::{GeoRefImageBase, GeorefError, ProcessingMode};
pub use location::{Location, OrderedMap, PictureInformation};
pub use projection::{Flat, Terrain};
pub use report::{ReportFiles, ReportSettings};
pub use run::FormSettings;
pub use time::DateTime;
