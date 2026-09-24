//! Missions: waypoint items and the file format users already have on disk.
//!
//! Replaces the mission half of `GCSViews/FlightPlanner.cs` and
//! `ExtLibs/Utilities/MissionFile.cs`.

#![forbid(unsafe_code)]

pub mod cameras;
pub mod circle;
pub mod circle_survey;
mod clipper;
pub mod commands;
pub mod corridor;
pub mod dbf;
pub mod dotnet;
pub mod fence;
pub mod fence_file;
pub mod geoutility;
pub mod grid;
pub mod gridui;
pub mod item;
pub mod polygon;
pub mod rotary;
pub mod rows;
pub mod shapefile;
pub mod survey;
pub mod text_mission;
mod utm;
pub mod validate;
pub mod waypoints;
pub mod wire;

pub use corridor::{CorridorArgs, create_corridor};
pub use fence::{FenceError, FenceItem, RallyPoint, fences_from_items};
pub use grid::{GridArgs, GridPoint, GridTag, StartPosition, create_grid};
pub use item::{MissionItem, MissionItemError};
pub use rotary::{RotaryArgs, RotaryError, create_rotary};
pub use survey::{GridError, GridOptions, grid};
pub use validate::{Finding, Severity, validate};
pub use waypoints::{WaypointFileError, read_waypoints, write_waypoints};
pub use wire::{MISSION_TYPE_MISSION, WireItem};
pub mod utm_grid;
