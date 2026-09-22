//! Missions: waypoint items and the file format users already have on disk.
//!
//! Replaces the mission half of `GCSViews/FlightPlanner.cs` and
//! `ExtLibs/Utilities/MissionFile.cs`.

#![forbid(unsafe_code)]

pub mod fence;
pub mod item;
pub mod survey;
pub mod waypoints;
pub mod wire;

pub use fence::{FenceError, FenceItem, RallyPoint, fences_from_items};
pub use item::{MissionItem, MissionItemError};
pub use survey::{GridError, GridOptions, grid};
pub use waypoints::{WaypointFileError, read_waypoints, write_waypoints};
pub use wire::{MISSION_TYPE_MISSION, WireItem};
