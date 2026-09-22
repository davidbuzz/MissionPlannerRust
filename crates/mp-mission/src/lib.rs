//! Missions: waypoint items and the file format users already have on disk.
//!
//! Replaces the mission half of `GCSViews/FlightPlanner.cs` and
//! `ExtLibs/Utilities/MissionFile.cs`.

#![forbid(unsafe_code)]

pub mod item;
pub mod waypoints;

pub use item::{MissionItem, MissionItemError};
pub use waypoints::{WaypointFileError, read_waypoints, write_waypoints};
