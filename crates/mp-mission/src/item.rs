//! A single mission item.

use mp_units::{LatLon, PositionError};

/// `MAV_CMD_NAV_WAYPOINT`, the default command.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;

/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`, the frame almost every mission uses: altitude above home.
pub const MAV_FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;

/// `MAV_FRAME_GLOBAL`, altitude above mean sea level. The home item uses this.
pub const MAV_FRAME_GLOBAL: u8 = 0;

/// One command in a mission.
///
/// Deliberately close to the wire format rather than to a tidier model: a mission editor has to
/// round-trip commands it does not understand, and a struct that only holds the fields we happen
/// to support would quietly discard the rest of someone's flight plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItem {
    /// Position in the mission, zero-based. Item 0 is conventionally home.
    pub seq: u16,
    /// Whether this is the active item. Only ever 1 on the home item in a file.
    pub current: u8,
    /// `MAV_FRAME` the coordinates are expressed in.
    pub frame: u8,
    /// `MAV_CMD` to execute.
    pub command: u16,
    /// Command-specific parameter 1.
    pub param1: f64,
    /// Command-specific parameter 2.
    pub param2: f64,
    /// Command-specific parameter 3.
    pub param3: f64,
    /// Command-specific parameter 4.
    pub param4: f64,
    /// Latitude in degrees, or a command-specific value for non-navigation commands.
    pub x: f64,
    /// Longitude in degrees, or a command-specific value.
    pub y: f64,
    /// Altitude in metres, in the item's frame.
    pub z: f64,
    /// Whether the vehicle continues to the next item automatically.
    pub autocontinue: u8,
}

/// Why a mission item is not usable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum MissionItemError {
    /// The coordinates are not a valid position.
    #[error("item {seq} has an invalid position")]
    Position {
        /// Which item.
        seq: u16,
    },
}

impl Default for MissionItem {
    fn default() -> Self {
        Self {
            seq: 0,
            current: 0,
            frame: MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command: MAV_CMD_NAV_WAYPOINT,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            autocontinue: 1,
        }
    }
}

impl MissionItem {
    /// Whether this command navigates somewhere, and therefore whether x and y are coordinates.
    ///
    /// The distinction matters: `MAV_CMD_DO_SET_SERVO` puts a servo number in `param1` and leaves
    /// x and y at zero, and treating that as a position at Null Island puts a phantom waypoint in
    /// the Gulf of Guinea on the map. Navigation commands are the 16..95 range.
    #[must_use]
    pub const fn is_navigation(&self) -> bool {
        self.command >= 16 && self.command <= 95
    }

    /// The position, if this item has one.
    pub fn position(&self) -> Result<Option<LatLon>, PositionError> {
        if !self.is_navigation() || (self.x == 0.0 && self.y == 0.0) {
            return Ok(None);
        }
        LatLon::new(self.x, self.y).map(Some)
    }

    /// Whether this is the home item: sequence zero.
    #[must_use]
    pub const fn is_home(&self) -> bool {
        self.seq == 0
    }
}
