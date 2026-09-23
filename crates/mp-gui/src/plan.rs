//! The plan screen: building a mission and moving it to and from the aircraft.
//!
//! Equivalent to Mission Planner's Flight Plan tab. The plan held here is the operator's, and it
//! is deliberately distinct from whatever the vehicle holds: the two are only equal immediately
//! after a read or a successful write, and pretending otherwise is how people fly a mission they
//! thought they had replaced.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_mavlink_dialects::all::MavCmd;
use mp_mission::fence::{FenceItem, RallyPoint};
use mp_mission::rows::Home;
use mp_mission::validate::{Context as ValidationContext, validate_with};
use mp_mission::{GridOptions, MissionItem, Severity, grid};
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::textfield::TextField;
use crate::ui::{action, panel, progress, theme};

/// `MAV_CMD_NAV_WAYPOINT`, the command a click on the map creates.
pub const CMD_WAYPOINT: u16 = 16;
/// How far the pointer may move between press and release and still count as a click.
///
/// **A divergence from the C#, small and deliberate.** `MainMap_MouseMove` sets `isMouseDraging`
/// on *any* movement - it compares the press position to the current one and returns early only
/// when they are exactly equal, and at any real zoom a one-pixel move is a different latitude. So
/// in Mission Planner, adding a waypoint by clicking requires a click that does not move a single
/// pixel, which is a thing people complain about. Three pixels absorbs a hand on a mouse and is
/// far too small to swallow a deliberate drag. Set to 0.0 to match the original exactly.
pub const CLICK_SLOP: f32 = 3.0;

/// What a release of the left button over the map should do.
///
/// Extracted from the event handler so it can be tested. The decision is four lines of `if`, and
/// four lines of `if` inside a gpui closure is four lines nothing can reach - which is how a rule
/// as load-bearing as "does this click add a waypoint to the mission" ends up verified by looking
/// at a screenshot.
///
/// `// C#: GCSViews/FlightPlanner.cs:7736-7745`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapRelease {
    /// Add a waypoint where the button came up.
    AddWaypoint,
    /// Do nothing: a drag, a grabbed waypoint, or not the planning screen.
    Nothing,
}

/// Decides what a release means.
///
/// `grabbed` is the waypoint the press landed on, if any - `CurentRectMarker` in the C#, and the
/// reason its comment reads "cant add WP in existing rect". `press` is where the button went
/// down, absent if the press was never seen.
#[must_use]
pub fn map_release(
    planning: bool,
    grabbed: Option<u16>,
    press: Option<(f32, f32)>,
    release: (f32, f32),
) -> MapRelease {
    if !planning || grabbed.is_some() {
        return MapRelease::Nothing;
    }
    // No recorded press means the button went down somewhere else and came up here - dragging in
    // from off the map, or a press the window never saw. Adding a waypoint for it would put one
    // wherever a stray release landed.
    let Some((press_x, press_y)) = press else {
        return MapRelease::Nothing;
    };
    let moved =
        (release.0 - press_x).abs() > CLICK_SLOP || (release.1 - press_y).abs() > CLICK_SLOP;
    if moved {
        MapRelease::Nothing
    } else {
        MapRelease::AddWaypoint
    }
}

/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`: altitude above home, which is what every pilot means.
pub const FRAME_RELATIVE: u8 = 3;
/// `MAV_FRAME_GLOBAL`: altitude above mean sea level. What the home item always uses.
pub const FRAME_ABSOLUTE: u8 = 0;
/// `MAV_FRAME_GLOBAL_TERRAIN_ALT`: altitude above the ground beneath the waypoint.
///
/// The one that changes what can be flown. A mission at 50 m relative, planned over flat ground
/// and flown over a hill, is a mission into the hill; the same mission in this frame clears it.
/// It needs terrain data on the vehicle - `TERRAIN_ENABLE`, and either an SD card of SRTM tiles
/// or a ground station feeding `TERRAIN_DATA` - and a vehicle without it will refuse the mission
/// rather than guess.
pub const FRAME_TERRAIN: u8 = 10;

/// Which altitude frame new items are created in.
///
/// Mission Planner's `altmode`, and the same three values:
///
/// ```csharp
/// public enum altmode {
///     Relative = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
///     Absolute = MAVLink.MAV_FRAME.GLOBAL,
///     Terrain  = MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT
/// }
/// ```
///
/// A per-screen choice, not a per-item one: `CMB_altmode` sets the frame a *new* row gets
/// (`FlightPlanner.cs:2347`), and every row keeps its own afterwards. So a mission can mix frames
/// and usually does not.
/// `// C#: GCSViews/FlightPlanner.cs:416-421`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AltitudeFrame {
    /// Above home. What almost every mission uses.
    #[default]
    Relative,
    /// Above mean sea level.
    Absolute,
    /// Above the ground beneath the waypoint.
    Terrain,
}

impl AltitudeFrame {
    /// The `MAV_FRAME` value.
    #[must_use]
    pub const fn mav_frame(self) -> u8 {
        match self {
            Self::Relative => FRAME_RELATIVE,
            Self::Absolute => FRAME_ABSOLUTE,
            Self::Terrain => FRAME_TERRAIN,
        }
    }

    /// The frame a `MAV_FRAME` value names, if it is one of the three a mission uses.
    #[must_use]
    pub const fn from_mav_frame(frame: u8) -> Option<Self> {
        match frame {
            FRAME_RELATIVE => Some(Self::Relative),
            FRAME_ABSOLUTE => Some(Self::Absolute),
            FRAME_TERRAIN => Some(Self::Terrain),
            _ => None,
        }
    }

    /// What to call it on screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Relative => "relative",
            Self::Absolute => "absolute",
            Self::Terrain => "terrain",
        }
    }

    /// The name the settings file stores, and reads back.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Relative => "relative",
            Self::Absolute => "absolute",
            Self::Terrain => "terrain",
        }
    }

    /// The frame a stored name means, defaulting to relative.
    ///
    /// A setting file that has been hand-edited into nonsense gives the frame every mission uses,
    /// rather than refusing to start or silently choosing terrain - which a vehicle without
    /// terrain data will reject at upload time, long after the choice was made.
    #[must_use]
    pub fn from_key(key: &str) -> Self {
        match key {
            "absolute" => Self::Absolute,
            "terrain" => Self::Terrain,
            _ => Self::Relative,
        }
    }

    /// All three, in the order `CMB_altmode` lists them.
    #[must_use]
    pub const fn all() -> [Self; 3] {
        [Self::Relative, Self::Absolute, Self::Terrain]
    }
}

/// A short label for an item's frame, for the mission list.
///
/// A frame the three-value enum does not cover is shown as its number rather than hidden: a
/// mission read back from a vehicle can hold `MAV_FRAME_LOCAL_NED` or anything else, and showing
/// nothing would say the item is relative when it is not.
#[must_use]
pub fn frame_label(frame: u8) -> String {
    AltitudeFrame::from_mav_frame(frame).map_or_else(
        || format!("frame {frame}"),
        |known| known.label().to_owned(),
    )
}
/// Altitude given to a waypoint created by clicking the map, in metres above home.
pub const DEFAULT_ALTITUDE: f64 = 50.0;
/// `MAV_CMD_NAV_RETURN_TO_LAUNCH`, which takes no position.
const CMD_RTL: u16 = 20;
/// `MAV_CMD_NAV_LAND` with no coordinates: land where the vehicle is.
const CMD_LAND_NO_POSITION: u16 = 21;

/// What a click on the map does.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum DrawMode {
    /// Add a waypoint.
    #[default]
    Waypoints,
    /// Add a vertex to the survey area.
    Area,
    /// Add a vertex to the geofence.
    Fence,
    /// Place a rally point.
    Rally,
}

impl DrawMode {
    /// The modes, in the order they are offered.
    pub const ALL: [Self; 4] = [Self::Waypoints, Self::Area, Self::Fence, Self::Rally];

    /// What the button says.
    pub const fn label(self) -> &'static str {
        match self {
            Self::Waypoints => "waypoints",
            Self::Area => "survey area",
            Self::Fence => "geofence",
            Self::Rally => "rally points",
        }
    }

    /// The id a test script clicks it by.
    pub const fn id(self) -> &'static str {
        match self {
            Self::Waypoints => "draw-waypoints",
            Self::Area => "draw-area",
            Self::Fence => "draw-fence",
            Self::Rally => "draw-rally",
        }
    }

    /// What a click does in this mode, said plainly.
    pub const fn hint(self) -> &'static str {
        match self {
            Self::Waypoints => "click the map to add a waypoint; right-click for the menu",
            Self::Area => "click the map to place the survey area's corners",
            Self::Fence => "click the map to place the fence's corners",
            Self::Rally => "click the map to place a rally point",
        }
    }
}

/// The mission being edited, separate from the vehicle's.
#[derive(Debug, Default)]
pub struct Plan {
    /// The rows of the `Commands` grid: waypoint 1 onwards, numbered from 1.
    ///
    /// Home is not one of them. Mission Planner keeps home in the Home Location boxes and puts it
    /// back at item 0 only when a mission is written, so the first thing clicked on an empty map
    /// is waypoint 1 and never home. See `mp_mission::rows`.
    items: Vec<MissionItem>,
    /// Where the current contents came from, for the header line.
    origin: Origin,
    /// The row the operator has selected, by its number, if any.
    selected: Option<u16>,
    /// What a click on the map does.
    draw_mode: DrawMode,
    /// The survey area, in the order its vertices were drawn.
    polygon: Vec<LatLon>,
    /// How the survey should be flown.
    survey: GridOptions,
    /// The geofence the vehicle must stay inside, in the order its vertices were drawn.
    fence: Vec<LatLon>,
    /// Rally points: where the vehicle goes on a failsafe instead of all the way home.
    rally: Vec<RallyPoint>,
    /// Why the rally points could not be sent, if they could not.
    rally_error: Option<String>,
    /// Why the fence could not be built or sent, if it could not.
    fence_error: Option<String>,
    /// Why the last survey could not be generated, if it could not.
    survey_error: Option<String>,
    /// The Home Location boxes, `TXT_homelat`, `TXT_homelng` and `TXT_homealt`: home, as the
    /// planning screen holds it and every write takes it.
    home: HomeBoxes,
    /// `cs.PlannedHomeLocation`: the home the operator set, which the boxes show when the vehicle
    /// has not sent one. Every edit of a box that parses writes through to it.
    planned_home: Home,
}

/// Which of the three Home Location boxes.
///
/// `// C#: GCSViews/FlightPlanner.Designer.cs:320-372; GCSViews/FlightPlanner.resx (panel1)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HomeBox {
    /// `TXT_homelat`, labelled "Lat".
    Lat,
    /// `TXT_homelng`, labelled "Long".
    Lng,
    /// `TXT_homealt`, labelled "ASL".
    Alt,
}

impl HomeBox {
    /// The three, top to bottom as `panel1` stacks them.
    pub const ALL: [Self; 3] = [Self::Lat, Self::Lng, Self::Alt];

    /// The label beside the box: `Label1`, `label2` and `label3` in the `.resx`.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Lat => "Lat",
            Self::Lng => "Long",
            Self::Alt => "ASL",
        }
    }

    /// The id a test script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Lat => "plan-home-lat",
            Self::Lng => "plan-home-lng",
            Self::Alt => "plan-home-alt",
        }
    }
}

/// The three boxes' text, which is what home is on this screen until something parses it.
#[derive(Debug)]
pub struct HomeBoxes {
    lat: TextField,
    lng: TextField,
    alt: TextField,
}

impl HomeBoxes {
    const fn get(&self, which: HomeBox) -> &TextField {
        match which {
            HomeBox::Lat => &self.lat,
            HomeBox::Lng => &self.lng,
            HomeBox::Alt => &self.alt,
        }
    }

    const fn get_mut(&mut self, which: HomeBox) -> &mut TextField {
        match which {
            HomeBox::Lat => &mut self.lat,
            HomeBox::Lng => &mut self.lng,
            HomeBox::Alt => &mut self.alt,
        }
    }
}

impl Default for HomeBoxes {
    /// Empty, as the `.resx` leaves them: no home until one is typed, set or sent.
    fn default() -> Self {
        Self {
            lat: TextField::new(""),
            lng: TextField::new(""),
            alt: TextField::new(""),
        }
    }
}

/// `double.ToString()`: the shortest text that reads back as the same number, which is what the
/// C# puts in a box when it sets one from a number.
#[must_use]
pub fn double_text(value: f64) -> String {
    format!("{value}")
}

/// `ToString("0.00")`: two decimals, the half rounded away from zero as .NET's custom formats do.
#[must_use]
pub fn two_decimals(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    format!("{rounded:.2}")
}

/// `Strings.ERROR`.
pub const ERROR: &str = "Error";
/// What `BUT_write_Click` says when the Home Location boxes do not parse.
/// `// C#: GCSViews/FlightPlanner.cs:670`
pub const HOME_INVALID: &str = "Your home location is invalid";
/// What the Home Location link says without a GPS position to take.
/// `// C#: GCSViews/FlightPlanner.cs:4297-4298`
pub const HOME_NEEDS_A_FIX: &str = "If you're at the field, connect to your APM and wait for GPS lock. Then click 'Home Location' link to set home to your location";
/// `MAV_AUTOPILOT_ARDUPILOTMEGA`: the one autopilot `saveWPs` puts home in front for.
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;

/// Where the plan on screen came from.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub enum Origin {
    /// Nothing has been loaded or drawn.
    #[default]
    Empty,
    /// Read back from the aircraft.
    Vehicle,
    /// Drawn on the map.
    Edited,
    /// Loaded from a file.
    File(String),
}

impl Origin {
    pub fn label(&self) -> String {
        match self {
            Self::Empty => "empty".to_owned(),
            Self::Vehicle => "read from the vehicle".to_owned(),
            Self::Edited => "edited here, not yet written".to_owned(),
            Self::File(name) => format!("loaded from {name}"),
        }
    }
}

impl Plan {
    /// The rows of the grid, in order: waypoint 1 onwards. Home is not among them.
    #[must_use]
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }

    /// Whether the grid has no rows. Home may still be set.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Where the contents came from.
    #[must_use]
    pub fn origin(&self) -> &Origin {
        &self.origin
    }

    /// The selected row's number.
    #[must_use]
    pub const fn selected(&self) -> Option<u16> {
        self.selected
    }

    /// Selects an item.
    pub fn select(&mut self, seq: Option<u16>) {
        self.selected = seq;
    }

    /// Replaces the rows with what the vehicle reported, and returns its home if the operator is
    /// to be asked whether to take it (see [`Plan::home_offer`]).
    ///
    /// `getWPs` hands the download to `processToScreen` whole, home and all, and item 0 leaves the
    /// grid there as it does for a file.
    /// `// C#: GCSViews/FlightPlanner.cs:1351-1373, 3978-4032`
    pub fn adopt_from_vehicle(&mut self, items: &[MissionItem]) -> Option<Home> {
        self.origin = Origin::Vehicle;
        self.adopt(items)
    }

    /// Replaces the rows with the contents of a file, and returns its home if the operator is to
    /// be asked whether to take it.
    ///
    /// `ReadWaypointFile` inserts a blank home when a file does not start at 0, so item 0 is
    /// always home by the time `processToScreen` sees the list.
    /// `// C#: GCSViews/FlightPlanner.cs:990-1010; ExtLibs/Utilities/MissionFile.cs:47-52`
    pub fn adopt_from_file(
        &mut self,
        name: impl Into<String>,
        items: &[MissionItem],
    ) -> Option<Home> {
        self.origin = Origin::File(name.into());
        self.adopt(items)
    }

    /// `processToScreen` for a mission, not appended: the grid replaced by everything after item
    /// 0, and item 0 offered to the Home Location boxes.
    /// `// C#: GCSViews/FlightPlanner.cs:5496-5676`
    fn adopt(&mut self, items: &[MissionItem]) -> Option<Home> {
        let (home, rows) = mp_mission::rows::split_home(items);
        self.items = rows;
        self.selected = None;
        home.and_then(|home| self.home_offer(&home))
    }

    /// Whether a mission's item 0 is to be offered as home, and if so the home it offers:
    /// "Reset Home to loaded coords", asked when its latitude, as the grid cell shows it, is
    /// neither what `TXT_homelat` holds nor `"0"`.
    ///
    /// Only the latitude is compared, as the C# compares it. The altitude the offer carries is
    /// the grid cell's, which holds `Locationwp.alt` - a `float`.
    /// `// C#: GCSViews/FlightPlanner.cs:5640-5660`
    #[must_use]
    pub fn home_offer(&self, item: &MissionItem) -> Option<Home> {
        let lat = double_text(item.x);
        if lat == self.home_text(HomeBox::Lat) || lat == "0" {
            return None;
        }
        #[allow(clippy::cast_possible_truncation)] // `Locationwp.alt` is a float
        let alt = f64::from(item.z as f32);
        Some(Home {
            lat: item.x,
            lng: item.y,
            alt,
        })
    }

    /// Yes to "Reset Home to loaded coords": the boxes take the loaded home, each through its
    /// `TextChanged`.
    /// `// C#: GCSViews/FlightPlanner.cs:5650-5657`
    pub fn reset_home_to(&mut self, loaded: Home) {
        self.set_home_text(HomeBox::Lat, double_text(loaded.lat));
        self.set_home_text(HomeBox::Lng, double_text(loaded.lng));
        self.set_home_text(HomeBox::Alt, two_decimals(loaded.alt));
    }

    /// One of the Home Location boxes.
    #[must_use]
    pub fn home_field(&self, which: HomeBox) -> &TextField {
        self.home.get(which)
    }

    /// What one of the Home Location boxes holds.
    #[must_use]
    pub fn home_text(&self, which: HomeBox) -> &str {
        self.home_field(which).value()
    }

    /// Sets a box as `TXT_home*.Text = ...` does, which raises its `TextChanged` when the text is
    /// different.
    pub fn set_home_text(&mut self, which: HomeBox, text: String) {
        if self.home_text(which) == text {
            return;
        }
        self.home.get_mut(which).set(text);
        self.home_text_changed(which);
    }

    /// A key pressed in one of the boxes, and its `TextChanged` when it changed the text.
    pub fn home_key(
        &mut self,
        which: HomeBox,
        event: &gpui::KeyDownEvent,
    ) -> crate::textfield::KeyOutcome {
        let outcome = self.home.get_mut(which).key(event);
        if outcome == crate::textfield::KeyOutcome::Changed {
            self.home_text_changed(which);
        }
        outcome
    }

    /// `TXT_homelat_TextChanged` and its two siblings: the planned home takes the box's number,
    /// or keeps what it had when the text does not parse. (Each also clears `sethome`, which
    /// belongs to `TXT_homelat_Enter` and is not ported, and redraws the map, which the caller
    /// does.)
    /// `// C#: GCSViews/FlightPlanner.cs:7001-7050`
    fn home_text_changed(&mut self, which: HomeBox) {
        let Some(value) = mp_mission::rows::parse_number(self.home_text(which)) else {
            return;
        };
        match which {
            HomeBox::Lat => self.planned_home.lat = value,
            HomeBox::Lng => self.planned_home.lng = value,
            HomeBox::Alt => self.planned_home.alt = value,
        }
    }

    /// Home as a write takes it: the three boxes parsed, or `None` - "Your home location is
    /// invalid" - when any one does not.
    /// `// C#: GCSViews/FlightPlanner.cs:658-671`
    #[must_use]
    pub fn home(&self) -> Option<Home> {
        Home::parse(
            self.home_text(HomeBox::Lat),
            self.home_text(HomeBox::Lng),
            self.home_text(HomeBox::Alt),
        )
    }

    /// `cs.PlannedHomeLocation`.
    #[cfg(test)]
    #[must_use]
    pub const fn planned_home(&self) -> Home {
        self.planned_home
    }

    /// Sets the planned home without touching the boxes, as `MainV2` does at start-up from the
    /// saved settings; the boxes show it when the planning screen is next activated.
    pub fn set_planned_home(&mut self, home: Home) {
        self.planned_home = home;
    }

    /// `updateHomeText`, which `Activate` runs each time the planning screen is shown: the boxes
    /// take the vehicle's home if it has sent one, else the planned home if there is one, and
    /// are left as they are otherwise. Setting them runs their `TextChanged`, so a vehicle's home
    /// becomes the planned home too.
    /// `// C#: GCSViews/FlightPlanner.cs:1344-1349, 7195-7218`
    pub fn update_home_text(&mut self, vehicle: Option<Home>) {
        let chosen = match vehicle {
            Some(home) if home.is_set() => home,
            _ if self.planned_home.is_set() => self.planned_home,
            _ => return,
        };
        self.set_home_text(HomeBox::Lat, double_text(chosen.lat));
        self.set_home_text(HomeBox::Lng, double_text(chosen.lng));
        self.set_home_text(HomeBox::Alt, two_decimals(chosen.alt));
    }

    /// The Home Location link, `label4_LinkClicked`: home is where the vehicle is, at its
    /// altitude above sea level - or, with no position, the C#'s advice about getting one. (It
    /// then zooms to home, which is `zoomToHomeToolStripMenuItem`'s and not ported.)
    /// `// C#: GCSViews/FlightPlanner.cs:4283-4300`
    pub fn home_from_vehicle(
        &mut self,
        vehicle: Option<(LatLon, f64)>,
    ) -> Result<(), &'static str> {
        let Some((position, altasl)) = vehicle.filter(|(position, _)| position.latitude() != 0.0)
        else {
            return Err(HOME_NEEDS_A_FIX);
        };
        self.set_home_text(HomeBox::Alt, two_decimals(altasl));
        self.set_home_text(HomeBox::Lat, double_text(position.latitude()));
        self.set_home_text(HomeBox::Lng, double_text(position.longitude()));
        Ok(())
    }

    /// What Write sends the vehicle: home from the boxes at item 0 on an ArduPilot, then the rows;
    /// or "Your home location is invalid", and nothing sent, when the boxes do not parse. That is
    /// checked first, whatever the autopilot.
    /// `// C#: GCSViews/FlightPlanner.cs:646-671, 6196-6227`
    pub fn vehicle_mission(&self, ardupilot: bool) -> Result<Vec<MissionItem>, &'static str> {
        let home = self.home().ok_or(HOME_INVALID)?;
        Ok(mp_mission::rows::upload_list(
            ardupilot.then_some(home),
            &self.items,
        ))
    }

    /// What Save File writes: the `.waypoints` text with home at record 0 and the rows after it.
    /// `// C#: GCSViews/FlightPlanner.cs:6108-6160`
    #[must_use]
    pub fn waypoints_file(&self) -> String {
        mp_mission::waypoints::write_planned(self.home(), &self.items)
    }

    /// Appends a waypoint at a position.
    ///
    /// Sequence numbers are reassigned from scratch rather than incremented, because a mission
    /// with a gap in its sequence is rejected by the vehicle at upload time - a failure that
    /// happens minutes after the mistake, with no indication of which edit caused it.
    #[cfg(test)]
    pub fn add_waypoint(&mut self, position: LatLon, altitude: f64) {
        self.add_waypoint_in(position, altitude, AltitudeFrame::Relative);
    }

    /// Adds a waypoint in a given altitude frame.
    ///
    /// The frame comes from the screen's `CMB_altmode` rather than the item, because that is where
    /// Mission Planner takes it from: `e.Row.Cells[Frame.Index].Value = CMB_altmode.SelectedValue`
    /// on row creation. `// C#: GCSViews/FlightPlanner.cs:2347`
    pub fn add_waypoint_in(&mut self, position: LatLon, altitude: f64, frame: AltitudeFrame) {
        self.items.push(MissionItem {
            seq: 0,
            current: 0,
            frame: frame.mav_frame(),
            command: CMD_WAYPOINT,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: position.latitude(),
            y: position.longitude(),
            z: altitude,
            autocontinue: 1,
        });
        self.renumber();
        self.origin = Origin::Edited;
    }

    /// Moves an item to a new position, keeping everything else about it.
    ///
    /// Only items that have a position are moved. Dragging what the map drew for a
    /// return-to-launch would write coordinates into a command that ignores them, and the map
    /// would then draw a waypoint the vehicle has no intention of visiting.
    pub fn move_to(&mut self, seq: u16, position: LatLon) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        if !matches!(item.position(), Ok(Some(_))) {
            return;
        }
        item.x = position.latitude();
        item.y = position.longitude();
        self.origin = Origin::Edited;
    }

    /// Removes an item.
    pub fn remove(&mut self, seq: u16) {
        self.items.retain(|item| item.seq != seq);
        self.renumber();
        self.origin = Origin::Edited;
        if self.selected == Some(seq) {
            self.selected = None;
        }
    }

    /// Moves an item up or down the list.
    pub fn move_item(&mut self, seq: u16, delta: i32) {
        let Some(index) = self.items.iter().position(|item| item.seq == seq) else {
            return;
        };
        let Ok(index) = i32::try_from(index) else {
            return;
        };
        let target = index + delta;
        let Ok(target) = usize::try_from(target) else {
            return;
        };
        let Ok(index) = usize::try_from(index) else {
            return;
        };
        if target >= self.items.len() {
            return;
        }
        self.items.swap(index, target);
        self.renumber();
        self.origin = Origin::Edited;
        self.selected = self.items.get(target).map(|item| item.seq);
    }

    /// Changes an item's altitude by a step, clamped so a held button cannot run away.
    ///
    /// Negative altitudes are allowed: a relative-frame waypoint below home is meaningful on a
    /// vehicle launched from a cliff or a rooftop, and refusing it would be the ground station
    /// deciding it knows the terrain better than the operator.
    pub fn nudge_altitude(&mut self, seq: u16, delta: f64) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        item.z = (item.z + delta).clamp(-MAX_STEPPED_ALTITUDE, MAX_STEPPED_ALTITUDE);
        self.origin = Origin::Edited;
    }

    /// Changes one of an item's four command parameters.
    ///
    /// Which parameter means what depends on the command, and the meanings come from the MAVLink
    /// definitions rather than from a table here - see `MavCmd::parameters`.
    pub fn set_param(&mut self, seq: u16, index: usize, value: f64) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        match index {
            0 => item.param1 = value,
            1 => item.param2 = value,
            2 => item.param3 = value,
            3 => item.param4 = value,
            _ => return,
        }
        self.origin = Origin::Edited;
    }

    /// One of an item's four command parameters.
    #[must_use]
    pub fn param(&self, seq: u16, index: usize) -> Option<f64> {
        let item = self.items.iter().find(|item| item.seq == seq)?;
        match index {
            0 => Some(item.param1),
            1 => Some(item.param2),
            2 => Some(item.param3),
            3 => Some(item.param4),
            _ => None,
        }
    }

    /// Changes what an item does.
    ///
    /// Return-to-launch and land take no position, so changing to one zeroes the coordinates.
    /// Leaving stale coordinates on a command that ignores them is how a mission looks right on
    /// the map and flies somewhere else: the map would keep drawing a waypoint the vehicle has no
    /// intention of visiting.
    pub fn set_command(&mut self, seq: u16, command: u16) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        item.command = command;
        if matches!(command, CMD_RTL | CMD_LAND_NO_POSITION) {
            item.x = 0.0;
            item.y = 0.0;
        }
        self.origin = Origin::Edited;
    }

    /// Discards everything, including the survey area and the fence - but not home, which is the
    /// Home Location boxes' and not the mission's.
    pub fn clear(&mut self) {
        self.items.clear();
        self.origin = Origin::Empty;
        self.selected = None;
        self.polygon.clear();
        self.survey_error = None;
        self.fence.clear();
        self.fence_error = None;
        self.rally.clear();
        self.rally_error = None;
    }

    /// The geofence's vertices.
    #[must_use]
    pub fn fence(&self) -> &[LatLon] {
        &self.fence
    }

    /// Adds a vertex to the geofence.
    pub fn add_fence_vertex(&mut self, position: LatLon) {
        self.fence.push(position);
        self.fence_error = None;
    }

    /// Removes the last fence vertex drawn.
    pub fn undo_fence_vertex(&mut self) {
        self.fence.pop();
        self.fence_error = None;
    }

    /// Discards the geofence, leaving the mission alone.
    pub fn clear_fence(&mut self) {
        self.fence.clear();
        self.fence_error = None;
    }

    /// The rally points.
    #[must_use]
    pub fn rally(&self) -> &[RallyPoint] {
        &self.rally
    }

    /// Adds a rally point at the survey altitude, which is the only altitude on this screen.
    pub fn add_rally_point(&mut self, position: LatLon) {
        self.rally.push(RallyPoint {
            position,
            altitude: self.survey.altitude,
            break_altitude: None,
        });
        self.rally_error = None;
    }

    /// Removes the last rally point placed.
    pub fn undo_rally_point(&mut self) {
        self.rally.pop();
        self.rally_error = None;
    }

    /// Discards the rally points.
    pub fn clear_rally(&mut self) {
        self.rally.clear();
        self.rally_error = None;
    }

    /// Why the rally points are not usable, if they are not.
    #[must_use]
    pub fn rally_error(&self) -> Option<&str> {
        self.rally_error.as_deref()
    }

    /// The rally points as the items the protocol carries.
    pub fn rally_items(&mut self) -> Option<Vec<MissionItem>> {
        if self.rally.is_empty() {
            self.rally_error = Some("place at least one rally point first".to_owned());
            return None;
        }
        self.rally_error = None;
        Some(
            self.rally
                .iter()
                .enumerate()
                .map(|(index, point)| point.to_item(u16::try_from(index).unwrap_or(u16::MAX)))
                .collect(),
        )
    }

    /// Replaces the rally points with what the vehicle reported.
    pub fn adopt_rally(&mut self, items: &[MissionItem]) {
        self.rally = items
            .iter()
            .filter_map(|item| {
                let position = item.position().ok().flatten()?;
                Some(RallyPoint {
                    position,
                    altitude: item.z,
                    break_altitude: (item.param2 != 0.0).then_some(item.param2),
                })
            })
            .collect();
        self.rally_error = self
            .rally
            .is_empty()
            .then(|| "the vehicle holds no rally points".to_owned());
    }

    /// Why the fence is not usable, if it is not.
    #[must_use]
    pub fn fence_error(&self) -> Option<&str> {
        self.fence_error.as_deref()
    }

    /// The fence as the items the protocol carries, or `None` if it is not valid.
    ///
    /// An inclusion polygon: the vehicle must stay inside it. Exclusion zones and circles exist in
    /// the protocol and are not offered here yet, because a fence drawn wrongly is worse than no
    /// fence - it either does nothing or triggers a return-to-launch in flight.
    pub fn fence_items(&mut self) -> Option<Vec<MissionItem>> {
        let fence = FenceItem::Polygon {
            inclusion: true,
            vertices: self.fence.clone(),
        };
        match fence.validate() {
            Ok(()) => {
                self.fence_error = None;
                Some(fence.to_items(0))
            }
            Err(why) => {
                self.fence_error = Some(why.to_string());
                None
            }
        }
    }

    /// Replaces the fence with what the vehicle reported.
    ///
    /// Only the first inclusion polygon is shown. A vehicle can hold several fences and this
    /// editor draws one; saying so is better than silently showing a fraction of what is loaded.
    pub fn adopt_fence(&mut self, items: &[MissionItem]) {
        match mp_mission::fences_from_items(items) {
            Ok(fences) => {
                let polygons: Vec<&FenceItem> = fences
                    .iter()
                    .filter(|fence| matches!(fence, FenceItem::Polygon { .. }))
                    .collect();
                match polygons.first() {
                    Some(FenceItem::Polygon { vertices, .. }) => {
                        self.fence = vertices.clone();
                        self.fence_error = (polygons.len() > 1).then(|| {
                            format!(
                                "the vehicle holds {} polygons; showing the first",
                                polygons.len()
                            )
                        });
                    }
                    _ => {
                        self.fence.clear();
                        self.fence_error = Some("the vehicle holds no polygon fence".to_owned());
                    }
                }
            }
            Err(why) => self.fence_error = Some(why.to_string()),
        }
    }

    /// What a click on the map does.
    #[must_use]
    pub const fn draw_mode(&self) -> DrawMode {
        self.draw_mode
    }

    /// Changes what a click on the map does.
    pub fn set_draw_mode(&mut self, mode: DrawMode) {
        self.draw_mode = mode;
    }

    /// The survey area's vertices.
    #[must_use]
    pub fn polygon(&self) -> &[LatLon] {
        &self.polygon
    }

    /// Adds a vertex to the survey area.
    pub fn add_area_vertex(&mut self, position: LatLon) {
        self.polygon.push(position);
        self.survey_error = None;
    }

    /// Removes the last vertex, which is the undo an operator reaches for while drawing.
    pub fn undo_area_vertex(&mut self) {
        self.polygon.pop();
        self.survey_error = None;
    }

    /// Discards the survey area, leaving the mission alone.
    pub fn clear_area(&mut self) {
        self.polygon.clear();
        self.survey_error = None;
    }

    /// How the survey should be flown.
    #[must_use]
    pub const fn survey_options(&self) -> GridOptions {
        self.survey
    }

    /// Adjusts the survey settings, keeping each within a range that produces a flyable grid.
    ///
    /// Spacing has a floor because a survey at one metre spacing over a field generates tens of
    /// thousands of waypoints, which no autopilot will accept and no operator intended. The angle
    /// wraps rather than clamps: a bearing is circular, and stopping at 359 would be arbitrary.
    pub fn adjust_survey(&mut self, spacing: f64, angle: f64, altitude: f64) {
        self.survey.spacing = (self.survey.spacing + spacing).clamp(5.0, 1000.0);
        self.survey.angle = (self.survey.angle + angle).rem_euclid(360.0);
        self.survey.altitude = (self.survey.altitude + altitude).clamp(1.0, MAX_STEPPED_ALTITUDE);
        self.survey_error = None;
    }

    /// Why the last survey could not be generated.
    #[must_use]
    pub fn survey_error(&self) -> Option<&str> {
        self.survey_error.as_deref()
    }

    /// Replaces the mission with a lawnmower pattern covering the survey area.
    ///
    /// Replaces rather than appends. A survey joined onto an existing mission would fly the old
    /// waypoints first and then transit to the area, which is almost never what was meant, and
    /// the operator who wanted that can save the two separately.
    pub fn generate_survey(&mut self) {
        match grid(&self.polygon, &self.survey) {
            Ok(positions) if positions.is_empty() => {
                self.survey_error = Some(
                    "the area is smaller than one line spacing; reduce the spacing".to_owned(),
                );
            }
            Ok(positions) => {
                let altitude = self.survey.altitude;
                self.items = positions
                    .into_iter()
                    .map(|position| MissionItem {
                        seq: 0,
                        current: 0,
                        frame: FRAME_RELATIVE,
                        command: CMD_WAYPOINT,
                        param1: 0.0,
                        param2: 0.0,
                        param3: 0.0,
                        param4: 0.0,
                        x: position.latitude(),
                        y: position.longitude(),
                        z: altitude,
                        autocontinue: 1,
                    })
                    .collect();
                self.renumber();
                self.origin = Origin::Edited;
                self.selected = None;
                self.survey_error = None;
            }
            Err(err) => self.survey_error = Some(err.to_string()),
        }
    }

    /// Numbers the rows 1..n, as the grid's headers read, so the sequence has no gaps.
    fn renumber(&mut self) {
        mp_mission::rows::number_rows(&mut self.items);
    }

    /// The altitude a new item gets: the stand-in for `TXT_DefaultAlt`.
    ///
    /// Mission Planner takes it from a box on the planning screen (`FlightPlanner.cs:6873`), which
    /// this screen does not have. A new item copies the last item that has a position, which is
    /// what an operator gets from that box anyway - set once, inherited by everything after - and
    /// falls back to [`DEFAULT_ALTITUDE`]. Items without a position are skipped because their
    /// altitude field is zero or means something else, and the C# never hands out zero:
    /// `if (ans == 0) cell.Value = 50;` (`FlightPlanner.cs:1197`).
    #[must_use]
    pub fn default_altitude(&self) -> f64 {
        self.items
            .iter()
            .rev()
            .find(|item| matches!(item.position(), Ok(Some(_))))
            .map_or(DEFAULT_ALTITUDE, |item| item.z)
    }

    /// `Commands.Rows.Add()` and the handler filling the new row: an item at the end.
    pub fn append(&mut self, item: MissionItem) {
        self.items.push(item);
        self.renumber();
        self.origin = Origin::Edited;
    }

    /// Insert Wp and Insert Spline WP: an item after the waypoint numbered `wpno`, or the C#'s
    /// refusal if it would throw on the number. The new row becomes the selected one, as
    /// `selectedrow = int.Parse(wpno)` makes it.
    /// `// C#: GCSViews/FlightPlanner.cs:4070-4091`
    pub fn insert_after(&mut self, wpno: &str, item: MissionItem) -> Result<u16, ()> {
        let index = mp_mission::rows::insert_index(self.items.len(), wpno).ok_or(())?;
        self.items.insert(index, item);
        self.renumber();
        self.origin = Origin::Edited;
        let seq = self.items.get(index).map_or(u16::MAX, |item| item.seq);
        self.selected = Some(seq);
        Ok(seq)
    }

    /// The number Insert Wp offers, `(selectedrow + 1)`: straight after the selected row.
    #[must_use]
    pub fn insert_offer(&self) -> usize {
        let selected_row = self
            .selected
            .and_then(|seq| self.items.iter().position(|item| item.seq == seq));
        mp_mission::rows::insert_offer(self.items.len(), selected_row)
    }

    /// Delete WP on the marker under the cursor: `Commands.Rows.RemoveAt(no - 1); // home is 0`.
    /// Only rows have markers here, so home is never the one under it.
    /// `// C#: GCSViews/FlightPlanner.cs:3118-3135`
    pub fn delete_marker(&mut self, seq: u16) -> bool {
        if !self.items.iter().any(|item| item.seq == seq) {
            return false;
        }
        self.remove(seq);
        true
    }

    /// Clear Mission: `Commands.Rows.Clear()`. Every row goes; home, not being one, stays.
    /// `// C#: GCSViews/FlightPlanner.cs:2064-2082`
    pub fn clear_mission(&mut self) {
        self.items.clear();
        self.selected = None;
        self.origin = Origin::Edited;
    }

    /// Reverse WPs: every row taken from the end and added again, so the grid is reversed.
    /// `// C#: GCSViews/FlightPlanner.cs:5840-5858`
    pub fn reverse_waypoints(&mut self) {
        self.items.reverse();
        self.renumber();
        self.selected = None;
        self.origin = Origin::Edited;
    }

    /// Modify Alt: every row's altitude times a factor plus a change. `"*2"` doubles, `"20"`
    /// adds twenty; anything `float.Parse` refuses is refused.
    /// `// C#: GCSViews/FlightPlanner.cs:4921-4952`
    pub fn modify_altitudes(&mut self, altdif: &str) -> Result<(), ()> {
        let (multiplier, change) = if altdif.contains('*') {
            (
                altdif
                    .replace('*', " ")
                    .trim()
                    .parse::<f32>()
                    .map_err(|_| ())?,
                0.0,
            )
        } else {
            (1.0, altdif.trim().parse::<f32>().map_err(|_| ())?)
        };
        for item in &mut self.items {
            // The C# does this in float: `float.Parse(cell) * multiplyer + altchange`.
            #[allow(clippy::cast_possible_truncation)] // the C#'s float arithmetic, on purpose
            let altitude = item.z as f32;
            item.z = f64::from(altitude * multiplier + change);
        }
        self.origin = Origin::Edited;
        Ok(())
    }

    /// Polygon > Draw a Polygon: the first choice starts drawing (`polygongridmode = true`), and
    /// each one after adds the corner under the menu.
    /// `// C#: GCSViews/FlightPlanner.cs:1731-1756`
    pub fn draw_polygon(&mut self, position: LatLon) {
        if self.draw_mode != DrawMode::Area {
            self.draw_mode = DrawMode::Area;
            return;
        }
        self.add_area_vertex(position);
    }

    /// Polygon > Clear Polygon: the corners go, and drawing stops.
    /// `// C#: GCSViews/FlightPlanner.cs:2084-2094`
    pub fn clear_polygon(&mut self) {
        if self.draw_mode == DrawMode::Area {
            self.draw_mode = DrawMode::Waypoints;
        }
        self.clear_area();
    }

    /// Polygon > From Current Waypoints: the survey polygon becomes the `WAYPOINT` rows. Nothing
    /// happens without rows, and whether to clear the mission afterwards is the operator's
    /// question to answer - the caller asks it.
    /// `// C#: GCSViews/FlightPlanner.cs:3619-3637`
    pub fn polygon_from_waypoints(&mut self) -> bool {
        if self.items.is_empty() {
            return false;
        }
        self.polygon = mp_mission::rows::waypoint_positions(&self.items);
        self.survey_error = None;
        true
    }

    /// `AddWPToMap`: what a click on the map adds, by what is being drawn.
    ///
    /// Drawing a polygon adds a corner; the geofence and rally modes are `cmb_missiontype` set to
    /// FENCE and RALLY; otherwise a waypoint, in the screen's altitude frame.
    /// `// C#: GCSViews/FlightPlanner.cs:558-600`
    pub fn add_wp_to_map(&mut self, position: LatLon, altitude: f64, frame: AltitudeFrame) {
        match self.draw_mode {
            DrawMode::Waypoints => self.add_waypoint_in(position, altitude, frame),
            DrawMode::Area => self.add_area_vertex(position),
            DrawMode::Fence => self.add_fence_vertex(position),
            DrawMode::Rally => self.add_rally_point(position),
        }
    }
}

/// The commands the editor offers, and what their altitude means.
///
/// Not every `MAV_CMD` - there are hundreds, most of which belong in a full command editor rather
/// than a list a pilot scans while planning. These are the navigation commands that make up the
/// shape of a mission, which is what the map shows.
pub const EDITABLE_COMMANDS: &[(u16, &str)] = &[
    (16, "waypoint"),
    (22, "takeoff"),
    (21, "land"),
    (20, "return to launch"),
    (19, "loiter for time"),
    (17, "loiter unlimited"),
    (18, "loiter turns"),
    (82, "spline waypoint"),
];

/// Altitude steps offered, in metres, with the name a test script clicks them by.
///
/// Coarse and fine, because both are needed and neither alone is enough: 10 m steps make setting a
/// survey height quick, and 1 m steps matter near the ground.
pub const ALTITUDE_STEPS: [(f64, &str); 4] = [
    (-10.0, "alt-minus-10"),
    (-1.0, "alt-minus-1"),
    (1.0, "alt-plus-1"),
    (10.0, "alt-plus-10"),
];

/// The highest altitude the stepper will reach, in metres above home.
///
/// Not a limit on what can be flown - a mission loaded from a file keeps whatever it holds. It
/// stops a held button from walking a waypoint into the stratosphere, which is a data entry
/// accident rather than a decision.
pub const MAX_STEPPED_ALTITUDE: f64 = 1000.0;

/// A short name for a mission command.
///
/// The generated enum gives `MAV_CMD_NAV_WAYPOINT`; a table of those is unreadable, so the common
/// prefix is stripped. An unknown command shows its number rather than a guess.
fn command_label(command: u16) -> String {
    MavCmd(u32::from(command)).name().map_or_else(
        || format!("cmd {command}"),
        |name| {
            name.strip_prefix("MAV_CMD_NAV_")
                .or_else(|| name.strip_prefix("MAV_CMD_DO_"))
                .or_else(|| name.strip_prefix("MAV_CMD_CONDITION_"))
                .or_else(|| name.strip_prefix("MAV_CMD_"))
                .unwrap_or(name)
                .to_lowercase()
        },
    )
}

/// Move and delete, shown only on the selected row.
///
/// Always-visible controls on every row turn a list into a minefield: the click that selects an
/// item is a pixel away from the click that deletes it. Requiring a selection first means a
/// destructive action always takes two deliberate clicks.
fn row_controls(seq: u16, selected: bool, cx: &mut Context<MissionPlanner>) -> Vec<AnyElement> {
    if !selected {
        return Vec::new();
    }

    let small = |id: (&'static str, usize), label: &'static str, colour: u32| {
        div()
            .id(id)
            .px_1()
            .rounded_sm()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(colour))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(label)
    };

    vec![
        small(("plan-up", usize::from(seq)), "up", theme::TEXT)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.move_item(seq, -1);
                this.sync_map_mission();
                cx.notify();
            }))
            .into_any_element(),
        small(("plan-down", usize::from(seq)), "down", theme::TEXT)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.move_item(seq, 1);
                this.sync_map_mission();
                cx.notify();
            }))
            .into_any_element(),
        small(("plan-delete", usize::from(seq)), "delete", theme::ALERT)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.remove(seq);
                this.sync_map_mission();
                cx.notify();
            }))
            .into_any_element(),
    ]
}

/// The waypoint table.
pub fn items_panel(
    plan_items: &[MissionItem],
    selected: Option<u16>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let mut rows = div().flex().flex_col();

    rows = rows.child(
        div()
            .flex()
            .gap_2()
            .pb_1()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .child(div().w(px(28.0)).child("#"))
            .child(div().w(px(120.0)).child("command"))
            .child(div().w(px(150.0)).child("position"))
            .child(div().w(px(70.0)).child("alt")),
    );

    if plan_items.is_empty() {
        rows =
            rows.child(
                div().pt_2().text_xs().text_color(rgb(theme::DIM)).child(
                    "no items - click the map to add a waypoint, or read one from the vehicle",
                ),
            );
    }

    for item in plan_items {
        let seq = item.seq;
        let is_selected = selected == Some(seq);
        let position = match item.position() {
            Ok(Some(position)) => {
                format!("{:.6}, {:.6}", position.latitude(), position.longitude())
            }
            // A non-navigation command carries parameters in x and y, not coordinates. Showing
            // 0.000000, 0.000000 for a DO_SET_SERVO would be a lie that looks like a bad waypoint.
            Ok(None) => "-".to_owned(),
            Err(_) => "invalid".to_owned(),
        };
        rows = rows.child(
            crate::probe::measured(format!("plan-row-{seq}"), div())
                .id(("plan-row", usize::from(seq)))
                .flex()
                .gap_2()
                .py_1()
                .text_xs()
                .cursor_pointer()
                .text_color(rgb(if is_selected {
                    theme::ACCENT
                } else {
                    theme::TEXT
                }))
                .bg(rgb(if is_selected {
                    theme::ACTION
                } else {
                    theme::PANEL
                }))
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(
                    div()
                        .w(px(28.0))
                        .text_color(rgb(theme::DIM))
                        .child(seq.to_string()),
                )
                .child(div().w(px(120.0)).child(command_label(item.command)))
                .child(div().w(px(150.0)).child(position))
                .child(div().w(px(70.0)).child(format!("{:.0} m", item.z)))
                // Which frame that altitude is in. The C# shows it as a `Frame` column on every
                // row, and it has to be visible: "50 m" means three different heights depending
                // on this, and one of them flies into a hill.
                // `// C#: GCSViews/FlightPlanner.cs:262, 2347`
                .child(
                    div()
                        .w(px(60.0))
                        .text_color(rgb(if item.frame == FRAME_TERRAIN {
                            theme::WARN
                        } else {
                            theme::DIM
                        }))
                        .child(frame_label(item.frame)),
                )
                .children(row_controls(seq, is_selected, cx))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    // Clicking the selected row clears the selection, so there is always a way
                    // back to "nothing selected" without hunting for empty space.
                    let already = this.plan.selected() == Some(seq);
                    this.plan.select(if already { None } else { Some(seq) });
                    cx.notify();
                })),
        );
    }

    // Scrolls, rather than clipping at a fixed height. A survey over a modest area generates
    // dozens of waypoints and clipping made everything past the twelfth unreachable - the operator
    // could see that item 40 existed, because the checks panel named it, and could not select it
    // to change or delete it.
    let count = plan_items.len();
    panel(
        "mission items",
        div()
            .flex()
            .flex_col()
            .child(
                div()
                    .id("plan-items")
                    .flex()
                    .flex_col()
                    .max_h(px(280.0))
                    .overflow_y_scroll()
                    .child(rows),
            )
            .children((count > 8).then(|| {
                div()
                    .pt_1()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(format!("{count} items - scroll for the rest"))
            })),
    )
}

/// The editor for the selected item: what it does and how high.
///
/// Steppers rather than typed numbers. Text entry in gpui needs a focus-managing input element
/// that does not exist here yet, and a stepper cannot produce a half-typed altitude that looks
/// like a number - "5" on the way to "50" is a valid altitude, and a mission editor that can
/// briefly hold one is a mission editor that can upload one.
pub fn editor_panel(
    plan_items: &[MissionItem],
    selected: Option<u16>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(item) = selected.and_then(|seq| plan_items.iter().find(|item| item.seq == seq)) else {
        return panel(
            "item",
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("select an item to change what it does"),
        )
        .into_any_element();
    };
    let seq = item.seq;
    let current_command = item.command;

    let mut commands = div().flex().flex_wrap().gap_1();
    for (command, label) in EDITABLE_COMMANDS {
        let chosen = *command == current_command;
        let command = *command;
        commands = commands.child(
            crate::probe::measured(*label, div())
                .id(("plan-cmd", usize::from(command)))
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .bg(rgb(if chosen { theme::ACTION } else { theme::PANEL }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(*label)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plan.set_command(seq, command);
                    this.sync_map_mission();
                    cx.notify();
                })),
        );
    }

    let mut steppers = div().flex().items_center().gap_2().child(
        div()
            .w(px(80.0))
            .text_lg()
            .text_color(rgb(theme::TEXT))
            .child(format!("{:.0} m", item.z)),
    );
    for (delta, name) in ALTITUDE_STEPS {
        steppers = steppers.child(
            crate::probe::measured(name, div())
                .id(name)
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(if delta > 0.0 {
                    format!("+{delta:.0}")
                } else {
                    format!("{delta:.0}")
                })
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plan.nudge_altitude(seq, delta);
                    this.sync_map_mission();
                    cx.notify();
                })),
        );
    }

    // What this command's parameters mean, from the MAVLink definitions. A command that uses none
    // of them - return-to-launch, land - shows none, rather than four controls doing nothing.
    let mut parameters = div().flex().flex_col().gap_1();
    let mut any_parameters = false;
    for (index, described) in MavCmd(u32::from(current_command))
        .parameters()
        .iter()
        .enumerate()
    {
        let Some((label, units)) = described else {
            continue;
        };
        any_parameters = true;
        let value =
            plan_items
                .iter()
                .find(|item| item.seq == seq)
                .map_or(0.0, |item| match index {
                    0 => item.param1,
                    1 => item.param2,
                    2 => item.param3,
                    _ => item.param4,
                });
        let shown = if units.is_empty() {
            format!("{value:.0}")
        } else {
            format!("{value:.0} {units}")
        };
        parameters = parameters.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(96.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child((*label).to_owned()),
                )
                .child(
                    div()
                        .w(px(68.0))
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(shown),
                )
                .child(param_stepper(seq, index, -1.0, "-1", cx))
                .child(param_stepper(seq, index, 1.0, "+1", cx))
                .child(param_stepper(seq, index, 10.0, "+10", cx)),
        );
    }

    panel(
        format!("item {seq}").as_str(),
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child("command"))
            .child(commands)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("altitude above home"),
            )
            .child(steppers)
            .children(any_parameters.then(|| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("command parameters")
            }))
            .child(parameters),
    )
    .into_any_element()
}

/// One step button for a command parameter.
///
/// Parameters are clamped at zero below. Every one the editor offers is a count, a duration, a
/// radius or an angle, and a negative loiter time is not a thing a vehicle can fly. The exception
/// the definitions do allow - a negative loiter radius, meaning counter-clockwise - is a direction
/// rather than a magnitude, and belongs in a control that says so rather than in a stepper that
/// happens to go below zero.
fn param_stepper(
    seq: u16,
    index: usize,
    delta: f64,
    label: &'static str,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    crate::probe::measured(format!("param-{index}-{label}"), div())
        .id((label, index))
        .px_2()
        .py_1()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(label)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            let current = this.plan.param(seq, index).unwrap_or(0.0);
            this.plan.set_param(seq, index, (current + delta).max(0.0));
            cx.notify();
        }))
}

/// Everything drawn on the map: which mode a click is in, and the controls for that mode.
///
/// One panel rather than four. The survey area, the geofence and the rally points are all placed
/// by clicking, so the question is always "what does a click do now" - and four panels each
/// answering it separately made a sidebar that had to be scrolled to find out.
pub struct DrawState<'a> {
    /// What a click does.
    pub mode: DrawMode,
    /// Survey area corners placed.
    pub area_vertices: usize,
    /// How the survey should be flown.
    pub survey: GridOptions,
    /// Why the last survey failed, if it did.
    pub survey_error: Option<&'a str>,
    /// Fence corners placed.
    pub fence_vertices: usize,
    /// Why the fence is unusable, if it is.
    pub fence_error: Option<&'a str>,
    /// Rally points placed.
    pub rally_points: usize,
    /// Why the rally points are unusable, if they are.
    pub rally_error: Option<&'a str>,
}

/// The draw panel.
pub fn draw_panel(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let mut modes = div().flex().flex_wrap().gap_1();
    for mode in DrawMode::ALL {
        let selected = mode == state.mode;
        modes = modes.child(
            crate::probe::measured(mode.id(), div())
                .id(mode.id())
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(if selected {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(if selected {
                    theme::ACTION
                } else {
                    theme::PANEL
                }))
                .text_xs()
                .text_color(rgb(if selected { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(mode.label())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plan.set_draw_mode(mode);
                    cx.notify();
                })),
        );
    }

    panel(
        "draw",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(modes)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(state.mode.hint()),
            )
            .child(mode_controls(state, view, cx)),
    )
}

/// The controls belonging to the active mode.
fn mode_controls(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    match state.mode {
        DrawMode::Waypoints => div()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .child("drag a waypoint to move it; select one to change what it does")
            .into_any_element(),
        DrawMode::Area => survey_controls(state, cx).into_any_element(),
        DrawMode::Fence => fence_controls(state, view, cx).into_any_element(),
        DrawMode::Rally => rally_controls(state, view, cx).into_any_element(),
    }
}

/// Spacing, angle, altitude and generate.
fn survey_controls(state: &DrawState<'_>, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let setting = |label: &'static str,
                   name: &'static str,
                   value: String,
                   down: &'static str,
                   up: &'static str,
                   step: f64,
                   which: usize,
                   cx: &mut Context<MissionPlanner>| {
        div()
            .flex()
            .items_center()
            .gap_2()
            .child(
                div()
                    .w(px(64.0))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(label),
            )
            .child(
                div()
                    .w(px(68.0))
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(value),
            )
            .child(stepper(down, name, -step, which, cx))
            .child(stepper(up, name, step, which, cx))
    };

    div()
        .flex()
        .flex_col()
        .gap_2()
        .child(
            div()
                .flex()
                .flex_wrap()
                .gap_2()
                .child(action(
                    "survey-undo",
                    "undo corner",
                    theme::TEXT,
                    state.area_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.undo_area_vertex();
                        this.sync_map_polygon();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "survey-clear",
                    "clear area",
                    theme::TEXT,
                    state.area_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.clear_area();
                        this.sync_map_polygon();
                        cx.notify();
                    }),
                )),
        )
        .child(corner_count(state.area_vertices, "corners"))
        .child(setting(
            "spacing",
            "survey-spacing",
            format!("{:.0} m", state.survey.spacing),
            "-5",
            "+5",
            5.0,
            0,
            cx,
        ))
        .child(setting(
            "angle",
            "survey-angle",
            format!("{:.0}°", state.survey.angle),
            "-15",
            "+15",
            15.0,
            1,
            cx,
        ))
        .child(setting(
            "altitude",
            "survey-altitude",
            format!("{:.0} m", state.survey.altitude),
            "-10",
            "+10",
            10.0,
            2,
            cx,
        ))
        .child(action(
            "survey-generate",
            "generate survey",
            theme::WARN,
            state.area_vertices >= 3,
            cx.listener(|this, _event: &(), _window, cx| {
                this.plan.generate_survey();
                this.sync_map_mission();
                cx.notify();
            }),
        ))
        .children(state.survey_error.map(problem))
}

/// Undo, clear, read and write for the geofence.
fn fence_controls(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    // Three is the fewest that encloses anything; the protocol and the vehicle both refuse fewer.
    let usable = state.fence_vertices >= 3;

    div()
        .flex()
        .flex_col()
        .gap_2()
        .child(
            div()
                .flex()
                .flex_wrap()
                .gap_2()
                .child(action(
                    "fence-undo",
                    "undo corner",
                    theme::TEXT,
                    state.fence_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.undo_fence_vertex();
                        this.sync_map_fence();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fence-clear",
                    "clear fence",
                    theme::TEXT,
                    state.fence_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.clear_fence();
                        this.sync_map_fence();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fence-read",
                    "read",
                    theme::ACCENT,
                    has_vehicle,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.request_fence();
                        this.adopt_vehicle_fence = true;
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fence-write",
                    "write",
                    theme::WARN,
                    has_vehicle && usable,
                    cx.listener(|this, _event: &(), _window, cx| {
                        if let Some(items) = this.plan.fence_items() {
                            this.telemetry.upload_fence(items);
                        }
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(if usable { theme::OK } else { theme::DIM }))
                .child(match state.fence_vertices {
                    0 => "no fence".to_owned(),
                    1 | 2 => format!("{} of at least 3 corners", state.fence_vertices),
                    n => format!("{n} corners - the vehicle must stay inside"),
                }),
        )
        .children(state.fence_error.map(problem))
}

/// Undo, clear, read and write for rally points.
fn rally_controls(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();

    div()
        .flex()
        .flex_col()
        .gap_2()
        .child(
            div()
                .flex()
                .flex_wrap()
                .gap_2()
                .child(action(
                    "rally-undo",
                    "undo point",
                    theme::TEXT,
                    state.rally_points > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.undo_rally_point();
                        this.sync_map_rally();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "rally-clear",
                    "clear",
                    theme::TEXT,
                    state.rally_points > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.clear_rally();
                        this.sync_map_rally();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "rally-read",
                    "read",
                    theme::ACCENT,
                    has_vehicle,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.request_rally();
                        this.adopt_vehicle_rally = true;
                        cx.notify();
                    }),
                ))
                .child(action(
                    "rally-write",
                    "write",
                    theme::WARN,
                    has_vehicle && state.rally_points > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        if let Some(items) = this.plan.rally_items() {
                            this.telemetry.upload_rally(items);
                        }
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(if state.rally_points > 0 {
                    theme::OK
                } else {
                    theme::DIM
                }))
                .child(match state.rally_points {
                    0 => "no rally points - a failsafe returns all the way home".to_owned(),
                    1 => "1 rally point".to_owned(),
                    n => format!("{n} rally points"),
                }),
        )
        .children(state.rally_error.map(problem))
}

/// How many corners are placed, and whether that is enough.
fn corner_count(count: usize, noun: &'static str) -> impl IntoElement {
    div()
        .text_xs()
        .text_color(rgb(if count >= 3 { theme::OK } else { theme::DIM }))
        .child(match count {
            0 => format!("no {noun} yet"),
            1 | 2 => format!("{count} of at least 3 {noun}"),
            n => format!("{n} {noun}"),
        })
}

/// A problem, said in the alert colour.
fn problem(text: &str) -> impl IntoElement {
    div()
        .text_xs()
        .text_color(rgb(theme::ALERT))
        .child(text.to_owned())
}

/// One step button for a survey setting./// One step button for a survey setting.
///
/// `which` selects the setting rather than passing a closure, because the three settings clamp
/// differently and that logic belongs on the plan, not in the view.
fn stepper(
    label: &'static str,
    name: &'static str,
    delta: f64,
    which: usize,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    crate::probe::measured(
        format!("{name}{}", if delta > 0.0 { "-up" } else { "-down" }),
        div(),
    )
    .id((name, which * 2 + usize::from(delta > 0.0)))
    .px_2()
    .py_1()
    .rounded_md()
    .border_1()
    .border_color(rgb(theme::BORDER))
    .text_xs()
    .text_color(rgb(theme::TEXT))
    .cursor_pointer()
    .hover(|style| style.bg(rgb(theme::BORDER)))
    .child(label)
    .on_click(cx.listener(move |this, _event, _window, cx| {
        let (spacing, angle, altitude) = match which {
            0 => (delta, 0.0, 0.0),
            1 => (0.0, delta, 0.0),
            _ => (0.0, 0.0, delta),
        };
        this.plan.adjust_survey(spacing, angle, altitude);
        cx.notify();
    }))
}

/// What the validator says about the plan.
pub fn checks_panel(plan_items: &[MissionItem], view: &TelemetryView) -> AnyElement {
    let findings = validate_with(
        plan_items,
        ValidationContext {
            home: view.state.as_ref().and_then(|s| s.home),
            vehicle_type: view.state.as_ref().map(|s| s.vehicle_type),
        },
    );

    if findings.is_empty() {
        return panel(
            "checks",
            div()
                .text_xs()
                .text_color(rgb(theme::OK))
                .child("nothing to report"),
        )
        .into_any_element();
    }

    let mut lines = div().flex().flex_col().gap_1();
    for finding in &findings {
        let colour = match finding.severity {
            Severity::Danger => theme::ALERT,
            Severity::Warning => theme::WARN,
            Severity::Note => theme::DIM,
        };
        let prefix = finding
            .seq
            .map_or_else(|| "mission".to_owned(), |seq| format!("item {seq}"));
        lines = lines.child(
            div()
                .flex()
                .gap_2()
                .text_xs()
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(56.0))
                        .text_color(rgb(theme::DIM))
                        .child(prefix),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_color(rgb(colour))
                        .child(finding.message.clone()),
                ),
        );
    }

    panel("checks", lines).into_any_element()
}

/// Read, write, load, save and clear.
/// The mission file name field and its focus, which travel together everywhere.
pub struct NameField<'a> {
    /// What has been typed.
    pub field: &'a crate::textfield::TextField,
    /// Its focus handle.
    pub focus: &'a gpui::FocusHandle,
    /// Whether it currently has focus.
    pub focused: bool,
}

pub fn actions_panel(
    plan_items: &[MissionItem],
    origin: &Origin,
    view: &TelemetryView,
    name: &NameField<'_>,
    frame: AltitudeFrame,
    tile_source: Option<&'static str>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let (name_focus, name_focused) = (name.focus, name.focused);
    let name = name.field;
    let has_vehicle = view.vehicle.is_some();
    let has_items = !plan_items.is_empty();

    // The altitude frame new waypoints get - `CMB_altmode` on the planning screen. A combo box
    // there, three buttons here, because gpui has no combo and three values do not need one.
    // `// C#: GCSViews/FlightPlanner.cs:234-239`
    let mut frames = div().flex().items_center().gap_1().child(
        div()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .child("new waypoints:"),
    );
    for choice in AltitudeFrame::all() {
        let chosen = choice == frame;
        frames = frames.child(
            crate::probe::measured(format!("plan-frame-{}", choice.key()), div())
                .id(gpui::SharedString::from(format!("frame-{}", choice.key())))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(choice.label())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.set_altitude_frame(choice);
                    cx.notify();
                })),
        );
    }
    // The map provider - `comboBoxMapType`, which lives on this screen in the C# and changes the
    // flight screen's map with it. Buttons rather than a combo, because gpui has no combo and
    // three providers do not need one.
    // `// C#: GCSViews/FlightPlanner.cs:176-180`
    let mut providers = div()
        .flex()
        .items_center()
        .flex_wrap()
        .gap_1()
        .child(div().text_xs().text_color(rgb(theme::DIM)).child("map:"));
    for source in mp_tiles::source::SOURCES {
        let chosen = Some(source.id) == tile_source;
        providers = providers.child(
            crate::probe::measured(format!("plan-map-{}", source.id), div())
                .id(gpui::SharedString::from(format!("map-{}", source.id)))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(source.label)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.set_tile_source(source);
                    cx.notify();
                })),
        );
    }

    // Said in words, because "terrain" on a button is not a warning and this one needs to be:
    // a vehicle without terrain data refuses the mission at upload, long after it was planned.
    let frame_note = (frame == AltitudeFrame::Terrain).then(|| {
        div()
            .text_xs()
            .text_color(rgb(theme::WARN))
            .child("terrain frame needs TERRAIN_ENABLE and terrain data on the vehicle")
    });

    let transfer_line = view.transfer.as_ref().map_or_else(
        || {
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(origin.label())
        },
        |status| {
            let colour = if status.failed {
                theme::ALERT
            } else if status.finished {
                theme::OK
            } else {
                theme::ACCENT
            };
            div()
                .text_xs()
                .text_color(rgb(colour))
                .child(status.label.clone())
        },
    );

    panel(
        "mission",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(frames)
            .children(frame_note)
            .child(providers)
            .child(
                div()
                    .flex()
                    .flex_wrap()
                    .gap_2()
                    .child(action(
                        "plan-read",
                        "read from vehicle",
                        theme::ACCENT,
                        has_vehicle,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.telemetry.request_mission();
                            this.adopt_vehicle_mission = true;
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-write",
                        "write to vehicle",
                        theme::WARN,
                        has_vehicle && has_items,
                        cx.listener(|this, _event: &(), window, cx| {
                            write_to_vehicle(this, window, cx);
                        }),
                    ))
                    .child(action(
                        "plan-save",
                        "save file",
                        theme::TEXT,
                        has_items,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.save_plan();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-load",
                        "load file",
                        theme::TEXT,
                        true,
                        cx.listener(|this, _event: &(), window, cx| {
                            this.load_plan();
                            // Reading a file whose home differs from the boxes asks about it.
                            if this.plan_menus.prompt.is_some() {
                                this.plan_prompt_focus.focus(window, cx);
                            }
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-clear",
                        "clear",
                        theme::TEXT,
                        has_items,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.plan.clear();
                            cx.notify();
                        }),
                    )),
            )
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(
                        div()
                            .w(px(36.0))
                            .text_xs()
                            .text_color(rgb(theme::DIM))
                            .child("file"),
                    )
                    .child(crate::textfield::text_field(
                        "plan-name",
                        name,
                        name_focus,
                        name_focused,
                        px(240.0),
                        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                            // Enter saves. A file name field where enter does nothing is a field
                            // that has to be followed by finding the button.
                            match this.plan_name.key(event) {
                                crate::textfield::KeyOutcome::Submitted => this.save_plan(),
                                crate::textfield::KeyOutcome::Ignored => return,
                                _ => {}
                            }
                            cx.notify();
                        }),
                    )),
            )
            .child(transfer_line)
            .children(view.transfer.as_ref().map(|status| {
                let colour = if status.failed {
                    theme::ALERT
                } else if status.finished {
                    theme::OK
                } else {
                    theme::ACCENT
                };
                progress(status.fraction, colour)
            })),
    )
}

// ---------------------------------------------------------------------------------------------
// Home: the Home Location panel, and what the rest of the application tells it.
// ---------------------------------------------------------------------------------------------

/// The vehicle's home, `cs.HomeLocation`, once it has sent `HOME_POSITION`.
///
/// **The altitude is a stand-in.** The C# takes `home.altitude / 1000.0` from `HOME_POSITION`
/// (`MAVLinkInterface.cs:5703-5707`), and `VehicleState` in this tree keeps only the message's
/// position. Until it keeps the altitude too, this takes the one ArduPilot's own
/// `GLOBAL_POSITION_INT` implies - `alt` above sea level less `relative_alt` above home - which is
/// the same number on an ArduPilot that has a home. Swap it for the state's home altitude when
/// there is one.
#[must_use]
pub fn vehicle_home(view: &TelemetryView) -> Option<Home> {
    let state = view.state.as_ref()?;
    let home = state.home?;
    Some(Home {
        lat: home.latitude(),
        lng: home.longitude(),
        alt: state.altitude_msl.0 - state.altitude_relative.0,
    })
}

/// The planned home `MainV2` starts with: `TXT_homelat`, `TXT_homelng` and `TXT_homealt` from
/// Mission Planner's `config.xml`, where the planning screen saves its boxes, each `GetDouble` -
/// 0 when absent or unreadable - and no home at all if the position is off the globe.
/// `// C#: MainV2.cs:1012-1025; ExtLibs/Utilities/Settings.cs:245-254; GCSViews/FlightPlanner.cs:2576-2578`
#[must_use]
pub fn planned_home_from_config(config: Option<&mp_settings::Config>) -> Home {
    let Some(config) = config else {
        return Home::default();
    };
    let get = |key: &str| {
        config
            .get(key)
            .and_then(mp_mission::rows::parse_number)
            .unwrap_or(0.0)
    };
    let home = Home {
        lat: get("TXT_homelat"),
        lng: get("TXT_homelng"),
        alt: get("TXT_homealt"),
    };
    // "remove invalid entrys"
    if home.lat.abs() > 90.0 || home.lng.abs() > 180.0 {
        return Home::default();
    }
    home
}

/// `FlightPlanner.Activate`'s `updateHome()`, run each time the planning screen is shown.
/// `// C#: GCSViews/FlightPlanner.cs:321, 1344-1349`
pub fn activate(this: &mut MissionPlanner) {
    let vehicle = vehicle_home(&this.telemetry.view());
    this.plan.update_home_text(vehicle);
}

/// Write: `BUT_write_Click`'s home check, then `saveWPs`' list - home from the boxes at item 0
/// when the vehicle is an ArduPilot - or "Your home location is invalid" and nothing sent.
/// `// C#: GCSViews/FlightPlanner.cs:646-671, 6196-6227`
fn write_to_vehicle(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let ardupilot = this
        .telemetry
        .view()
        .state
        .as_ref()
        .is_some_and(|state| state.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA);
    match this.plan.vehicle_mission(ardupilot) {
        Ok(items) => this.telemetry.upload_mission(items),
        Err(why) => {
            this.plan_menus.say(ERROR, why);
            this.plan_prompt_focus.focus(window, cx);
        }
    }
    cx.notify();
}

/// The Home Location link, `label4_LinkClicked`.
/// `// C#: GCSViews/FlightPlanner.cs:4283-4300`
fn home_link_clicked(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let view = this.telemetry.view();
    let vehicle = view.state.as_ref().and_then(|state| {
        state
            .position
            .map(|position| (position, state.altitude_msl.0))
    });
    if let Err(why) = this.plan.home_from_vehicle(vehicle) {
        this.plan_menus.say("", why);
        this.plan_prompt_focus.focus(window, cx);
    }
    cx.notify();
}

/// The Home Location boxes' focus handles, top to bottom, and which one has the keyboard.
pub struct HomeFocus<'a> {
    /// One per box, in [`HomeBox::ALL`] order.
    pub handles: &'a [gpui::FocusHandle; 3],
    /// Whether each has focus.
    pub focused: [bool; 3],
}

/// The Home Location panel, `panel1`: the link that takes the vehicle's position, and the three
/// boxes home is typed into, Lat, Long and ASL, one under the other with their labels to the
/// left - as the `.resx` places them, below the Read and Write buttons.
/// `// C#: GCSViews/FlightPlanner.resx (panel1, label4, Label1-3, TXT_homelat/lng/alt)`
pub fn home_panel(
    plan: &Plan,
    focus: &HomeFocus<'_>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let link = crate::probe::measured("plan-home-link", div())
        .id("plan-home-link")
        .text_xs()
        .text_color(rgb(theme::ACCENT))
        .underline()
        .cursor_pointer()
        .child("Home Location")
        .on_click(cx.listener(|this, _event, window, cx| {
            home_link_clicked(this, window, cx);
        }));

    let mut boxes = div().flex().flex_col().gap_1();
    for (index, which) in HomeBox::ALL.into_iter().enumerate() {
        let (Some(handle), Some(focused)) =
            (focus.handles.get(index), focus.focused.get(index).copied())
        else {
            continue;
        };
        boxes = boxes.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(36.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(which.label()),
                )
                .child(crate::textfield::text_field(
                    which.id(),
                    plan.home_field(which),
                    handle,
                    focused,
                    px(160.0),
                    cx.listener(move |this, event: &gpui::KeyDownEvent, _window, cx| {
                        if this.plan.home_key(which, event) == crate::textfield::KeyOutcome::Ignored
                        {
                            return;
                        }
                        cx.notify();
                    }),
                )),
        );
    }

    crate::probe::measured("panel:Home Location", div())
        .flex()
        .flex_col()
        .gap_2()
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .child(link)
        .child(boxes)
}

// ---------------------------------------------------------------------------------------------
// The map's right-click menu: `contextMenuStrip1`, and the `InputBox` prompts its items open.
// ---------------------------------------------------------------------------------------------

/// What choosing a menu entry does here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuAction {
    /// `deleteWPToolStripMenuItem_Click`.
    DeleteWp,
    /// `insertWpToolStripMenuItem_Click`.
    InsertWp,
    /// `currentPositionToolStripMenuItem_Click`.
    InsertAtCurrentPosition,
    /// `insertSplineWPToolStripMenuItem_Click`.
    InsertSplineWp,
    /// `loiterForeverToolStripMenuItem_Click`.
    LoiterForever,
    /// `loitertimeToolStripMenuItem_Click`.
    LoiterTime,
    /// `loitercirclesToolStripMenuItem_Click`.
    LoiterCircles,
    /// `jumpstartToolStripMenuItem_Click`.
    JumpStart,
    /// `jumpwPToolStripMenuItem_Click`.
    JumpWp,
    /// `rTLToolStripMenuItem_Click`.
    Rtl,
    /// `landToolStripMenuItem_Click`.
    Land,
    /// `takeoffToolStripMenuItem_Click`.
    Takeoff,
    /// `setROIToolStripMenuItem_Click`.
    SetRoi,
    /// `clearMissionToolStripMenuItem_Click`.
    ClearMission,
    /// `addPolygonPointToolStripMenuItem_Click`.
    DrawPolygon,
    /// `clearPolygonToolStripMenuItem_Click`.
    ClearPolygon,
    /// `fromCurrentWaypointsMenuItem_Click`.
    PolygonFromWaypoints,
    /// `ContextMeasure_Click`.
    MeasureDistance,
    /// `reverseWPsToolStripMenuItem_Click`.
    ReverseWps,
    /// `loadWPFileToolStripMenuItem_Click`, which is `BUT_loadwpfile_Click`.
    LoadWpFile,
    /// `saveWPFileToolStripMenuItem_Click`, which is `SaveFile_Click`.
    SaveWpFile,
    /// `modifyAltToolStripMenuItem_Click`.
    ModifyAlt,
}

/// One entry of `contextMenuStrip1` or of one of its drop-downs.
#[derive(Debug, Clone, Copy)]
pub struct MenuEntry {
    /// The id a test script clicks it by: `menu-` and the Designer's name without
    /// `ToolStripMenuItem`. Empty for the separator.
    pub id: &'static str,
    /// The control's name in `FlightPlanner.Designer.cs`. Read by the coverage tests, which hold
    /// the menu to the Designer by it.
    #[cfg_attr(not(test), allow(dead_code))]
    pub control: &'static str,
    /// Its `Text` in `FlightPlanner.resx`. Empty for the separator.
    pub text: &'static str,
    /// What choosing it does. `None` on an entry not ported here, which is drawn dimmed and
    /// inert so the menu keeps the C#'s shape, and on an entry that only opens a drop-down.
    pub action: Option<MenuAction>,
    /// Its drop-down, in the order the Designer adds them.
    pub children: &'static [MenuEntry],
}

impl MenuEntry {
    /// Whether this is `toolStripSeparator1`.
    #[must_use]
    pub const fn is_separator(&self) -> bool {
        self.text.is_empty()
    }

    /// Whether choosing it does anything here.
    #[must_use]
    pub const fn is_live(&self) -> bool {
        self.action.is_some() || !self.children.is_empty()
    }
}

const fn item(
    id: &'static str,
    control: &'static str,
    text: &'static str,
    action: Option<MenuAction>,
) -> MenuEntry {
    MenuEntry {
        id,
        control,
        text,
        action,
        children: &[],
    }
}

const fn drop_down(
    id: &'static str,
    control: &'static str,
    text: &'static str,
    action: Option<MenuAction>,
    children: &'static [MenuEntry],
) -> MenuEntry {
    MenuEntry {
        id,
        control,
        text,
        action,
        children,
    }
}

/// `contextMenuStrip1`, in the order `contextMenuStrip1.Items.AddRange` adds it, each drop-down in
/// its own `DropDownItems.AddRange` order, with the `.resx` text. A test holds all three to the C#
/// tree when it is present.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:899-922`
pub const MAP_MENU: &[MenuEntry] = {
    use MenuAction::{
        ClearMission, ClearPolygon, DeleteWp, DrawPolygon, InsertAtCurrentPosition, InsertSplineWp,
        InsertWp, JumpStart, JumpWp, Land, LoadWpFile, LoiterCircles, LoiterForever, LoiterTime,
        MeasureDistance, ModifyAlt, PolygonFromWaypoints, ReverseWps, Rtl, SaveWpFile, SetRoi,
        Takeoff,
    };
    &[
        item(
            "menu-deleteWP",
            "deleteWPToolStripMenuItem",
            "Delete WP",
            Some(DeleteWp),
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:937-938`
        drop_down(
            "menu-insertWp",
            "insertWpToolStripMenuItem",
            "Insert Wp",
            Some(InsertWp),
            &[item(
                "menu-currentPosition",
                "currentPositionToolStripMenuItem",
                "At Current Position",
                Some(InsertAtCurrentPosition),
            )],
        ),
        item(
            "menu-insertSplineWP",
            "insertSplineWPToolStripMenuItem",
            "Insert Spline WP",
            Some(InsertSplineWp),
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:957-960`
        drop_down(
            "menu-loiter",
            "loiterToolStripMenuItem",
            "Loiter",
            None,
            &[
                item(
                    "menu-loiterForever",
                    "loiterForeverToolStripMenuItem",
                    "Forever",
                    Some(LoiterForever),
                ),
                item(
                    "menu-loitertime",
                    "loitertimeToolStripMenuItem",
                    "Time",
                    Some(LoiterTime),
                ),
                item(
                    "menu-loitercircles",
                    "loitercirclesToolStripMenuItem",
                    "Circles",
                    Some(LoiterCircles),
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:984-986`
        drop_down(
            "menu-jump",
            "jumpToolStripMenuItem",
            "Jump",
            None,
            &[
                item(
                    "menu-jumpstart",
                    "jumpstartToolStripMenuItem",
                    "Start",
                    Some(JumpStart),
                ),
                item(
                    "menu-jumpwP",
                    "jumpwPToolStripMenuItem",
                    "WP #",
                    Some(JumpWp),
                ),
            ],
        ),
        item("menu-rTL", "rTLToolStripMenuItem", "RTL", Some(Rtl)),
        item("menu-land", "landToolStripMenuItem", "Land", Some(Land)),
        item(
            "menu-takeoff",
            "takeoffToolStripMenuItem",
            "Takeoff",
            Some(Takeoff),
        ),
        item(
            "menu-setROI",
            "setROIToolStripMenuItem",
            "DO_SET_ROI",
            Some(SetRoi),
        ),
        item(
            "menu-clearMission",
            "clearMissionToolStripMenuItem",
            "Clear Mission",
            Some(ClearMission),
        ),
        item("", "toolStripSeparator1", "", None),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1039-1047`
        drop_down(
            "menu-polygon",
            "polygonToolStripMenuItem",
            "Polygon",
            None,
            &[
                item(
                    "menu-addPolygonPoint2",
                    "addPolygonPointToolStripMenuItem2",
                    "Draw a Polygon",
                    Some(DrawPolygon),
                ),
                item(
                    "menu-clearPolygon2",
                    "clearPolygonToolStripMenuItem2",
                    "Clear Polygon",
                    Some(ClearPolygon),
                ),
                item(
                    "menu-savePolygon2",
                    "savePolygonToolStripMenuItem2",
                    "Save Polygon",
                    None,
                ),
                item(
                    "menu-loadPolygon2",
                    "loadPolygonToolStripMenuItem2",
                    "Load Polygon",
                    None,
                ),
                item(
                    "menu-fromSHP2",
                    "fromSHPToolStripMenuItem2",
                    "From SHP",
                    None,
                ),
                item(
                    "menu-fromCurrentWaypoints",
                    "fromCurrentWaypointsToolStripMenuItem",
                    "From Current Waypoints",
                    Some(PolygonFromWaypoints),
                ),
                item(
                    "menu-offsetPolygon2",
                    "offsetPolygonToolStripMenuItem2",
                    "Offset Polygon",
                    None,
                ),
                item("menu-area2", "areaToolStripMenuItem2", "Area", None),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1101-1107`
        drop_down(
            "menu-geoFence",
            "geoFenceToolStripMenuItem",
            "Geo-Fence",
            None,
            &[
                item(
                    "menu-GeoFenceupload",
                    "GeoFenceuploadToolStripMenuItem",
                    "Upload",
                    None,
                ),
                item(
                    "menu-GeoFencedownload",
                    "GeoFencedownloadToolStripMenuItem",
                    "Download",
                    None,
                ),
                item(
                    "menu-setReturnLocation",
                    "setReturnLocationToolStripMenuItem",
                    "Set Return Location",
                    None,
                ),
                item(
                    "menu-loadFromFile",
                    "loadFromFileToolStripMenuItem",
                    "Load from File",
                    None,
                ),
                item(
                    "menu-saveToFile",
                    "saveToFileToolStripMenuItem",
                    "Save to File",
                    None,
                ),
                item("menu-clear", "clearToolStripMenuItem", "Clear", None),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1149-1155`
        drop_down(
            "menu-rallyPoints",
            "rallyPointsToolStripMenuItem",
            "Rally Points",
            None,
            &[
                item(
                    "menu-setRallyPoint",
                    "setRallyPointToolStripMenuItem",
                    "Set Rally Point",
                    None,
                ),
                item(
                    "menu-getRallyPoints",
                    "getRallyPointsToolStripMenuItem",
                    "Download",
                    None,
                ),
                item(
                    "menu-saveRallyPoints",
                    "saveRallyPointsToolStripMenuItem",
                    "Upload",
                    None,
                ),
                item(
                    "menu-clearRallyPoints",
                    "clearRallyPointsToolStripMenuItem",
                    "Clear Rally Points",
                    None,
                ),
                item(
                    "menu-saveToFile1",
                    "saveToFileToolStripMenuItem1",
                    "Save Rally to File",
                    None,
                ),
                item(
                    "menu-loadFromFile1",
                    "loadFromFileToolStripMenuItem1",
                    "Load Rally from File",
                    None,
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1197-1203`
        drop_down(
            "menu-autoWP",
            "autoWPToolStripMenuItem",
            "Auto WP",
            None,
            &[
                item(
                    "menu-createWpCircle",
                    "createWpCircleToolStripMenuItem",
                    "Create Wp Circle",
                    None,
                ),
                item(
                    "menu-createSplineCircle",
                    "createSplineCircleToolStripMenuItem",
                    "Create Spline Circle",
                    None,
                ),
                item("menu-area1", "areaToolStripMenuItem1", "Area", None),
                item("menu-text", "textToolStripMenuItem", "Text", None),
                item(
                    "menu-createCircleSurvey",
                    "createCircleSurveyToolStripMenuItem",
                    "Create Circle Survey",
                    None,
                ),
                item(
                    "menu-surveyGrid",
                    "surveyGridToolStripMenuItem",
                    "Survey (Grid)",
                    None,
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1245-1254`
        drop_down(
            "menu-mapTool",
            "mapToolToolStripMenuItem",
            "Map Tool",
            None,
            &[
                item(
                    "menu-ContextMeasure",
                    "ContextMeasure",
                    "Measure Distance",
                    Some(MeasureDistance),
                ),
                item(
                    "menu-rotateMap",
                    "rotateMapToolStripMenuItem",
                    "Rotate Map",
                    None,
                ),
                item("menu-zoomTo", "zoomToToolStripMenuItem", "Zoom To", None),
                item(
                    "menu-prefetch",
                    "prefetchToolStripMenuItem",
                    "Prefetch",
                    None,
                ),
                item(
                    "menu-prefetchWPPath",
                    "prefetchWPPathToolStripMenuItem",
                    "Prefetch WP Path",
                    None,
                ),
                item(
                    "menu-kMLOverlay",
                    "kMLOverlayToolStripMenuItem",
                    "KML Overlay",
                    None,
                ),
                item(
                    "menu-elevationGraph",
                    "elevationGraphToolStripMenuItem",
                    "Elevation Graph",
                    None,
                ),
                item(
                    "menu-reverseWPs",
                    "reverseWPsToolStripMenuItem",
                    "Reverse WPs",
                    Some(ReverseWps),
                ),
                item(
                    "menu-gDALOpacity",
                    "gDALOpacityToolStripMenuItem",
                    "GDAL Opacity",
                    None,
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1308-1313`
        drop_down(
            "menu-fileLoadSave",
            "fileLoadSaveToolStripMenuItem",
            "File Load/Save",
            None,
            &[
                item(
                    "menu-loadWPFile",
                    "loadWPFileToolStripMenuItem",
                    "Load WP File",
                    Some(LoadWpFile),
                ),
                item(
                    "menu-loadAndAppend",
                    "loadAndAppendToolStripMenuItem",
                    "Load and Append",
                    None,
                ),
                item(
                    "menu-saveWPFile",
                    "saveWPFileToolStripMenuItem",
                    "Save WP File",
                    Some(SaveWpFile),
                ),
                item(
                    "menu-loadKMLFile",
                    "loadKMLFileToolStripMenuItem",
                    "Load KML File",
                    None,
                ),
                item(
                    "menu-loadSHPFile",
                    "loadSHPFileToolStripMenuItem",
                    "Load SHP File",
                    None,
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1349-1352`
        drop_down(
            "menu-pOI",
            "pOIToolStripMenuItem",
            "POI",
            None,
            &[
                item("menu-poiadd", "poiaddToolStripMenuItem", "Add", None),
                item(
                    "menu-poidelete",
                    "poideleteToolStripMenuItem",
                    "Delete",
                    None,
                ),
                item("menu-poiedit", "poieditToolStripMenuItem", "Edit", None),
            ],
        ),
        item(
            "menu-trackerHome",
            "trackerHomeToolStripMenuItem",
            "Tracker Home",
            None,
        ),
        item(
            "menu-modifyAlt",
            "modifyAltToolStripMenuItem",
            "Modify Alt",
            Some(ModifyAlt),
        ),
        item(
            "menu-enterUTMCoord",
            "enterUTMCoordToolStripMenuItem",
            "Enter UTM Coord",
            None,
        ),
        item(
            "menu-switchDocking",
            "switchDockingToolStripMenuItem",
            "Switch Docking",
            None,
        ),
        item(
            "menu-setHomeHere",
            "setHomeHereToolStripMenuItem",
            "Set Home Here",
            None,
        ),
    ]
};

/// Every entry, drop-downs included, in menu order.
#[cfg(test)]
pub fn menu_entries() -> impl Iterator<Item = &'static MenuEntry> {
    MAP_MENU
        .iter()
        .flat_map(|entry| std::iter::once(entry).chain(entry.children.iter()))
}

/// The menu while it is open.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpenMenu {
    /// Where the right button came up, in window coordinates: where the menu is drawn.
    pub at: (f32, f32),
    /// The map position there. The C# reads `MouseDownStart` in some handlers and `MouseDownEnd`
    /// in others - where the right button went down and where it came up - and a right click
    /// that did not drag makes them the same point, so one position stands for both.
    pub position: LatLon,
    /// The waypoint under the cursor when the menu opened: `CurentRectMarker`, which is what
    /// decides whether Delete WP is enabled (`FlightPlanner.cs:2669-2677`).
    pub marker: Option<u16>,
    /// Which top-level entry's drop-down is showing, by its index in [`MAP_MENU`].
    pub submenu: Option<usize>,
}

/// What an `InputBox` or `CustomMessageBox` the menu opened is for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PromptKind {
    /// "Insert WP after wp#", for a waypoint or a spline waypoint at the menu's position.
    InsertWp {
        /// Insert Spline WP rather than Insert Wp.
        spline: bool,
        /// Where the menu was opened.
        position: LatLon,
    },
    /// "Loiter Time".
    LoiterTime {
        /// Where the menu was opened.
        position: LatLon,
    },
    /// "Loiter Turns".
    LoiterTurns {
        /// Where the menu was opened.
        position: LatLon,
    },
    /// Jump > Start's "Number of times to Repeat".
    JumpStartRepeat,
    /// Jump > WP #'s "Jump to WP no?".
    JumpWpNumber,
    /// Jump > WP #'s "Number of times to Repeat", once the target is known.
    JumpWpRepeat {
        /// The item to jump to.
        target: f64,
    },
    /// Takeoff's "Please enter your takeoff altitude in m".
    TakeoffAltitude {
        /// Whether a pitch is asked for next.
        ask_pitch: bool,
    },
    /// Takeoff's "Please enter your takeoff pitch", on ArduPlane.
    TakeoffPitch {
        /// The altitude already given.
        altitude: f64,
    },
    /// Modify Alt's change.
    ModifyAlt,
    /// From Current Waypoints' "Clear current waypoints?", Yes or No.
    ClearWaypoints,
    /// `processToScreen`'s "Reset Home to loaded coords", Yes or No, with the home a mission
    /// just read carries at item 0.
    ResetHome(Home),
    /// A message with an OK.
    Message,
}

/// A dialog: an `InputBox` with its field, a Yes/No question, or a message.
///
/// Drawn over the screen and taking the keyboard, as the C#'s modal forms do. Enter is OK and
/// Escape is Cancel, as `InputBox`'s accept and cancel buttons are.
#[derive(Debug)]
pub struct Prompt {
    /// The window caption.
    pub title: &'static str,
    /// The prompt text.
    pub text: String,
    /// The value being typed, for an `InputBox`; `None` for a question or a message.
    pub field: Option<TextField>,
    /// What OK does.
    pub kind: PromptKind,
}

impl Prompt {
    /// An `InputBox.Show(title, text, ref value)`.
    fn input(title: &'static str, text: &str, value: impl Into<String>, kind: PromptKind) -> Self {
        let mut field = TextField::new("");
        field.set(value);
        Self {
            title,
            text: text.to_owned(),
            field: Some(field),
            kind,
        }
    }

    /// A `CustomMessageBox.Show(text, title)`.
    fn message(title: &'static str, text: impl Into<String>) -> Self {
        Self {
            title,
            text: text.into(),
            field: None,
            kind: PromptKind::Message,
        }
    }

    /// What has been typed, or nothing.
    #[must_use]
    pub fn value(&self) -> &str {
        self.field.as_ref().map_or("", TextField::value)
    }

    /// Whether this asks Yes or No.
    #[must_use]
    pub const fn is_question(&self) -> bool {
        matches!(
            self.kind,
            PromptKind::ClearWaypoints | PromptKind::ResetHome(_)
        )
    }
}

/// `Strings.InvalidNumberEntered`, without the resource's trailing newline.
const INVALID_NUMBER: &str = "Invalid number entered";

/// What the menu needs to know that the plan does not.
#[derive(Debug, Clone, Copy)]
pub struct MenuContext {
    /// `CMB_altmode`: the frame a new row gets.
    pub frame: AltitudeFrame,
    /// The vehicle's position and altitude above home, for Insert Wp > At Current Position.
    pub vehicle: Option<(LatLon, f64)>,
    /// Whether Takeoff asks for a pitch: on ArduPlane, unless `Q_OPTIONS` lacks bit 1.
    pub takeoff_pitch: bool,
}

impl MenuContext {
    /// Whether Takeoff asks for a pitch, from the vehicle's `MAV_TYPE` and parameters.
    ///
    /// `cs.firmware == Firmwares.ArduPlane` is the C#'s test, and `ArduPlane` is what it sets for
    /// a fixed wing, a flapping wing and every VTOL type (`MAVLinkInterface.cs:6724-6735`). A
    /// quadplane whose `Q_OPTIONS` lacks bit 1 skips the question.
    /// `// C#: GCSViews/FlightPlanner.cs:6797-6813`
    #[must_use]
    pub fn asks_takeoff_pitch(mav_type: Option<u8>, parameters: &[(String, f64)]) -> bool {
        let plane = matches!(mav_type, Some(1 | 16 | 19..=25));
        if !plane {
            return false;
        }
        match parameters.iter().find(|(name, _)| name == "Q_OPTIONS") {
            #[allow(clippy::cast_possible_truncation)] // `(int)MAV.param["Q_OPTIONS"]`
            Some((_, value)) => (*value as i64) & (1 << 1) != 0,
            None => true,
        }
    }
}

/// The distance and bearing Measure Distance reports, in the C#'s words:
/// `"Distance: " + FormatDistance(GetDistance(a, b), true) + " AZ: " + GetBearing(a, b)`.
///
/// `GetDistance` is GMap's `PureProjection.GetDistance`: a haversine on the projection's `Axis`,
/// 6378137 m for Mercator, in kilometres; `FormatDistance(km, true)` in the default metres writes
/// `(km * 1000).ToString("0.00 m")`. The bearing is `PureProjection.GetBearing` as `"0"`.
/// `// C#: GCSViews/FlightPlanner.cs:2649-2655, 3505-3531; ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs:436-472`
#[must_use]
pub fn measure_text(from: LatLon, to: LatLon) -> String {
    let (lat1, lng1) = (from.latitude().to_radians(), from.longitude().to_radians());
    let (lat2, lng2) = (to.latitude().to_radians(), to.longitude().to_radians());
    let a = ((lat2 - lat1) / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * ((lng2 - lng1) / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    let kilometres = (6_378_137.0 / 1000.0) * c;
    let bearing = from.bearing_to(to).0.0;
    // .NET's custom numeric formats round half away from zero; `{:.2}` rounds half to even.
    let metres = (kilometres * 1000.0 * 100.0).round() / 100.0;
    format!("Distance: {metres:.2} m AZ: {:.0}", bearing.round())
}

/// The planning screen's menu and the dialogs it opens.
#[derive(Debug, Default)]
pub struct PlanMenus {
    /// The menu, while it is open.
    pub open: Option<OpenMenu>,
    /// The dialog, while one is showing.
    pub prompt: Option<Prompt>,
    /// Measure Distance's first point, `startmeasure`, once chosen.
    pub measure_from: Option<LatLon>,
    /// Where the press that closed the menu went down. A click off an open menu closes it and
    /// does nothing else, so the map must not take the same press as a click that adds a
    /// waypoint.
    dismissed_at: Option<(f32, f32)>,
}

impl PlanMenus {
    /// Opens the menu where the right button came up. `contextMenuStrip1_Opening` runs here: the
    /// marker under the cursor decides Delete WP.
    pub fn open_at(&mut self, at: (f32, f32), position: LatLon, marker: Option<u16>) {
        self.open = Some(OpenMenu {
            at,
            position,
            marker,
            submenu: None,
        });
        self.dismissed_at = None;
    }

    /// Closes the menu because a press landed somewhere else.
    pub fn dismiss(&mut self, at: (f32, f32)) {
        if self.open.take().is_some() {
            self.dismissed_at = Some(at);
        }
    }

    /// Whether a press on the map is the one that just closed the menu, which it then consumes.
    pub fn swallows_press(&mut self, at: (f32, f32)) -> bool {
        self.dismissed_at.take() == Some(at)
    }

    /// Whether Delete WP is enabled: only over a marker.
    #[must_use]
    pub fn delete_enabled(&self) -> bool {
        self.open.is_some_and(|menu| menu.marker.is_some())
    }

    /// Shows a top-level entry's drop-down, or none.
    pub fn show_submenu(&mut self, index: Option<usize>) {
        if let Some(menu) = self.open.as_mut() {
            menu.submenu = index;
        }
    }

    fn ask(&mut self, prompt: Prompt) {
        self.prompt = Some(prompt);
    }

    fn tell(&mut self, title: &'static str, text: impl Into<String>) {
        self.prompt = Some(Prompt::message(title, text));
    }

    /// A `CustomMessageBox.Show(text, title)` from the screen rather than the menu: a write the
    /// Home Location boxes refused, the Home Location link without a position.
    pub fn say(&mut self, title: &'static str, text: impl Into<String>) {
        self.tell(title, text);
    }

    /// Asks "Reset Home to loaded coords", as `processToScreen` does when a mission it has just
    /// read carries a home at item 0 that the boxes do not hold. Yes puts it in the boxes; No
    /// leaves them.
    /// `// C#: GCSViews/FlightPlanner.cs:5645-5658`
    pub fn offer_home_reset(&mut self, loaded: Home) {
        self.ask(Prompt {
            title: "Reset Home Coords",
            text: "Reset Home to loaded coords".to_owned(),
            field: None,
            kind: PromptKind::ResetHome(loaded),
        });
    }

    /// Chooses an entry: what its handler does, up to the first `InputBox` it shows.
    ///
    /// Choosing closes the menu, as clicking a `ToolStripMenuItem` does. Load and Save are the
    /// caller's, because they are the screen's file handling rather than an edit.
    pub fn choose(&mut self, plan: &mut Plan, action: MenuAction, context: &MenuContext) {
        let Some(menu) = self.open.take() else {
            return;
        };
        let position = menu.position;
        let frame = context.frame.mav_frame();
        let altitude = plan.default_altitude();
        match action {
            MenuAction::DeleteWp => {
                if let Some(seq) = menu.marker {
                    plan.delete_marker(seq);
                }
            }
            MenuAction::InsertWp | MenuAction::InsertSplineWp => self.ask(Prompt::input(
                "Insert WP",
                "Insert WP after wp#",
                plan.insert_offer().to_string(),
                PromptKind::InsertWp {
                    spline: action == MenuAction::InsertSplineWp,
                    position,
                },
            )),
            MenuAction::InsertAtCurrentPosition => {
                // AddWPToMap(cs.lat, cs.lng, (int) cs.alt): an altitude of 0 takes the default.
                // With no position the C# would place it at 0,0; that is refused here in the
                // words zoomToVehicle uses for the same condition.
                // `// C#: GCSViews/FlightPlanner.cs:3051-3054, 1195-1197`
                let Some((at, vehicle_altitude)) = context.vehicle else {
                    self.tell(ERROR, "Invalid Location");
                    return;
                };
                let vehicle_altitude = vehicle_altitude.trunc();
                let altitude = if vehicle_altitude == 0.0 {
                    altitude
                } else {
                    vehicle_altitude
                };
                // Drawing a polygon, AddWPToMap adds a corner at MouseDownStart, the menu's
                // position, not the vehicle's.
                if plan.draw_mode() == DrawMode::Area {
                    plan.add_area_vertex(position);
                } else {
                    plan.add_wp_to_map(at, altitude, context.frame);
                }
            }
            MenuAction::LoiterForever => {
                plan.append(mp_mission::commands::loiter_unlimited(
                    position, altitude, frame,
                ));
            }
            MenuAction::LoiterTime => self.ask(Prompt::input(
                "Loiter Time",
                "Loiter Time",
                "5",
                PromptKind::LoiterTime { position },
            )),
            MenuAction::LoiterCircles => self.ask(Prompt::input(
                "Loiter Turns",
                "Loiter Turns",
                "3",
                PromptKind::LoiterTurns { position },
            )),
            MenuAction::JumpStart => self.ask(Prompt::input(
                "Jump repeat",
                "Number of times to Repeat",
                "5",
                PromptKind::JumpStartRepeat,
            )),
            MenuAction::JumpWp => self.ask(Prompt::input(
                "WP No",
                "Jump to WP no?",
                "1",
                PromptKind::JumpWpNumber,
            )),
            MenuAction::Rtl => plan.append(mp_mission::commands::return_to_launch(frame)),
            MenuAction::Land => plan.append(mp_mission::commands::land(position, frame)),
            // `CurrentState.AltUnit == "m" ? "10" : "30"`; this application is metric.
            MenuAction::Takeoff => self.ask(Prompt::input(
                "Altitude",
                "Please enter your takeoff altitude in m",
                "10",
                PromptKind::TakeoffAltitude {
                    ask_pitch: context.takeoff_pitch,
                },
            )),
            // `cmdParamNames.ContainsKey("DO_SET_ROI")` is true in all three of mavcmd.xml's
            // vehicle sections, so the C#'s "not enabled in your firmware" never shows.
            MenuAction::SetRoi => {
                plan.append(mp_mission::commands::set_roi(position, altitude, frame));
            }
            MenuAction::ClearMission => plan.clear_mission(),
            MenuAction::DrawPolygon => plan.draw_polygon(position),
            MenuAction::ClearPolygon => plan.clear_polygon(),
            MenuAction::PolygonFromWaypoints => {
                if plan.polygon_from_waypoints() {
                    self.ask(Prompt {
                        title: "Confirm",
                        text: "Clear current waypoints?".to_owned(),
                        field: None,
                        kind: PromptKind::ClearWaypoints,
                    });
                }
            }
            // The C# also drops a red marker at each end; the map here has no overlay for them.
            MenuAction::MeasureDistance => match self.measure_from.take() {
                None => {
                    self.measure_from = Some(position);
                    self.tell(
                        "Measure Dist",
                        "You can now pan/zoom around.\nClick this option again to get the distance.",
                    );
                }
                Some(from) => self.tell("", measure_text(from, position)),
            },
            MenuAction::ReverseWps => plan.reverse_waypoints(),
            MenuAction::ModifyAlt => self.ask(Prompt::input(
                "Alt Change",
                "Please enter the alitude change you require.\n(20 = up 20, *2 = up by alt * 2)",
                "0",
                PromptKind::ModifyAlt,
            )),
            MenuAction::LoadWpFile | MenuAction::SaveWpFile => {}
        }
    }

    /// OK, or Yes: the rest of the handler, with what was typed.
    ///
    /// A value the C# would refuse is refused with its message. The loiter and jump handlers put
    /// the typed text in the grid unchecked and fail later, at "Invalid number on row" when the
    /// mission is written; an item here holds a number, so that text is refused as it is typed,
    /// with `Strings.InvalidNumberEntered`.
    pub fn submit(&mut self, plan: &mut Plan, context: &MenuContext) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let value = prompt.value().to_owned();
        let frame = context.frame.mav_frame();
        let altitude = plan.default_altitude();
        let number = || {
            value
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|number| number.is_finite())
        };
        // `int.TryParse`, for the takeoff's two numbers.
        let integer = || value.trim().parse::<i32>().ok().map(f64::from);
        match prompt.kind {
            PromptKind::InsertWp { spline, position } => {
                let item = if spline {
                    mp_mission::commands::spline_waypoint(position, altitude, frame)
                } else {
                    mp_mission::commands::waypoint(position, altitude, frame)
                };
                if plan.insert_after(&value, item).is_err() {
                    // `// C#: GCSViews/FlightPlanner.cs:4046, 4081`
                    self.tell(
                        ERROR,
                        if spline {
                            INVALID_NUMBER
                        } else {
                            "Invalid insert position"
                        },
                    );
                }
            }
            PromptKind::LoiterTime { position } => match number() {
                Some(seconds) => plan.append(mp_mission::commands::loiter_time(
                    position, altitude, frame, seconds,
                )),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::LoiterTurns { position } => match number() {
                Some(turns) => plan.append(mp_mission::commands::loiter_turns(
                    position, altitude, frame, turns,
                )),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            // Jump > Start jumps to item 1: `Param1.Value = 1`.
            PromptKind::JumpStartRepeat => match number() {
                Some(repeat) => plan.append(mp_mission::commands::do_jump(1.0, repeat, frame)),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::JumpWpNumber => match number() {
                Some(target) => self.ask(Prompt::input(
                    "Jump repeat",
                    "Number of times to Repeat",
                    "5",
                    PromptKind::JumpWpRepeat { target },
                )),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::JumpWpRepeat { target } => match number() {
                Some(repeat) => plan.append(mp_mission::commands::do_jump(target, repeat, frame)),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::TakeoffAltitude { ask_pitch } => match integer() {
                Some(altitude) if ask_pitch => self.ask(Prompt::input(
                    "Takeoff Pitch",
                    "Please enter your takeoff pitch",
                    "15",
                    PromptKind::TakeoffPitch { altitude },
                )),
                Some(altitude) => {
                    plan.append(mp_mission::commands::takeoff(altitude, 0.0, frame));
                }
                // `MessageBox.Show("Bad Alt")`. `// C#: GCSViews/FlightPlanner.cs:6792`
                None => self.tell("", "Bad Alt"),
            },
            PromptKind::TakeoffPitch { altitude } => match integer() {
                Some(pitch) => plan.append(mp_mission::commands::takeoff(altitude, pitch, frame)),
                // `// C#: GCSViews/FlightPlanner.cs:6818`
                None => self.tell("", "Bad Takeoff pitch"),
            },
            PromptKind::ModifyAlt => {
                if plan.modify_altitudes(&value).is_err() {
                    self.tell(ERROR, INVALID_NUMBER);
                }
            }
            PromptKind::ClearWaypoints => plan.clear_mission(),
            PromptKind::ResetHome(loaded) => plan.reset_home_to(loaded),
            PromptKind::Message => {}
        }
    }

    /// Cancel, or No: the handler returns.
    pub fn cancel(&mut self) {
        self.prompt = None;
    }
}

/// The context the menu needs, from what the application knows right now.
fn menu_context(this: &MissionPlanner) -> MenuContext {
    let view = this.telemetry.view();
    let state = view.state.as_ref();
    MenuContext {
        frame: this.altitude_frame,
        vehicle: state.and_then(|state| {
            state
                .position
                .map(|position| (position, state.altitude_relative.0))
        }),
        takeoff_pitch: MenuContext::asks_takeoff_pitch(
            state.map(|state| state.vehicle_type),
            &view.parameters,
        ),
    }
}

/// Pushes everything the menu can change to the map.
fn sync_everything(this: &MissionPlanner) {
    this.sync_map_mission();
    this.sync_map_polygon();
    this.sync_map_fence();
    this.sync_map_rally();
}

/// Opens the menu for a right click on the map, as `MainMap.ContextMenuStrip = contextMenuStrip1`
/// does, with the waypoint under the cursor for Delete WP.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:875; GCSViews/FlightPlanner.cs:2667-2695`
pub fn open_map_menu(this: &mut MissionPlanner, x: f32, y: f32) {
    let (position, marker) = {
        let map = this.map.borrow();
        (map.position_at(x, y), map.waypoint_at(x, y))
    };
    let Some(position) = position else {
        return;
    };
    // The same reason as placing a waypoint: an edit made from the menu must not refit the view
    // and move the ground under the cursor.
    this.map.borrow_mut().freeze_view();
    this.plan_menus.open_at((x, y), position, marker);
}

/// Chooses an entry from the open menu, and focuses the dialog if it opened one.
fn choose_entry(
    this: &mut MissionPlanner,
    action: MenuAction,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    match action {
        MenuAction::LoadWpFile => {
            this.plan_menus.open = None;
            this.load_plan();
        }
        MenuAction::SaveWpFile => {
            this.plan_menus.open = None;
            this.save_plan();
        }
        _ => {
            let context = menu_context(this);
            this.plan_menus.choose(&mut this.plan, action, &context);
        }
    }
    sync_everything(this);
    if this.plan_menus.prompt.is_some() {
        this.plan_prompt_focus.focus(window, cx);
    }
    cx.notify();
}

/// OK, Yes, or Enter in a dialog.
fn submit_prompt(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let context = menu_context(this);
    this.plan_menus.submit(&mut this.plan, &context);
    sync_everything(this);
    if this.plan_menus.prompt.is_some() {
        this.plan_prompt_focus.focus(window, cx);
    }
    cx.notify();
}

/// Height of one menu entry, fixed so a drop-down can line up with the entry that opened it.
const MENU_ROW: f32 = 22.0;
/// Height of the separator.
const MENU_SEPARATOR: f32 = 9.0;
/// The menu's padding above its first entry.
const MENU_PADDING: f32 = 4.0;
/// The width of the menu and of each drop-down. Fixed, where a `ToolStripDropDown` sizes itself
/// to its widest item, because the drop-down is placed beside the menu by arithmetic rather than
/// by layout (see `map_menu`), and that arithmetic needs the width before the text is measured.
/// 190 px holds the widest entry, "From Current Waypoints", at this font with room to spare.
const MENU_WIDTH: f32 = 190.0;

/// The height of a column of entries: its padding, its border, and each row or separator.
fn column_height(entries: &[MenuEntry]) -> f32 {
    2.0 * MENU_PADDING
        + 2.0
        + entries
            .iter()
            .map(|entry| {
                if entry.is_separator() {
                    MENU_SEPARATOR
                } else {
                    MENU_ROW
                }
            })
            .sum::<f32>()
}

/// Where a drop-down of `height` starts, relative to the menu's top, when the menu's top is at
/// `menu_top` in a window `viewport_height` tall and the entry that opens it starts `offset`
/// down the menu.
///
/// It lines up with its entry when there is room below, and otherwise moves up just enough to
/// fit, as a `ToolStripDropDown` does. What it never does is move the menu: the menu and the
/// drop-down used to be one snapped body, so a long drop-down opening near the bottom of the
/// window pushed the whole menu up, the pointer that had just clicked "Map Tool" found itself on
/// "File Load/Save", and that entry's drop-down replaced the one the click had opened.
/// `tests/gui/plan-reverse-wps.gui` found it.
fn dropdown_top(menu_top: f32, offset: f32, height: f32, viewport_height: f32) -> f32 {
    let aligned = offset - MENU_PADDING;
    let lowest = viewport_height - height - menu_top;
    aligned.min(lowest).max(-menu_top)
}

/// How far down the menu a top-level entry starts.
fn menu_offset(index: usize) -> f32 {
    MENU_PADDING
        + MAP_MENU
            .iter()
            .take(index)
            .map(|entry| {
                if entry.is_separator() {
                    MENU_SEPARATOR
                } else {
                    MENU_ROW
                }
            })
            .sum::<f32>()
}

/// One entry, as a `ToolStripMenuItem` draws: its text, an arrow if it has a drop-down, dimmed if
/// it does nothing here.
fn menu_row(
    entry: &'static MenuEntry,
    top_level: Option<usize>,
    enabled: bool,
    highlighted: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if entry.is_separator() {
        return div()
            .h(px(MENU_SEPARATOR))
            .flex()
            .items_center()
            .px_2()
            .child(div().h(px(1.0)).w_full().bg(rgb(theme::BORDER)))
            .into_any_element();
    }
    let row = crate::probe::measured(entry.id, div())
        .id(entry.id)
        .h(px(MENU_ROW))
        .px_3()
        .flex()
        .items_center()
        .justify_between()
        .gap_4()
        .text_xs()
        .child(entry.text)
        .children((!entry.children.is_empty()).then_some("›"));
    if !enabled {
        return row.text_color(rgb(theme::DIM)).into_any_element();
    }
    let row = row
        .text_color(rgb(theme::TEXT))
        .bg(rgb(if highlighted {
            theme::ACTION
        } else {
            theme::PANEL
        }))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)));
    // A top-level entry shows its drop-down while the pointer is on it, and hides any other, as
    // a ToolStrip does; an entry inside a drop-down leaves it be.
    let row = match top_level {
        Some(index) => {
            let opens = (!entry.children.is_empty()).then_some(index);
            row.on_hover(cx.listener(move |this, hovered: &bool, _window, cx| {
                if *hovered {
                    this.plan_menus.show_submenu(opens);
                    cx.notify();
                }
            }))
        }
        None => row,
    };
    let action = entry.action;
    let opens = top_level.filter(|_| !entry.children.is_empty());
    row.on_click(cx.listener(move |this, _event, window, cx| {
        match action {
            Some(action) => choose_entry(this, action, window, cx),
            None => {
                // An entry that only opens a drop-down opens it on a click too.
                this.plan_menus.show_submenu(opens);
                cx.notify();
            }
        }
    }))
    .into_any_element()
}

/// A column of entries: the menu, or a drop-down.
fn menu_column(children: Vec<AnyElement>) -> gpui::Div {
    div()
        .flex()
        .flex_col()
        .w(px(MENU_WIDTH))
        .py(px(MENU_PADDING))
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .children(children)
}

/// `contextMenuStrip1`, drawn where the right button came up, above everything else.
fn map_menu(
    menus: &PlanMenus,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let menu = menus.open?;
    let delete_enabled = menus.delete_enabled();
    // Where the snapped menu ends up: `snap_to_window` moves a top-left anchored element up and
    // left just enough to fit, which with fixed sizes is this arithmetic.
    let viewport = window.viewport_size();
    let (viewport_width, viewport_height) = (f32::from(viewport.width), f32::from(viewport.height));
    let menu_height = column_height(MAP_MENU);
    let menu_left = menu.at.0.min(viewport_width - MENU_WIDTH).max(0.0);
    let menu_top = menu.at.1.min(viewport_height - menu_height).max(0.0);
    // The drop-down's rectangle in window coordinates, so a press on it is not a press outside
    // the menu. `None` while no drop-down shows.
    let dropdown = menu
        .submenu
        .and_then(|index| Some((index, MAP_MENU.get(index)?)))
        .map(|(index, entry)| {
            let height = column_height(entry.children);
            let top = dropdown_top(menu_top, menu_offset(index), height, viewport_height);
            (top, height)
        });
    let dropdown_contains = move |(x, y): (f32, f32)| {
        dropdown.is_some_and(|(top, height)| {
            let left = menu_left + MENU_WIDTH;
            let above = menu_top + top;
            (left..=left + MENU_WIDTH).contains(&x) && (above..=above + height).contains(&y)
        })
    };
    let rows = MAP_MENU
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            let enabled =
                entry.is_live() && (entry.action != Some(MenuAction::DeleteWp) || delete_enabled);
            menu_row(entry, Some(index), enabled, menu.submenu == Some(index), cx)
        })
        .collect();
    let mut body = div()
        .id("plan-menu")
        .flex()
        .items_start()
        .occlude()
        .on_mouse_down_out(
            cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                let at = (f32::from(event.position.x), f32::from(event.position.y));
                // The drop-down sits outside the body's own bounds, so a press on it arrives
                // here as "outside"; it is not.
                if dropdown_contains(at) {
                    return;
                }
                this.plan_menus.dismiss(at);
                cx.notify();
            }),
        )
        .child(menu_column(rows));
    if let Some(((top, _), entry)) =
        dropdown.zip(menu.submenu.and_then(|index| MAP_MENU.get(index)))
    {
        let rows = entry
            .children
            .iter()
            .map(|child| menu_row(child, None, child.is_live(), false, cx))
            .collect();
        // Absolutely positioned, so opening it changes nothing about the body the window snaps,
        // and the menu stays where the pointer is.
        body = body.child(
            div()
                .absolute()
                .left(px(MENU_WIDTH))
                .top(px(top))
                .occlude()
                .child(menu_column(rows)),
        );
    }
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(menu.at.0), px(menu.at.1)))
                .snap_to_window()
                .child(body),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// The dialog, centred over the window with everything behind it inert, as `ShowDialog` makes it.
fn prompt_dialog(
    menus: &PlanMenus,
    focus: &gpui::FocusHandle,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let prompt = menus.prompt.as_ref()?;
    let focused = focus.is_focused(window);
    let size = window.viewport_size();

    let (accept, refuse) = if prompt.is_question() {
        ("Yes", Some("No"))
    } else if prompt.field.is_some() {
        ("OK", Some("Cancel"))
    } else {
        ("OK", None)
    };
    let buttons = div()
        .flex()
        .justify_end()
        .gap_2()
        .child(action(
            "plan-prompt-ok",
            accept,
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), window, cx| submit_prompt(this, window, cx)),
        ))
        .children(refuse.map(|label| {
            action(
                "plan-prompt-cancel",
                label,
                theme::TEXT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.plan_menus.cancel();
                    cx.notify();
                }),
            )
        }));

    let on_key = cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
        let outcome = match this.plan_menus.prompt.as_mut() {
            Some(Prompt {
                field: Some(field), ..
            }) => field.key(event),
            // A question or a message has no field; Enter and Escape still answer it.
            Some(_) => match event.keystroke.key.as_str() {
                "enter" => crate::textfield::KeyOutcome::Submitted,
                "escape" => crate::textfield::KeyOutcome::Cancelled,
                _ => crate::textfield::KeyOutcome::Ignored,
            },
            None => return,
        };
        match outcome {
            crate::textfield::KeyOutcome::Submitted => submit_prompt(this, window, cx),
            crate::textfield::KeyOutcome::Cancelled => {
                this.plan_menus.cancel();
                cx.notify();
            }
            crate::textfield::KeyOutcome::Changed => cx.notify(),
            crate::textfield::KeyOutcome::Ignored => {}
        }
    });

    let mut dialog = crate::probe::measured("plan-prompt", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(prompt.title.to_owned()),
        )
        .child(
            div()
                .flex()
                .flex_col()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .children(prompt.text.lines().map(ToOwned::to_owned)),
        );
    let body = match &prompt.field {
        Some(field) => {
            dialog = dialog.child(crate::textfield::text_field(
                "plan-prompt-field",
                field,
                focus,
                focused,
                px(310.0),
                on_key,
            ));
            dialog.child(buttons).into_any_element()
        }
        None => dialog
            .child(buttons)
            .id("plan-prompt-keys")
            .track_focus(focus)
            .on_key_down(on_key)
            .into_any_element(),
    };

    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("plan-prompt-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(body),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

/// The menu and the dialog, for the planning screen to draw over itself.
pub fn overlays(
    menus: &PlanMenus,
    focus: &gpui::FocusHandle,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    map_menu(menus, window, cx)
        .into_iter()
        .chain(prompt_dialog(menus, focus, window, cx))
        .collect()
}

/// What a `.waypoints` text holds, for the facts: every record's `seq:frame:command`, and record
/// 0's latitude, longitude and altitude as the file has them.
#[must_use]
pub fn written_facts(file: &str) -> (String, String) {
    let records: Vec<Vec<&str>> = file
        .lines()
        .skip(1)
        .map(|line| line.split('\t').collect())
        .collect();
    fn field<'a>(fields: &[&'a str], index: usize) -> &'a str {
        fields.get(index).copied().unwrap_or("")
    }
    let summary = records
        .iter()
        .map(|fields| {
            format!(
                "{}:{}:{}",
                field(fields, 0),
                field(fields, 2),
                field(fields, 3)
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    let home = records
        .first()
        .and_then(|fields| fields.get(8..11))
        .map_or_else(String::new, |position| position.join(","));
    (summary, home)
}

/// Facts a UI test asserts on for this screen: what the mission holds, item by item, and what the
/// menu and its dialogs are doing.
pub fn record_facts(plan: &Plan, menus: &PlanMenus) {
    use crate::facts::record;
    let items = plan.items();
    record(
        "mission.commands",
        items
            .iter()
            .map(|item| item.command.to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
    // Home: whether the Home Location boxes hold one a write would take, and what they hold.
    record("mission.home", plan.home().is_some());
    record(
        "plan.home",
        HomeBox::ALL
            .iter()
            .map(|which| plan.home_text(*which))
            .collect::<Vec<_>>()
            .join(","),
    );
    // What Save File would write, read from the text it would write, so a test proves home is
    // item 0 and the first row item 1 on the path the product takes.
    let (written, written_home) = written_facts(&plan.waypoints_file());
    record("mission.written", written);
    record("mission.written.home", written_home);
    record(
        "mission.selected",
        plan.selected()
            .map_or_else(|| "none".to_owned(), |seq| seq.to_string()),
    );
    let last = items.last();
    record(
        "mission.last.command",
        last.map_or_else(|| "none".to_owned(), |item| item.command.to_string()),
    );
    record("mission.last.p1", last.map_or(0.0, |item| item.param1));
    record("mission.last.p2", last.map_or(0.0, |item| item.param2));
    record("mission.last.alt", last.map_or(0.0, |item| item.z));
    record(
        "mission.last.position",
        last.and_then(|item| item.position().ok().flatten())
            .is_some(),
    );
    record("survey.points", plan.polygon().len());
    record("plan.draw", plan.draw_mode().id());
    record(
        "plan.menu",
        if menus.open.is_some() {
            "open"
        } else {
            "closed"
        },
    );
    record(
        "plan.menu.submenu",
        menus
            .open
            .and_then(|menu| menu.submenu)
            .and_then(|index| MAP_MENU.get(index))
            .map_or("none", |entry| entry.id),
    );
    record("plan.menu.delete", menus.delete_enabled());
    record(
        "plan.prompt",
        menus.prompt.as_ref().map_or("none", |prompt| {
            if prompt.title.is_empty() {
                "message"
            } else {
                prompt.title
            }
        }),
    );
    record(
        "plan.prompt.text",
        menus
            .prompt
            .as_ref()
            .map_or("", |prompt| prompt.text.as_str()),
    );
    record(
        "plan.prompt.value",
        menus.prompt.as_ref().map_or("", Prompt::value),
    );
    record("plan.measure", menus.measure_from.is_some());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three frames are the three MAV_FRAME values the C# enum names.
    #[test]
    fn the_altitude_frames_are_the_mav_frame_values_they_claim() {
        // C#: Relative = GLOBAL_RELATIVE_ALT (3), Absolute = GLOBAL (0),
        //     Terrain = GLOBAL_TERRAIN_ALT (10).
        assert_eq!(AltitudeFrame::Relative.mav_frame(), 3);
        assert_eq!(AltitudeFrame::Absolute.mav_frame(), 0);
        assert_eq!(AltitudeFrame::Terrain.mav_frame(), 10);
    }

    /// Every frame survives the trip out to a number and back.
    #[test]
    fn a_frame_round_trips_through_its_mav_frame_value() {
        for frame in AltitudeFrame::all() {
            assert_eq!(
                AltitudeFrame::from_mav_frame(frame.mav_frame()),
                Some(frame),
                "{} did not round trip",
                frame.label()
            );
        }
    }

    /// And through the settings file, which stores a name rather than a number.
    #[test]
    fn a_frame_round_trips_through_the_settings_key() {
        for frame in AltitudeFrame::all() {
            assert_eq!(AltitudeFrame::from_key(frame.key()), frame);
        }
    }

    /// A settings file hand-edited into nonsense gives the frame every mission uses.
    ///
    /// Not terrain: a vehicle without terrain data refuses a terrain mission at upload, long
    /// after the choice was made, and nobody would connect the refusal to a typo in a config file.
    #[test]
    fn an_unrecognised_frame_name_falls_back_to_relative() {
        for nonsense in ["", "TERRAIN", "3", "above ground", "🙂"] {
            assert_eq!(AltitudeFrame::from_key(nonsense), AltitudeFrame::Relative);
        }
    }

    /// A waypoint is created in the frame the screen is set to.
    #[test]
    fn a_waypoint_takes_the_chosen_frame() {
        for frame in AltitudeFrame::all() {
            let mut plan = Plan::default();
            plan.add_waypoint_in(
                LatLon::new(-35.363_262, 149.165_237).expect("valid"),
                50.0,
                frame,
            );
            assert_eq!(
                plan.items().first().map(|item| item.frame),
                Some(frame.mav_frame()),
                "a waypoint added in {} did not get that frame",
                frame.label()
            );
        }
    }

    /// A terrain mission survives being written to a file and read back.
    ///
    /// The frame is column three of the QGC WPL format, and losing it turns a mission that clears
    /// a hill into one that flies at the same number above home.
    #[test]
    fn a_terrain_frame_survives_a_waypoints_file() {
        let mut plan = Plan::default();
        for (latitude, longitude) in [(-35.36, 149.16), (-35.35, 149.17)] {
            plan.add_waypoint_in(
                LatLon::new(latitude, longitude).expect("valid"),
                80.0,
                AltitudeFrame::Terrain,
            );
        }

        let text = plan.waypoints_file();
        assert!(
            text.lines()
                .skip(2)
                .all(|line| line.split('\t').nth(2) == Some("10")),
            "the terrain frame should be column three of every row after home:\n{text}"
        );

        let read = mp_mission::read_waypoints(&text).expect("our own output must parse");
        let (_, rows) = mp_mission::rows::split_home(&read);
        assert_eq!(rows.len(), plan.items().len());
        assert!(
            rows.iter().all(|item| item.frame == FRAME_TERRAIN),
            "the frame did not survive the round trip"
        );
    }

    /// An item in a frame the enum does not name is shown as its number, not as relative.
    ///
    /// A mission read back from a vehicle can hold anything; showing nothing for an unknown frame
    /// tells the operator it is relative when it is not.
    #[test]
    fn an_unknown_frame_is_shown_rather_than_hidden() {
        assert_eq!(frame_label(FRAME_RELATIVE), "relative");
        assert_eq!(frame_label(FRAME_ABSOLUTE), "absolute");
        assert_eq!(frame_label(FRAME_TERRAIN), "terrain");
        assert_eq!(frame_label(1), "frame 1");
        assert_eq!(frame_label(255), "frame 255");
    }

    /// A still click on empty map adds a waypoint. This is the whole feature.
    #[test]
    fn a_still_click_on_empty_map_adds_a_waypoint() {
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (100.0, 100.0)),
            MapRelease::AddWaypoint
        );
    }

    /// A drag does not. Moving the map is the commonest thing anybody does to it, and a drag that
    /// dropped a waypoint every time would make the map unusable.
    #[test]
    fn a_drag_does_not_add_a_waypoint() {
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (400.0, 260.0)),
            MapRelease::Nothing
        );
        // Either axis is enough.
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (100.0, 260.0)),
            MapRelease::Nothing
        );
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (400.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// Releasing over a waypoint that was grabbed adds nothing - "cant add WP in existing rect".
    #[test]
    fn releasing_a_grabbed_waypoint_adds_nothing() {
        assert_eq!(
            map_release(true, Some(3), Some((100.0, 100.0)), (100.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// The flight screen's map is for watching, not editing.
    #[test]
    fn a_click_outside_the_planning_screen_adds_nothing() {
        assert_eq!(
            map_release(false, None, Some((100.0, 100.0)), (100.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// A release with no press behind it is a button that went down somewhere else.
    ///
    /// Dragging in from off the map, or a press the window never saw. Adding a waypoint for it
    /// puts one wherever a stray release landed.
    #[test]
    fn a_release_with_no_press_adds_nothing() {
        assert_eq!(
            map_release(true, None, None, (100.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// The slop is a tolerance, not a licence: inside it is a click, outside it is a drag.
    #[test]
    fn the_boundary_of_the_click_tolerance_is_where_it_says_it_is() {
        let press = Some((100.0, 100.0));
        // Exactly at the tolerance is still a click - the test is `>`, not `>=`.
        let at = 100.0 + CLICK_SLOP;
        assert_eq!(
            map_release(true, None, press, (at, 100.0)),
            MapRelease::AddWaypoint
        );
        // A hair past it is a drag.
        let past = 100.0 + CLICK_SLOP + 0.5;
        assert_eq!(
            map_release(true, None, press, (past, 100.0)),
            MapRelease::Nothing
        );
        // And it works in the negative direction too, which an `abs()` that went missing would
        // break silently in one direction only.
        let back = 100.0 - CLICK_SLOP - 0.5;
        assert_eq!(
            map_release(true, None, press, (back, 100.0)),
            MapRelease::Nothing
        );
    }

    /// Three clicks make three waypoints, in the order they were clicked.
    ///
    /// The decision and the effect together, because "the click was recognised" and "the mission
    /// grew by one item at that position" are different claims.
    #[test]
    fn clicking_three_times_builds_a_three_item_mission() {
        let mut plan = Plan::default();
        let points = [
            (-35.363_262, 149.165_237),
            (-35.362_000, 149.166_000),
            (-35.361_000, 149.167_000),
        ];
        for (latitude, longitude) in points {
            let press = Some((10.0, 10.0));
            assert_eq!(
                map_release(true, None, press, (10.0, 10.0)),
                MapRelease::AddWaypoint
            );
            plan.add_waypoint(
                LatLon::new(latitude, longitude).expect("a valid position"),
                50.0,
            );
        }

        assert_eq!(plan.items().len(), 3);
        for (index, (latitude, longitude)) in points.iter().enumerate() {
            let item = &plan.items()[index];
            // Waypoint 1 is the first click: the grid numbers from 1, because home is 0.
            assert_eq!(u16::try_from(index + 1).unwrap_or(0), item.seq, "sequence");
            assert!((item.x - latitude).abs() < 1e-9, "latitude of item {index}");
            assert!(
                (item.y - longitude).abs() < 1e-9,
                "longitude of item {index}"
            );
            assert_eq!(item.frame, FRAME_RELATIVE);
            assert_eq!(item.command, CMD_WAYPOINT);
        }
    }

    fn at(lat: f64, lon: f64) -> LatLon {
        LatLon::new(lat, lon).expect("valid position")
    }

    #[test]
    fn a_new_plan_is_empty_and_says_so() {
        let plan = Plan::default();
        assert!(plan.is_empty());
        assert_eq!(*plan.origin(), Origin::Empty);
        assert_eq!(plan.origin().label(), "empty");
    }

    /// The rows are numbered as the grid's headers read, from 1: home is 0 and is not a row.
    #[test]
    fn waypoints_are_numbered_from_one_without_gaps() {
        let mut plan = Plan::default();
        for n in 0..4 {
            plan.add_waypoint(at(-35.36 + f64::from(n) * 0.001, 149.16), 50.0);
        }
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3, 4]);
    }

    #[test]
    fn deleting_renumbers_so_the_vehicle_will_accept_the_mission() {
        // A mission with a gap in its sequence is rejected at upload time, minutes after the edit
        // that caused it and with no indication of which one it was.
        let mut plan = Plan::default();
        for n in 0..4 {
            plan.add_waypoint(at(-35.36 + f64::from(n) * 0.001, 149.16), 50.0);
        }
        plan.remove(2);
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3]);
        assert_eq!(plan.items().len(), 3);
    }

    #[test]
    fn deleting_the_selected_item_clears_the_selection() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);
        plan.select(Some(1));
        plan.remove(1);
        assert_eq!(plan.selected(), None);
    }

    #[test]
    fn moving_an_item_reorders_positions_not_just_numbers() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.360, 149.16), 50.0);
        plan.add_waypoint(at(-35.361, 149.16), 50.0);
        plan.add_waypoint(at(-35.362, 149.16), 50.0);

        plan.move_item(3, -1);

        let latitudes: Vec<f64> = plan.items().iter().map(|i| i.x).collect();
        assert!((latitudes[1] - -35.362).abs() < 1e-9, "{latitudes:?}");
        assert!((latitudes[2] - -35.361).abs() < 1e-9, "{latitudes:?}");
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3]);
        assert_eq!(plan.selected(), Some(2), "the moved row stays selected");
    }

    #[test]
    fn moving_past_either_end_does_nothing() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);

        plan.move_item(1, -1);
        plan.move_item(2, 1);

        let latitudes: Vec<f64> = plan.items().iter().map(|i| i.x).collect();
        assert!((latitudes[0] - -35.36).abs() < 1e-9, "{latitudes:?}");
        assert!((latitudes[1] - -35.37).abs() < 1e-9, "{latitudes:?}");
    }

    #[test]
    fn moving_an_item_that_is_not_there_is_ignored() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.move_item(99, 1);
        assert_eq!(plan.items().len(), 1);
    }

    #[test]
    fn an_edit_marks_the_plan_as_no_longer_the_vehicles() {
        // The header line is the only thing telling the operator that what they see is not what
        // the aircraft holds, so it has to change the instant an edit happens.
        let mut plan = Plan::default();
        let _ = plan.adopt_from_vehicle(&[]);
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn a_loaded_plan_names_its_file() {
        let mut plan = Plan::default();
        let _ = plan.adopt_from_file("survey.waypoints", &[]);
        assert_eq!(plan.origin().label(), "loaded from survey.waypoints");
    }

    #[test]
    fn adopting_clears_a_stale_selection() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.select(Some(1));
        let _ = plan.adopt_from_vehicle(&[]);
        assert_eq!(plan.selected(), None);
    }

    #[test]
    fn map_clicks_produce_items_a_vehicle_will_fly() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.363, 149.165), DEFAULT_ALTITUDE);
        let item = plan.items().first().expect("one item");
        assert_eq!(item.command, CMD_WAYPOINT);
        // Relative to home, not above the ellipsoid: an operator typing 50 means 50 above where
        // they are standing, and the difference in Canberra is about 600 metres.
        assert_eq!(item.frame, FRAME_RELATIVE);
        assert_eq!(item.autocontinue, 1);
        assert_eq!(item.current, 0);
    }

    #[test]
    fn command_labels_are_readable_and_never_guessed() {
        assert_eq!(command_label(16), "waypoint");
        assert_eq!(command_label(22), "takeoff");
        assert_eq!(command_label(20), "return_to_launch");
        // An unknown command shows its number rather than a wrong name.
        assert_eq!(command_label(60_000), "cmd 60000");
    }

    #[test]
    fn stepping_an_altitude_changes_only_that_item() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);

        plan.nudge_altitude(2, 10.0);

        assert!((plan.items()[0].z - 50.0).abs() < 1e-9);
        assert!((plan.items()[1].z - 60.0).abs() < 1e-9);
    }

    #[test]
    fn a_held_stepper_cannot_walk_a_waypoint_into_the_stratosphere() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for _ in 0..500 {
            plan.nudge_altitude(1, 10.0);
        }
        assert!((plan.items()[0].z - MAX_STEPPED_ALTITUDE).abs() < 1e-9);
    }

    #[test]
    fn a_waypoint_below_home_is_allowed() {
        // Meaningful on a vehicle launched from a cliff or a rooftop. Refusing it would be the
        // ground station deciding it knows the terrain better than the operator.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 5.0);
        plan.nudge_altitude(1, -10.0);
        assert!(plan.items()[0].z < 0.0);
    }

    #[test]
    fn changing_to_a_positionless_command_clears_the_coordinates() {
        // Leaving stale coordinates on a command that ignores them is how a mission looks right
        // on the map and flies somewhere else: the map would keep drawing a waypoint the vehicle
        // has no intention of visiting.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(1, CMD_RTL);
        assert!((plan.items()[0].x).abs() < f64::EPSILON);
        assert!((plan.items()[0].y).abs() < f64::EPSILON);
        assert_eq!(plan.items()[0].command, CMD_RTL);
    }

    #[test]
    fn changing_to_a_positioned_command_keeps_the_coordinates() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(1, 22);
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
        assert_eq!(plan.items()[0].command, 22);
    }

    #[test]
    fn editing_an_item_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        let _ = plan.adopt_from_vehicle(&downloaded(&[MissionItem {
            seq: 1,
            current: 0,
            frame: FRAME_RELATIVE,
            command: CMD_WAYPOINT,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -35.36,
            y: 149.16,
            z: 50.0,
            autocontinue: 1,
        }]));
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.nudge_altitude(1, 10.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn editing_an_item_that_is_not_there_is_ignored() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.nudge_altitude(99, 10.0);
        plan.set_command(99, 22);
        assert!((plan.items()[0].z - 50.0).abs() < 1e-9);
        assert_eq!(plan.items()[0].command, CMD_WAYPOINT);
    }

    #[test]
    fn every_offered_command_has_a_readable_label() {
        // The chips in the editor are how a command is chosen; one showing a bare number would be
        // unusable.
        for (command, label) in EDITABLE_COMMANDS {
            assert!(!label.is_empty());
            assert!(
                !command_label(*command).starts_with("cmd "),
                "command {command} has no name in this dialect"
            );
        }
    }

    #[test]
    fn the_altitude_steps_are_coarse_and_fine_in_both_directions() {
        let deltas: Vec<f64> = ALTITUDE_STEPS.iter().map(|(delta, _)| *delta).collect();
        assert!(deltas.iter().any(|d| *d > 0.0));
        assert!(deltas.iter().any(|d| *d < 0.0));
        // Names must be unique: they address the controls a test script clicks.
        let mut names: Vec<&str> = ALTITUDE_STEPS.iter().map(|(_, name)| *name).collect();
        names.sort_unstable();
        let count = names.len();
        names.dedup();
        assert_eq!(names.len(), count, "duplicate stepper names");
    }

    /// A square about 300 m on a side near Canberra.
    fn area() -> Vec<LatLon> {
        vec![
            at(-35.3640, 149.1640),
            at(-35.3640, 149.1680),
            at(-35.3610, 149.1680),
            at(-35.3610, 149.1640),
        ]
    }

    #[test]
    fn clicks_build_the_area_in_the_order_they_are_made() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        assert_eq!(plan.polygon().len(), 4);
        assert!((plan.polygon()[0].latitude() - -35.3640).abs() < 1e-9);
    }

    #[test]
    fn undo_removes_the_last_vertex_drawn() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.undo_area_vertex();
        assert_eq!(plan.polygon().len(), 3);
        assert!((plan.polygon()[2].latitude() - -35.3610).abs() < 1e-9);
    }

    #[test]
    fn undo_on_an_empty_area_does_not_panic() {
        let mut plan = Plan::default();
        plan.undo_area_vertex();
        assert!(plan.polygon().is_empty());
    }

    #[test]
    fn a_survey_covers_the_area_it_was_drawn_over() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.adjust_survey(0.0, 0.0, 0.0);
        plan.generate_survey();

        assert!(plan.survey_error().is_none(), "{:?}", plan.survey_error());
        assert!(
            plan.items().len() >= 4,
            "a 300 m square at default spacing should need several lines, got {}",
            plan.items().len()
        );
        // Every generated point must be inside the area, or the aircraft flies outside what the
        // operator drew.
        for item in plan.items() {
            let position = item
                .position()
                .expect("a valid position")
                .expect("a position");
            assert!(
                mp_mission::survey::contains_within(plan.polygon(), position, 5.0),
                "generated {position:?} outside the drawn area"
            );
        }
    }

    #[test]
    fn a_survey_uses_the_chosen_altitude_and_a_relative_frame() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.adjust_survey(0.0, 0.0, 25.0);
        let expected = plan.survey_options().altitude;
        plan.generate_survey();

        assert!(!plan.items().is_empty());
        for item in plan.items() {
            assert!((item.z - expected).abs() < 1e-9, "{} vs {expected}", item.z);
            assert_eq!(item.frame, FRAME_RELATIVE);
            assert_eq!(item.command, CMD_WAYPOINT);
        }
    }

    #[test]
    fn a_survey_replaces_the_mission_rather_than_appending_to_it() {
        // A survey joined onto an existing mission would fly the old waypoints first and then
        // transit to the area, which is almost never what was meant.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.30, 149.10), 50.0);
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.generate_survey();

        let strays = plan
            .items()
            .iter()
            .filter(|item| (item.x - -35.30).abs() < 1e-9)
            .count();
        assert_eq!(strays, 0, "the old waypoint survived the survey");
    }

    #[test]
    fn too_few_corners_reports_why_rather_than_generating_nothing() {
        let mut plan = Plan::default();
        plan.add_area_vertex(at(-35.36, 149.16));
        plan.add_area_vertex(at(-35.36, 149.17));
        plan.generate_survey();

        assert!(plan.items().is_empty());
        let error = plan.survey_error().expect("an explanation");
        assert!(error.contains('3'), "{error}");
    }

    #[test]
    fn spacing_cannot_be_set_low_enough_to_generate_an_unflyable_mission() {
        // At one metre spacing a field-sized area generates tens of thousands of waypoints, which
        // no autopilot accepts and no operator intended.
        let mut plan = Plan::default();
        for _ in 0..100 {
            plan.adjust_survey(-100.0, 0.0, 0.0);
        }
        assert!(plan.survey_options().spacing >= 5.0);
    }

    #[test]
    fn the_survey_angle_wraps_rather_than_sticking_at_one_end() {
        // A bearing is circular; stopping at 359 would be arbitrary.
        let mut plan = Plan::default();
        plan.adjust_survey(0.0, -15.0, 0.0);
        assert!((plan.survey_options().angle - 345.0).abs() < 1e-9);
        plan.adjust_survey(0.0, 30.0, 0.0);
        assert!((plan.survey_options().angle - 15.0).abs() < 1e-9);
    }

    #[test]
    fn changing_the_angle_changes_the_pattern() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.generate_survey();
        let north_south: Vec<f64> = plan.items().iter().map(|item| item.x).collect();

        plan.adjust_survey(0.0, 90.0, 0.0);
        plan.generate_survey();
        let east_west: Vec<f64> = plan.items().iter().map(|item| item.x).collect();

        assert_ne!(north_south, east_west, "the angle had no effect");
    }

    #[test]
    fn clearing_the_area_leaves_the_mission_alone() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.clear_area();
        assert!(plan.polygon().is_empty());
        assert_eq!(plan.items().len(), 1);
    }

    #[test]
    fn drawing_mode_decides_what_a_map_click_does() {
        let mut plan = Plan::default();
        assert_eq!(plan.draw_mode(), DrawMode::Waypoints);
        plan.set_draw_mode(DrawMode::Area);
        assert_eq!(plan.draw_mode(), DrawMode::Area);
    }

    #[test]
    fn dragging_a_waypoint_moves_only_its_position() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 75.0);
        plan.move_to(1, at(-35.37, 149.17));

        let item = &plan.items()[0];
        assert!((item.x - -35.37).abs() < 1e-9);
        assert!((item.y - 149.17).abs() < 1e-9);
        // Altitude and command survive the move: dragging on a map says where, not what or how
        // high.
        assert!((item.z - 75.0).abs() < 1e-9);
        assert_eq!(item.command, CMD_WAYPOINT);
    }

    #[test]
    fn a_command_with_no_position_cannot_be_dragged_somewhere() {
        // Writing coordinates into a return-to-launch would make the map draw a waypoint the
        // vehicle has no intention of visiting.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(1, CMD_RTL);
        plan.move_to(1, at(-35.37, 149.17));

        assert!((plan.items()[0].x).abs() < f64::EPSILON);
        assert!((plan.items()[0].y).abs() < f64::EPSILON);
    }

    #[test]
    fn dragging_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        let _ = plan.adopt_from_vehicle(&downloaded(plan.items()));
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.move_to(1, at(-35.37, 149.17));
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn dragging_an_item_that_is_not_there_is_ignored() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.move_to(99, at(0.0, 0.0));
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
    }

    #[test]
    fn a_fence_needs_three_corners_before_it_can_be_sent() {
        // Fewer encloses nothing, and both the protocol and the vehicle refuse it. Saying why is
        // better than a write button that fails silently.
        let mut plan = Plan::default();
        plan.add_fence_vertex(at(-35.364, 149.164));
        plan.add_fence_vertex(at(-35.364, 149.168));
        assert!(plan.fence_items().is_none());
        let error = plan.fence_error().expect("an explanation");
        assert!(error.contains('3'), "{error}");

        plan.add_fence_vertex(at(-35.361, 149.168));
        let items = plan.fence_items().expect("three corners should be enough");
        assert_eq!(items.len(), 3);
        assert!(plan.fence_error().is_none());
    }

    #[test]
    fn a_drawn_fence_becomes_inclusion_polygon_items() {
        // Inclusion: the vehicle must stay inside. An exclusion fence drawn by mistake would do
        // nothing useful, or trigger a return-to-launch in flight.
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_fence_vertex(vertex);
        }
        let items = plan.fence_items().expect("a valid fence");
        assert_eq!(items.len(), 4);
        for item in &items {
            assert_eq!(item.command, mp_mission::fence::CMD_FENCE_POLYGON_INCLUSION);
            // Every vertex declares the total, which is how the run is delimited on the wire.
            assert!(
                (item.param1 - 4.0).abs() < f64::EPSILON,
                "{:?}",
                item.param1
            );
        }
    }

    #[test]
    fn a_fence_round_trips_through_the_items_the_protocol_carries() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_fence_vertex(vertex);
        }
        let items = plan.fence_items().expect("a valid fence");

        let mut received = Plan::default();
        received.adopt_fence(&items);
        assert_eq!(received.fence().len(), 4);
        for (sent, got) in area().iter().zip(received.fence()) {
            assert!((sent.latitude() - got.latitude()).abs() < 1e-9);
            assert!((sent.longitude() - got.longitude()).abs() < 1e-9);
        }
    }

    #[test]
    fn undoing_and_clearing_the_fence_leave_the_mission_alone() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for vertex in area() {
            plan.add_fence_vertex(vertex);
        }

        plan.undo_fence_vertex();
        assert_eq!(plan.fence().len(), 3);
        plan.clear_fence();
        assert!(plan.fence().is_empty());
        assert_eq!(plan.items().len(), 1, "the mission should be untouched");
    }

    #[test]
    fn the_fence_and_the_survey_area_are_separate_shapes() {
        // They are both polygons drawn by clicking, and confusing them would send a survey
        // boundary to the vehicle as a fence.
        let mut plan = Plan::default();
        plan.set_draw_mode(DrawMode::Area);
        plan.add_area_vertex(at(-35.360, 149.160));
        plan.set_draw_mode(DrawMode::Fence);
        plan.add_fence_vertex(at(-35.370, 149.170));

        assert_eq!(plan.polygon().len(), 1);
        assert_eq!(plan.fence().len(), 1);
        assert!((plan.polygon()[0].latitude() - -35.360).abs() < 1e-9);
        assert!((plan.fence()[0].latitude() - -35.370).abs() < 1e-9);
    }

    #[test]
    fn adopting_a_vehicle_with_no_polygon_fence_says_so() {
        // Rather than showing an empty fence that looks like one was read successfully.
        let mut plan = Plan::default();
        plan.adopt_fence(&[]);
        assert!(plan.fence().is_empty());
        assert!(plan.fence_error().is_some());
    }

    #[test]
    fn a_rally_point_takes_the_altitude_on_screen() {
        // There is one altitude control on this screen and it is the survey's; a rally point that
        // silently used a different number would be a surprise in a failsafe.
        let mut plan = Plan::default();
        plan.adjust_survey(0.0, 0.0, 30.0);
        let expected = plan.survey_options().altitude;
        plan.add_rally_point(at(-35.3625, 149.1655));

        assert_eq!(plan.rally().len(), 1);
        assert!((plan.rally()[0].altitude - expected).abs() < 1e-9);
    }

    #[test]
    fn rally_points_round_trip_through_the_items_the_protocol_carries() {
        let mut plan = Plan::default();
        plan.add_rally_point(at(-35.3625, 149.1655));
        plan.add_rally_point(at(-35.3630, 149.1660));
        let items = plan.rally_items().expect("two points should send");
        assert_eq!(items.len(), 2);
        for (index, item) in items.iter().enumerate() {
            assert_eq!(item.command, mp_mission::fence::CMD_RALLY_POINT);
            assert_eq!(usize::from(item.seq), index);
        }

        let mut received = Plan::default();
        received.adopt_rally(&items);
        assert_eq!(received.rally().len(), 2);
        assert!((received.rally()[0].position.latitude() - -35.3625).abs() < 1e-9);
    }

    #[test]
    fn sending_no_rally_points_says_why_rather_than_sending_nothing() {
        // An empty upload would clear the vehicle's rally points, which is a different intent from
        // not having drawn any yet.
        let mut plan = Plan::default();
        assert!(plan.rally_items().is_none());
        assert!(plan.rally_error().is_some());
    }

    #[test]
    fn adopting_a_vehicle_with_no_rally_points_says_so() {
        let mut plan = Plan::default();
        plan.adopt_rally(&[]);
        assert!(plan.rally().is_empty());
        assert!(plan.rally_error().is_some());
    }

    #[test]
    fn the_four_drawing_modes_are_distinct_and_named() {
        // The ids address controls a test script clicks; two sharing one would make a click land
        // on the wrong mode.
        let mut ids: Vec<&str> = DrawMode::ALL.iter().map(|m| m.id()).collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count);

        for mode in DrawMode::ALL {
            assert!(!mode.label().is_empty());
            // A click places things, as AddWPToMap does; the right button is the menu's.
            assert!(mode.hint().starts_with("click the map"), "{}", mode.hint());
        }
    }

    #[test]
    fn the_shapes_do_not_share_storage() {
        // Four things are placed by clicking on the same map. Mixing any two would send one
        // to the vehicle as another.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.300, 149.100), 50.0);
        plan.add_area_vertex(at(-35.310, 149.110));
        plan.add_fence_vertex(at(-35.320, 149.120));
        plan.add_rally_point(at(-35.330, 149.130));

        assert_eq!(plan.items().len(), 1);
        assert_eq!(plan.polygon().len(), 1);
        assert_eq!(plan.fence().len(), 1);
        assert_eq!(plan.rally().len(), 1);
        assert!((plan.items()[0].x - -35.300).abs() < 1e-9);
        assert!((plan.polygon()[0].latitude() - -35.310).abs() < 1e-9);
        assert!((plan.fence()[0].latitude() - -35.320).abs() < 1e-9);
        assert!((plan.rally()[0].position.latitude() - -35.330).abs() < 1e-9);
    }

    #[test]
    fn command_parameters_are_labelled_from_the_definitions() {
        // Not from a table here. A mission editor that shows "param1" is one the operator has to
        // look the meaning up for, and the definitions already carry it.
        let waypoint = MavCmd(u32::from(CMD_WAYPOINT)).parameters();
        assert_eq!(waypoint[0].map(|(label, _)| label), Some("Hold"));
        assert_eq!(waypoint[0].map(|(_, units)| units), Some("s"));

        let loiter_time = MavCmd(19).parameters();
        assert_eq!(loiter_time[0].map(|(label, _)| label), Some("Time"));
        assert_eq!(loiter_time[0].map(|(_, units)| units), Some("s"));
    }

    #[test]
    fn a_command_that_uses_no_parameters_offers_none() {
        // Return-to-launch takes nothing. Four controls doing nothing would imply otherwise.
        let rtl = MavCmd(u32::from(CMD_RTL)).parameters();
        assert!(rtl.iter().all(Option::is_none), "{rtl:?}");
    }

    #[test]
    fn setting_a_parameter_changes_only_that_one() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_param(1, 0, 12.0);

        assert!((plan.items()[0].param1 - 12.0).abs() < f64::EPSILON);
        assert!(plan.items()[0].param2.abs() < f64::EPSILON);
        assert!(plan.items()[0].param3.abs() < f64::EPSILON);
        assert!(plan.items()[0].param4.abs() < f64::EPSILON);
        // And nothing about where or how high it is.
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
        assert!((plan.items()[0].z - 50.0).abs() < f64::EPSILON);
    }

    #[test]
    fn every_parameter_slot_is_reachable_and_reads_back() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for index in 0..4 {
            #[allow(clippy::cast_precision_loss)]
            let value = (index + 1) as f64;
            plan.set_param(1, index, value);
            assert!((plan.param(1, index).expect("a value") - value).abs() < f64::EPSILON);
        }
        // Out of range is ignored rather than wrapping onto another parameter.
        plan.set_param(1, 9, 99.0);
        assert!(plan.param(1, 9).is_none());
    }

    #[test]
    fn editing_a_parameter_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        let _ = plan.adopt_from_vehicle(&downloaded(plan.items()));
        plan.set_param(1, 0, 5.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn clearing_resets_everything() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.select(Some(1));
        plan.set_home_text(HomeBox::Lat, "-35.36".to_owned());
        plan.clear();
        assert!(plan.is_empty());
        assert_eq!(plan.selected(), None);
        assert_eq!(*plan.origin(), Origin::Empty);
        assert_eq!(
            plan.home_text(HomeBox::Lat),
            "-35.36",
            "home is the boxes', not the mission's"
        );
    }

    // ---- The map menu: each entry against its C# handler ----

    use mp_mission::commands as cmd;

    const CLICK: (f32, f32) = (400.0, 300.0);

    fn canberra() -> LatLon {
        at(-35.363_262, 149.165_237)
    }

    fn context() -> MenuContext {
        MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
        }
    }

    /// The home the vehicle in these tests has: 580 m above sea level.
    const VEHICLE_HOME: Home = Home {
        lat: -35.36,
        lng: 149.16,
        alt: 580.0,
    };

    /// Rows as a vehicle sends them back: its home at item 0, then the rows from 1.
    fn downloaded(rows: &[MissionItem]) -> Vec<MissionItem> {
        mp_mission::rows::upload_list(Some(VEHICLE_HOME), rows)
    }

    /// A plan as it arrives from a vehicle, the operator having said Yes to its home: home at
    /// 580 m in the boxes, and two waypoints at 100 in the grid.
    fn from_vehicle() -> Plan {
        let mut plan = Plan::default();
        let offer = plan.adopt_from_vehicle(&downloaded(&[
            cmd::waypoint(at(-35.361, 149.161), 100.0, FRAME_RELATIVE),
            cmd::waypoint(at(-35.362, 149.162), 100.0, FRAME_RELATIVE),
        ]));
        plan.reset_home_to(offer.expect("empty boxes are offered the vehicle's home"));
        plan
    }

    /// Opens the menu over `marker` at Canberra and chooses `action`.
    fn choose(plan: &mut Plan, menus: &mut PlanMenus, action: MenuAction, marker: Option<u16>) {
        menus.open_at(CLICK, canberra(), marker);
        menus.choose(plan, action, &context());
    }

    /// Types `value` into the open prompt, replacing what it offered, and presses OK.
    fn answer(plan: &mut Plan, menus: &mut PlanMenus, value: &str) {
        menus
            .prompt
            .as_mut()
            .and_then(|prompt| prompt.field.as_mut())
            .expect("a prompt with a field")
            .set(value);
        menus.submit(plan, &context());
    }

    fn commands(plan: &Plan) -> Vec<u16> {
        plan.items().iter().map(|item| item.command).collect()
    }

    fn last(plan: &Plan) -> MissionItem {
        *plan.items().last().expect("an item")
    }

    /// The constructors' command ids are the MAVLink ones the C#'s `MAV_CMD` names resolve to.
    #[test]
    fn the_menu_commands_are_the_mavlink_commands_the_c_sharp_names() {
        for (id, name) in [
            (cmd::WAYPOINT, "MAV_CMD_NAV_WAYPOINT"),
            (cmd::LOITER_UNLIM, "MAV_CMD_NAV_LOITER_UNLIM"),
            (cmd::LOITER_TURNS, "MAV_CMD_NAV_LOITER_TURNS"),
            (cmd::LOITER_TIME, "MAV_CMD_NAV_LOITER_TIME"),
            (cmd::RETURN_TO_LAUNCH, "MAV_CMD_NAV_RETURN_TO_LAUNCH"),
            (cmd::LAND, "MAV_CMD_NAV_LAND"),
            (cmd::TAKEOFF, "MAV_CMD_NAV_TAKEOFF"),
            (cmd::SPLINE_WAYPOINT, "MAV_CMD_NAV_SPLINE_WAYPOINT"),
            (cmd::DO_JUMP, "MAV_CMD_DO_JUMP"),
            (cmd::DO_SET_ROI, "MAV_CMD_DO_SET_ROI"),
        ] {
            assert_eq!(MavCmd(u32::from(id)).name(), Some(name), "command {id}");
        }
    }

    /// Delete WP is enabled over a marker and nowhere else, as `contextMenuStrip1_Opening` has it.
    #[test]
    fn delete_wp_is_enabled_only_over_a_marker() {
        let mut menus = PlanMenus::default();
        assert!(!menus.delete_enabled(), "nothing is open");
        menus.open_at(CLICK, canberra(), None);
        assert!(!menus.delete_enabled());
        menus.open_at(CLICK, canberra(), Some(2));
        assert!(menus.delete_enabled());
    }

    /// Delete WP removes the waypoint under the cursor and renumbers the rest.
    #[test]
    fn delete_wp_removes_the_waypoint_under_the_cursor() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::DeleteWp, Some(1));
        assert_eq!(plan.items().len(), 1);
        assert!(
            (plan.items()[0].x - -35.362).abs() < 1e-9,
            "the second waypoint moved up"
        );
        assert_eq!(plan.items()[0].seq, 1, "and is waypoint 1 now");
        assert!(menus.open.is_none(), "choosing closes the menu");
    }

    /// Home is not a row, so Delete WP never reaches it: nothing is numbered 0, and the boxes
    /// keep home.
    #[test]
    fn delete_wp_never_reaches_home() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::DeleteWp, Some(0));
        assert_eq!(plan.items().len(), 2);
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
    }

    /// Insert Wp offers the number after the selected waypoint, and accepting it puts a WAYPOINT
    /// there at the menu's position, in the screen's frame, at the default altitude.
    #[test]
    fn insert_wp_puts_a_waypoint_after_the_number_given() {
        let mut plan = from_vehicle();
        plan.select(Some(1));
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertWp, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(prompt.title, "Insert WP");
        assert_eq!(prompt.text, "Insert WP after wp#");
        assert_eq!(prompt.value(), "1", "(selectedrow + 1): after waypoint 1");
        menus.submit(&mut plan, &context());
        assert_eq!(commands(&plan), vec![16, 16, 16]);
        let inserted = plan.items()[1];
        assert_eq!(inserted.position(), Ok(Some(canberra())));
        assert!((inserted.z - 100.0).abs() < 1e-9, "the default altitude");
        assert_eq!(inserted.frame, FRAME_RELATIVE);
        assert_eq!(plan.selected(), Some(2), "the new row is selected");
    }

    /// With nothing selected the offer is after the last waypoint.
    #[test]
    fn insert_wp_offers_the_end_with_nothing_selected() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertWp, None);
        assert_eq!(menus.prompt.as_ref().map(Prompt::value), Some("2"));
        answer(&mut plan, &mut menus, "0");
        assert_eq!(plan.items().len(), 3);
        assert_eq!(
            plan.items()[0].position(),
            Ok(Some(canberra())),
            "0 is after home: waypoint 1"
        );
        assert_eq!(plan.items()[0].seq, 1);
    }

    /// A number `Rows.Insert` would throw on is refused in the C#'s words, and nothing changes.
    #[test]
    fn insert_wp_refuses_a_position_that_is_not_there() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertWp, None);
        answer(&mut plan, &mut menus, "9");
        assert_eq!(plan.items().len(), 2);
        let message = menus.prompt.as_ref().expect("a message");
        assert_eq!(message.title, "Error");
        assert_eq!(message.text, "Invalid insert position");
        assert_eq!(message.kind, PromptKind::Message);
    }

    /// Insert Spline WP puts a SPLINE_WAYPOINT, and refuses with Strings.InvalidNumberEntered.
    #[test]
    fn insert_spline_wp_puts_a_spline_waypoint() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertSplineWp, None);
        answer(&mut plan, &mut menus, "1");
        assert_eq!(commands(&plan), vec![16, 82, 16]);

        choose(&mut plan, &mut menus, MenuAction::InsertSplineWp, None);
        answer(&mut plan, &mut menus, "x");
        assert_eq!(plan.items().len(), 3);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some("Invalid number entered")
        );
    }

    /// Loiter > Forever appends LOITER_UNLIM at the click, no prompt.
    #[test]
    fn loiter_forever_appends_an_unlimited_loiter() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterForever, None);
        assert!(menus.prompt.is_none());
        let item = last(&plan);
        assert_eq!(item.command, 17);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 100.0).abs() < 1e-9);
    }

    /// Loiter > Time asks "Loiter Time", offering 5, and puts the answer in Param1.
    #[test]
    fn loiter_time_asks_for_a_time_and_puts_it_in_param1() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterTime, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str()),
            ("Loiter Time", "Loiter Time")
        );
        assert_eq!(prompt.value(), "5");
        answer(&mut plan, &mut menus, "12");
        let item = last(&plan);
        assert_eq!(item.command, 19);
        assert!((item.param1 - 12.0).abs() < 1e-9);
        assert_eq!(item.position(), Ok(Some(canberra())));
    }

    /// Loiter > Circles asks "Loiter Turns", offering 3.
    #[test]
    fn loiter_circles_asks_for_turns_and_puts_them_in_param1() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterCircles, None);
        assert_eq!(menus.prompt.as_ref().map(|p| p.title), Some("Loiter Turns"));
        assert_eq!(menus.prompt.as_ref().map(Prompt::value), Some("3"));
        menus.submit(&mut plan, &context());
        let item = last(&plan);
        assert_eq!(item.command, 18);
        assert!((item.param1 - 3.0).abs() < 1e-9);
    }

    /// Cancel is the handler's `return`: nothing is added.
    #[test]
    fn cancelling_a_prompt_adds_nothing() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterTime, None);
        menus.cancel();
        assert!(menus.prompt.is_none());
        assert_eq!(plan.items().len(), 2);
    }

    /// Jump > Start jumps to item 1 the number of times asked, offering 5.
    #[test]
    fn jump_start_jumps_to_item_one() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::JumpStart, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("Jump repeat", "Number of times to Repeat", "5")
        );
        menus.submit(&mut plan, &context());
        let item = last(&plan);
        assert_eq!(item.command, 177);
        assert!((item.param1 - 1.0).abs() < 1e-9);
        assert!((item.param2 - 5.0).abs() < 1e-9);
        assert_eq!(item.position(), Ok(None));
    }

    /// Jump > WP # asks for the target, offering 1, then the repeat count.
    #[test]
    fn jump_wp_asks_for_the_target_then_the_repeat() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::JumpWp, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("WP No", "Jump to WP no?", "1")
        );
        answer(&mut plan, &mut menus, "2");
        assert_eq!(menus.prompt.as_ref().map(|p| p.title), Some("Jump repeat"));
        answer(&mut plan, &mut menus, "3");
        let item = last(&plan);
        assert_eq!(item.command, 177);
        assert!((item.param1 - 2.0).abs() < 1e-9);
        assert!((item.param2 - 3.0).abs() < 1e-9);
    }

    /// Text that is not a number is refused with Strings.InvalidNumberEntered.
    #[test]
    fn a_jump_that_is_not_a_number_is_refused() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::JumpStart, None);
        answer(&mut plan, &mut menus, "lots");
        assert_eq!(plan.items().len(), 2);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some("Invalid number entered")
        );
    }

    /// RTL appends RETURN_TO_LAUNCH and nothing else.
    #[test]
    fn rtl_appends_a_return_to_launch() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Rtl, None);
        let item = last(&plan);
        assert_eq!(item.command, 20);
        assert_eq!((item.x, item.y, item.z), (0.0, 0.0, 0.0));
        assert_eq!(item.frame, FRAME_RELATIVE);
    }

    /// Land appends LAND at the click, one metre up.
    #[test]
    fn land_appends_a_land_at_the_click() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Land, None);
        let item = last(&plan);
        assert_eq!(item.command, 21);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 1.0).abs() < 1e-9);
    }

    /// Takeoff asks for the altitude, offering 10 in metres, and writes it with no position.
    #[test]
    fn takeoff_asks_for_an_altitude() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Takeoff, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("Altitude", "Please enter your takeoff altitude in m", "10")
        );
        answer(&mut plan, &mut menus, "25");
        assert!(menus.prompt.is_none(), "not a plane: no pitch");
        let item = last(&plan);
        assert_eq!(item.command, 22);
        assert!((item.z - 25.0).abs() < 1e-9);
        assert!((item.param1 - 0.0).abs() < 1e-9);
        assert_eq!(item.position(), Ok(None));
    }

    /// On a plane it asks for the pitch too, offering 15, and puts it in Param1.
    #[test]
    fn takeoff_on_a_plane_asks_for_the_pitch() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        let plane = MenuContext {
            takeoff_pitch: true,
            ..context()
        };
        menus.open_at(CLICK, canberra(), None);
        menus.choose(&mut plan, MenuAction::Takeoff, &plane);
        menus.submit(&mut plan, &plane);
        let prompt = menus.prompt.as_ref().expect("the pitch");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("Takeoff Pitch", "Please enter your takeoff pitch", "15")
        );
        menus.submit(&mut plan, &plane);
        let item = last(&plan);
        assert_eq!(item.command, 22);
        assert!((item.z - 10.0).abs() < 1e-9);
        assert!((item.param1 - 15.0).abs() < 1e-9);
    }

    /// `int.TryParse` refuses a fractional altitude: "Bad Alt".
    #[test]
    fn a_takeoff_altitude_int_parse_refuses_is_a_bad_alt() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Takeoff, None);
        answer(&mut plan, &mut menus, "10.5");
        assert_eq!(plan.items().len(), 2);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some("Bad Alt")
        );
    }

    /// Which vehicles are asked for a pitch: ArduPlane's types, and a quadplane only with bit 1.
    #[test]
    fn only_a_plane_is_asked_for_a_takeoff_pitch() {
        let none: Vec<(String, f64)> = Vec::new();
        assert!(
            MenuContext::asks_takeoff_pitch(Some(1), &none),
            "fixed wing"
        );
        assert!(
            MenuContext::asks_takeoff_pitch(Some(16), &none),
            "flapping wing"
        );
        assert!(
            MenuContext::asks_takeoff_pitch(Some(20), &none),
            "VTOL quad"
        );
        assert!(
            !MenuContext::asks_takeoff_pitch(Some(2), &none),
            "quadrotor"
        );
        assert!(!MenuContext::asks_takeoff_pitch(Some(10), &none), "rover");
        assert!(!MenuContext::asks_takeoff_pitch(None, &none), "no vehicle");
        let without = vec![("Q_OPTIONS".to_owned(), 1.0)];
        let with = vec![("Q_OPTIONS".to_owned(), 2.0)];
        assert!(!MenuContext::asks_takeoff_pitch(Some(1), &without));
        assert!(MenuContext::asks_takeoff_pitch(Some(1), &with));
    }

    /// DO_SET_ROI appends a region of interest at the click, which is a position.
    #[test]
    fn set_roi_appends_a_region_of_interest() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::SetRoi, None);
        let item = last(&plan);
        assert_eq!(item.command, 201);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 100.0).abs() < 1e-9);
    }

    /// Clear Mission empties the rows and keeps home, which is not one of them.
    #[test]
    fn clear_mission_keeps_home() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ClearMission, None);
        assert!(plan.is_empty());
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
        // A plan drawn here has rows only, and they all go.
        let mut drawn = Plan::default();
        drawn.add_waypoint(canberra(), 50.0);
        drawn.add_waypoint(canberra(), 50.0);
        choose(&mut drawn, &mut menus, MenuAction::ClearMission, None);
        assert!(drawn.is_empty());
    }

    /// Reverse WPs turns the rows round; home is still what is written first.
    #[test]
    fn reverse_wps_turns_the_rows_round_and_home_stays_first() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ReverseWps, None);
        let latitudes: Vec<f64> = plan.items().iter().map(|item| item.x).collect();
        assert_eq!(latitudes, vec![-35.362, -35.361]);
        let seqs: Vec<u16> = plan.items().iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![1, 2]);
        let written = plan.vehicle_mission(true).expect("a home");
        let latitudes: Vec<f64> = written.iter().map(|item| item.x).collect();
        assert_eq!(latitudes, vec![-35.36, -35.362, -35.361]);
    }

    /// Draw a Polygon starts drawing the first time and adds the corner under the menu after.
    #[test]
    fn draw_a_polygon_starts_drawing_then_adds_corners() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::DrawPolygon, None);
        assert_eq!(plan.draw_mode(), DrawMode::Area);
        assert!(
            plan.polygon().is_empty(),
            "the first choice only starts drawing"
        );
        choose(&mut plan, &mut menus, MenuAction::DrawPolygon, None);
        assert_eq!(plan.polygon(), &[canberra()]);
        // And a click on the map adds a corner while drawing: AddWPToMap's polygongridmode.
        plan.add_wp_to_map(at(-35.37, 149.17), 50.0, AltitudeFrame::Relative);
        assert_eq!(plan.polygon().len(), 2);
        assert!(plan.is_empty(), "no waypoint was added");
    }

    /// Clear Polygon removes the corners and stops drawing.
    #[test]
    fn clear_polygon_removes_the_corners_and_stops_drawing() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        plan.set_draw_mode(DrawMode::Area);
        plan.add_area_vertex(canberra());
        choose(&mut plan, &mut menus, MenuAction::ClearPolygon, None);
        assert!(plan.polygon().is_empty());
        assert_eq!(plan.draw_mode(), DrawMode::Waypoints);
    }

    /// From Current Waypoints makes the polygon from the WAYPOINT rows and asks about clearing.
    #[test]
    fn from_current_waypoints_makes_the_polygon_and_asks() {
        let mut plan = from_vehicle();
        plan.append(cmd::loiter_time(
            at(-35.9, 149.9),
            50.0,
            FRAME_RELATIVE,
            5.0,
        ));
        let mut menus = PlanMenus::default();
        choose(
            &mut plan,
            &mut menus,
            MenuAction::PolygonFromWaypoints,
            None,
        );
        assert_eq!(
            plan.polygon(),
            &[at(-35.361, 149.161), at(-35.362, 149.162)],
            "the two waypoints, not home and not the loiter"
        );
        let question = menus.prompt.as_ref().expect("the question");
        assert_eq!(
            (question.title, question.text.as_str()),
            ("Confirm", "Clear current waypoints?")
        );
        assert!(question.is_question());
        // No keeps the mission.
        menus.cancel();
        assert_eq!(plan.items().len(), 3);
        // Yes clears it; home, not a row, stays.
        choose(
            &mut plan,
            &mut menus,
            MenuAction::PolygonFromWaypoints,
            None,
        );
        menus.submit(&mut plan, &context());
        assert!(plan.is_empty());
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
    }

    /// With no rows it does nothing and asks nothing.
    #[test]
    fn from_current_waypoints_with_no_rows_does_nothing() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(
            &mut plan,
            &mut menus,
            MenuAction::PolygonFromWaypoints,
            None,
        );
        assert!(menus.prompt.is_none());
        assert!(plan.polygon().is_empty());
    }

    /// Measure Distance: the first choice remembers the point, the second reports distance and
    /// bearing on GMap's 6378137 m sphere, in the C#'s words and formats.
    #[test]
    fn measure_distance_takes_two_choices() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        menus.open_at(CLICK, at(0.0, 0.0), None);
        menus.choose(&mut plan, MenuAction::MeasureDistance, &context());
        assert!(menus.measure_from.is_some());
        assert_eq!(menus.prompt.as_ref().map(|p| p.title), Some("Measure Dist"));
        menus.cancel();
        menus.open_at(CLICK, at(0.0, 1.0), None);
        menus.choose(&mut plan, MenuAction::MeasureDistance, &context());
        assert!(menus.measure_from.is_none(), "startmeasure is reset");
        // One degree of the equator on a 6378137 m sphere is 111319.490793 m.
        assert_eq!(
            menus.prompt.as_ref().map(|p| p.text.as_str()),
            Some("Distance: 111319.49 m AZ: 90")
        );
    }

    /// Modify Alt adds, or multiplies with a star, every row's altitude and leaves home alone.
    #[test]
    fn modify_alt_changes_every_row_and_not_home() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ModifyAlt, None);
        assert_eq!(menus.prompt.as_ref().map(Prompt::value), Some("0"));
        answer(&mut plan, &mut menus, "20");
        let altitudes: Vec<f64> = plan.items().iter().map(|item| item.z).collect();
        assert_eq!(altitudes, vec![120.0, 120.0]);
        choose(&mut plan, &mut menus, MenuAction::ModifyAlt, None);
        answer(&mut plan, &mut menus, "*2");
        let altitudes: Vec<f64> = plan.items().iter().map(|item| item.z).collect();
        assert_eq!(altitudes, vec![240.0, 240.0]);
        assert_eq!(plan.home_text(HomeBox::Alt), "580.00", "home is left alone");
    }

    /// Insert Wp > At Current Position adds a waypoint where the vehicle is, at its altitude.
    #[test]
    fn at_current_position_adds_a_waypoint_at_the_vehicle() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        let flying = MenuContext {
            vehicle: Some((at(-35.4, 149.2), 42.7)),
            ..context()
        };
        menus.open_at(CLICK, canberra(), None);
        menus.choose(&mut plan, MenuAction::InsertAtCurrentPosition, &flying);
        let item = last(&plan);
        assert_eq!(item.command, 16);
        assert_eq!(item.position(), Ok(Some(at(-35.4, 149.2))));
        assert!((item.z - 42.0).abs() < 1e-9, "(int) cs.alt");
        // Without a vehicle it says so and adds nothing.
        choose(
            &mut plan,
            &mut menus,
            MenuAction::InsertAtCurrentPosition,
            None,
        );
        assert_eq!(plan.items().len(), 3);
        assert_eq!(
            menus.prompt.as_ref().map(|p| p.text.as_str()),
            Some("Invalid Location")
        );
    }

    /// A click adds what is being drawn: waypoint, fence corner or rally point.
    #[test]
    fn a_click_adds_what_the_draw_mode_says() {
        let mut plan = Plan::default();
        plan.add_wp_to_map(canberra(), 60.0, AltitudeFrame::Terrain);
        assert_eq!(plan.items().len(), 1);
        assert_eq!(plan.items()[0].frame, FRAME_TERRAIN);
        plan.set_draw_mode(DrawMode::Fence);
        plan.add_wp_to_map(canberra(), 60.0, AltitudeFrame::Relative);
        assert_eq!(plan.fence().len(), 1);
        plan.set_draw_mode(DrawMode::Rally);
        plan.add_wp_to_map(canberra(), 60.0, AltitudeFrame::Relative);
        assert_eq!(plan.rally().len(), 1);
        assert_eq!(plan.items().len(), 1, "only the first click was a waypoint");
    }

    /// The default altitude copies the last item with a position, and never a zero from an RTL.
    #[test]
    fn the_default_altitude_skips_items_without_a_position() {
        let mut plan = Plan::default();
        assert!((plan.default_altitude() - DEFAULT_ALTITUDE).abs() < 1e-9);
        plan.append(cmd::waypoint(canberra(), 75.0, FRAME_RELATIVE));
        plan.append(cmd::return_to_launch(FRAME_RELATIVE));
        assert!((plan.default_altitude() - 75.0).abs() < 1e-9);
    }

    // ---- Home is item 0: the Home Location boxes against BUT_write_Click, saveWPs,
    // ---- savewaypoints, processToScreen and updateHomeText ----

    /// Types a home into the three boxes, as an operator does.
    fn type_home(plan: &mut Plan, lat: &str, lng: &str, alt: &str) {
        plan.set_home_text(HomeBox::Lat, lat.to_owned());
        plan.set_home_text(HomeBox::Lng, lng.to_owned());
        plan.set_home_text(HomeBox::Alt, alt.to_owned());
    }

    /// The defect this rule exists for: a mission drawn on an empty map, with no home anywhere,
    /// starts at waypoint 1. The first click is never item 0.
    #[test]
    fn the_first_click_on_an_empty_map_is_waypoint_one_not_home() {
        let mut plan = Plan::default();
        plan.add_wp_to_map(canberra(), 50.0, AltitudeFrame::Relative);
        plan.add_wp_to_map(at(-35.364, 149.166), 50.0, AltitudeFrame::Relative);
        assert_eq!(plan.items()[0].seq, 1);
        assert!(plan.items().iter().all(|item| !item.is_home()));
        assert_eq!(plan.home(), None, "nothing has set a home");

        // The file still starts with a home record - savewaypoints' blank one - so reading it
        // back gives both clicks back as rows.
        let file = plan.waypoints_file();
        let mut lines = file.lines();
        assert_eq!(lines.next(), Some("QGC WPL 110"));
        assert_eq!(lines.next(), Some(mp_mission::waypoints::BLANK_HOME_RECORD));
        assert!(
            lines
                .next()
                .is_some_and(|line| line.starts_with("1\t0\t3\t16\t")),
            "{file}"
        );
        let read = mp_mission::read_waypoints(&file).expect("parses");
        let mut again = Plan::default();
        assert_eq!(again.adopt_from_file("drawn.waypoints", &read), None);
        assert_eq!(again.items(), plan.items());
    }

    /// Write refuses without a home, in the C#'s words, whatever the autopilot.
    #[test]
    fn writing_without_a_home_is_refused() {
        let mut plan = Plan::default();
        plan.add_waypoint(canberra(), 50.0);
        assert_eq!(plan.vehicle_mission(true), Err(HOME_INVALID));
        assert_eq!(plan.vehicle_mission(false), Err(HOME_INVALID));
        assert_eq!(HOME_INVALID, "Your home location is invalid");
        // Any one box that does not parse is no home.
        type_home(&mut plan, "-35.363262", "149.165237", "high");
        assert_eq!(plan.vehicle_mission(true), Err(HOME_INVALID));
    }

    /// With a home in the boxes, what an ArduPilot is sent has it at item 0 - a GLOBAL waypoint
    /// at the boxes' position and altitude - and the rows after it from 1.
    #[test]
    fn a_written_mission_has_the_boxes_home_at_zero() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.361, 149.161), 50.0);
        plan.add_waypoint(at(-35.362, 149.162), 60.0);
        type_home(&mut plan, "-35.363262", "149.165237", "584.00");

        let sent = plan.vehicle_mission(true).expect("a home");
        assert_eq!(sent.len(), 3);
        let home = sent[0];
        assert_eq!((home.seq, home.frame, home.command), (0, 0, 16));
        assert_eq!((home.x, home.y, home.z), (-35.363_262, 149.165_237, 584.0));
        assert!(
            (sent[1].x - -35.361).abs() < 1e-12,
            "the first row is item 1"
        );
        assert_eq!(
            sent.iter().map(|item| item.seq).collect::<Vec<_>>(),
            vec![0, 1, 2]
        );

        // Any other autopilot is sent the rows alone.
        let sent = plan.vehicle_mission(false).expect("a home");
        assert_eq!(sent.len(), 2);
        assert!((sent[0].x - -35.361).abs() < 1e-12);

        // And the file has it as record 0, in savewaypoints' format.
        let file = plan.waypoints_file();
        assert_eq!(
            file.lines().nth(1),
            Some("0\t1\t0\t16\t0\t0\t0\t0\t-35.3632620\t149.1652370\t584.000000\t1")
        );
        assert_eq!(
            written_facts(&file),
            (
                "0:0:16,1:3:16,2:3:16".to_owned(),
                "-35.3632620,149.1652370,584.000000".to_owned()
            )
        );
    }

    /// What the facts say is written comes from the text Save File writes: with nothing planned,
    /// that is still a home record, the blank one.
    #[test]
    fn the_written_facts_read_the_file() {
        let plan = Plan::default();
        assert_eq!(
            written_facts(&plan.waypoints_file()),
            ("0:0:0".to_owned(), "0,0,0".to_owned())
        );
    }

    /// The boxes show the vehicle's home when it has sent one, else the planned home, else stay
    /// as they are - and whatever they are set to becomes the planned home.
    #[test]
    fn activating_shows_the_vehicles_home_else_the_planned_one() {
        let mut plan = Plan::default();
        plan.update_home_text(None);
        assert_eq!(plan.home(), None, "neither: the boxes stay empty");

        let planned = Home {
            lat: -27.509_708_302_151_7,
            lng: 153.015_389_442_444,
            alt: 8.1,
        };
        plan.set_planned_home(planned);
        plan.update_home_text(None);
        assert_eq!(plan.home_text(HomeBox::Lat), "-27.5097083021517");
        assert_eq!(plan.home_text(HomeBox::Lng), "153.015389442444");
        assert_eq!(plan.home_text(HomeBox::Alt), "8.10");

        // A vehicle's home at 0,0 is no home.
        plan.update_home_text(Some(Home::default()));
        assert_eq!(plan.home_text(HomeBox::Lat), "-27.5097083021517");

        plan.update_home_text(Some(VEHICLE_HOME));
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
        assert_eq!(plan.home_text(HomeBox::Alt), "580.00");
        assert_eq!(
            plan.planned_home(),
            VEHICLE_HOME,
            "TextChanged made it the planned home"
        );
    }

    /// Typing moves the planned home when the text parses and leaves it when it does not.
    #[test]
    fn a_box_that_parses_sets_the_planned_home() {
        let mut plan = Plan::default();
        type_home(&mut plan, "-35.5", "149.5", "600");
        assert_eq!(
            plan.planned_home(),
            Home {
                lat: -35.5,
                lng: 149.5,
                alt: 600.0
            }
        );
        plan.set_home_text(HomeBox::Lat, "-35.5x".to_owned());
        assert!((plan.planned_home().lat - -35.5).abs() < f64::EPSILON);
        assert_eq!(
            plan.home(),
            None,
            "but the write takes the box, which does not parse"
        );
    }

    /// Reading takes item 0 as home and never as a row, and offers it when the boxes differ.
    #[test]
    fn reading_a_mission_takes_item_zero_as_home() {
        let rows = [
            cmd::waypoint(at(-35.361, 149.161), 100.0, FRAME_RELATIVE),
            cmd::waypoint(at(-35.362, 149.162), 100.0, FRAME_RELATIVE),
        ];
        let mut plan = Plan::default();
        let offer = plan.adopt_from_file("x.waypoints", &downloaded(&rows));
        assert_eq!(offer, Some(VEHICLE_HOME), "empty boxes: asked");
        assert_eq!(plan.items().len(), 2);
        assert_eq!(plan.items()[0].seq, 1);
        assert_eq!(plan.home(), None, "not taken until the operator says Yes");

        // The boxes already say that latitude: not asked.
        plan.reset_home_to(VEHICLE_HOME);
        assert_eq!(plan.adopt_from_vehicle(&downloaded(&rows)), None);

        // A blank home - latitude 0 - is never offered.
        let mut blank = downloaded(&rows);
        blank[0] = MissionItem::default();
        assert_eq!(Plan::default().adopt_from_file("x", &blank), None);
    }

    /// Yes to "Reset Home to loaded coords" puts the loaded home in the boxes; No leaves them.
    #[test]
    fn yes_to_reset_home_takes_the_loaded_home() {
        let mut plan = Plan::default();
        type_home(&mut plan, "-35.5", "149.5", "600");
        let mut menus = PlanMenus::default();
        let offer = plan
            .adopt_from_vehicle(&downloaded(&[cmd::waypoint(canberra(), 50.0, 3)]))
            .expect("the boxes say otherwise");
        menus.offer_home_reset(offer);
        let question = menus.prompt.as_ref().expect("asked");
        assert_eq!(
            (question.title, question.text.as_str()),
            ("Reset Home Coords", "Reset Home to loaded coords")
        );
        assert!(question.is_question());
        menus.cancel();
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.5", "No");

        menus.offer_home_reset(offer);
        menus.submit(&mut plan, &context());
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.36");
        assert_eq!(plan.home_text(HomeBox::Lng), "149.16");
        assert_eq!(plan.home_text(HomeBox::Alt), "580.00");
    }

    /// Saving and reading back gives the same rows and the same home, and asks nothing.
    #[test]
    fn a_saved_mission_reads_back_whole() {
        let mut plan = Plan::default();
        plan.add_waypoint_in(at(-35.361, 149.161), 50.0, AltitudeFrame::Terrain);
        plan.append(cmd::return_to_launch(FRAME_RELATIVE));
        type_home(&mut plan, "-35.363262", "149.165237", "584.25");
        let read = mp_mission::read_waypoints(&plan.waypoints_file()).expect("parses");
        let mut again = Plan::default();
        type_home(&mut again, "-35.363262", "149.165237", "584.25");
        assert_eq!(again.adopt_from_file("x", &read), None);
        assert_eq!(again.items(), plan.items());
        assert_eq!(again.home(), plan.home());
    }

    /// The Home Location link takes the vehicle's position and altitude above sea level, or says
    /// how to get one.
    #[test]
    fn the_home_location_link_takes_the_vehicles_position() {
        let mut plan = Plan::default();
        assert_eq!(plan.home_from_vehicle(None), Err(HOME_NEEDS_A_FIX));
        assert_eq!(plan.home(), None);
        plan.home_from_vehicle(Some((canberra(), 612.3)))
            .expect("a position");
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.363262");
        assert_eq!(plan.home_text(HomeBox::Lng), "149.165237");
        assert_eq!(plan.home_text(HomeBox::Alt), "612.30");
    }

    /// The planned home `MainV2` starts with comes from the saved boxes, and none off the globe.
    #[test]
    fn the_planned_home_comes_from_the_saved_boxes() {
        assert_eq!(planned_home_from_config(None), Home::default());
        let mut config = mp_settings::Config::default();
        config.set("TXT_homelat", "-27.5097083021517");
        config.set("TXT_homelng", "153.015389442444");
        config.set("TXT_homealt", "8.10");
        assert_eq!(
            planned_home_from_config(Some(&config)),
            Home {
                lat: -27.509_708_302_151_7,
                lng: 153.015_389_442_444,
                alt: 8.1
            }
        );
        config.set("TXT_homealt", "");
        assert!(
            planned_home_from_config(Some(&config)).alt.abs() < f64::EPSILON,
            "GetDouble's default"
        );
        config.set("TXT_homelng", "200");
        assert_eq!(planned_home_from_config(Some(&config)), Home::default());
    }

    /// The boxes carry the resx labels and distinct ids.
    #[test]
    fn the_home_boxes_are_lat_long_and_asl() {
        let labels: Vec<&str> = HomeBox::ALL.iter().map(|which| which.label()).collect();
        assert_eq!(labels, vec!["Lat", "Long", "ASL"]);
        let ids: Vec<&str> = HomeBox::ALL.iter().map(|which| which.id()).collect();
        assert_eq!(ids, vec!["plan-home-lat", "plan-home-lng", "plan-home-alt"]);
    }

    /// A press that closed the menu is swallowed once, at that point only.
    #[test]
    fn the_press_that_closes_the_menu_adds_nothing() {
        let mut menus = PlanMenus::default();
        menus.open_at(CLICK, canberra(), None);
        menus.dismiss((10.0, 20.0));
        assert!(menus.open.is_none());
        assert!(!menus.swallows_press((11.0, 20.0)), "another press");
        menus.open_at(CLICK, canberra(), None);
        menus.dismiss((10.0, 20.0));
        assert!(menus.swallows_press((10.0, 20.0)));
        assert!(!menus.swallows_press((10.0, 20.0)), "only once");
        // A dismissal with no menu open records nothing.
        menus.dismiss((5.0, 5.0));
        assert!(!menus.swallows_press((5.0, 5.0)));
    }

    /// Choosing with no menu open does nothing: a stale click after the menu went away.
    #[test]
    fn choosing_with_no_menu_open_does_nothing() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        menus.choose(&mut plan, MenuAction::Rtl, &context());
        assert_eq!(plan.items().len(), 2);
    }

    /// Every id is distinct, so a script never clicks the wrong entry, and every drop-down lines
    /// up with its entry.
    #[test]
    fn the_menu_ids_are_distinct_and_drop_downs_line_up() {
        let mut ids: Vec<&str> = menu_entries()
            .filter(|entry| !entry.is_separator())
            .map(|entry| entry.id)
            .collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count);
        assert!((menu_offset(0) - MENU_PADDING).abs() < f32::EPSILON);
        // After ten entries and the separator.
        let polygon = MAP_MENU
            .iter()
            .position(|entry| entry.control == "polygonToolStripMenuItem")
            .expect("Polygon");
        assert!(
            (menu_offset(polygon) - (MENU_PADDING + 10.0 * MENU_ROW + MENU_SEPARATOR)).abs()
                < f32::EPSILON
        );
    }

    /// A drop-down lines up with its entry when it fits, moves up just enough when it would run
    /// off the bottom, and never above the window; and none of that moves the menu, whose top is
    /// an input here rather than an output.
    #[test]
    fn a_drop_down_moves_up_to_fit_and_the_menu_does_not_move() {
        // Room below: aligned with the entry (its padding removed so the rows line up).
        assert!((dropdown_top(100.0, 50.0, 200.0, 1200.0) - 46.0).abs() < f32::EPSILON);
        // The menu opened at 837 in a 1200-tall window and Map Tool is 12 rows down: a drop-down
        // of 7 rows would end below the window, so it starts higher, ending exactly at the bottom.
        let map_tool = MENU_PADDING + 12.0 * MENU_ROW;
        let height = column_height(&[]) + 7.0 * MENU_ROW;
        let top = dropdown_top(837.0, map_tool, height, 1200.0);
        assert!(top < map_tool - MENU_PADDING);
        assert!((837.0 + top + height - 1200.0).abs() < f32::EPSILON);
        // Taller than the window: pinned to the window's top, not above it.
        assert!((dropdown_top(500.0, 50.0, 5000.0, 1200.0) + 500.0).abs() < f32::EPSILON);
        // The menu's own height is what the window snaps, and a drop-down is not part of it.
        let menu = column_height(MAP_MENU);
        assert!(menu > 10.0 * MENU_ROW);
        assert!(
            menu < 1200.0,
            "the menu itself fits a 1200-tall window: {menu}"
        );
    }

    /// Every entry the plan's list says the tests prove is live, and the rest are dimmed.
    #[test]
    fn the_ported_entries_are_live() {
        let live: Vec<&str> = menu_entries()
            .filter(|entry| entry.action.is_some())
            .map(|entry| entry.id)
            .collect();
        assert_eq!(
            live,
            vec![
                "menu-deleteWP",
                "menu-insertWp",
                "menu-currentPosition",
                "menu-insertSplineWP",
                "menu-loiterForever",
                "menu-loitertime",
                "menu-loitercircles",
                "menu-jumpstart",
                "menu-jumpwP",
                "menu-rTL",
                "menu-land",
                "menu-takeoff",
                "menu-setROI",
                "menu-clearMission",
                "menu-addPolygonPoint2",
                "menu-clearPolygon2",
                "menu-fromCurrentWaypoints",
                "menu-ContextMeasure",
                "menu-reverseWPs",
                "menu-loadWPFile",
                "menu-saveWPFile",
                "menu-modifyAlt",
            ]
        );
    }
}
