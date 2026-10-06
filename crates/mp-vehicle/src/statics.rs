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

//! What `CurrentState` keeps in statics: one value shared by every vehicle on every link.
//!
//! The C# class has a handful of `static` fields behind instance properties - the K-index, the
//! stream-rate defaults, the planned home, the antenna tracker's position, and the names the
//! custom fields have been given. They are process-wide there, so they are process-wide here:
//! each is a static of this module, read and written through [`crate::VehicleState`]'s associated
//! functions (or [`StreamRates`]'), and never part of a snapshot. Every setter is public, because
//! in the C# something outside `CurrentState` writes each of them; the setter's documentation
//! says what.

use mp_os::Lock as _;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};

use crate::state::{LatLngAlt, VehicleState};

/// `KIndexstatic`, -1 until known. `// C#: ExtLibs/ArduPilot/CurrentState.cs:51`
static KINDEX: AtomicI32 = AtomicI32::new(-1);

/// `rateattitudebackup` to `ratercbackup`, in [`StreamRates`]' field order, with the static
/// constructor's values. `// C#: ExtLibs/ArduPilot/CurrentState.cs:45-49, 201-206`
static RATE_BACKUPS: [AtomicI32; 5] = [
    AtomicI32::new(4),
    AtomicI32::new(2),
    AtomicI32::new(2),
    AtomicI32::new(2),
    AtomicI32::new(2),
];

/// `_plannedhomelocation`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:41`
static PLANNED_HOME: Mutex<LatLngAlt> = Mutex::new(LatLngAlt::ZERO);

/// `_trackerloc`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:43`
static TRACKER_LOCATION: Mutex<LatLngAlt> = Mutex::new(LatLngAlt::ZERO);

/// `custom_field_names`: which custom field carries which value, `(field number, name)` in the
/// order they were added - the order a .NET `Dictionary` enumerates them in, which decides which
/// field a name finds when two carry it. `// C#: ExtLibs/ArduPilot/CurrentState.cs:236-237`
static CUSTOM_FIELD_NAMES: Mutex<Vec<(usize, String)>> = Mutex::new(Vec::new());

/// How many custom fields there are, `customfield0` to `customfield19`.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:239-258`
pub const CUSTOM_FIELDS: usize = 20;

/// A lock that a panic elsewhere cannot poison: every value behind one is whole after any
/// single write.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.os_lock().unwrap_or_else(PoisonError::into_inner)
}

/// The stream rates Mission Planner asks a vehicle for, in hertz (`rateattitude`,
/// `rateposition`, `ratestatus`, `ratesensors`, `raterc`).
///
/// Each vehicle has its own, starting from the saved defaults ([`StreamRates::backups`]) when it
/// is first seen, as `ResetInternals` copies them. They are what the link's stream requests send
/// every 30 seconds while the link is quiet (`CurrentState.cs:4633-4664`), what Planner's
/// telemetry-rate combos show and set (`ConfigPlanner.cs:179-183, 577-625`), and what the compass
/// and radio calibrations turn down and restore around themselves (`MagCalib.cs:437-444,
/// 701-716`; `ConfigRadioInput.cs:209-217, 388-391`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2002-2007, 4396-4400`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamRates {
    /// `rateattitude`: `EXTRA1` and `EXTRA2` - attitude, and the HUD's `VFR_HUD`.
    pub attitude: i32,
    /// `rateposition`: `POSITION`.
    pub position: i32,
    /// `ratestatus`: `EXTENDED_STATUS`, and the camera's message intervals.
    pub status: i32,
    /// `ratesensors`: `EXTRA3` and `RAW_SENSORS`.
    pub sensors: i32,
    /// `raterc`: `RC_CHANNELS`.
    pub rc: i32,
}

impl Default for StreamRates {
    /// The static constructor's defaults: attitude 4 Hz, everything else 2 Hz.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:201-206`
    fn default() -> Self {
        Self {
            attitude: 4,
            position: 2,
            status: 2,
            sensors: 2,
            rc: 2,
        }
    }
}

impl StreamRates {
    /// The saved defaults a newly seen vehicle starts with (`rateattitudebackup` to
    /// `ratercbackup`).
    #[must_use]
    pub fn backups() -> Self {
        let [attitude, position, status, sensors, rc] = RATE_BACKUPS
            .each_ref()
            .map(|rate| rate.load(Ordering::Relaxed));
        Self {
            attitude,
            position,
            status,
            sensors,
            rc,
        }
    }

    /// Replaces the saved defaults. The C# sets them from the `CMB_rate*` settings at start-up
    /// (`MainV2.cs:983-993`) and when Planner's combos change (`ConfigPlanner.cs:581-622`);
    /// vehicles already seen keep their own rates.
    pub fn set_backups(rates: Self) {
        let values = [
            rates.attitude,
            rates.position,
            rates.status,
            rates.sensors,
            rates.rc,
        ];
        for (backup, value) in RATE_BACKUPS.iter().zip(values) {
            backup.store(value, Ordering::Relaxed);
        }
    }
}

impl VehicleState {
    /// `KIndex` and `KIndexstatic`: the planetary K-index, -1 until known.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:51, 2073-2074`
    #[must_use]
    pub fn kindex() -> i32 {
        KINDEX.load(Ordering::Relaxed)
    }

    /// Sets the K-index. The C# sets it at start-up from the `kindex` setting and then from
    /// `KIndex.GetKIndex`'s download, through `KIndex_KIndex` (`MainV2.cs:3947-3988`).
    pub fn set_kindex(kindex: i32) {
        KINDEX.store(kindex, Ordering::Relaxed);
    }

    /// `PlannedHomeLocation`: the home the planner plans from, (0, 0, 0) until set.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:41, 1584-1589`
    #[must_use]
    pub fn planned_home() -> LatLngAlt {
        *lock(&PLANNED_HOME)
    }

    /// Sets the planned home. The C# loads it at start-up from the `TXT_homelat`, `TXT_homelng`
    /// and `TXT_homealt` settings, and puts (0, 0, 0) back if the latitude or longitude is out of
    /// range (`MainV2.cs:1014-1027`); the planner reads it for the mission's home, the grid and
    /// the flight screen's planned-home marker (`FlightPlanner.cs:386, 1075`; `FlightData.cs:
    /// 3819`).
    pub fn set_planned_home(home: LatLngAlt) {
        *lock(&PLANNED_HOME) = home;
    }

    /// Sets `TrackerLocation`, the antenna tracker's own position, which
    /// [`VehicleState::tracker_location`] prefers to home once its longitude is not 0. The C#
    /// sets it from the planner's "Set tracker home" (`FlightPlanner.cs:760, 6974`) and from the
    /// tracker-home plugin's GPS (`ExtLibs/TrackerHome/TrackerHomeGPS.cs:73, 99, 112`).
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:43, 1606-1615`
    pub fn set_tracker_location(location: LatLngAlt) {
        *lock(&TRACKER_LOCATION) = location;
    }

    /// `TrackerLocation`: the antenna tracker's position if one has been set - a longitude other
    /// than 0 - and otherwise home, which is where the C# measures distance to home, and the
    /// tracker's elevation and bearing to the vehicle, from.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1606-1615`
    #[must_use]
    pub fn tracker_location(&self) -> LatLngAlt {
        let tracker = *lock(&TRACKER_LOCATION);
        if tracker.lng != 0.0 {
            return tracker;
        }
        // C#: CurrentState.cs:1574-1582, HomeLocation: (0, 0) until HOME_POSITION.
        let (lat, lng) = self
            .home
            .map_or((0.0, 0.0), |home| (home.latitude(), home.longitude()));
        LatLngAlt {
            lat,
            lng,
            alt: self.home_altitude.0,
        }
    }

    /// The name custom field `index` carries, if it has one: `MAV_` and a `NAMED_VALUE_FLOAT`'s
    /// name in capitals, or whatever a setting gave it. What the C#'s field chooser, graph and
    /// quick view show for `customfield<index>` (`CurrentState.cs:4491-4497, 4531-4537`).
    #[must_use]
    pub fn custom_field_name(index: usize) -> Option<String> {
        lock(&CUSTOM_FIELD_NAMES)
            .iter()
            .find(|(field, _)| *field == index)
            .map(|(_, name)| name.clone())
    }

    /// Names custom field `index`, as `custom_field_names.Add` does; false, changing nothing,
    /// when the field already has a name or there is no such field (where `Add` throws). The C#
    /// adds the `customfield0` to `customfield19` settings at start-up, in capitals
    /// (`MainV2.cs:995-1002`), and the tuning graph's saved selection (`FlightData.cs:285-298`);
    /// a `NAMED_VALUE_FLOAT` claims the first field still free.
    pub fn add_custom_field_name(index: usize, name: &str) -> bool {
        if index >= CUSTOM_FIELDS {
            return false;
        }
        let mut names = lock(&CUSTOM_FIELD_NAMES);
        if names.iter().any(|(field, _)| *field == index) {
            return false;
        }
        names.push((index, name.to_owned()));
        true
    }

    /// Forgets every custom field's name. Nothing in Mission Planner does this - the names live
    /// for the process - so this is for tests that replay more than one flight in one process,
    /// each of which names the fields as its own log does.
    pub fn clear_custom_field_names() {
        lock(&CUSTOM_FIELD_NAMES).clear();
    }

    /// The custom field a `NAMED_VALUE_FLOAT` called `name` goes in: the first one named
    /// `"MAV_" + name.ToUpper()`, or else the lowest-numbered field without a name, which then
    /// takes it; `None` when all twenty have names.
    ///
    /// `Encoding.UTF8.GetString` and `ToUpper` as the C# spells them, without allocating unless a
    /// field is claimed: the name ends at its first NUL, bytes that are not UTF-8 read as U+FFFD,
    /// and each character is upper-cased where it has a single upper-case form - .NET's simple
    /// case mapping, which leaves `ß` as it is where Rust's full mapping would write `SS`.
    /// **Divergence**, for names that are not ASCII only: `ToUpper` is the current culture's
    /// (Turkish capitalises `i` differently) and Unicode's simple mapping stands in for it, and
    /// a multi-byte sequence cut short may give a different number of U+FFFD than .NET's decoder.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3918-3945`
    pub(crate) fn custom_field_for(name: &[u8]) -> Option<usize> {
        let text = String::from_utf8_lossy(name);
        // C#: CurrentState.cs:3924-3926
        let text = text.split('\0').next().unwrap_or_default();
        let upper = |c: char| {
            let mut upper = c.to_uppercase();
            match (upper.next(), upper.next()) {
                (Some(single), None) => single,
                _ => c,
            }
        };
        let wanted = || "MAV_".chars().chain(text.chars().map(upper));
        let mut names = lock(&CUSTOM_FIELD_NAMES);
        // C#: CurrentState.cs:3930
        if let Some((field, _)) = names.iter().find(|(_, name)| name.chars().eq(wanted())) {
            return Some(*field);
        }
        // C#: CurrentState.cs:3933-3944
        let free =
            (0..CUSTOM_FIELDS).find(|index| names.iter().all(|(field, _)| field != index))?;
        names.push((free, wanted().collect()));
        Some(free)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The only test in this binary that touches the custom field names.
    #[test]
    fn a_name_is_read_as_the_csharp_decodes_and_capitalises_it() {
        // C#: ExtLibs/ArduPilot/CurrentState.cs:3920-3928
        let first = VehicleState::custom_field_for(b"stra\xc3\x9fe\0\0\0").unwrap();
        // .NET's simple case mapping has no capital for `ß`, so it stays.
        assert_eq!(
            VehicleState::custom_field_name(first).as_deref(),
            Some("MAV_STRA\u{df}E")
        );
        // A byte that is not UTF-8 reads as U+FFFD, and the name ends at the first NUL.
        let second = VehicleState::custom_field_for(b"a\xffb\0zzzzzz").unwrap();
        assert_eq!(
            VehicleState::custom_field_name(second).as_deref(),
            Some("MAV_A\u{fffd}B")
        );
        assert_ne!(first, second);
        assert_eq!(VehicleState::custom_field_for(b"A\xffB"), Some(second));
    }
}
