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

//! Other aircraft: ADS-B reports and other MAVLink vehicles.
//!
//! A ground station that cannot show nearby traffic is missing the one thing that prevents a
//! collision. ArduPilot forwards ADS-B it receives as `ADSB_VEHICLE`, one message per aircraft per
//! update, and the ground station is expected to keep the set.
//!
//! Keeping a set means deciding when to forget. An aircraft that has stopped reporting has either
//! landed, gone out of range, or had its transponder fail - and in every case a stale symbol on
//! the map is worse than no symbol, because it says something is somewhere it is not.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use mp_units::LatLon;

/// How long an aircraft stays on the map after its last report.
///
/// ADS-B updates about once a second. Thirty is long enough to ride out a gap in reception and
/// short enough that a departed aircraft does not linger.
pub const FORGET_AFTER: Duration = Duration::from_secs(30);

/// How long before a symbol is shown as stale rather than current.
///
/// Between this and [`FORGET_AFTER`] the aircraft is drawn differently: it was there, and we no
/// longer know that it still is.
pub const STALE_AFTER: Duration = Duration::from_secs(5);

/// One aircraft heard about.
#[derive(Debug, Clone, PartialEq)]
pub struct Traffic {
    /// ICAO 24-bit address, which is the aircraft's identity.
    pub icao: u32,
    /// Callsign, when the transponder reports one.
    pub callsign: Option<String>,
    /// Where it is.
    pub position: LatLon,
    /// Altitude in metres, as reported.
    pub altitude: f64,
    /// Heading in degrees, or `None` if not reported.
    pub heading: Option<f64>,
    /// Ground speed in metres per second, or `None` if not reported.
    pub speed: Option<f64>,
    /// When it was last heard from.
    pub last_seen: Instant,
}

impl Traffic {
    /// Whether this report is old enough to be doubted.
    #[must_use]
    pub fn is_stale(&self, now: Instant) -> bool {
        now.duration_since(self.last_seen) > STALE_AFTER
    }

    /// What to show as its name.
    ///
    /// The callsign when there is one, else the ICAO address in hex, which is how every other
    /// tool refers to an aircraft with no callsign.
    #[must_use]
    pub fn label(&self) -> String {
        self.callsign
            .clone()
            .unwrap_or_else(|| format!("{:06X}", self.icao))
    }
}

/// Every aircraft currently known about.
#[derive(Debug, Default)]
pub struct TrafficReport {
    aircraft: BTreeMap<u32, Traffic>,
}

impl TrafficReport {
    /// An empty report.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Records a sighting, replacing any previous one for the same aircraft.
    ///
    /// Keyed on the ICAO address rather than on position: a report without a valid position still
    /// updates what is known about an aircraft that had one.
    pub fn observe(&mut self, traffic: Traffic) {
        self.aircraft.insert(traffic.icao, traffic);
    }

    /// Drops aircraft not heard from recently.
    ///
    /// Returns how many were dropped, so a caller can say so rather than have symbols vanish
    /// silently.
    pub fn forget_old(&mut self, now: Instant) -> usize {
        let before = self.aircraft.len();
        self.aircraft
            .retain(|_, traffic| now.duration_since(traffic.last_seen) <= FORGET_AFTER);
        before - self.aircraft.len()
    }

    /// Everything known, in a stable order.
    #[must_use]
    pub fn all(&self) -> Vec<Traffic> {
        self.aircraft.values().cloned().collect()
    }

    /// How many aircraft are known.
    #[must_use]
    pub fn len(&self) -> usize {
        self.aircraft.len()
    }

    /// Whether nothing is known.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.aircraft.is_empty()
    }

    /// The nearest aircraft to a position, with its distance in metres.
    ///
    /// What a pilot actually wants from a traffic display: not a list, but the one to worry about.
    #[must_use]
    pub fn nearest(&self, to: LatLon) -> Option<(Traffic, f64)> {
        self.aircraft
            .values()
            .map(|traffic| (traffic.clone(), to.distance_to(traffic.position).0))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }
}

/// Decodes a callsign from its fixed-width field.
///
/// Padded with spaces or NULs depending on the transponder, and occasionally with neither when it
/// fills the field. A callsign of blanks is no callsign rather than a name made of spaces.
#[must_use]
pub fn decode_callsign(raw: &[u8]) -> Option<String> {
    let end = raw.iter().position(|byte| *byte == 0).unwrap_or(raw.len());
    let text = String::from_utf8_lossy(raw.get(..end).unwrap_or(raw));
    let trimmed = text.trim();
    (!trimmed.is_empty()).then(|| trimmed.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(lat: f64, lon: f64) -> LatLon {
        LatLon::new(lat, lon).expect("a valid position")
    }

    fn traffic(icao: u32, seen: Instant) -> Traffic {
        Traffic {
            icao,
            callsign: None,
            position: at(-35.36, 149.16),
            altitude: 500.0,
            heading: Some(90.0),
            speed: Some(50.0),
            last_seen: seen,
        }
    }

    #[test]
    fn an_aircraft_is_keyed_on_its_identity_not_its_position() {
        // Otherwise a moving aircraft becomes a trail of separate symbols.
        let mut report = TrafficReport::new();
        let now = Instant::now();
        report.observe(traffic(0x40_0001, now));
        let mut moved = traffic(0x40_0001, now);
        moved.position = at(-35.37, 149.17);
        report.observe(moved);

        assert_eq!(report.len(), 1);
        assert!((report.all()[0].position.latitude() - -35.37).abs() < 1e-9);
    }

    #[test]
    fn aircraft_that_stop_reporting_are_forgotten() {
        // A stale symbol is worse than no symbol: it says something is somewhere it is not.
        let mut report = TrafficReport::new();
        let now = Instant::now();
        report.observe(traffic(0x40_0001, now));
        report.observe(traffic(
            0x40_0002,
            now - FORGET_AFTER - Duration::from_secs(1),
        ));

        assert_eq!(report.forget_old(now), 1);
        assert_eq!(report.len(), 1);
        assert_eq!(report.all()[0].icao, 0x40_0001);
    }

    #[test]
    fn a_recent_gap_does_not_forget_an_aircraft() {
        // ADS-B reception is patchy; dropping an aircraft on one missed second would make the
        // display flicker rather than inform.
        let mut report = TrafficReport::new();
        let now = Instant::now();
        report.observe(traffic(0x40_0001, now - Duration::from_secs(10)));
        assert_eq!(report.forget_old(now), 0);
        assert_eq!(report.len(), 1);
    }

    #[test]
    fn an_old_report_is_marked_stale_before_it_is_forgotten() {
        // There is a window where the aircraft was there and we no longer know that it still is.
        let now = Instant::now();
        let fresh = traffic(0x40_0001, now);
        let old = traffic(0x40_0002, now - STALE_AFTER - Duration::from_secs(1));
        assert!(!fresh.is_stale(now));
        assert!(old.is_stale(now));
        assert!(
            STALE_AFTER < FORGET_AFTER,
            "stale must come before forgotten"
        );
    }

    #[test]
    fn the_nearest_aircraft_is_the_one_to_worry_about() {
        let mut report = TrafficReport::new();
        let now = Instant::now();
        let mut near = traffic(0x40_0001, now);
        near.position = at(-35.3605, 149.16);
        let mut far = traffic(0x40_0002, now);
        far.position = at(-35.40, 149.16);
        report.observe(far);
        report.observe(near);

        let (closest, distance) = report
            .nearest(at(-35.36, 149.16))
            .expect("something nearby");
        assert_eq!(closest.icao, 0x40_0001);
        assert!(distance < 100.0, "expected metres, got {distance}");
    }

    #[test]
    fn nearest_on_an_empty_report_is_nothing_rather_than_a_panic() {
        let report = TrafficReport::new();
        assert!(report.nearest(at(-35.36, 149.16)).is_none());
    }

    #[test]
    fn a_callsign_of_blanks_is_no_callsign() {
        // Transponders pad with spaces, NULs, or neither. A name made of spaces is not a name.
        assert_eq!(decode_callsign(b"QFA123\0\0\0").as_deref(), Some("QFA123"));
        assert_eq!(decode_callsign(b"QFA123   ").as_deref(), Some("QFA123"));
        assert_eq!(decode_callsign(b"         "), None);
        assert_eq!(decode_callsign(b"\0\0\0\0\0\0\0\0\0"), None);
        // A callsign that fills the field has no terminator at all.
        assert_eq!(decode_callsign(b"ABCDEFGHI").as_deref(), Some("ABCDEFGHI"));
    }

    #[test]
    fn an_aircraft_without_a_callsign_is_named_by_its_address() {
        // Which is how every other tool refers to one.
        let anonymous = traffic(0x7C_1A2B, Instant::now());
        assert_eq!(anonymous.label(), "7C1A2B");

        let mut named = traffic(0x7C_1A2B, Instant::now());
        named.callsign = Some("QFA123".to_owned());
        assert_eq!(named.label(), "QFA123");
    }
}
