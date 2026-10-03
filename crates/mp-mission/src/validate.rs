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

//! Mission validation: the checks worth running before a mission is uploaded.
//!
//! A vehicle accepts almost anything. It will take a mission whose first waypoint is 40 km away,
//! whose altitudes are in the wrong frame, or which has no takeoff on a copter that needs one,
//! and it will fly the result. These checks exist so a problem is raised on the ground, where the
//! cost of being wrong is an edit rather than an airframe.
//!
//! Every finding is advisory except where noted: a ground station that refuses to upload what the
//! pilot asked for is a ground station people work around.

use mp_units::{LatLon, Metres};

use crate::item::MissionItem;

/// How serious a finding is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Worth knowing, probably fine.
    Note,
    /// Likely a mistake; the mission will fly but not as intended.
    Warning,
    /// Almost certainly wrong, and dangerous if flown.
    Danger,
}

/// Something worth telling the pilot before takeoff.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    /// How serious.
    pub severity: Severity,
    /// The item it concerns, if it is about one.
    pub seq: Option<u16>,
    /// What is wrong, in a sentence a pilot can act on.
    pub message: String,
}

/// `MAV_CMD_NAV_TAKEOFF`.
const CMD_TAKEOFF: u16 = 22;
/// `MAV_CMD_NAV_LAND`.
const CMD_LAND: u16 = 21;
/// `MAV_CMD_NAV_RETURN_TO_LAUNCH`.
const CMD_RTL: u16 = 20;
/// `MAV_FRAME_GLOBAL`: altitude above mean sea level.
const FRAME_GLOBAL: u8 = 0;
/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`: altitude above home.
const FRAME_RELATIVE: u8 = 3;
/// `MAV_FRAME_GLOBAL_TERRAIN_ALT`.
const FRAME_TERRAIN: u8 = 10;

/// How far the first waypoint may sit from home before it is suspicious, in metres.
///
/// Chosen because the common mistake is a mission built at one site and flown at another, which
/// produces a first leg of kilometres rather than hundreds of metres.
const FIRST_LEG_WARNING_M: f64 = 2_000.0;

/// Altitude above which a relative-frame waypoint is probably a mis-set frame, in metres.
///
/// A mission in `GLOBAL` frame loaded as `GLOBAL_RELATIVE_ALT` turns a 600 m site elevation into a
/// 600 m climb. Most survey work happens under 150 m.
const SUSPICIOUS_RELATIVE_ALTITUDE_M: f64 = 500.0;

/// `MAV_TYPE` values that do not fly: rover, boat, submarine.
///
/// Altitude is meaningless for these, and a waypoint at zero is correct rather than suspicious.
/// Applying air-vehicle altitude rules to a boat produces a warning on every well-formed mission,
/// and warnings that fire on correct input are how pilots learn to ignore warnings.
const GROUND_AND_MARINE: &[u8] = &[10, 11, 12];

/// What the mission will be flown by, when it is known.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Context {
    /// The vehicle's home position.
    pub home: Option<LatLon>,
    /// `MAV_TYPE`, when known.
    pub vehicle_type: Option<u8>,
}

impl Context {
    /// Whether altitude checks apply.
    fn altitude_matters(self) -> bool {
        self.vehicle_type
            .is_none_or(|kind| !GROUND_AND_MARINE.contains(&kind))
    }

    /// How serious an altitude problem is.
    ///
    /// Without knowing the vehicle, a suspicious altitude is a warning rather than a danger: it is
    /// dangerous on an aircraft and meaningless on a boat, and guessing wrong in the alarming
    /// direction trains people to dismiss the alarm.
    fn altitude_severity(self) -> Severity {
        match self.vehicle_type {
            Some(kind) if !GROUND_AND_MARINE.contains(&kind) => Severity::Danger,
            _ => Severity::Warning,
        }
    }
}

/// Runs every check.
///
/// Checks that need context are skipped without it rather than guessed at.
#[must_use]
pub fn validate_with(items: &[MissionItem], context: Context) -> Vec<Finding> {
    let mut findings = Vec::new();

    if items.is_empty() {
        findings.push(Finding {
            severity: Severity::Warning,
            seq: None,
            message: "the mission is empty".to_owned(),
        });
        return findings;
    }

    check_sequence(items, &mut findings);
    check_frames(items, context, &mut findings);
    check_structure(items, &mut findings);
    check_geometry(items, context.home, &mut findings);

    findings.sort_by(|a, b| b.severity.cmp(&a.severity).then(a.seq.cmp(&b.seq)));
    findings
}

/// Sequence numbers must be contiguous from zero.
fn check_sequence(items: &[MissionItem], findings: &mut Vec<Finding>) {
    for (index, item) in items.iter().enumerate() {
        let expected = u16::try_from(index).unwrap_or(u16::MAX);
        if item.seq != expected {
            findings.push(Finding {
                severity: Severity::Warning,
                seq: Some(item.seq),
                message: format!(
                    "item {} is numbered {} but sits at position {index}; the vehicle indexes by \
                     position, so the numbering is misleading",
                    index, item.seq
                ),
            });
            // One report is enough; after the first gap everything downstream is shifted.
            break;
        }
    }
}

/// Altitude frames must be consistent and plausible.
fn check_frames(items: &[MissionItem], context: Context, findings: &mut Vec<Finding>) {
    let mut frames: Vec<u8> = items
        .iter()
        .filter(|item| item.is_navigation() && !item.is_home())
        .map(|item| item.frame)
        .collect();
    frames.sort_unstable();
    frames.dedup();

    if frames.len() > 1 {
        findings.push(Finding {
            severity: Severity::Warning,
            seq: None,
            message: format!(
                "the mission mixes {} altitude frames; check that is deliberate, because a \
                 relative altitude and an absolute one that read the same number are hundreds of \
                 metres apart",
                frames.len()
            ),
        });
    }

    for item in items.iter().filter(|i| i.is_navigation() && !i.is_home()) {
        // Land and return-to-launch do not carry a target altitude in any useful sense: landing
        // means reaching the ground, and RTL uses the vehicle's own RTL altitude parameter.
        // Checking their altitude produces a warning on every correctly written mission.
        let altitude_is_a_target = !matches!(item.command, CMD_LAND | CMD_RTL);
        if altitude_is_a_target
            && context.altitude_matters()
            && item.frame == FRAME_RELATIVE
            && item.z > SUSPICIOUS_RELATIVE_ALTITUDE_M
        {
            findings.push(Finding {
                severity: context.altitude_severity(),
                seq: Some(item.seq),
                message: format!(
                    "{:.0} m above home is unusually high; if this mission was saved in absolute \
                     altitude, loading it as relative has turned the site elevation into a climb",
                    item.z
                ),
            });
        }
        if altitude_is_a_target
            && context.altitude_matters()
            && item.frame == FRAME_RELATIVE
            && item.z <= 0.0
        {
            findings.push(Finding {
                severity: context.altitude_severity(),
                seq: Some(item.seq),
                message: format!("{:.0} m above home is at or below ground level", item.z),
            });
        }
        if item.frame != FRAME_GLOBAL && item.frame != FRAME_RELATIVE && item.frame != FRAME_TERRAIN
        {
            findings.push(Finding {
                severity: Severity::Note,
                seq: Some(item.seq),
                message: format!("uses altitude frame {}, which is unusual", item.frame),
            });
        }
    }
}

/// The mission should start and end the way the vehicle expects.
fn check_structure(items: &[MissionItem], findings: &mut Vec<Finding>) {
    let commands: Vec<u16> = items.iter().map(|item| item.command).collect();

    if !commands.contains(&CMD_TAKEOFF) {
        findings.push(Finding {
            severity: Severity::Note,
            seq: None,
            message: "no takeoff command; a copter starting on the ground will not climb on its \
                      own"
            .to_owned(),
        });
    }

    let ends_safely = items
        .last()
        .is_some_and(|item| matches!(item.command, CMD_LAND | CMD_RTL));
    if !ends_safely {
        findings.push(Finding {
            severity: Severity::Warning,
            seq: items.last().map(|item| item.seq),
            message: "the mission does not end with a land or return-to-launch; what the vehicle \
                      does after the last waypoint depends on its failsafe settings"
                .to_owned(),
        });
    }

    // Takeoff partway through a mission is legal but almost always a mistake.
    for (index, item) in items.iter().enumerate() {
        if item.command == CMD_TAKEOFF && index > 1 {
            findings.push(Finding {
                severity: Severity::Warning,
                seq: Some(item.seq),
                message: "a takeoff appears after the start of the mission".to_owned(),
            });
        }
    }
}

/// Distances that suggest the mission belongs somewhere else.
fn check_geometry(items: &[MissionItem], home: Option<LatLon>, findings: &mut Vec<Finding>) {
    let navigation: Vec<&MissionItem> = items
        .iter()
        .filter(|item| item.is_navigation() && !item.is_home())
        .collect();

    // The first item that actually has coordinates, not merely the first navigation command: a
    // takeoff carries no position, and most missions begin with one. Checking `navigation.first()`
    // silently skipped this test on every realistic mission.
    let first_positioned = navigation.iter().find_map(|item| {
        item.position()
            .ok()
            .flatten()
            .map(|position| (item.seq, position))
    });

    if let (Some(home), Some((seq, position))) = (home, first_positioned) {
        let distance = home.distance_to(position);
        if distance > Metres(FIRST_LEG_WARNING_M) {
            findings.push(Finding {
                severity: Severity::Danger,
                seq: Some(seq),
                message: format!(
                    "the first waypoint is {:.1} km from home; this mission may have been built \
                     for a different site",
                    distance.0 / 1000.0
                ),
            });
        }
    }

    // Consecutive waypoints at the same place are a symptom of a duplicated row.
    for pair in navigation.windows(2) {
        let (Some(previous), Some(current)) = (pair.first(), pair.last()) else {
            continue;
        };
        let (Ok(Some(a)), Ok(Some(b))) = (previous.position(), current.position()) else {
            continue;
        };
        if a.distance_to(b) < Metres(0.5) && (previous.z - current.z).abs() < 0.5 {
            findings.push(Finding {
                severity: Severity::Note,
                seq: Some(current.seq),
                message: "this waypoint is in the same place as the one before it".to_owned(),
            });
        }
    }
}

/// Runs every check with only a home position, for callers that do not know the vehicle type.
#[must_use]
pub fn validate(items: &[MissionItem], home: Option<LatLon>) -> Vec<Finding> {
    validate_with(
        items,
        Context {
            home,
            vehicle_type: None,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A copter, for checks where the vehicle type matters.
    fn copter() -> Context {
        Context {
            home: None,
            vehicle_type: Some(2),
        }
    }

    fn at(lat: f64, lon: f64) -> LatLon {
        LatLon::new(lat, lon).expect("valid")
    }

    fn waypoint(seq: u16, lat: f64, lon: f64, alt: f64) -> MissionItem {
        MissionItem {
            seq,
            command: 16,
            frame: FRAME_RELATIVE,
            x: lat,
            y: lon,
            z: alt,
            ..MissionItem::default()
        }
    }

    /// A well-formed mission: home, takeoff, two waypoints, land.
    fn good_mission() -> Vec<MissionItem> {
        vec![
            MissionItem {
                seq: 0,
                command: 16,
                frame: FRAME_GLOBAL,
                x: -35.363,
                y: 149.165,
                ..MissionItem::default()
            },
            MissionItem {
                seq: 1,
                command: CMD_TAKEOFF,
                frame: FRAME_RELATIVE,
                z: 30.0,
                ..MissionItem::default()
            },
            waypoint(2, -35.364, 149.166, 50.0),
            waypoint(3, -35.365, 149.167, 50.0),
            MissionItem {
                seq: 4,
                command: CMD_LAND,
                frame: FRAME_RELATIVE,
                x: -35.363,
                y: 149.165,
                ..MissionItem::default()
            },
        ]
    }

    fn severities(findings: &[Finding]) -> Vec<Severity> {
        findings.iter().map(|f| f.severity).collect()
    }

    #[test]
    fn a_well_formed_mission_raises_nothing_serious() {
        let findings = validate(&good_mission(), Some(at(-35.363, 149.165)));
        assert!(
            !severities(&findings).contains(&Severity::Danger),
            "a good mission should not raise danger: {findings:?}"
        );
        assert!(
            !severities(&findings).contains(&Severity::Warning),
            "a good mission should not raise warnings: {findings:?}"
        );
    }

    #[test]
    fn a_mission_built_for_another_site_is_flagged() {
        // The failure this catches: a mission saved in Colorado, flown in Canberra. It is exactly
        // what happened the first time a sample mission was loaded here.
        let mut mission = good_mission();
        mission[2] = waypoint(2, 40.07, -105.23, 50.0);

        let findings = validate(&mission, Some(at(-35.363, 149.165)));
        let danger = findings.iter().find(|f| f.severity == Severity::Danger);
        assert!(
            danger.is_some(),
            "a waypoint on another continent must be flagged: {findings:?}"
        );
        assert!(
            danger.unwrap().message.contains("km from home"),
            "the message should say how far: {:?}",
            danger.unwrap().message
        );
    }

    #[test]
    fn an_absolute_altitude_loaded_as_relative_is_flagged() {
        // A site at 600 m elevation saved in GLOBAL frame, reloaded as relative, becomes a 600 m
        // climb. The number looks reasonable in isolation, which is what makes it dangerous.
        let mut mission = good_mission();
        mission[2] = waypoint(2, -35.364, 149.166, 634.0);

        let findings = validate_with(
            &mission,
            Context {
                home: Some(at(-35.363, 149.165)),
                ..copter()
            },
        );
        assert!(
            findings
                .iter()
                .any(|f| f.severity == Severity::Danger && f.message.contains("climb")),
            "a 634 m relative altitude should be flagged: {findings:?}"
        );
    }

    #[test]
    fn altitude_checks_do_not_fire_on_a_boat() {
        // A rover or boat waypoint at zero altitude is correct. Warning about it on every
        // well-formed ground mission is how people learn to dismiss warnings.
        let mut mission = good_mission();
        mission[2] = waypoint(2, -35.364, 149.166, 0.0);

        let as_boat = validate_with(
            &mission,
            Context {
                home: None,
                vehicle_type: Some(11),
            },
        );
        assert!(
            !as_boat.iter().any(|f| f.message.contains("ground level")),
            "a boat should not be warned about altitude: {as_boat:?}"
        );

        let as_copter = validate_with(
            &mission,
            Context {
                home: None,
                vehicle_type: Some(2),
            },
        );
        assert!(
            as_copter.iter().any(|f| f.severity == Severity::Danger),
            "a copter at zero altitude is a danger: {as_copter:?}"
        );
    }

    #[test]
    fn an_unknown_vehicle_warns_rather_than_alarms() {
        // Without the vehicle type the same altitude is dangerous on an aircraft and meaningless
        // on a boat. Guessing in the alarming direction is what trains people to ignore alarms.
        let mut mission = good_mission();
        mission[2] = waypoint(2, -35.364, 149.166, 0.0);

        let findings = validate(&mission, None);
        let altitude = findings.iter().find(|f| f.message.contains("ground level"));
        assert_eq!(
            altitude.map(|f| f.severity),
            Some(Severity::Warning),
            "unknown vehicle should warn, not alarm: {findings:?}"
        );
    }

    #[test]
    fn a_waypoint_at_or_below_ground_is_flagged() {
        let mut mission = good_mission();
        mission[2] = waypoint(2, -35.364, 149.166, 0.0);
        let copter = Context {
            home: None,
            vehicle_type: Some(2),
        };
        assert!(
            validate_with(&mission, copter)
                .iter()
                .any(|f| f.severity == Severity::Danger),
            "zero altitude above home is at ground level"
        );

        mission[2] = waypoint(2, -35.364, 149.166, -10.0);
        assert!(
            validate_with(&mission, copter)
                .iter()
                .any(|f| f.severity == Severity::Danger)
        );
    }

    #[test]
    fn a_mission_that_does_not_end_safely_is_flagged() {
        let mut mission = good_mission();
        mission.pop(); // remove the land
        let findings = validate(&mission, None);
        assert!(
            findings
                .iter()
                .any(|f| f.message.contains("land or return-to-launch")),
            "{findings:?}"
        );
    }

    #[test]
    fn mixed_altitude_frames_are_flagged() {
        let mut mission = good_mission();
        mission[3].frame = FRAME_GLOBAL;
        let findings = validate(&mission, None);
        assert!(
            findings
                .iter()
                .any(|f| f.message.contains("altitude frames")),
            "{findings:?}"
        );
    }

    #[test]
    fn duplicated_waypoints_are_noted_but_not_alarming() {
        let mut mission = good_mission();
        mission[3] = waypoint(3, -35.364, 149.166, 50.0); // same as item 2
        let findings = validate(&mission, None);
        let duplicate = findings.iter().find(|f| f.message.contains("same place"));
        assert!(duplicate.is_some(), "{findings:?}");
        assert_eq!(
            duplicate.unwrap().severity,
            Severity::Note,
            "a duplicate is not a danger"
        );
    }

    #[test]
    fn an_empty_mission_is_reported_once() {
        let findings = validate(&[], None);
        assert_eq!(findings.len(), 1);
        assert!(findings[0].message.contains("empty"));
    }

    #[test]
    fn findings_are_ordered_worst_first() {
        let mut mission = good_mission();
        mission[2] = waypoint(2, 40.07, -105.23, 634.0); // far away and too high
        mission.pop(); // and no landing

        let findings = validate(&mission, Some(at(-35.363, 149.165)));
        assert!(findings.len() >= 3, "{findings:?}");
        for pair in findings.windows(2) {
            assert!(
                pair[0].severity >= pair[1].severity,
                "findings must be worst first"
            );
        }
    }
}
