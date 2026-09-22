//! Flight mode naming.
//!
//! The mode is the field a pilot looks at most, and the numbers are vehicle-specific: mode 4 is
//! Guided on a copter and ACRO on a plane. A lookup that ignores the vehicle type is not
//! imprecise, it is wrong.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use mp_vehicle::{VehicleFamily, flight_mode_name};

#[test]
fn copter_modes_match_ardupilot() {
    let copter = VehicleFamily::from_mav_type(2).expect("quadrotor is a copter");
    assert_eq!(copter, VehicleFamily::Copter);
    assert_eq!(copter.mode_name(0), Some("Stabilize"));
    assert_eq!(copter.mode_name(3), Some("Auto"));
    assert_eq!(copter.mode_name(4), Some("Guided"));
    assert_eq!(copter.mode_name(5), Some("Loiter"));
    assert_eq!(copter.mode_name(6), Some("RTL"));
    assert_eq!(copter.mode_name(9), Some("Land"));
}

#[test]
fn the_same_number_means_different_things_on_different_vehicles() {
    // This is why the vehicle type is part of the lookup.
    let copter = flight_mode_name(2, 4);
    let plane = flight_mode_name(1, 4);
    assert_eq!(copter, Some("Guided"));
    assert_eq!(plane, Some("ACRO"));
    assert_ne!(
        copter, plane,
        "mode 4 must not resolve to the same name on both"
    );
}

#[test]
fn every_rotorcraft_type_resolves_to_copter() {
    // Hexa, octo, tri and helicopter all fly copter modes. Missing one shows raw numbers to a
    // pilot flying a hexacopter, which is a regression nobody notices until they are in the field.
    for mav_type in [2u8, 3, 4, 13, 14, 15, 29] {
        assert_eq!(
            VehicleFamily::from_mav_type(mav_type),
            Some(VehicleFamily::Copter),
            "MAV_TYPE {mav_type} should be a copter"
        );
    }
    assert_eq!(VehicleFamily::from_mav_type(1), Some(VehicleFamily::Plane));
    assert_eq!(VehicleFamily::from_mav_type(10), Some(VehicleFamily::Rover));
}

#[test]
fn an_unknown_vehicle_type_returns_nothing_rather_than_guessing() {
    // Showing copter mode names for an unrecognised airframe is worse than showing the number.
    assert_eq!(VehicleFamily::from_mav_type(200), None);
    assert_eq!(flight_mode_name(200, 0), None);
}

#[test]
fn an_unknown_mode_number_returns_nothing() {
    let copter = VehicleFamily::from_mav_type(2).unwrap();
    assert_eq!(copter.mode_name(9999), None);
}

#[test]
fn the_tables_are_substantial_and_ordered() {
    for family in [
        VehicleFamily::Copter,
        VehicleFamily::Plane,
        VehicleFamily::Rover,
    ] {
        let modes = family.modes();
        assert!(
            modes.len() >= 10,
            "{family:?} has only {} modes",
            modes.len()
        );

        let mut last = None;
        for (number, name) in modes {
            assert!(!name.is_empty(), "{family:?} mode {number} has no name");
            if let Some(previous) = last {
                assert!(
                    *number > previous,
                    "{family:?} modes must be ordered by number"
                );
            }
            last = Some(*number);
            // Every listed mode must resolve back through the lookup.
            assert_eq!(family.mode_name(*number), Some(*name));
        }
    }
}
