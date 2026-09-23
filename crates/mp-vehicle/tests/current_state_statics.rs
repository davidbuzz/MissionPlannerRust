//! The `CurrentState` fields behind C# statics: one value for every vehicle on every link.
//!
//! `KIndexstatic`, the `rate*backup` defaults, `_plannedhomelocation`, `_trackerloc` and
//! `custom_field_names` are process-wide in the C#, and so here; every test in this file takes
//! [`LOCK`], and puts back what it changed, because they share them.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use std::sync::{Mutex, MutexGuard, PoisonError};

use mp_mavlink::Message;
use mp_mavlink_dialects::all::{MavMessage, NamedValueFloat};
use mp_units::{LatLon, Metres};
use mp_vehicle::{DateTime, LatLngAlt, StreamRates, VehicleId, VehicleRegistry, VehicleState};

static LOCK: Mutex<()> = Mutex::new(());

fn lock() -> MutexGuard<'static, ()> {
    LOCK.lock().unwrap_or_else(PoisonError::into_inner)
}

fn named(name: &[u8], value: f32) -> MavMessage {
    let Some(MavMessage::NamedValueFloat(mut m)) =
        MavMessage::decode(<NamedValueFloat as Message>::ID, &[])
    else {
        panic!("no NAMED_VALUE_FLOAT")
    };
    m.name[..name.len()].copy_from_slice(name);
    m.value = value;
    MavMessage::NamedValueFloat(m)
}

/// Any message that is not a named value, which would claim a custom field.
fn heartbeat() -> MavMessage {
    MavMessage::decode(0, &[]).expect("HEARTBEAT")
}

#[test]
fn the_kindex_is_minus_one_until_set_and_shared() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:51, 2073-2074; MainV2.cs:3977-3981
    let _guard = lock();
    assert_eq!(VehicleState::kindex(), -1);
    VehicleState::set_kindex(4);
    assert_eq!(
        VehicleState::kindex(),
        4,
        "every vehicle reads the one value"
    );
    VehicleState::set_kindex(-1);
}

#[test]
fn a_new_vehicle_takes_the_saved_stream_rates() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:201-206, 227, 4393-4397
    let _guard = lock();
    let defaults = StreamRates {
        attitude: 4,
        position: 2,
        status: 2,
        sensors: 2,
        rc: 2,
    };
    assert_eq!(StreamRates::default(), defaults);
    assert_eq!(StreamRates::backups(), defaults);
    let mut registry = VehicleRegistry::new();
    registry.apply(1, 1, 0, &heartbeat());
    assert_eq!(
        registry.working(VehicleId::new(1, 1)).unwrap().rates,
        defaults
    );

    // Planner's combo sets the saved default and this vehicle's rate (ConfigPlanner.cs:576-584):
    // a vehicle seen after takes the new default, one seen before keeps its own.
    let changed = StreamRates {
        attitude: 10,
        ..defaults
    };
    StreamRates::set_backups(changed);
    registry.apply(2, 1, 0, &heartbeat());
    assert_eq!(
        registry.working(VehicleId::new(2, 1)).unwrap().rates,
        changed
    );
    assert_eq!(
        registry.working(VehicleId::new(1, 1)).unwrap().rates,
        defaults
    );
    // And the calibration pages turn one vehicle's down and back (MagCalib.cs:442-444).
    let state = registry.working_mut(VehicleId::new(1, 1)).unwrap();
    state.rates.attitude = 0;
    assert_eq!(state.rates.attitude, 0);
    StreamRates::set_backups(defaults);
}

#[test]
fn the_planned_home_is_shared_and_unset_until_given() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:41, 1584-1589; MainV2.cs:1012-1025
    let _guard = lock();
    assert_eq!(VehicleState::planned_home(), LatLngAlt::ZERO);
    let home = LatLngAlt {
        lat: -35.363_261,
        lng: 149.165_230,
        alt: 584.0,
    };
    VehicleState::set_planned_home(home);
    assert_eq!(VehicleState::planned_home(), home);
    VehicleState::set_planned_home(LatLngAlt::ZERO);
}

#[test]
fn the_tracker_location_is_home_until_it_has_a_longitude() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:43, 1606-1615
    let _guard = lock();
    let mut state = VehicleState::default();
    // Nothing: home, which is (0, 0) with its altitude until HOME_POSITION.
    state.home_altitude = Metres(12.0);
    assert_eq!(
        state.tracker_location(),
        LatLngAlt {
            lat: 0.0,
            lng: 0.0,
            alt: 12.0
        }
    );
    state.home = Some(LatLon::new(-35.36, 149.16).unwrap());
    let home = LatLngAlt {
        lat: -35.36,
        lng: 149.16,
        alt: 12.0,
    };
    assert_eq!(state.tracker_location(), home);
    // A tracker on the equator's longitude 0 does not count: the C# tests the longitude only.
    VehicleState::set_tracker_location(LatLngAlt {
        lat: -35.2,
        lng: 0.0,
        alt: 3.0,
    });
    assert_eq!(state.tracker_location(), home);
    let tracker = LatLngAlt {
        lat: 0.0,
        lng: 149.2,
        alt: 3.0,
    };
    VehicleState::set_tracker_location(tracker);
    assert_eq!(state.tracker_location(), tracker);
    VehicleState::set_tracker_location(LatLngAlt::ZERO);
}

#[test]
fn named_values_claim_custom_fields_in_order_and_run_out_at_twenty() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:236-258, 3913-4012, 2283-2285
    let _guard = lock();
    let mut registry = VehicleRegistry::new();
    let autopilot = VehicleId::new(1, 1);
    let at = |seconds: i64| DateTime::from_ticks(639_000_000_000_000_000 + seconds * 10_000_000);
    // A field a setting named first is kept for it (MainV2.cs:993-1000), in capitals.
    assert!(VehicleState::add_custom_field_name(0, "MAV_SETTING"));
    assert!(
        !VehicleState::add_custom_field_name(0, "MAV_OTHER"),
        "Add on a taken key"
    );
    assert!(
        !VehicleState::add_custom_field_name(20, "MAV_NONE"),
        "no such field"
    );

    // "alpha" is MAV_ALPHA, and takes the first free field.
    registry.apply_at(1, 1, 0, &named(b"alpha", 1.5), at(0));
    assert_eq!(
        VehicleState::custom_field_name(1).as_deref(),
        Some("MAV_ALPHA")
    );
    assert_eq!(registry.working(autopilot).unwrap().custom_fields[1], 1.5);
    // The same name in other cases is the same field; a name ends at its first NUL.
    registry.apply_at(1, 1, 0, &named(b"ALPHA\0junk", -2.0), at(1));
    assert_eq!(registry.working(autopilot).unwrap().custom_fields[1], -2.0);
    // The setting's name is found by a value called "setting".
    registry.apply_at(1, 1, 0, &named(b"setting", 7.0), at(1));
    assert_eq!(registry.working(autopilot).unwrap().custom_fields[0], 7.0);

    // Another component of the same system gets it too; another system does not.
    registry.apply_at(1, 154, 0, &named(b"alpha", 0.0), at(2));
    registry.apply_at(2, 1, 0, &named(b"alpha", 0.0), at(2));
    registry.apply_at(1, 1, 0, &named(b"alpha", 3.0), at(3));
    assert_eq!(
        registry
            .working(VehicleId::new(1, 154))
            .unwrap()
            .custom_fields[1],
        3.0,
        "propagateNamedFloats"
    );
    assert_eq!(
        registry
            .working(VehicleId::new(2, 1))
            .unwrap()
            .custom_fields[1],
        0.0
    );
    registry.propagate_named_floats = false;
    registry.apply_at(1, 1, 0, &named(b"alpha", 4.0), at(4));
    assert_eq!(
        registry
            .working(VehicleId::new(1, 154))
            .unwrap()
            .custom_fields[1],
        3.0
    );

    // Eighteen more names fill the twenty fields; the next has nowhere to go.
    for n in 0..18_u8 {
        let name = [b'n', b'0' + n / 10, b'0' + n % 10];
        registry.apply_at(1, 1, 0, &named(&name, f32::from(n)), at(5));
    }
    assert_eq!(
        VehicleState::custom_field_name(19).as_deref(),
        Some("MAV_N17")
    );
    assert_eq!(registry.working(autopilot).unwrap().custom_fields[19], 17.0);
    let before = registry.working(autopilot).unwrap().custom_fields;
    assert!(
        !registry
            .working_mut(autopilot)
            .unwrap()
            .apply(&named(b"late", 99.0))
    );
    assert_eq!(registry.working(autopilot).unwrap().custom_fields, before);
    assert_eq!(VehicleState::custom_field_name(20), None);
}
