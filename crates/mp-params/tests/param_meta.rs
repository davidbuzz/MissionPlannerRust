//! Parameter metadata: the descriptions, ranges and enumerations that make 1,408 numbers usable.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use mp_params::param_meta::{copter, lookup};
use mp_params::{ParamMeta, UserLevel};

#[test]
fn the_table_is_substantial_and_sorted() {
    let params = copter::PARAMETERS;
    assert!(
        params.len() > 800,
        "only {} parameters documented",
        params.len()
    );

    // lookup() binary searches, so ordering is load-bearing, not cosmetic.
    let mut last: Option<&str> = None;
    for meta in params {
        if let Some(previous) = last {
            assert!(
                meta.name > previous,
                "table is not sorted: {previous} then {}",
                meta.name
            );
        }
        last = Some(meta.name);
        assert!(!meta.name.is_empty());
    }
}

#[test]
fn well_known_parameters_carry_their_documentation() {
    let speed = lookup("WPNAV_SPEED").expect("WPNAV_SPEED is documented");
    assert!(
        speed.display_name.contains("Speed"),
        "display name was {:?}",
        speed.display_name
    );
    assert_eq!(
        speed.units, "cm/s",
        "units matter: this is centimetres per second, not metres"
    );
    let (low, high) = speed.range.expect("WPNAV_SPEED has a documented range");
    assert!(low > 0.0 && high > low, "range was {low}..{high}");
    assert!(!speed.description.is_empty());
}

#[test]
fn a_range_is_enforced_rather_than_decorative() {
    // ArduPilot accepts a write it considers invalid and then behaves unexpectedly, so the editor
    // is the last place this can be caught.
    let speed = lookup("WPNAV_SPEED").unwrap();
    let (low, high) = speed.range.unwrap();

    assert!(speed.accepts((low + high) / 2.0));
    assert!(speed.accepts(low), "the range is inclusive");
    assert!(speed.accepts(high), "the range is inclusive");
    assert!(!speed.accepts(low - 1.0));
    assert!(!speed.accepts(high + 1.0));

    // A parameter with no documented range accepts anything rather than refusing everything.
    let unranged = ParamMeta {
        name: "X",
        display_name: "",
        description: "",
        units: "",
        range: None,
        range_text: "",
        increment: None,
        values: &[],
        bitmask: &[],
        user_level: UserLevel::Standard,
        reboot_required: false,
    };
    assert!(unranged.accepts(-1e9) && unranged.accepts(1e9));
}

#[test]
fn bitmask_parameters_decode_to_names() {
    let checks = lookup("ARMING_CHECK").expect("ARMING_CHECK is documented");
    assert!(checks.is_bitmask(), "ARMING_CHECK is a bitmask");

    let names = checks.bit_names(0b0000_0110);
    assert_eq!(
        names.len(),
        2,
        "two bits set should name two checks, got {names:?}"
    );

    // Nothing set names nothing; that is different from "all".
    assert!(checks.bit_names(0).is_empty());
}

#[test]
fn enumerated_parameters_name_their_values() {
    // Find any enumeration in the table and check the naming round-trips.
    let enumerated = copter::PARAMETERS
        .iter()
        .find(|meta| meta.is_enumeration() && meta.values.len() > 2)
        .expect("the table contains enumerations");

    for (number, name) in enumerated.values {
        assert_eq!(enumerated.value_name(*number), Some(*name));
    }
    // A value outside the enumeration is not invented.
    assert_eq!(enumerated.value_name(i64::MAX), None);
}

#[test]
fn indexed_parameter_names_find_their_own_documentation() {
    // The table documents each index by its own name, as the XML it is generated from does, and
    // a lookup finds exactly that entry (`ParameterMetaDataRepositoryAPM.cs:70-104`).
    let direct = lookup("SERVO1_FUNCTION").expect("SERVO1_FUNCTION is documented");
    let higher = lookup("SERVO9_FUNCTION").expect("SERVO9_FUNCTION is documented");
    assert_eq!(direct.name, "SERVO1_FUNCTION");
    assert_eq!(higher.name, "SERVO9_FUNCTION");
    assert_eq!(
        direct.units, higher.units,
        "indexed siblings share documentation"
    );
}

#[test]
fn user_levels_classify_most_parameters() {
    let basic = copter::PARAMETERS
        .iter()
        .filter(|m| m.user_level.is_basic())
        .count();
    let total = copter::PARAMETERS.len();
    assert!(
        basic > 50,
        "only {basic} of {total} parameters are marked Standard"
    );
    assert!(basic < total, "everything cannot be Standard");

    // Unspecified counts as advanced: an undocumented parameter is not one to show by default.
    assert!(!UserLevel::Unspecified.is_basic());
    assert!(!UserLevel::Advanced.is_basic());
    assert!(UserLevel::Standard.is_basic());
}

#[test]
fn a_parameter_that_does_not_exist_returns_nothing() {
    assert!(lookup("DEFINITELY_NOT_A_PARAMETER").is_none());
    assert!(lookup("").is_none());
}
