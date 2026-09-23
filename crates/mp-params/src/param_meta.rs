//! Parameter metadata: what each parameter means, and what values it will accept.
//!
//! A parameter download gives names and numbers. This is everything that turns those into a
//! configuration screen a pilot can use without a wiki tab open - and, more importantly, into one
//! that refuses a value the firmware will reject.

/// How exposed a parameter should be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UserLevel {
    /// Safe for anyone to change.
    Standard,
    /// Hidden unless advanced settings are enabled.
    Advanced,
    /// The metadata does not say; treated as advanced.
    Unspecified,
}

impl UserLevel {
    /// Whether to show this parameter in the ordinary configuration view.
    ///
    /// Unspecified counts as advanced: a parameter nobody documented well enough to classify is
    /// not one to put in front of a new user by default.
    #[must_use]
    pub const fn is_basic(self) -> bool {
        matches!(self, Self::Standard)
    }
}

/// Everything known about one parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamMeta {
    /// Parameter name, e.g. `WPNAV_SPEED`.
    pub name: &'static str,
    /// Short label for a form field.
    pub display_name: &'static str,
    /// Full description.
    pub description: &'static str,
    /// Units as the firmware documents them, e.g. `cm/s`.
    pub units: &'static str,
    /// Inclusive minimum and maximum.
    pub range: Option<(f64, f64)>,
    /// Suggested step for a spinner.
    pub increment: Option<f64>,
    /// Named values, when the parameter is an enumeration.
    pub values: &'static [(i64, &'static str)],
    /// Named bits, when it is a bitmask.
    pub bitmask: &'static [(u32, &'static str)],
    /// How exposed it should be.
    pub user_level: UserLevel,
    /// Whether a reboot is needed for a change to take effect.
    pub reboot_required: bool,
}

impl ParamMeta {
    /// Whether a value is inside the documented range.
    ///
    /// Out-of-range values are not merely untidy: ArduPilot will accept a parameter write it
    /// considers invalid and then behave unexpectedly, so catching it in the editor is the last
    /// chance to catch it at all.
    #[must_use]
    pub fn accepts(&self, value: f64) -> bool {
        match self.range {
            Some((low, high)) => value >= low && value <= high,
            None => true,
        }
    }

    /// The name of an enumerated value.
    #[must_use]
    pub fn value_name(&self, value: i64) -> Option<&'static str> {
        self.values
            .iter()
            .find(|(number, _)| *number == value)
            .map(|(_, name)| *name)
    }

    /// The bits set in a bitmask value, by name.
    #[must_use]
    pub fn bit_names(&self, value: u32) -> Vec<&'static str> {
        self.bitmask
            .iter()
            .filter(|(bit, _)| *bit < 32 && value & (1u32 << *bit) != 0)
            .map(|(_, name)| *name)
            .collect()
    }

    /// Whether this parameter is an enumeration rather than a free number.
    #[must_use]
    pub const fn is_enumeration(&self) -> bool {
        !self.values.is_empty()
    }

    /// Whether this parameter is a bitmask.
    #[must_use]
    pub const fn is_bitmask(&self) -> bool {
        !self.bitmask.is_empty()
    }
}

/// Copter parameter metadata.
#[path = "generated/param_meta_copter.rs"]
pub mod copter;

/// Looks a parameter up by its exact name.
///
/// Mission Planner's rule, and the only one: its fallback, `ParameterMetaDataRepositoryAPM`, asks
/// the XML this table is generated from for the element named exactly as the parameter
/// (`// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:70-104`), as the `apm.pdef.xml`
/// it reads first is asked for exactly `Vehicle:NAME` or `NAME`
/// (`// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:219-231`). A numbered name
/// the table does not list - `SERVO33_FUNCTION` against a table that stops at 32 - has no
/// documentation, as it has none in Mission Planner; it does not borrow a sibling's.
///
/// A bisection of the table, which is sorted by name.
#[must_use]
pub fn lookup(name: &str) -> Option<&'static ParamMeta> {
    copter::PARAMETERS
        .binary_search_by(|meta| meta.name.cmp(name))
        .ok()
        .and_then(|index| copter::PARAMETERS.get(index))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SITL's 1,408 names, as `testdata/params/sitl-copter.param` lists them.
    fn sitl_names() -> Vec<String> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = std::fs::read_to_string(&fixture).expect("the SITL parameter dump");
        let names: Vec<String> = text
            .lines()
            .filter(|line| !line.starts_with('#'))
            .filter_map(|line| line.split(',').next())
            .map(str::to_owned)
            .collect();
        assert_eq!(names.len(), 1408, "the dump PLAN.md 10.5 counted");
        names
    }

    /// The lookup is exact, as `ParameterMetaDataRepositoryAPM.GetParameterMetaData` asks the
    /// XML for the element of exactly that name (`ParameterMetaDataRepositoryAPM.cs:70-104`) and
    /// the pdef lookup for exactly `Vehicle:NAME` or `NAME`
    /// (`ParameterMetaDataRepositoryAPMpdef.cs:219-231`): `RC5_MIN` is documented only if the
    /// table has `RC5_MIN`, and a renumbered name the table lacks is not documented at all - not
    /// by a sibling with other digits, which is what a digit-folding retry here used to do.
    #[test]
    fn the_bundled_lookup_is_exact() {
        let listed = |name: &str| copter::PARAMETERS.iter().any(|meta| meta.name == name);

        for name in ["RC5_MIN", "SERVO9_FUNCTION", "BATT2_MONITOR"] {
            assert_eq!(
                lookup(name).map(|meta| meta.name),
                listed(name).then_some(name),
                "{name}"
            );
        }
        assert!(listed("RC5_MIN"), "the table documents RC5_MIN by name");

        // Renumbered names the table lacks, each with a sibling it does list.
        for (absent, sibling) in [
            ("RC99_MIN", "RC5_MIN"),
            ("SERVO99_FUNCTION", "SERVO9_FUNCTION"),
            ("BATT99_MONITOR", "BATT2_MONITOR"),
            ("SERVO_FUNCTION", "SERVO1_FUNCTION"),
        ] {
            assert!(listed(sibling), "{sibling} is in the table");
            assert!(!listed(absent), "{absent} is not in the table");
            assert_eq!(
                lookup(absent),
                None,
                "{absent} borrowed a sibling's documentation"
            );
        }

        // Every name in the table finds its own entry, and every SITL name finds an entry of
        // exactly its own name or nothing.
        for meta in copter::PARAMETERS {
            assert!(
                lookup(meta.name).is_some_and(|found| std::ptr::eq(found, meta)),
                "{}",
                meta.name
            );
        }
        let names = sitl_names();
        let mut documented = 0;
        for name in &names {
            match lookup(name) {
                Some(meta) => {
                    assert_eq!(meta.name, name.as_str());
                    documented += 1;
                }
                None => assert!(!listed(name), "{name} is in the table"),
            }
        }
        eprintln!(
            "bundled table documents {documented} of {} SITL names, by exact name",
            names.len()
        );
    }
}
