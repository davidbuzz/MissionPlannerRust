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

/// Looks a parameter up by name.
///
/// Parameter names on the vehicle sometimes carry an index suffix the metadata does not, such as
/// `SERVO9_FUNCTION` against a documented `SERVO_FUNCTION`. Exact match first, then a digit-folded
/// retry, so a servo output screen is not blank.
#[must_use]
pub fn lookup(name: &str) -> Option<&'static ParamMeta> {
    if let Ok(index) = copter::PARAMETERS.binary_search_by(|meta| meta.name.cmp(name)) {
        return copter::PARAMETERS.get(index);
    }

    // Fold trailing and embedded digits: SERVO9_FUNCTION -> SERVO_FUNCTION.
    let folded: String = name.chars().filter(|c| !c.is_ascii_digit()).collect();
    if folded == name {
        return None;
    }
    copter::PARAMETERS.iter().find(|meta| {
        meta.name
            .chars()
            .filter(|c| !c.is_ascii_digit())
            .eq(folded.chars())
    })
}
