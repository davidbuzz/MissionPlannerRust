//! A field's scaler and offset: `DataModifer`, which a double click on a field in the tree asks
//! for and the next graph of that field applies.
//!
//! The text is up to two commands, split at the first space or comma: `x` or `*` sets the scale,
//! `/` or `\` sets it to one over the number, `+` and `-` set the offset - applied before the
//! scale when it is the first command - and `&` makes the field a mask, shifted down to its
//! lowest set bit. Anything else makes the whole text invalid, which removes the field's
//! modifier.
//! `// C#: Log/LogBrowse.cs:66-182, 1134-1152, 1236-1241, 1530-1556, 3037-3077`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

/// The input box's title for a field: `"Apply scaler and offset to " + nodeName`.
/// `// C#: Log/LogBrowse.cs:3065`
#[must_use]
pub fn title(node: &str) -> String {
    format!("Apply scaler and offset to {node}")
}

/// The input box's instructions, as the C# writes them.
/// `// C#: Log/LogBrowse.cs:3066-3069`
pub const INSTRUCTIONS: &str = "Enter modifer then value, they are applied in the order you \
provide. Modifiers are x + - /\nExample: Convert cm to to m with an offset of 50: '/100 +50' or \
'x0.01 +50' or '*0.01,+50'";

/// A parsed modifier.
#[derive(Debug, Clone, PartialEq)]
pub struct Modifier {
    /// `commandString`: the text as typed, trimmed - added to the curve's label.
    pub command: String,
    offset: f64,
    scalar: f64,
    offset_first: bool,
    mask: Option<u32>,
}

impl Modifier {
    /// `new DataModifer(text)`, if it is valid.
    /// `// C#: Log/LogBrowse.cs:84-168`
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        let command = text.trim().to_owned();
        let mut modifier = Self {
            command: command.clone(),
            offset: 0.0,
            scalar: 1.0,
            offset_first: false,
            mask: None,
        };
        // `Split(new[] {' ', ','}, 2, RemoveEmptyEntries)`: at most two pieces, the second
        // everything after the first separator.
        let trimmed = command.trim_start_matches([' ', ',']);
        let pieces: Vec<&str> = match trimmed.split_once([' ', ',']) {
            Some((first, rest)) => {
                let rest = rest.trim_start_matches([' ', ',']);
                if rest.is_empty() {
                    vec![first]
                } else {
                    vec![first, rest]
                }
            }
            None if trimmed.is_empty() => Vec::new(),
            None => vec![trimmed],
        };
        if pieces.is_empty() {
            return None;
        }
        for (index, piece) in pieces.iter().enumerate() {
            let piece = piece.trim();
            let mut chars = piece.chars();
            let command = chars.next()?;
            let parameter = chars.as_str();
            if parameter.is_empty() {
                return None;
            }
            // `NumberStyles.Number`: no exponent, no internal spaces.
            if parameter.contains(['e', 'E', ' ']) {
                return None;
            }
            let value = mp_log::netfmt::parse_double(parameter)?;
            match command {
                'x' | '*' => modifier.scalar = value,
                '\\' | '/' => modifier.scalar = 1.0 / value,
                '+' => {
                    modifier.offset_first = index == 0;
                    modifier.offset = value;
                }
                '-' => {
                    modifier.offset_first = index == 0;
                    modifier.offset = -value;
                }
                '&' => {
                    if value < 0.0 {
                        return None;
                    }
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    // `(uint)value`
                    {
                        modifier.mask = Some(value as u32);
                    }
                }
                _ => return None,
            }
        }
        Some(modifier)
    }

    /// Applies the modifier to a value as `GraphItem_GetList` does.
    /// `// C#: Log/LogBrowse.cs:1530-1556`
    #[must_use]
    pub fn apply(&self, value: f64) -> f64 {
        if let Some(mask) = self.mask {
            let shift = (0..32).find(|shift| (mask >> shift) & 1 != 0).unwrap_or(32);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // `(uint)value`
            let bits = value as u32;
            return f64::from((bits & mask).checked_shr(shift).unwrap_or(0));
        }
        if self.offset_first {
            (value + self.offset) * self.scalar
        } else {
            value.mul_add(self.scalar, self.offset)
        }
    }
}

/// A field's name as the modifiers are kept by: `DataModifer.GetNodeName`, `ATT.Roll` or
/// `IMU[1].AccX`.
/// `// C#: Log/LogBrowse.cs:175-180`
#[must_use]
pub fn node_name(message: &str, instance: Option<i64>, field: &str) -> String {
    match instance {
        Some(instance) if instance >= 0 => format!("{message}[{instance}].{field}"),
        _ => format!("{message}.{field}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The instructions' own examples: centimetres to metres, then fifty added.
    #[test]
    fn the_examples_convert_centimetres_to_metres() {
        for text in ["/100 +50", "x0.01 +50", "*0.01,+50"] {
            let modifier = Modifier::parse(text).expect(text);
            assert!((modifier.apply(250.0) - 52.5).abs() < 1e-9, "{text}");
            assert_eq!(modifier.command, text);
        }
    }

    /// An offset first is added before the scale.
    #[test]
    fn an_offset_first_is_added_before_scaling() {
        let modifier = Modifier::parse("+10 x2").expect("valid");
        assert!((modifier.apply(5.0) - 30.0).abs() < 1e-9);
        let modifier = Modifier::parse("-10").expect("valid");
        assert!((modifier.apply(5.0) + 5.0).abs() < 1e-9);
    }

    /// A mask keeps its bits and shifts them down.
    #[test]
    fn a_mask_is_shifted_to_its_lowest_bit() {
        let modifier = Modifier::parse("&12").expect("valid");
        assert!((modifier.apply(13.0) - 3.0).abs() < 1e-9);
        assert!((modifier.apply(4.0) - 1.0).abs() < 1e-9);
        assert!(Modifier::parse("&-1").is_none());
    }

    /// Only two commands: a third is part of the second, which then does not parse.
    #[test]
    fn what_does_not_parse_is_no_modifier() {
        assert!(Modifier::parse("").is_none());
        assert!(Modifier::parse("x").is_none());
        assert!(Modifier::parse("q10").is_none());
        assert!(Modifier::parse("/100 +50 x2").is_none());
        assert!(Modifier::parse("x1e3").is_none());
    }

    #[test]
    fn nodes_are_named_as_the_tree_names_them() {
        assert_eq!(node_name("ATT", None, "Roll"), "ATT.Roll");
        assert_eq!(node_name("IMU", Some(1), "AccX"), "IMU[1].AccX");
        assert_eq!(title("ATT.Roll"), "Apply scaler and offset to ATT.Roll");
    }
}
