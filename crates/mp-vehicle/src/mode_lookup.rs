//! A flight mode's number from its name: what `setMode(string)` does before it sends
//! `SET_MODE` - the name uppercased against the vehicle's mode list - kept apart from the
//! generated table it reads, which `cargo xtask codegen modes` writes over.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs (setMode); ExtLibs/ArduPilot/Modes.cs`

use crate::modes::VehicleFamily;

impl VehicleFamily {
    /// The mode number for a name, matched without regard to case or surrounding spaces:
    /// "AUTO", "Auto" and " auto " are all Auto. `None` for a name this family has not got.
    #[must_use]
    pub fn mode_number(self, name: &str) -> Option<u32> {
        let wanted = name.trim().to_ascii_uppercase();
        self.modes()
            .iter()
            .find(|(_, mode)| mode.to_ascii_uppercase() == wanted)
            .map(|(number, _)| *number)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The name is matched without regard to case; a name the family lacks is `None`, and the
    /// number found names the same mode back.
    #[test]
    fn a_mode_number_is_found_by_name_without_regard_to_case() {
        assert_eq!(VehicleFamily::Copter.mode_number("AUTO"), Some(3));
        assert_eq!(VehicleFamily::Copter.mode_number("auto"), Some(3));
        assert_eq!(VehicleFamily::Copter.mode_number(" Guided "), Some(4));
        assert_eq!(VehicleFamily::Plane.mode_number("FBWA"), Some(5));
        assert_eq!(VehicleFamily::Rover.mode_number("Steering"), Some(3));
        assert_eq!(VehicleFamily::Copter.mode_number("Steering"), None);
        assert_eq!(VehicleFamily::Copter.mode_number(""), None);
        for family in [
            VehicleFamily::Copter,
            VehicleFamily::Plane,
            VehicleFamily::Rover,
        ] {
            for (number, name) in family.modes() {
                assert_eq!(family.mode_number(name), Some(*number), "{name}");
                assert_eq!(family.mode_name(*number), Some(*name), "{number}");
            }
        }
    }
}
