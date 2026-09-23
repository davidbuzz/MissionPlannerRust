//! Sensor health, as the vehicle reports it in `SYS_STATUS`.
//!
//! Three bitmasks over the same bits: what the airframe has, what is switched on, and what is
//! working. A sensor that is present and enabled but not healthy is the usual reason an aircraft
//! refuses to arm, and it is the one thing a pilot most wants named.

use mp_mavlink_dialects::all::MavSysStatusSensor;

/// What the vehicle says about its sensors.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Sensors {
    /// Sensors the airframe has.
    pub present: u32,
    /// Sensors that are switched on.
    pub enabled: u32,
    /// Sensors that are working.
    pub health: u32,
    /// Whether anything has been reported at all.
    ///
    /// Distinguishes "everything is fine" from "the vehicle has not said": all-zero masks look
    /// identical, and reporting a silent vehicle as healthy is the wrong way round.
    pub reported: bool,
}

impl Sensors {
    /// The sensors that are switched on but not working, by name.
    ///
    /// Only the enabled ones. A copter with no airspeed sensor reports differential pressure as
    /// absent and unhealthy, and listing it would bury the one that matters.
    #[must_use]
    pub fn unhealthy(&self) -> Vec<&'static str> {
        if !self.reported {
            return Vec::new();
        }
        let failing = self.enabled & self.present & !self.health;
        (0..32)
            .map(|bit| 1_u32 << bit)
            .filter(|mask| failing & mask != 0)
            .filter_map(|mask| sensor_name(mask))
            .collect()
    }

    /// How many sensors are switched on but not working.
    ///
    /// Counted from the bits rather than from [`Sensors::unhealthy`], so a sensor this dialect has
    /// no name for is still counted. A pilot told "all sensors healthy" while one is failing is
    /// worse off than one told "1 sensor unhealthy" with no name for it.
    #[must_use]
    pub const fn unhealthy_count(&self) -> u32 {
        if !self.reported {
            return 0;
        }
        (self.enabled & self.present & !self.health).count_ones()
    }

    /// Whether everything switched on is working.
    #[must_use]
    pub const fn all_healthy(&self) -> bool {
        self.reported && self.unhealthy_count() == 0
    }
}

/// A short name for one sensor bit.
///
/// Derived from the generated enum, so it follows the definitions rather than a hand-written list
/// that would drift. The shared prefix is stripped, because a column of `MAV_SYS_STATUS_...` is
/// unreadable and every entry has it.
///
/// Two prefixes, not one: most bits are named `MAV_SYS_STATUS_SENSOR_3D_GYRO`, but the ones added
/// later are `MAV_SYS_STATUS_AHRS` and `MAV_SYS_STATUS_PREARM_CHECK` with no `SENSOR` in them -
/// and those are exactly the bits a pre-arm panel shows most often.
#[must_use]
pub fn sensor_name(mask: u32) -> Option<&'static str> {
    MavSysStatusSensor(mask).name().map(|name| {
        name.strip_prefix("MAV_SYS_STATUS_SENSOR_")
            .or_else(|| name.strip_prefix("MAV_SYS_STATUS_"))
            .unwrap_or(name)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 3D gyro, accelerometer, magnetometer and GPS.
    const GYRO: u32 = 1;
    const ACCEL: u32 = 2;
    const MAG: u32 = 4;
    const GPS: u32 = 32;

    #[test]
    fn a_vehicle_that_has_said_nothing_is_not_reported_as_healthy() {
        // All-zero masks look identical to "everything is fine"; reporting a silent vehicle as
        // healthy is the wrong way round.
        let quiet = Sensors::default();
        assert!(!quiet.all_healthy());
        assert!(quiet.unhealthy().is_empty());
        assert_eq!(quiet.unhealthy_count(), 0);
    }

    #[test]
    fn an_enabled_sensor_that_is_not_working_is_named() {
        let sensors = Sensors {
            present: GYRO | ACCEL | MAG,
            enabled: GYRO | ACCEL | MAG,
            health: GYRO | ACCEL,
            reported: true,
        };
        assert_eq!(sensors.unhealthy(), vec!["3D_MAG"]);
        assert_eq!(sensors.unhealthy_count(), 1);
        assert!(!sensors.all_healthy());
    }

    #[test]
    fn a_sensor_the_airframe_does_not_have_is_not_a_fault() {
        // A copter with no airspeed sensor reports differential pressure absent and unhealthy.
        // Listing it would bury the one that matters.
        let sensors = Sensors {
            present: GYRO | ACCEL,
            enabled: GYRO | ACCEL,
            health: GYRO | ACCEL,
            reported: true,
        };
        assert!(sensors.unhealthy().is_empty());
        assert!(sensors.all_healthy());
    }

    #[test]
    fn a_sensor_that_is_present_but_switched_off_is_not_a_fault() {
        let sensors = Sensors {
            present: GYRO | GPS,
            enabled: GYRO,
            health: GYRO,
            reported: true,
        };
        assert!(sensors.all_healthy(), "{:?}", sensors.unhealthy());
    }

    #[test]
    fn an_unnamed_sensor_is_still_counted() {
        // A pilot told "all sensors healthy" while one is failing is worse off than one told
        // "1 unhealthy" with no name for it.
        let unknown = 1_u32 << 31;
        let sensors = Sensors {
            present: unknown,
            enabled: unknown,
            health: 0,
            reported: true,
        };
        assert_eq!(sensors.unhealthy_count(), 1);
        assert!(!sensors.all_healthy());
    }

    #[test]
    fn several_faults_are_all_listed() {
        let sensors = Sensors {
            present: GYRO | ACCEL | MAG | GPS,
            enabled: GYRO | ACCEL | MAG | GPS,
            health: GYRO,
            reported: true,
        };
        let failing = sensors.unhealthy();
        assert_eq!(sensors.unhealthy_count(), 3);
        assert!(failing.contains(&"3D_ACCEL"), "{failing:?}");
        assert!(failing.contains(&"3D_MAG"), "{failing:?}");
        assert!(failing.contains(&"GPS"), "{failing:?}");
    }

    #[test]
    fn names_come_from_the_definitions_and_lose_the_shared_prefix() {
        // A column of MAV_SYS_STATUS_SENSOR_... is unreadable and every entry shares it.
        assert_eq!(sensor_name(GYRO), Some("3D_GYRO"));
        assert_eq!(sensor_name(GPS), Some("GPS"));
        assert_eq!(sensor_name(1 << 31), None);

        // The bits added later have no SENSOR in their name, and they are exactly the ones a
        // pre-arm panel shows most often. Against a real board these read as
        // "mav_sys_status_ahrs" until both prefixes are handled.
        let named: Vec<&str> = (0..32)
            .map(|bit| 1_u32 << bit)
            .filter_map(sensor_name)
            .collect();
        assert!(
            named.iter().all(|name| !name.starts_with("MAV_")),
            "a prefix survived: {named:?}"
        );
        assert!(named.contains(&"AHRS"), "{named:?}");
        assert!(named.contains(&"PREARM_CHECK"), "{named:?}");
    }
}
