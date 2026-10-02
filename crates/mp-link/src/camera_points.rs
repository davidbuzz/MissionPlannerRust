//! `MAVState.camerapoints`: every `CAMERA_FEEDBACK` a vehicle has sent, in arrival order, which
//! the flight screen's map draws as photo markers and works `timesincelastshot` out from.
//!
//! `processInfoFromStream` keeps a shot unless it has the time of the one before it (the
//! autopilot sends each twice), first dropping any earlier shot with the same camera and image
//! numbers - a re-sent feedback replaces the shot it is for. The list is unbounded, as the C#'s
//! is, a shot being a few dozen bytes; it starts over with the link, as `MAVState`'s does.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5736-5745, MAVState.cs:108, 319`

use mp_mavlink_dialects::all::CameraFeedback;
use mp_vehicle::VehicleId;

/// Every vehicle's shots, the vehicle's own list each.
#[derive(Debug, Default)]
pub struct CameraPoints {
    by_vehicle: Vec<(VehicleId, Vec<CameraFeedback>)>,
}

impl CameraPoints {
    /// A vehicle's shots, oldest first: `MAV.camerapoints.ToArray()`.
    #[must_use]
    pub fn points(&self, id: VehicleId) -> Vec<CameraFeedback> {
        self.list(id).cloned().unwrap_or_default()
    }

    /// `CAMERA_FEEDBACK.time_usec` of each shot, in the list's order, for `timesincelastshot`.
    pub fn times(&self, id: VehicleId) -> impl Iterator<Item = u64> + '_ {
        self.list(id)
            .into_iter()
            .flat_map(|list| list.iter().map(|point| point.time_usec))
    }

    /// `processInfoFromStream`'s `CAMERA_FEEDBACK`: nothing when the last shot has this one's
    /// time; else every shot with this one's `cam_idx * 256 + img_idx` removed and this one added
    /// at the end. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5736-5745`
    pub fn observe(&mut self, id: VehicleId, point: &CameraFeedback) {
        let list = self.list_mut(id);
        if list
            .last()
            .is_some_and(|last| last.time_usec == point.time_usec)
        {
            return;
        }
        let key = |shot: &CameraFeedback| u32::from(shot.cam_idx) * 256 + u32::from(shot.img_idx);
        let same = key(point);
        list.retain(|shot| key(shot) != same);
        list.push(*point);
    }

    /// `camerapoints.Clear()`.
    pub fn clear(&mut self, id: VehicleId) {
        self.list_mut(id).clear();
    }

    fn list(&self, id: VehicleId) -> Option<&Vec<CameraFeedback>> {
        self.by_vehicle
            .iter()
            .find(|(vehicle, _)| *vehicle == id)
            .map(|(_, list)| list)
    }

    fn list_mut(&mut self, id: VehicleId) -> &mut Vec<CameraFeedback> {
        let index = match self
            .by_vehicle
            .iter()
            .position(|(vehicle, _)| *vehicle == id)
        {
            Some(index) => index,
            None => {
                self.by_vehicle.push((id, Vec::new()));
                self.by_vehicle.len() - 1
            }
        };
        // Just found or just pushed.
        #[allow(clippy::indexing_slicing)]
        &mut self.by_vehicle[index].1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VEHICLE: VehicleId = VehicleId::new(1, 1);

    fn shot(time_usec: u64, cam_idx: u8, img_idx: u16) -> CameraFeedback {
        CameraFeedback {
            time_usec,
            cam_idx,
            img_idx,
            lat: -274_698_000,
            lng: 1_530_251_000,
            alt_msl: 63.8,
            alt_rel: 38.7,
            roll: 0.0,
            pitch: 0.0,
            yaw: 90.0,
            foc_len: 0.0,
            target_system: 1,
            flags: 0,
            completed_captures: 0,
        }
    }

    /// A shot is kept once though sent twice; a re-sent image replaces its earlier shot; the
    /// list is the vehicle's own and empty for another.
    #[test]
    fn shots_are_kept_once_and_a_resend_replaces_its_image() {
        let mut points = CameraPoints::default();
        points.observe(VEHICLE, &shot(1_000, 0, 1));
        points.observe(VEHICLE, &shot(1_000, 0, 1));
        points.observe(VEHICLE, &shot(2_000, 0, 2));
        assert_eq!(points.points(VEHICLE).len(), 2);
        // Image 1 again, later: the old one goes, the new one is last.
        points.observe(VEHICLE, &shot(3_000, 0, 1));
        let list = points.points(VEHICLE);
        assert_eq!(list.iter().map(|p| p.img_idx).collect::<Vec<_>>(), [2, 1]);
        assert_eq!(points.times(VEHICLE).collect::<Vec<_>>(), [2_000, 3_000]);
        // Another camera's image 1 is a different key.
        points.observe(VEHICLE, &shot(4_000, 1, 1));
        assert_eq!(points.points(VEHICLE).len(), 3);
        assert!(points.points(VehicleId::new(2, 1)).is_empty());
        points.clear(VEHICLE);
        assert!(points.points(VEHICLE).is_empty());
    }
}
