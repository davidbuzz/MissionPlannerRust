//! How long each protocol step waits for its answer, and how often it asks again.
//!
//! Every number here is Mission Planner's, read from `MAVLinkInterface.cs` and cited where it is
//! set. They live in one struct rather than as constants beside each machine for one reason: a
//! test has to run the C#'s retry *counts* without spending the C#'s wall-clock time. Sixty
//! seconds of `getWP` timeouts is not a test anyone runs twice, so a test divides the waits with
//! [`ProtocolTimeouts::faster`] and keeps every count exactly as the C# has it.

use std::time::Duration;

/// One step's patience: how long to wait for an answer, and how many times to send again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Retry {
    /// How long one send waits for its answer before the step is sent again.
    pub timeout: Duration,
    /// How many times the step is sent again after the first send goes unanswered. The step goes
    /// on the wire `retries + 1` times before the machine gives up, because that is how the C#
    /// loops count: `int retrys = 3;` then one resend per timeout while `retrys > 0`.
    pub retries: u8,
}

impl Retry {
    const fn new(millis: u64, retries: u8) -> Self {
        Self {
            timeout: Duration::from_millis(millis),
            retries,
        }
    }

    /// How many times the step goes on the wire before the machine gives up.
    #[must_use]
    pub const fn sends(self) -> u16 {
        self.retries as u16 + 1
    }
}

/// Every wait and every retry count the protocol machines use.
///
/// [`Default`] is Mission Planner's values. Only a test should want anything else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProtocolTimeouts {
    /// `PARAM_SET` until the vehicle echoes the parameter back.
    ///
    /// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1748 (`retrys = 3`), :1754 (700 ms).
    pub param_set: Retry,
    /// `PARAM_REQUEST_READ` until the parameter arrives.
    ///
    /// C#: MAVLinkInterface.cs:2329 (`retrys = 3`), :2333 (700 ms).
    pub param_read: Retry,
    /// `COMMAND_LONG` until its `COMMAND_ACK`.
    ///
    /// C#: MAVLinkInterface.cs:2729 (`retrys = 3`), :2731 (`timeout = 2000`).
    pub command: Retry,
    /// `MAV_CMD_COMPONENT_ARM_DISARM`, which waits longer "as may need an imu calib".
    ///
    /// C#: MAVLinkInterface.cs:2764-2768 (`timeout = 10000`, the retries left at 3).
    pub command_arm: Retry,
    /// `MAV_CMD_PREFLIGHT_CALIBRATION` and `MAV_CMD_FLASH_BOOTLOADER`: slow, and sent twice at most.
    ///
    /// C#: MAVLinkInterface.cs:2748-2757 (`retrys = 1; timeout = 25000;` for both).
    pub command_slow: Retry,
    /// `MISSION_SET_CURRENT` until a `MISSION_CURRENT` arrives.
    ///
    /// C#: MAVLinkInterface.cs:2472 (`retrys = 5`), :2476 (2000 ms).
    pub set_current: Retry,
    /// `MISSION_REQUEST_LIST` until `MISSION_COUNT`: the first step of a download.
    ///
    /// C#: MAVLinkInterface.cs:3297 (`retrys = 6`), :3301 (700 ms), `getWPCountAsync`.
    pub mission_list: Retry,
    /// `MISSION_REQUEST_INT` until that item arrives.
    ///
    /// C#: MAVLinkInterface.cs:3459 (`retrys = 5`), :3463 (2500 ms), `getWPAsync`.
    pub mission_item_request: Retry,
    /// `MISSION_COUNT` until the vehicle asks for the first item: the first step of an upload.
    ///
    /// C#: MAVLinkInterface.cs:3779 (`retrys = 3`), :3783 (700 ms), `setWPTotalAsync`.
    pub mission_count: Retry,
    /// `MISSION_ITEM_INT` until the vehicle asks for the next one or acknowledges the upload.
    ///
    /// C#: MAVLinkInterface.cs:4250 (`retrys = 10`), :4254 (450 ms), `setWPAsync`.
    pub mission_item_send: Retry,
    /// After `MAV_MISSION_INVALID_SEQUENCE`, how long to wait for the vehicle to say which item
    /// it wants before telling it where to resume.
    ///
    /// C#: MAVLinkInterface.cs:4380 (1500 ms), `getRequestedWPNoAsync`, called from
    /// ExtLibs/ArduPilot/mav_mission.cs:133.
    pub mission_resync: Duration,
    /// How long the parameter stream may be quiet before the download starts asking again.
    ///
    /// C#: MAVLinkInterface.cs:2114 (4000 ms "between valid packets").
    pub param_list_quiet: Duration,
    /// How often, once asking one by one, the next burst of `PARAM_REQUEST_READ` goes out.
    ///
    /// C#: MAVLinkInterface.cs:2135 (`lastonebyone.AddMilliseconds(1000)`).
    pub param_list_round: Duration,
    /// How many times the whole list is asked for again while less than three quarters arrived.
    ///
    /// C#: MAVLinkInterface.cs:2117 (`retry < 2`).
    pub param_list_full_retries: u8,
    /// Every MAVFTP command's `RetryTimeout`, from `MAVFtp.cs`; see [`mp_ftp::mavftp::retry`].
    pub ftp: mp_ftp::mavftp::FtpTimeouts,
    /// How long after asking a vehicle for its telemetry streams `UpdateCurrentSettings` asks
    /// again: it sets `lastdata` thirty seconds ahead "to prevent flooding" and asks once it is
    /// eight seconds past that, whether or not the streams came.
    ///
    /// C#: ExtLibs/ArduPilot/CurrentState.cs:4633 (`lastdata.AddSeconds(8)`), :4662
    /// (`DateTime.Now.AddSeconds(30)`).
    pub stream_rerequest: Duration,
    /// `RALLY_FETCH_POINT` until the vehicle sends the point back, `getRallyPoint`.
    ///
    /// C#: MAVLinkInterface.cs:6363 (`retrys = 3`), :6367 (700 ms).
    pub rally_fetch: Retry,
    /// `getHomePositionAsync`: `GET_HOME_POSITION` until a `HOME_POSITION` arrives.
    /// C#: MAVLinkInterface.cs:3362 (`retrys = 3`), :3366 (700 ms).
    pub home_position: Retry,
}

impl Default for ProtocolTimeouts {
    fn default() -> Self {
        Self {
            param_set: Retry::new(700, 3),
            param_read: Retry::new(700, 3),
            command: Retry::new(2000, 3),
            command_arm: Retry::new(10_000, 3),
            command_slow: Retry::new(25_000, 1),
            set_current: Retry::new(2000, 5),
            mission_list: Retry::new(700, 6),
            mission_item_request: Retry::new(2500, 5),
            mission_count: Retry::new(700, 3),
            mission_item_send: Retry::new(450, 10),
            mission_resync: Duration::from_millis(1500),
            param_list_quiet: Duration::from_millis(4000),
            param_list_round: Duration::from_millis(1000),
            param_list_full_retries: 2,
            ftp: mp_ftp::mavftp::FtpTimeouts::default(),
            stream_rerequest: Duration::from_secs(30 + 8),
            rally_fetch: Retry::new(700, 3),
            home_position: Retry::new(700, 3),
        }
    }
}

impl ProtocolTimeouts {
    /// Every wait divided by `divisor`, every count unchanged.
    ///
    /// For tests, which need the C#'s retry counts on the wire without its wall-clock time. Not
    /// something to fly with: the C#'s waits are tuned to a 57,600 baud radio, and a GCS that
    /// gives up faster than the link can answer turns a slow link into a failed one.
    #[must_use]
    pub fn faster(self, divisor: u32) -> Self {
        let divisor = divisor.max(1);
        let scale = |retry: Retry| Retry {
            timeout: retry.timeout / divisor,
            retries: retry.retries,
        };
        Self {
            param_set: scale(self.param_set),
            param_read: scale(self.param_read),
            command: scale(self.command),
            command_arm: scale(self.command_arm),
            command_slow: scale(self.command_slow),
            set_current: scale(self.set_current),
            mission_list: scale(self.mission_list),
            mission_item_request: scale(self.mission_item_request),
            mission_count: scale(self.mission_count),
            mission_item_send: scale(self.mission_item_send),
            mission_resync: self.mission_resync / divisor,
            param_list_quiet: self.param_list_quiet / divisor,
            param_list_round: self.param_list_round / divisor,
            param_list_full_retries: self.param_list_full_retries,
            ftp: self.ftp.faster(divisor),
            stream_rerequest: self.stream_rerequest / divisor,
            rally_fetch: scale(self.rally_fetch),
            home_position: scale(self.home_position),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn faster_keeps_every_count() {
        let fast = ProtocolTimeouts::default().faster(100);
        let slow = ProtocolTimeouts::default();
        assert_eq!(fast.param_set.retries, slow.param_set.retries);
        assert_eq!(fast.mission_item_send.retries, 10);
        assert_eq!(fast.param_list_full_retries, 2);
        assert_eq!(fast.param_set.timeout, Duration::from_millis(7));
        assert_eq!(fast.param_list_quiet, Duration::from_millis(40));
        assert_eq!(fast.ftp.other.retries, 30);
        assert_eq!(fast.ftp.other.timeout, Duration::from_millis(10));
    }
}
