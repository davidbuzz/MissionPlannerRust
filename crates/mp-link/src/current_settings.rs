//! `CurrentState.UpdateCurrentSettings` as the link runs it: which vehicles, how often, and the
//! telemetry stream requests it makes.
//!
//! Mission Planner's serial reader reads whatever each port has waiting and then calls
//! `UpdateCurrentSettings` on every vehicle listed on that port, once a pass of a loop that waits
//! a millisecond between passes (`MainV2.cs:2617-2621, 3027-3069`); a log played on the flight
//! screen does the same after each packet (`GCSViews/FlightData.cs:3533-3543`). The method rate
//! limits itself to one run in 50 ms of the wall clock, per vehicle, from the moment that
//! vehicle's state was made (`CurrentState.cs:131, 4585-4587`). The link thread does exactly
//! that, after each read: [`Clocks::due`] is the rate limit, and
//! [`mp_vehicle::VehicleState::update_current_settings`] is the once-a-second counting inside it.
//!
//! "Every vehicle listed" is `MAVList`'s visible list: a system and component become visible
//! when they send a `HEARTBEAT`, a `HIGH_LATENCY2` or a `UAVCAN_NODE_STATUS`; anything else they
//! send makes them a hidden entry that is never updated (`MAVList.cs:25-30, 101-126`;
//! `MAVLinkInterface.cs:5041-5047, 5276-5360`).
//!
//! Inside the rate limit, the method also asks the vehicle for its telemetry: seven
//! `REQUEST_DATA_STREAM`s at the vehicle's own [`StreamRates`] - the first time it runs with the
//! port open, then [`crate::ProtocolTimeouts::stream_rerequest`] after each time - never while a
//! log is being played, which the C# plays with its port closed (`CurrentState.cs:4632-4663`).
//! Each goes out as `requestDatastream` sends it: twice, with the rate as a byte, and not at all
//! for a rate of -1 (`MAVLinkInterface.cs:3061-3073, 3218-3220, 3247-3264`).
//!
//! Not ported, each for a reason given where it would be:
//! * `requestDatastream`'s `hzratecheck`, which skips a stream the vehicle already sends at about
//!   the rate asked: this application does not count packets per message, so every rate but -1
//!   is sent - as the Planner page's port of the same method does (`config/planner.rs`);
//! * `MAV.Camera?.RequestMessageIntervals` and `MAV.GimbalManager?.Discover()` after the streams:
//!   there is no camera or gimbal manager object for them to call;
//! * `linkqualitygcs`, which `mp_vehicle::link_quality` works out from each packet, and
//!   `dowindcalc`, which is not ported (see `VehicleState::wind_speed`).

use std::time::{Duration, Instant};

use mp_mavlink_dialects::all::{MavMessage, RequestDataStream};
use mp_vehicle::{StreamRates, VehicleId};

/// `UpdateCurrentSettings`' rate limit: `DateTime.Now > lastupdate.AddMilliseconds(50)`.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4585`
pub const UPDATE_INTERVAL: Duration = Duration::from_millis(50);

/// `MAV_DATA_STREAM_RAW_SENSORS`.
const RAW_SENSORS: u8 = 1;
/// `MAV_DATA_STREAM_EXTENDED_STATUS`.
const EXTENDED_STATUS: u8 = 2;
/// `MAV_DATA_STREAM_RC_CHANNELS`.
const RC_CHANNELS: u8 = 3;
/// `MAV_DATA_STREAM_POSITION`.
const POSITION: u8 = 6;
/// `MAV_DATA_STREAM_EXTRA1`.
const EXTRA1: u8 = 10;
/// `MAV_DATA_STREAM_EXTRA2`.
const EXTRA2: u8 = 11;
/// `MAV_DATA_STREAM_EXTRA3`.
const EXTRA3: u8 = 12;

/// The link's own clocks for one vehicle: the C#'s `lastupdate` and `lastdata`, and whether the
/// vehicle is on `MAVList`'s visible list.
#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    /// `lastupdate`: when `UpdateCurrentSettings` last ran, starting when the state was made -
    /// on the vehicle's first packet. `// C#: ExtLibs/ArduPilot/CurrentState.cs:131`
    last_update: Instant,
    /// When the streams may be asked for again: `lastdata` plus eight seconds. `None` is the C#'s
    /// `DateTime.MinValue`, due at once. `// C#: ExtLibs/ArduPilot/CurrentState.cs:121`
    streams_due: Option<Instant>,
    /// Whether it has sent what makes `MAVList.Create` list it.
    visible: bool,
}

impl Clocks {
    /// A vehicle first heard at `now`.
    #[must_use]
    pub const fn new(now: Instant) -> Self {
        Self {
            last_update: now,
            streams_due: None,
            visible: false,
        }
    }

    /// Takes note of a message the vehicle sent: `HEARTBEAT`, `HIGH_LATENCY2` and
    /// `UAVCAN_NODE_STATUS` list it. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5276-5360`
    pub const fn heard(&mut self, message: &MavMessage) {
        if matches!(
            message,
            MavMessage::Heartbeat(_)
                | MavMessage::HighLatency2(_)
                | MavMessage::UavcanNodeStatus(_)
        ) {
            self.visible = true;
        }
    }

    /// Whether `UpdateCurrentSettings` runs for this vehicle now, and if so, notes that it has:
    /// listed, and more than [`UPDATE_INTERVAL`] since it last ran.
    /// `// C#: MainV2.cs:3058-3069; ExtLibs/ArduPilot/CurrentState.cs:4583-4587`
    pub fn due(&mut self, now: Instant) -> bool {
        if !self.visible || now.saturating_duration_since(self.last_update) <= UPDATE_INTERVAL {
            return false;
        }
        self.last_update = now;
        true
    }

    /// Whether the streams are to be asked for now, and if so, notes that they have been and
    /// when to ask again: `!(lastdata.AddSeconds(8) > DateTime.Now)`, then `lastdata =
    /// DateTime.Now.AddSeconds(30)` - `rerequest` is the two together.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4633-4634, 4662`
    pub fn streams_due(&mut self, now: Instant, rerequest: Duration) -> bool {
        if self.streams_due.is_some_and(|due| now < due) {
            return false;
        }
        self.streams_due = Some(now + rerequest);
        true
    }
}

/// The seven streams `UpdateCurrentSettings` asks for, in its order, each with the rate it reads
/// from the vehicle's state. `// C#: ExtLibs/ArduPilot/CurrentState.cs:4638-4652`
#[must_use]
pub const fn stream_requests(rates: StreamRates) -> [(u8, i32); 7] {
    [
        (EXTENDED_STATUS, rates.status),
        (POSITION, rates.position),
        (EXTRA1, rates.attitude),
        (EXTRA2, rates.attitude),
        (EXTRA3, rates.sensors),
        (RAW_SENSORS, rates.sensors),
        (RC_CHANNELS, rates.rc),
    ]
}

/// `requestDatastream(stream, hz, sysid, compid)`'s request: `REQUEST_DATA_STREAM` starting the
/// stream at the rate as a byte - which the caller sends twice, as `getDatastream` does - or
/// nothing for a rate of -1. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3061-3073,
/// 3218-3220, 3247-3264`
#[must_use]
pub fn request_datastream(target: VehicleId, stream: u8, hz: i32) -> Option<MavMessage> {
    if hz == -1 {
        return None;
    }
    // `(byte) hzrate`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let rate = u16::from(hz as u8);
    Some(MavMessage::RequestDataStream(RequestDataStream {
        req_message_rate: rate,
        target_system: target.sysid,
        target_component: target.compid,
        req_stream_id: stream,
        start_stop: 1,
    }))
}

#[cfg(test)]
mod tests {
    use mp_mavlink_dialects::all::{Attitude, Heartbeat};

    use super::*;

    #[test]
    fn a_vehicle_is_updated_once_listed_and_then_every_fifty_milliseconds() {
        let start = Instant::now();
        let mut clocks = Clocks::new(start);
        let later = |millis| start + Duration::from_millis(millis);
        // Hidden until it sends a heartbeat.
        clocks.heard(&MavMessage::Attitude(Attitude {
            time_boot_ms: 0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        }));
        assert!(!clocks.due(later(100)));
        clocks.heard(&MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 2,
            autopilot: 3,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 3,
        }));
        // Counted from when the state was made, not from the heartbeat.
        assert!(clocks.due(later(100)));
        // Exactly 50 ms is not more than 50 ms.
        assert!(!clocks.due(later(150)));
        assert!(clocks.due(later(151)));
    }

    #[test]
    fn the_streams_go_at_once_then_thirty_eight_seconds_after_each_time() {
        let start = Instant::now();
        let mut clocks = Clocks::new(start);
        let rerequest = crate::ProtocolTimeouts::default().stream_rerequest;
        assert_eq!(rerequest, Duration::from_secs(38));
        assert!(clocks.streams_due(start, rerequest));
        assert!(!clocks.streams_due(start + Duration::from_secs(37), rerequest));
        assert!(clocks.streams_due(start + Duration::from_secs(38), rerequest));
    }

    #[test]
    fn the_requests_read_each_rate_and_a_rate_of_minus_one_sends_nothing() {
        let rates = StreamRates {
            attitude: 10,
            position: 3,
            status: 1,
            sensors: 7,
            rc: -1,
        };
        assert_eq!(
            stream_requests(rates),
            [(2, 1), (6, 3), (10, 10), (11, 10), (12, 7), (1, 7), (3, -1)]
        );
        let target = VehicleId::new(3, 1);
        assert!(request_datastream(target, RC_CHANNELS, -1).is_none());
        let Some(MavMessage::RequestDataStream(request)) =
            request_datastream(target, POSITION, 300)
        else {
            panic!("a request");
        };
        // `(byte) 300`.
        assert_eq!(request.req_message_rate, 44);
        assert_eq!((request.target_system, request.target_component), (3, 1));
        assert_eq!((request.req_stream_id, request.start_stop), (POSITION, 1));
    }
}
