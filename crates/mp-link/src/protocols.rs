// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! The link thread's side of [`crate::camera`] and [`crate::gimbal_manager`]: each made when its
//! component's first heartbeat arrives (`MAVDetected`), started two seconds on, told every packet,
//! and asked for its messages when the telemetry streams are.
//!
//! "Two seconds on" is from the heartbeat. The C# waits first for its `Open` to finish
//! (`_openComplete`), which a link here has no equivalent of: it has no connect sequence that
//! holds the port.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:500-586, ExtLibs/ArduPilot/CurrentState.cs:4657-4658`

use mp_os::Lock as _;
use web_time::{Duration, Instant};

use mp_mavlink_dialects::all::MavMessage;
use mp_vehicle::VehicleId;

use crate::camera::{self, Camera, InfoRequest};
use crate::gimbal_manager::GimbalManager;
use crate::requests::{Request, RequestKind, RequestOutcome, RequestState};
use crate::{RequestId, Shared};

/// `await Task.Delay(2000)` before `StartID` and `Discover`.
pub(crate) const START_DELAY: Duration = Duration::from_secs(2);

/// Components waiting for their `StartID` and first `Discover`.
pub(crate) type Starts = Vec<(VehicleId, Instant)>;

/// `OnMAVDetected`: a camera and a gimbal manager for the components the C# makes them for,
/// replacing any there were, started [`START_DELAY`] from `now`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:500-586`
pub(crate) fn detected(shared: &Shared, id: VehicleId, now: Instant, starts: &mut Starts) {
    let camera = camera::is_camera_component(id.compid);
    let manager = camera::is_gimbal_manager_component(id.compid);
    if camera && let Ok(mut held) = shared.cameras.os_lock() {
        held.insert(id, Camera::new(id));
    }
    if manager && let Ok(mut held) = shared.gimbal_managers.os_lock() {
        held.insert(id, GimbalManager::default());
    }
    if camera || manager {
        starts.push((id, now + START_DELAY));
    }
}

/// Every packet: each gimbal manager's `MessagesHandler`, whoever sent it, and the sender's own
/// camera's `ParseMessages`.
/// `// C#: GimbalManagerProtocol.cs:52-84; CameraProtocol.cs:270-298`
pub(crate) fn observe(shared: &Shared, from: VehicleId, message: &MavMessage) {
    match message {
        MavMessage::GimbalManagerInformation(_)
        | MavMessage::GimbalManagerStatus(_)
        | MavMessage::GimbalDeviceAttitudeStatus(_) => {
            if let Ok(mut held) = shared.gimbal_managers.os_lock() {
                for manager in held.values_mut() {
                    manager.observe(message);
                }
            }
        }
        MavMessage::CameraInformation(_)
        | MavMessage::CameraSettings(_)
        | MavMessage::CameraCaptureStatus(_)
        | MavMessage::VideoStreamInformation(_)
        | MavMessage::CameraFovStatus(_)
        | MavMessage::CameraTrackingImageStatus(_) => {
            if let (Ok(mut cameras), Ok(mut streams)) =
                (shared.cameras.os_lock(), shared.video_streams.os_lock())
                && let Some(camera) = cameras.get_mut(&from)
            {
                camera.observe(from, message, &mut streams);
            }
        }
        _ => {}
    }
}

/// `RequestCameraInformationAsync`'s first step: `REQUEST_MESSAGE(CAMERA_INFORMATION)` queued as
/// a command waited on, unless one is already waiting. False for a component with no camera, or
/// one not started. `// C#: CameraProtocol.cs:224-236`
pub(crate) fn request_information(shared: &Shared, id: VehicleId) -> bool {
    let Ok(mut cameras) = shared.cameras.os_lock() else {
        return false;
    };
    let Some(camera) = cameras.get_mut(&id) else {
        return false;
    };
    if camera.info_request != InfoRequest::Idle {
        return true;
    }
    let Some((command, params)) = camera.information_request() else {
        return false;
    };
    let request = RequestId(
        shared
            .next_request
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed),
    );
    if let Ok(mut queue) = shared.request_queue.os_lock() {
        queue.push((
            request,
            Request::new(
                id,
                RequestKind::Command {
                    command,
                    params,
                    require_ack: true,
                },
            ),
        ));
    }
    camera.info_request = InfoRequest::Waiting(request);
    true
}

/// Whether a camera's `RequestCameraInformationAsync` is still under way.
pub(crate) fn information_pending(shared: &Shared, id: VehicleId) -> bool {
    shared
        .cameras
        .os_lock()
        .ok()
        .and_then(|held| {
            held.get(&id)
                .map(|camera| camera.info_request != InfoRequest::Idle)
        })
        .unwrap_or(false)
}

/// Each pass: the components whose two seconds are up started - the camera's `StartID` and its
/// information asked for, the gimbal manager's `Discover` - and each camera whose information
/// request has its answer given the rest of `RequestCameraInformationAsync`. What goes out
/// unwaited is put in `send`. `// C#: MAVLinkInterface.cs:512-533, 573-585; CameraProtocol.cs:224-262`
pub(crate) fn tick(shared: &Shared, now: Instant, starts: &mut Starts, send: &mut Vec<MavMessage>) {
    let mut due = Vec::new();
    starts.retain(|(id, at)| {
        if now >= *at {
            due.push(*id);
            false
        } else {
            true
        }
    });
    for id in due {
        let started = shared
            .cameras
            .os_lock()
            .ok()
            .and_then(|mut held| held.get_mut(&id).map(Camera::start))
            .is_some();
        if started {
            request_information(shared, id);
        }
        if let Ok(mut held) = shared.gimbal_managers.os_lock()
            && let Some(manager) = held.get_mut(&id)
        {
            send.push(manager.discover());
        }
    }
    // The answers: the C# awaits the first request, then sends the rest. A request that timed
    // out is its `catch`, which logs and sends nothing more.
    let Ok(mut cameras) = shared.cameras.os_lock() else {
        return;
    };
    for camera in cameras.values_mut() {
        let InfoRequest::Waiting(request) = camera.info_request else {
            continue;
        };
        let state = shared
            .requests
            .os_lock()
            .ok()
            .and_then(|held| held.get(&request).map(Request::state));
        let queued = shared
            .request_queue
            .os_lock()
            .map(|queue| queue.iter().any(|(id, _)| *id == request))
            .unwrap_or(false);
        match state {
            Some(RequestState::Finished(outcome)) => {
                camera.info_request = InfoRequest::Idle;
                match outcome {
                    RequestOutcome::Accepted { .. } | RequestOutcome::Sent => {
                        send.extend(camera.information_follow_up(true));
                    }
                    RequestOutcome::Rejected(_) => {
                        send.extend(camera.information_follow_up(false));
                    }
                    _ => {}
                }
            }
            None if !queued => camera.info_request = InfoRequest::Idle,
            _ => {}
        }
    }
}

/// Inside `UpdateCurrentSettings`' stream request: `MAV.Camera?.RequestMessageIntervals(
/// cs.ratestatus)` - its intervals and its information asked for again - and
/// `MAV.GimbalManager?.Discover()`, for the vehicle the streams were asked of.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4657-4658; CameraProtocol.cs:305-369`
pub(crate) fn on_streams(
    shared: &Shared,
    id: VehicleId,
    ratestatus: i32,
    send: &mut Vec<MavMessage>,
) {
    let started = if let Ok(held) = shared.cameras.os_lock()
        && let Some(camera) = held.get(&id)
    {
        send.extend(camera.message_intervals(ratestatus));
        ratestatus >= 0 && camera.is_started()
    } else {
        false
    };
    if started {
        request_information(shared, id);
    }
    if let Ok(mut held) = shared.gimbal_managers.os_lock()
        && let Some(manager) = held.get_mut(&id)
    {
        send.push(manager.discover());
    }
}
