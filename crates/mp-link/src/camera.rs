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

//! `CameraProtocol`: what a MAVLink camera says about itself and its video streams, and the
//! capture, zoom and tracking commands the gimbal video control sends it.
//!
//! The C# makes one for the autopilot and for each camera component (`MAV_COMP_ID_CAMERA` to
//! `MAV_COMP_ID_CAMERA6`) when that component's first heartbeat arrives, and two seconds later
//! `StartID` subscribes it to the link's packets - its own component's only - and asks for the
//! camera's information (`RequestCameraInformationAsync`). `UpdateCurrentSettings` then calls
//! `RequestMessageIntervals` each time it asks for the telemetry streams. The link thread does the
//! making, the starting and the asking (`lib.rs`); this is the state, the commands and the
//! pipeline a video stream is played with.
//! `// C#: ExtLibs/ArduPilot/Mavlink/CameraProtocol.cs:1-664,
//! ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:500-535, ExtLibs/ArduPilot/CurrentState.cs:4657`
//!
//! `VideoStreams`, static in the C# and so shared by every link, is one table per link here
//! ([`crate::Link::video_streams`]): this application has one link.
//!
//! Not ported, because nothing calls them: `StartIntervalCaptureAsync`,
//! `StopIntervalCaptureAsync` and `CalculateImagePointVector`. The image-point geometry that
//! needs `PointLatLngAlt` (`CalculateImagePointLocation`) is the flight screen's, beside the map
//! it marks (`crates/mp-gui/src/gimbal_video.rs`).

use mp_mavlink_dialects::all::{
    CameraCapFlags, CameraCaptureStatus, CameraFovStatus, CameraInformation, CameraMode,
    CameraSettings, CameraTrackingImageStatus, CameraZoomType, MavCmd, MavMessage,
    VideoStreamEncoding, VideoStreamInformation, VideoStreamType,
};
use mp_vehicle::VehicleId;

use crate::commands;
use crate::gimbal_manager::{Quaternion, Vector3};

/// `MAVLINK_MSG_ID.CAMERA_INFORMATION`.
pub const CAMERA_INFORMATION: f32 = 259.0;
/// `MAVLINK_MSG_ID.CAMERA_SETTINGS`.
const CAMERA_SETTINGS: f32 = 260.0;
/// `MAVLINK_MSG_ID.CAMERA_CAPTURE_STATUS`.
const CAMERA_CAPTURE_STATUS: f32 = 262.0;
/// `MAVLINK_MSG_ID.VIDEO_STREAM_INFORMATION`.
pub const VIDEO_STREAM_INFORMATION: f32 = 269.0;
/// `MAVLINK_MSG_ID.CAMERA_FOV_STATUS`.
const CAMERA_FOV_STATUS: f32 = 271.0;
/// `MAVLINK_MSG_ID.CAMERA_TRACKING_IMAGE_STATUS`.
const CAMERA_TRACKING_IMAGE_STATUS: f32 = 275.0;

/// A `MAV_CMD` as the `u16` a `COMMAND_LONG` carries.
fn cmd(command: MavCmd) -> u16 {
    u16::try_from(command.0).unwrap_or(u16::MAX)
}

/// Whether the C# makes a `CameraProtocol` for this component: the autopilot or a camera.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:505-507`
#[must_use]
pub const fn is_camera_component(compid: u8) -> bool {
    compid == 1 || (compid >= 100 && compid <= 105)
}

/// Whether the C# makes a `GimbalManagerProtocol` for this component: the autopilot, or
/// `MAV_COMP_ID_MISSIONPLANNER` (190) to `MAV_COMP_ID_ONBOARD_COMPUTER4` (194).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:565-567`
#[must_use]
pub const fn is_gimbal_manager_component(compid: u8) -> bool {
    compid == 1 || (compid >= 190 && compid <= 194)
}

/// A zero-terminated MAVLink string: `Encoding.UTF8.GetString(bytes).Split('\0')[0]`.
#[must_use]
pub fn c_string(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(bytes.get(..end).unwrap_or_default()).into_owned()
}

/// The digits after the first colon: `Regex.Match(uri, ":(\d+)")`.
fn port_after_colon(uri: &str) -> Option<i64> {
    let mut rest = uri;
    while let Some(colon) = rest.find(':') {
        rest = rest.get(colon + 1..).unwrap_or_default();
        let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
        if !digits.is_empty() {
            return digits.parse().ok();
        }
    }
    None
}

/// `GStreamerPipeline(stream)`: the pipeline that plays a reported video stream - the URI itself
/// after a `gst://`, otherwise one per stream type, and `""` for a UDP stream with no usable port,
/// a TCP one with no host and port, or a type it does not know.
/// `// C#: ExtLibs/ArduPilot/Mavlink/CameraProtocol.cs:37-93`
#[must_use]
pub fn gstreamer_pipeline(stream: &VideoStreamInformation) -> String {
    let kind = VideoStreamType(u32::from(stream.r#type));
    let uri = c_string(&stream.uri);
    // "my personal hack to allow for custom pipelines for testing", the C# says.
    if let Some(pipeline) = uri.strip_prefix("gst://") {
        return pipeline.to_owned();
    }
    let mut port = 0;
    if kind == VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP
        || kind == VideoStreamType::VIDEO_STREAM_TYPE_MPEG_TS
    {
        // `int.TryParse(uri)`, else the digits after a colon.
        port = uri
            .trim()
            .parse::<i64>()
            .ok()
            .or_else(|| port_after_colon(&uri))
            .unwrap_or(0);
        if !(1..=65535).contains(&port) {
            return String::new();
        }
    }
    match kind {
        VideoStreamType::VIDEO_STREAM_TYPE_RTSP => {
            // `Regex.Replace(uri, "^.*://", "")`: greedy, up to the last "://".
            let bare = uri
                .rfind("://")
                .map_or(uri.as_str(), |at| uri.get(at + 3..).unwrap_or_default());
            format!(
                "rtspsrc location=rtsp://{bare} latency=41 udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! decodebin3 ! queue leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false"
            )
        }
        VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP => {
            // Unknown encodings are taken as H264.
            let encoding = if VideoStreamEncoding(u32::from(stream.encoding))
                == VideoStreamEncoding::VIDEO_STREAM_ENCODING_H265
            {
                "H265"
            } else {
                "H264"
            };
            format!(
                "udpsrc port={port} buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string){encoding} ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false"
            )
        }
        VideoStreamType::VIDEO_STREAM_TYPE_TCP_MPEG => {
            // `^(?:.*://)?([^:/]+):(\d+)`, the scheme's `.*` greedy.
            let bare = uri
                .rfind("://")
                .map_or(uri.as_str(), |at| uri.get(at + 3..).unwrap_or_default());
            let host: String = bare
                .chars()
                .take_while(|c| *c != ':' && *c != '/')
                .collect();
            let after = bare.get(host.len()..).unwrap_or_default();
            let digits: String = after
                .strip_prefix(':')
                .unwrap_or_default()
                .chars()
                .take_while(char::is_ascii_digit)
                .collect();
            if host.is_empty() || digits.is_empty() {
                return String::new();
            }
            format!(
                "tcpclientsrc host={host} port={digits} ! decodebin ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false"
            )
        }
        VideoStreamType::VIDEO_STREAM_TYPE_MPEG_TS => format!(
            "udpsrc port={port} buffer-size=90000 ! tsparse ! tsdemux ! decodebin ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false"
        ),
        _ => String::new(),
    }
}

/// Where `RequestCameraInformationAsync` is: the `REQUEST_MESSAGE` it waits on, if any.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InfoRequest {
    /// Not asking.
    #[default]
    Idle,
    /// Waiting for the answer to `REQUEST_MESSAGE(CAMERA_INFORMATION)`.
    Waiting(crate::RequestId),
}

/// One `CameraProtocol`.
#[derive(Debug, Clone, PartialEq)]
pub struct Camera {
    /// `parent`: the vehicle and component it belongs to.
    pub vehicle: VehicleId,
    /// Whether `StartID` has run (`parent` set, `ParseMessages` subscribed).
    started: bool,
    /// `have_camera_information`.
    pub have_camera_information: bool,
    /// `CameraInformation`; `None` is the C#'s zeroed struct.
    pub information: Option<CameraInformation>,
    /// `CameraSettings`.
    pub settings: Option<CameraSettings>,
    /// `CameraCaptureStatus`.
    pub capture_status: Option<CameraCaptureStatus>,
    /// `CameraFOVStatus`.
    pub fov_status: Option<CameraFovStatus>,
    /// `CameraTrackingImageStatus`.
    pub tracking_image_status: Option<CameraTrackingImageStatus>,
    /// `UseFOVStatus`.
    pub use_fov_status: bool,
    /// `_hfov`, degrees.
    pub hfov_set: f32,
    /// `_vfov`, degrees.
    pub vfov_set: f32,
    /// `_image_sequence`: the next `IMAGE_START_CAPTURE`'s sequence number.
    image_sequence: i32,
    /// `RequestCameraInformationAsync` under way.
    pub info_request: InfoRequest,
}

impl Camera {
    /// `new CameraProtocol()` for a component, not yet started.
    #[must_use]
    pub const fn new(vehicle: VehicleId) -> Self {
        Self {
            vehicle,
            started: false,
            have_camera_information: false,
            information: None,
            settings: None,
            capture_status: None,
            fov_status: None,
            tracking_image_status: None,
            use_fov_status: true,
            hfov_set: f32::NAN,
            vfov_set: f32::NAN,
            image_sequence: 1,
            info_request: InfoRequest::Idle,
        }
    }

    /// Whether `StartID` has run.
    #[must_use]
    pub const fn is_started(&self) -> bool {
        self.started
    }

    /// `StartID`: subscribed to its component's packets. The caller then asks for its
    /// information ([`Camera::information_request`]). `// C#: CameraProtocol.cs:212-219`
    pub const fn start(&mut self) {
        self.started = true;
    }

    /// `ParseMessages`: its own component's camera messages, once started; a video stream goes
    /// into `streams` under this camera's system and component and the stream's id.
    /// `// C#: CameraProtocol.cs:270-298`
    pub fn observe(
        &mut self,
        from: VehicleId,
        message: &MavMessage,
        streams: &mut std::collections::BTreeMap<(u32, u8, u8), VideoStreamInformation>,
    ) {
        if !self.started || from != self.vehicle {
            return;
        }
        match message {
            MavMessage::CameraInformation(m) => {
                self.have_camera_information = true;
                self.information = Some(*m);
            }
            MavMessage::CameraSettings(m) => self.settings = Some(*m),
            MavMessage::CameraCaptureStatus(m) => self.capture_status = Some(*m),
            MavMessage::VideoStreamInformation(m) => {
                streams.insert((self.vehicle.sysid, self.vehicle.compid, m.stream_id), *m);
            }
            MavMessage::CameraFovStatus(m) => self.fov_status = Some(*m),
            MavMessage::CameraTrackingImageStatus(m) => self.tracking_image_status = Some(*m),
            _ => {}
        }
    }

    /// `CameraInformation.flags`, zero before any.
    fn flags(&self) -> u32 {
        self.information.map_or(0, |i| i.flags)
    }

    /// `CameraSettings.mode_id`, zero before any.
    fn mode(&self) -> u32 {
        self.settings.map_or(0, |s| u32::from(s.mode_id))
    }

    /// `HasModes`. `// C#: CameraProtocol.cs:98`
    #[must_use]
    pub fn has_modes(&self) -> bool {
        self.flags() & CameraCapFlags::CAMERA_CAP_FLAGS_HAS_MODES.0 > 0
    }

    /// `HasZoom`. `// C#: CameraProtocol.cs:103`
    #[must_use]
    pub fn has_zoom(&self) -> bool {
        self.flags() & CameraCapFlags::CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM.0 > 0
    }

    /// `HasFocus`. `// C#: CameraProtocol.cs:108`
    #[must_use]
    pub fn has_focus(&self) -> bool {
        self.flags() & CameraCapFlags::CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS.0 > 0
    }

    /// `CanCaptureVideo`: capable, and in video mode or able to record in image mode.
    /// `// C#: CameraProtocol.cs:113-136`
    #[must_use]
    pub fn can_capture_video(&self) -> bool {
        let flags = self.flags();
        if flags & CameraCapFlags::CAMERA_CAP_FLAGS_CAPTURE_VIDEO.0 == 0 {
            return false;
        }
        let mode = self.mode();
        if !self.has_modes() || mode == CameraMode::CAMERA_MODE_VIDEO.0 {
            return true;
        }
        if mode == CameraMode::CAMERA_MODE_IMAGE.0 || mode == CameraMode::CAMERA_MODE_IMAGE_SURVEY.0
        {
            return flags & CameraCapFlags::CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE.0 > 0;
        }
        false
    }

    /// `CanCaptureImage`: capable, and in an image mode or able to take one in video mode.
    /// `// C#: CameraProtocol.cs:141-167`
    #[must_use]
    pub fn can_capture_image(&self) -> bool {
        let flags = self.flags();
        if flags & CameraCapFlags::CAMERA_CAP_FLAGS_CAPTURE_IMAGE.0 == 0 {
            return false;
        }
        let mode = self.mode();
        if !self.has_modes()
            || mode == CameraMode::CAMERA_MODE_IMAGE.0
            || mode == CameraMode::CAMERA_MODE_IMAGE_SURVEY.0
        {
            return true;
        }
        if mode == CameraMode::CAMERA_MODE_VIDEO.0 {
            return flags & CameraCapFlags::CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE.0 > 0;
        }
        false
    }

    /// `HFOV`, degrees: the one set, or with `UseFOVStatus` the camera's - always the camera's,
    /// zero before its first `CAMERA_FOV_STATUS`, because the C#'s `hfov == float.NaN` is never
    /// true. `// C#: CameraProtocol.cs:175-189`
    #[must_use]
    pub fn hfov(&self) -> f32 {
        if !self.use_fov_status {
            return self.hfov_set;
        }
        self.fov_status.map_or(0.0, |s| s.hfov)
    }

    /// `VFOV`, degrees, as [`Camera::hfov`]. `// C#: CameraProtocol.cs:192-206`
    #[must_use]
    pub fn vfov(&self) -> f32 {
        if !self.use_fov_status {
            return self.vfov_set;
        }
        self.fov_status.map_or(0.0, |s| s.vfov)
    }

    /// `RequestCameraInformationAsync`'s first request, `REQUEST_MESSAGE(CAMERA_INFORMATION)`,
    /// which is waited on; `None` before `StartID` (`parent?.parent == null`).
    /// `// C#: CameraProtocol.cs:224-236`
    #[must_use]
    pub fn information_request(&self) -> Option<(u16, [f32; 7])> {
        self.started.then(|| {
            (
                cmd(MavCmd::MAV_CMD_REQUEST_MESSAGE),
                [CAMERA_INFORMATION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        })
    }

    /// The rest of `RequestCameraInformationAsync` once the first request has its answer: the
    /// deprecated `REQUEST_CAMERA_INFORMATION` where it was not accepted, then
    /// `REQUEST_MESSAGE(VIDEO_STREAM_INFORMATION)`, each sent once and not waited on.
    /// Returned as they are made, not gathered first: the link thread allocates nothing for them
    /// (crates/mp-link/tests/no_alloc_ingest.rs, where the gathering showed as a fourth allocation
    /// on the COMMAND_ACK that answered the first request, 2026-10-06).
    /// `// C#: CameraProtocol.cs:237-255`
    pub fn information_follow_up(&self, accepted: bool) -> impl Iterator<Item = MavMessage> {
        let vehicle = self.vehicle;
        (!accepted)
            .then(|| {
                commands::command_long(
                    vehicle,
                    cmd(MavCmd::MAV_CMD_REQUEST_CAMERA_INFORMATION),
                    [0.0; 7],
                )
            })
            .into_iter()
            .chain(std::iter::once(commands::command_long(
                vehicle,
                cmd(MavCmd::MAV_CMD_REQUEST_MESSAGE),
                [VIDEO_STREAM_INFORMATION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )))
    }

    /// `SET_MESSAGE_INTERVAL` for one message, not waited on.
    fn interval(&self, message: f32, interval_us: f32) -> MavMessage {
        commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL),
            [message, interval_us, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// `RequestMessageIntervals(ratehz)`'s intervals, each sent once and not waited on:
    /// `CAMERA_FOV_STATUS` always, `CAMERA_SETTINGS` for a camera with modes, zoom or focus,
    /// `CAMERA_CAPTURE_STATUS` for one that can capture. The caller also starts
    /// `RequestCameraInformationAsync`. Nothing below 0 Hz, nor before `StartID`; 0 Hz stops
    /// them (an interval of -1). `// C#: CameraProtocol.cs:305-369`
    #[must_use]
    pub fn message_intervals(&self, ratehz: i32) -> Vec<MavMessage> {
        if ratehz < 0 || !self.started {
            return Vec::new();
        }
        #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
        let interval_us = if ratehz > 0 {
            (1e6 / f64::from(ratehz)) as f32
        } else {
            -1.0
        };
        let mut out = vec![self.interval(CAMERA_FOV_STATUS, interval_us)];
        if self.has_modes() || self.has_zoom() || self.has_focus() {
            out.push(self.interval(CAMERA_SETTINGS, interval_us));
        }
        let flags = self.flags();
        if flags
            & (CameraCapFlags::CAMERA_CAP_FLAGS_CAPTURE_VIDEO.0
                | CameraCapFlags::CAMERA_CAP_FLAGS_CAPTURE_IMAGE.0)
            > 0
        {
            out.push(self.interval(CAMERA_CAPTURE_STATUS, interval_us));
        }
        out
    }

    /// `RequestTrackingMessageInterval(ratehz)`: `CAMERA_TRACKING_IMAGE_STATUS`'s interval, not
    /// waited on; nothing before `StartID`. `// C#: CameraProtocol.cs:371-391`
    #[must_use]
    pub fn tracking_message_interval(&self, ratehz: i32) -> Option<MavMessage> {
        #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
        let interval_us = (1e6 / f64::from(ratehz)) as f32;
        self.started
            .then(|| self.interval(CAMERA_TRACKING_IMAGE_STATUS, interval_us))
    }

    /// `TakeSinglePictureAsync(camera)`: `IMAGE_START_CAPTURE` for one image, with the next
    /// sequence number so a retry cannot take two. `// C#: CameraProtocol.cs:397-409`
    #[allow(clippy::cast_precision_loss)] // small numbers
    pub fn take_single_picture(&mut self, camera: i32) -> MavMessage {
        let sequence = self.image_sequence;
        self.image_sequence += 1;
        commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_IMAGE_START_CAPTURE),
            [camera as f32, 0.0, 1.0, sequence as f32, 0.0, 0.0, 0.0],
        )
    }

    /// `StartRecordingAsync(stream_id)`: `VIDEO_START_CAPTURE`.
    /// `// C#: CameraProtocol.cs:450-461`
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // small numbers
    pub fn start_recording(&self, stream_id: i32) -> MavMessage {
        commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_VIDEO_START_CAPTURE),
            [
                stream_id as f32,
                f32::NAN,
                f32::NAN,
                f32::NAN,
                0.0,
                0.0,
                f32::NAN,
            ],
        )
    }

    /// `StopRecordingAsync(stream_id)`: `VIDEO_STOP_CAPTURE`.
    /// `// C#: CameraProtocol.cs:467-477`
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // small numbers
    pub fn stop_recording(&self, stream_id: i32) -> MavMessage {
        commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_VIDEO_STOP_CAPTURE),
            [
                stream_id as f32,
                f32::NAN,
                f32::NAN,
                f32::NAN,
                0.0,
                0.0,
                f32::NAN,
            ],
        )
    }

    /// `SetZoomAsync(zoom_level, zoom_type)`: `SET_CAMERA_ZOOM`.
    /// `// C#: CameraProtocol.cs:484-493`
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // small numbers
    pub fn set_zoom(&self, zoom_level: f32, zoom_type: CameraZoomType) -> MavMessage {
        commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_SET_CAMERA_ZOOM),
            [zoom_type.0 as f32, zoom_level, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// `SetTrackingPointAsync(x, y)`: `CAMERA_TRACK_POINT` at the point taken from -1..1 to
    /// 0..1, if the camera can track a point. `// C#: CameraProtocol.cs:501-517`
    #[must_use]
    pub fn set_tracking_point(&self, x: f32, y: f32) -> Option<MavMessage> {
        if self.flags() & CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_POINT.0 == 0 {
            return None;
        }
        Some(commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_CAMERA_TRACK_POINT),
            [(x + 1.0) / 2.0, (y + 1.0) / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ))
    }

    /// `SetTrackingRectangleAsync(x1, y1, x2, y2)`: `CAMERA_TRACK_RECTANGLE` with the corners
    /// taken to 0..1 and ordered, if the camera can track a rectangle.
    /// `// C#: CameraProtocol.cs:528-558`
    #[must_use]
    pub fn set_tracking_rectangle(&self, x1: f32, y1: f32, x2: f32, y2: f32) -> Option<MavMessage> {
        if self.flags() & CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE.0 == 0 {
            return None;
        }
        let (mut x1, mut y1, mut x2, mut y2) = (
            (x1 + 1.0) / 2.0,
            (y1 + 1.0) / 2.0,
            (x2 + 1.0) / 2.0,
            (y2 + 1.0) / 2.0,
        );
        if x1 > x2 {
            std::mem::swap(&mut x1, &mut x2);
        }
        if y1 > y2 {
            std::mem::swap(&mut y1, &mut y2);
        }
        Some(commands::command_long(
            self.vehicle,
            cmd(MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE),
            [x1, y1, x2, y2, 0.0, 0.0, 0.0],
        ))
    }

    /// `CalculateImagePointVectorCameraFrame(x, y)`: straight ahead, turned by the fields of view
    /// only when both `x` and `y` are non-zero - the C#'s `x != 0 && y != 0`, and its
    /// `HFOV != float.NaN`, which is always true. `// C#: CameraProtocol.cs:604-618`
    #[must_use]
    #[allow(clippy::float_cmp)] // the C#'s `!= 0`
    pub fn image_point_vector_camera_frame(&self, x: f64, y: f64) -> Vector3 {
        let mut vector = Vector3::new(1.0, 0.0, 0.0);
        if x != 0.0 && y != 0.0 {
            let hfov = f64::from(self.hfov()).to_radians();
            let vfov = f64::from(self.vfov()).to_radians();
            vector.y = (x * hfov / 2.0).tan();
            vector.z = (y * vfov / 2.0).tan();
            vector = vector.normalized();
        }
        vector
    }

    /// `CalculateImagePointRotation(x, y)`: the rotation taking the camera's centre to the point.
    /// `// C#: CameraProtocol.cs:643-662`
    #[must_use]
    #[allow(clippy::float_cmp)] // the C#'s `== 0`
    pub fn image_point_rotation(&self, x: f64, y: f64) -> Quaternion {
        let v1 = self.image_point_vector_camera_frame(0.0, 0.0);
        let v2 = self.image_point_vector_camera_frame(x, y);
        if v1 == -v2 {
            return Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), std::f64::consts::PI);
        }
        let axis = v1.cross(v2);
        if axis.length() == 0.0 {
            return Quaternion::default();
        }
        Quaternion::from_axis_angle(axis.normalized(), v1.dot(v2).acos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stream(kind: VideoStreamType, uri: &str, encoding: u8) -> VideoStreamInformation {
        let mut bytes = [0u8; 160];
        bytes[..uri.len()].copy_from_slice(uri.as_bytes());
        let mut name = [0u8; 32];
        name[..4].copy_from_slice(b"main");
        #[allow(clippy::cast_possible_truncation)]
        VideoStreamInformation {
            framerate: 30.0,
            bitrate: 0,
            flags: 0,
            resolution_h: 1280,
            resolution_v: 720,
            rotation: 0,
            hfov: 60,
            stream_id: 1,
            count: 1,
            r#type: kind.0 as u8,
            name,
            uri: bytes,
            encoding,
            camera_device_id: 0,
        }
    }

    #[test]
    fn each_stream_type_has_the_csharps_pipeline() {
        assert_eq!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_RTSP,
                "gst://videotestsrc ! appsink name=outsink",
                0
            )),
            "videotestsrc ! appsink name=outsink"
        );
        assert!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_RTSP,
                "http://192.168.1.10:8554/live",
                0
            ))
            .starts_with("rtspsrc location=rtsp://192.168.1.10:8554/live latency=41")
        );
        let udp = gstreamer_pipeline(&stream(
            VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP,
            "udp://127.0.0.1:5600",
            2,
        ));
        assert!(
            udp.starts_with("udpsrc port=5600 buffer-size=90000"),
            "{udp}"
        );
        assert!(udp.contains("encoding-name=(string)H265"));
        assert!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP,
                "5600",
                0
            ))
            .contains("encoding-name=(string)H264")
        );
        assert_eq!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP,
                "99999",
                0
            )),
            ""
        );
        assert!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_TCP_MPEG,
                "tcp://10.0.0.2:5000",
                0
            ))
            .starts_with("tcpclientsrc host=10.0.0.2 port=5000 ! decodebin")
        );
        assert_eq!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_TCP_MPEG,
                "nowhere",
                0
            )),
            ""
        );
        assert!(
            gstreamer_pipeline(&stream(
                VideoStreamType::VIDEO_STREAM_TYPE_MPEG_TS,
                "5601",
                0
            ))
            .starts_with("udpsrc port=5601 buffer-size=90000 ! tsparse ! tsdemux")
        );
        assert_eq!(
            gstreamer_pipeline(&stream(VideoStreamType(9), "5601", 0)),
            ""
        );
    }

    fn information(flags: u32) -> MavMessage {
        MavMessage::CameraInformation(CameraInformation {
            time_boot_ms: 0,
            firmware_version: 0,
            focal_length: f32::NAN,
            sensor_size_h: f32::NAN,
            sensor_size_v: f32::NAN,
            flags,
            resolution_h: 0,
            resolution_v: 0,
            cam_definition_version: 0,
            vendor_name: [0; 32],
            model_name: [0; 32],
            lens_id: 0,
            cam_definition_uri: [0; 140],
            gimbal_device_id: 0,
            camera_device_id: 0,
        })
    }

    #[test]
    fn only_its_own_components_messages_count_and_only_once_started() {
        let me = VehicleId::new(1, 1);
        let mut streams = std::collections::BTreeMap::new();
        let mut camera = Camera::new(me);
        camera.observe(me, &information(2), &mut streams);
        assert!(!camera.have_camera_information);
        assert!(camera.information_request().is_none());
        assert!(camera.message_intervals(4).is_empty());
        camera.start();
        camera.observe(VehicleId::new(1, 100), &information(2), &mut streams);
        assert!(!camera.have_camera_information);
        camera.observe(me, &information(2), &mut streams);
        assert!(camera.can_capture_image());
        assert!(!camera.can_capture_video());
        camera.observe(
            me,
            &MavMessage::VideoStreamInformation(stream(
                VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP,
                "5600",
                0,
            )),
            &mut streams,
        );
        assert!(streams.contains_key(&(1, 1, 1)));
        let (command, params) = camera.information_request().expect("started");
        assert_eq!(command, 512);
        assert!((params[0] - 259.0).abs() < f32::EPSILON);
        // Refused: the old request, then the streams'.
        let follow: Vec<MavMessage> = camera.information_follow_up(false).collect();
        assert_eq!(follow.len(), 2);
        assert_eq!(camera.information_follow_up(true).count(), 1);
        // FOV always; capture status for a camera that captures; no settings without modes.
        let intervals = camera.message_intervals(4);
        assert_eq!(intervals.len(), 2);
        let MavMessage::CommandLong(fov) = &intervals[0] else {
            panic!()
        };
        assert_eq!(fov.command, 511);
        assert!((fov.param1 - 271.0).abs() < f32::EPSILON);
        assert!((fov.param2 - 250_000.0).abs() < 1.0);
        assert!(camera.message_intervals(-1).is_empty());
    }

    #[test]
    fn capture_and_tracking_commands_are_the_csharps() {
        let me = VehicleId::new(1, 1);
        let mut camera = Camera::new(me);
        let MavMessage::CommandLong(first) = camera.take_single_picture(0) else {
            panic!()
        };
        let MavMessage::CommandLong(second) = camera.take_single_picture(0) else {
            panic!()
        };
        assert_eq!(first.command, 2000);
        assert!((first.param3 - 1.0).abs() < f32::EPSILON);
        assert!((first.param4 - 1.0).abs() < f32::EPSILON);
        assert!(
            (second.param4 - 2.0).abs() < f32::EPSILON,
            "the sequence counts"
        );
        let MavMessage::CommandLong(start) = camera.start_recording(0) else {
            panic!()
        };
        assert_eq!(start.command, 2500);
        assert!(start.param2.is_nan());
        let MavMessage::CommandLong(zoom) =
            camera.set_zoom(-1.0, CameraZoomType::ZOOM_TYPE_CONTINUOUS)
        else {
            panic!()
        };
        assert_eq!(zoom.command, 531);
        assert!((zoom.param1 - 1.0).abs() < f32::EPSILON);
        assert!((zoom.param2 + 1.0).abs() < f32::EPSILON);
        // No tracking without the capability.
        assert!(camera.set_tracking_point(0.0, 0.0).is_none());
        camera.start();
        let mut streams = std::collections::BTreeMap::new();
        camera.observe(
            me,
            &information(
                CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_POINT.0
                    | CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE.0,
            ),
            &mut streams,
        );
        let Some(MavMessage::CommandLong(point)) = camera.set_tracking_point(0.0, -1.0) else {
            panic!()
        };
        assert!((point.param1 - 0.5).abs() < f32::EPSILON && point.param2.abs() < f32::EPSILON);
        let Some(MavMessage::CommandLong(rect)) =
            camera.set_tracking_rectangle(1.0, 1.0, -1.0, 0.0)
        else {
            panic!()
        };
        assert_eq!(rect.command, 2005);
        assert_eq!(
            [rect.param1, rect.param2, rect.param3, rect.param4],
            [0.0, 0.5, 1.0, 1.0]
        );
    }

    #[test]
    fn the_field_of_view_is_the_cameras_even_when_unknown() {
        let mut camera = Camera::new(VehicleId::new(1, 1));
        camera.hfov_set = 40.0;
        // `UseFOVStatus` and no status: zero, not the 40 set, as the C#'s NaN test never holds.
        assert!(camera.hfov().abs() < f32::EPSILON);
        camera.use_fov_status = false;
        assert!((camera.hfov() - 40.0).abs() < f32::EPSILON);
        camera.vfov_set = 30.0;
        // The centre needs no turn; a corner turns by half of each field of view.
        assert_eq!(camera.image_point_rotation(0.0, 0.0), Quaternion::default());
        // Only one of x and y non-zero: straight ahead, as the C#'s `&&` has it.
        assert_eq!(camera.image_point_rotation(1.0, 0.0), Quaternion::default());
        let q = camera.image_point_rotation(1.0, 1.0);
        let turned = q.body_to_earth(Vector3::new(1.0, 0.0, 0.0));
        let want = camera.image_point_vector_camera_frame(1.0, 1.0);
        assert!((turned.x - want.x).abs() < 1e-9 && (turned.y - want.y).abs() < 1e-9);
    }

    #[test]
    fn components_are_the_csharps() {
        assert!(is_camera_component(1) && is_camera_component(100) && is_camera_component(105));
        assert!(!is_camera_component(106) && !is_camera_component(154));
        assert!(is_gimbal_manager_component(1) && is_gimbal_manager_component(191));
        assert!(!is_gimbal_manager_component(195) && !is_gimbal_manager_component(154));
        assert_eq!(c_string(b"main\0junk"), "main");
    }
}
