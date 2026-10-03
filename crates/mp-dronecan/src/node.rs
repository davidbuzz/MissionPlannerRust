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

//! The `DroneCAN` class: this ground station's node on the bus.
//!
//! What it does once started, each as the C# does it:
//!
//! * every second, a `NodeStatus` from node 127 (healthy, operational, its uptime), and on a
//!   wall-clock second that is a multiple of ten a `GetNodeInfo` request to the next node of the
//!   node list in turn (`StartSLCAN`'s third thread, `DroneCAN.cs:313-360`);
//! * the node list: a node's first `NodeStatus` raises [`Event::NodeAdded`], its first
//!   `GetNodeInfo` response [`Event::NodeInfoAdded`], and the newest of each is kept; a
//!   `GetNodeInfo` request to us is answered as `org.missionplanner` (`:363-400`);
//! * the file server: a `file.Read` request for a file being served is answered with up to 256
//!   bytes from its offset, with [`Event::FileSendProgress`] and, at the end,
//!   [`Event::FileSendComplete`] (`SetupFileServer`, `:612-660`);
//! * the dynamic node-id allocator: a node's three anonymous `Allocation` messages gathered into
//!   its unique id, and an id given - the one it had before, the one it asked for if free, or the
//!   highest free one from 125 down (`SetupDynamicNodeAllocator`, `:937-1048`).
//!
//! Every message this node sends is also handed to its own handlers as if received
//! (`PackageMessageSLCAN` calls `InvokeMessageReceived`, "used mainly for GCS loopback viewing"),
//! so node 127 appears in its own node list, names itself when asked, and the page lists it.
//!
//! **Not the C#'s:** the C# raises `MessageReceived` from inside the send, so a handler that
//! sends runs the handlers again before it returns; here what is sent or received waits in a
//! queue and is handed on in turn. The same handlers see the same messages, in an order that
//! differs only where one message's handlers send another. `NodeList.Keys`, which the
//! ten-second poll walks, is a `ConcurrentDictionary`'s bucket order in the C#; here it is
//! ascending. The `DateTime.Now` reads are the `Instant` and wall-clock second handed in.
//! `// C#: ExtLibs/DroneCAN/DroneCAN.cs:187-401, 604-660, 937-1048, 1292-1301, 1347-1421,
//! 1522-1556`

use std::collections::{BTreeMap, VecDeque};
use std::io::{Read, Seek, SeekFrom};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crate::dsdl::{
    Allocation, GetNodeInfoRes, HEALTH_OK, HardwareVersion, MODE_OPERATIONAL, Message, NodeStatus,
    SoftwareVersion, ascii,
};
use crate::frame::{Frame, TransferType};
use crate::slcan::{self, Line};
use crate::transfer::{Outcome, Reassembler, package};

/// `SourceNode`'s default: the node id the C# gives itself.
pub const SOURCE_NODE: u8 = 127;

/// The priority the C# sends its own messages at.
pub const PRIORITY: u8 = 30;

/// What `GetNodeInfo` answers with for this node: the planner's version, as
/// `FileVersionInfo` has the C#'s - `ProductMajorPart`, `ProductMinorPart`, and the build part's
/// digits read as hex for the commit (`uint.Parse(ProductBuildPart.ToString(), HexNumber)`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Identity {
    /// `software_version.major`.
    pub major: u8,
    /// `software_version.minor`.
    pub minor: u8,
    /// `software_version.vcs_commit`.
    pub vcs_commit: u32,
}

impl Identity {
    /// From a version string such as `0.1.0`: major, minor, and the build's digits as hex.
    #[must_use]
    pub fn from_version(version: &str) -> Self {
        let mut parts = version.split('.');
        let mut part = || parts.next().unwrap_or("0");
        let major = part().parse().unwrap_or(0);
        let minor = part().parse().unwrap_or(0);
        let build = part()
            .split(|c: char| !c.is_ascii_digit())
            .next()
            .unwrap_or("0");
        Self {
            major,
            minor,
            vcs_commit: u32::from_str_radix(build, 16).unwrap_or(0),
        }
    }
}

/// A message received, or sent and handed back: `MessageReceived(frame, msg, transferID)`.
#[derive(Debug, Clone, PartialEq)]
pub struct Received {
    /// The frame's identifier: who sent it, to whom, what type.
    pub frame: Frame,
    /// The message.
    pub message: Message,
    /// Its transfer id.
    pub transfer_id: u8,
}

/// The C#'s other events: what it raises besides `MessageReceived`.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// `NodeAdded(id, status)`: a node's first `NodeStatus`.
    NodeAdded(u8, NodeStatus),
    /// `NodeInfoAdded(id, info)`: a node's first `GetNodeInfo` response.
    NodeInfoAdded(u8, Box<GetNodeInfoRes>),
    /// `FileSendProgress(node, file, percent)`.
    FileSendProgress {
        /// The node reading.
        node: u8,
        /// The file's served name.
        file: String,
        /// How far through, 0 to 100.
        percent: f64,
    },
    /// `FileSendComplete(node, file)`.
    FileSendComplete {
        /// The node reading.
        node: u8,
        /// The file's served name.
        file: String,
    },
}

/// `new DroneCAN()` and what `StartSLCAN`, `SetupFileServer` and `SetupDynamicNodeAllocator`
/// add to it.
#[derive(Debug)]
pub struct Node {
    source_node: u8,
    /// `NodeStatus`: whether the 1 Hz status goes out.
    pub send_node_status: bool,
    transfer_id: u8,
    reassembler: Reassembler,
    node_list: BTreeMap<u8, NodeStatus>,
    node_info: BTreeMap<u8, GetNodeInfoRes>,
    /// `allocated`, in insertion order: `First` finds the earliest.
    allocated: Vec<(u8, Vec<u8>)>,
    dynamic_bytes: Vec<u8>,
    dynamic_allocator: bool,
    file_server: bool,
    /// `fileServerList`: served name to path, in insertion order.
    served: Vec<(String, PathBuf)>,
    /// `run`.
    running: bool,
    /// `uptime`: when the object was made.
    made: Instant,
    next_status: Option<Instant>,
    /// The ten-second poll's position in the node list.
    poll: usize,
    identity: Identity,
    /// The lines `WriteToStreamSLCAN` writes, each with its `\r`.
    outbox: VecDeque<String>,
    pending: VecDeque<Received>,
    pumping: bool,
    delivered: VecDeque<Received>,
    events: VecDeque<Event>,
    /// `FrameError`s raised: toggles out of turn, bad CRCs.
    pub frame_errors: u64,
}

impl Node {
    /// `new DroneCAN()`: node 127, its uptime counted from `now`.
    #[must_use]
    pub fn new(identity: Identity, now: Instant) -> Self {
        Self {
            source_node: SOURCE_NODE,
            send_node_status: true,
            transfer_id: 0,
            reassembler: Reassembler::new(),
            node_list: BTreeMap::new(),
            node_info: BTreeMap::new(),
            allocated: Vec::new(),
            dynamic_bytes: Vec::new(),
            dynamic_allocator: false,
            file_server: false,
            served: Vec::new(),
            running: false,
            made: now,
            next_status: None,
            poll: 0,
            identity,
            outbox: VecDeque::new(),
            pending: VecDeque::new(),
            pumping: false,
            delivered: VecDeque::new(),
            events: VecDeque::new(),
            frame_errors: 0,
        }
    }

    /// `SourceNode`.
    #[must_use]
    pub const fn source_node(&self) -> u8 {
        self.source_node
    }

    /// `SourceNode = id`.
    pub const fn set_source_node(&mut self, id: u8) {
        self.source_node = id;
    }

    /// `StartSLCAN`'s part once the port is set up: `run`, the 1 Hz sender from now, and the node
    /// list's handler.
    pub fn start(&mut self, now: Instant) {
        self.running = true;
        self.next_status = Some(now);
    }

    /// `SetupFileServer`.
    pub const fn setup_file_server(&mut self) {
        self.file_server = true;
    }

    /// `SetupDynamicNodeAllocator`: once.
    pub const fn setup_dynamic_node_allocator(&mut self) {
        self.dynamic_allocator = true;
    }

    /// `DynamicNodeAllocator`.
    #[must_use]
    pub const fn dynamic_node_allocator(&self) -> bool {
        self.dynamic_allocator
    }

    /// `Stop(closestream)`: `run` false and every `MessageReceived` handler removed - so nothing
    /// more is handed on - and, when the stream is to be closed, the adapter's close command,
    /// which the caller writes before closing it.
    pub fn stop(&mut self, close_stream: bool) -> Option<&'static [u8]> {
        self.running = false;
        self.file_server = false;
        self.dynamic_allocator = false;
        self.pending.clear();
        self.delivered.clear();
        close_stream.then_some(slcan::CLOSE)
    }

    /// `run`.
    #[must_use]
    pub const fn is_running(&self) -> bool {
        self.running
    }

    /// `TransferID++`: the next transfer id, the counter moved on.
    pub const fn next_transfer_id(&mut self) -> u8 {
        let id = self.transfer_id;
        self.transfer_id = self.transfer_id.wrapping_add(1);
        id
    }

    /// `NodeList`.
    #[must_use]
    pub const fn node_list(&self) -> &BTreeMap<u8, NodeStatus> {
        &self.node_list
    }

    /// `NodeInfo`.
    #[must_use]
    pub const fn node_info(&self) -> &BTreeMap<u8, GetNodeInfoRes> {
        &self.node_info
    }

    /// `GetNodeName(id)`: the name it gave, NULs trimmed; "Anonymous" for 0; else "?".
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1292-1301`
    #[must_use]
    pub fn node_name(&self, id: u8) -> String {
        if let Some(info) = self.node_info.get(&id) {
            return ascii(&info.name).trim_end_matches('\0').to_owned();
        }
        if id == 0 {
            return "Anonymous".to_owned();
        }
        "?".to_owned()
    }

    /// `ServeFile(path, name)`: `name` read from `path` when a node asks for it.
    pub fn serve_file(&mut self, path: PathBuf, name: &str) {
        if let Some(entry) = self.served.iter_mut().find(|(served, _)| served == name) {
            entry.1 = path;
        } else {
            self.served.push((name.to_owned(), path));
        }
    }

    /// The lines written since the last call, each ending `\r`.
    pub fn take_outgoing(&mut self) -> Vec<String> {
        self.outbox.drain(..).collect()
    }

    /// The next message for the page's handlers and the calls waiting on answers.
    pub fn next_received(&mut self) -> Option<Received> {
        self.delivered.pop_front()
    }

    /// The events raised since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        self.events.drain(..).collect()
    }

    /// `PackageMessageSLCAN(dest, priority, transferID, msg)` and `WriteToStreamSLCAN`: the frames'
    /// lines queued, and the message handed to this node's own handlers.
    pub fn send(&mut self, destination: u8, priority: u8, transfer_id: u8, message: Message) {
        if !self.running {
            return;
        }
        let frames = package(
            self.source_node,
            destination,
            priority,
            transfer_id,
            &message,
            false,
        );
        let Some(first) = frames.first().map(|(frame, _)| *frame) else {
            return;
        };
        for (frame, payload) in &frames {
            self.outbox
                .push_back(slcan::format_frame(frame, payload, false));
        }
        self.pending.push_back(Received {
            frame: first,
            message,
            transfer_id,
        });
        self.pump();
    }

    /// `ReadMessageSLCAN(line)`: a line from the bus.
    pub fn receive_line(&mut self, line: &str) {
        if let Line::Frame(frame, packet_id, payload) = slcan::read_line(line) {
            self.receive_frame(frame, packet_id, &payload);
        }
    }

    /// `ProcessFrame`: a frame from the bus, and the message it completes handed on.
    pub fn receive_frame(&mut self, frame: Frame, packet_id: u32, payload: &crate::Payload) {
        if !self.running {
            return;
        }
        match self.reassembler.push(frame, packet_id, payload) {
            Outcome::Complete(done) => {
                let (frame, message, transfer_id) = *done;
                self.pending.push_back(Received {
                    frame,
                    message,
                    transfer_id,
                });
                self.pump();
            }
            Outcome::Error => self.frame_errors += 1,
            Outcome::Pending | Outcome::Unknown => {}
        }
    }

    /// Once a frame: the 1 Hz sender. `wall_second` is `DateTime.Now.Second`.
    pub fn tick(&mut self, now: Instant, wall_second: u32) {
        if !self.running {
            return;
        }
        let Some(due) = self.next_status else {
            return;
        };
        if now < due {
            return;
        }
        self.next_status = Some(now + Duration::from_secs(1));
        if !self.send_node_status {
            return;
        }
        let uptime = now.saturating_duration_since(self.made).as_secs();
        let status = NodeStatus {
            uptime_sec: u32::try_from(uptime).unwrap_or(u32::MAX),
            health: HEALTH_OK,
            mode: MODE_OPERATIONAL,
            sub_mode: 0,
            vendor_specific_status_code: 0,
        };
        let id = self.next_transfer_id();
        self.send(self.source_node, PRIORITY, id, Message::NodeStatus(status));
        if wall_second.is_multiple_of(10) && !self.node_list.is_empty() {
            let count = self.node_list.len();
            let target = self.node_list.keys().nth(self.poll % count).copied();
            if let Some(target) = target {
                let id = self.next_transfer_id();
                self.send(target, PRIORITY, id, Message::GetNodeInfoReq);
                self.poll += 1;
            }
        }
    }

    /// Hands queued messages to the handlers in turn: each to the node list's, the file
    /// server's and the allocator's, then on to the page.
    fn pump(&mut self) {
        if self.pumping {
            return;
        }
        self.pumping = true;
        while let Some(received) = self.pending.pop_front() {
            if !self.running {
                continue;
            }
            self.node_list_handler(&received);
            if self.file_server {
                self.file_server_handler(&received);
            }
            let received = if self.dynamic_allocator {
                self.allocator_handler(received)
            } else {
                received
            };
            self.delivered.push_back(received);
        }
        self.pumping = false;
    }

    /// `StartSLCAN`'s "build nodelist" handler.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:363-400`
    fn node_list_handler(&mut self, received: &Received) {
        let frame = &received.frame;
        let source = frame.source_node();
        match &received.message {
            Message::NodeStatus(status) => {
                if !self.node_list.contains_key(&source) {
                    self.events.push_back(Event::NodeAdded(source, *status));
                }
                self.node_list.insert(source, *status);
            }
            Message::GetNodeInfoRes(info) if frame.is_service() => {
                if !self.node_info.contains_key(&source) {
                    self.events
                        .push_back(Event::NodeInfoAdded(source, info.clone()));
                }
                self.node_info.insert(source, (**info).clone());
            }
            Message::GetNodeInfoReq
                if frame.is_service() && frame.svc_destination_node() == self.source_node =>
            {
                let uptime = self.made.elapsed().as_secs();
                let mut unique_id = [0u8; 16];
                for (slot, byte) in unique_id.iter_mut().zip(b"MissionPlanner") {
                    *slot = *byte;
                }
                let response = GetNodeInfoRes {
                    status: NodeStatus {
                        uptime_sec: u32::try_from(uptime).unwrap_or(u32::MAX),
                        health: HEALTH_OK,
                        mode: MODE_OPERATIONAL,
                        sub_mode: 0,
                        vendor_specific_status_code: 0,
                    },
                    software_version: SoftwareVersion {
                        major: self.identity.major,
                        minor: self.identity.minor,
                        optional_field_flags: 0,
                        vcs_commit: self.identity.vcs_commit,
                        image_crc: 0,
                    },
                    hardware_version: HardwareVersion {
                        major: 0,
                        minor: 0,
                        unique_id,
                        certificate_of_authenticity: Vec::new(),
                    },
                    name: b"org.missionplanner".to_vec(),
                };
                self.send(
                    source,
                    frame.priority(),
                    received.transfer_id,
                    Message::GetNodeInfoRes(Box::new(response)),
                );
            }
            _ => {}
        }
    }

    /// `SetupFileServer`'s handler: a `file.Read` request to us for a file being served.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:612-660`
    fn file_server_handler(&mut self, received: &Received) {
        let frame = &received.frame;
        if !frame.is_service() || frame.svc_destination_node() != self.source_node {
            return;
        }
        let Message::FileReadReq { offset, path } = &received.message else {
            return;
        };
        let requested = ascii(path).trim_end_matches('\0').to_owned();
        let Some((_, file)) = self.served.iter().find(|(name, _)| *name == requested) else {
            // "File read request for file we are not serving".
            return;
        };
        // `File.OpenRead`, `Seek(offset)`, `Read(buffer, 0, 256)`; a file that will not open
        // throws out of the handler, which answers nothing.
        let Ok(mut handle) = std::fs::File::open(file) else {
            return;
        };
        let Ok(length) = handle.metadata().map(|meta| meta.len()) else {
            return;
        };
        if handle.seek(SeekFrom::Start(*offset)).is_err() {
            return;
        }
        let mut data = Vec::with_capacity(256);
        if handle.take(256).read_to_end(&mut data).is_err() {
            return;
        }
        let read = u64::try_from(data.len()).unwrap_or(0);
        self.send(
            frame.source_node(),
            frame.priority(),
            received.transfer_id,
            Message::FileReadRes {
                error: crate::dsdl::FILE_ERROR_OK,
                data,
            },
        );
        #[allow(clippy::cast_precision_loss)] // a file's length as a percentage
        let percent = (*offset + read) as f64 / length as f64 * 100.0;
        self.events.push_back(Event::FileSendProgress {
            node: frame.source_node(),
            file: requested.clone(),
            percent,
        });
        if length == *offset + read {
            self.events.push_back(Event::FileSendComplete {
                node: frame.source_node(),
                file: requested,
            });
        }
    }

    /// `allocated[id] = unique_id`, keeping an existing entry's place.
    fn allocate(&mut self, id: u8, unique_id: Vec<u8>) {
        if let Some(entry) = self.allocated.iter_mut().find(|(held, _)| *held == id) {
            entry.1 = unique_id;
        } else {
            self.allocated.push((id, unique_id));
        }
    }

    /// Whether an id is taken: in the node list or allocated.
    fn taken(&self, id: u8) -> bool {
        self.node_list.contains_key(&id) || self.allocated.iter().any(|(held, _)| *held == id)
    }

    /// `SetupDynamicNodeAllocator`'s handler. It changes the allocation request it answers (the
    /// first part's flag), which the handlers after it then see, as the C#'s object is shared.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:937-1048`
    fn allocator_handler(&mut self, mut received: Received) -> Received {
        let frame = received.frame;
        match &mut received.message {
            Message::GetNodeInfoRes(info) if frame.transfer_type() == TransferType::Service => {
                let unique_id = info.hardware_version.unique_id.to_vec();
                self.allocate(frame.source_node(), unique_id);
            }
            Message::Allocation(allocation) if frame.transfer_type() == TransferType::Anonymous => {
                let transfer_id = received.transfer_id;
                if allocation.first_part_of_unique_id {
                    allocation.first_part_of_unique_id = false;
                    self.dynamic_bytes.clear();
                    self.dynamic_bytes.extend_from_slice(&allocation.unique_id);
                    let answer = allocation.clone();
                    self.send(
                        self.source_node,
                        frame.priority(),
                        transfer_id,
                        Message::Allocation(answer),
                    );
                } else if allocation.unique_id.len() == 6 && self.dynamic_bytes.len() == 6 {
                    self.dynamic_bytes.extend_from_slice(&allocation.unique_id);
                    allocation.unique_id.clone_from(&self.dynamic_bytes);
                    let answer = allocation.clone();
                    self.send(
                        self.source_node,
                        PRIORITY,
                        transfer_id,
                        Message::Allocation(answer),
                    );
                } else if self.dynamic_bytes.len() == 12 {
                    self.dynamic_bytes.extend_from_slice(&allocation.unique_id);
                    allocation.unique_id.clone_from(&self.dynamic_bytes);
                    if allocation.unique_id.len() >= 16 {
                        self.choose_id(allocation);
                        self.dynamic_bytes.clear();
                    }
                    let answer = allocation.clone();
                    self.send(
                        self.source_node,
                        PRIORITY,
                        transfer_id,
                        Message::Allocation(answer),
                    );
                } else {
                    self.dynamic_bytes.clear();
                }
            }
            _ => {}
        }
        received
    }

    /// The id for a whole unique id: "Allocate again" for one seen before, the one asked for if
    /// free, else the highest free from 125 down.
    fn choose_id(&mut self, allocation: &mut Allocation) {
        if let Some((id, _)) = self
            .allocated
            .iter()
            .find(|(_, held)| *held == allocation.unique_id)
        {
            allocation.node_id = *id;
            return;
        }
        if allocation.node_id != 0 && !self.taken(allocation.node_id) {
            self.allocate(allocation.node_id, allocation.unique_id.clone());
            return;
        }
        if let Some(id) = (1..=125u8).rev().find(|id| !self.taken(*id)) {
            allocation.node_id = id;
            self.allocate(id, allocation.unique_id.clone());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsdl::{FILE_ERROR_OK, MODE_SOFTWARE_UPDATE};
    use crate::transfer::Reassembler;

    /// A node on the bus: what it sends, as lines the planner's node reads.
    fn lines_from(source: u8, destination: u8, transfer_id: u8, message: &Message) -> Vec<String> {
        package(source, destination, PRIORITY, transfer_id, message, false)
            .iter()
            .map(|(frame, payload)| slcan::format_frame(frame, payload, false))
            .collect()
    }

    /// What our node sent, read back as a bus would.
    fn decode_outgoing(node: &mut Node) -> Vec<(Frame, Message)> {
        let mut reassembler = Reassembler::new();
        let mut messages = Vec::new();
        for line in node.take_outgoing() {
            if let Line::Frame(frame, id, payload) = slcan::read_line(&line)
                && let Outcome::Complete(done) = reassembler.push(frame, id, &payload)
            {
                messages.push((done.0, done.1));
            }
        }
        messages
    }

    fn drain(node: &mut Node) -> Vec<Received> {
        std::iter::from_fn(|| node.next_received()).collect()
    }

    /// Started, the node says it is there at once and every second after, and hears itself:
    /// node 127 in its own list, asked for its info and answering as `org.missionplanner`.
    #[test]
    fn the_node_hears_itself() {
        let start = Instant::now();
        let mut node = Node::new(Identity::from_version("1.3.7"), start);
        node.start(start);
        node.tick(start, 3);
        let sent = decode_outgoing(&mut node);
        assert_eq!(sent.len(), 1);
        assert!(matches!(sent[0].1, Message::NodeStatus(status) if status.health == HEALTH_OK));
        assert_eq!(sent[0].0.source_node(), 127);
        assert_eq!(
            node.take_events(),
            [Event::NodeAdded(127, NodeStatus::default())]
        );
        let delivered = drain(&mut node);
        assert_eq!(delivered.len(), 1);

        // Not again within the second; again after it, and on a tenth second the poll asks
        // node 127 - us - for its info, which we answer.
        node.tick(start + Duration::from_millis(500), 3);
        assert!(node.take_outgoing().is_empty());
        node.tick(start + Duration::from_secs(1), 10);
        let sent = decode_outgoing(&mut node);
        assert_eq!(sent.len(), 3, "{sent:?}");
        assert!(matches!(sent[1].1, Message::GetNodeInfoReq));
        let Message::GetNodeInfoRes(info) = &sent[2].1 else {
            panic!("an answer");
        };
        assert_eq!(info.name, b"org.missionplanner");
        assert_eq!(info.software_version.major, 1);
        assert_eq!(info.software_version.minor, 3);
        assert_eq!(info.software_version.vcs_commit, 7);
        assert_eq!(&info.hardware_version.unique_id[..14], b"MissionPlanner");
        assert_eq!(node.node_name(127), "org.missionplanner");
        assert!(matches!(
            node.take_events().as_slice(),
            [Event::NodeInfoAdded(127, _)]
        ));
        assert_eq!(node.node_name(0), "Anonymous");
        assert_eq!(node.node_name(5), "?");
    }

    /// Another node's status and answer reach the list and the page, and its request for our
    /// info is answered with its own transfer id.
    #[test]
    fn another_node_is_listed_and_answered() {
        let now = Instant::now();
        let mut node = Node::new(Identity::default(), now);
        node.start(now);
        let status = NodeStatus {
            uptime_sec: 42,
            mode: MODE_SOFTWARE_UPDATE,
            ..NodeStatus::default()
        };
        for line in lines_from(10, 0, 5, &Message::NodeStatus(status)) {
            node.receive_line(&line);
        }
        assert_eq!(node.take_events(), [Event::NodeAdded(10, status)]);
        assert_eq!(node.node_list().get(&10), Some(&status));
        let delivered = drain(&mut node);
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].frame.source_node(), 10);

        for line in lines_from(10, 127, 9, &Message::GetNodeInfoReq) {
            node.receive_line(&line);
        }
        let sent = decode_outgoing(&mut node);
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].0.svc_destination_node(), 10);
        // A request to another node is not ours to answer.
        for line in lines_from(10, 50, 9, &Message::GetNodeInfoReq) {
            node.receive_line(&line);
        }
        assert!(node.take_outgoing().is_empty());
    }

    /// The file server: a read for a served file answered from its offset, progress and the
    /// end raised; a file not served is not answered.
    #[test]
    fn the_file_server_answers_reads() {
        let dir = std::env::temp_dir().join(format!("mp-dronecan-fs-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory");
        let path = dir.join("image.bin");
        let image: Vec<u8> = (0..300u32)
            .map(|i| u8::try_from(i % 251).unwrap())
            .collect();
        std::fs::write(&path, &image).expect("written");

        let now = Instant::now();
        let mut node = Node::new(Identity::default(), now);
        node.start(now);
        node.setup_file_server();
        node.serve_file(path.clone(), "fw.bin");
        let read = |offset: u64| Message::FileReadReq {
            offset,
            path: b"fw.bin".to_vec(),
        };
        for line in lines_from(20, 127, 1, &read(0)) {
            node.receive_line(&line);
        }
        let sent = decode_outgoing(&mut node);
        assert_eq!(
            sent[0].1,
            Message::FileReadRes {
                error: FILE_ERROR_OK,
                data: image[..256].to_vec()
            }
        );
        for line in lines_from(20, 127, 2, &read(256)) {
            node.receive_line(&line);
        }
        let sent = decode_outgoing(&mut node);
        assert_eq!(
            sent[0].1,
            Message::FileReadRes {
                error: FILE_ERROR_OK,
                data: image[256..].to_vec()
            }
        );
        let events = node.take_events();
        assert!(
            matches!(&events[0], Event::FileSendProgress { node: 20, percent, .. }
            if (*percent - 256.0 / 300.0 * 100.0).abs() < 1e-9)
        );
        assert_eq!(
            events.last(),
            Some(&Event::FileSendComplete {
                node: 20,
                file: "fw.bin".to_owned()
            })
        );

        // A file not served, and a read addressed to another node: nothing.
        let other = Message::FileReadReq {
            offset: 0,
            path: b"other.bin".to_vec(),
        };
        for line in lines_from(20, 127, 3, &other) {
            node.receive_line(&line);
        }
        for line in lines_from(20, 99, 4, &read(0)) {
            node.receive_line(&line);
        }
        assert!(node.take_outgoing().is_empty());
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The allocator: a node's three anonymous messages - six bytes of its unique id, six more,
    /// the last four - each echoed with what has been gathered, the last with the highest free id;
    /// the same unique id asking again gets the same id.
    #[test]
    fn the_allocator_gives_an_id() {
        let now = Instant::now();
        let mut node = Node::new(Identity::default(), now);
        node.start(now);
        node.setup_dynamic_node_allocator();
        let unique_id: Vec<u8> = (1..=16).collect();
        let ask = |node: &mut Node, first: bool, part: &[u8]| {
            let allocation = Message::Allocation(Allocation {
                node_id: 0,
                first_part_of_unique_id: first,
                unique_id: part.to_vec(),
            });
            for line in lines_from(0, 0, 1, &allocation) {
                node.receive_line(&line);
            }
            let sent = decode_outgoing(node);
            assert_eq!(sent.len(), 1);
            assert_eq!(sent[0].0.source_node(), 127);
            let Message::Allocation(answer) = sent[0].1.clone() else {
                panic!("an allocation");
            };
            answer
        };
        for round in 0..2 {
            let first = ask(&mut node, true, &unique_id[..6]);
            assert!(!first.first_part_of_unique_id);
            assert_eq!(first.unique_id, &unique_id[..6]);
            let second = ask(&mut node, false, &unique_id[6..12]);
            assert_eq!(second.unique_id, &unique_id[..12]);
            let last = ask(&mut node, false, &unique_id[12..]);
            assert_eq!(last.unique_id, unique_id);
            assert_eq!(last.node_id, 125, "round {round}");
        }
        // Out of order: the gathered bytes are dropped and nothing is said.
        let stray = Message::Allocation(Allocation {
            node_id: 0,
            first_part_of_unique_id: false,
            unique_id: vec![1, 2, 3],
        });
        for line in lines_from(0, 0, 2, &stray) {
            node.receive_line(&line);
        }
        assert!(node.take_outgoing().is_empty());
        // A node heard with its own unique id keeps its id; the next new one gets 124.
        let mut other: Vec<u8> = vec![0xaa; 16];
        other[0] = 0xbb;
        let _ = ask(&mut node, true, &other[..6]);
        let _ = ask(&mut node, false, &other[6..12]);
        let last = ask(&mut node, false, &other[12..]);
        assert_eq!(last.node_id, 124);
    }
}
