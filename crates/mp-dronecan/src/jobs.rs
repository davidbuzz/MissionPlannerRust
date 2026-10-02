//! The calls the C# makes and blocks on until an answer comes or it gives up - each a thread
//! behind a progress window or a click handler waiting - as machines the caller hands every
//! received message to ([`Job::handle`]) and polls once a frame with the time ([`Job::poll`]).
//!
//! * [`GetParameters`]: `GetParameters(node)` - `param.GetSet` by index from 0, the next asked
//!   as each answer comes or the same one again after 333 ms, until a nameless answer or two
//!   seconds with none (`DroneCAN.cs:534-602`);
//! * [`Request`]: `SetParameter`, `SaveConfig` and `RestartNode` - one request sent every second,
//!   at most four times, until the node answers it (`:430-480, 2034-2165, 2203-2251`);
//! * [`Update`]: `Update(node, name, hwversion, file)` - the file served as `fw.bin`, the node
//!   asked for its info every second until it is updating, told to begin, and watched until the
//!   file has been read to its end, the node gives an error, or a minute passes (`:1098-1290`).
//!
//! Each answer is taken only from the node asked, and a service answer only when it is addressed
//! to us, as each C# handler checks.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::dsdl::{
    BEGIN_ERROR_IN_PROGRESS, BEGIN_ERROR_OK, GetSetReq, GetSetRes, MODE_SOFTWARE_UPDATE,
    OPCODE_SAVE, RESTART_MAGIC_NUMBER, Value, ascii,
};
use crate::node::{Event, Node, PRIORITY, Received};
use crate::{Message, frame::Frame};

/// What a job is doing.
#[derive(Debug, Clone, PartialEq)]
pub enum Poll<T> {
    /// Still waiting.
    Pending,
    /// Over, with this.
    Done(T),
}

/// A job a caller drives.
pub trait Job {
    /// What it ends with.
    type Output;

    /// A received message, any of them: the job takes what it is waiting for.
    fn handle(&mut self, received: &Received, node: &mut Node, now: Instant);

    /// Once a frame: sends what is due, and says whether it is over.
    fn poll(&mut self, node: &mut Node, now: Instant) -> Poll<Self::Output>;
}

/// Whether an answer is for us from `from`: a service answer must be addressed to us.
fn answer_from(frame: &Frame, from: u8, us: u8) -> bool {
    !(frame.is_service() && frame.svc_destination_node() != us) && frame.source_node() == from
}

/// `GetParameters(node)`.
#[derive(Debug, Clone)]
pub struct GetParameters {
    node: u8,
    index: u16,
    timeout: Instant,
    /// The next send: now, or after the 333 ms wait.
    next_send: Option<Instant>,
    list: Vec<GetSetRes>,
    ended: bool,
}

impl GetParameters {
    /// Asking `node` for its parameters, from index 0.
    #[must_use]
    pub fn new(node: u8, now: Instant) -> Self {
        Self {
            node,
            index: 0,
            timeout: now + Duration::from_secs(2),
            next_send: None,
            list: Vec::new(),
            ended: false,
        }
    }

    /// The parameters so far.
    #[must_use]
    pub fn list(&self) -> &[GetSetRes] {
        &self.list
    }
}

impl Job for GetParameters {
    type Output = Vec<GetSetRes>;

    fn handle(&mut self, received: &Received, node: &mut Node, now: Instant) {
        if !answer_from(&received.frame, self.node, node.source_node()) {
            return;
        }
        let Message::GetSetRes(res) = &received.message else {
            return;
        };
        if res.name.is_empty() {
            // `timeout = DateTime.MinValue`: the list is over.
            self.ended = true;
            return;
        }
        if !self.list.iter().any(|held| held.name == res.name) {
            self.list.push(res.clone());
        }
        self.timeout = now + Duration::from_secs(2);
        self.index = self.index.wrapping_add(1);
        // `wait.Release()`: the next request goes at once.
        self.next_send = None;
    }

    fn poll(&mut self, node: &mut Node, now: Instant) -> Poll<Self::Output> {
        if self.ended || now > self.timeout {
            return Poll::Done(std::mem::take(&mut self.list));
        }
        if self.next_send.is_none_or(|at| now >= at) {
            let id = node.next_transfer_id();
            node.send(
                self.node,
                PRIORITY,
                id,
                Message::GetSetReq(GetSetReq {
                    index: self.index,
                    ..GetSetReq::default()
                }),
            );
            self.next_send = Some(now + Duration::from_millis(333));
        }
        Poll::Pending
    }
}

/// What a [`Request`] waits for.
#[derive(Debug, Clone, PartialEq)]
enum Awaiting {
    /// `SetParameter`: a `GetSet` answer with the name and the value's type.
    Set { name: Vec<u8>, tag: u8 },
    /// `SaveConfig`: an `ExecuteOpcode` answer, its `ok`.
    Opcode,
    /// `RestartNode`: a `RestartNode` answer, its `ok`.
    Restart,
}

/// `SetParameter`, `SaveConfig` or `RestartNode`: a request every second, four at most.
#[derive(Debug, Clone)]
pub struct Request {
    node: u8,
    message: Message,
    awaiting: Awaiting,
    tries: u32,
    next_send: Option<Instant>,
    answer: Option<bool>,
}

/// `SetParameter`'s value, as `valuein` arrives: a number, or text for a string parameter.
#[derive(Debug, Clone, PartialEq)]
pub enum SetValue {
    /// A number the user typed, `IConvertible.ToDouble`.
    Number(f64),
    /// Text that is not a number; it counts as 0 for a numeric parameter.
    Text(String),
}

impl Request {
    /// `SetParameter(node, name, value, type)`, the type the parameter's `value` had when the list
    /// was read (`_paramlistcache`). An empty type sends an empty value.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:2034-2165`
    #[must_use]
    pub fn set_parameter(node: u8, name: &str, value: &SetValue, kind: &Value) -> Self {
        let number = match value {
            SetValue::Number(number) => *number,
            SetValue::Text(_) => 0.0,
        };
        // `(int)value`, `(float)value`: .NET's casts.
        #[allow(clippy::cast_possible_truncation)]
        let as_int = number as i32;
        #[allow(clippy::cast_possible_truncation)]
        let as_float = number as f32;
        let value = match kind {
            Value::Empty => Value::Empty,
            Value::Boolean(_) => Value::Boolean(u8::from(number > 0.0)),
            Value::Integer(_) => Value::Integer(i64::from(as_int)),
            Value::Real(_) => Value::Real(as_float),
            Value::String(_) => Value::String(
                match value {
                    SetValue::Number(number) => number.to_string(),
                    SetValue::Text(text) => text.clone(),
                }
                .into_bytes(),
            ),
        };
        let tag = value.tag();
        Self {
            node,
            message: Message::GetSetReq(GetSetReq {
                index: 0,
                value,
                name: name.as_bytes().to_vec(),
            }),
            awaiting: Awaiting::Set {
                name: name.as_bytes().to_vec(),
                tag,
            },
            tries: 0,
            next_send: None,
            answer: None,
        }
    }

    /// `SaveConfig(node)`: `ExecuteOpcode` save.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:430-480`
    #[must_use]
    pub const fn save_config(node: u8) -> Self {
        Self {
            node,
            message: Message::ExecuteOpcodeReq {
                opcode: OPCODE_SAVE,
                argument: 0,
            },
            awaiting: Awaiting::Opcode,
            tries: 0,
            next_send: None,
            answer: None,
        }
    }

    /// `RestartNode(node)`.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:2203-2251`
    #[must_use]
    pub const fn restart_node(node: u8) -> Self {
        Self {
            node,
            message: Message::RestartNodeReq {
                magic_number: RESTART_MAGIC_NUMBER,
            },
            awaiting: Awaiting::Restart,
            tries: 0,
            next_send: None,
            answer: None,
        }
    }
}

impl Job for Request {
    type Output = bool;

    fn handle(&mut self, received: &Received, node: &mut Node, _now: Instant) {
        if !answer_from(&received.frame, self.node, node.source_node()) {
            return;
        }
        match (&self.awaiting, &received.message) {
            (Awaiting::Set { name, tag }, Message::GetSetRes(res))
                if res.name == *name && res.value.tag() == *tag =>
            {
                self.answer = Some(true);
            }
            (Awaiting::Opcode, Message::ExecuteOpcodeRes { ok, .. })
            | (Awaiting::Restart, Message::RestartNodeRes { ok }) => self.answer = Some(*ok),
            _ => {}
        }
    }

    fn poll(&mut self, node: &mut Node, now: Instant) -> Poll<bool> {
        if let Some(answer) = self.answer {
            return Poll::Done(answer);
        }
        if self.next_send.is_none_or(|at| at < now) {
            if self.tries > 3 {
                return Poll::Done(false);
            }
            let id = node.next_transfer_id();
            node.send(self.node, PRIORITY, id, self.message.clone());
            self.next_send = Some(now + Duration::from_secs(1));
            self.tries += 1;
        }
        Poll::Pending
    }
}

/// The two app-descriptor signatures `Update` looks for in a firmware file; the CRC is the eight
/// bytes after the signature.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1207-1227`
pub const APP_DESCRIPTOR_SIGNATURES: [[u8; 8]; 2] = [
    [0xd7, 0xe4, 0xf7, 0xba, 0xd0, 0x0f, 0x9b, 0xee],
    [0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06],
];

/// The firmware's `image_crc` from its app descriptor: the first match of the first signature,
/// else of the second, and the 64 bits after it - zeros for bytes the file does not have.
/// `u64::MAX` (unknown) without one.
#[must_use]
pub fn firmware_crc(image: &[u8]) -> u64 {
    let find = |signature: &[u8; 8]| {
        image
            .windows(signature.len())
            .position(|window| window == signature)
    };
    let Some(offset) =
        find(&APP_DESCRIPTOR_SIGNATURES[0]).or_else(|| find(&APP_DESCRIPTOR_SIGNATURES[1]))
    else {
        return u64::MAX;
    };
    let mut crc = [0u8; 8];
    for (slot, byte) in crc.iter_mut().zip(image.iter().skip(offset + 8)) {
        *slot = *byte;
    }
    u64::from_le_bytes(crc)
}

/// How an update ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UpdateEnd {
    /// The node read the file to its end, or already had it ("No need to upload, crc matchs").
    Done,
    /// The loop gave up: a minute without the update starting or finishing, or cancelled. The C#
    /// returns without a word.
    GaveUp,
    /// `Begin Firmware Update returned an error`: the C#'s exception.
    Error(String),
}

/// `Update(nodeid, devicename, hwversion, firmware_name, cancel)`.
#[derive(Debug, Clone)]
pub struct Update {
    node: u8,
    firmware_crc: u64,
    error: Option<String>,
    done: bool,
    in_update_mode: bool,
    accept_begin: bool,
    /// The loop's next pass: a second after the last.
    next_pass: Instant,
    /// `b`: passes counted since the node's status last moved.
    passes: u32,
    /// `timestamp`: the node's uptime at the last pass.
    timestamp: u32,
    cancelled: bool,
}

/// The name the file is served under, and the one a node asking for `fw.bi` gets too.
pub const SERVED_NAMES: [&str; 2] = ["fw.bin", "fw.bi"];

impl Update {
    /// The file served to `node` and its CRC read; the first pass a second from `now`.
    ///
    /// # Errors
    /// The file cannot be read: the C#'s `File.OpenRead` throws out of `Update`.
    pub fn new(node: u8, firmware: &Path, can: &mut Node, now: Instant) -> std::io::Result<Self> {
        let image = std::fs::read(firmware)?;
        for name in SERVED_NAMES {
            can.serve_file(PathBuf::from(firmware), name);
        }
        Ok(Self {
            node,
            firmware_crc: firmware_crc(&image),
            error: None,
            done: false,
            in_update_mode: false,
            accept_begin: false,
            next_pass: now + Duration::from_secs(1),
            passes: 0,
            timestamp: 0,
            cancelled: false,
        })
    }

    /// The progress window's Cancel: the loop ends at its next pass.
    pub const fn cancel(&mut self) {
        self.cancelled = true;
    }

    /// `FileSendComplete` for our node: the update is over.
    pub fn event(&mut self, event: &Event) {
        if let Event::FileSendComplete { node, .. } = event
            && *node == self.node
        {
            self.done = true;
        }
    }
}

impl Job for Update {
    type Output = UpdateEnd;

    fn handle(&mut self, received: &Received, node: &mut Node, _now: Instant) {
        let frame = &received.frame;
        if !answer_from(frame, self.node, node.source_node()) {
            return;
        }
        match &received.message {
            Message::BeginFirmwareUpdateRes { error, .. }
                if *error != BEGIN_ERROR_IN_PROGRESS && *error != BEGIN_ERROR_OK =>
            {
                self.error = Some(format!(
                    "{} Begin Firmware Update returned an error",
                    frame.source_node()
                ));
            }
            Message::GetNodeInfoRes(info) => {
                if self.accept_begin {
                    return;
                }
                if self.firmware_crc != info.software_version.image_crc
                    || self.firmware_crc == u64::MAX
                {
                    if info.status.mode == MODE_SOFTWARE_UPDATE {
                        self.in_update_mode = true;
                    } else {
                        node.send(
                            frame.source_node(),
                            frame.priority(),
                            received.transfer_id,
                            Message::BeginFirmwareUpdateReq {
                                source_node_id: node.source_node(),
                                path: SERVED_NAMES[0].as_bytes().to_vec(),
                            },
                        );
                    }
                } else {
                    // "No need to upload, crc matchs": `FileSendComplete`, which ends the loop.
                    self.done = true;
                }
            }
            Message::FileReadReq { .. } => self.accept_begin = true,
            _ => {}
        }
    }

    fn poll(&mut self, node: &mut Node, now: Instant) -> Poll<UpdateEnd> {
        if self.done {
            return Poll::Done(UpdateEnd::Done);
        }
        if now < self.next_pass {
            return Poll::Pending;
        }
        self.next_pass = now + Duration::from_secs(1);
        if let Some(error) = self.error.take() {
            return Poll::Done(UpdateEnd::Error(error));
        }
        if self.cancelled {
            return Poll::Done(UpdateEnd::GaveUp);
        }
        match node.node_list().get(&self.node) {
            Some(status) if status.mode == MODE_SOFTWARE_UPDATE => {
                if status.uptime_sec != self.timestamp {
                    self.passes = 0;
                }
                // `b > 10`: "Possible update issue", written to the console only.
                self.timestamp = status.uptime_sec;
            }
            _ => {
                if !self.in_update_mode {
                    let id = node.next_transfer_id();
                    node.send(self.node, PRIORITY, id, Message::GetNodeInfoReq);
                }
            }
        }
        self.passes += 1;
        if self.passes > 60 {
            return Poll::Done(UpdateEnd::GaveUp);
        }
        Poll::Pending
    }
}

/// `GetNodeName`-style text of a parameter's name.
#[must_use]
pub fn name_of(res: &GetSetRes) -> String {
    ascii(&res.name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsdl::{FILE_ERROR_OK, GetNodeInfoRes, NodeStatus, NumericValue};
    use crate::node::Identity;
    use crate::slcan;
    use crate::transfer::{Outcome, Reassembler, package};

    /// A scripted node on the bus: it reads what our node sends and answers.
    struct Peer {
        id: u8,
        reassembler: Reassembler,
        params: Vec<GetSetRes>,
        restart_ok: bool,
        save_ok: bool,
        /// What the peer has been asked, in order.
        heard: Vec<Message>,
        /// Whether to answer at all.
        answering: bool,
    }

    impl Peer {
        fn new(id: u8) -> Self {
            let param = |name: &str, value: Value| GetSetRes {
                value: value.clone(),
                default_value: value,
                max_value: NumericValue::Integer(100),
                min_value: NumericValue::Integer(0),
                name: name.as_bytes().to_vec(),
            };
            Self {
                id,
                reassembler: Reassembler::new(),
                params: vec![
                    param("CAN_NODE", Value::Integer(10)),
                    param("GPS_TYPE", Value::Integer(1)),
                    param("LED_BRIGHTNESS", Value::Real(0.5)),
                ],
                restart_ok: true,
                save_ok: true,
                heard: Vec::new(),
                answering: true,
            }
        }

        /// Reads our node's lines and answers each request to it.
        fn serve(&mut self, node: &mut Node) {
            let mut answers = Vec::new();
            for line in node.take_outgoing() {
                let slcan::Line::Frame(frame, id, payload) = slcan::read_line(&line) else {
                    continue;
                };
                let Outcome::Complete(done) = self.reassembler.push(frame, id, &payload) else {
                    continue;
                };
                let (frame, message, transfer_id) = *done;
                if !frame.is_service()
                    || frame.svc_destination_node() != self.id
                    || !frame.svc_is_request()
                {
                    continue;
                }
                self.heard.push(message.clone());
                if !self.answering {
                    continue;
                }
                let answer = match message {
                    Message::GetSetReq(req) if req.name.is_empty() => Some(Message::GetSetRes(
                        self.params
                            .get(usize::from(req.index))
                            .cloned()
                            .unwrap_or_default(),
                    )),
                    Message::GetSetReq(req) => {
                        let held = self.params.iter_mut().find(|p| p.name == req.name);
                        held.map(|held| {
                            if req.value != Value::Empty {
                                held.value = req.value.clone();
                            }
                            Message::GetSetRes(held.clone())
                        })
                    }
                    Message::ExecuteOpcodeReq { .. } => Some(Message::ExecuteOpcodeRes {
                        argument: 0,
                        ok: self.save_ok,
                    }),
                    Message::RestartNodeReq { magic_number }
                        if magic_number == RESTART_MAGIC_NUMBER =>
                    {
                        Some(Message::RestartNodeRes {
                            ok: self.restart_ok,
                        })
                    }
                    _ => None,
                };
                if let Some(answer) = answer {
                    answers.push((frame.source_node(), transfer_id, answer));
                }
            }
            for (to, transfer_id, answer) in answers {
                for (frame, payload) in package(self.id, to, PRIORITY, transfer_id, &answer, false)
                {
                    node.receive_line(&slcan::format_frame(&frame, &payload, false));
                }
            }
        }
    }

    /// Drives a job to its end against a peer, the clock moving on 50 ms a step.
    fn run<J: Job>(job: &mut J, node: &mut Node, peer: &mut Peer, start: Instant) -> J::Output {
        let mut now = start;
        for _ in 0..2000 {
            if let Poll::Done(output) = job.poll(node, now) {
                return output;
            }
            peer.serve(node);
            while let Some(received) = node.next_received() {
                job.handle(&received, node, now);
            }
            now += Duration::from_millis(50);
        }
        panic!("the job never ended");
    }

    fn started() -> (Node, Instant) {
        let now = Instant::now();
        let mut node = Node::new(Identity::default(), now);
        node.start(now);
        (node, now)
    }

    /// The whole list, by index, ending at the nameless answer past the last.
    #[test]
    fn get_parameters_reads_the_list() {
        let (mut node, now) = started();
        let mut peer = Peer::new(10);
        let mut job = GetParameters::new(10, now);
        let list = run(&mut job, &mut node, &mut peer, now);
        let names: Vec<String> = list.iter().map(name_of).collect();
        assert_eq!(names, ["CAN_NODE", "GPS_TYPE", "LED_BRIGHTNESS"]);
        assert_eq!(peer.heard.len(), 4, "three, and the one past the end");
    }

    /// A node that never answers: the same index asked every 333 ms until two seconds pass.
    #[test]
    fn get_parameters_gives_up_after_two_seconds() {
        let (mut node, now) = started();
        let mut peer = Peer::new(10);
        peer.answering = false;
        let mut job = GetParameters::new(10, now);
        let list = run(&mut job, &mut node, &mut peer, now);
        assert!(list.is_empty());
        assert!(
            (6..=8).contains(&peer.heard.len()),
            "{} asks",
            peer.heard.len()
        );
        assert!(
            peer.heard
                .iter()
                .all(|message| matches!(message, Message::GetSetReq(req) if req.index == 0))
        );
    }

    /// `SetParameter` with the type the list had: the node's value changes and the answer that
    /// names it, with the same type, is the yes.
    #[test]
    fn set_parameter_is_answered() {
        let (mut node, now) = started();
        let mut peer = Peer::new(10);
        let mut job =
            Request::set_parameter(10, "GPS_TYPE", &SetValue::Number(2.7), &Value::Integer(0));
        assert!(run(&mut job, &mut node, &mut peer, now));
        assert_eq!(peer.params[1].value, Value::Integer(2), "(int)2.7");

        let mut job = Request::set_parameter(
            10,
            "LED_BRIGHTNESS",
            &SetValue::Number(0.75),
            &Value::Real(0.0),
        );
        assert!(run(&mut job, &mut node, &mut peer, now));
        assert_eq!(peer.params[2].value, Value::Real(0.75));

        // A parameter the node has not got: four tries, a second apart, then no.
        let mut job =
            Request::set_parameter(10, "NOT_THERE", &SetValue::Number(1.0), &Value::Integer(0));
        let asked = peer.heard.len();
        assert!(!run(&mut job, &mut node, &mut peer, now));
        assert_eq!(peer.heard.len() - asked, 4);
    }

    /// Save and restart: the node's `ok`.
    #[test]
    fn save_and_restart_answer_ok() {
        let (mut node, now) = started();
        let mut peer = Peer::new(10);
        assert!(run(
            &mut Request::save_config(10),
            &mut node,
            &mut peer,
            now
        ));
        peer.restart_ok = false;
        assert!(!run(
            &mut Request::restart_node(10),
            &mut node,
            &mut peer,
            now
        ));
        peer.save_ok = false;
        assert!(!run(
            &mut Request::save_config(10),
            &mut node,
            &mut peer,
            now
        ));
        // Nobody answering node 10: four tries, then no.
        let mut peer = Peer::new(11);
        let mut job = Request::restart_node(10);
        assert!(!run(&mut job, &mut node, &mut peer, now));
    }

    /// The app descriptor's CRC: the first signature preferred, the bytes after it.
    #[test]
    fn the_firmware_crc_is_read_from_the_descriptor() {
        let mut image = vec![0u8; 64];
        image[10..18].copy_from_slice(&APP_DESCRIPTOR_SIGNATURES[1]);
        image[18..26].copy_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes());
        assert_eq!(firmware_crc(&image), 0x1122_3344_5566_7788);
        image[30..38].copy_from_slice(&APP_DESCRIPTOR_SIGNATURES[0]);
        image[38..46].copy_from_slice(&42u64.to_le_bytes());
        assert_eq!(firmware_crc(&image), 42);
        assert_eq!(firmware_crc(&[1, 2, 3]), u64::MAX);
        let mut short = APP_DESCRIPTOR_SIGNATURES[0].to_vec();
        short.extend([1, 2]);
        assert_eq!(firmware_crc(&short), 0x0201);
    }

    /// An update end to end against a scripted bootloader: asked for its info, told to begin,
    /// it reads the file from us to the end - and the update is done.
    #[test]
    fn an_update_runs_to_the_end() {
        let dir = std::env::temp_dir().join(format!("mp-dronecan-update-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory");
        let path = dir.join("firmware.bin");
        let image: Vec<u8> = (0..600u32)
            .map(|i| u8::try_from(i % 256).unwrap())
            .collect();
        std::fs::write(&path, &image).expect("written");

        let (mut node, now) = started();
        node.setup_file_server();
        let mut job = Update::new(10, &path, &mut node, now).expect("readable");
        let mut reassembler = Reassembler::new();
        let mut received_image = Vec::new();
        let mut mode = crate::dsdl::MODE_OPERATIONAL;
        let mut clock = now;
        let mut outcome = None;
        for _ in 0..1000 {
            if let Poll::Done(end) = job.poll(&mut node, clock) {
                outcome = Some(end);
                break;
            }
            // The bootloader: its status every pass; GetNodeInfo answered with its mode; a begin
            // answered OK and the file read 256 bytes at a time.
            let mut replies = vec![(
                0,
                0,
                Message::NodeStatus(NodeStatus {
                    uptime_sec: 5,
                    mode,
                    ..NodeStatus::default()
                }),
            )];
            for line in node.take_outgoing() {
                let slcan::Line::Frame(frame, id, payload) = slcan::read_line(&line) else {
                    continue;
                };
                let Outcome::Complete(done) = reassembler.push(frame, id, &payload) else {
                    continue;
                };
                let (frame, message, transfer_id) = *done;
                assert!(frame.is_service() && frame.svc_destination_node() == 10);
                match message {
                    Message::FileReadRes { error, data } => {
                        assert_eq!(error, FILE_ERROR_OK);
                        received_image.extend_from_slice(&data);
                        if data.len() == 256 {
                            let offset = u64::try_from(received_image.len()).unwrap();
                            replies.push((
                                127,
                                100,
                                Message::FileReadReq {
                                    offset,
                                    path: b"fw.bin".to_vec(),
                                },
                            ));
                        }
                    }
                    Message::GetNodeInfoReq => replies.push((
                        127,
                        transfer_id,
                        Message::GetNodeInfoRes(Box::new(GetNodeInfoRes {
                            status: NodeStatus {
                                mode,
                                ..NodeStatus::default()
                            },
                            ..GetNodeInfoRes::default()
                        })),
                    )),
                    Message::BeginFirmwareUpdateReq {
                        source_node_id,
                        path,
                    } => {
                        assert_eq!(source_node_id, 127);
                        assert_eq!(path, b"fw.bin");
                        mode = MODE_SOFTWARE_UPDATE;
                        replies.push((
                            127,
                            transfer_id,
                            Message::BeginFirmwareUpdateRes {
                                error: BEGIN_ERROR_OK,
                                optional_error_message: Vec::new(),
                            },
                        ));
                        replies.push((
                            127,
                            99,
                            Message::FileReadReq {
                                offset: 0,
                                path: b"fw.bin".to_vec(),
                            },
                        ));
                    }
                    _ => {}
                }
            }
            for (to, transfer_id, reply) in replies {
                for (frame, payload) in package(10, to, PRIORITY, transfer_id, &reply, false) {
                    node.receive_line(&slcan::format_frame(&frame, &payload, false));
                }
            }
            while let Some(received) = node.next_received() {
                job.handle(&received, &mut node, clock);
            }
            for event in node.take_events() {
                job.event(&event);
            }
            clock += Duration::from_millis(100);
        }
        assert_eq!(outcome, Some(UpdateEnd::Done));
        // The last answer went out in the pass that served it; read it off the bus.
        for line in node.take_outgoing() {
            if let slcan::Line::Frame(frame, id, payload) = slcan::read_line(&line)
                && let Outcome::Complete(done) = reassembler.push(frame, id, &payload)
                && let Message::FileReadRes { data, .. } = done.1
            {
                received_image.extend_from_slice(&data);
            }
        }
        assert_eq!(received_image, image);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A begin refused: the C#'s exception, at the next pass.
    #[test]
    fn a_refused_begin_is_an_error() {
        let (mut node, now) = started();
        let dir = std::env::temp_dir().join(format!("mp-dronecan-refused-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory");
        let path = dir.join("fw.bin");
        std::fs::write(&path, [0u8; 16]).expect("written");
        let mut job = Update::new(10, &path, &mut node, now).expect("readable");
        for (frame, payload) in package(
            10,
            127,
            PRIORITY,
            1,
            &Message::BeginFirmwareUpdateRes {
                error: crate::dsdl::BEGIN_ERROR_INVALID_MODE,
                optional_error_message: Vec::new(),
            },
            false,
        ) {
            node.receive_line(&slcan::format_frame(&frame, &payload, false));
        }
        while let Some(received) = node.next_received() {
            job.handle(&received, &mut node, now);
        }
        assert_eq!(
            job.poll(&mut node, now),
            Poll::Pending,
            "the loop sleeps first"
        );
        assert_eq!(
            job.poll(&mut node, now + Duration::from_secs(1)),
            Poll::Done(UpdateEnd::Error(
                "10 Begin Firmware Update returned an error".to_owned()
            ))
        );
        std::fs::remove_dir_all(&dir).ok();
    }
}
