//! The data types the DroneCAN page and its windows send and read, each coded as its generated
//! `out/src/<type>.cs` codes it.
//!
//! A type's top-level `encode`/`decode` passes `tao = !fdcan` (tail array optimisation on for
//! classic CAN): the last field, when it is a dynamic array, carries no length and runs to the
//! end of the payload. Every nested type is coded with `tao = false`, except where the generated
//! code passes its own `tao` on (`file.Path` as the last field of `file.Read` and
//! `file.BeginFirmwareUpdate`'s requests). A dynamic array is a `Vec`, whose length is the C#'s
//! `*_len` field.
//! `// C#: ExtLibs/DroneCAN/out/src/*.cs; ExtLibs/DroneCAN/out/include/*.cs`

use crate::bits::{BitReader, BitWriter, low_byte_u64};
use crate::names::{MSG_INFO, MsgInfo};

/// `uavcan.protocol.NodeStatus` (341).
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.NodeStatus.cs:36-69`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NodeStatus {
    /// `uptime_sec`.
    pub uptime_sec: u32,
    /// `health`, two bits: [`HEALTH_OK`] and on.
    pub health: u8,
    /// `mode`, three bits: [`MODE_OPERATIONAL`] and on.
    pub mode: u8,
    /// `sub_mode`, three bits.
    pub sub_mode: u8,
    /// `vendor_specific_status_code`.
    pub vendor_specific_status_code: u16,
}

/// `UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK`.
pub const HEALTH_OK: u8 = 0;
/// `..._HEALTH_WARNING`.
pub const HEALTH_WARNING: u8 = 1;
/// `..._HEALTH_ERROR`.
pub const HEALTH_ERROR: u8 = 2;
/// `..._HEALTH_CRITICAL`.
pub const HEALTH_CRITICAL: u8 = 3;
/// `UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL`.
pub const MODE_OPERATIONAL: u8 = 0;
/// `..._MODE_INITIALIZATION`.
pub const MODE_INITIALIZATION: u8 = 1;
/// `..._MODE_MAINTENANCE`.
pub const MODE_MAINTENANCE: u8 = 2;
/// `..._MODE_SOFTWARE_UPDATE`.
pub const MODE_SOFTWARE_UPDATE: u8 = 3;
/// `..._MODE_OFFLINE`.
pub const MODE_OFFLINE: u8 = 7;

impl NodeStatus {
    fn encode(&self, w: &mut BitWriter) {
        w.put(u64::from(self.uptime_sec), 32);
        w.put(u64::from(self.health), 2);
        w.put(u64::from(self.mode), 3);
        w.put(u64::from(self.sub_mode), 3);
        w.put(u64::from(self.vendor_specific_status_code), 16);
    }

    fn decode(r: &mut BitReader<'_>) -> Self {
        Self {
            uptime_sec: u32::try_from(r.get(32)).unwrap_or(0),
            health: low_byte_u64(r.get(2)),
            mode: low_byte_u64(r.get(3)),
            sub_mode: low_byte_u64(r.get(3)),
            vendor_specific_status_code: u16::try_from(r.get(16)).unwrap_or(0),
        }
    }
}

/// A dynamic `uint8[]` field: its length in `len_bits` unless it is the tail of a transfer under
/// tail array optimisation, then its bytes.
fn put_array(w: &mut BitWriter, bytes: &[u8], len_bits: u32, tao: bool) {
    if !tao {
        w.put(u64::try_from(bytes.len()).unwrap_or(u64::MAX), len_bits);
    }
    for byte in bytes {
        w.put(u64::from(*byte), 8);
    }
}

/// The decoder's side of [`put_array`].
fn get_array(r: &mut BitReader<'_>, len_bits: u32, tao: bool) -> Vec<u8> {
    let len = if tao {
        r.tail_len(len_bits)
    } else {
        usize::try_from(r.get(len_bits)).unwrap_or(0)
    };
    r.bytes(len)
}

/// `uavcan.protocol.SoftwareVersion`.
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.SoftwareVersion.cs:36-69`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SoftwareVersion {
    /// `major`.
    pub major: u8,
    /// `minor`.
    pub minor: u8,
    /// `optional_field_flags`.
    pub optional_field_flags: u8,
    /// `vcs_commit`.
    pub vcs_commit: u32,
    /// `image_crc`.
    pub image_crc: u64,
}

impl SoftwareVersion {
    fn encode(&self, w: &mut BitWriter) {
        w.put(u64::from(self.major), 8);
        w.put(u64::from(self.minor), 8);
        w.put(u64::from(self.optional_field_flags), 8);
        w.put(u64::from(self.vcs_commit), 32);
        w.put(self.image_crc, 64);
    }

    fn decode(r: &mut BitReader<'_>) -> Self {
        Self {
            major: low_byte_u64(r.get(8)),
            minor: low_byte_u64(r.get(8)),
            optional_field_flags: low_byte_u64(r.get(8)),
            vcs_commit: u32::try_from(r.get(32)).unwrap_or(0),
            image_crc: r.get(64),
        }
    }
}

/// `uavcan.protocol.HardwareVersion`.
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.HardwareVersion.cs:36-84`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HardwareVersion {
    /// `major`.
    pub major: u8,
    /// `minor`.
    pub minor: u8,
    /// `unique_id[16]`.
    pub unique_id: [u8; 16],
    /// `certificate_of_authenticity`, up to 255 bytes.
    pub certificate_of_authenticity: Vec<u8>,
}

impl HardwareVersion {
    fn encode(&self, w: &mut BitWriter, tao: bool) {
        w.put(u64::from(self.major), 8);
        w.put(u64::from(self.minor), 8);
        for byte in self.unique_id {
            w.put(u64::from(byte), 8);
        }
        put_array(w, &self.certificate_of_authenticity, 8, tao);
    }

    fn decode(r: &mut BitReader<'_>, tao: bool) -> Self {
        let major = low_byte_u64(r.get(8));
        let minor = low_byte_u64(r.get(8));
        let mut unique_id = [0u8; 16];
        for byte in &mut unique_id {
            *byte = low_byte_u64(r.get(8));
        }
        Self {
            major,
            minor,
            unique_id,
            certificate_of_authenticity: get_array(r, 8, tao),
        }
    }
}

/// `uavcan.protocol.GetNodeInfo`'s response (service 1).
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.GetNodeInfo_res.cs:36-71`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct GetNodeInfoRes {
    /// `status`.
    pub status: NodeStatus,
    /// `software_version`.
    pub software_version: SoftwareVersion,
    /// `hardware_version`.
    pub hardware_version: HardwareVersion,
    /// `name`, up to 80 bytes.
    pub name: Vec<u8>,
}

impl GetNodeInfoRes {
    fn encode(&self, w: &mut BitWriter, tao: bool) {
        self.status.encode(w);
        self.software_version.encode(w);
        self.hardware_version.encode(w, false);
        put_array(w, &self.name, 7, tao);
    }

    fn decode(r: &mut BitReader<'_>, tao: bool) -> Self {
        Self {
            status: NodeStatus::decode(r),
            software_version: SoftwareVersion::decode(r),
            hardware_version: HardwareVersion::decode(r, false),
            name: get_array(r, 7, tao),
        }
    }
}

/// `uavcan.protocol.RestartNode`'s magic number, which the request must carry.
pub const RESTART_MAGIC_NUMBER: u64 = 742_196_058_910;

/// `uavcan.protocol.param.Value`: a union, its tag in three bits.
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.param.Value.cs:36-127`
#[derive(Debug, Clone, Default, PartialEq)]
pub enum Value {
    /// `empty`.
    #[default]
    Empty,
    /// `integer_value`, int64.
    Integer(i64),
    /// `real_value`, float32.
    Real(f32),
    /// `boolean_value`, a uint8.
    Boolean(u8),
    /// `string_value`, up to 128 bytes.
    String(Vec<u8>),
}

impl Value {
    /// The union's tag: `uavcan_protocol_param_Value_type_t`.
    #[must_use]
    pub const fn tag(&self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Integer(_) => 1,
            Self::Real(_) => 2,
            Self::Boolean(_) => 3,
            Self::String(_) => 4,
        }
    }

    fn encode(&self, w: &mut BitWriter) {
        w.put(u64::from(self.tag()), 3);
        match self {
            Self::Empty => {}
            Self::Integer(value) => w.put_signed(*value, 64),
            Self::Real(value) => w.put_f32(*value),
            Self::Boolean(value) => w.put(u64::from(*value), 8),
            Self::String(bytes) => put_array(w, bytes, 8, false),
        }
    }

    fn decode(r: &mut BitReader<'_>) -> Self {
        match r.get(3) {
            1 => Self::Integer(r.get_signed(64)),
            2 => Self::Real(r.get_f32()),
            3 => Self::Boolean(low_byte_u64(r.get(8))),
            4 => Self::String(get_array(r, 8, false)),
            // 0, and the tags the union does not have: the C#'s switch reads nothing for them.
            _ => Self::Empty,
        }
    }
}

/// `uavcan.protocol.param.NumericValue`: a union, its tag in two bits.
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.param.NumericValue.cs:36-86`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum NumericValue {
    /// `empty`.
    #[default]
    Empty,
    /// `integer_value`, int64.
    Integer(i64),
    /// `real_value`, float32.
    Real(f32),
}

impl NumericValue {
    fn encode(&self, w: &mut BitWriter) {
        match self {
            Self::Empty => w.put(0, 2),
            Self::Integer(value) => {
                w.put(1, 2);
                w.put_signed(*value, 64);
            }
            Self::Real(value) => {
                w.put(2, 2);
                w.put_f32(*value);
            }
        }
    }

    fn decode(r: &mut BitReader<'_>) -> Self {
        match r.get(2) {
            1 => Self::Integer(r.get_signed(64)),
            2 => Self::Real(r.get_f32()),
            _ => Self::Empty,
        }
    }
}

/// `uavcan.protocol.param.GetSet`'s request (service 11).
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.param.GetSet_req.cs:36-71`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GetSetReq {
    /// `index`, thirteen bits.
    pub index: u16,
    /// `value`: empty to read.
    pub value: Value,
    /// `name`, up to 92 bytes; empty to read by index.
    pub name: Vec<u8>,
}

impl GetSetReq {
    fn encode(&self, w: &mut BitWriter, tao: bool) {
        w.put(u64::from(self.index), 13);
        self.value.encode(w);
        put_array(w, &self.name, 7, tao);
    }

    fn decode(r: &mut BitReader<'_>, tao: bool) -> Self {
        Self {
            index: u16::try_from(r.get(13)).unwrap_or(0),
            value: Value::decode(r),
            name: get_array(r, 7, tao),
        }
    }
}

/// `uavcan.protocol.param.GetSet`'s response.
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.param.GetSet_res.cs:36-86`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GetSetRes {
    /// `value`.
    pub value: Value,
    /// `default_value`.
    pub default_value: Value,
    /// `max_value`.
    pub max_value: NumericValue,
    /// `min_value`.
    pub min_value: NumericValue,
    /// `name`: empty past the last parameter.
    pub name: Vec<u8>,
}

impl GetSetRes {
    fn encode(&self, w: &mut BitWriter, tao: bool) {
        w.skip(5);
        self.value.encode(w);
        w.skip(5);
        self.default_value.encode(w);
        w.skip(6);
        self.max_value.encode(w);
        w.skip(6);
        self.min_value.encode(w);
        put_array(w, &self.name, 7, tao);
    }

    fn decode(r: &mut BitReader<'_>, tao: bool) -> Self {
        r.skip(5);
        let value = Value::decode(r);
        r.skip(5);
        let default_value = Value::decode(r);
        r.skip(6);
        let max_value = NumericValue::decode(r);
        r.skip(6);
        let min_value = NumericValue::decode(r);
        Self {
            value,
            default_value,
            max_value,
            min_value,
            name: get_array(r, 7, tao),
        }
    }
}

/// `uavcan.protocol.param.ExecuteOpcode`'s `OPCODE_SAVE`.
pub const OPCODE_SAVE: u8 = 0;
/// `..._OPCODE_ERASE`.
pub const OPCODE_ERASE: u8 = 1;

/// `uavcan.protocol.file.Error`'s `OK`.
pub const FILE_ERROR_OK: i16 = 0;
/// `..._NOT_FOUND`.
pub const FILE_ERROR_NOT_FOUND: i16 = 2;

/// `uavcan.protocol.file.BeginFirmwareUpdate`'s `ERROR_OK`.
pub const BEGIN_ERROR_OK: u8 = 0;
/// `..._ERROR_INVALID_MODE`.
pub const BEGIN_ERROR_INVALID_MODE: u8 = 1;
/// `..._ERROR_IN_PROGRESS`.
pub const BEGIN_ERROR_IN_PROGRESS: u8 = 2;

/// `uavcan.protocol.debug.LogMessage` (16383).
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.debug.LogMessage.cs:36-81`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LogMessage {
    /// `level.value`, three bits.
    pub level: u8,
    /// `source`, up to 31 bytes.
    pub source: Vec<u8>,
    /// `text`, up to 90 bytes.
    pub text: Vec<u8>,
}

/// `uavcan.protocol.dynamic_node_id.Allocation` (1), sent anonymously by a node without an id
/// and answered by the allocator.
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.protocol.dynamic_node_id.Allocation.cs:36-74`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Allocation {
    /// `node_id`, seven bits.
    pub node_id: u8,
    /// `first_part_of_unique_id`.
    pub first_part_of_unique_id: bool,
    /// `unique_id`, up to 16 bytes.
    pub unique_id: Vec<u8>,
}

/// `dronecan.protocol.Stats` (342).
/// `// C#: ExtLibs/DroneCAN/out/src/dronecan.protocol.Stats.cs:36-111`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// `tx_frames`.
    pub tx_frames: u32,
    /// `tx_errors`.
    pub tx_errors: u16,
    /// `rx_frames`.
    pub rx_frames: u32,
    /// `rx_error_oom`.
    pub rx_error_oom: u16,
    /// `rx_error_internal`.
    pub rx_error_internal: u16,
    /// `rx_error_missed_start`.
    pub rx_error_missed_start: u16,
    /// `rx_error_wrong_toggle`.
    pub rx_error_wrong_toggle: u16,
    /// `rx_error_short_frame`.
    pub rx_error_short_frame: u16,
    /// `rx_error_bad_crc`.
    pub rx_error_bad_crc: u16,
    /// `rx_ignored_wrong_address`.
    pub rx_ignored_wrong_address: u16,
    /// `rx_ignored_not_wanted`.
    pub rx_ignored_not_wanted: u16,
    /// `rx_ignored_unexpected_tid`.
    pub rx_ignored_unexpected_tid: u16,
}

/// `dronecan.protocol.CanStats` (343).
/// `// C#: ExtLibs/DroneCAN/out/src/dronecan.protocol.CanStats.cs:36-105`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CanStats {
    /// `interface`.
    pub interface: u8,
    /// `tx_requests`.
    pub tx_requests: u32,
    /// `tx_rejected`.
    pub tx_rejected: u16,
    /// `tx_overflow`.
    pub tx_overflow: u16,
    /// `tx_success`.
    pub tx_success: u16,
    /// `tx_timedout`.
    pub tx_timedout: u16,
    /// `tx_abort`.
    pub tx_abort: u16,
    /// `rx_received`.
    pub rx_received: u32,
    /// `rx_overflow`.
    pub rx_overflow: u16,
    /// `rx_errors`.
    pub rx_errors: u16,
    /// `busoff_errors`.
    pub busoff_errors: u16,
}

/// `uavcan.tunnel.Protocol`'s `GPS_GENERIC`.
pub const TUNNEL_PROTOCOL_GPS_GENERIC: u8 = 2;
/// `uavcan.tunnel.Targetted`'s `OPTION_LOCK_PORT`.
pub const TUNNEL_OPTION_LOCK_PORT: u8 = 1;

/// `uavcan.tunnel.Targetted` (3001).
/// `// C#: ExtLibs/DroneCAN/out/src/uavcan.tunnel.Targetted.cs:36-89`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TunnelTargetted {
    /// `protocol.protocol`.
    pub protocol: u8,
    /// `target_node`, seven bits.
    pub target_node: u8,
    /// `serial_id`, a signed five bits; -1 for any.
    pub serial_id: i8,
    /// `options`, four bits.
    pub options: u8,
    /// `baudrate`, 24 bits.
    pub baudrate: u32,
    /// `buffer`, up to 120 bytes.
    pub buffer: Vec<u8>,
}

/// One of the data types here, or another the table knows, undecoded.
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    /// `uavcan.protocol.NodeStatus`.
    NodeStatus(NodeStatus),
    /// `uavcan.protocol.GetNodeInfo` request: no fields.
    GetNodeInfoReq,
    /// `uavcan.protocol.GetNodeInfo` response.
    GetNodeInfoRes(Box<GetNodeInfoRes>),
    /// `uavcan.protocol.RestartNode` request: `magic_number`, 40 bits.
    RestartNodeReq {
        /// `magic_number`.
        magic_number: u64,
    },
    /// `uavcan.protocol.RestartNode` response.
    RestartNodeRes {
        /// `ok`.
        ok: bool,
    },
    /// `uavcan.protocol.param.GetSet` request.
    GetSetReq(GetSetReq),
    /// `uavcan.protocol.param.GetSet` response.
    GetSetRes(GetSetRes),
    /// `uavcan.protocol.param.ExecuteOpcode` request.
    ExecuteOpcodeReq {
        /// `opcode`.
        opcode: u8,
        /// `argument`, a signed 48 bits.
        argument: i64,
    },
    /// `uavcan.protocol.param.ExecuteOpcode` response.
    ExecuteOpcodeRes {
        /// `argument`, a signed 48 bits.
        argument: i64,
        /// `ok`.
        ok: bool,
    },
    /// `uavcan.protocol.file.Read` request.
    FileReadReq {
        /// `offset`, 40 bits.
        offset: u64,
        /// `path.path`, up to 200 bytes.
        path: Vec<u8>,
    },
    /// `uavcan.protocol.file.Read` response.
    FileReadRes {
        /// `error.value`.
        error: i16,
        /// `data`, up to 256 bytes.
        data: Vec<u8>,
    },
    /// `uavcan.protocol.file.BeginFirmwareUpdate` request.
    BeginFirmwareUpdateReq {
        /// `source_node_id`.
        source_node_id: u8,
        /// `image_file_remote_path.path`.
        path: Vec<u8>,
    },
    /// `uavcan.protocol.file.BeginFirmwareUpdate` response.
    BeginFirmwareUpdateRes {
        /// `error`.
        error: u8,
        /// `optional_error_message`, up to 127 bytes.
        optional_error_message: Vec<u8>,
    },
    /// `uavcan.protocol.dynamic_node_id.Allocation`.
    Allocation(Allocation),
    /// `uavcan.protocol.debug.LogMessage`.
    LogMessage(LogMessage),
    /// `dronecan.protocol.Stats`.
    Stats(Stats),
    /// `dronecan.protocol.CanStats`.
    CanStats(CanStats),
    /// `uavcan.equipment.gnss.RTCMStream` (1062).
    RtcmStream {
        /// `protocol_id`.
        protocol_id: u8,
        /// `data`, up to 128 bytes.
        data: Vec<u8>,
    },
    /// `ardupilot.gnss.MovingBaselineData` (20005).
    MovingBaselineData {
        /// `data`, up to 300 bytes.
        data: Vec<u8>,
    },
    /// `uavcan.tunnel.Targetted`.
    TunnelTargetted(TunnelTargetted),
    /// A type the table knows and nothing here decodes: its row, and the payload's bytes.
    Other {
        /// Its `MSG_INFO` row.
        info: &'static MsgInfo,
        /// The transfer's payload.
        payload: Vec<u8>,
    },
}

/// The row of a type, by name; the table always has the names this module uses.
fn info(name: &str) -> &'static MsgInfo {
    static EMPTY: MsgInfo = MsgInfo {
        name: "",
        id: 0,
        signature: 0,
    };
    MSG_INFO
        .iter()
        .find(|row| row.name == name)
        .unwrap_or(&EMPTY)
}

impl Message {
    /// The type's `MSG_INFO` row: its id and signature.
    #[must_use]
    pub fn info(&self) -> &'static MsgInfo {
        match self {
            Self::NodeStatus(_) => info("uavcan_protocol_NodeStatus"),
            Self::GetNodeInfoReq => info("uavcan_protocol_GetNodeInfo_req"),
            Self::GetNodeInfoRes(_) => info("uavcan_protocol_GetNodeInfo_res"),
            Self::RestartNodeReq { .. } => info("uavcan_protocol_RestartNode_req"),
            Self::RestartNodeRes { .. } => info("uavcan_protocol_RestartNode_res"),
            Self::GetSetReq(_) => info("uavcan_protocol_param_GetSet_req"),
            Self::GetSetRes(_) => info("uavcan_protocol_param_GetSet_res"),
            Self::ExecuteOpcodeReq { .. } => info("uavcan_protocol_param_ExecuteOpcode_req"),
            Self::ExecuteOpcodeRes { .. } => info("uavcan_protocol_param_ExecuteOpcode_res"),
            Self::FileReadReq { .. } => info("uavcan_protocol_file_Read_req"),
            Self::FileReadRes { .. } => info("uavcan_protocol_file_Read_res"),
            Self::BeginFirmwareUpdateReq { .. } => {
                info("uavcan_protocol_file_BeginFirmwareUpdate_req")
            }
            Self::BeginFirmwareUpdateRes { .. } => {
                info("uavcan_protocol_file_BeginFirmwareUpdate_res")
            }
            Self::Allocation(_) => info("uavcan_protocol_dynamic_node_id_Allocation"),
            Self::LogMessage(_) => info("uavcan_protocol_debug_LogMessage"),
            Self::Stats(_) => info("dronecan_protocol_Stats"),
            Self::CanStats(_) => info("dronecan_protocol_CanStats"),
            Self::RtcmStream { .. } => info("uavcan_equipment_gnss_RTCMStream"),
            Self::MovingBaselineData { .. } => info("ardupilot_gnss_MovingBaselineData"),
            Self::TunnelTargetted(_) => info("uavcan_tunnel_Targetted"),
            Self::Other { info, .. } => info,
        }
    }

    /// `encode(dronecan_transmit_chunk_handler, state, fdcan)` and `state.ToBytes()`: the
    /// payload, tail array optimisation on unless `fd`.
    #[must_use]
    pub fn encode(&self, fd: bool) -> Vec<u8> {
        let tao = !fd;
        let mut w = BitWriter::new();
        match self {
            Self::NodeStatus(status) => status.encode(&mut w),
            Self::GetNodeInfoReq => {}
            Self::GetNodeInfoRes(res) => res.encode(&mut w, tao),
            Self::RestartNodeReq { magic_number } => w.put(*magic_number, 40),
            Self::RestartNodeRes { ok } => w.put_bool(*ok),
            Self::GetSetReq(req) => req.encode(&mut w, tao),
            Self::GetSetRes(res) => res.encode(&mut w, tao),
            Self::ExecuteOpcodeReq { opcode, argument } => {
                w.put(u64::from(*opcode), 8);
                w.put_signed(*argument, 48);
            }
            Self::ExecuteOpcodeRes { argument, ok } => {
                w.put_signed(*argument, 48);
                w.put_bool(*ok);
            }
            Self::FileReadReq { offset, path } => {
                w.put(*offset, 40);
                put_array(&mut w, path, 8, tao);
            }
            Self::FileReadRes { error, data } => {
                w.put_signed(i64::from(*error), 16);
                put_array(&mut w, data, 9, tao);
            }
            Self::BeginFirmwareUpdateReq {
                source_node_id,
                path,
            } => {
                w.put(u64::from(*source_node_id), 8);
                put_array(&mut w, path, 8, tao);
            }
            Self::BeginFirmwareUpdateRes {
                error,
                optional_error_message,
            } => {
                w.put(u64::from(*error), 8);
                put_array(&mut w, optional_error_message, 7, tao);
            }
            Self::Allocation(allocation) => {
                w.put(u64::from(allocation.node_id), 7);
                w.put_bool(allocation.first_part_of_unique_id);
                put_array(&mut w, &allocation.unique_id, 5, tao);
            }
            Self::LogMessage(log) => {
                w.put(u64::from(log.level), 3);
                put_array(&mut w, &log.source, 5, false);
                put_array(&mut w, &log.text, 7, tao);
            }
            Self::Stats(stats) => {
                w.put(u64::from(stats.tx_frames), 32);
                w.put(u64::from(stats.tx_errors), 16);
                w.put(u64::from(stats.rx_frames), 32);
                for value in [
                    stats.rx_error_oom,
                    stats.rx_error_internal,
                    stats.rx_error_missed_start,
                    stats.rx_error_wrong_toggle,
                    stats.rx_error_short_frame,
                    stats.rx_error_bad_crc,
                    stats.rx_ignored_wrong_address,
                    stats.rx_ignored_not_wanted,
                    stats.rx_ignored_unexpected_tid,
                ] {
                    w.put(u64::from(value), 16);
                }
            }
            Self::CanStats(stats) => {
                w.put(u64::from(stats.interface), 8);
                w.put(u64::from(stats.tx_requests), 32);
                for value in [
                    stats.tx_rejected,
                    stats.tx_overflow,
                    stats.tx_success,
                    stats.tx_timedout,
                    stats.tx_abort,
                ] {
                    w.put(u64::from(value), 16);
                }
                w.put(u64::from(stats.rx_received), 32);
                for value in [stats.rx_overflow, stats.rx_errors, stats.busoff_errors] {
                    w.put(u64::from(value), 16);
                }
            }
            Self::RtcmStream { protocol_id, data } => {
                w.put(u64::from(*protocol_id), 8);
                put_array(&mut w, data, 8, tao);
            }
            Self::MovingBaselineData { data } => put_array(&mut w, data, 9, tao),
            Self::TunnelTargetted(tunnel) => {
                w.put(u64::from(tunnel.protocol), 8);
                w.put(u64::from(tunnel.target_node), 7);
                w.put_signed(i64::from(tunnel.serial_id), 5);
                w.put(u64::from(tunnel.options), 4);
                w.put(u64::from(tunnel.baudrate), 24);
                put_array(&mut w, &tunnel.buffer, 7, tao);
            }
            Self::Other { payload, .. } => return payload.clone(),
        }
        w.into_bytes()
    }

    /// `ByteArrayToDroneCANMsg(transfer, 0, fdcan)` for the type `info`: the payload decoded,
    /// tail array optimisation on unless `fd`.
    #[must_use]
    pub fn decode(info: &'static MsgInfo, payload: &[u8], fd: bool) -> Self {
        let tao = !fd;
        let mut r = BitReader::new(payload);
        let r = &mut r;
        let u16_of = |value: u64| u16::try_from(value).unwrap_or(0);
        let u32_of = |value: u64| u32::try_from(value).unwrap_or(0);
        match info.name {
            "uavcan_protocol_NodeStatus" => Self::NodeStatus(NodeStatus::decode(r)),
            "uavcan_protocol_GetNodeInfo_req" => Self::GetNodeInfoReq,
            "uavcan_protocol_GetNodeInfo_res" => {
                Self::GetNodeInfoRes(Box::new(GetNodeInfoRes::decode(r, tao)))
            }
            "uavcan_protocol_RestartNode_req" => Self::RestartNodeReq {
                magic_number: r.get(40),
            },
            "uavcan_protocol_RestartNode_res" => Self::RestartNodeRes { ok: r.get_bool() },
            "uavcan_protocol_param_GetSet_req" => Self::GetSetReq(GetSetReq::decode(r, tao)),
            "uavcan_protocol_param_GetSet_res" => Self::GetSetRes(GetSetRes::decode(r, tao)),
            "uavcan_protocol_param_ExecuteOpcode_req" => Self::ExecuteOpcodeReq {
                opcode: low_byte_u64(r.get(8)),
                argument: r.get_signed(48),
            },
            "uavcan_protocol_param_ExecuteOpcode_res" => Self::ExecuteOpcodeRes {
                argument: r.get_signed(48),
                ok: r.get_bool(),
            },
            "uavcan_protocol_file_Read_req" => Self::FileReadReq {
                offset: r.get(40),
                path: get_array(r, 8, tao),
            },
            "uavcan_protocol_file_Read_res" => Self::FileReadRes {
                error: i16::try_from(r.get_signed(16)).unwrap_or(0),
                data: get_array(r, 9, tao),
            },
            "uavcan_protocol_file_BeginFirmwareUpdate_req" => Self::BeginFirmwareUpdateReq {
                source_node_id: low_byte_u64(r.get(8)),
                path: get_array(r, 8, tao),
            },
            "uavcan_protocol_file_BeginFirmwareUpdate_res" => Self::BeginFirmwareUpdateRes {
                error: low_byte_u64(r.get(8)),
                optional_error_message: get_array(r, 7, tao),
            },
            "uavcan_protocol_dynamic_node_id_Allocation" => Self::Allocation(Allocation {
                node_id: low_byte_u64(r.get(7)),
                first_part_of_unique_id: r.get_bool(),
                unique_id: get_array(r, 5, tao),
            }),
            "uavcan_protocol_debug_LogMessage" => Self::LogMessage(LogMessage {
                level: low_byte_u64(r.get(3)),
                source: get_array(r, 5, false),
                text: get_array(r, 7, tao),
            }),
            "dronecan_protocol_Stats" => Self::Stats(Stats {
                tx_frames: u32_of(r.get(32)),
                tx_errors: u16_of(r.get(16)),
                rx_frames: u32_of(r.get(32)),
                rx_error_oom: u16_of(r.get(16)),
                rx_error_internal: u16_of(r.get(16)),
                rx_error_missed_start: u16_of(r.get(16)),
                rx_error_wrong_toggle: u16_of(r.get(16)),
                rx_error_short_frame: u16_of(r.get(16)),
                rx_error_bad_crc: u16_of(r.get(16)),
                rx_ignored_wrong_address: u16_of(r.get(16)),
                rx_ignored_not_wanted: u16_of(r.get(16)),
                rx_ignored_unexpected_tid: u16_of(r.get(16)),
            }),
            "dronecan_protocol_CanStats" => Self::CanStats(CanStats {
                interface: low_byte_u64(r.get(8)),
                tx_requests: u32_of(r.get(32)),
                tx_rejected: u16_of(r.get(16)),
                tx_overflow: u16_of(r.get(16)),
                tx_success: u16_of(r.get(16)),
                tx_timedout: u16_of(r.get(16)),
                tx_abort: u16_of(r.get(16)),
                rx_received: u32_of(r.get(32)),
                rx_overflow: u16_of(r.get(16)),
                rx_errors: u16_of(r.get(16)),
                busoff_errors: u16_of(r.get(16)),
            }),
            "uavcan_equipment_gnss_RTCMStream" => Self::RtcmStream {
                protocol_id: low_byte_u64(r.get(8)),
                data: get_array(r, 8, tao),
            },
            "ardupilot_gnss_MovingBaselineData" => Self::MovingBaselineData {
                data: get_array(r, 9, tao),
            },
            "uavcan_tunnel_Targetted" => Self::TunnelTargetted(TunnelTargetted {
                protocol: low_byte_u64(r.get(8)),
                target_node: low_byte_u64(r.get(7)),
                serial_id: i8::try_from(r.get_signed(5)).unwrap_or(0),
                options: low_byte_u64(r.get(4)),
                baudrate: u32_of(r.get(24)),
                buffer: get_array(r, 7, tao),
            }),
            _ => Self::Other {
                info,
                payload: payload.to_vec(),
            },
        }
    }
}

/// `ASCIIEncoding.ASCII.GetString(bytes)`: each byte below 128 as itself, the rest as `?`.
#[must_use]
pub fn ascii(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|byte| {
            if byte.is_ascii() {
                char::from(*byte)
            } else {
                '?'
            }
        })
        .collect()
}

/// `Extension.GetValue(Value)`, as the parameter grid's cell shows it: the integer, the real
/// through `single` (the cell's `float.ToString()`, which the caller has from .NET's rules), the
/// boolean byte, the string, or "Empty".
/// `// C#: ExtLibs/DroneCAN/Extension.cs:29-51`
#[must_use]
pub fn value_text(value: &Value, single: impl Fn(f32) -> String) -> String {
    match value {
        Value::Empty => "Empty".to_owned(),
        Value::Integer(value) => value.to_string(),
        Value::Real(value) => single(*value),
        Value::Boolean(value) => value.to_string(),
        Value::String(bytes) => ascii(bytes),
    }
}

/// `Extension.GetValue(NumericValue)`: the integer, the real, or "" for empty.
/// `// C#: ExtLibs/DroneCAN/Extension.cs:53-68`
#[must_use]
pub fn numeric_text(value: &NumericValue, single: impl Fn(f32) -> String) -> String {
    match value {
        NumericValue::Empty => String::new(),
        NumericValue::Integer(value) => value.to_string(),
        NumericValue::Real(value) => single(*value),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::names::by_name;

    fn round_trip(message: &Message, fd: bool) -> Message {
        let bytes = message.encode(fd);
        Message::decode(message.info(), &bytes, fd)
    }

    /// Every type here survives being coded and decoded, under tail array optimisation and
    /// without it.
    #[test]
    fn every_type_round_trips() {
        let mut unique_id = [0u8; 16];
        unique_id[..14].copy_from_slice(b"MissionPlanner");
        let messages = [
            Message::NodeStatus(NodeStatus {
                uptime_sec: 1234,
                health: HEALTH_WARNING,
                mode: MODE_MAINTENANCE,
                sub_mode: 5,
                vendor_specific_status_code: 0xbeef,
            }),
            Message::GetNodeInfoReq,
            Message::GetNodeInfoRes(Box::new(GetNodeInfoRes {
                status: NodeStatus {
                    uptime_sec: 7,
                    ..NodeStatus::default()
                },
                software_version: SoftwareVersion {
                    major: 1,
                    minor: 3,
                    optional_field_flags: 3,
                    vcs_commit: 0x8979,
                    image_crc: 0x0123_4567_89ab_cdef,
                },
                hardware_version: HardwareVersion {
                    major: 2,
                    minor: 1,
                    unique_id,
                    certificate_of_authenticity: vec![9, 8, 7],
                },
                name: b"org.ardupilot.periph".to_vec(),
            })),
            Message::RestartNodeReq {
                magic_number: RESTART_MAGIC_NUMBER,
            },
            Message::RestartNodeRes { ok: true },
            Message::GetSetReq(GetSetReq {
                index: 4095,
                value: Value::String(b"hello".to_vec()),
                name: b"CAN_NODE".to_vec(),
            }),
            Message::GetSetRes(GetSetRes {
                value: Value::Real(0.25),
                default_value: Value::Integer(-5),
                max_value: NumericValue::Integer(127),
                min_value: NumericValue::Real(-1.5),
                name: b"GPS_TYPE".to_vec(),
            }),
            Message::ExecuteOpcodeReq {
                opcode: OPCODE_ERASE,
                argument: -2,
            },
            Message::ExecuteOpcodeRes {
                argument: 0,
                ok: true,
            },
            Message::FileReadReq {
                offset: 0x12_3456_7890,
                path: b"fw.bin".to_vec(),
            },
            Message::FileReadRes {
                error: FILE_ERROR_OK,
                data: (0..=255).collect(),
            },
            Message::BeginFirmwareUpdateReq {
                source_node_id: 127,
                path: b"fw.bin".to_vec(),
            },
            Message::BeginFirmwareUpdateRes {
                error: BEGIN_ERROR_IN_PROGRESS,
                optional_error_message: b"busy".to_vec(),
            },
            Message::Allocation(Allocation {
                node_id: 125,
                first_part_of_unique_id: true,
                unique_id: vec![1, 2, 3, 4, 5, 6],
            }),
            Message::LogMessage(LogMessage {
                level: 2,
                source: b"GPS".to_vec(),
                text: b"no fix".to_vec(),
            }),
            Message::Stats(Stats {
                tx_frames: 100_000,
                rx_error_bad_crc: 3,
                rx_ignored_unexpected_tid: 9,
                ..Stats::default()
            }),
            Message::CanStats(CanStats {
                interface: 1,
                rx_received: 77,
                busoff_errors: 2,
                ..CanStats::default()
            }),
            Message::RtcmStream {
                protocol_id: 3,
                data: vec![0xd3, 0, 1],
            },
            Message::MovingBaselineData { data: vec![1; 300] },
            Message::TunnelTargetted(TunnelTargetted {
                protocol: TUNNEL_PROTOCOL_GPS_GENERIC,
                target_node: 125,
                serial_id: -1,
                options: TUNNEL_OPTION_LOCK_PORT,
                baudrate: 230_400,
                buffer: vec![0x55, 0x55, 0xb5, 0x62],
            }),
        ];
        for message in &messages {
            assert_eq!(&round_trip(message, false), message, "classic");
            assert_eq!(&round_trip(message, true), message, "FD");
        }
    }

    /// The payload sizes the specification gives: NodeStatus 7 bytes; a GetNodeInfo request
    /// none; RestartNode's request 5; the ExecuteOpcode pair 7 each, as `SaveConfig`'s comment
    /// shows them on the wire.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:432-433`
    #[test]
    fn payload_sizes() {
        assert_eq!(
            Message::NodeStatus(NodeStatus::default())
                .encode(false)
                .len(),
            7
        );
        assert!(Message::GetNodeInfoReq.encode(false).is_empty());
        assert_eq!(
            Message::RestartNodeReq {
                magic_number: RESTART_MAGIC_NUMBER
            }
            .encode(false)
            .len(),
            5
        );
        assert_eq!(
            Message::ExecuteOpcodeReq {
                opcode: OPCODE_SAVE,
                argument: 0
            }
            .encode(false),
            [0; 7]
        );
        assert_eq!(
            Message::ExecuteOpcodeRes {
                argument: 0,
                ok: true
            }
            .encode(false),
            [0, 0, 0, 0, 0, 0, 0x80]
        );
    }

    /// A name read under tail array optimisation is the rest of the payload; without it, the
    /// length field says.
    #[test]
    fn tail_arrays() {
        let res = GetSetRes {
            name: b"NAME".to_vec(),
            ..GetSetRes::default()
        };
        let classic = Message::GetSetRes(res.clone()).encode(false);
        let fd = Message::GetSetRes(res).encode(true);
        // void5 + tag3 + void5 + tag3 + void6 + tag2 + void6 + tag2 = 32 bits, then the name;
        // FD puts the seven-bit length first.
        assert_eq!(classic.len(), 4 + 4);
        assert_eq!(fd.len(), (32 + 7 + 32usize).div_ceil(8));
        let info = by_name("uavcan_protocol_param_GetSet_res").expect("row");
        let Message::GetSetRes(read) = Message::decode(info, &classic, false) else {
            panic!("a GetSet response");
        };
        assert_eq!(read.name, b"NAME");
    }

    /// The grid's texts.
    #[test]
    fn value_texts() {
        let single = |value: f32| format!("<{value}>");
        assert_eq!(value_text(&Value::Empty, single), "Empty");
        assert_eq!(value_text(&Value::Integer(-3), single), "-3");
        assert_eq!(value_text(&Value::Real(0.5), single), "<0.5>");
        assert_eq!(value_text(&Value::Boolean(1), single), "1");
        assert_eq!(value_text(&Value::String(b"abc".to_vec()), single), "abc");
        assert_eq!(numeric_text(&NumericValue::Empty, single), "");
        assert_eq!(numeric_text(&NumericValue::Integer(7), single), "7");
        assert_eq!(ascii(&[b'a', 0xff]), "a?");
    }
}
