//! A CAN frame's identifier and its data, as `CANFrame` and `CANPayload` read them.
//!
//! The identifier is held as the C# holds it, the four bytes `BitConverter.GetBytes(id)` makes -
//! so an identifier with the extended-frame flag in bit 31, as ArduPilot forwards one, keeps it,
//! and the priority reads past it (`packet_data[3] & 0x1f`). The tail byte is the payload's last.
//! `// C#: ExtLibs/DroneCAN/CANFrame.cs:1-134; ExtLibs/DroneCAN/CANPayload.cs:1-81`

/// `CANFrame.FrameType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferType {
    /// Source node 0: a node without an id yet.
    Anonymous,
    /// A request or a response.
    Service,
    /// A broadcast.
    Message,
}

/// `CANFrame`: the identifier's bytes, and whether the frame was extended and CAN FD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// `packet_data`: the identifier, little-endian.
    pub bytes: [u8; 4],
    /// `Extended`. Every frame the C# makes is, and `ReadMessageSLCAN` says so of every line.
    pub extended: bool,
    /// `FDCan`.
    pub fd: bool,
    /// `SizeofEntireMsg`: the received transfer's payload bytes, CRC excluded; zero on a frame
    /// that was not received.
    pub size_of_entire_msg: usize,
}

impl Frame {
    /// `new CANFrame(BitConverter.GetBytes(id), extended, fdcan)`.
    #[must_use]
    pub const fn from_id(id: u32, extended: bool, fd: bool) -> Self {
        Self {
            bytes: id.to_le_bytes(),
            extended,
            fd,
            size_of_entire_msg: 0,
        }
    }

    /// `BitConverter.ToUInt32(packet_data, 0)`.
    #[must_use]
    pub const fn id(&self) -> u32 {
        u32::from_le_bytes(self.bytes)
    }

    /// `TransferType`.
    #[must_use]
    pub const fn transfer_type(&self) -> TransferType {
        if self.source_node() == 0 {
            TransferType::Anonymous
        } else if self.is_service() {
            TransferType::Service
        } else {
            TransferType::Message
        }
    }

    /// `SourceNode`, 0 to 127.
    #[must_use]
    pub const fn source_node(&self) -> u8 {
        self.bytes[0] & 0x7f
    }

    /// `SourceNode = value`.
    pub const fn set_source_node(&mut self, value: u8) {
        self.bytes[0] = (self.bytes[0] & !0x7f) | (value & 0x7f);
    }

    /// `IsServiceMsg`.
    #[must_use]
    pub const fn is_service(&self) -> bool {
        self.bytes[0] & 0x80 != 0
    }

    /// `IsServiceMsg = value`.
    pub const fn set_service(&mut self, value: bool) {
        self.bytes[0] = (self.bytes[0] & !0x80) | if value { 0x80 } else { 0 };
    }

    /// The identifier's message-type bytes, `BitConverter.ToUInt16(packet_data, 1)`.
    const fn type_bytes(&self) -> u16 {
        u16::from_le_bytes([self.bytes[1], self.bytes[2]])
    }

    /// `MsgTypeID`: the low two bits for an anonymous frame, the sixteen for a message, the
    /// service type for a service.
    #[must_use]
    pub const fn msg_type_id(&self) -> u16 {
        match self.transfer_type() {
            TransferType::Anonymous => self.type_bytes() & 0x3,
            TransferType::Message => self.type_bytes(),
            TransferType::Service => self.svc_type_id() as u16,
        }
    }

    /// `MsgTypeID = value`.
    pub const fn set_msg_type_id(&mut self, value: u16) {
        let [low, high] = value.to_le_bytes();
        self.bytes[1] = low;
        self.bytes[2] = high;
    }

    /// `Priority`, 0 (highest) to 31.
    #[must_use]
    pub const fn priority(&self) -> u8 {
        self.bytes[3] & 0x1f
    }

    /// `Priority = value`.
    pub const fn set_priority(&mut self, value: u8) {
        self.bytes[3] = (self.bytes[3] & !0x1f) | (value & 0x1f);
    }

    /// `AnonDiscriminator`.
    #[must_use]
    pub const fn anon_discriminator(&self) -> u16 {
        self.type_bytes() >> 2
    }

    /// `SvcDestinationNode`.
    #[must_use]
    pub const fn svc_destination_node(&self) -> u8 {
        self.bytes[1] & 0x7f
    }

    /// `SvcDestinationNode = value`.
    pub const fn set_svc_destination_node(&mut self, value: u8) {
        self.bytes[1] = (self.bytes[1] & !0x7f) | (value & 0x7f);
    }

    /// `SvcIsRequest`.
    #[must_use]
    pub const fn svc_is_request(&self) -> bool {
        self.bytes[1] & 0x80 != 0
    }

    /// `SvcIsRequest = value`.
    pub const fn set_svc_is_request(&mut self, value: bool) {
        self.bytes[1] = (self.bytes[1] & !0x80) | if value { 0x80 } else { 0 };
    }

    /// `SvcTypeID`: the service type of a service frame, else 0.
    #[must_use]
    pub const fn svc_type_id(&self) -> u8 {
        match self.transfer_type() {
            TransferType::Service => self.bytes[2],
            _ => 0,
        }
    }

    /// `SvcTypeID = value`.
    pub const fn set_svc_type_id(&mut self, value: u8) {
        self.bytes[2] = value;
    }

    /// `ToHex()`: the four bytes, high first, two capital hex digits each.
    #[must_use]
    pub fn to_hex(&self) -> String {
        format!("{:08X}", self.id())
    }
}

/// `CANPayload`: a frame's data, the tail byte last.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Payload {
    /// `packet_data`.
    pub bytes: Vec<u8>,
}

impl Payload {
    /// `new CANPayload(packet_data)`.
    #[must_use]
    pub const fn new(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }

    /// The tail byte; a payload has one by the time it is made, and an empty one reads as 0.
    fn tail(&self) -> u8 {
        self.bytes.last().copied().unwrap_or(0)
    }

    /// Sets the tail byte's bits under `mask` to `bits`.
    fn set_tail(&mut self, mask: u8, bits: u8) {
        if let Some(tail) = self.bytes.last_mut() {
            *tail = (*tail & !mask) | (bits & mask);
        }
    }

    /// `TransferID`, 0 to 31.
    #[must_use]
    pub fn transfer_id(&self) -> u8 {
        self.tail() & 0x1f
    }

    /// `TransferID = value`.
    pub fn set_transfer_id(&mut self, value: u8) {
        self.set_tail(0x1f, value);
    }

    /// `Toggle`.
    #[must_use]
    pub fn toggle(&self) -> bool {
        self.tail() & 0x20 != 0
    }

    /// `Toggle = value`.
    pub fn set_toggle(&mut self, value: bool) {
        self.set_tail(0x20, if value { 0x20 } else { 0 });
    }

    /// `EOT`: end of transfer.
    #[must_use]
    pub fn eot(&self) -> bool {
        self.tail() & 0x40 != 0
    }

    /// `EOT = value`.
    pub fn set_eot(&mut self, value: bool) {
        self.set_tail(0x40, if value { 0x40 } else { 0 });
    }

    /// `SOT`: start of transfer.
    #[must_use]
    pub fn sot(&self) -> bool {
        self.tail() & 0x80 != 0
    }

    /// `SOT = value`.
    pub fn set_sot(&mut self, value: bool) {
        self.set_tail(0x80, if value { 0x80 } else { 0 });
    }

    /// `Payload`: the data before the tail byte.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        self.bytes.split_last().map_or(&[][..], |(_, data)| data)
    }

    /// `ToHex(totallength)`: every byte as two capital hex digits, padded on the right with `0`
    /// to `totallength` bytes' worth.
    #[must_use]
    pub fn to_hex(&self, total_length: usize) -> String {
        use std::fmt::Write as _;
        let mut text = String::with_capacity(self.bytes.len() * 2);
        for byte in &self.bytes {
            let _ = write!(text, "{byte:02X}");
        }
        while text.len() < total_length * 2 {
            text.push('0');
        }
        text
    }
}

/// `dlcToDataLength`: a data length code's bytes - itself to 8, then 12, 16, 20, 24, 32, 48, 64.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1696-1730`
#[must_use]
pub const fn dlc_to_data_length(dlc: u8) -> u8 {
    match dlc {
        0..=8 => dlc,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        _ => 64,
    }
}

/// `dataLengthToDlc`: the smallest code that holds `data_length` bytes.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1732-1763`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // the arm holds it to 0..=8
pub const fn data_length_to_dlc(data_length: usize) -> u8 {
    match data_length {
        0..=8 => data_length as u8,
        9..=12 => 9,
        13..=16 => 10,
        17..=20 => 11,
        21..=24 => 12,
        25..=32 => 13,
        33..=48 => 14,
        _ => 15,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The identifiers `SaveConfig`'s comment records - our node 126 asking node 121 to save,
    /// and 121's answer - read as the C# reads them.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:432-433`
    #[test]
    fn the_execute_opcode_frames_read_back() {
        let request = Frame::from_id(0x1E0A_F9FE, true, false);
        assert_eq!(request.transfer_type(), TransferType::Service);
        assert_eq!(request.source_node(), 126);
        assert_eq!(request.svc_destination_node(), 121);
        assert!(request.svc_is_request());
        assert_eq!(request.svc_type_id(), 10);
        assert_eq!(request.msg_type_id(), 10);
        assert_eq!(request.priority(), 30);
        assert_eq!(request.to_hex(), "1E0AF9FE");

        let response = Frame::from_id(0x1E0A_7EF9, true, false);
        assert_eq!(response.source_node(), 121);
        assert_eq!(response.svc_destination_node(), 126);
        assert!(!response.svc_is_request());

        let tail = Payload::new(vec![0, 0, 0, 0, 0, 0, 0x80, 0xC1]);
        assert!(tail.sot() && tail.eot() && !tail.toggle());
        assert_eq!(tail.transfer_id(), 1);
        assert_eq!(tail.data().len(), 7);
    }

    /// Building one: the setters touch only their own bits, and a forwarded identifier's
    /// extended flag (bit 31) does not reach the priority.
    #[test]
    fn setters_and_the_extended_flag() {
        let mut frame = Frame::from_id(0, true, false);
        frame.set_source_node(127);
        frame.set_priority(30);
        frame.set_msg_type_id(341);
        assert_eq!(frame.transfer_type(), TransferType::Message);
        assert_eq!(frame.msg_type_id(), 341);
        assert_eq!(frame.id(), 0x1E01_557F);
        let forwarded = Frame::from_id(frame.id() | 0x8000_0000, true, false);
        assert_eq!(forwarded.priority(), 30);
        assert_eq!(forwarded.msg_type_id(), 341);

        let anonymous = Frame::from_id(0x1E00_0500 | (0x2a << 10), true, false);
        assert_eq!(anonymous.transfer_type(), TransferType::Anonymous);
        assert_eq!(anonymous.msg_type_id(), 1);

        let mut payload = Payload::new(vec![1, 2, 0]);
        payload.set_sot(true);
        payload.set_toggle(true);
        payload.set_transfer_id(33);
        assert_eq!(payload.bytes, [1, 2, 0b1010_0001]);
        assert_eq!(payload.to_hex(4), "0102A100");
    }

    /// The data length codes, both ways.
    #[test]
    fn data_length_codes() {
        for dlc in 0..=15u8 {
            let length = dlc_to_data_length(dlc);
            assert_eq!(data_length_to_dlc(usize::from(length)), dlc);
        }
        assert_eq!(data_length_to_dlc(9), 9);
        assert_eq!(data_length_to_dlc(63), 15);
    }
}
