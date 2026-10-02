//! The MAVLink side of `StartMavlinkCAN`: the vehicle forwards its CAN bus as `CAN_FRAME`s once
//! asked with `MAV_CMD_CAN_FORWARD`, and takes frames to put on the bus the same way.
//!
//! Both directions go through SLCAN text, as the C#'s do: a `CAN_FRAME` heard becomes the line
//! the subscription writes into the node's port ([`crate::slcan::from_can_frame`]), and a line the
//! node writes is read by a second `DroneCAN` object, whose `FrameReceived` makes the
//! `CAN_FRAME` (or `CANFD_FRAME` for more than eight bytes) - the identifier with bit 31 set for
//! an extended frame, as pydronecan's `mavcan` driver sends it, on bus `bus - 1`.
//! `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:88-202`

use mp_mavlink_dialects::all::{
    CanFilterModify, CanFilterOp, CanFrame, CanfdFrame, CommandLong, MavCmd, MavMessage,
};

use crate::frame::data_length_to_dlc;
use crate::slcan::{self, Line};

/// `CAN_FRAME`'s message id, for a subscription that listens for it alone.
pub const CAN_FRAME_ID: u32 = 386;

/// `doCommand(sysid, compid, MAV_CMD.CAN_FORWARD, bus, 0, 0, 0, 0, 0, 0, false)`: forward bus
/// `bus` (1 or 2), sent every second without waiting for an answer.
#[must_use]
pub const fn can_forward(target_system: u8, target_component: u8, bus: u8) -> MavMessage {
    MavMessage::CommandLong(CommandLong {
        param1: bus as f32,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MAV_CMD_CAN_FORWARD,
        target_system,
        target_component,
        confirmation: 0,
    })
}

/// `MAV_CMD_CAN_FORWARD`.
pub const MAV_CMD_CAN_FORWARD: u16 = 32000;

/// The `MAV_CMD` constant the dialect has, checked against [`MAV_CMD_CAN_FORWARD`] by a test.
#[must_use]
pub const fn dialect_can_forward() -> MavCmd {
    MavCmd::MAV_CMD_CAN_FORWARD
}

/// The line the subscription writes for a `CAN_FRAME` heard.
#[must_use]
pub fn line_of(frame: &CanFrame) -> String {
    slcan::from_can_frame(frame.id, frame.len, &frame.data)
}

/// A line the node wrote, as `FrameReceived` sends it to the vehicle: `None` for what
/// `ReadMessageSLCAN` drops (an empty frame, a command).
#[must_use]
pub fn message_of(
    line: &str,
    target_system: u8,
    target_component: u8,
    bus: u8,
) -> Option<MavMessage> {
    let Line::Frame(frame, _, payload) = slcan::read_line(line) else {
        return None;
    };
    let id = frame
        .id()
        .wrapping_add(if frame.extended { 0x8000_0000 } else { 0 });
    let len = data_length_to_dlc(payload.bytes.len());
    let bus = bus.wrapping_sub(1);
    if payload.bytes.len() > 8 {
        let mut data = [0u8; 64];
        for (slot, byte) in data.iter_mut().zip(&payload.bytes) {
            *slot = *byte;
        }
        return Some(MavMessage::CanfdFrame(CanfdFrame {
            id,
            target_system,
            target_component,
            bus,
            len,
            data,
        }));
    }
    let mut data = [0u8; 8];
    for (slot, byte) in data.iter_mut().zip(&payload.bytes) {
        *slot = *byte;
    }
    Some(MavMessage::CanFrame(CanFrame {
        id,
        target_system,
        target_component,
        bus,
        len,
        data,
    }))
}

/// The Filter window's `CAN_FILTER_MODIFY`: the ids made sixteen long (`MakeSize(16)`), replacing
/// the vehicle's filter on `bus` - `BusInUse` as it is, 1 or 2, where the frames go on `bus - 1`.
/// `num_ids` is what the C# passes: the list's count for a message's box, 0 for "ALL".
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:924-944, 952-983`
#[must_use]
pub fn filter_modify(
    ids: &[u16],
    target_system: u8,
    target_component: u8,
    bus: u8,
    num_ids: u8,
) -> MavMessage {
    let mut sixteen = [0u16; 16];
    for (slot, id) in sixteen.iter_mut().zip(ids) {
        *slot = *id;
    }
    MavMessage::CanFilterModify(CanFilterModify {
        ids: sixteen,
        target_system,
        target_component,
        bus,
        operation: CanFilterOp::CAN_FILTER_REPLACE.0.to_le_bytes()[0],
        num_ids,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsdl::NodeStatus;
    use crate::node::{Identity, Node};
    use std::time::Instant;

    /// The node's status, through `FrameReceived`, is a `CAN_FRAME` with the extended flag, on
    /// bus 0 for bus 1; and the vehicle's `CAN_FRAME` back becomes a line our node reads.
    #[test]
    fn frames_both_ways() {
        let now = Instant::now();
        let mut node = Node::new(Identity::default(), now);
        node.start(now);
        node.tick(now, 1);
        let lines = node.take_outgoing();
        assert_eq!(lines.len(), 1);
        let Some(MavMessage::CanFrame(out)) = message_of(&lines[0], 1, 1, 1) else {
            panic!("a CAN_FRAME");
        };
        assert_eq!(out.id, 0x8000_0000 | 0x1E01_557F);
        assert_eq!(out.bus, 0);
        assert_eq!(out.len, 8);
        assert_eq!(
            out.data[7], 0xC0,
            "the tail byte: start and end, transfer 0"
        );

        // The same frame, as if from node 10 - read by a node of our own.
        let mut other = Node::new(Identity::default(), now);
        other.start(now);
        let mut heard = out;
        heard.id = (heard.id & !0x7f) | 10;
        other.receive_line(&line_of(&heard));
        let received = other.next_received().expect("a status");
        assert_eq!(received.frame.source_node(), 10);
        assert!(matches!(
            received.message,
            crate::Message::NodeStatus(NodeStatus { .. })
        ));
        assert_eq!(message_of("C\r", 1, 1, 1), None);
    }

    /// The command and the filter, as the C# fills them.
    #[test]
    fn command_and_filter() {
        assert_eq!(dialect_can_forward().0, u32::from(MAV_CMD_CAN_FORWARD));
        let MavMessage::CommandLong(command) = can_forward(1, 1, 2) else {
            panic!("a command");
        };
        assert_eq!(command.command, 32000);
        assert!((command.param1 - 2.0).abs() < f32::EPSILON);
        let MavMessage::CanFilterModify(filter) = filter_modify(&[0, 341, 1], 1, 1, 2, 3) else {
            panic!("a filter");
        };
        assert_eq!(filter.ids[..4], [0, 341, 1, 0]);
        assert_eq!(filter.bus, 2);
        assert_eq!(filter.num_ids, 3);
        assert_eq!(filter.operation, 0);
    }
}
