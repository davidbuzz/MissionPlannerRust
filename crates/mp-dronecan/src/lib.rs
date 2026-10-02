//! DroneCAN (UAVCAN v0), as Mission Planner's `ExtLibs/DroneCAN` speaks it for the DroneCAN/UAVCAN
//! page (`GCSViews/ConfigurationView/ConfigDroneCAN.cs`).
//!
//! Ported from `ExtLibs/DroneCAN` @ efb0801 (GPL-3.0-or-later):
//!
//! * [`bits`]: libcanard v0's scalar coding - `Canard.cs`'s `canardEncodeScalar`,
//!   `canardDecodeScalar` and `copyBitArray`, and `dronecan_transmit_chunk_handler`'s bit stream;
//! * [`crc`]: `TransferCRC.cs`, CRC-16-CCITT seeded with a data type's signature;
//! * [`frame`]: `CANFrame.cs` and `CANPayload.cs`, the 29-bit identifier's fields and the tail
//!   byte, with `dlcToDataLength`/`dataLengthToDlc`;
//! * [`transfer`]: `DroneCAN.PackageMessage` (a transfer cut into frames, the CRC in front of a
//!   multi-frame one) and `DroneCAN.ProcessFrame` (frames put back together, toggles and CRC
//!   checked);
//! * [`slcan`]: `ReadMessageSLCAN` and `PackageMessageSLCAN`'s text lines, and the line the page's
//!   MAVLink subscription makes of a `CAN_FRAME`;
//! * [`dsdl`]: the data types the page and its windows use - `uavcan.protocol.NodeStatus`,
//!   `GetNodeInfo`, `RestartNode`, `param.GetSet`, `param.ExecuteOpcode`, `file.Read`,
//!   `file.BeginFirmwareUpdate`, `dynamic_node_id.Allocation`, `debug.LogMessage`,
//!   `dronecan.protocol.Stats` and `CanStats`, `equipment.gnss.RTCMStream`,
//!   `ardupilot.gnss.MovingBaselineData` and `tunnel.Targetted` - each the generated code of
//!   `out/src/*.cs`, by hand;
//! * [`names`]: `canard_dsdlc/messages.cs`'s `MSG_INFO`, every type's name, id and signature, for
//!   telling a frame's type and checking its CRC, and for the Filter window's list;
//! * [`node`]: the `DroneCAN` class - our node 127, its 1 Hz `NodeStatus`, the node list, the
//!   `GetNodeInfo` it answers, the file server and the dynamic node-id allocator;
//! * [`jobs`]: the calls the C# blocks on - `GetParameters`, `SetParameter`, `SaveConfig`,
//!   `RestartNode` and `Update` - as machines a caller drives once a frame;
//! * [`mavlink`]: the page's `CAN_FRAME`/`CANFD_FRAME` mapping, both ways, and the
//!   `CAN_FILTER_MODIFY` the Filter window sends;
//! * [`mcast`]: `StartmcastCAN`'s multicast bus (pydronecan's `mcast.py` framing).
//!
//! What is not here, and why:
//!
//! * the other 140 data types of `out/src`: nothing the page shows decodes them. A frame of one
//!   is still told by [`names::MSG_INFO`] and its CRC checked, and it is handed on as
//!   [`dsdl::Message::Other`] with its bytes. PLAN.md's `dsdlgen`, which would write them all,
//!   is not written;
//! * `FileGetDirectoryEntrys`, `FileRead`, `FileWrite`, `ExecuteOpCode`, `ReadSLCAN`,
//!   `PrintDebugToConsole`, `ServeFile`'s callers other than `Update`, `testFile` and `test`:
//!   nothing on the page or its windows calls them (`DroneCANFileUI`, their caller, is dropped
//!   as having no callers itself);
//! * the blocking threads: the C#'s reader, processor and 1 Hz sender are one [`node::Node`]
//!   a caller feeds lines and frames to and ticks; the waits of the blocking calls are
//!   [`jobs`] polled with the time.

pub mod bits;
pub mod crc;
pub mod dsdl;
pub mod frame;
pub mod jobs;
pub mod mavlink;
pub mod mcast;
pub mod names;
pub mod node;
pub mod slcan;
pub mod transfer;

pub use dsdl::Message;
pub use frame::{Frame, Payload};
pub use node::{Event, Identity, Node, Received};
