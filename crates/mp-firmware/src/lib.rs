//! Firmware files and the px4 bootloader protocol.
//!
//! Ported from `ExtLibs/px4uploader/` @ efb0801 (GPL-3.0-or-later). This is the one path in the
//! application with no simulator: a wrong byte here does not produce a wrong reading, it produces a
//! board that will not boot. PLAN.md R9 rates it the only *fatal*-impact risk with no SITL
//! equivalent, and D13 requires the byte protocol to be proven against a mock bootloader before
//! any real board is touched.
//!
//! So the shape of this crate is: everything that can be a pure function is one, the protocol is
//! driven over a `Read + Write` so a test can be the other end of it, and nothing here opens a
//! serial port by itself.
//!
//! Three details are transliterated rather than improved, because the bootloader on the other end
//! is not going to change to suit us. Each is marked where it appears.

pub mod detect;
pub mod firmware;
pub mod manifest;
pub mod protocol;
pub mod uploader;

pub use detect::{Boards, Detected, DeviceInfo, detect_board, match_ports};
pub use firmware::Firmware;
pub use protocol::{Code, Info};
pub use uploader::{Uploader, UploaderError};
