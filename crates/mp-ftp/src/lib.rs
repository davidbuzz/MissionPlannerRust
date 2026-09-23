//! Getting files off a vehicle.
//!
//! Today that is one thing: [`logs`], dataflash log download over `LOG_REQUEST_LIST` and
//! `LOG_DATA` - the transfer half of `MAVLinkInterface.GetLogList` and `GetLog`
//! (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5977, 6203`), which
//! `Log/LogDownloadMavLink.cs` drives. MAVFTP (`ExtLibs/ArduPilot/Mavlink/MAVFtp.cs`), which this
//! crate is named for in PLAN.md §5.1, is not ported yet; when it is, it goes here beside
//! [`logs`], because both are the vehicle handing over a file in chunks that arrive out of order.
//!
//! L3, beside the link rather than inside it. The link thread still sends the requests and routes
//! each `LOG_ENTRY` and `LOG_DATA` into the types here; what a chunk means, where it goes and
//! when the file is whole is this crate's.

#![forbid(unsafe_code)]

pub mod logs;

/// Errors from this crate.
///
/// None, and uninhabited so that none can be made up: a chunk that does not fit is refused by
/// [`logs::LogDownload::receive`] returning false, and an incomplete download assembles with its
/// holes zero-filled rather than failing. This is the one enum PLAN.md §5.3 gives each crate, so
/// the first operation that can fail - MAVFTP will have several - has somewhere to say how.
#[derive(Debug, thiserror::Error)]
pub enum FtpError {}
