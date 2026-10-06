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

//! Getting files off a vehicle, and onto it.
//!
//! Two ways, side by side because both are the vehicle handing over a file in chunks that arrive
//! out of order:
//!
//! * [`logs`], dataflash log download over `LOG_REQUEST_LIST` and `LOG_DATA` - the transfer half
//!   of `MAVLinkInterface.GetLogList` and `GetLog` (`ExtLibs/ArduPilot/Mavlink/
//!   MAVLinkInterface.cs:5977, 6165`), which `Log/LogDownloadMavLink.cs` drives.
//! * [`mavftp`], the vehicle's file system over `FILE_TRANSFER_PROTOCOL`
//!   (`ExtLibs/ArduPilot/Mavlink/MAVFtp.cs`): listings, burst and plain reads, uploads, removes,
//!   renames, directories and the file CRC.
//!
//! L3, beside the link rather than inside it. The link thread still sends the requests and routes
//! each `LOG_ENTRY`, `LOG_DATA` and `FILE_TRANSFER_PROTOCOL` into the types here; what a chunk
//! means, where it goes and when the file is whole is this crate's.

#![forbid(unsafe_code)]

pub mod logs;
pub mod mavftp;

use mavftp::wire::{Errno, ErrorCode, Opcode};

/// Errors from this crate: what the C#'s MAVFTP commands throw.
///
/// Log download has none - a chunk that does not fit is refused by
/// [`logs::LogDownload::receive`] returning false, and an incomplete download assembles with its
/// holes zero-filled. MAVFTP has the C#'s exceptions, worded as the C# words them, because the
/// wording is what Mission Planner shows the operator (e.g. `CustomMessageBox.Show(exception.
/// Message)`, Controls/MavFTPUI.cs:397).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FtpError {
    /// The vehicle refused a command with `kErrFailErrno`.
    ///
    /// C#: e.g. MAVFtp.cs:671-672, `new Exception("Mavftp responded - " + ftphead.req_opcode + " "
    /// + errorcode + " " + _ftp_errno)`.
    #[error("Mavftp responded - {req_opcode} {error} {errno}")]
    Responded {
        /// Which command the vehicle refused.
        req_opcode: Opcode,
        /// `kErrFailErrno`.
        error: ErrorCode,
        /// Why, as the vehicle's errno.
        errno: Errno,
    },
    /// The same refusal as [`FtpError::Responded`], in the other wording the C# uses for it,
    /// whether or not the command opens a file.
    ///
    /// C#: e.g. MAVFtp.cs:991-992 (`kCmdCalcFileCRC32`), :1324-1325 (`kCmdListDirectory`),
    /// :1783-1784 (`kCmdRemoveFile`).
    #[error("Failed to OpenFile - {req_opcode} {error} {errno}")]
    FailedToOpenFile {
        /// Which command the vehicle refused.
        req_opcode: Opcode,
        /// `kErrFailErrno`.
        error: ErrorCode,
        /// Why, as the vehicle's errno.
        errno: Errno,
    },
    /// The vehicle refused a command with `kErrFail`.
    ///
    /// C#: e.g. MAVFtp.cs:683, `new Exception("Mavftp responded - Err Fail")`.
    #[error("Mavftp responded - Err Fail")]
    ErrFail,
    /// The vehicle said the file is not there: `kErrFileNotFound`.
    ///
    /// C#: e.g. MAVFtp.cs:690, `new FileNotFoundException("File Not Found", file)`.
    #[error("File Not Found")]
    FileNotFound {
        /// The path asked for.
        path: String,
    },
    /// The vehicle opened a file and gave its size as negative - two gigabytes or more, read as
    /// an `int`.
    ///
    /// C#: MAVFtp.cs:752 and :1562, `new MemoryStream(size)`, which throws
    /// `ArgumentOutOfRangeException` for it. The wording is this port's: the C#'s is .NET's.
    #[error("the vehicle gave the file's size as {0}, which is not a size")]
    NegativeSize(i32),
}
