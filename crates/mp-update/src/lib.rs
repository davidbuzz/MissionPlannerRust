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

//! The planner's own update: a release's manifest read from GitHub, the build asked what it is,
//! and - on "Update Now" - the release's archive fetched, checked against the manifest's SHA-256,
//! unpacked beside the planner, tried, and moved into place file by file, the old files moved
//! aside and deleted at the next start. One program, no updater beside it.
//!
//! This is not Mission Planner's update (`Utilities/Update.cs`, `Updater/Program.cs`), which was
//! ported here first and replaced on the owner's word of 2026-11-18 ("i dont think i want it to
//! work exactly like the old C# app"): that one hashes some 1,300 loose .NET files with MD5
//! against a `checksums.txt`, downloads the ones that differ and starts `Updater.exe` to move
//! them, from firmware.ardupilot.org and Mission Planner's own GitHub releases. This planner is
//! one executable with its licence files, published by this repository's release workflow.
//! updates.md is the design and its reasons; its section 9 holds what Windows and macOS allow of
//! a running program's file, tested before this was written.
//!
//! What is kept of the C#: the conversation - `CheckForUpdate`'s "Update Found" question with
//! its ChangeLog link, `DoUpdate`'s "Check for Updates" progress dialog with its Cancel, "No
//! update available.", the write test, the once-a-day check at start - and its words, below.
//!
//! Without a window: [`check::check_for_update`] says what a channel offers this build,
//! [`install::install`] does the update, and the planner's Help page puts the questions and the
//! restart around them. In a web page there is nothing to update - the page is whatever the site
//! serves - so only the parts that read ([`identity`], [`manifest`], [`release`]) are built.
//! `// C#: Utilities/Update.cs; Updater/Program.cs; GCSViews/Help.cs; MainV2.cs:3664-3677`

pub mod identity;
pub mod manifest;
pub mod release;

#[cfg(not(target_family = "wasm"))]
pub mod check;
#[cfg(not(target_family = "wasm"))]
pub mod download;
#[cfg(not(target_family = "wasm"))]
pub mod install;
#[cfg(not(target_family = "wasm"))]
pub mod unpack;

/// `Strings.UpdateFound`. `// C#: ExtLibs/Strings/Strings.resx:426`
pub const UPDATE_FOUND: &str = "Update Found";
/// `Strings.UpdateNow`.
pub const UPDATE_NOW: &str = "Update Now";
/// `Strings.UpdateNotFound`.
pub const UPDATE_NOT_FOUND: &str = "No update available.";
/// The progress dialog's caption. `// C#: Update.cs:202`
pub const CHECK_FOR_UPDATES: &str = "Check for Updates";
/// "Update Failed " before the exception's message. `// C#: Update.cs:103, 108`
pub const UPDATE_FAILED: &str = "Update Failed ";
/// What the buttons say of a build that is no release: `cargo build`'s, a developer's.
pub const DEVELOPMENT_BUILD: &str =
    "This is a development build: it is updated from its source, not from a release";
