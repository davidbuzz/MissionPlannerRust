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

//! Selects the gpui platform backend for the current target.
//!
//! This mirrors zed's own `gpui_platform::current_platform`. We reimplement the ~20 lines rather
//! than depending on that crate because, consumed as a git dependency, its
//! `gpui.workspace = true` inheritance resolves against crates.io - where an unrelated crate
//! named `gpui` exists at a much higher version - and the build silently tries to compile it
//! against a stranger's code.
//!
//! Keeping the selection here also keeps every supported target visible in one place:
//!
//! | target | backend | renderer |
//! |---|---|---|
//! | Linux, FreeBSD | `gpui_linux` | wgpu (X11 and Wayland) |
//! | Windows | `gpui_windows` | Direct3D 11 |
//! | macOS | `gpui_macos` | Metal |
//! | wasm | `gpui_web` | wgpu / WebGL |

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::rc::Rc;

use gpui::{Application, Platform};

/// Builds the platform backend for this target.
// A page is started by gpui_web's own entry (main.rs), not through this.
#[cfg_attr(target_family = "wasm", allow(dead_code))]
#[must_use]
pub fn current_platform(headless: bool) -> Rc<dyn Platform> {
    #[cfg(any(target_os = "linux", target_os = "freebsd"))]
    {
        gpui_linux::current_platform(headless)
    }

    #[cfg(target_os = "windows")]
    {
        match gpui_windows::WindowsPlatform::new(headless) {
            Ok(platform) => Rc::new(platform),
            Err(error) => {
                // No window can open without the platform; say why and stop, as the program
                // cannot run on.
                eprintln!("planner: the Windows platform did not initialise: {error:?}");
                std::process::exit(1);
            }
        }
    }

    #[cfg(target_os = "macos")]
    {
        Rc::new(gpui_macos::MacPlatform::new(headless))
    }

    // A web page (the browser experiment): WebGPU, else WebGL2, as gpui_platform chooses.
    #[cfg(target_family = "wasm")]
    {
        let _ = headless;
        Rc::new(gpui_web::WebPlatform::new_with_backend(
            true,
            gpui_web::WebBackendPreference::Auto,
        ))
    }
}

/// An application bound to this target's platform.
#[must_use]
#[cfg(not(target_family = "wasm"))]
pub fn application() -> Application {
    Application::with_platform(current_platform(false))
}

/// An application in a web page, its HTTP through the browser's `fetch`, as gpui_platform's
/// `application_with_web_backend` builds it.
#[must_use]
#[cfg(target_family = "wasm")]
pub fn application() -> Application {
    let platform = Rc::new(gpui_web::WebPlatform::new_with_backend(
        true,
        gpui_web::WebBackendPreference::Auto,
    ));
    let http_client = std::sync::Arc::new(platform.fetch_http_client());
    Application::with_platform(platform).with_http_client(http_client)
}
