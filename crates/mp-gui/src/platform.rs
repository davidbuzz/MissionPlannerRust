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
#[must_use]
pub fn current_platform(headless: bool) -> Rc<dyn Platform> {
    #[cfg(any(target_os = "linux", target_os = "freebsd"))]
    {
        gpui_linux::current_platform(headless)
    }

    #[cfg(target_os = "windows")]
    {
        Rc::new(
            gpui_windows::WindowsPlatform::new(headless)
                .expect("failed to initialise the Windows platform"),
        )
    }

    #[cfg(target_os = "macos")]
    {
        Rc::new(gpui_macos::MacPlatform::new(headless))
    }
}

/// An application bound to this target's platform.
#[must_use]
pub fn application() -> Application {
    Application::with_platform(current_platform(false))
}
