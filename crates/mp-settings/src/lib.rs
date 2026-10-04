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

//! Where Mission Planner keeps things on disk.
//!
//! Ported from `ExtLibs/Utilities/Settings.cs` (GPL-3.0-only): the directory rules only,
//! because they are what every file this application keeps hangs off.
//!
//! The rules are the C#'s; the name is not. The C# calls its directory `Mission Planner`
//! (`Settings.AppConfigName`), and this application calls its own [`APP_CONFIG_NAME`],
//! `MissionPlannerRust` - the owner's ruling of 2026-09-25 (PLAN.md section 12, D11): a user
//! running both applications must not lose data to the other one's writes. The formats stay the
//! C#'s, so a file copied from one directory to the other reads the same; [`migrate`] copies the
//! C#'s files into this application's directory once, on the first start that finds it empty, and
//! leaves the C#'s copies as they were.
//!
//! The rules are not what a Linux user would guess. `GetUserDataDirectory` asks .NET for
//! `MyDocuments`, and under mono that is `$HOME`, not `~/Documents` - so the C# application on
//! Linux looks for `~/Mission Planner`, does not find it, and settles in
//! `~/.local/share/Mission Planner`. That is where a real installation on this machine keeps its
//! logs, its parameter metadata and its map cache. This application settles in
//! `~/.local/share/MissionPlannerRust` (`$XDG_DATA_HOME/MissionPlannerRust`) on Linux, and never
//! in `~/MissionPlannerRust`: see [`Folders::user_data_directory`] for why its rule departs from
//! the C#'s there. Measured by asking mono itself (`Environment.GetFolderPath(SpecialFolder.MyDocuments)` printed
//! the home directory), not inferred from the enum's name - which is how this crate's predecessor
//! came to record flights under `~/Documents/Mission Planner/logs`, a directory the C#
//! application never reads.

pub mod config;
pub mod migrate;

pub use config::{Config, ConfigError};

use std::path::{Path, PathBuf};

/// This application's `Settings.AppConfigName`: the directory name under every base the C# rules
/// use. Not the C#'s `Mission Planner` ([`CSHARP_APP_CONFIG_NAME`]): the owner's ruling of
/// 2026-09-25 (PLAN.md section 12, D11) gives this application a directory of its own.
/// `// C#: ExtLibs/Utilities/Settings.cs:20`
pub const APP_CONFIG_NAME: &str = "MissionPlannerRust";

/// The C#'s `Settings.AppConfigName`, with the space: the directory [`migrate`] imports from, and
/// never writes to.
/// `// C#: ExtLibs/Utilities/Settings.cs:20`
pub const CSHARP_APP_CONFIG_NAME: &str = "Mission Planner";

/// The special folders the C# rules are written in terms of, resolved once.
///
/// Every rule is a pure function of these, so the rules can be tested against directories a test
/// made up rather than against the environment - which is `unsafe` to change in this edition and
/// denied by the workspace.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Folders {
    /// `Environment.SpecialFolder.MyDocuments`. `$HOME` under mono; `Documents` on Windows.
    pub my_documents: PathBuf,
    /// `Environment.SpecialFolder.LocalApplicationData`. `~/.local/share` under mono;
    /// `%LOCALAPPDATA%` on Windows.
    pub local_application_data: PathBuf,
    /// `Environment.SpecialFolder.CommonApplicationData`. `%ProgramData%` on Windows. Under mono
    /// it is `/usr/share`, and the C# never uses it there.
    pub common_application_data: PathBuf,
    /// Whether the rules take their mono branch.
    ///
    /// The C# tests two things, `isMono()` and `OSVersion.Platform == Unix`, in different
    /// functions. They agree on every platform this builds for - both false on Windows, both true
    /// everywhere else, because everywhere else the C# application runs under mono - so one flag
    /// stands for both.
    pub unix: bool,
}

impl Folders {
    /// The folders for this process, or `None` if there is no home directory to hang them off.
    ///
    /// On Windows `MyDocuments` is taken as `%USERPROFILE%\Documents`. The shell can redirect the
    /// real folder elsewhere (a OneDrive-managed Documents, for instance), and .NET follows that
    /// redirection where this does not. Reading the known-folder registry is owed when a Windows
    /// user reports the difference; it is not worth a platform crate before one does.
    #[must_use]
    pub fn from_environment() -> Option<Self> {
        if cfg!(windows) {
            let profile = std::env::var_os("USERPROFILE").map(PathBuf::from)?;
            Some(Self {
                my_documents: profile.join("Documents"),
                local_application_data: std::env::var_os("LOCALAPPDATA")
                    .map_or_else(|| profile.join("AppData").join("Local"), PathBuf::from),
                common_application_data: std::env::var_os("ProgramData")
                    .map_or_else(|| PathBuf::from(r"C:\ProgramData"), PathBuf::from),
                unix: false,
            })
        } else {
            let home = std::env::var_os("HOME").map(PathBuf::from)?;
            Some(Self {
                // mono: MyDocuments and Personal are the same folder, and it is $HOME.
                my_documents: home.clone(),
                // mono: XDG_DATA_HOME, defaulting to ~/.local/share.
                local_application_data: std::env::var_os("XDG_DATA_HOME")
                    .map_or_else(|| home.join(".local").join("share"), PathBuf::from),
                common_application_data: PathBuf::from("/usr/share"),
                unix: true,
            })
        }
    }

    /// `Settings.GetUserDataDirectory`: user-specific data, this application's.
    ///
    /// On unix, local application data - `$XDG_DATA_HOME/MissionPlannerRust`, by default
    /// `~/.local/share/MissionPlannerRust` - always; on Windows, `Documents\MissionPlannerRust`.
    ///
    /// A departure from the C#, by the owner's ruling of 2026-09-25 (PLAN.md section 12, D11).
    /// The C# first tries `MyDocuments/<AppConfigName>` and keeps to it if that directory exists
    /// ("do not migrate to new approach if directory exists"), and mono maps `MyDocuments` to
    /// `$HOME`. Under this application's name that is `~/MissionPlannerRust` - which is where this
    /// repository's checkout lives, so the rule would put config, logs and tiles into the source
    /// tree. The check is dropped for this application's directories on unix; the C#'s own keep
    /// it ([`Folders::csharp_user_data_directory`]), because the import must find
    /// `~/Mission Planner` where mono put it.
    /// `// C#: ExtLibs/Utilities/Settings.cs:340-365`
    #[must_use]
    pub fn user_data_directory(&self) -> PathBuf {
        if self.unix {
            // Not `MyDocuments/<name>` first, as the C# has it: PLAN.md section 12, D11.
            self.local_application_data.join(APP_CONFIG_NAME)
        } else {
            self.my_documents.join(APP_CONFIG_NAME)
        }
    }

    /// The C#'s `Settings.GetUserDataDirectory`: the same rule under the C#'s name, which is
    /// where [`migrate`] finds the files it imports.
    /// `// C#: ExtLibs/Utilities/Settings.cs:344-363`
    #[must_use]
    pub fn csharp_user_data_directory(&self) -> PathBuf {
        self.csharp_user_data_directory_named(CSHARP_APP_CONFIG_NAME)
    }

    /// `GetUserDataDirectory` as the C# has it, old-approach test and all, for a given
    /// `AppConfigName`. Only the C#'s own directories are resolved this way.
    /// `// C#: ExtLibs/Utilities/Settings.cs:344-363`
    fn csharp_user_data_directory_named(&self, name: &str) -> PathBuf {
        let old_approach = self.my_documents.join(name);
        if self.unix && !old_approach.is_dir() {
            self.local_application_data.join(name)
        } else {
            old_approach
        }
    }

    /// `Settings.GetDataDirectory`: data shared between users of a machine, this application's.
    ///
    /// Only Windows actually shares it - `%ProgramData%\MissionPlannerRust`. Under mono it is the
    /// user data directory, so the map cache and the logs sit side by side.
    /// `// C#: ExtLibs/Utilities/Settings.cs:325-336`
    #[must_use]
    pub fn data_directory(&self) -> PathBuf {
        if self.unix {
            self.user_data_directory()
        } else {
            self.common_application_data.join(APP_CONFIG_NAME)
        }
    }

    /// The C#'s `Settings.GetDataDirectory`: the same rule under the C#'s name.
    /// `// C#: ExtLibs/Utilities/Settings.cs:325-336`
    #[must_use]
    pub fn csharp_data_directory(&self) -> PathBuf {
        if self.unix {
            self.csharp_user_data_directory()
        } else {
            self.common_application_data.join(CSHARP_APP_CONFIG_NAME)
        }
    }

    /// `Settings.GetDefaultLogDir`: where flights are recorded when nothing says otherwise.
    ///
    /// The C# also creates it here. That is left to the caller, which knows whether it is about
    /// to write anything - a directory created by a function whose name says it only answers a
    /// question is the kind of side effect a test trips over.
    /// `// C#: ExtLibs/Utilities/Settings.cs:146-158`
    #[must_use]
    pub fn default_log_directory(&self) -> PathBuf {
        self.user_data_directory().join("logs")
    }

    /// `MyImageCache.CacheLocation`: where map tiles are cached.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:27-28`
    #[must_use]
    pub fn map_cache_directory(&self) -> PathBuf {
        self.data_directory().join("gmapcache")
    }

    /// The C#'s `MyImageCache.CacheLocation`: its map tiles, which [`migrate`] leaves where they
    /// are. Read by the tests that hold this application's cache layout to tiles the C# wrote.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:27-28`
    #[must_use]
    pub fn csharp_map_cache_directory(&self) -> PathBuf {
        self.csharp_data_directory().join("gmapcache")
    }
}

/// The shared data directory for this process, if there is a home directory to derive it from.
#[must_use]
pub fn data_directory() -> Option<PathBuf> {
    Folders::from_environment().map(|folders| folders.data_directory())
}

/// The user data directory for this process, if there is a home directory to derive it from.
#[must_use]
pub fn user_data_directory() -> Option<PathBuf> {
    Folders::from_environment().map(|folders| folders.user_data_directory())
}

/// Where flights are recorded by default, if there is a home directory to derive it from.
#[must_use]
pub fn default_log_directory() -> Option<PathBuf> {
    Folders::from_environment().map(|folders| folders.default_log_directory())
}

/// Where map tiles are cached, if there is a home directory to derive it from.
#[must_use]
pub fn map_cache_directory() -> Option<PathBuf> {
    Folders::from_environment().map(|folders| folders.map_cache_directory())
}

/// Whether a path is somewhere under another, without touching the filesystem.
#[must_use]
pub fn is_under(path: &Path, base: &Path) -> bool {
    path.starts_with(base)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A temporary directory that removes itself, so a test that creates the "old approach"
    /// directory leaves nothing behind for the next one to find.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = mp_os::temp_dir().join(format!(
                "mp-settings-{name}-{}-{:?}",
                mp_os::process_id(),
                wasm_thread::current().id()
            ));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).ok();
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn mono(scratch: &Scratch) -> Folders {
        Folders {
            my_documents: scratch.0.join("home"),
            local_application_data: scratch.0.join("home").join(".local").join("share"),
            common_application_data: PathBuf::from("/usr/share"),
            unix: true,
        }
    }

    fn windows(scratch: &Scratch) -> Folders {
        Folders {
            my_documents: scratch.0.join("Users").join("pilot").join("Documents"),
            local_application_data: scratch
                .0
                .join("Users")
                .join("pilot")
                .join("AppData")
                .join("Local"),
            common_application_data: scratch.0.join("ProgramData"),
            unix: false,
        }
    }

    #[test]
    fn on_linux_a_fresh_installation_lives_under_local_share() {
        // The C#'s rule is not what anyone would guess: MyDocuments is $HOME under mono, so its
        // "old approach" path is ~/Mission Planner, and a machine that never had one gets
        // ~/.local/share/Mission Planner - where the real C# installation on the machine this
        // was written on keeps its logs and its map cache. This application is always beside it,
        // in ~/.local/share/MissionPlannerRust.
        let scratch = Scratch::new("fresh-linux");
        let folders = mono(&scratch);
        assert_eq!(
            folders.user_data_directory(),
            scratch.0.join("home/.local/share/MissionPlannerRust")
        );
        assert_eq!(folders.data_directory(), folders.user_data_directory());
        assert_eq!(
            folders.csharp_user_data_directory(),
            scratch.0.join("home/.local/share/Mission Planner")
        );
        assert_eq!(
            folders.csharp_data_directory(),
            folders.csharp_user_data_directory()
        );
    }

    #[test]
    fn on_linux_an_old_csharp_installation_is_found_where_mono_put_it() {
        // "Do not use new AppData path if old path already exists" - the C# comment. A decade
        // of logs in ~/Mission Planner stays there, and the import looks for it there.
        let scratch = Scratch::new("old-linux");
        let folders = mono(&scratch);
        let old = folders.my_documents.join(CSHARP_APP_CONFIG_NAME);
        std::fs::create_dir_all(&old).expect("create the old directory");
        assert_eq!(folders.csharp_user_data_directory(), old);
        assert_eq!(folders.csharp_data_directory(), old);
        // And this application stays where it always is.
        assert_eq!(
            folders.user_data_directory(),
            folders.local_application_data.join(APP_CONFIG_NAME)
        );
    }

    #[test]
    fn on_linux_a_mission_planner_rust_directory_in_home_changes_nothing() {
        // ~/MissionPlannerRust is this repository's checkout on the machine it is written on.
        // The C#'s old-approach rule would settle there; this application never does (PLAN.md
        // section 12, D11).
        let scratch = Scratch::new("checkout-in-home");
        let folders = mono(&scratch);
        let checkout = folders.my_documents.join(APP_CONFIG_NAME);
        std::fs::create_dir_all(checkout.join("crates")).expect("a checkout");
        std::fs::write(checkout.join("Cargo.toml"), b"[workspace]").expect("a manifest");
        let ours = folders.local_application_data.join(APP_CONFIG_NAME);
        assert_eq!(folders.user_data_directory(), ours);
        assert_eq!(folders.data_directory(), ours);
        assert_eq!(folders.default_log_directory(), ours.join("logs"));
        assert_eq!(folders.map_cache_directory(), ours.join("gmapcache"));
        assert_eq!(
            migrate::Directories::of(&folders).to_user,
            ours,
            "the import writes there too"
        );
    }

    #[test]
    fn on_linux_a_file_named_like_the_csharps_old_directory_does_not_count() {
        // Directory.Exists is false for a file.
        let scratch = Scratch::new("file-not-dir");
        let folders = mono(&scratch);
        std::fs::create_dir_all(&folders.my_documents).expect("home");
        std::fs::write(folders.my_documents.join(CSHARP_APP_CONFIG_NAME), b"").expect("a file");
        assert!(
            folders
                .csharp_user_data_directory()
                .starts_with(&folders.local_application_data)
        );
    }

    #[test]
    fn on_windows_user_data_is_under_documents_whether_or_not_it_exists() {
        let scratch = Scratch::new("windows-user");
        let folders = windows(&scratch);
        assert_eq!(
            folders.user_data_directory(),
            folders.my_documents.join("MissionPlannerRust")
        );
        assert_eq!(
            folders.csharp_user_data_directory(),
            folders.my_documents.join("Mission Planner")
        );
    }

    #[test]
    fn on_windows_shared_data_is_under_program_data() {
        // Which is where the C# keeps gmapcache on Windows: C:\ProgramData\Mission Planner; and
        // this application, under its own name, C:\ProgramData\MissionPlannerRust.
        let scratch = Scratch::new("windows-shared");
        let folders = windows(&scratch);
        assert_eq!(
            folders.data_directory(),
            scratch.0.join("ProgramData").join("MissionPlannerRust")
        );
        assert_eq!(
            folders.map_cache_directory(),
            scratch.0.join("ProgramData/MissionPlannerRust/gmapcache")
        );
        assert_eq!(
            folders.csharp_data_directory(),
            scratch.0.join("ProgramData").join("Mission Planner")
        );
    }

    #[test]
    fn logs_are_a_directory_called_logs_under_the_user_data() {
        let scratch = Scratch::new("logs");
        for folders in [mono(&scratch), windows(&scratch)] {
            assert_eq!(
                folders.default_log_directory(),
                folders.user_data_directory().join("logs")
            );
        }
    }

    #[test]
    fn the_directory_names_are_the_owners_and_the_csharps() {
        // Ours is the owner's name for it (PLAN.md section 12, D11). The C#'s is the one with the
        // space in it: "MissionPlanner", "mission-planner" and "mission_planner" all look
        // reasonable and are all a different directory from the one the C# application uses, and
        // an import from any of them would find nothing.
        assert_eq!(APP_CONFIG_NAME, "MissionPlannerRust");
        assert_eq!(CSHARP_APP_CONFIG_NAME, "Mission Planner");
    }

    #[test]
    fn the_map_cache_hangs_off_the_shared_data_directory() {
        let scratch = Scratch::new("cache");
        let folders = mono(&scratch);
        assert_eq!(
            folders.map_cache_directory(),
            folders.data_directory().join("gmapcache")
        );
        assert_eq!(
            folders.csharp_map_cache_directory(),
            scratch
                .0
                .join("home/.local/share/Mission Planner/gmapcache")
        );
    }

    #[test]
    fn this_process_resolves_somewhere_named_after_the_application() {
        // The one test that reads the real environment. It cannot know which branch applies on
        // the machine running it, only that both branches end in the same place.
        let Some(directory) = data_directory() else {
            eprintln!("skipped: no home directory in this environment");
            return;
        };
        assert!(
            directory.ends_with(APP_CONFIG_NAME),
            "{}",
            directory.display()
        );
        assert!(is_under(
            &map_cache_directory().expect("same environment"),
            &directory
        ));
        assert!(
            default_log_directory()
                .expect("same environment")
                .ends_with("MissionPlannerRust/logs"),
            "{:?}",
            default_log_directory()
        );
    }
}
