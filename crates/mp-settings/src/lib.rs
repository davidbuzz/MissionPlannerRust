//! Where Mission Planner keeps things on disk.
//!
//! Ported from `ExtLibs/Utilities/Settings.cs` (GPL-3.0-or-later): the directory rules only,
//! because they are what every file format shared with the C# application hangs off. A recording,
//! a parameter file or a map tile in the right format in the wrong directory is one the other
//! application never finds, and "a user can run both apps against the same data directory"
//! (DELIVERABLES.md D17) is a promise about paths before it is one about bytes.
//!
//! The rules are not what a Linux user would guess. `GetUserDataDirectory` asks .NET for
//! `MyDocuments`, and under mono that is `$HOME`, not `~/Documents` - so the C# application on
//! Linux looks for `~/Mission Planner`, does not find it, and settles in
//! `~/.local/share/Mission Planner`. That is where a real installation on this machine keeps its
//! logs, its parameter metadata and its map cache. Measured by asking mono itself
//! (`Environment.GetFolderPath(SpecialFolder.MyDocuments)` printed the home directory), not
//! inferred from the enum's name - which is how this crate's predecessor came to record flights
//! under `~/Documents/Mission Planner/logs`, a directory the C# application never reads.

pub mod config;

pub use config::{Config, ConfigError};

use std::path::{Path, PathBuf};

/// `Settings.AppConfigName`: the directory name under every base the C# uses. With the space.
/// `// C#: ExtLibs/Utilities/Settings.cs:20`
pub const APP_CONFIG_NAME: &str = "Mission Planner";

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

    /// `Settings.GetUserDataDirectory`: user-specific data.
    ///
    /// The `MyDocuments` location if it already holds a `Mission Planner` directory, otherwise
    /// on unix only, local application data. The C# comment on the check reads "do not migrate
    /// to new approach if directory exists", so an installation that started life under the old
    /// rule stays there for good. That has to be honoured here too, or a user with a decade of
    /// logs in the old place gets a second, empty data directory and both applications disagree
    /// about which one is real.
    /// `// C#: ExtLibs/Utilities/Settings.cs:340-365`
    #[must_use]
    pub fn user_data_directory(&self) -> PathBuf {
        let old_approach = self.my_documents.join(APP_CONFIG_NAME);
        if self.unix && !old_approach.is_dir() {
            self.local_application_data.join(APP_CONFIG_NAME)
        } else {
            old_approach
        }
    }

    /// `Settings.GetDataDirectory`: data shared between users of a machine.
    ///
    /// Only Windows actually shares it - `%ProgramData%\Mission Planner`. Under mono it is the
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
            let path = std::env::temp_dir().join(format!(
                "mp-settings-{name}-{}-{:?}",
                std::process::id(),
                std::thread::current().id()
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
        // The rule that is not what anyone would guess: MyDocuments is $HOME under mono, so the
        // "old approach" path is ~/Mission Planner, and a machine that never had one gets
        // ~/.local/share/Mission Planner - which is where the real installation on the machine
        // this was written on keeps its logs and its map cache.
        let scratch = Scratch::new("fresh-linux");
        let folders = mono(&scratch);
        assert_eq!(
            folders.user_data_directory(),
            scratch.0.join("home/.local/share/Mission Planner")
        );
        assert_eq!(folders.data_directory(), folders.user_data_directory());
    }

    #[test]
    fn on_linux_an_old_installation_is_never_migrated() {
        // "Do not use new AppData path if old path already exists" - the C# comment. A decade
        // of logs in ~/Mission Planner stays there.
        let scratch = Scratch::new("old-linux");
        let folders = mono(&scratch);
        let old = folders.my_documents.join(APP_CONFIG_NAME);
        std::fs::create_dir_all(&old).expect("create the old directory");
        assert_eq!(folders.user_data_directory(), old);
        assert_eq!(folders.data_directory(), old);
    }

    #[test]
    fn on_linux_a_file_named_like_the_old_directory_does_not_count() {
        // Directory.Exists is false for a file.
        let scratch = Scratch::new("file-not-dir");
        let folders = mono(&scratch);
        std::fs::create_dir_all(&folders.my_documents).expect("home");
        std::fs::write(folders.my_documents.join(APP_CONFIG_NAME), b"").expect("a file");
        assert!(
            folders
                .user_data_directory()
                .starts_with(&folders.local_application_data)
        );
    }

    #[test]
    fn on_windows_user_data_is_under_documents_whether_or_not_it_exists() {
        let scratch = Scratch::new("windows-user");
        let folders = windows(&scratch);
        assert_eq!(
            folders.user_data_directory(),
            folders.my_documents.join("Mission Planner")
        );
    }

    #[test]
    fn on_windows_shared_data_is_under_program_data() {
        // Which is where the C# keeps gmapcache on Windows: C:\ProgramData\Mission Planner.
        let scratch = Scratch::new("windows-shared");
        let folders = windows(&scratch);
        assert_eq!(
            folders.data_directory(),
            scratch.0.join("ProgramData").join("Mission Planner")
        );
        assert_eq!(
            folders.map_cache_directory(),
            scratch.0.join("ProgramData/Mission Planner/gmapcache")
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
    fn the_directory_name_is_the_one_with_the_space_in_it() {
        // "MissionPlanner", "mission-planner" and "mission_planner" all look reasonable and are
        // all a different directory from the one the C# application uses.
        assert_eq!(APP_CONFIG_NAME, "Mission Planner");
    }

    #[test]
    fn the_map_cache_hangs_off_the_shared_data_directory() {
        let scratch = Scratch::new("cache");
        let folders = mono(&scratch);
        assert_eq!(
            folders.map_cache_directory(),
            folders.data_directory().join("gmapcache")
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
                .ends_with("Mission Planner/logs"),
            "{:?}",
            default_log_directory()
        );
    }
}
