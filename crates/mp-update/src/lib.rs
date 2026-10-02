//! Mission Planner's own update: `Utilities/Update.cs`, which checks a channel's `version.txt`
//! against the one beside the program and, on "Update Now", downloads every file whose MD5 differs
//! from the channel's `checksums.txt` as `<file>.new`; and `Updater/Program.cs`, the separate
//! program it then starts, which waits for the planner to exit, moves each `.new` into place
//! (the old file to `.old`) and starts the planner again.
//!
//! The channel is `app.config`'s `UpdateLocationVersion`, `UpdateLocation` and
//! `UpdateLocationMD5` (the stable channel), `BetaUpdateLocationVersion`, `BetaUpdateLocationMD5`
//! and `BetaUpdateLocationZip` (beta, read out of a zip) and `MasterUpdateLocationMD5` and
//! `MasterUpdateLocationZip` (master, with no version to check). Mission Planner's point at
//! firmware.ardupilot.org and its GitHub releases, which hold Mission Planner's .NET files, not
//! this program's; so here the keys are settings of the same names, and empty - no channel -
//! until the owner publishes one, which is what `CheckForUpdate` does with an empty
//! `UpdateLocationVersion`: nothing (`Update.cs:120-124`).
//!
//! Without a window: [`check`] returns what `CheckForUpdate` found and what `DoUpdate` did, and
//! the callers (the planner's Help page and its once-a-day check at startup, `headless-planner
//! update-apply` as `Updater.exe`) put the questions, the progress dialog and the restart around
//! them. Divergences, written at their sites: the files are hashed and fetched one after another
//! where the C# runs them in parallel; a zip channel is fetched whole once where the C# reads
//! each entry by HTTP range (`DownloadStream`).
//! `// C#: Utilities/Update.cs; Updater/Program.cs; app.config:10-18`

pub mod apply;
pub mod check;
pub mod md5;
pub mod version;

pub use mp_firmware::manifest::Fetch;

/// The `app.config` keys of the three channels, read as settings of the same names.
/// `// C#: app.config:10-18`
pub mod keys {
    /// The stable channel's `version.txt`.
    pub const UPDATE_LOCATION_VERSION: &str = "UpdateLocationVersion";
    /// The stable channel's files, a directory URL.
    pub const UPDATE_LOCATION: &str = "UpdateLocation";
    /// The stable channel's `checksums.txt`.
    pub const UPDATE_LOCATION_MD5: &str = "UpdateLocationMD5";
    /// The beta channel's `version.txt`.
    pub const BETA_UPDATE_LOCATION_VERSION: &str = "BetaUpdateLocationVersion";
    /// The beta channel's `checksums.txt`.
    pub const BETA_UPDATE_LOCATION_MD5: &str = "BetaUpdateLocationMD5";
    /// The beta channel's files, a zip.
    pub const BETA_UPDATE_LOCATION_ZIP: &str = "BetaUpdateLocationZip";
    /// The master channel's `checksums.txt`.
    pub const MASTER_UPDATE_LOCATION_MD5: &str = "MasterUpdateLocationMD5";
    /// The master channel's files, a zip.
    pub const MASTER_UPDATE_LOCATION_ZIP: &str = "MasterUpdateLocationZip";
}

/// One channel's addresses: where its version is, where its checksums are, and where its files
/// are (a directory, or a zip when the address has `.zip` in it).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Channel {
    /// `version.txt`; empty for the master channel, which is never version-checked.
    pub version_url: String,
    /// `checksums.txt`.
    pub md5_url: String,
    /// The files: `UpdateLocation`, or the `*Zip`.
    pub base_url: String,
}

/// Which channel `Update.dobeta` and `Update.domaster` pick. `// C#: Update.cs:28-29, 44-59`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Which {
    /// Neither flag.
    Stable,
    /// `dobeta`.
    Beta,
    /// `domaster` (which wins over `dobeta`).
    Master,
}

impl Which {
    /// The channel out of the settings, by the keys: `updateCheckMain`'s three branches and
    /// `CheckForUpdate`'s version URL. `// C#: Update.cs:44-59, 120-124`
    pub fn channel(self, setting: &dyn Fn(&str) -> Option<String>) -> Channel {
        let get = |key: &str| setting(key).unwrap_or_default();
        match self {
            Self::Stable => Channel {
                version_url: get(keys::UPDATE_LOCATION_VERSION),
                md5_url: get(keys::UPDATE_LOCATION_MD5),
                base_url: get(keys::UPDATE_LOCATION),
            },
            Self::Beta => Channel {
                version_url: get(keys::BETA_UPDATE_LOCATION_VERSION),
                md5_url: get(keys::BETA_UPDATE_LOCATION_MD5),
                base_url: get(keys::BETA_UPDATE_LOCATION_ZIP),
            },
            Self::Master => Channel {
                version_url: String::new(),
                md5_url: get(keys::MASTER_UPDATE_LOCATION_MD5),
                base_url: get(keys::MASTER_UPDATE_LOCATION_ZIP),
            },
        }
    }
}

/// `Strings.UpdateFound`. `// C#: ExtLibs/Strings/Strings.resx:426`
pub const UPDATE_FOUND: &str = "Update Found";
/// `Strings.UpdateNow`.
pub const UPDATE_NOW: &str = "Update Now";
/// `Strings.UpdateNotFound`.
pub const UPDATE_NOT_FOUND: &str = "No update available.";
/// `Strings.Getting`, a trailing space included.
pub const GETTING: &str = "Getting ";
/// `Strings.Checking`, a trailing space included.
pub const CHECKING: &str = "Checking ";
/// The progress dialog's caption. `// C#: Update.cs:202`
pub const CHECK_FOR_UPDATES: &str = "Check for Updates";
/// "Update Failed " before the exception's message. `// C#: Update.cs:103, 108`
pub const UPDATE_FAILED: &str = "Update Failed ";
/// The box the beta button shows with Control held. `// C#: GCSViews/Help.cs:74`
pub const MASTER_WARNING: &str = "This will update to MASTER release";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channels_come_from_the_keys_and_are_empty_without_them() {
        let none = |_: &str| None;
        assert_eq!(Which::Stable.channel(&none), Channel::default());
        let some = |key: &str| Some(format!("https://x/{key}"));
        let beta = Which::Beta.channel(&some);
        assert_eq!(beta.version_url, "https://x/BetaUpdateLocationVersion");
        assert_eq!(beta.base_url, "https://x/BetaUpdateLocationZip");
        let master = Which::Master.channel(&some);
        assert_eq!(master.version_url, "");
        assert_eq!(master.md5_url, "https://x/MasterUpdateLocationMD5");
    }
}
