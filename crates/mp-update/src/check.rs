//! `Update.CheckForUpdate` and the work behind `Update.DoUpdate`: the version check, and
//! `CheckMD5`, which hashes every file the channel lists and fetches the ones that differ as
//! `<file>.new` beside the program. `// C#: Utilities/Update.cs:118-203, 219-440, 474-640, 684-738`

use std::path::{Component, Path, PathBuf};

use mp_firmware::manifest::Fetch;

use crate::version::Version;
use crate::{CHECKING, Channel, GETTING, md5};

/// What `CheckForUpdate` found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Check {
    /// No `UpdateLocationVersion`: the check does nothing. `// C#: Update.cs:123-124`
    NoChannel,
    /// The channel's version is not above the local one.
    UpToDate,
    /// The channel's version is above the local one, or there is no local `version.txt`; the
    /// link the question offers is the channel's `ChangeLog.txt`.
    UpdateFound {
        /// `baseurl.Replace("version.txt", "ChangeLog.txt")`.
        changelog_url: String,
    },
}

/// `StreamReader.ReadLine()` on a body: the first line, without its line end.
fn first_line(text: &str) -> Result<&str, String> {
    if text.is_empty() {
        // `new Version(null)`.
        return Err("Value cannot be null.".to_owned());
    }
    Ok(text.lines().next().unwrap_or(""))
}

/// `CheckForUpdate`'s finding: the channel's `version.txt` fetched first, then compared with the
/// first line of `install_dir/version.txt`; no local file is an update.
///
/// # Errors
///
/// The fetch failed, or either version does not parse: what the C# lets escape to its callers,
/// which log it ("Update check failed") or show it (the Help page's Error box).
/// `// C#: Utilities/Update.cs:118-178`
pub fn check_for_update(
    fetch: &dyn Fetch,
    version_url: &str,
    install_dir: &Path,
) -> Result<Check, String> {
    if version_url.is_empty() {
        return Ok(Check::NoChannel);
    }
    let path = install_dir.join("version.txt");
    let body = fetch.get(version_url)?;
    let update_found = if path.exists() {
        let local_text = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
        let local = Version::parse(first_line(&local_text)?)?;
        let remote_text = String::from_utf8_lossy(&body);
        let remote = Version::parse(first_line(&remote_text)?)?;
        local < remote
    } else {
        // "File does not exist: Getting"
        true
    };
    Ok(if update_found {
        Check::UpdateFound {
            changelog_url: version_url.replace("version.txt", "ChangeLog.txt"),
        }
    } else {
        Check::UpToDate
    })
}

/// `DoUpdateWorker_DoWork`'s write test: a file written and deleted in the install directory.
///
/// # Errors
///
/// "Unable to write to the install directory". `// C#: Utilities/Update.cs:711-731`
pub fn write_test(install_dir: &Path) -> Result<(), String> {
    let probe = install_dir.join("writetest.txt");
    let outcome = std::fs::write(&probe, "this is a test")
        .map_err(|_| "Unable to write to the install directory".to_owned());
    // "Write test cleanup failed" is only logged.
    let _ = std::fs::remove_file(&probe);
    outcome
}

/// `MD5File(filename, hash)`: whether the file is there and hashes to `hash`.
/// `// C#: Utilities/Update.cs:446-472`
#[must_use]
pub fn md5_file(path: &Path, hash: &str) -> bool {
    match std::fs::read(path) {
        Ok(data) => md5::hex(&data) == hash,
        Err(_) => false,
    }
}

/// One line of `checksums.txt` as the regex `([^\s]+)\s+[^/]+/(.*)` reads it: the hash and the
/// path after the first folder.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Listed {
    hash: String,
    file: String,
}

/// `CheckMD5`'s regex over the whole text. `// C#: Utilities/Update.cs:226`
fn listed(text: &str) -> Vec<Listed> {
    // The pattern is a constant, so it compiles.
    let Ok(regex) = regex::Regex::new(r"([^\s]+)\s+[^/]+/(.*)") else {
        return Vec::new();
    };
    regex
        .captures_iter(text)
        .map(|found| Listed {
            hash: found.get(1).map_or("", |m| m.as_str()).to_owned(),
            file: found.get(2).map_or("", |m| m.as_str()).to_owned(),
        })
        .collect()
}

/// `Path.GetFullPath(Path.Combine(dir, file))`, lower-cased: the file's place under the install
/// directory with `.` and `..` folded, as the cleanup compares them.
fn full_path_lower(dir: &Path, file: &str) -> String {
    let mut out = PathBuf::new();
    for component in dir.join(file).components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other.as_os_str()),
        }
    }
    out.to_string_lossy().to_lowercase()
}

/// Every file under `dir` with the extension, as `Directory.GetFiles(dir, "*.ext",
/// AllDirectories)` lists them (the extension compared without case).
fn files_with_extension(dir: &Path, extension: &str) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut pending = vec![dir.to_path_buf()];
    while let Some(folder) = pending.pop() {
        let Ok(entries) = std::fs::read_dir(&folder) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                pending.push(path);
            } else if path
                .extension()
                .is_some_and(|e| e.to_string_lossy().eq_ignore_ascii_case(extension))
            {
                out.push(path);
            }
        }
    }
    out
}

/// The sort `CheckMD5` gives the files to fetch: `.exe` first, then `.dll`, then the rest, each
/// group by name. (The C# compares with `String.CompareTo`, the culture's order; this is the
/// byte order.) `// C#: Utilities/Update.cs:266-281`
fn fetch_order(a: &str, b: &str) -> std::cmp::Ordering {
    let rank = |name: &str| {
        let lower = name.to_lowercase();
        if lower.ends_with(".exe") {
            0
        } else if lower.ends_with(".dll") {
            1
        } else {
            2
        }
    };
    rank(a).cmp(&rank(b)).then_with(|| a.cmp(b))
}

/// The update in progress: the network and, for a zip channel, the zip once fetched.
struct Work<'a> {
    fetch: &'a dyn Fetch,
    zip: Option<Vec<mp_log::zip::Entry>>,
}

impl Work<'_> {
    /// `GetNewFile`: the file fetched from `baseurl + subdir + file + "?" + random` as
    /// `<file>.new`, two attempts, progress said as it comes.
    /// `// C#: Utilities/Update.cs:513-610`
    fn get_new_file(
        &self,
        base_url: &str,
        subdir: &str,
        file: &str,
        dest: &Path,
        progress: &mut dyn FnMut(i32, &str),
        cancel_requested: &dyn Fn() -> bool,
    ) -> Result<(), String> {
        let mut fail = String::new();
        let mut attempt = 0;
        while attempt < 2 {
            if cancel_requested() {
                return Err("Cancel".to_owned());
            }
            let url = format!("{base_url}{subdir}{file}?{}", random());
            let fetched = self.fetch.get_progress(&url, &mut |got, total| {
                if let Some(total) = total.filter(|t| *t > 0) {
                    #[allow(clippy::cast_precision_loss)] // a percentage
                    let percent = got as f64 / total as f64 * 100.0;
                    #[allow(clippy::cast_possible_truncation)] // 0..=100
                    progress(percent as i32, &format!("{GETTING}{file}: {percent:.1}%"));
                }
            });
            match fetched.and_then(|bytes| std::fs::write(dest, bytes).map_err(|e| e.to_string())) {
                Ok(()) => return Ok(()),
                Err(e) => {
                    fail = e;
                    attempt += 1;
                }
            }
        }
        Err(fail)
    }

    /// `GetNewFileZip`: the entry `subdir/file` of the channel's zip as `<file>.new`; an entry
    /// the zip has not got is logged and skipped (the hash check then fails). The zip is fetched
    /// whole, once, where the C# reads each entry by HTTP range.
    /// `// C#: Utilities/Update.cs:474-511`
    fn get_new_file_zip(
        &mut self,
        base_url: &str,
        subdir: &str,
        file: &str,
        dest: &Path,
    ) -> Result<(), String> {
        if self.zip.is_none() {
            let bytes = self.fetch.get(base_url)?;
            self.zip = Some(mp_log::zip::read(&bytes).map_err(|e| e.to_string())?);
        }
        let name = format!(
            "{}{file}",
            subdir.trim_start_matches(['/', '\\']).replace('\\', "/")
        );
        let Some(entry) = self
            .zip
            .as_deref()
            .and_then(|entries| entries.iter().find(|e| e.name == name))
        else {
            // "zip missing entry"
            return Ok(());
        };
        std::fs::write(dest, &entry.data).map_err(|e| e.to_string())
    }
}

/// `new Random().Next()` on the URL, so no cache answers.
fn random() -> u32 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.subsec_nanos());
    nanos ^ std::process::id()
}

/// `DoUpdateWorker_DoWork` and `updateCheckMain` up to "Starting Updater": "Getting Base URL",
/// the write test, then `CheckMD5` - the channel's `checksums.txt` kept as
/// `checksums.txt.new`, every listed file hashed, the ones that differ fetched as `<file>.new`
/// and checked against the list, and the `.dll` and `.exe` files under the install directory
/// that the list has not got deleted. Returns the `.new` files written. `progress` is the
/// dialog's `UpdateProgressAndStatus(percent, text)`, -1 a marquee; `cancel_requested` is its
/// Cancel.
///
/// # Errors
///
/// "Unable to write to the install directory"; the fetches' errors; "User Request" or "Cancel"
/// when cancelled; "File downloaded does not match hash: <file>".
/// `// C#: Utilities/Update.cs:219-440, 684-738`
pub fn do_update(
    fetch: &dyn Fetch,
    channel: &Channel,
    install_dir: &Path,
    progress: &mut dyn FnMut(i32, &str),
    cancel_requested: &dyn Fn() -> bool,
) -> Result<Vec<PathBuf>, String> {
    progress(-1, "Getting Base URL");
    write_test(install_dir)?;
    let text = String::from_utf8_lossy(&fetch.get(&channel.md5_url)?).into_owned();
    std::fs::write(install_dir.join("checksums.txt.new"), &text).map_err(|e| e.to_string())?;
    let list = listed(&text);
    if list.is_empty() {
        return Ok(Vec::new());
    }
    progress(-1, "Hashing Files");
    // cleanup dll's with the same exe name
    let dlls = files_with_extension(install_dir, "dll");
    let exes = files_with_extension(install_dir, "exe");
    let files: Vec<String> = list.iter().map(|l| l.file.clone()).collect();
    // hash everything
    let mut tasks: Vec<(String, String, bool)> = list
        .iter()
        .map(|l| (l.file.trim().to_owned(), l.hash.trim().to_owned()))
        .filter(|(file, _)| !file.to_lowercase().ends_with("files.html"))
        .map(|(file, hash)| {
            let matches = md5_file(&install_dir.join(&file), &hash);
            (file, hash, matches)
        })
        .collect();
    let count = tasks.iter().filter(|(_, _, matches)| !matches).count();
    tasks.sort_by(|a, b| fetch_order(&a.0, &b.0));
    let total = tasks.len();
    let mut work = Work { fetch, zip: None };
    let is_zip = channel.base_url.to_lowercase().contains(".zip");
    let mut written = Vec::new();
    let mut done = 0usize;
    for (file, hash, matches) in &tasks {
        if *matches {
            // "Same File"
            progress(-1, &format!("{CHECKING}{file}"));
            continue;
        }
        done += 1;
        // "Newer File"
        if cancel_requested() {
            return Err("User Request".to_owned());
        }
        let dest = install_dir.join(format!("{file}.new"));
        // check is we have already downloaded and matchs hash
        if md5_file(&dest, hash) {
            // "already got new File"
            continue;
        }
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)] // a percentage
        let percent = (done as f64 / count.max(1) as f64 * 100.0) as i32;
        progress(
            percent,
            &format!("{GETTING}{file}\n{done} of {count} of total {total}"),
        );
        let path = Path::new(file);
        let parent = path
            .parent()
            .map_or(String::new(), |p| p.to_string_lossy().into_owned());
        let name = path
            .file_name()
            .map_or(String::new(), |n| n.to_string_lossy().into_owned());
        // `Path.GetDirectoryName(file) + DirectorySeparatorChar`, a file at the root getting a
        // bare separator, so its URL has two slashes in a row, as the C#'s does.
        let subdir = format!("{}/", parent.replace('\\', "/"));
        if let Some(dir) = dest.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        if is_zip {
            work.get_new_file_zip(&channel.base_url, &subdir, &name, &dest)?;
        } else {
            work.get_new_file(
                &channel.base_url,
                &subdir,
                &name,
                &dest,
                progress,
                cancel_requested,
            )?;
        }
        // check the new downloaded file matchs hash
        if !md5_file(&dest, hash) {
            return Err(format!("File downloaded does not match hash: {file}"));
        }
        written.push(dest);
    }
    // cleanup unused dlls and exes
    let listed_paths: Vec<String> = files
        .iter()
        .map(|file| full_path_lower(install_dir, file))
        .collect();
    for stray in dlls.iter().chain(exes.iter()) {
        let here = stray.to_string_lossy().to_lowercase();
        if !listed_paths.contains(&here) {
            let _ = std::fs::remove_file(stray);
        }
    }
    Ok(written)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// A server of the given URLs, the `?random` on a request ignored, counting the requests.
    struct Server {
        pages: HashMap<String, Vec<u8>>,
        requests: std::cell::RefCell<Vec<String>>,
    }

    impl Server {
        fn new(pages: &[(&str, &[u8])]) -> Self {
            Self {
                pages: pages
                    .iter()
                    .map(|(url, body)| ((*url).to_owned(), body.to_vec()))
                    .collect(),
                requests: std::cell::RefCell::new(Vec::new()),
            }
        }
    }

    impl Fetch for Server {
        fn get(&self, url: &str) -> Result<Vec<u8>, String> {
            let bare = url.split('?').next().unwrap_or(url);
            self.requests.borrow_mut().push(bare.to_owned());
            self.pages
                .get(bare)
                .cloned()
                .ok_or_else(|| format!("http status 404 for {bare}"))
        }
    }

    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("mp-update-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn the_version_check_compares_first_lines_and_a_missing_file_is_an_update() {
        let dir = scratch("check");
        let server = Server::new(&[("https://x/upgrade/version.txt", b"1.3.80.0\r\nnotes\n")]);
        assert_eq!(
            check_for_update(&server, "", &dir).unwrap(),
            Check::NoChannel
        );
        assert_eq!(
            check_for_update(&server, "https://x/upgrade/version.txt", &dir).unwrap(),
            Check::UpdateFound {
                changelog_url: "https://x/upgrade/ChangeLog.txt".to_owned()
            }
        );
        std::fs::write(dir.join("version.txt"), "1.3.80.0\n").unwrap();
        assert_eq!(
            check_for_update(&server, "https://x/upgrade/version.txt", &dir).unwrap(),
            Check::UpToDate
        );
        std::fs::write(dir.join("version.txt"), "1.3.79.2\n").unwrap();
        assert!(matches!(
            check_for_update(&server, "https://x/upgrade/version.txt", &dir).unwrap(),
            Check::UpdateFound { .. }
        ));
        std::fs::write(dir.join("version.txt"), "soon\n").unwrap();
        assert_eq!(
            check_for_update(&server, "https://x/upgrade/version.txt", &dir).unwrap_err(),
            "Version string portion was too short or too long."
        );
        std::fs::write(dir.join("version.txt"), "1.x\n").unwrap();
        assert_eq!(
            check_for_update(&server, "https://x/upgrade/version.txt", &dir).unwrap_err(),
            "Input string was not in a correct format."
        );
        assert!(check_for_update(&server, "https://x/none.txt", &dir).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn checksums_are_read_by_the_regex() {
        let text = "0cc175b9c0f1b6a831c399e269772661  ./planner\n\
                    900150983cd24fb0d6963f7d28e17f72 MissionPlanner/assets/x.bin\r\n\
                    d41d8cd98f00b204e9800998ecf8427e  ./files.html\n";
        let list = listed(text);
        assert_eq!(list.len(), 3);
        assert_eq!(list[0].hash, "0cc175b9c0f1b6a831c399e269772661");
        assert_eq!(list[0].file, "planner");
        assert_eq!(list[1].file, "assets/x.bin\r");
        assert!(listed("nothing here").is_empty());
        assert_eq!(fetch_order("b.dll", "a.exe"), std::cmp::Ordering::Greater);
        assert_eq!(fetch_order("z.exe", "a.dll"), std::cmp::Ordering::Less);
        assert_eq!(fetch_order("a.txt", "b.txt"), std::cmp::Ordering::Less);
        assert_eq!(
            full_path_lower(Path::new("/Inst"), "./Sub/../A.DLL"),
            "/inst/a.dll"
        );
    }

    #[test]
    fn differing_files_are_fetched_as_new_and_checked() {
        let dir = scratch("do-update");
        std::fs::create_dir_all(dir.join("assets")).unwrap();
        std::fs::write(dir.join("planner"), b"old planner").unwrap();
        std::fs::write(dir.join("assets/same.bin"), b"abc").unwrap();
        std::fs::write(dir.join("stray.dll"), b"x").unwrap();
        std::fs::write(dir.join("keep.dll"), b"keep").unwrap();
        let checksums = format!(
            "{}  ./planner\n{}  ./assets/same.bin\n{}  ./keep.dll\n{}  ./files.html\n",
            md5::hex(b"new planner"),
            md5::hex(b"abc"),
            md5::hex(b"keep"),
            md5::hex(b"")
        );
        let server = Server::new(&[
            ("https://x/checksums.txt", checksums.as_bytes()),
            ("https://x/upgrade//planner", b"new planner"),
        ]);
        let channel = Channel {
            version_url: String::new(),
            md5_url: "https://x/checksums.txt".to_owned(),
            base_url: "https://x/upgrade/".to_owned(),
        };
        let mut said = Vec::new();
        let written = do_update(
            &server,
            &channel,
            &dir,
            &mut |percent, text| said.push((percent, text.to_owned())),
            &|| false,
        )
        .unwrap();
        assert_eq!(written, vec![dir.join("planner.new")]);
        assert_eq!(
            std::fs::read(dir.join("planner.new")).unwrap(),
            b"new planner"
        );
        assert_eq!(
            std::fs::read_to_string(dir.join("checksums.txt.new")).unwrap(),
            checksums
        );
        assert!(
            !dir.join("stray.dll").exists(),
            "a dll the list has not got is deleted"
        );
        assert!(dir.join("keep.dll").exists());
        assert!(!dir.join("writetest.txt").exists());
        assert_eq!(said.first(), Some(&(-1, "Getting Base URL".to_owned())));
        assert!(said.contains(&(-1, "Hashing Files".to_owned())));
        assert!(
            said.contains(&(100, "Getting planner\n1 of 1 of total 3".to_owned())),
            "{said:?}"
        );
        assert!(said.contains(&(-1, "Checking assets/same.bin".to_owned())));
        // The URL for a file at the root has the C#'s two slashes.
        assert!(
            server
                .requests
                .borrow()
                .contains(&"https://x/upgrade//planner".to_owned())
        );

        // Already fetched and matching: nothing fetched again.
        server.requests.borrow_mut().clear();
        let again = do_update(&server, &channel, &dir, &mut |_, _| {}, &|| false).unwrap();
        assert!(again.is_empty());
        assert_eq!(
            server.requests.borrow().as_slice(),
            ["https://x/checksums.txt"]
        );

        // A server whose file does not hash as listed.
        std::fs::remove_file(dir.join("planner.new")).unwrap();
        let wrong = Server::new(&[
            ("https://x/checksums.txt", checksums.as_bytes()),
            ("https://x/upgrade//planner", b"something else"),
        ]);
        assert_eq!(
            do_update(&wrong, &channel, &dir, &mut |_, _| {}, &|| false).unwrap_err(),
            "File downloaded does not match hash: planner"
        );
        // Cancelled before the first fetch.
        std::fs::remove_file(dir.join("planner.new")).unwrap();
        assert_eq!(
            do_update(&server, &channel, &dir, &mut |_, _| {}, &|| true).unwrap_err(),
            "User Request"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_zip_channel_is_read_once_for_every_file() {
        let dir = scratch("zip");
        std::fs::write(dir.join("a.txt"), b"old a").unwrap();
        let entries = vec![
            mp_log::zip::Entry {
                name: "a.txt".to_owned(),
                data: b"new a".to_vec(),
            },
            mp_log::zip::Entry {
                name: "sub/b.txt".to_owned(),
                data: b"new b".to_vec(),
            },
        ];
        let stamp = mp_log::zip::DosTime {
            year: 2026,
            month: 10,
            day: 3,
            hour: 12,
            minute: 0,
            second: 0,
        };
        let zip = mp_log::zip::write(&entries, stamp).unwrap();
        let checksums = format!(
            "{}  ./a.txt\n{}  ./sub/b.txt\n{}  ./zz-missing.txt\n",
            md5::hex(b"new a"),
            md5::hex(b"new b"),
            md5::hex(b"never")
        );
        let server = Server::new(&[
            ("https://x/beta/checksums.txt", checksums.as_bytes()),
            ("https://x/beta/Beta.zip", &zip),
        ]);
        let channel = Channel {
            version_url: String::new(),
            md5_url: "https://x/beta/checksums.txt".to_owned(),
            base_url: "https://x/beta/Beta.zip".to_owned(),
        };
        let outcome = do_update(&server, &channel, &dir, &mut |_, _| {}, &|| false);
        // a.txt and sub/b.txt came out of the zip; zz-missing.txt, fetched last by the sort, is
        // not in it, so its hash fails.
        assert_eq!(
            outcome.unwrap_err(),
            "File downloaded does not match hash: zz-missing.txt"
        );
        assert_eq!(std::fs::read(dir.join("a.txt.new")).unwrap(), b"new a");
        assert_eq!(std::fs::read(dir.join("sub/b.txt.new")).unwrap(), b"new b");
        let zip_requests = server
            .requests
            .borrow()
            .iter()
            .filter(|u| u.ends_with("Beta.zip"))
            .count();
        assert_eq!(zip_requests, 1);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn an_unwritable_install_directory_is_said() {
        let outcome = write_test(Path::new("/proc/no-such-dir-for-mp-update"));
        assert_eq!(
            outcome.unwrap_err(),
            "Unable to write to the install directory"
        );
    }
}
