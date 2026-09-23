//! Which serial ports Mission Planner offers, as pure functions of what the OS shows it.
//!
//! Ports `MissionPlanner.Comms.SerialPort.GetPortNames()` (`ExtLibs/Comms/CommsSerialPort.cs:209`)
//! and the pieces it leans on. Everything here takes a listing and returns a list, so the rules are
//! tested against checked-in listings from machines we do not have (`testdata/ports/`), and
//! [`crate::list_ports`] is left with nothing to do but produce the listing on this one.
//!
//! # Two sources, and which runtime answers the second
//!
//! `GetPortNames()` concatenates two sources and keeps the first of each duplicate:
//!
//! 1. seven `Directory.GetFiles` globs over `/dev` ([`UNIX_GLOBS`]);
//! 2. `System.IO.Ports.SerialPort.GetPortNames()`, trimmed and passed through
//!    `FixBlueToothPortNameBug` ([`fix_bluetooth_port_name`]).
//!
//! The second is runtime code. On Linux and macOS Mission Planner runs under Mono (`README.md:67`),
//! and Mono's `GetPortNames()` is the one the C# tree carries a copy of as `MonoSerialPort`
//! (`ExtLibs/Comms/System.IO.Ports/SerialPort.cs:536`, left out of the build by
//! `MissionPlanner.Comms.csproj:24` because Mono's own `System.dll` supplies the same class). That
//! copy is [`mono_port_names`], and it is why every `/dev/ttyS*` node is in the C# list on a Linux
//! PC whether a UART is behind it or not. On Windows the runtime reads the `SERIALCOMM` registry
//! key, which is [`windows_port_names`].
//!
//! # Runtime behaviour the tree does not show
//!
//! `Directory.GetFiles` is runtime code too, and three of its properties decide the list. They are
//! the .NET contract rather than anything in the C# tree, so they are stated here instead of being
//! assumed quietly:
//!
//! - it returns files only: a directory whose name matches (`/dev/usb/`, `/dev/vboxusb/`) is not
//!   listed, while device nodes and symlinks to them are, and a symlink counts as what it points at;
//! - it returns entries in directory order, unsorted. On Linux's devtmpfs that is newest first,
//!   which is why `/dev/ttyACM1` can come before `/dev/ttyACM0`;
//! - its wildcards keep Win32 semantics on every platform, case-sensitive on Unix, and a pattern
//!   ending in `.*` also matches the bare name - `tty.*` lists `/dev/tty`, as `dir foo.*` lists
//!   `foo` on Windows. Matches come in one pass in directory order here. Mono's older native
//!   globber is recalled to append the bare-name match after the dotted ones instead; that would
//!   move `/dev/tty` only where it precedes a `/dev/tty.*` node in directory order, as it may on
//!   macOS.
//!
//! Nothing here sorts. Mission Planner puts the list into its combo box as it comes
//! (`MainV2.cs:1296`), so the order is part of what has to match.

use std::collections::HashSet;

/// A discovered serial port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// Device path (`/dev/ttyACM0`, `/dev/serial/by-id/...`) or Windows port name (`COM3`).
    pub name: String,
    /// USB vendor id, when the port is a USB device.
    pub vid: Option<u16>,
    /// USB product id, when the port is a USB device.
    pub pid: Option<u16>,
    /// USB serial number, used to tell two identical boards apart.
    pub serial_number: Option<String>,
    /// Manufacturer string.
    pub manufacturer: Option<String>,
    /// Product string.
    pub product: Option<String>,
}

impl PortInfo {
    /// A port the OS told us nothing more about than its name.
    #[must_use]
    pub const fn bare(name: String) -> Self {
        Self {
            name,
            vid: None,
            pid: None,
            serial_number: None,
            manufacturer: None,
            product: None,
        }
    }

    /// A one-line label for the connection dropdown.
    #[must_use]
    pub fn label(&self) -> String {
        match (&self.product, self.vid, self.pid) {
            (Some(product), Some(vid), Some(pid)) => {
                format!("{} - {product} ({vid:04x}:{pid:04x})", self.name)
            }
            (Some(product), _, _) => format!("{} - {product}", self.name),
            _ => self.name.clone(),
        }
    }
}

/// The seven `Directory.GetFiles(directory, pattern)` calls, in the order the C# makes them.
///
/// `/dev/serial/by-id/` comes first on purpose: those names survive replugging and renumbering,
/// so they are the ones worth picking.
pub const UNIX_GLOBS: [(&str, &str); 7] = [
    ("/dev/serial/by-id/", "*"), // C#: ExtLibs/Comms/CommsSerialPort.cs:224
    ("/dev/", "ttyACM*"),        // C#: ExtLibs/Comms/CommsSerialPort.cs:232
    ("/dev/", "ttyUSB*"),        // C#: ExtLibs/Comms/CommsSerialPort.cs:240
    ("/dev/", "rfcomm*"),        // C#: ExtLibs/Comms/CommsSerialPort.cs:248
    ("/dev/", "*usb*"),          // C#: ExtLibs/Comms/CommsSerialPort.cs:256
    ("/dev/", "tty.*"),          // C#: ExtLibs/Comms/CommsSerialPort.cs:264
    ("/dev/", "cu.*"),           // C#: ExtLibs/Comms/CommsSerialPort.cs:272
];

/// The entries of the connection combo box that are not ports, and so never get a friendly name.
// C#: ExtLibs/Comms/CommsSerialPort.cs:321
pub const NOT_PORTS: [&str; 5] = ["AUTO", "UDP", "UDPCl", "TCP", "WS"];

/// Whether `name` matches a `Directory.GetFiles` search pattern.
///
/// `*` matches any run of characters, everything else matches itself, case-sensitively. A pattern
/// ending in `.*` also matches its stem alone: .NET keeps DOS's rule that a name without an
/// extension has an empty one, so `tty.*` matches `tty`. `?` is not handled because the C# never
/// passes it.
#[must_use]
pub fn matches_search_pattern(pattern: &str, name: &str) -> bool {
    wildcard(pattern.as_bytes(), name.as_bytes())
        || pattern
            .strip_suffix(".*")
            .is_some_and(|stem| wildcard(stem.as_bytes(), name.as_bytes()))
}

/// `*`-only wildcard match, backtracking to the most recent star.
fn wildcard(pattern: &[u8], name: &[u8]) -> bool {
    let (mut p, mut n) = (0usize, 0usize);
    // Where the last star was, and how much of the name it has swallowed so far.
    let mut star: Option<(usize, usize)> = None;
    while n < name.len() {
        match pattern.get(p) {
            Some(b'*') => {
                star = Some((p, n));
                p += 1;
            }
            Some(c) if Some(c) == name.get(n) => {
                p += 1;
                n += 1;
            }
            _ => match star {
                Some((star_p, star_n)) => {
                    p = star_p + 1;
                    n = star_n + 1;
                    star = Some((star_p, star_n + 1));
                }
                None => return false,
            },
        }
    }
    pattern
        .get(p..)
        .is_some_and(|rest| rest.iter().all(|&c| c == b'*'))
}

/// `Directory.GetFiles(directory, pattern)` over a listing.
///
/// `entries` are absolute paths in directory order, a directory written with a trailing `/`.
/// Returns the non-directory entries directly inside `directory` whose name matches, in listing
/// order, as the full path the C# gets back.
fn get_files(entries: &[&str], directory: &str, pattern: &str) -> Vec<String> {
    entries
        .iter()
        .filter_map(|entry| {
            let name = entry.strip_prefix(directory)?;
            // A slash left in the name means a subdirectory's contents, or a directory itself.
            if name.is_empty() || name.contains('/') {
                return None;
            }
            matches_search_pattern(pattern, name).then(|| (*entry).to_owned())
        })
        .collect()
}

/// Keeps the first of each repeated name, in order: LINQ's `Distinct()`.
fn distinct(names: impl IntoIterator<Item = String>) -> Vec<String> {
    let mut seen = HashSet::new();
    names
        .into_iter()
        .filter(|name| seen.insert(name.clone()))
        .collect()
}

/// The seven `/dev` globs applied to a listing, in the C#'s order, duplicates removed.
///
/// `entries` are absolute paths in directory order - `/dev` itself and `/dev/serial/by-id` are the
/// only directories read - with each directory written with a trailing `/`. A `/dev/ttyACM0` that
/// a by-id link points at is still listed in its own right: the C# compares names, not devices.
// C#: ExtLibs/Comms/CommsSerialPort.cs:216-277
#[must_use]
pub fn unix_candidates(entries: &[&str]) -> Vec<String> {
    distinct(
        UNIX_GLOBS
            .iter()
            .flat_map(|(directory, pattern)| get_files(entries, directory, pattern)),
    )
}

/// Mono's `System.IO.Ports.SerialPort.GetPortNames()` on Unix, over the same listing.
///
/// Lists `/dev/tty*`. If any of them is a Linux-style serial node (`ttyS`, `ttyUSB`, `ttyACM`), only
/// those are kept; otherwise - macOS, the BSDs - every `tty*` but `/dev/tty` itself and `ttyC*`.
// C#: ExtLibs/Comms/System.IO.Ports/SerialPort.cs:536-564
#[must_use]
pub fn mono_port_names(entries: &[&str]) -> Vec<String> {
    const LINUX_STYLE: [&str; 3] = ["/dev/ttyS", "/dev/ttyUSB", "/dev/ttyACM"];
    let is_linux_style = |dev: &str| LINUX_STYLE.iter().any(|prefix| dev.starts_with(prefix));

    let ttys = get_files(entries, "/dev/", "tty*");
    // One Linux-style node anywhere switches the whole list to the Linux rule.
    let linux_style = ttys.iter().any(|dev| is_linux_style(dev));
    ttys.into_iter()
        .filter(|dev| {
            if linux_style {
                is_linux_style(dev)
            } else {
                dev != "/dev/tty" && dev.starts_with("/dev/tty") && !dev.starts_with("/dev/ttyC")
            }
        })
        .collect()
}

/// The runtime's port names on Windows: the data of each `HARDWARE\DEVICEMAP\SERIALCOMM` value,
/// in registry order, skipping empty ones.
// C#: ExtLibs/Comms/System.IO.Ports/SerialPort.cs:566-575
#[must_use]
pub fn windows_port_names(serialcomm_values: &[&str]) -> Vec<String> {
    serialcomm_values
        .iter()
        .filter(|port| !port.is_empty())
        .map(|port| (*port).to_owned())
        .collect()
}

/// Repairs a Windows port name that .NET enumerated with junk on the end.
///
/// .NET sometimes reads a Bluetooth port's registry value past its end, so `COM10` comes back as
/// `COM10c`. The C# keeps `COM` and whichever of the next three characters are digits. It says
/// itself that this fails when the junk is a digit, and it also truncates anything past `COM999`;
/// both are kept, because the list has to match.
// C#: ExtLibs/Comms/CommsSerialPort.cs:391-411
#[must_use]
pub fn fix_bluetooth_port_name(port_name: &str) -> String {
    let Some(rest) = port_name.strip_prefix("COM") else {
        return port_name.to_owned();
    };
    // .NET's char.IsDigit also accepts non-ASCII decimal digits; a port name never holds one.
    let digits: String = rest.chars().take(3).filter(char::is_ascii_digit).collect();
    format!("COM{digits}")
}

/// The whole of `SerialPort.GetPortNames()`: the `/dev` globs, then the runtime's names with
/// trailing whitespace trimmed and the Bluetooth repair applied, keeping the first of each.
///
/// On Windows `entries` is empty, since `/dev/` does not exist there.
// C#: ExtLibs/Comms/CommsSerialPort.cs:209-309
#[must_use]
pub fn get_port_names<S: AsRef<str>>(entries: &[&str], runtime_names: &[S]) -> Vec<String> {
    let runtime = runtime_names
        .iter()
        .map(|port| fix_bluetooth_port_name(port.as_ref().trim_end()));
    distinct(unix_candidates(entries).into_iter().chain(runtime))
}

/// One row of WMI's `Win32_SerialPort`, the source of the friendly names Windows shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WmiSerialPort<'a> {
    /// `DeviceID`, the port name (`COM3`).
    pub device_id: &'a str,
    /// `Name`, the friendly name (`Communications Port (COM1)`).
    pub name: &'a str,
}

/// The friendly name Mission Planner shows beside a port on Windows: the `Name` of the first
/// `Win32_SerialPort` row whose `DeviceID` matches ignoring case, or nothing.
// C#: ExtLibs/Comms/CommsSerialPort.cs:314-345 (GetNiceName), Program.cs:508-524 (the WMI query)
#[must_use]
pub fn nice_name(port: &str, wmi: &[WmiSerialPort<'_>]) -> String {
    if NOT_PORTS.contains(&port) {
        return String::new();
    }
    let wanted = port.to_uppercase();
    wmi.iter()
        .find(|row| row.device_id.to_uppercase() == wanted)
        .map(|row| row.name.to_owned())
        .unwrap_or_default()
}

/// What a line of the connection combo box reads on Windows: the port, a space, its friendly name.
///
/// The space is there even when the name is empty. Under Mono the C# draws the port alone.
// C#: Controls/ConnectionControl.cs:64-68
#[must_use]
pub fn display_text(port: &str, nice: &str) -> String {
    format!("{port} {nice}")
}

/// Annotates the names the rules produced with what the OS knows about each device.
///
/// `resolve` turns a name into the device node it stands for - a by-id symlink into
/// `/dev/ttyACM0` - or `None` to use the name as it is. `known` is the OS's own list with USB ids,
/// matched on the resolved name, or on the known name after the same Bluetooth repair the C#
/// applies. The list itself, its order and its length, is the rules' and is not changed.
#[must_use]
pub fn with_usb_metadata<F>(names: Vec<String>, resolve: F, known: &[PortInfo]) -> Vec<PortInfo>
where
    F: Fn(&str) -> Option<String>,
{
    names
        .into_iter()
        .map(|name| {
            let node = resolve(&name).unwrap_or_else(|| name.clone());
            let found = known.iter().find(|port| {
                port.name == node || fix_bluetooth_port_name(port.name.trim_end()) == node
            });
            match found {
                Some(port) => PortInfo {
                    name,
                    ..port.clone()
                },
                None => PortInfo::bare(name),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_star_matches_any_run_including_none() {
        assert!(matches_search_pattern("*", "anything"));
        assert!(matches_search_pattern("ttyACM*", "ttyACM"));
        assert!(matches_search_pattern("ttyACM*", "ttyACM12"));
        assert!(matches_search_pattern("*usb*", "usbmon0"));
        assert!(matches_search_pattern("*usb*", "cu.usbmodem1101"));
        assert!(!matches_search_pattern("ttyACM*", "xttyACM0"));
    }

    #[test]
    fn matching_is_case_sensitive_as_on_unix() {
        // Which is why *usb* does not pick up ttyUSB0 on Linux: ttyUSB* does, two globs earlier.
        assert!(!matches_search_pattern("*usb*", "ttyUSB0"));
    }

    #[test]
    fn a_dot_star_pattern_also_matches_the_bare_stem() {
        assert!(matches_search_pattern("tty.*", "tty.usbmodem1101"));
        assert!(matches_search_pattern("tty.*", "tty"));
        assert!(!matches_search_pattern("tty.*", "tty0"));
        assert!(!matches_search_pattern("tty.*", "ttyS0"));
        assert!(matches_search_pattern("cu.*", "cu"));
    }

    #[test]
    fn get_files_skips_directories_and_subdirectories() {
        let entries = [
            "/dev/usb/",
            "/dev/usbmon0",
            "/dev/bus/usb/001/002",
            "/dev/serial/by-id/usb-x-if00",
        ];
        assert_eq!(get_files(&entries, "/dev/", "*usb*"), ["/dev/usbmon0"]);
        assert_eq!(
            get_files(&entries, "/dev/serial/by-id/", "*"),
            ["/dev/serial/by-id/usb-x-if00"]
        );
    }

    #[test]
    fn distinct_keeps_the_first_of_each_in_order() {
        let names = ["b", "a", "b", "c", "a"].map(str::to_owned);
        assert_eq!(distinct(names), ["b", "a", "c"]);
    }
}
