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

//! Serial enumeration against checked-in listings from each OS.
//!
//! Deliverable 3 asks for the device list Mission Planner shows on the same hardware. The hardware is not
//! here, so its listings are (`testdata/ports/`), and each test asserts the exact list, in order,
//! that `SerialPort.GetPortNames()` would build from one. Where the C# tree is present, the rules'
//! constants are also checked against its source, so a change upstream fails here first.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use mp_transport::enumerate::{
    NOT_PORTS, PortInfo, UNIX_GLOBS, WmiSerialPort, display_text, fix_bluetooth_port_name,
    get_port_names, mono_port_names, nice_name, unix_candidates, windows_port_names,
    with_usb_metadata,
};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn fixture(name: &str) -> String {
    let path = repo().join("testdata/ports").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// Lines that carry data: comments and blank lines dropped, Windows line endings tolerated.
fn data_lines(text: &str) -> impl Iterator<Item = &str> {
    text.lines()
        .map(|line| line.trim_end_matches('\r'))
        .filter(|line| !line.trim().is_empty() && !line.starts_with('#'))
}

/// A `/dev` listing fixture: the entries in order, the symlinks, and the `serialport` crate's
/// view of each USB node.
struct DevListing {
    entries: Vec<String>,
    links: Vec<(String, String)>,
    usb: Vec<PortInfo>,
}

impl DevListing {
    fn load(name: &str) -> Self {
        let text = fixture(name);
        let mut listing = Self {
            entries: Vec::new(),
            links: Vec::new(),
            usb: Vec::new(),
        };
        for line in data_lines(&text) {
            if let Some(usb) = line.strip_prefix("usb|") {
                let fields: Vec<&str> = usb.split('|').collect();
                let [node, ids, serial, manufacturer, product] = fields[..] else {
                    panic!("bad usb line {line:?}");
                };
                let (vid, pid) = ids.split_once(':').unwrap();
                listing.usb.push(PortInfo {
                    name: node.to_owned(),
                    vid: Some(u16::from_str_radix(vid, 16).unwrap()),
                    pid: Some(u16::from_str_radix(pid, 16).unwrap()),
                    serial_number: Some(serial.to_owned()),
                    manufacturer: Some(manufacturer.to_owned()),
                    product: Some(product.to_owned()),
                    hardware_id: None,
                    description: None,
                });
            } else if let Some((path, target)) = line.split_once(" -> ") {
                listing.entries.push(path.to_owned());
                listing.links.push((path.to_owned(), target.to_owned()));
            } else {
                listing.entries.push(line.to_owned());
            }
        }
        listing
    }

    fn entries(&self) -> Vec<&str> {
        self.entries.iter().map(String::as_str).collect()
    }

    /// What realpath() would say, worked out from the fixture's link targets.
    fn resolve(&self, name: &str) -> Option<String> {
        let (link, target) = self.links.iter().find(|(link, _)| link == name)?;
        let mut parts: Vec<&str> = link.split('/').filter(|p| !p.is_empty()).collect();
        parts.pop(); // the link's own name
        for part in target.split('/') {
            match part {
                ".." => {
                    parts.pop();
                }
                "." | "" => {}
                other => parts.push(other),
            }
        }
        Some(format!("/{}", parts.join("/")))
    }

    /// The whole of GetPortNames() on this listing, as it runs under Mono.
    fn port_names(&self) -> Vec<String> {
        let entries = self.entries();
        get_port_names(&entries, &mono_port_names(&entries))
    }
}

const PIXHAWK_IF00: &str =
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_3A0045001351333031383238-if00";
const PIXHAWK_IF02: &str =
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_3A0045001351333031383238-if02";
const FTDI: &str = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10K3Z9B-if00-port0";

#[test]
fn linux_lists_by_id_first_then_each_glob_in_directory_order_then_monos_ttys() {
    let listing = DevListing::load("linux-dev.txt");
    assert_eq!(
        listing.port_names(),
        [
            // /dev/serial/by-id/*: the stable names, first.
            PIXHAWK_IF00,
            PIXHAWK_IF02,
            FTDI,
            // ttyACM*, in devtmpfs order: newest first, so 1 before 0. Nothing sorts them.
            "/dev/ttyACM1",
            "/dev/ttyACM0",
            // ttyUSB*, then rfcomm*, though rfcomm0 heads the directory.
            "/dev/ttyUSB0",
            "/dev/rfcomm0",
            // *usb*: case-sensitive, so not ttyUSB0 again; directories usb/ and vboxusb/ skipped.
            "/dev/usbmon2",
            "/dev/usbmon1",
            "/dev/usbmon0",
            // tty.* also matches the bare stem.
            "/dev/tty",
            // Mono's GetPortNames(): Linux-style nodes; ttyACM/ttyUSB were already listed.
            "/dev/ttyS3",
            "/dev/ttyS2",
            "/dev/ttyS1",
            "/dev/ttyS0",
        ]
    );
}

#[test]
fn linux_leaves_out_the_nodes_that_are_not_ports() {
    let names = DevListing::load("linux-dev.txt").port_names();
    for noise in [
        "/dev/tty0",
        "/dev/tty1",
        "/dev/tty2",
        "/dev/ttyprintk",
        "/dev/console",
        "/dev/ptmx",
        "/dev/null",
        "/dev/random",
        "/dev/urandom",
        "/dev/zero",
        "/dev/hidraw0",
        "/dev/hidraw2",
        "/dev/usb/",
        "/dev/usb",
        "/dev/vboxusb",
        "/dev/serial",
    ] {
        assert!(
            !names
                .iter()
                .any(|name| name.trim_end_matches('/') == noise.trim_end_matches('/')),
            "{noise} is not a port but was listed"
        );
    }
}

#[test]
fn linux_globs_alone_are_the_seven_in_order() {
    // The globs without Mono's list: what CommsSerialPort.cs adds itself.
    let listing = DevListing::load("linux-dev.txt");
    let globs = unix_candidates(&listing.entries());
    assert_eq!(globs.len(), 11, "{globs:#?}");
    assert_eq!(globs.last().map(String::as_str), Some("/dev/tty"));
    assert!(!globs.iter().any(|name| name.starts_with("/dev/ttyS")));
}

#[test]
fn monos_list_on_linux_is_every_linux_style_node_and_nothing_else() {
    let listing = DevListing::load("linux-dev.txt");
    assert_eq!(
        mono_port_names(&listing.entries()),
        [
            "/dev/ttyACM1",
            "/dev/ttyACM0",
            "/dev/ttyUSB0",
            "/dev/ttyS3",
            "/dev/ttyS2",
            "/dev/ttyS1",
            "/dev/ttyS0",
        ]
    );
}

#[test]
fn linux_ports_carry_the_usb_ids_of_the_device_they_name() {
    // A by-id name and the node it links to describe the same board, so both carry its ids;
    // everything else is listed bare.
    let listing = DevListing::load("linux-dev.txt");
    let ports = with_usb_metadata(listing.port_names(), |n| listing.resolve(n), &listing.usb);
    let labels: Vec<String> = ports.iter().map(PortInfo::label).collect();
    assert_eq!(
        labels[..7],
        [
            format!("{PIXHAWK_IF00} - Pixhawk6X (1209:5741)"),
            format!("{PIXHAWK_IF02} - Pixhawk6X (1209:5741)"),
            format!("{FTDI} - FT232R USB UART (0403:6001)"),
            "/dev/ttyACM1 - Pixhawk6X (1209:5741)".to_owned(),
            "/dev/ttyACM0 - Pixhawk6X (1209:5741)".to_owned(),
            "/dev/ttyUSB0 - FT232R USB UART (0403:6001)".to_owned(),
            "/dev/rfcomm0".to_owned(),
        ]
    );
    assert_eq!(ports[2].serial_number.as_deref(), Some("A10K3Z9B"));
    assert_eq!(ports[0].manufacturer.as_deref(), Some("ArduPilot"));
    // The join annotates; it never adds, drops or reorders.
    let names: Vec<&str> = ports.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, listing.port_names());
}

#[test]
fn macos_lists_the_usb_nodes_first_then_tty_dot_then_cu_dot_then_monos_ttys() {
    let listing = DevListing::load("macos-dev.txt");
    assert_eq!(
        listing.port_names(),
        [
            // *usb* comes before tty.* and cu.*, so the USB devices lead, in directory order.
            "/dev/cu.usbmodem14101",
            "/dev/tty.usbmodem14101",
            "/dev/cu.usbserial-A10K3Z9B",
            "/dev/tty.usbserial-A10K3Z9B",
            // tty.*: the rest of the dial-in nodes, and /dev/tty through the bare-stem rule.
            "/dev/tty.Bluetooth-Incoming-Port",
            "/dev/tty.debug-console",
            "/dev/tty",
            // cu.*: the rest of the call-out nodes.
            "/dev/cu.Bluetooth-Incoming-Port",
            "/dev/cu.debug-console",
            // Mono's GetPortNames(): no Linux-style node, so every tty* but /dev/tty and ttyC*.
            "/dev/ttys001",
            "/dev/ttys000",
            "/dev/ttyp1",
            "/dev/ttyp0",
        ]
    );
}

#[test]
fn macos_leaves_out_the_nodes_that_are_not_ports() {
    let names = DevListing::load("macos-dev.txt").port_names();
    for noise in [
        "/dev/ptmx",
        "/dev/console",
        "/dev/fd",
        "/dev/fd/",
        "/dev/disk0",
        "/dev/null",
        "/dev/random",
        "/dev/urandom",
        "/dev/zero",
    ] {
        assert!(!names.iter().any(|n| n == noise), "{noise} was listed");
    }
}

/// `reg query` output: the data of each REG_SZ value, in order.
fn serialcomm_values(text: &str) -> Vec<&str> {
    data_lines(text)
        .filter_map(|line| line.split_once("    REG_SZ    ").map(|(_, port)| port))
        .collect()
}

/// A two-column CSV as PowerShell's ConvertTo-Csv writes it, header dropped.
fn wmi_rows(text: &str) -> Vec<WmiSerialPort<'_>> {
    data_lines(text)
        .skip(1)
        .map(|line| {
            let (id, name) = line.split_once("\",\"").unwrap();
            WmiSerialPort {
                device_id: id.trim_start_matches('"'),
                name: name.trim_end_matches('"'),
            }
        })
        .collect()
}

#[test]
fn windows_lists_the_serialcomm_ports_in_registry_order() {
    let text = fixture("windows-serialcomm.txt");
    let runtime = windows_port_names(&serialcomm_values(&text));
    // No /dev on Windows, so the runtime's list is the whole list.
    assert_eq!(
        get_port_names(&[], &runtime),
        ["COM1", "COM3", "COM4", "COM10", "COM11"]
    );
}

#[test]
fn windows_shows_each_port_beside_its_wmi_friendly_name() {
    let serialcomm = fixture("windows-serialcomm.txt");
    let wmi_text = fixture("windows-win32_serialport.csv");
    let wmi = wmi_rows(&wmi_text);
    let lines: Vec<String> =
        get_port_names(&[], &windows_port_names(&serialcomm_values(&serialcomm)))
            .iter()
            .map(|port| display_text(port, &nice_name(port, &wmi)))
            .collect();
    assert_eq!(
        lines,
        [
            "COM1 Communications Port (COM1)",
            // No Win32_SerialPort row: the name is empty and the separating space stays.
            "COM3 ",
            "COM4 USB Serial Port (COM4)",
            "COM10 Standard Serial over Bluetooth link (COM10)",
            "COM11 Standard Serial over Bluetooth link (COM11)",
        ]
    );
}

#[test]
fn windows_friendly_names_match_the_device_id_ignoring_case_and_skip_the_non_ports() {
    let wmi = [
        WmiSerialPort {
            device_id: "com7",
            name: "Lower-case id (COM7)",
        },
        WmiSerialPort {
            device_id: "AUTO",
            name: "never shown",
        },
    ];
    assert_eq!(nice_name("COM7", &wmi), "Lower-case id (COM7)");
    for entry in NOT_PORTS {
        assert_eq!(nice_name(entry, &wmi), "", "{entry}");
    }
}

#[test]
fn the_bluetooth_repair_keeps_com_and_the_digits_among_the_next_three_characters() {
    // The case the C# comment names.
    assert_eq!(fix_bluetooth_port_name("COM10c"), "COM10");
    assert_eq!(fix_bluetooth_port_name("COM3"), "COM3");
    // Take(3): junk between digits is dropped, and anything past three characters is lost -
    // which truncates a port above COM999. Faithful, because the list has to match.
    assert_eq!(fix_bluetooth_port_name("COM1c2"), "COM12");
    assert_eq!(fix_bluetooth_port_name("COM1234"), "COM123");
    assert_eq!(fix_bluetooth_port_name("COM"), "COM");
    // StartsWith("COM") is case-sensitive, and nothing else is touched.
    assert_eq!(fix_bluetooth_port_name("com3x"), "com3x");
    assert_eq!(fix_bluetooth_port_name("/dev/ttyACM0"), "/dev/ttyACM0");
}

#[test]
fn runtime_names_are_trimmed_and_repaired_before_duplicates_are_removed() {
    let runtime = ["COM3", "COM3 ", "COM10c", "COM10\t", "COM4"];
    assert_eq!(get_port_names(&[], &runtime), ["COM3", "COM10", "COM4"]);
}

#[test]
fn a_port_in_both_sources_is_listed_once_where_the_globs_put_it() {
    // Mono lists ttyACM0 too; Distinct() keeps the first, the glob's.
    let entries = ["/dev/ttyS0", "/dev/ttyACM0"];
    let names = get_port_names(&entries, &mono_port_names(&entries));
    assert_eq!(names, ["/dev/ttyACM0", "/dev/ttyS0"]);
}

#[test]
fn an_empty_listing_lists_nothing() {
    // No /dev at all - a sandbox, or Windows - and no runtime ports.
    assert!(get_port_names::<&str>(&[], &[]).is_empty());
    assert!(mono_port_names(&[]).is_empty());
}

// ---------------------------------------------------------------------------------------------
// Cross-checks against the C# source, when the tree is present.
// ---------------------------------------------------------------------------------------------

/// `MP_SRC` names a clone of https://github.com/ArduPilot/MissionPlanner.
fn csharp(relative: &str) -> Option<String> {
    let Some(tree) = std::env::var_os("MP_SRC") else {
        println!("MP_SRC not set; cross-check skipped");
        return None;
    };
    let path = PathBuf::from(tree).join(relative);
    match std::fs::read_to_string(&path) {
        Ok(text) => Some(text),
        Err(_) => {
            println!("{} not present; cross-check skipped", path.display());
            None
        }
    }
}

/// The contents of every "..." literal on a line, in order.
fn string_literals(line: &str) -> Vec<&str> {
    line.split('"').skip(1).step_by(2).collect()
}

/// The text between two markers, which must both be present.
fn between<'a>(text: &'a str, start: &str, end: &str) -> &'a str {
    let from = text
        .find(start)
        .unwrap_or_else(|| panic!("{start:?} not found"));
    let rest = &text[from..];
    let to = rest
        .find(end)
        .unwrap_or_else(|| panic!("{end:?} not found"));
    &rest[..to]
}

#[test]
fn the_seven_globs_are_the_ones_in_comms_serial_port_cs() {
    let Some(source) = csharp("ExtLibs/Comms/CommsSerialPort.cs") else {
        return;
    };
    let body = between(
        &source,
        "public new static string[] GetPortNames()",
        "public static Func<List<string>> GetCustomPorts;",
    );
    let globs: Vec<(String, String)> = body
        .lines()
        .filter(|line| line.contains("Directory.GetFiles("))
        .map(|line| {
            let literals = string_literals(line);
            assert_eq!(literals.len(), 2, "{line}");
            (literals[0].to_owned(), literals[1].to_owned())
        })
        .collect();
    let ours: Vec<(String, String)> = UNIX_GLOBS
        .iter()
        .map(|(dir, pattern)| ((*dir).to_owned(), (*pattern).to_owned()))
        .collect();
    assert_eq!(globs, ours);

    // And the runtime's names are trimmed, repaired, and everything de-duplicated.
    assert!(body.contains("p?.TrimEnd()"));
    assert!(body.contains("Select(FixBlueToothPortNameBug)"));
    assert!(body.contains("allPorts.Distinct()"));
}

#[test]
fn the_bluetooth_repair_and_the_non_port_entries_match_comms_serial_port_cs() {
    let Some(source) = csharp("ExtLibs/Comms/CommsSerialPort.cs") else {
        return;
    };
    let repair = between(
        &source,
        "private static string FixBlueToothPortNameBug",
        "return newPortName;",
    );
    assert!(repair.contains("StartsWith(\"COM\")"));
    assert!(repair.contains("Substring(3).Take(3)"));
    assert!(repair.contains("char.IsDigit"));

    let nice = between(
        &source,
        "public static string GetNiceName",
        "comportnamecache.ContainsKey",
    );
    let line = nice
        .lines()
        .find(|line| line.contains("port == \"AUTO\""))
        .expect("the non-port check");
    assert_eq!(string_literals(line), NOT_PORTS);
}

#[test]
fn monos_rules_match_the_copy_in_the_csharp_tree() {
    let Some(source) = csharp("ExtLibs/Comms/System.IO.Ports/SerialPort.cs") else {
        return;
    };
    let body = between(
        &source,
        "public static string [] GetPortNames ()",
        "static bool IsWindows",
    );
    let literals: Vec<&str> = body.lines().flat_map(string_literals).collect();
    assert_eq!(
        literals,
        [
            "/dev/",
            "tty*",
            "/dev/ttyS",
            "/dev/ttyUSB",
            "/dev/ttyACM",
            "/dev/ttyS",
            "/dev/ttyUSB",
            "/dev/ttyACM",
            "/dev/tty",
            "/dev/tty",
            "/dev/ttyC",
            "HARDWARE\\\\DEVICEMAP\\\\SERIALCOMM",
            "",
            "",
        ]
    );
}

#[test]
fn the_friendly_name_comes_from_win32_serial_port_matched_on_device_id() {
    let Some(source) = csharp("Program.cs") else {
        return;
    };
    let query = between(
        &source,
        "private static string SerialPort_GetDeviceName",
        "private static void LoadDlls",
    );
    assert!(query.contains("SELECT * FROM Win32_SerialPort"));
    assert!(
        query.contains("Properties[\"DeviceID\"].Value.ToString().ToUpper() == port.ToUpper()")
    );
    assert!(query.contains("Properties[\"Name\"]"));

    let Some(control) = csharp("Controls/ConnectionControl.cs") else {
        return;
    };
    assert!(control.contains("text = text + \" \" + SerialPort.GetNiceName(text);"));
}

#[cfg(all(unix, feature = "serial"))]
#[test]
fn list_ports_on_this_machine_has_no_duplicates_and_leads_with_the_stable_names() {
    // Read-only: enumerating stats /dev and never opens a port.
    let ports = mp_transport::list_ports();
    for port in &ports {
        assert!(!port.name.is_empty());
        assert!(!port.label().is_empty());
    }
    let names: Vec<&str> = ports.iter().map(|p| p.name.as_str()).collect();
    let mut unique = names.clone();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(unique.len(), names.len(), "duplicates in {names:#?}");
    // Stable names lead whenever there are any.
    if let Some(first_other) = names
        .iter()
        .position(|n| !n.starts_with("/dev/serial/by-id/"))
    {
        assert!(
            names[first_other..]
                .iter()
                .all(|n| !n.starts_with("/dev/serial/by-id/")),
            "{names:#?}"
        );
    }
}
