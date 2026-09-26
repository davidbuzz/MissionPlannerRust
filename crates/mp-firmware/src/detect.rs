//! Which board is on the other end of a port: `BoardDetect.DetectBoard`.
//!
//! Ported from `Utilities/BoardDetect.cs` @ efb0801 (GPL-3.0-or-later), with the device record it
//! is handed (`ExtLibs/ArduPilot/DeviceInfo.cs`) and the bootloader identify it calls
//! (`ExtLibs/px4uploader/Uploader.cs:867-893`).
//!
//! # What the C# does, in order
//!
//! `DetectBoard(port, ports)` is one five-hundred-line method. Stripped of its logging it makes six
//! decisions, and the first one that answers is the result:
//!
//! 1. Under Mono - Mission Planner on Linux and macOS - it reads nothing. It asks the operator a
//!    chain of yes/no questions and returns the answer ([`mono_questions`]). Everything below is
//!    the Windows path.
//! 2. The device list: each device's USB hardware id and bus-reported product string, first match
//!    wins ([`match_devices`]).
//! 3. WMI's `Win32_SerialPort` table: each row's PNP device id, first match wins
//!    ([`match_serial_ports`]).
//! 4. "Is this a Linux board?", for a Bebop2 or a Disco ([`linux_questions`]).
//! 5. The port itself: an STK500v1 sync at 57600 baud finds an APM1 1280, then an STK500v2 command
//!    at 115200 finds a 2560, and WMI decides whether that 2560 is an APM 2 ([`stk500v2_board`]).
//! 6. More questions ([`last_questions`]).
//!
//! Steps 2 and 3 can instead say "unplug the board and plug it back in". Mission Planner then
//! tries every port for thirty seconds with the px4 bootloader's identify, and the board id, flash
//! size and bootloader revision it reads decide the board ([`bootloader_board`]).
//!
//! Every decision over data is a pure function here, tested against fixtures in
//! `testdata/boards/`. What needs a person or a port goes through [`DetectHost`], and
//! [`detect_board`] runs the six steps over it in the C#'s order. The live probes speak through
//! [`TransportPort`], an `mp-transport` link, so the same code runs against the px4 mock in the
//! tests and against [`open_serial`] on hardware. Nothing here writes to flash: the probes send
//! sync, identify and load-address commands and nothing else.
//!
//! # Two things the C# does not do that a caller might expect
//!
//! - It never compares a device with the port it was asked about. The comparison is commented out
//!   (`BoardDetect.cs:76`), so the first recognised device on the machine wins, whichever port the
//!   operator picked. Only the APM 2 rules read the port name.
//! - Its own test cases do not all pass against it. `testdata/boards/detect_board_tests.json`
//!   records, case by case, what the C# test asserts and what `BoardDetect.cs` returns.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use mp_transport::{PortInfo, Transport};

use crate::uploader::{Board, Uploader, UploaderError};

/// The boards `DetectBoard` can name, with the C#'s names and values.
///
/// `// C#: Utilities/BoardDetect.cs:18-49`. The values are the C# enum's implicit ones, zero up in
/// declaration order, because Mission Planner stores and logs them as numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Boards {
    /// `none`: nothing was recognised. The C# shows `CantDetectBoardVersion` and stops.
    None = 0,
    /// `b1280`: APM1 with an ATmega1280.
    B1280 = 1,
    /// `b2560`: APM1 with an ATmega2560.
    B2560 = 2,
    /// `b2560v2`: APM 2 and later.
    B2560v2 = 3,
    /// `px4`: PX4 FMU v1.
    Px4 = 4,
    /// `px4rl`.
    Px4rl = 5,
    /// `px4v2`: Pixhawk.
    Px4v2 = 6,
    /// `px4v3`: Cube, or a Pixhawk with 2 MB of flash.
    Px4v3 = 7,
    /// `px4v4`: Pixracer.
    Px4v4 = 8,
    /// `px4v4pro`: Pixhawk 3 Pro.
    Px4v4pro = 9,
    /// `fmuv5`: PixHack v5 and Pixhawk 4.
    Fmuv5 = 10,
    /// `vrbrainv40`.
    Vrbrainv40 = 11,
    /// `vrbrainv45`.
    Vrbrainv45 = 12,
    /// `vrbrainv50`.
    Vrbrainv50 = 13,
    /// `vrbrainv51`.
    Vrbrainv51 = 14,
    /// `vrbrainv52`.
    Vrbrainv52 = 15,
    /// `vrbrainv54`.
    Vrbrainv54 = 16,
    /// `vrcorev10`.
    Vrcorev10 = 17,
    /// `vrubrainv51`.
    Vrubrainv51 = 18,
    /// `vrubrainv52`.
    Vrubrainv52 = 19,
    /// `bebop2`.
    Bebop2 = 20,
    /// `disco`.
    Disco = 21,
    /// `solo`.
    Solo = 22,
    /// `revomini`.
    Revomini = 23,
    /// `mindpxv2`.
    Mindpxv2 = 24,
    /// `minipix`.
    Minipix = 25,
    /// `chbootloader`: an ArduPilot (ChibiOS) bootloader, which names its own board. The name is
    /// in [`Detected::chbootloader`].
    Chbootloader = 26,
    /// `pass`.
    Pass = 27,
    /// `nxpfmuk66`: NXP RDDRONE-FMUK66.
    Nxpfmuk66 = 28,
}

impl Boards {
    /// Every board, in the C#'s declaration order.
    pub const ALL: [Self; 29] = [
        Self::None,
        Self::B1280,
        Self::B2560,
        Self::B2560v2,
        Self::Px4,
        Self::Px4rl,
        Self::Px4v2,
        Self::Px4v3,
        Self::Px4v4,
        Self::Px4v4pro,
        Self::Fmuv5,
        Self::Vrbrainv40,
        Self::Vrbrainv45,
        Self::Vrbrainv50,
        Self::Vrbrainv51,
        Self::Vrbrainv52,
        Self::Vrbrainv54,
        Self::Vrcorev10,
        Self::Vrubrainv51,
        Self::Vrubrainv52,
        Self::Bebop2,
        Self::Disco,
        Self::Solo,
        Self::Revomini,
        Self::Mindpxv2,
        Self::Minipix,
        Self::Chbootloader,
        Self::Pass,
        Self::Nxpfmuk66,
    ];

    /// The C# identifier, as `board.ToString()` prints it and Mission Planner logs it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::B1280 => "b1280",
            Self::B2560 => "b2560",
            Self::B2560v2 => "b2560v2",
            Self::Px4 => "px4",
            Self::Px4rl => "px4rl",
            Self::Px4v2 => "px4v2",
            Self::Px4v3 => "px4v3",
            Self::Px4v4 => "px4v4",
            Self::Px4v4pro => "px4v4pro",
            Self::Fmuv5 => "fmuv5",
            Self::Vrbrainv40 => "vrbrainv40",
            Self::Vrbrainv45 => "vrbrainv45",
            Self::Vrbrainv50 => "vrbrainv50",
            Self::Vrbrainv51 => "vrbrainv51",
            Self::Vrbrainv52 => "vrbrainv52",
            Self::Vrbrainv54 => "vrbrainv54",
            Self::Vrcorev10 => "vrcorev10",
            Self::Vrubrainv51 => "vrubrainv51",
            Self::Vrubrainv52 => "vrubrainv52",
            Self::Bebop2 => "bebop2",
            Self::Disco => "disco",
            Self::Solo => "solo",
            Self::Revomini => "revomini",
            Self::Mindpxv2 => "mindpxv2",
            Self::Minipix => "minipix",
            Self::Chbootloader => "chbootloader",
            Self::Pass => "pass",
            Self::Nxpfmuk66 => "nxpfmuk66",
        }
    }

    /// The C# enum's numeric value.
    #[must_use]
    pub const fn value(self) -> u8 {
        self as u8
    }

    /// The board a C# identifier names, exactly as spelled.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|board| board.name() == name)
    }
}

impl std::fmt::Display for Boards {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// What `DetectBoard` returns, with the static it sets on the way.
///
/// The C# returns the enum and leaves the ChibiOS board name in a static,
/// `BoardDetect.chbootloader` (`BoardDetect.cs:51`), which `Firmware.cs:502` substitutes into the
/// firmware URL. Here it travels with the result instead: it is `Some` exactly when this call is
/// one that assigned the static.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Detected {
    /// The board.
    pub board: Boards,
    /// The ChibiOS bootloader's board name, when the C# set `BoardDetect.chbootloader`.
    pub chbootloader: Option<String>,
}

impl Detected {
    /// A board with no ChibiOS name.
    #[must_use]
    pub const fn board(board: Boards) -> Self {
        Self {
            board,
            chbootloader: None,
        }
    }

    /// A ChibiOS bootloader naming itself.
    #[must_use]
    pub fn chbootloader(name: impl Into<String>) -> Self {
        Self {
            board: Boards::Chbootloader,
            chbootloader: Some(name.into()),
        }
    }
}

/// One device as Mission Planner lists it: `MissionPlanner.ArduPilot.DeviceInfo`.
///
/// `// C#: ExtLibs/ArduPilot/DeviceInfo.cs:3-21`. A C# struct of four strings, any of which can be
/// null. Only `board` and `hardwareid` are read by the rules; a null one of those where the C#
/// dereferences it throws, and [`match_devices`] reproduces what the catch around it does.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceInfo {
    /// `name`: the COM port name on Windows (`Win32DeviceMgnt.cs:447`).
    pub name: Option<String>,
    /// `description`: the Windows device description, `SPDRP_DEVICEDESC` (`Win32DeviceMgnt.cs:464`).
    pub description: Option<String>,
    /// `board`: the USB product string, `DEVPKEY_Device_BusReportedDeviceDesc`
    /// (`Win32DeviceMgnt.cs:513-549`); on Linux the sysfs `product` file (`Linux.cs:33-37`).
    pub board: Option<String>,
    /// `hardwareid`: `USB\VID_xxxx&PID_xxxx...`, `SPDRP_HARDWAREID` (`Win32DeviceMgnt.cs:476`).
    pub hardwareid: Option<String>,
}

impl DeviceInfo {
    /// A device with a product string and a hardware id, the two fields the rules read.
    #[must_use]
    pub fn new(board: &str, hardwareid: &str) -> Self {
        Self {
            name: Some(String::new()),
            description: Some(String::new()),
            board: Some(board.to_owned()),
            hardwareid: Some(hardwareid.to_owned()),
        }
    }

    /// The record Windows would give Mission Planner for an enumerated port.
    ///
    /// On Windows the port list carries what `Win32DeviceMgmt` reads, and this takes it as it is:
    /// `hardwareid` Windows' `SPDRP_HARDWAREID` (`USB\VID_2DAE&PID_1016&REV_0200&MI_00`), `board`
    /// the bus-reported name, `description` `SPDRP_DEVICEDESC`. Elsewhere `hardwareid` takes
    /// Windows' form built from the VID and PID, upper-case hex (`USB\VID_2DAE&PID_1016`), because
    /// the rules that read it were written against it, and `description` is the product string, as
    /// `Linux.cs:38` fills it.
    ///
    /// A port with no USB ids is not in the list at all (`None`). Neither of Mission Planner's
    /// lists holds a device without a hardware id: `Linux.cs:23` finds only USB ttys, and
    /// `Win32DeviceMgnt.cs:474-484` skips a device whose id cannot be read, while a serial port that
    /// is not USB has a non-USB id no rule matches. Passing it as a null id instead would make the
    /// C#'s null handling drop every device after it.
    ///
    /// Mission Planner's own Linux list (`Linux.cs:40`) writes the ids in lower case, from sysfs.
    /// It never reaches the rules, because under Mono `DetectBoard` asks questions instead; were it
    /// to, `VID_2dae` would not match the case-sensitive `VID_2DAE` rule.
    #[must_use]
    pub fn from_port(port: &PortInfo) -> Option<Self> {
        let (Some(vid), Some(pid)) = (port.vid, port.pid) else {
            return None;
        };
        // On Windows, what `Win32DeviceMgmt` reads (mp-transport's `win32.rs`): `SPDRP_DEVICEDESC`,
        // the bus-reported name as the board, and `SPDRP_HARDWAREID` - whose `&REV_...&MI_...`
        // after the PID is what `vid_pid`'s `VID_..&PID_..&` pattern needs. Elsewhere the
        // product for both, and the id built from the VID and PID.
        // `// C#: Utilities/Win32DeviceMgnt.cs:462-549`
        Some(Self {
            name: Some(port.name.clone()),
            description: port.description.clone().or_else(|| port.product.clone()),
            board: port.product.clone(),
            hardwareid: Some(
                port.hardware_id
                    .clone()
                    .unwrap_or_else(|| hardware_id(vid, pid)),
            ),
        })
    }
}

/// A USB hardware id as Windows spells it: `USB\VID_26AC&PID_0011`.
#[must_use]
pub fn hardware_id(vid: u16, pid: u16) -> String {
    format!(r"USB\VID_{vid:04X}&PID_{pid:04X}")
}

/// One row of WMI's `Win32_SerialPort`, the two properties `DetectBoard` reads.
///
/// `// C#: Utilities/BoardDetect.cs:199-201` (`SELECT * FROM Win32_SerialPort`), `:213-216`
/// (`PNPDeviceID`, `Name`). `mp_transport::enumerate::WmiSerialPort` carries `DeviceID` and
/// `Name` for the connection list's friendly names; this needs the PNP id instead.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Win32SerialPort {
    /// `PNPDeviceID`: `USB\VID_26AC&PID_0032\0`.
    pub pnp_device_id: String,
    /// `Name`: the friendly name, which ends `(COM3)`.
    pub name: String,
}

impl Win32SerialPort {
    /// A row.
    #[must_use]
    pub fn new(pnp_device_id: &str, name: &str) -> Self {
        Self {
            pnp_device_id: pnp_device_id.to_owned(),
            name: name.to_owned(),
        }
    }

    /// The row Windows would list for an enumerated port, built from what `PortInfo` knows.
    ///
    /// The PNP id is the hardware id, then the serial number as Windows appends it. `PortInfo` has
    /// no Windows friendly name, so `Name` is the product string and the port in brackets, which is
    /// the shape Windows gives it and the only part of it the rules read.
    #[must_use]
    pub fn from_port(port: &PortInfo) -> Self {
        let pnp_device_id = match (port.vid, port.pid) {
            (Some(vid), Some(pid)) => format!(
                r"{}\{}",
                hardware_id(vid, pid),
                port.serial_number.as_deref().unwrap_or_default()
            ),
            _ => String::new(),
        };
        let name = match &port.product {
            Some(product) => format!("{product} ({})", port.name),
            None => format!("({})", port.name),
        };
        Self {
            pnp_device_id,
            name,
        }
    }
}

/// What one of the rule lists concluded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict {
    /// A rule named the board.
    Board(Detected),
    /// A rule named a bootloader to look for: the C# asks for a replug and probes every port.
    Probe(Probe),
    /// No rule matched, and the C# goes on to its next step.
    NoMatch,
}

/// Which replug-and-probe the C# runs. The two copies differ in what they can conclude.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Probe {
    /// From the device list, a board string of `PX4 FMU v2.x` or `PX4 FMU v3.x`: `px4v3` or
    /// `px4v2`. `// C#: Utilities/BoardDetect.cs:148-190`
    FmuV2OrV3,
    /// From WMI, a ChibiOS or px4 bootloader's USB id: `px4v3`, `fmuv5` or `px4v2`.
    /// `// C#: Utilities/BoardDetect.cs:241-291`
    ChibiosOrPx4,
}

/// Whether a hardware id is one of the new-style (ArduPilot ChibiOS) bootloaders.
///
/// `// C#: Utilities/BoardDetect.cs:78-85`. Two prefixes and two substrings, all case-sensitive:
/// `StartsWith` and `Regex.IsMatch` with a pattern that has no special characters.
#[must_use]
pub fn is_new_style_bootloader(hardwareid: &str) -> bool {
    hardwareid.starts_with(r"USB\VID_0483&PID_5740")
        || hardwareid.contains("VID_2DAE")
        || hardwareid.contains("VID_3162")
        || hardwareid.starts_with(r"USB\VID_1209&PID_5740")
}

/// The new-style bootloaders that name a board the C# maps to an enum value of its own.
///
/// `// C#: Utilities/BoardDetect.cs:87-103`. The C# also recognises `fmuv5`, `revo-mini`,
/// `mini-pix` and `mindpx-v2` (`:105-126`), but only logs them - their returns are commented out -
/// so they fall through to the ChibiOS result below with everything else.
const NEW_STYLE_BOARDS: [(&str, Boards); 3] = [
    ("fmuv2", Boards::Px4v2),
    ("fmuv3", Boards::Px4v3),
    ("fmuv4", Boards::Px4v4),
];

/// The old-style (px4 bootloader) product strings, matched exactly, in the C#'s order.
///
/// `// C#: Utilities/BoardDetect.cs:132-147`
const OLD_STYLE_BOARDS: [(&str, Boards); 3] = [
    ("PX4 FMU v5.x", Boards::Fmuv5),
    ("PX4 FMU v4.x", Boards::Px4v4),
    ("PX4 FMU v1.x", Boards::Px4),
];

/// Step 2: the device list.
///
/// `// C#: Utilities/BoardDetect.cs:66-197`. For each device in order, first match wins:
///
/// - a new-style bootloader id with product `fmuv2`, `fmuv3` or `fmuv4` - exactly, or the same
///   with `-bl` in any case - is `px4v2`, `px4v3` or `px4v4`;
/// - any other product on a new-style id is `chbootloader`, named by the product string with
///   `-bl`, `-Bl` and `-BL` removed;
/// - `PX4 FMU v5.x`, `v4.x` and `v1.x`, on any id, are `fmuv5`, `px4v4` and `px4`;
/// - `PX4 FMU v2.x` and `v3.x` need the bootloader probed ([`Probe::FmuV2OrV3`]).
///
/// The whole loop sits in a `try` whose `catch` goes on to WMI. A device with no hardware id, or a
/// new-style one with no product string, throws a `NullReferenceException` there, so the rest of
/// the list is never looked at: that is [`Verdict::NoMatch`] here, not a skip to the next device.
#[must_use]
pub fn match_devices(devices: &[DeviceInfo]) -> Verdict {
    for item in devices {
        // C#: BoardDetect.cs:82 - `item.hardwareid.StartsWith` on a null id throws.
        let Some(hardwareid) = item.hardwareid.as_deref() else {
            return Verdict::NoMatch;
        };
        if is_new_style_bootloader(hardwareid) {
            // C#: BoardDetect.cs:87 - `item.board.ToLower()` on a null board throws.
            let Some(board) = item.board.as_deref() else {
                return Verdict::NoMatch;
            };
            let lower = board.to_lowercase();
            // C#: BoardDetect.cs:87-103
            for (name, value) in NEW_STYLE_BOARDS {
                if board == name || lower == format!("{name}-bl") {
                    return Verdict::Board(Detected::board(value));
                }
            }
            // C#: BoardDetect.cs:128-129
            let named = board
                .replace("-bl", "")
                .replace("-Bl", "")
                .replace("-BL", "");
            return Verdict::Board(Detected::chbootloader(named));
        }
        // C#: BoardDetect.cs:132-147 - `==` on strings, which is null-safe.
        let board = item.board.as_deref();
        if let Some((_, value)) = OLD_STYLE_BOARDS
            .iter()
            .find(|(name, _)| board == Some(*name))
        {
            return Verdict::Board(Detected::board(*value));
        }
        // C#: BoardDetect.cs:148
        if matches!(board, Some("PX4 FMU v2.x" | "PX4 FMU v3.x")) {
            return Verdict::Probe(Probe::FmuV2OrV3);
        }
    }
    Verdict::NoMatch
}

/// What a `Win32_SerialPort` row whose PNP id matches a rule concludes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialPortAction {
    /// `b2560v2`, but only when the row's `Name` contains the port being asked about; otherwise
    /// this row is done and the next row is tried.
    Apm2OnThisPort,
    /// This board.
    Board(Boards),
    /// A ChibiOS bootloader with this fixed name.
    Chbootloader(&'static str),
    /// A replug and a bootloader probe.
    Probe(Probe),
}

/// A `Win32_SerialPort` rule: any of these substrings of `PNPDeviceID`, and what it concludes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SerialPortRule {
    /// Substrings of `PNPDeviceID`, any one of which matches.
    pub ids: &'static [&'static str],
    /// What a match concludes.
    pub action: SerialPortAction,
}

/// The `Win32_SerialPort` rules, in the order the C#'s `if`/`else if` chain tests them.
///
/// `// C#: Utilities/BoardDetect.cs:212-367`. Every comparison is `String.Contains`, ordinal and
/// case-sensitive. The commented-out `26AC` alternatives at `:316` are not rules.
pub const SERIAL_PORT_RULES: [SerialPortRule; 19] = [
    // C#: BoardDetect.cs:213-221
    SerialPortRule {
        ids: &[r"USB\VID_2341&PID_0010"],
        action: SerialPortAction::Apm2OnThisPort,
    },
    // C#: BoardDetect.cs:222-230 - its port-name check is commented out.
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0010"],
        action: SerialPortAction::Board(Boards::Px4),
    },
    // C#: BoardDetect.cs:231-240
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0032"],
        action: SerialPortAction::Chbootloader("CUAVv5"),
    },
    // C#: BoardDetect.cs:241-291 - "chibios or normal px4"
    SerialPortRule {
        ids: &[
            r"USB\VID_0483&PID_5740",
            r"USB\VID_26AC&PID_0011",
            r"USB\VID_1209&PID_5740",
        ],
        action: SerialPortAction::Probe(Probe::ChibiosOrPx4),
    },
    // C#: BoardDetect.cs:292-296
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0021"],
        action: SerialPortAction::Board(Boards::Px4v3),
    },
    // C#: BoardDetect.cs:297-301
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0012"],
        action: SerialPortAction::Board(Boards::Px4v4),
    },
    // C#: BoardDetect.cs:302-306
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0013"],
        action: SerialPortAction::Board(Boards::Px4v4pro),
    },
    // C#: BoardDetect.cs:307-311
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0001"],
        action: SerialPortAction::Board(Boards::Px4v2),
    },
    // C#: BoardDetect.cs:312-315
    SerialPortRule {
        ids: &[r"USB\VID_26AC&PID_0016"],
        action: SerialPortAction::Board(Boards::Px4rl),
    },
    // C#: BoardDetect.cs:318-322
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1140"],
        action: SerialPortAction::Board(Boards::Vrbrainv40),
    },
    // C#: BoardDetect.cs:323-327
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1145"],
        action: SerialPortAction::Board(Boards::Vrbrainv45),
    },
    // C#: BoardDetect.cs:328-332
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1150"],
        action: SerialPortAction::Board(Boards::Vrbrainv50),
    },
    // C#: BoardDetect.cs:333-337
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1151"],
        action: SerialPortAction::Board(Boards::Vrbrainv51),
    },
    // C#: BoardDetect.cs:338-342
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1152"],
        action: SerialPortAction::Board(Boards::Vrbrainv52),
    },
    // C#: BoardDetect.cs:343-347
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1154"],
        action: SerialPortAction::Board(Boards::Vrbrainv54),
    },
    // C#: BoardDetect.cs:348-352
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1910"],
        action: SerialPortAction::Board(Boards::Vrcorev10),
    },
    // C#: BoardDetect.cs:353-357
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1351"],
        action: SerialPortAction::Board(Boards::Vrubrainv51),
    },
    // C#: BoardDetect.cs:358-362
    SerialPortRule {
        ids: &[r"USB\VID_27AC&PID_1352"],
        action: SerialPortAction::Board(Boards::Vrubrainv52),
    },
    // C#: BoardDetect.cs:363-367
    SerialPortRule {
        ids: &[r"USB\VID_1FC9&PID_001C"],
        action: SerialPortAction::Board(Boards::Nxpfmuk66),
    },
];

/// Whether a row's friendly name is the port's: `Name.ToUpper().Contains(PortName.ToUpper())`.
///
/// A substring test, so `COM1` is found in `(COM10)` as well. `// C#: Utilities/BoardDetect.cs:216`
fn names_port(row: &Win32SerialPort, port: &str) -> bool {
    row.name.to_uppercase().contains(&port.to_uppercase())
}

/// Step 3: the `Win32_SerialPort` table.
///
/// `// C#: Utilities/BoardDetect.cs:199-368`. For each row in order, the first rule in
/// [`SERIAL_PORT_RULES`] whose id the row's PNP id contains decides it. An Arduino Mega id on a
/// row for another port decides nothing, and the next row is tried.
#[must_use]
pub fn match_serial_ports(port: &str, rows: &[Win32SerialPort]) -> Verdict {
    for row in rows {
        let Some(rule) = SERIAL_PORT_RULES
            .iter()
            .find(|rule| rule.ids.iter().any(|id| row.pnp_device_id.contains(id)))
        else {
            continue;
        };
        match rule.action {
            SerialPortAction::Apm2OnThisPort => {
                if names_port(row, port) {
                    return Verdict::Board(Detected::board(Boards::B2560v2));
                }
            }
            SerialPortAction::Board(board) => return Verdict::Board(Detected::board(board)),
            SerialPortAction::Chbootloader(name) => {
                return Verdict::Board(Detected::chbootloader(name));
            }
            SerialPortAction::Probe(probe) => return Verdict::Probe(probe),
        }
    }
    Verdict::NoMatch
}

/// Steps 2 and 3 over one enumeration, without opening anything or asking anyone.
///
/// The device list and the `Win32_SerialPort` table are both built from `ports`
/// ([`DeviceInfo::from_port`], [`Win32SerialPort::from_port`]). This is the part of `DetectBoard`
/// that is a function of the USB descriptors alone; [`detect_board`] runs it and then the steps
/// that need a person or a port.
#[must_use]
pub fn match_ports(port: &str, ports: &[PortInfo]) -> Verdict {
    let devices: Vec<DeviceInfo> = ports.iter().filter_map(DeviceInfo::from_port).collect();
    match match_devices(&devices) {
        Verdict::NoMatch => {}
        decided => return decided,
    }
    let rows: Vec<Win32SerialPort> = ports.iter().map(Win32SerialPort::from_port).collect();
    match_serial_ports(port, &rows)
}

/// The largest firmware a 2 MB STM32F4 px4 bootloader reports, which the C# reads as a Cube.
///
/// `// C#: Utilities/BoardDetect.cs:169, 265, 270`
pub const FLASH_SIZE_2MB: usize = 2_080_768;

/// What a bootloader's identify means, for each of the two probes.
///
/// `// C#: Utilities/BoardDetect.cs:169-178` ([`Probe::FmuV2OrV3`]) and `:265-279`
/// ([`Probe::ChibiosOrPx4`]). A board id of 9 (`fmuv2`, `Uploader.cs:246`) with 2,080,768 bytes of
/// flash and a revision 5 or later bootloader is `px4v3`; under the WMI probe, board id 50 with
/// the same is `fmuv5`; anything else that answers is `px4v2`.
#[must_use]
pub fn bootloader_board(probe: Probe, board: &Board) -> Boards {
    let big_modern = board.flash_size == FLASH_SIZE_2MB && board.bootloader_revision >= 5;
    if big_modern && board.board_id == 9 {
        Boards::Px4v3
    } else if probe == Probe::ChibiosOrPx4 && big_modern && board.board_id == 50 {
        Boards::Fmuv5
    } else {
        Boards::Px4v2
    }
}

/// The APM 2 test after an STK500v2 answer: `b2560v2` when a `Win32_SerialPort` row with an
/// Arduino Mega id names the port, `b2560` otherwise.
///
/// `// C#: Utilities/BoardDetect.cs:489-523`
#[must_use]
pub fn stk500v2_board(port: &str, rows: &[Win32SerialPort]) -> Boards {
    let apm2 = rows
        .iter()
        .any(|row| row.pnp_device_id.contains(r"USB\VID_2341&PID_0010") && names_port(row, port));
    if apm2 { Boards::B2560v2 } else { Boards::B2560 }
}

/// STK500v1 `Cmnd_STK_GET_SYNC`, `Sync_CRC_EOP`: `"0 "`. `// C#: Utilities/BoardDetect.cs:442`
pub const STK500V1_GET_SYNC: [u8; 2] = [b'0', b' '];
/// STK500v1 `Resp_STK_INSYNC`, `Resp_STK_OK`. `// C#: Utilities/BoardDetect.cs:451`
pub const STK500V1_IN_SYNC_OK: [u8; 2] = [0x14, 0x10];
/// The STK500v2 message the C# sends: `CMD_LOAD_ADDRESS` to address zero.
/// `// C#: Utilities/BoardDetect.cs:476`
pub const STK500V2_LOAD_ADDRESS: [u8; 5] = [0x06, 0, 0, 0, 0];
/// The STK500v2 reply that means a 2560 answered: the command echoed, then `STATUS_CMD_OK`.
/// `// C#: Utilities/BoardDetect.cs:483`
pub const STK500V2_OK: [u8; 2] = [0x06, 0x00];
/// STK500v2 `MESSAGE_START`. `// C#: Utilities/BoardDetect.cs:573`
const STK500V2_START: u8 = 0x1b;
/// STK500v2 `TOKEN`. `// C#: Utilities/BoardDetect.cs:581`
const STK500V2_TOKEN: u8 = 0x0e;

/// Frames an STK500v2 message: start, sequence 1, length big-endian, token, message, XOR checksum.
///
/// `// C#: Utilities/BoardDetect.cs:568-594` (`genstkv2packet`). The sequence number is always 1.
/// The C# builds it in a 300-byte buffer, so it cannot frame a message longer than 294 bytes; the
/// only message it ever sends is five.
#[must_use]
pub fn stk500v2_packet(message: &[u8]) -> Vec<u8> {
    let length = u16::try_from(message.len())
        .unwrap_or(u16::MAX)
        .to_be_bytes();
    let mut data = Vec::with_capacity(message.len() + 6);
    data.extend_from_slice(&[STK500V2_START, 0x01, length[0], length[1], STK500V2_TOKEN]);
    data.extend_from_slice(message);
    let checksum = data.iter().fold(0u8, |check, byte| check ^ byte);
    data.push(checksum);
    data
}

/// Reads one STK500v2 reply and returns its message.
///
/// `// C#: Utilities/BoardDetect.cs:612-673` (`readpacket`), transliterated. Bytes before a
/// `MESSAGE_START` are skipped; the length field sizes the message; the checksum byte is read and
/// not checked. A read that fails - a timeout - ends it early and returns what was filled in so
/// far, which is `[0x00, 0xC0]` if the length was never reached. The C# reads into a 4000-byte
/// buffer inside the `try`, so running off its end also just ends the read.
pub fn read_stk500v2_packet(port: &mut impl Read) -> Vec<u8> {
    const BUFFER: usize = 4000;
    let mut temp = [0u8; BUFFER];
    let mut message = vec![0x00, 0xC0];
    let mut wanted = 7usize;
    let mut count = 0usize;

    while count < wanted {
        let Ok(byte) = read_byte(port) else {
            break;
        };
        let Some(slot) = temp.get_mut(count) else {
            break;
        };
        *slot = byte;
        // C#: BoardDetect.cs:636 - tests temp[0], not the byte just read, so once the start byte
        // is in place nothing resets the count.
        if temp.first() != Some(&STK500V2_START) {
            count = 0;
            continue;
        }
        if count == 3 {
            let high = usize::from(temp.get(2).copied().unwrap_or(0));
            let low = usize::from(temp.get(3).copied().unwrap_or(0));
            let length = (high << 8) + low;
            message = vec![0; length];
            wanted = length + 5;
        }
        if count >= 5
            && let Some(slot) = message.get_mut(count - 5)
        {
            *slot = byte;
        }
        count += 1;
    }
    // C#: BoardDetect.cs:657-664 - the checksum, read and discarded; a failure is ignored.
    let _ = read_byte(port);
    message
}

/// Reads one byte, as `ReadByte` does.
fn read_byte(port: &mut impl Read) -> io::Result<u8> {
    let mut byte = [0u8; 1];
    port.read_exact(&mut byte)?;
    Ok(byte[0])
}

/// The operations `DetectBoard` needs from a serial port beyond reading and writing.
///
/// Reads must fail, not return zero, when the read timeout passes with nothing to read, as
/// `SerialPort.Read` throws `TimeoutException`.
pub trait ProbePort: Read + Write {
    /// `SerialPort.ReadTimeout`.
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()>;
    /// `SerialPort.DiscardInBuffer()`: drops whatever has arrived and not been read.
    fn discard_in_buffer(&mut self) -> io::Result<()>;
    /// `SerialPort.BytesToRead`: how many bytes can be read now without waiting.
    fn bytes_to_read(&mut self) -> io::Result<usize>;
}

/// An `mp-transport` link as a [`ProbePort`]: the probes run over the same transport the rest of
/// the application uses.
///
/// A [`Transport`] reports an idle read as `Ok(0)`; here that becomes a `TimedOut` error, which is
/// what the C#'s reads throw and what `read_exact` needs to stop waiting.
#[derive(Debug)]
pub struct TransportPort<T> {
    transport: T,
    /// Bytes read ahead by [`ProbePort::bytes_to_read`] and not yet handed out.
    pending: VecDeque<u8>,
    /// The read timeout in force, restored after a look-ahead read.
    timeout: Duration,
}

impl<T: Transport> TransportPort<T> {
    /// Wraps an open link. Its read timeout is assumed to be the transport default until set.
    pub const fn new(transport: T) -> Self {
        Self {
            transport,
            pending: VecDeque::new(),
            timeout: mp_transport::DEFAULT_READ_TIMEOUT,
        }
    }

    /// Gives the link back.
    pub fn into_inner(self) -> T {
        self.transport
    }

    /// Takes whatever has already arrived into `pending`, waiting as little as the link allows.
    fn look_ahead(&mut self) -> io::Result<()> {
        const NOW: Duration = Duration::from_millis(1);
        self.transport.set_read_timeout(NOW)?;
        let mut buffer = [0u8; 4096];
        let read = self.transport.read(&mut buffer);
        self.transport.set_read_timeout(self.timeout)?;
        let count = read?;
        self.pending
            .extend(buffer.iter().take(count.min(buffer.len())));
        Ok(())
    }
}

impl<T: Transport> Read for TransportPort<T> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.pending.is_empty() {
            let count = self.transport.read(buf)?;
            if count == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "nothing arrived before the read timeout",
                ));
            }
            return Ok(count);
        }
        let count = buf.len().min(self.pending.len());
        for (slot, byte) in buf.iter_mut().zip(self.pending.drain(..count)) {
            *slot = byte;
        }
        Ok(count)
    }
}

impl<T: Transport> Write for TransportPort<T> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.transport.write_all(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<T: Transport> ProbePort for TransportPort<T> {
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.timeout = timeout;
        self.transport.set_read_timeout(timeout)
    }

    fn discard_in_buffer(&mut self) -> io::Result<()> {
        self.look_ahead()?;
        self.pending.clear();
        Ok(())
    }

    fn bytes_to_read(&mut self) -> io::Result<usize> {
        self.look_ahead()?;
        Ok(self.pending.len())
    }
}

/// Opens a serial port for a probe, through `mp-transport`.
///
/// `dtr` is the C#'s `DtrEnable = true` before the STK500 probes (`BoardDetect.cs:431, 467`). The
/// bootloader probe opens with .NET's default of false (`Uploader.cs:150`); on Linux the kernel
/// raises DTR on every open whatever is asked, so false leaves the line as the OS set it rather
/// than failing on a port that cannot change it.
pub fn open_serial(
    port: &str,
    baud: u32,
    dtr: bool,
) -> io::Result<TransportPort<mp_transport::SerialTransport>> {
    let mut serial = mp_transport::SerialTransport::open(port, baud)
        .map_err(|error| io::Error::other(error.to_string()))?;
    if dtr {
        serial.set_dtr(true)?;
    }
    Ok(TransportPort::new(serial))
}

/// Which runtime Mission Planner is on, which `DetectBoard` checks first.
///
/// `// C#: Utilities/BoardDetect.cs:60-61` - `Type.GetType("Mono.Runtime") != null`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Runtime {
    /// .NET on Windows: the USB, WMI and serial rules run.
    DotNet,
    /// Mono, which is how Mission Planner runs on Linux (`README.md:67`): only questions.
    Mono,
}

/// What `DetectBoard` needs from outside: a person, WMI, the port list, ports and a clock.
pub trait DetectHost {
    /// The ports this host opens.
    type Port: ProbePort;

    /// Which runtime to behave as.
    fn runtime(&self) -> Runtime;
    /// `CustomMessageBox.Show(text)`: a message with an OK button.
    fn show(&mut self, text: &str);
    /// `CustomMessageBox.Show(text, caption, YesNo) == Yes`.
    fn ask(&mut self, text: &str, caption: &str) -> bool;
    /// `SELECT * FROM Win32_SerialPort`.
    fn win32_serial_ports(&mut self) -> Vec<Win32SerialPort>;
    /// `SerialPort.GetPortNames()`.
    fn port_names(&mut self) -> Vec<String>;
    /// Opens a port at a baud rate, with DTR raised if `dtr`.
    fn open(&mut self, port: &str, baud: u32, dtr: bool) -> io::Result<Self::Port>;
    /// `DateTime.Now`.
    fn now(&mut self) -> Instant;
    /// `Thread.Sleep`.
    fn sleep(&mut self, duration: Duration);
}

/// `Strings.PleaseUnplugTheBoardAnd`, shown before the bootloader probe.
///
/// `// C#: ExtLibs/Strings/Strings.resx:552-554` - the line break is a bare LF in the resource.
pub const PLEASE_UNPLUG_THE_BOARD_AND: &str = "Please unplug the board, and then press OK and \
     plug back in.\nMission Planner will look for 30 seconds to find the board";

/// How long the bootloader probe keeps trying. `// C#: Utilities/BoardDetect.cs:152, 248`
pub const PROBE_WINDOW: Duration = Duration::from_secs(30);

/// The bootloader baud rate. `// C#: Utilities/BoardDetect.cs:163, 259`
pub const BOOTLOADER_BAUD: u32 = 115_200;

/// The whole of `DetectBoard`, in the C#'s order.
///
/// `// C#: Utilities/BoardDetect.cs:58-560`. The steps are listed in the module documentation.
/// Errors are the ones the C# lets escape: opening or writing the port during the STK500 probes,
/// which are outside any `try`.
pub fn detect_board<H: DetectHost>(
    port: &str,
    devices: &[DeviceInfo],
    host: &mut H,
) -> io::Result<Detected> {
    // C#: BoardDetect.cs:66, 370-410
    if host.runtime() == Runtime::Mono {
        return Ok(Detected::board(mono_questions(host)));
    }

    // C#: BoardDetect.cs:66-197
    match match_devices(devices) {
        Verdict::Board(found) => return Ok(found),
        Verdict::Probe(probe) => return Ok(Detected::board(probe_bootloader(probe, host))),
        Verdict::NoMatch => {}
    }

    // C#: BoardDetect.cs:199-368
    let rows = host.win32_serial_ports();
    match match_serial_ports(port, &rows) {
        Verdict::Board(found) => return Ok(found),
        Verdict::Probe(probe) => return Ok(Detected::board(probe_bootloader(probe, host))),
        Verdict::NoMatch => {}
    }

    // C#: BoardDetect.cs:412-426
    if let Some(board) = linux_questions(host) {
        return Ok(Detected::board(board));
    }

    // C#: BoardDetect.cs:428-463
    if stk500v1_probe(port, host)? {
        return Ok(Detected::board(Boards::B1280));
    }
    // C#: BoardDetect.cs:465
    host.sleep(Duration::from_millis(500));

    // C#: BoardDetect.cs:467-533
    if stk500v2_probe(port, host)? {
        // C#: BoardDetect.cs:487-524 - `!MONO` is always true here: Mono returned above.
        let rows = host.win32_serial_ports();
        return Ok(Detected::board(stk500v2_board(port, &rows)));
    }

    // C#: BoardDetect.cs:535-559
    Ok(Detected::board(last_questions(host)))
}

/// Mono's whole answer: questions.
///
/// `// C#: Utilities/BoardDetect.cs:370-410`
pub fn mono_questions<H: DetectHost>(host: &mut H) -> Boards {
    if host.ask("Is this a APM 2+?", "APM 2+") {
        return Boards::B2560v2;
    }
    if !host.ask("Is this a CUBE/PX4/PIXHAWK/PIXRACER?", "PX4/PIXHAWK") {
        return Boards::B2560;
    }
    if host.ask("Is this a PIXRACER?", "PIXRACER") {
        Boards::Px4v4
    } else if host.ask("Is this a CUBE?", "CUBE") {
        Boards::Px4v3
    } else if host.ask("Is this a PIXHAWK?", "PIXHAWK") {
        Boards::Px4v2
    } else {
        Boards::Px4
    }
}

/// Step 4: the Linux boards, which only a person can name.
///
/// `// C#: Utilities/BoardDetect.cs:412-426`. `None` when neither is confirmed, and detection goes
/// on to the serial probes.
pub fn linux_questions<H: DetectHost>(host: &mut H) -> Option<Boards> {
    if !host.ask("Is this a Linux board?", "Linux") {
        return None;
    }
    if host.ask("Is this Bebop2?", "Bebop2") {
        return Some(Boards::Bebop2);
    }
    if host.ask("Is this Disco?", "Disco") {
        return Some(Boards::Disco);
    }
    None
}

/// Step 6: what is left, asked.
///
/// `// C#: Utilities/BoardDetect.cs:535-559`
pub fn last_questions<H: DetectHost>(host: &mut H) -> Boards {
    if host.ask("Is this a APM 2+?", "APM 2+") {
        return Boards::B2560v2;
    }
    if !host.ask("Is this a PX4/PIXHAWK?", "PX4/PIXHAWK") {
        return Boards::B2560;
    }
    if host.ask("Is this a PIXHAWK?", "PIXHAWK") {
        Boards::Px4v2
    } else {
        Boards::Px4
    }
}

/// The replug prompt, then thirty seconds of trying every port with the bootloader's identify.
///
/// `// C#: Utilities/BoardDetect.cs:150-189` and `:246-290`. The first port that answers decides;
/// a port that will not open or does not answer is skipped, and the list is fetched afresh each
/// pass because the board comes back as a new port. The deadline is checked between passes, not
/// between ports. Nothing answering for thirty seconds is `none`.
pub fn probe_bootloader<H: DetectHost>(probe: Probe, host: &mut H) -> Boards {
    host.show(PLEASE_UNPLUG_THE_BOARD_AND);
    let deadline = host.now() + PROBE_WINDOW;
    while host.now() < deadline {
        for name in host.port_names() {
            // C#: BoardDetect.cs:161-184 - each port in its own try; any failure moves on.
            if let Ok(board) = identify_on(host, &name) {
                return bootloader_board(probe, &board);
            }
        }
    }
    Boards::None
}

/// `new Uploader(port, 115200)` and `identify()` on one port.
///
/// `// C#: ExtLibs/px4uploader/Uploader.cs:121-131` (open, 50 ms timeouts), `:869`
/// (`DiscardInBuffer`), then [`Uploader::identify`].
fn identify_on<H: DetectHost>(host: &mut H, name: &str) -> Result<Board, UploaderError> {
    let mut port = host.open(name, BOOTLOADER_BAUD, false)?;
    port.set_read_timeout(Duration::from_millis(50))?;
    port.discard_in_buffer()?;
    Uploader::new(port).identify()
}

/// Step 5a: twenty STK500v1 syncs at 57600 baud, fifty milliseconds apart.
///
/// `// C#: Utilities/BoardDetect.cs:428-463`. True when a sync is answered `INSYNC OK`, which is an
/// APM1 1280. The port is closed either way when this returns.
fn stk500v1_probe<H: DetectHost>(port: &str, host: &mut H) -> io::Result<bool> {
    let mut serial = host.open(port, 57_600, true)?;
    host.sleep(Duration::from_millis(100));
    for _ in 0..20 {
        serial.discard_in_buffer()?;
        serial.write_all(&STK500V1_GET_SYNC)?;
        host.sleep(Duration::from_millis(50));
        if serial.bytes_to_read()? >= 2 {
            let reply = [read_byte(&mut serial)?, read_byte(&mut serial)?];
            if reply == STK500V1_IN_SYNC_OK {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Step 5b: four STK500v2 load-address commands at 115200 baud.
///
/// `// C#: Utilities/BoardDetect.cs:467-533`. True when one is answered `[0x06, 0x00]`. Each reply
/// is read with a one-second timeout (`:619`).
fn stk500v2_probe<H: DetectHost>(port: &str, host: &mut H) -> io::Result<bool> {
    let mut serial = host.open(port, 115_200, true)?;
    host.sleep(Duration::from_millis(100));
    for _ in 0..4 {
        serial.write_all(&stk500v2_packet(&STK500V2_LOAD_ADDRESS))?;
        serial.set_read_timeout(Duration::from_millis(1000))?;
        let reply = read_stk500v2_packet(&mut serial);
        host.sleep(Duration::from_millis(50));
        if reply == STK500V2_OK {
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_enum_values_count_up_from_zero_in_declaration_order() {
        for (index, board) in Boards::ALL.into_iter().enumerate() {
            assert_eq!(usize::from(board.value()), index, "{board}");
            assert_eq!(Boards::from_name(board.name()), Some(board));
        }
    }

    #[test]
    fn the_hardware_id_is_windows_upper_case_hex() {
        assert_eq!(hardware_id(0x2dae, 0x1016), r"USB\VID_2DAE&PID_1016");
        assert_eq!(hardware_id(0x1209, 0x5740), r"USB\VID_1209&PID_5740");
    }

    #[test]
    fn a_packet_checksum_covers_the_header() {
        // 1b ^ 01 ^ 00 ^ 05 ^ 0e ^ 06 = 0x17
        assert_eq!(
            stk500v2_packet(&STK500V2_LOAD_ADDRESS),
            [0x1b, 0x01, 0x00, 0x05, 0x0e, 0x06, 0, 0, 0, 0, 0x17]
        );
    }
}
