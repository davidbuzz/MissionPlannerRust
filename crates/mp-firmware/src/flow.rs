//! What the firmware pages do once a firmware is chosen: detect the board, download the file and
//! read it - up to the board, and no further.
//!
//! Ported from `Utilities/Firmware.cs` @ efb0801 (GPL-3.0-or-later) - `updateLegacy`,
//! `CheckChibiOS`, `UploadFlash` and the start of each upload it calls, `readIntelHEXv2` - with
//! `ExtLibs/Utilities/Download.cs`'s `getFilefromNet`, and the click handlers of
//! `GCSViews/ConfigurationView/ConfigFirmware.cs` (`findfirmware`, `Custom_firmware_label_Click`)
//! and `ConfigFirmwareManifest.cs` (`LookForPort`'s download, `Lbl_Custom_firmware_label_Click`)
//! that run them.
//!
//! Every one of those ends in a write to a board: a reboot into the bootloader and the px4
//! upload, an STK500 upload through the serial port, a DFU flash, a push over the network to a
//! Parrot or a Solo. Each flow here runs the C# in its order until the step that would open a
//! port or send to a board, and stops there with a [`Stop`] saying which step that was. Nothing
//! in this module opens a port: the board detection it runs is [`crate::detect::detect_board`]
//! over a host that refuses to open one and ends the flow where the C# would have.
//!
//! The person the C# asks - its `CustomMessageBox`es - and its progress events are a
//! [`Dialogue`], so the pages put them on screen and the tests script them.

use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::detect::{
    self, Boards, DetectHost, Detected, DeviceInfo, PLEASE_UNPLUG_THE_BOARD_AND, PROBE_WINDOW,
    ProbePort, Runtime, Win32SerialPort,
};
use crate::detect::BOOTLOADER_BAUD;
use crate::firmware::Firmware;
use crate::legacy::{Software, get_url};
use crate::manifest::Fetch;
use crate::uploader::{Board, Uploader, UploaderError};

/// What a page says in place of the upload.
pub const NOT_ENABLED: &str = "flashing is not enabled in this build";

/// `Strings.ERROR`. `// C#: ExtLibs/Strings/Strings.resx:130-132`
pub const ERROR: &str = "Error";
/// `Strings.ErrorFirmwareFile`. `// C#: ExtLibs/Strings/Strings.resx:146-148`
pub const ERROR_FIRMWARE_FILE: &str = "Error loading firmware file";
/// `Strings.DownloadedFromInternet`. `// C#: ExtLibs/Strings/Strings.resx:229-231`
pub const DOWNLOADED_FROM_INTERNET: &str = "Downloaded from internet";
/// `Strings.CantDetectBoardVersion`. `// C#: ExtLibs/Strings/Strings.resx:332-334`
pub const CANT_DETECT_BOARD_VERSION: &str =
    "Cant detect your Board version. Please check your cabling";
/// `Strings.DetectedA`. `// C#: ExtLibs/Strings/Strings.resx:341-343`
pub const DETECTED_A: &str = "Detected a ";
/// `Strings.DetectingBoardVersion`. `// C#: ExtLibs/Strings/Strings.resx:344-346`
pub const DETECTING_BOARD_VERSION: &str = "Detecting Board Version";
/// `Strings.DownloadingFromInternet`. `// C#: ExtLibs/Strings/Strings.resx:347-349`
pub const DOWNLOADING_FROM_INTERNET: &str = "Downloading from Internet";
/// `Strings.FailedDownload`. `// C#: ExtLibs/Strings/Strings.resx:357-359`
pub const FAILED_DOWNLOAD: &str = "Failed download";
/// `Strings.FailedReadHEX`. `// C#: ExtLibs/Strings/Strings.resx:360-362`
pub const FAILED_READ_HEX: &str = "Failed read HEX";
/// `Strings.FailedToReadHex`. `// C#: ExtLibs/Strings/Strings.resx:363-365`
pub const FAILED_TO_READ_HEX: &str = "Failed to read firmware.hex :";
/// `Strings.InvalidBoardType`. `// C#: ExtLibs/Strings/Strings.resx:375-377`
pub const INVALID_BOARD_TYPE: &str = "Invalid Board Type";
/// `Strings.ReadingHexFile`. `// C#: ExtLibs/Strings/Strings.resx:381-383`
pub const READING_HEX_FILE: &str = "Reading Hex File";
/// `Strings.ReadingHex`. `// C#: ExtLibs/Strings/Strings.resx:402-404`
pub const READING_HEX: &str = "Reading Hex";
/// `Strings.Warning`. `// C#: ExtLibs/Strings/Strings.resx:408-410`
pub const WARNING: &str = "Warning";
/// `Strings.WarningAC32`. `// C#: ExtLibs/Strings/Strings.resx:411-413`
pub const WARNING_AC32: &str =
    "Warning, if you are installing AC 3.2 for the first time you MUST redo a Compass calibration.";
/// `Strings.WarningAC31`. `// C#: ExtLibs/Strings/Strings.resx:414-416`
pub const WARNING_AC31: &str = "Warning, as of AC 3.1 motors will spin when armed, configurable \
                                through the MOT_SPIN_ARMED parameter";
/// `Strings.AreYouSureYouWantToUpload`. `// C#: ExtLibs/Strings/Strings.resx:459-461`
pub const ARE_YOU_SURE_YOU_WANT_TO_UPLOAD: &str = "Are you sure you want to upload ";
/// `Strings.CanNotConnectToComPortAnd`. `// C#: ExtLibs/Strings/Strings.resx:468-470`
pub const CAN_NOT_CONNECT_TO_COM_PORT_AND: &str =
    "Can not connect to com port and detect board type";
/// `Strings.Continue`. `// C#: ExtLibs/Strings/Strings.resx:471-473`
pub const CONTINUE: &str = "Continue";
/// `Strings.ErrorUploadingFirmware`. `// C#: ExtLibs/Strings/Strings.resx:474-476`
pub const ERROR_UPLOADING_FIRMWARE: &str = "Error uploading firmware";
/// `Strings.QuestionMark`. `// C#: ExtLibs/Strings/Strings.resx:477-479`
pub const QUESTION_MARK: &str = "?";
/// `Strings.Note`. `// C#: ExtLibs/Strings/Strings.resx:549-551`
pub const NOTE: &str = "Note";
/// `Strings.ThisBoardHasBeenRetired`. `// C#: ExtLibs/Strings/Strings.resx:556-558`
pub const THIS_BOARD_HAS_BEEN_RETIRED: &str = "This board has been retired, Mission Planner this \
                                               will upload the last available version to your \
                                               board (AC 3.2.1/AP 3.4.0)";
/// `Strings.No_firmware_available_for_this_board`. `// C#: ExtLibs/Strings/Strings.resx:646-648`
pub const NO_FIRMWARE_AVAILABLE: &str = "No firmware available for this board!";

/// The largest image a 1280 takes. `// C#: Utilities/Firmware.cs:1373`
pub const MAX_1280_IMAGE: usize = 126_976;

/// `CustomMessageBox`'s buttons, for a question.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Buttons {
    /// Yes and No.
    YesNo,
    /// OK and Cancel.
    OkCancel,
}

/// The person at the screen, and the progress bar and status line: what the C#'s
/// `CustomMessageBox.Show` calls and `Progress` events reach.
pub trait Dialogue {
    /// A question, modal: true for Yes (or OK).
    fn ask(&mut self, text: &str, caption: &str, buttons: Buttons) -> bool;
    /// A message with an OK, modal.
    fn show(&mut self, text: &str, caption: &str);
    /// `Progress(percent, status)`: `-1` leaves the bar where it is.
    fn progress(&mut self, percent: i32, status: &str);
}

/// The step a flow stopped before, because it writes to a board.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Stop {
    /// `DetectBoard` found a px4 bootloader to look for: "Please unplug the board ..." and thirty
    /// seconds of opening every serial port and sending the bootloader's identify.
    /// `// C#: Utilities/BoardDetect.cs:148-190, 241-291`
    BootloaderProbe,
    /// `DetectBoard`'s STK500 syncs, sent through this port.
    /// `// C#: Utilities/BoardDetect.cs:428-533`
    OpenPort(String),
    /// `UploadPX4`'s `AttemptRebootToBootloader`: every port opened, a MAVLink reboot into the
    /// bootloader, then the upload.
    /// `// C#: Utilities/Firmware.cs:605, 752-840`
    RebootToBootloader,
    /// `UploadArduino`: this port opened for an STK500 upload.
    /// `// C#: Utilities/Firmware.cs:1388-1394`
    ArduinoUpload(String),
    /// `UploadVRBRAIN`: the MAVLink port opened, a reboot, then the upload.
    /// `// C#: Utilities/Firmware.cs:857-866`
    Vrbrain,
    /// `UploadParrot`: the vehicle's Wi-Fi, then a push to it.
    /// `// C#: Utilities/Firmware.cs:989-1020`
    Parrot,
    /// `UploadSolo`: `Solo.flash_px4` over SSH.
    /// `// C#: Utilities/Firmware.cs:1336-1349`
    Solo,
    /// `DFU.Flash`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:433-454`
    Dfu,
    /// `MAV_CMD_FLASH_BOOTLOADER` to the vehicle connected, which rewrites its bootloader.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:33`
    FlashBootloader,
}

impl Stop {
    /// What the step would have done.
    #[must_use]
    pub fn step(&self) -> String {
        match self {
            Self::BootloaderProbe => {
                "ask for the board to be replugged and open every serial port to find its \
                 bootloader"
                    .to_owned()
            }
            Self::OpenPort(port) => format!("open {port} for an STK500 sync"),
            Self::RebootToBootloader => {
                "reboot the board into its bootloader and upload the image".to_owned()
            }
            Self::ArduinoUpload(port) => format!("open {port} for an STK500 upload"),
            Self::Vrbrain => "reboot the board over MAVLink and upload the image".to_owned(),
            Self::Parrot => "connect to the vehicle's Wi-Fi and push the image to it".to_owned(),
            Self::Solo => "push the image to the Solo over SSH".to_owned(),
            Self::Dfu => "flash the image through DFU".to_owned(),
            Self::FlashBootloader => {
                "send MAV_CMD_FLASH_BOOTLOADER, which rewrites the board's bootloader".to_owned()
            }
        }
    }

    /// What a page says: the step, and that it is not taken.
    #[must_use]
    pub fn text(&self) -> String {
        format!("the next step would {}: {NOT_ENABLED}", self.step())
    }
}

/// A firmware file as `ProcessFirmware` read it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Summary {
    /// `board_id`, which the bootloader must agree with.
    pub board_id: u32,
    /// `description`.
    pub description: String,
    /// The image, decompressed and padded.
    pub image_len: usize,
}

/// How far a flow got: what the page shows, and the facts a test reads.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Reached {
    /// The board detected, as `board.ToString()` names it, or chosen by extension.
    pub board: Option<String>,
    /// The URL downloaded.
    pub url: Option<String>,
    /// Where the file is, downloaded or chosen.
    pub file: Option<PathBuf>,
    /// Its size on disk.
    pub size: Option<u64>,
    /// The firmware file as read.
    pub firmware: Option<Summary>,
    /// An Intel HEX image as read: its length.
    pub hex_len: Option<usize>,
    /// The step it stopped before.
    pub stop: Option<Stop>,
    /// The board a px4 upload wrote, as its bootloader described it.
    pub flashed: Option<Board>,
}

impl Reached {
    /// Notes the file as it is on disk now.
    fn file(&mut self, path: &Path) {
        self.file = Some(path.to_owned());
        self.size = std::fs::metadata(path).ok().map(|meta| meta.len());
    }
}

/// What a flow runs with.
pub struct Cx<'a> {
    /// The person and the progress bar.
    pub dialogue: &'a mut dyn Dialogue,
    /// The network.
    pub fetch: &'a dyn Fetch,
    /// `Settings.GetUserDataDirectory()`, where `updateLegacy` saves `firmware.hex`.
    pub user_data: PathBuf,
    /// `Path.GetTempPath()`, where `LookForPort` saves its download.
    pub temp_dir: PathBuf,
}

impl std::fmt::Debug for Cx<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Cx")
            .field("user_data", &self.user_data)
            .field("temp_dir", &self.temp_dir)
            .finish_non_exhaustive()
    }
}

/// Why a step of `updateLegacy`'s `try` ended early.
enum Halt {
    /// The next step writes to a board.
    Stop(Stop),
    /// It threw: the `catch` says "Failed to download new firmware".
    Threw(String),
    /// It returned this.
    Return(bool),
}

// ---------------------------------------------------------------------------------------------
// Board detection, with no port opened.
// ---------------------------------------------------------------------------------------------

/// A port that cannot exist: the host below never opens one.
#[derive(Debug)]
pub enum NoPort {}

impl Read for NoPort {
    fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
        match *self {}
    }
}

impl Write for NoPort {
    fn write(&mut self, _buf: &[u8]) -> io::Result<usize> {
        match *self {}
    }

    fn flush(&mut self) -> io::Result<()> {
        match *self {}
    }
}

impl ProbePort for NoPort {
    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        match *self {}
    }

    fn discard_in_buffer(&mut self) -> io::Result<()> {
        match *self {}
    }

    fn bytes_to_read(&mut self) -> io::Result<usize> {
        match *self {}
    }
}

/// `DetectBoard`'s host: the person through the dialogue, the ports as listed - and no port
/// opened. The first step that would open one, or the bootloader probe's replug prompt, ends the
/// detection there, with every question after it answered No and the probe's clock run out.
struct Probing<'a> {
    dialogue: &'a mut dyn Dialogue,
    rows: &'a [Win32SerialPort],
    stop: Option<Stop>,
    clock: Instant,
    ticks: u32,
}

impl DetectHost for Probing<'_> {
    type Port = NoPort;

    /// The Windows path, which reads the device list and the ports' USB ids; Mono's is a chain of
    /// questions with no reading at all. This application follows the Windows code where the C#
    /// splits on the platform, as its SETUP and CONFIG lists do (`setup.rs`, `Guard::OnboardOsd`).
    fn runtime(&self) -> Runtime {
        Runtime::DotNet
    }

    fn show(&mut self, text: &str) {
        if self.stop.is_some() {
            return;
        }
        if text == PLEASE_UNPLUG_THE_BOARD_AND {
            self.stop = Some(Stop::BootloaderProbe);
            return;
        }
        self.dialogue.show(text, "");
    }

    fn ask(&mut self, text: &str, caption: &str) -> bool {
        self.stop.is_none() && self.dialogue.ask(text, caption, Buttons::YesNo)
    }

    fn win32_serial_ports(&mut self) -> Vec<Win32SerialPort> {
        self.rows.to_vec()
    }

    fn port_names(&mut self) -> Vec<String> {
        Vec::new()
    }

    fn open(&mut self, port: &str, _baud: u32, _dtr: bool) -> io::Result<NoPort> {
        self.stop
            .get_or_insert_with(|| Stop::OpenPort(port.to_owned()));
        Err(io::Error::other(NOT_ENABLED))
    }

    /// Each call past the probe's window, so a probe loop that is reached ends at once.
    fn now(&mut self) -> Instant {
        self.ticks = self.ticks.saturating_add(1);
        self.clock + (PROBE_WINDOW + Duration::from_secs(1)) * self.ticks
    }

    fn sleep(&mut self, _duration: Duration) {}
}

/// `BoardDetect.DetectBoard(comport, ports)` as far as it goes without a port: the device list's
/// rules, the ports' USB-id rules and the Linux-board questions. Where it would open a port - the
/// bootloader probe, or the STK500 syncs - it stops.
/// `// C#: Utilities/BoardDetect.cs:58-560`
///
/// # Errors
/// The step it stopped before.
pub fn detect_board(
    comport: &str,
    devices: &[DeviceInfo],
    rows: &[Win32SerialPort],
    dialogue: &mut dyn Dialogue,
) -> Result<Detected, Stop> {
    let mut host = Probing {
        dialogue,
        rows,
        stop: None,
        clock: Instant::now(),
        ticks: 0,
    };
    let detected = detect::detect_board(comport, devices, &mut host);
    if let Some(stop) = host.stop {
        return Err(stop);
    }
    detected.map_err(|_| Stop::OpenPort(comport.to_owned()))
}

// ---------------------------------------------------------------------------------------------
// The download.
// ---------------------------------------------------------------------------------------------

/// `formatTimeSpan`. `// C#: ExtLibs/Utilities/Download.cs:657-665`
#[must_use]
pub fn format_time_span(seconds: f64) -> String {
    if seconds >= 3600.0 {
        return format!("{:.1} Hours", seconds / 3600.0);
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // under an hour
    let whole = seconds.max(0.0) as u64;
    if seconds >= 60.0 {
        return format!("{}:{:02} Minutes", whole / 60 % 60, whole % 60);
    }
    format!("{whole} Seconds")
}

/// `Download.getFilefromNet(url, saveto, status)`: the file saved as `saveto.new` and moved over
/// `saveto`, with "Downloading.. ETA: ..." once a second; false for anything that goes wrong,
/// which it catches. The C# skips the download when the file on disk is newer than the server's
/// `Last-Modified` and the same length; the fetch here does not see the headers, so it always
/// downloads.
/// `// C#: ExtLibs/Utilities/Download.cs:450-554`
pub fn get_file_from_net(
    fetch: &dyn Fetch,
    url: &str,
    saveto: &Path,
    status: &mut dyn FnMut(i32, &str),
) -> bool {
    let started = Instant::now();
    let mut last_second = None;
    let fetched = fetch.get_progress(url, &mut |got, length| {
        let Some(length) = length.filter(|length| *length > 0) else {
            return;
        };
        let elapsed = started.elapsed().as_secs_f64();
        let second = started.elapsed().as_secs();
        if last_second == Some(second) {
            return;
        }
        last_second = Some(second);
        #[allow(clippy::cast_precision_loss)] // a download's bytes fit an f64 well enough
        let percent = got as f64 / length as f64 * 100.0;
        if percent <= 0.0 {
            return;
        }
        let left = elapsed / percent * (100.0 - percent);
        #[allow(clippy::cast_possible_truncation)] // 0 to 100
        status(
            percent as i32,
            &format!("Downloading.. ETA: {}", format_time_span(left)),
        );
    });
    let Ok(bytes) = fetched else {
        return false;
    };
    let staged = PathBuf::from(format!("{}.new", saveto.display()));
    let saved = saveto
        .parent()
        .map_or(Ok(()), std::fs::create_dir_all)
        .and_then(|()| std::fs::write(&staged, &bytes))
        .and_then(|()| {
            if saveto.exists() {
                std::fs::remove_file(saveto)?;
            }
            std::fs::rename(&staged, saveto)
        });
    saved.is_ok()
}

/// `Path.GetTempFileName()`: a new, empty file in the temporary directory.
fn temp_file_name(dir: &Path) -> io::Result<PathBuf> {
    std::fs::create_dir_all(dir)?;
    let pid = std::process::id();
    for attempt in 0..1000_u32 {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |since| since.subsec_nanos());
        let path = dir.join(format!("tmp{pid:x}{nanos:x}{attempt:x}.tmp"));
        match std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
        {
            Ok(_) => return Ok(path),
            Err(err) if err.kind() == io::ErrorKind::AlreadyExists => {}
            Err(err) => return Err(err),
        }
    }
    Err(io::Error::other("no temporary file name was free"))
}

// ---------------------------------------------------------------------------------------------
// UploadFlash, to the board.
// ---------------------------------------------------------------------------------------------

/// `px4uploader.Firmware.ProcessFirmware`, as the uploads begin: "Reading Hex File", then the
/// file read - `Strings.ErrorFirmwareFile` and the error if it cannot be.
/// `// C#: Utilities/Firmware.cs:593-603, 845-855`
fn read_firmware(cx: &mut Cx<'_>, filename: &Path, percent: i32, reached: &mut Reached) -> bool {
    cx.dialogue.progress(percent, READING_HEX_FILE);
    match Firmware::load(filename) {
        Ok(firmware) => {
            reached.firmware = Some(Summary {
                board_id: firmware.board_id,
                description: firmware.description.clone(),
                image_len: firmware.image.len(),
            });
            true
        }
        Err(err) => {
            cx.dialogue
                .show(&format!("{ERROR_FIRMWARE_FILE}\n\n{err}"), ERROR);
            false
        }
    }
}

/// `UploadFlash(comport, filename, board)`: by board, the px4 upload, the VRBRAIN one, Parrot,
/// Solo or the Arduino one - each as far as the board, which is where it stops.
/// `// C#: Utilities/Firmware.cs:1294-1334`
///
/// # Errors
/// The step it stopped before, which every upload reaches unless the file cannot be read.
pub fn upload_flash(
    cx: &mut Cx<'_>,
    comport: &str,
    filename: &Path,
    board: Boards,
    reached: &mut Reached,
) -> Result<bool, Stop> {
    match board {
        Boards::Px4
        | Boards::Px4v2
        | Boards::Px4v3
        | Boards::Px4v4
        | Boards::Px4v4pro
        | Boards::Fmuv5
        | Boards::Revomini
        | Boards::Mindpxv2
        | Boards::Minipix
        | Boards::Chbootloader
        | Boards::Pass
        | Boards::Nxpfmuk66 => {
            // `UploadPX4`. `// C#: Utilities/Firmware.cs:591-605`
            if read_firmware(cx, filename, -1, reached) {
                Err(Stop::RebootToBootloader)
            } else {
                Ok(false)
            }
        }
        Boards::Vrbrainv40
        | Boards::Vrbrainv45
        | Boards::Vrbrainv50
        | Boards::Vrbrainv51
        | Boards::Vrbrainv52
        | Boards::Vrbrainv54
        | Boards::Vrcorev10
        | Boards::Vrubrainv51
        | Boards::Vrubrainv52 => {
            // `UploadVRBRAIN`. `// C#: Utilities/Firmware.cs:842-866`
            if read_firmware(cx, filename, 0, reached) {
                Err(Stop::Vrbrain)
            } else {
                Ok(false)
            }
        }
        Boards::Bebop2 => Err(Stop::Parrot),
        Boards::Solo => Err(Stop::Solo),
        _ => upload_arduino(cx, comport, filename, board, reached),
    }
}

/// `UploadArduino` to the port: the HEX file read, a 1280 refused an image too big for it, then
/// the port opened - where it stops.
/// `// C#: Utilities/Firmware.cs:1351-1394`
fn upload_arduino(
    cx: &mut Cx<'_>,
    comport: &str,
    filename: &Path,
    board: Boards,
    reached: &mut Reached,
) -> Result<bool, Stop> {
    cx.dialogue.progress(0, READING_HEX_FILE);
    let read = std::fs::read(filename)
        .map_err(|err| err.to_string())
        .and_then(|bytes| read_intel_hex(&String::from_utf8_lossy(&bytes), cx.dialogue));
    let flash = match read {
        Ok(flash) => flash,
        Err(message) => {
            cx.dialogue.progress(0, FAILED_READ_HEX);
            cx.dialogue
                .show(&format!("{FAILED_TO_READ_HEX}{message}"), "");
            return Ok(false);
        }
    };
    reached.hex_len = Some(flash.len());
    if board == Boards::B1280 && flash.len() > MAX_1280_IMAGE {
        cx.dialogue.show(
            "Firmware is to big for a 1280, Please upgrade your hardware!!",
            "",
        );
        return Ok(false);
    }
    Err(Stop::ArduinoUpload(comport.to_owned()))
}

/// `readIntelHEXv2`: data records at their addresses in a 1 MB image, extended segment addresses
/// shifting them, the end record required and every line's checksum checked - the image cut to
/// its highest address. A bad checksum and a missing end each say so before failing.
/// `// C#: Utilities/Firmware.cs:1498-1566`
///
/// # Errors
/// What the C# throws: a malformed line, an address past the image, a bad checksum, no end.
pub fn read_intel_hex(text: &str, dialogue: &mut dyn Dialogue) -> Result<Vec<u8>, String> {
    let mut flash = vec![0u8; 1024 * 1024];
    let mut option_offset = 0usize;
    let mut total = 0usize;
    let mut hit_end = false;
    let length_of_text = text.len().max(1);
    let mut consumed = 0usize;
    let mut last_percent = None;
    let hex = |line: &str, at: usize, len: usize| -> Result<usize, String> {
        let part = line
            .get(at..at + len)
            .ok_or_else(|| format!("line too short: {line}"))?;
        usize::from_str_radix(part, 16).map_err(|err| format!("{part}: {err}"))
    };
    for line in text.lines() {
        // `updateProgress(position / length * 100, Strings.ReadingHex)`, once per new percent.
        let percent = consumed * 100 / length_of_text;
        if last_percent != Some(percent) {
            last_percent = Some(percent);
            #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)] // 0 to 100
            dialogue.progress(percent as i32, READING_HEX);
        }
        consumed += line.len() + 1;
        if !line.starts_with(':') {
            continue;
        }
        let length = hex(line, 1, 2)?;
        let mut address = hex(line, 3, 4)?;
        let option = hex(line, 7, 2)?;
        match option {
            0 => {
                for i in 0..length {
                    let byte = u8::try_from(hex(line, 9 + i * 2, 2)?).unwrap_or_default();
                    let at = option_offset + address;
                    *flash
                        .get_mut(at)
                        .ok_or_else(|| format!("address {at:#x} is past the image"))? = byte;
                    address += 1;
                    total = total.max(option_offset + address);
                }
            }
            2 => option_offset = hex(line, 9, 4)? << 4,
            1 => hit_end = true,
            _ => {}
        }
        let checksum = hex(line, line.len().saturating_sub(2), 2)?;
        let mut actual = 0u8;
        for z in 0..(line.len().saturating_sub(3) / 2) {
            actual =
                actual.wrapping_add(u8::try_from(hex(line, z * 2 + 1, 2)?).unwrap_or_default());
        }
        let actual = 0u8.wrapping_sub(actual);
        if usize::from(actual) != checksum {
            dialogue.show("The hex file loaded is invalid, please try again.", "");
            return Err("Checksum Failed - Invalid Hex".to_owned());
        }
    }
    if !hit_end {
        dialogue.show("The hex file did no contain an end flag. aborting", "");
        return Err("No end flag in file".to_owned());
    }
    flash.truncate(total);
    Ok(flash)
}

// ---------------------------------------------------------------------------------------------
// The legacy page: a vehicle clicked.
// ---------------------------------------------------------------------------------------------

/// `CheckChibiOS`: the ChibiOS build instead, when there is one and the person says so.
/// `// C#: Utilities/Firmware.cs:565-585`
fn check_chibios(cx: &mut Cx<'_>, existing: String, chibios: &str) -> Result<String, Halt> {
    // `CheckHTTPFileExists` throwing is not the `UriFormatException` this catches: it reaches
    // `updateLegacy`'s `catch`.
    if chibios.is_empty() || !cx.fetch.exists(chibios).map_err(Halt::Threw)? {
        return Ok(existing);
    }
    if cx.dialogue.ask("Upload ChibiOS", "ChibiOS", Buttons::YesNo) {
        return Ok(chibios.to_owned());
    }
    Ok(existing)
}

/// `updateLegacy`'s `try`: the board, then its URL, then the download.
/// `// C#: Utilities/Firmware.cs:366-543`
fn legacy_download(
    cx: &mut Cx<'_>,
    comport: &str,
    temp: &Software,
    history: &str,
    devices: &[DeviceInfo],
    rows: &[Win32SerialPort],
    reached: &mut Reached,
) -> Result<(Boards, PathBuf), Halt> {
    cx.dialogue.progress(-1, DETECTING_BOARD_VERSION);
    let detected = detect_board(comport, devices, rows, cx.dialogue).map_err(Halt::Stop)?;
    let mut board = detected.board;
    // `BoardDetect.chbootloader`, a static: what this detection named, if it named one.
    let mut chbootloader = detected.chbootloader.unwrap_or_default();
    if board == Boards::None {
        cx.dialogue.show(CANT_DETECT_BOARD_VERSION, "");
        return Err(Halt::Return(false));
    }
    cx.dialogue.progress(-1, &format!("{DETECTED_A}{board}"));
    // `// C#: :383-393`
    if matches!(board, Boards::Px4v2 | Boards::Px4v3)
        && history.is_empty()
        && cx
            .dialogue
            .ask("Is this a CubeBlack?", "CubeBlack", Buttons::YesNo)
    {
        chbootloader = "CubeBlack".to_owned();
        board = Boards::Chbootloader;
    }
    reached.board = Some(if board == Boards::Chbootloader {
        format!("{board} {chbootloader}")
    } else {
        board.name().to_owned()
    });
    // `// C#: :395-518`
    let exists = |cx: &mut Cx<'_>, url: &str| cx.fetch.exists(url).map_err(Halt::Threw);
    let mut baseurl = match board {
        Boards::B2560 => temp.url2560.clone(),
        Boards::B1280 => temp.url.clone(),
        Boards::B2560v2 => temp.url2560_2.clone(),
        Boards::Px4 => temp.urlpx4v1.clone(),
        Boards::Px4rl => temp.urlpx4rl.clone(),
        Boards::Px4v2 => check_chibios(cx, temp.urlpx4v2.clone(), &temp.urlfmuv2)?,
        Boards::Px4v3 => {
            let mut baseurl = temp.urlpx4v3.clone();
            if baseurl.is_empty() || !exists(cx, &baseurl)? {
                baseurl.clone_from(&temp.urlpx4v2);
            }
            check_chibios(cx, baseurl, &temp.urlfmuv3)?
        }
        Boards::Px4v4 => check_chibios(cx, temp.urlpx4v4.clone(), &temp.urlfmuv4)?,
        Boards::Fmuv5 => temp.urlfmuv5.clone(),
        Boards::Px4v4pro => temp.urlpx4v4pro.clone(),
        Boards::Vrbrainv40 => temp.urlvrbrainv40.clone(),
        Boards::Vrbrainv45 => temp.urlvrbrainv45.clone(),
        Boards::Vrbrainv50 => temp.urlvrbrainv50.clone(),
        Boards::Vrbrainv51 => temp.urlvrbrainv51.clone(),
        Boards::Vrbrainv52 => temp.urlvrbrainv52.clone(),
        Boards::Vrbrainv54 => temp.urlvrbrainv54.clone(),
        Boards::Vrcorev10 => temp.urlvrcorev10.clone(),
        Boards::Vrubrainv51 => temp.urlvrubrainv51.clone(),
        Boards::Vrubrainv52 => temp.urlvrubrainv52.clone(),
        Boards::Bebop2 => temp.urlbebop2.clone(),
        Boards::Disco => temp.urldisco.clone(),
        Boards::Revomini => temp.urlrevomini.clone(),
        Boards::Mindpxv2 => temp.urlmindpxv2.clone(),
        Boards::Nxpfmuk66 => temp.urlnxpfmuk66.clone(),
        Boards::Chbootloader => {
            let baseurl = temp.urlfmuv2.replace("fmuv2", &chbootloader);
            if baseurl.is_empty() || !exists(cx, &baseurl)? {
                cx.dialogue.show(NO_FIRMWARE_AVAILABLE, "");
                return Err(Halt::Return(false));
            }
            baseurl
        }
        Boards::Pass => String::new(),
        _ => {
            cx.dialogue.show(INVALID_BOARD_TYPE, "");
            return Err(Halt::Return(false));
        }
    };
    // `// C#: :520-526`
    if board.value() < Boards::Px4.value() {
        cx.dialogue.show(THIS_BOARD_HAS_BEEN_RETIRED, NOTE);
    }
    if !history.is_empty() {
        baseurl = get_url(history, &baseurl).map_err(Halt::Threw)?;
    }
    // `// C#: :528-542` - `getFilefromNet`'s answer is not looked at.
    let saveto = cx.user_data.join("firmware.hex");
    reached.url = Some(baseurl.clone());
    let dialogue = &mut *cx.dialogue;
    get_file_from_net(cx.fetch, &baseurl, &saveto, &mut |percent, status| {
        dialogue.progress(percent, status);
    });
    cx.dialogue.progress(100, DOWNLOADED_FROM_INTERNET);
    reached.file(&saveto);
    Ok((board, saveto))
}

/// `updateLegacy(comport, temp, historyhash, ports)`: the board detected, its URL chosen from the
/// entry, the file downloaded to `firmware.hex` in the user data directory, and `UploadFlash`. A
/// failure anywhere in the first three says "Failed to download new firmware" with the error.
/// `// C#: Utilities/Firmware.cs:361-563`
///
/// # Errors
/// The step it stopped before.
pub fn update_legacy(
    cx: &mut Cx<'_>,
    comport: &str,
    temp: &Software,
    history: &str,
    devices: &[DeviceInfo],
    rows: &[Win32SerialPort],
    reached: &mut Reached,
) -> Result<bool, Stop> {
    let (board, saveto) = match legacy_download(cx, comport, temp, history, devices, rows, reached)
    {
        Ok(chosen) => chosen,
        Err(Halt::Stop(stop)) => return Err(stop),
        Err(Halt::Return(returned)) => return Ok(returned),
        Err(Halt::Threw(error)) => {
            cx.dialogue.progress(50, FAILED_DOWNLOAD);
            cx.dialogue
                .show(&format!("Failed to download new firmware : {error}"), "");
            return Ok(false);
        }
    };
    // `Tracking.AddFW` and `AddTiming` are Mission Planner's analytics, which this application
    // does not send.
    upload_flash(cx, comport, &saveto, board, reached)
}

/// `findfirmware`, the Install Firmware Legacy page's click on a vehicle: "Are you sure you want
/// to upload ...?", the entry's URLs rewritten for a history entry chosen, `updateLegacy`, and its
/// answer - the AC 3.1 and 3.2 warnings after an upload, "Error uploading firmware" after a
/// failure. The page is listed only while disconnected, so `MainV2.comPort.BaseStream.Close()`
/// has nothing to close.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:393-464`
pub fn find_firmware(
    cx: &mut Cx<'_>,
    comport: &str,
    fwtoupload: &Software,
    history: &str,
    devices: &[DeviceInfo],
    rows: &[Win32SerialPort],
) -> Reached {
    let mut reached = Reached::default();
    let question = format!(
        "{ARE_YOU_SURE_YOU_WANT_TO_UPLOAD}{}{QUESTION_MARK}",
        fwtoupload.name
    );
    if !cx.dialogue.ask(&question, CONTINUE, Buttons::YesNo) {
        return reached;
    }
    let fw = if history.is_empty() {
        fwtoupload.clone()
    } else {
        fwtoupload.for_history(history)
    };
    match update_legacy(cx, comport, &fw, history, devices, rows, &mut reached) {
        Err(stop) => reached.stop = Some(stop),
        Ok(true) => {
            let copter = fw.url2560_2.to_lowercase().contains("copter");
            let name = fw.name.to_lowercase();
            if copter && name.contains("3.1") {
                cx.dialogue.show(WARNING_AC31, WARNING);
            }
            if copter && name.contains("3.2") {
                cx.dialogue.show(WARNING_AC32, WARNING);
            }
        }
        Ok(false) => cx.dialogue.show(ERROR_UPLOADING_FIRMWARE, ERROR),
    }
    reached
}

/// Whether a file name ends with one of these extensions, ignoring case, as `ToLower().EndsWith`.
fn ends_with_any(filename: &Path, extensions: &[&str]) -> bool {
    let lower = filename.to_string_lossy().to_lowercase();
    extensions
        .iter()
        .any(|extension| lower.ends_with(extension))
}

/// "Load custom firmware" on the legacy page, for a file that exists: a `.px4` or `.apj` is a
/// `px4v3` - or a Solo, when one answers on its network and the person says so - and anything
/// else is `DetectBoard`'s; then `UploadFlash`.
///
/// `solo.Solo.is_solo_alive` is an ICMP ping of `10.1.1.10`, which needs a raw socket this
/// application does not open: no Solo is ever alive here, so the question is never asked.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:521-595`
pub fn custom_legacy(
    cx: &mut Cx<'_>,
    comport: &str,
    filename: &Path,
    devices: &[DeviceInfo],
    rows: &[Win32SerialPort],
) -> Reached {
    let mut reached = Reached::default();
    reached.file(filename);
    let board = if ends_with_any(filename, &[".px4", ".apj"]) {
        Boards::Px4v3
    } else {
        match detect_board(comport, devices, rows, cx.dialogue) {
            Ok(detected) => detected.board,
            Err(stop) => {
                reached.stop = Some(stop);
                return reached;
            }
        }
    };
    if board == Boards::None {
        cx.dialogue.show(CANT_DETECT_BOARD_VERSION, "");
        return reached;
    }
    reached.board = Some(board.name().to_owned());
    if let Err(stop) = upload_flash(cx, comport, filename, board, &mut reached) {
        reached.stop = Some(stop);
    }
    reached
}

/// "Load custom firmware" on the manifest page, for a file that exists: a `.dfu` is flashed
/// through DFU; a `.hex` through DFU if the person says OK, else it falls through to no board; a
/// `.bin` through DFU at `0x08000000` if they say OK, and nothing either way after; a `.px4` or
/// `.apj` is a `px4v2` - or a Solo, as on the legacy page, never here - and anything else is
/// `DetectBoard`'s; then `UploadFlash`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:413-511`
pub fn custom_manifest(
    cx: &mut Cx<'_>,
    comport: &str,
    filename: &Path,
    devices: &[DeviceInfo],
    rows: &[Win32SerialPort],
) -> Reached {
    let mut reached = Reached::default();
    reached.file(filename);
    let mut board = Boards::None;
    if ends_with_any(filename, &[".dfu"]) {
        reached.stop = Some(Stop::Dfu);
        return reached;
    } else if ends_with_any(filename, &[".hex"]) {
        if cx
            .dialogue
            .ask("Do you want to upload this via DFU?", "", Buttons::OkCancel)
        {
            reached.stop = Some(Stop::Dfu);
            return reached;
        }
    } else if ends_with_any(filename, &[".bin"]) {
        if cx
            .dialogue
            .ask("Flashing image to 0x08000000", "", Buttons::OkCancel)
        {
            reached.stop = Some(Stop::Dfu);
        }
        return reached;
    } else if ends_with_any(filename, &[".px4", ".apj"]) {
        board = Boards::Px4v2;
    } else {
        match detect_board(comport, devices, rows, cx.dialogue) {
            Ok(detected) => board = detected.board,
            Err(stop) => {
                reached.stop = Some(stop);
                return reached;
            }
        }
    }
    if board == Boards::None {
        cx.dialogue.show(CANT_DETECT_BOARD_VERSION, "");
        return reached;
    }
    reached.board = Some(board.name().to_owned());
    if let Err(stop) = upload_flash(cx, comport, filename, board, &mut reached) {
        reached.stop = Some(stop);
    }
    reached
}

// ---------------------------------------------------------------------------------------------
// The manifest page: LookForPort's download.
// ---------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------
// The px4 upload to a board: `UploadPX4` past `ProcessFirmware`.
// ---------------------------------------------------------------------------------------------

/// What `AttemptRebootToBootloader` did with the MAVLink link.
/// `// C#: Utilities/Firmware.cs:797-837`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkReboot {
    /// `MainV2.comPort.BaseStream` is not a `SerialPort`: nothing is sent.
    NotSerial,
    /// A heartbeat was seen and `doReboot(true, false)` went out; the port is closed.
    Rebooted,
    /// "No HeartBeat found", or the link failed: the operator is asked to replug the board.
    NoHeartbeat,
}

/// The machine a px4 upload runs on: its serial ports, opened at the bootloader's baud, its
/// clock, and the MAVLink link the vehicle is on.
pub trait FlashHost {
    /// The ports this host opens.
    type Port: ProbePort;

    /// `SerialPort.GetPortNames()`, read afresh each time - the bootloader's port appears after
    /// the reboot.
    fn port_names(&mut self) -> Vec<String>;
    /// `new Uploader(port, 115200)`: the port at `baud`.
    fn open(&mut self, port: &str, baud: u32) -> io::Result<Self::Port>;
    /// The link's part of `AttemptRebootToBootloader`.
    fn reboot_link(&mut self) -> LinkReboot;
    /// `DateTime.Now`.
    fn now(&mut self) -> Instant;
    /// `Thread.Sleep`.
    fn sleep(&mut self, duration: Duration);
}

/// `Strings.NoNeedToUpload`.
/// `// C#: ExtLibs/Strings/Strings.resx:564-566`
pub const NO_NEED_TO_UPLOAD: &str = "No need to upload. already on the board";
/// `Uploader.ConfirmEvent`'s question when the board already holds the firmware, and its caption.
/// `// C#: ExtLibs/px4uploader/Uploader.cs:857-858; Utilities/Firmware.cs:657-658`
pub const SAME_FIRMWARE_QUESTION: &str =
    "The board already has the same firmware version.\nUpload anyway?";
/// The question's caption.
pub const SAME_FIRMWARE_CAPTION: &str = "Same Firmware";
/// The words when the port fails between finding the board and uploading.
/// `// C#: Utilities/Firmware.cs:702, 710`
pub const LOST_COMMUNICATION: &str = "lost communication with the board.";
/// `UploadPX4`'s last words when no bootloader answered in time.
/// `// C#: Utilities/Firmware.cs:747`
pub const NO_RESPONSE_FROM_BOARD: &str = "ERROR: No Response from board";
/// How long `UploadPX4` scans for the bootloader.
/// `// C#: Utilities/Firmware.cs:607`
pub const SCAN_WINDOW: Duration = Duration::from_secs(30);

/// `new Uploader(port, 115200)` then `identify()`: the port opened with 50 ms timeouts and its
/// input discarded, as the C# constructor does, and the bootloader asked who it is.
/// `// C#: ExtLibs/px4uploader/Uploader.cs:121-131, 869`
fn identify_port<H: FlashHost>(
    host: &mut H,
    name: &str,
) -> Result<(Uploader<H::Port>, Board), UploaderError> {
    let mut port = host.open(name, BOOTLOADER_BAUD)?;
    port.set_read_timeout(Duration::from_millis(50))?;
    port.discard_in_buffer()?;
    let mut uploader = Uploader::new(port);
    let board = uploader.identify()?;
    Ok((uploader, board))
}

/// `AttemptRebootToBootloader`: every port tried for a bootloader already answering - found,
/// nothing more is done - else, over a serial link, the vehicle's heartbeat looked for and
/// `doReboot(true, false)` sent, with "Please unplug the board ..." when no heartbeat came.
/// `// C#: Utilities/Firmware.cs:752-840`
fn attempt_reboot_to_bootloader<H: FlashHost>(cx: &mut Cx<'_>, host: &mut H) {
    for name in host.port_names() {
        if identify_port(host, &name).is_ok() {
            return;
        }
    }
    cx.dialogue.progress(-1, "Look for HeartBeat");
    match host.reboot_link() {
        LinkReboot::NotSerial => {}
        LinkReboot::Rebooted => cx.dialogue.progress(-1, "Reboot to Bootloader"),
        LinkReboot::NoHeartbeat => cx.dialogue.show(PLEASE_UNPLUG_THE_BOARD_AND, ""),
    }
}

/// `UploadPX4` after the file is read: the reboot attempt, then up to thirty seconds of scanning
/// every port for a bootloader whose board id is the firmware's (another board's is "keep
/// looking"), then `currentChecksum` - the same firmware already there asks "Upload anyway?",
/// and No is "No need to upload" - and the upload with its progress, "Upload Done" at the end.
/// A port failing once the board is found is "lost communication with the board."; an upload
/// failing is "ERROR: " and the reason; no board in time is "ERROR: No Response from board".
///
/// The C# scans the ports in parallel; here they are tried in turn, which on a machine with one
/// board is the same conversation.
/// `// C#: Utilities/Firmware.cs:591-750`
pub fn upload_px4<H: FlashHost>(
    cx: &mut Cx<'_>,
    host: &mut H,
    firmware: &Firmware,
    reached: &mut Reached,
) -> bool {
    attempt_reboot_to_bootloader(cx, host);
    let deadline = host.now() + SCAN_WINDOW;
    cx.dialogue.progress(-1, "Scanning comports");
    while host.now() < deadline {
        for name in host.port_names() {
            let Ok((mut uploader, board)) = identify_port(host, &name) else {
                continue;
            };
            cx.dialogue.progress(-1, &format!("{name} Identify"));
            if board.board_id != firmware.board_id {
                // "Board type mismatch - keep looking".
                continue;
            }
            cx.dialogue.progress(-1, "Connecting");
            // "test if pausing here stops - System.TimeoutException: The write timed out."
            host.sleep(Duration::from_millis(500));
            match uploader.same_firmware(firmware, &board) {
                Ok(true) => {
                    if !cx.dialogue.ask(
                        SAME_FIRMWARE_QUESTION,
                        SAME_FIRMWARE_CAPTION,
                        Buttons::YesNo,
                    ) {
                        let _ = uploader.reboot();
                        cx.dialogue.show(NO_NEED_TO_UPLOAD, "");
                        return true;
                    }
                }
                Ok(false) => {}
                Err(UploaderError::Io(error)) => {
                    let caption = if error.kind() == io::ErrorKind::TimedOut {
                        "comms timeout"
                    } else {
                        "lost comms"
                    };
                    cx.dialogue.show(LOST_COMMUNICATION, caption);
                    return false;
                }
                // Any other exception in `currentChecksum` lands in the C#'s bare `catch`:
                // the board is rebooted and "No need to upload" is said.
                Err(_) => {
                    let _ = uploader.reboot();
                    cx.dialogue.show(NO_NEED_TO_UPLOAD, "");
                    return true;
                }
            }
            cx.dialogue.progress(0, "Upload");
            let dialogue = &mut *cx.dialogue;
            #[allow(clippy::cast_possible_truncation)] // 0 to 100
            let outcome = uploader.upload(firmware, |fraction| {
                dialogue.progress((fraction * 100.0) as i32, "Upload");
            });
            return match outcome {
                Ok(board) => {
                    cx.dialogue.progress(100, "Upload Done");
                    reached.flashed = Some(board);
                    true
                }
                Err(error) => {
                    cx.dialogue.progress(0, &format!("ERROR: {error}"));
                    false
                }
            };
        }
    }
    cx.dialogue.progress(0, NO_RESPONSE_FROM_BOARD);
    false
}

/// [`upload_flash`] with a board to write to: a px4-family board goes through [`upload_px4`]
/// instead of stopping before it; every other board stops as before.
///
/// # Errors
/// The step it stopped before, for the boards this application does not flash.
pub fn upload_flash_with<H: FlashHost>(
    cx: &mut Cx<'_>,
    host: &mut H,
    comport: &str,
    filename: &Path,
    board: Boards,
    reached: &mut Reached,
) -> Result<bool, Stop> {
    match upload_flash(cx, comport, filename, board, reached) {
        Err(Stop::RebootToBootloader) => {
            // `UploadPX4`: the file is read (and `reached` told of it) by `upload_flash`.
            let firmware = match Firmware::load(filename) {
                Ok(firmware) => firmware,
                Err(_) => return Ok(false),
            };
            Ok(upload_px4(cx, host, &firmware, reached))
        }
        other => other,
    }
}

/// A flow that stopped before rebooting a px4-family board into its bootloader, taken on to the
/// board: [`upload_px4`] over the file it downloaded or was given. Any other outcome is returned
/// as it was.
pub fn flash_if_stopped<H: FlashHost>(cx: &mut Cx<'_>, host: &mut H, mut reached: Reached) -> Reached {
    if reached.stop == Some(Stop::RebootToBootloader)
        && let Some(file) = reached.file.clone()
    {
        reached.stop = None;
        if let Ok(firmware) = Firmware::load(&file) {
            upload_px4(cx, host, &firmware, &mut reached);
        }
    }
    reached
}

/// [`download_and_flash`] with a board to write to.
pub fn download_and_flash_with<H: FlashHost>(
    cx: &mut Cx<'_>,
    host: &mut H,
    baseurl: &str,
    device_name: &str,
) -> Reached {
    let reached = download_and_flash(cx, baseurl, device_name);
    flash_if_stopped(cx, host, reached)
}

/// `LookForPort` once it has a URL: the download to a temporary file - "Downloading from
/// Internet" as it goes, `Strings.FailedDownload` if it fails, which includes a server that sends
/// no `Content-Length` - then `UploadFlash(device, file, pass)`. After "No firmware available"
/// the C# goes on to download an empty URL, which fails; so does this.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:261-343`
pub fn download_and_flash(cx: &mut Cx<'_>, baseurl: &str, device_name: &str) -> Reached {
    let mut reached = Reached::default();
    let downloaded = temp_file_name(&cx.temp_dir)
        .map_err(|err| err.to_string())
        .and_then(|tempfile| {
            let dialogue = &mut *cx.dialogue;
            let mut started = false;
            let mut length_known = true;
            let bytes = cx.fetch.get_progress(baseurl, &mut |got, length| {
                if !started {
                    started = true;
                    dialogue.progress(0, DOWNLOADING_FROM_INTERNET);
                }
                match length {
                    Some(0) => dialogue.progress(50, DOWNLOADING_FROM_INTERNET),
                    Some(length) => {
                        #[allow(clippy::cast_possible_truncation)] // 0 to 100
                        let percent = (got.saturating_mul(100) / length) as i32;
                        dialogue.progress(percent, DOWNLOADING_FROM_INTERNET);
                    }
                    None => length_known = false,
                }
            })?;
            if !length_known {
                return Err("no Content-Length".to_owned());
            }
            std::fs::write(&tempfile, bytes).map_err(|err| err.to_string())?;
            Ok(tempfile)
        });
    let tempfile = match downloaded {
        Ok(tempfile) => tempfile,
        Err(_) => {
            cx.dialogue.show(FAILED_DOWNLOAD, ERROR);
            return reached;
        }
    };
    cx.dialogue.progress(100, DOWNLOADED_FROM_INTERNET);
    reached.url = Some(baseurl.to_owned());
    reached.file(&tempfile);
    reached.board = Some(Boards::Pass.name().to_owned());
    if let Err(stop) = upload_flash(cx, device_name, &tempfile, Boards::Pass, &mut reached) {
        reached.stop = Some(stop);
    }
    reached
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A person who answers from a script, and remembers what they were shown.
    #[derive(Default)]
    struct Scripted {
        answers: Vec<bool>,
        asked: Vec<String>,
        shown: Vec<String>,
        progress: Vec<(i32, String)>,
    }

    impl Dialogue for Scripted {
        fn ask(&mut self, text: &str, _caption: &str, _buttons: Buttons) -> bool {
            self.asked.push(text.to_owned());
            if self.answers.is_empty() {
                false
            } else {
                self.answers.remove(0)
            }
        }

        fn show(&mut self, text: &str, _caption: &str) {
            self.shown.push(text.to_owned());
        }

        fn progress(&mut self, percent: i32, status: &str) {
            self.progress.push((percent, status.to_owned()));
        }
    }

    #[test]
    fn intel_hex_is_read_as_the_csharp_reads_it() {
        let mut person = Scripted::default();
        // Two data records, one at an extended segment, and the end.
        let text = ":0400000001020304F2\n:020000021000EC\n:02000000AABB99\n:00000001FF\n";
        let flash = read_intel_hex(text, &mut person).expect("reads");
        assert_eq!(flash.len(), 0x10002);
        assert_eq!(&flash[..4], &[1, 2, 3, 4]);
        assert_eq!(&flash[0x10000..], &[0xAA, 0xBB]);
        assert!(person.shown.is_empty());

        let mut person = Scripted::default();
        assert!(read_intel_hex(":0400000001020304F3\n:00000001FF\n", &mut person).is_err());
        assert_eq!(
            person.shown,
            ["The hex file loaded is invalid, please try again."]
        );
        let mut person = Scripted::default();
        assert!(read_intel_hex(":0400000001020304F2\n", &mut person).is_err());
        assert_eq!(
            person.shown,
            ["The hex file did no contain an end flag. aborting"]
        );
    }

    #[test]
    fn time_spans_read_as_format_time_span_writes_them() {
        assert_eq!(format_time_span(5.4), "5 Seconds");
        assert_eq!(format_time_span(125.0), "2:05 Minutes");
        assert_eq!(format_time_span(5400.0), "1.5 Hours");
    }

    /// The Windows path's device rules name the board; the replug-and-probe and the STK500 syncs
    /// each stop detection where they would open a port.
    #[test]
    fn detection_stops_where_a_port_would_be_opened() {
        let mut person = Scripted::default();
        let cube = DeviceInfo::new("CubeOrange-BL", r"USB\VID_2DAE&PID_1016");
        assert_eq!(
            detect_board("COM3", &[cube], &[], &mut person),
            Ok(Detected::chbootloader("CubeOrange"))
        );
        let old = DeviceInfo::new("PX4 FMU v2.x", r"USB\VID_26AC&PID_0011");
        assert_eq!(
            detect_board("COM3", &[old], &[], &mut person),
            Err(Stop::BootloaderProbe)
        );
        assert!(person.shown.is_empty(), "the replug prompt is not shown");
        // Nothing recognised: "Is this a Linux board?", then the STK500 sync on the port.
        let mut person = Scripted::default();
        assert_eq!(
            detect_board("/dev/ttyUSB0", &[], &[], &mut person),
            Err(Stop::OpenPort("/dev/ttyUSB0".to_owned()))
        );
        assert_eq!(person.asked, ["Is this a Linux board?"]);
        // A Bebop2 is named by the questions alone.
        let mut person = Scripted {
            answers: vec![true, true],
            ..Scripted::default()
        };
        assert_eq!(
            detect_board("/dev/ttyUSB0", &[], &[], &mut person),
            Ok(Detected::board(Boards::Bebop2))
        );
    }
}
