//! The wire half of the Sik Radio page's handlers (`Radio/Sikradio.cs`): what each says to the
//! radio, in the C#'s order, and what it reads back for the page to show.
//!
//! The C# runs each handler on the UI thread, setting `lbl_status` and its controls as it goes.
//! Here each is a function over the session, run where it may block, that tells a [`Report`]
//! what `lbl_status`, the progress bar and the boxes would say as it goes, and returns what the
//! controls are to show ([`Loaded`]); the page applies that in the C#'s order once the handler
//! is done, while its controls are disabled as the C#'s are.
//!
//! The boxes are split as the owner's ruling of 2026-09-25 splits them: a radio or port that
//! failed ("Failed to enter command mode", "Set Command error", ...) is [`Report::failed`], for
//! the window's status line; a warning, a validation or a report stays [`Report::message`], a
//! box.

use std::io;
use std::path::PathBuf;

use crate::at::{self, remove_multipoint_node_id};
use crate::modem;
use crate::session::{Mode, Session};
use crate::settings::Settings;
use crate::uploader::{Board, Frequency};
use crate::{Port, parse_int_any, parse_int_hex};

/// What a handler says while it runs.
pub trait Report {
    /// `lbl_status.Text = text`.
    fn status(&mut self, text: &str);
    /// `Progressbar.Value`, as a fraction (the C# clamps it to 100 %).
    fn progress(&mut self, completed: f64);
    /// A box that stays a box.
    fn message(&mut self, text: &str);
    /// A box for a radio or port that failed: the window's status line.
    fn failed(&mut self, text: &str);
    /// "Programming firmware failed.  Try again?" with Yes, No and Cancel: true for Yes.
    fn try_again(&mut self) -> bool;
    /// The firmware's `OpenFileDialog`, with its filter: the file, or `None` for Cancel.
    fn choose_firmware(&mut self, filter: &str) -> Option<PathBuf>;
}

/// The page's `doCommand`, its "Doing Command" in `lbl_status`.
fn cmd<P: Port>(
    session: &mut Session<P>,
    report: &mut dyn Report,
    command: &str,
    multi_line: bool,
    level: u8,
) -> io::Result<String> {
    at::do_command(
        session.wire(),
        &mut |text| report.status(text),
        command,
        multi_line,
        level,
    )
}

/// "Input string was not in a correct format.", the C#'s `FormatException`.
fn format_error() -> io::Error {
    io::Error::other("Input string was not in a correct format.")
}

/// `text.Substring(from)`, or the C#'s `ArgumentOutOfRangeException`.
fn substring(text: &str, from: usize) -> io::Result<&str> {
    text.get(from..)
        .ok_or_else(|| io::Error::other("startIndex cannot be larger than length of string."))
}

/// `int.Parse(text.ToLower().Replace("x", ""), style)` - hex when the text has an x - for
/// `Enum.Parse` of a byte enum.
fn parse_code(text: &str) -> io::Result<u8> {
    let lower = text.to_lowercase();
    let digits = lower.replace('x', "");
    let value = if lower.contains('x') {
        parse_int_hex(&digits)
    } else {
        parse_int_any(&digits)
    }
    .ok_or_else(format_error)?;
    u8::try_from(value).map_err(|_| {
        io::Error::other("Value was either too large or too small for an unsigned byte.")
    })
}

/// The key box after a load: the radio's key, or disabled and empty.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KeyBox {
    /// `AESKEY.Text = key; AESKEY.Enabled = true`.
    Shown(String),
    /// `AESKEY.Text = ""; AESKEY.Enabled = false`.
    Disabled,
}

/// What Load Settings read from the remote radio.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteLoaded {
    /// `RTI2.Text`, when `RTI2` answered with a board.
    pub rti2: Option<String>,
    /// `RAESKEY`.
    pub key: KeyBox,
    /// `SetupComboForMavlink(RMAVLINK, simple)`.
    pub simple_mavlink: bool,
    /// The `RTI5` answer.
    pub rti5: String,
    /// `RemoteSettings`.
    pub settings: Settings,
}

/// What Load Settings read, for the page to show, in the order the C# shows it. A field is
/// `None` when the handler did not get that far.
/// `// C#: Radio/Sikradio.cs:1467-1903`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Loaded {
    /// Whether the radio went into AT command mode.
    pub entered: bool,
    /// `ATI.Text`.
    pub ati: Option<String>,
    /// `multipoint_fix`: -1 for a point-to-point radio, else where the values start after the
    /// multipoint `[n]`.
    pub multipoint_fix: i64,
    /// The band, `ATI3.Text`.
    pub freq: Option<Frequency>,
    /// The board, `ATI2.Text`.
    pub board: Option<Board>,
    /// `txtCountry.Text`.
    pub country: Option<String>,
    /// `AESKEY`.
    pub key: Option<KeyBox>,
    /// `SetupComboForMavlink(MAVLINK, simple)`.
    pub simple_mavlink: bool,
    /// `RSSI.Text`.
    pub rssi: Option<String>,
    /// The `ATI5` answer.
    pub ati5: Option<String>,
    /// `Settings`, from `ATI5?` and `ATI5`.
    pub settings: Option<Settings>,
    /// `RTI.Text`: "" with no remote radio.
    pub rti: Option<String>,
    /// `txtRCountry.Text`.
    pub remote_country: Option<String>,
    /// The remote radio's, when there is one.
    pub remote: Option<RemoteLoaded>,
    /// The exception `BUT_getcurrent_Click`'s `catch` caught: "Error" and its box.
    pub error: Option<String>,
}

/// The box Load Settings shows when the remote radio's ranges are the local one's but the
/// firmware differs.
/// `// C#: Radio/Sikradio.cs:1821-1824`
pub const RANGES_WARNING: &str = "The ranges and options shown for the remote modem may not be accurate.  To ensure accurate, use the same firmware version in both the local and remote modems";

/// Load Settings' failure to enter command mode.
/// `// C#: Radio/Sikradio.cs:1883, 2163`
pub const LOAD_NO_COMMAND_MODE: &str = "Failed to enter command mode.  Try power-cycling modem.";

/// `BUT_getcurrent_Click`'s wire half: AT command mode, `ATI`, `ATI3`, `ATI2`, the country, the
/// key, `ATI7`, `ATI5` and the settings, then `RTI` and, when a remote radio answers, the same of
/// it; transparent mode again.
/// `// C#: Radio/Sikradio.cs:1467-1903`
pub fn load<P: Port>(session: &mut Session<P>, report: &mut dyn Report) -> Loaded {
    let mut loaded = Loaded {
        multipoint_fix: -1,
        ..Loaded::default()
    };
    report.status("Connecting");
    if let Err(error) = load_inner(session, report, &mut loaded) {
        report.status("Error");
        report.failed(&format!("Error during read {error}"));
        loaded.error = Some(error.to_string());
    }
    loaded
}

#[allow(clippy::too_many_lines)]
fn load_inner<P: Port>(
    s: &mut Session<P>,
    report: &mut dyn Report,
    loaded: &mut Loaded,
) -> io::Result<()> {
    if s.put_into_at_command_mode()? != Mode::AtCommand {
        // Off hook.
        s.put_into_transparent_mode()?;
        report.status("Fail");
        report.failed(LOAD_NO_COMMAND_MODE);
        return Ok(());
    }
    loaded.entered = true;
    // Cleanup.
    cmd(s, report, "AT&T", false, 1)?;
    s.wire().discard_in_buffer()?;
    report.status("Doing Command ATI & RTI");
    // The radio's version; a multipoint radio's answers start with its [n].
    let ati = cmd(s, report, "ATI", false, 0)?.trim().to_owned();
    let mut fix: i64 = -1;
    if ati.starts_with('[') {
        fix = ati
            .find(']')
            .map_or(0, |i| i64::try_from(i + 1).unwrap_or(0));
    }
    loaded.ati = Some(ati.clone());
    let local_fw = modem::firmware_version(&ati);
    // The band. Some multipoint firmware puts its [n] on ATI3 and not on ATI.
    let mut freq_text = cmd(s, report, "ATI3", false, 0)?.trim().to_owned();
    if fix < 0 && freq_text.starts_with('[') {
        fix = freq_text
            .find(']')
            .map_or(0, |i| i64::try_from(i + 1).unwrap_or(0));
    }
    let skip = usize::try_from(fix).unwrap_or(0);
    if fix > 0 {
        freq_text = substring(&freq_text, skip)?.trim().to_owned();
    }
    let freq = Frequency(parse_code(&freq_text)?);
    loaded.freq = Some(freq);
    // The board.
    let mut board_text = cmd(s, report, "ATI2", false, 0)?.trim().to_owned();
    if fix > 0 {
        board_text = substring(&board_text, skip)?.trim().to_owned();
    }
    let board = Board(parse_code(&board_text)?);
    s.board = board;
    loaded.country = Some(s.country_text(false)?);
    loaded.board = Some(board);
    loaded.multipoint_fix = fix;
    if fix == -1 {
        let key = cmd(s, report, "AT&E?", false, 0)?.trim().to_owned();
        loaded.key = Some(if key.contains("ERROR") {
            KeyBox::Disabled
        } else {
            KeyBox::Shown(key)
        });
    } else {
        loaded.key = Some(KeyBox::Disabled);
        loaded.simple_mavlink = true;
    }
    loaded.rssi = Some(cmd(s, report, "ATI7", false, 0)?.trim().to_owned());
    report.status("Doing Command ATI5");
    let answer = cmd(s, report, "ATI5", true, 0)?;
    let (settings, _) = s.get_settings(false, board, &answer, None)?;
    loaded.ati5 = Some(answer);
    loaded.settings = Some(settings.clone());
    // The remote radio.
    s.wire().discard_in_buffer()?;
    let rti_text = cmd(s, report, "RTI", false, 0)?;
    let rti = if rti_text.len() < 5 || rti_text.starts_with("ERROR") {
        String::new()
    } else {
        rti_text
    };
    loaded.rti = Some(rti.clone());
    let remote_fw = modem::firmware_version(&rti);
    loaded.remote_country = Some(s.country_text(true)?);
    if !rti.is_empty() {
        // `RTI2` in a `try`: its failure is swallowed.
        let rti2 = cmd(s, report, "RTI2", false, 0)
            .ok()
            .filter(|resp| !resp.trim().is_empty())
            .and_then(|resp| Board::parse(&resp))
            .map(|b| b.to_string());
        let (key, simple_mavlink) = if fix == -1 {
            let key = cmd(s, report, "RT&E?", false, 0)?.trim().to_owned();
            let key = if key.contains("ERROR") {
                KeyBox::Disabled
            } else {
                KeyBox::Shown(key)
            };
            (key, false)
        } else {
            (KeyBox::Disabled, true)
        };
        report.status("Doing Command RTI5");
        let rti5 = cmd(s, report, "RTI5", true, 0)?;
        // The local radio's ranges serve when the firmware is the same.
        let ranges = (local_fw == remote_fw).then_some(&settings);
        let (remote_settings, used_ranges) = s.get_settings(true, board, &rti5, ranges)?;
        if remote_fw.is_some() && local_fw != remote_fw && used_ranges {
            report.message(RANGES_WARNING);
        }
        loaded.remote = Some(RemoteLoaded {
            rti2,
            key,
            simple_mavlink,
            rti5,
            settings: remote_settings,
        });
    }
    // Off hook.
    s.put_into_transparent_mode()?;
    Ok(())
}

/// A setting Save Settings writes: `GetUpdatedSettingsFromGroupBox`'s, made by the page.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Change {
    /// The setting's name, the key of the C#'s dictionary.
    pub name: String,
    /// Its designator: `S3`, `&E`.
    pub designator: String,
    /// `GetValueAsString`.
    pub value: String,
    /// A `TSetting` whose value is 1, which a PPM or SBUS output pairs with `PO=1`.
    pub is_one: bool,
}

/// The encryption key Save Settings writes, when the level says the radio encrypts: the key
/// padded to its length, or the length it was too long for.
pub type KeyPlan = Option<Result<String, i32>>;

/// What Save Settings writes, worked out by the page from its controls.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SavePlan {
    /// The remote radio's changes, `None` when `RTI.Text` is "".
    pub remote: Option<Vec<Change>>,
    /// The local radio's changes.
    pub local: Vec<Change>,
    /// The remote key.
    pub remote_key: KeyPlan,
    /// The local key.
    pub local_key: KeyPlan,
}

/// "Set Command error".
/// `// C#: Radio/Sikradio.cs:752, 766`
pub const SET_COMMAND_ERROR: &str = "Set Command error";

/// `SaveSettingsFromGroupBox`'s wire half: each changed setting but the format set; an output
/// paired with `PO=1` or `PI=1`, an input with `PI=1`; a remote encryption level the remote
/// refused set on the local radio too.
/// `// C#: Radio/Sikradio.cs:712-770`
fn save_changes<P: Port>(
    s: &mut Session<P>,
    report: &mut dyn Report,
    changes: &[Change],
    remote: bool,
) -> io::Result<()> {
    let prefix = if remote { "R" } else { "A" };
    for change in changes {
        if change.name.contains("FORMAT") {
            continue;
        }
        let command = format!("{prefix}T{}={}", change.designator, change.value);
        let mut answer = cmd(s, report, &command, change.designator.contains("&E"), 0)?;
        if answer.contains("OK") {
            if change.name.contains("GPO1_1R_COUT") || change.name.contains("GPO1_3SBUSOUT") {
                // The output also needs PO, or PI when off.
                let pair = if change.is_one { "PO=1" } else { "PI=1" };
                answer = cmd(s, report, &format!("{prefix}T{pair}"), false, 0)?;
            } else if change.name.contains("GPI1_1R_CIN") || change.name.contains("GPO1_3SBUSIN") {
                answer = cmd(s, report, &format!("{prefix}TPI=1"), false, 0)?;
            }
            if !answer.contains("OK") {
                report.failed(SET_COMMAND_ERROR);
            }
        } else if remote && change.name == "ENCRYPTION_LEVEL" {
            // Set on the local radio as well: both now use the default key.
            cmd(
                s,
                report,
                &format!("AT{}={}", change.designator, change.value),
                false,
                0,
            )?;
        } else {
            report.failed(SET_COMMAND_ERROR);
        }
    }
    Ok(())
}

/// Save Settings' box for a key that is not hex or is longer than the level's.
/// `// C#: Radio/Sikradio.cs:950, 967`
#[must_use]
pub fn key_invalid_text(max_length: i32) -> String {
    format!("Encryption key not valid hex number <= {max_length} hex numerals")
}

/// Save Settings' failure to enter command mode.
/// `// C#: Radio/Sikradio.cs:1001, 2424`
pub const SAVE_NO_COMMAND_MODE: &str = "Failed to enter command mode";

/// `BUT_savesettings_Click`'s wire half, the checks passed: the remote radio's changes, the local
/// one's, the keys, `&W` and `Z` to each; "Done", or "Fail" when command mode was not had; then
/// `+++` to the rebooted radio. Whether command mode was had.
/// `// C#: Radio/Sikradio.cs:849-1013`
///
/// # Errors
/// The port's: the C# has no `catch` here.
pub fn save<P: Port>(
    s: &mut Session<P>,
    plan: &SavePlan,
    report: &mut dyn Report,
) -> io::Result<bool> {
    report.status("Connecting");
    let entered = s.put_into_at_command_mode()? == Mode::AtCommand;
    if entered {
        // Cleanup.
        cmd(s, report, "AT&T", false, 1)?;
        s.wire().discard_in_buffer()?;
        report.status("Doing Command");
        if let Some(remote) = &plan.remote {
            cmd(s, report, "RTI5", true, 0)?;
            save_changes(s, report, remote, true)?;
            s.wire().sleep(100);
        }
        s.wire().discard_in_buffer()?;
        for _ in 0..5 {
            if !cmd(s, report, "ATI5", true, 0)?.is_empty() {
                break;
            }
        }
        save_changes(s, report, &plan.local, false)?;
        // The keys at the same time, so an encrypted link is not lost.
        for (key, command) in [(&plan.remote_key, "RT&E="), (&plan.local_key, "AT&E=")] {
            match key {
                Some(Ok(key)) => {
                    cmd(s, report, &format!("{command}{key}"), true, 0)?;
                }
                Some(Err(max_length)) => {
                    report.status("Fail");
                    report.message(&key_invalid_text(*max_length));
                }
                None => {}
            }
        }
        if plan.remote.is_some() {
            cmd(s, report, "RT&W", false, 0)?;
            cmd(s, report, "RTZ", false, 0)?;
        }
        if !cmd(s, report, "AT&W", false, 0)?.contains("OK") {
            report.failed("Failed to save parameters");
        }
        cmd(s, report, "ATZ", false, 0)?;
        report.status("Done");
    } else {
        cmd(s, report, "ATZ", false, 0)?;
        report.status("Fail");
        report.failed(SAVE_NO_COMMAND_MODE);
    }
    // The radio rebooted.
    s.put_into_at_command_mode_assuming_in_transparent_mode()?;
    Ok(entered)
}

/// `BUT_resettodefault_Click`'s wire half: `&F`, `&W` and `Z` to the remote radio when there is
/// one, then to the local one, each failure of the local one's said.
/// `// C#: Radio/Sikradio.cs:2108-2165`
///
/// # Errors
/// The port's: the C# has no `catch` here.
pub fn reset<P: Port>(s: &mut Session<P>, remote: bool, report: &mut dyn Report) -> io::Result<()> {
    report.status("Connecting");
    if s.put_into_at_command_mode()? != Mode::AtCommand {
        // Off hook.
        s.put_into_transparent_mode()?;
        report.status("Fail");
        report.failed(LOAD_NO_COMMAND_MODE);
        return Ok(());
    }
    // Cleanup.
    if remote {
        cmd(s, report, "RT&T", false, 0)?;
        s.wire().discard_in_buffer()?;
        report.status("Doing Command RTI & AT&F");
        cmd(s, report, "RT&F", false, 0)?;
        cmd(s, report, "RT&W", false, 0)?;
        report.status("Reset");
        cmd(s, report, "RTZ", false, 0)?;
        cmd(s, report, "RT&T", false, 0)?;
    }
    cmd(s, report, "AT&T", false, 1)?;
    s.wire().discard_in_buffer()?;
    report.status("Doing Command ATI & AT&F");
    // `DoCommandShowErrorIfNotOK`.
    if !cmd(s, report, "AT&F", false, 0)?.contains("OK") {
        report.failed("Failed to reset parameters to factory defaults");
    }
    if !cmd(s, report, "AT&W", false, 0)?.contains("OK") {
        report.failed("Failed to write parameters to EEPROM");
    }
    report.status("Reset");
    cmd(s, report, "ATZ", false, 0)?;
    // The modem rebooted.
    s.put_into_at_command_mode_assuming_in_transparent_mode()?;
    Ok(())
}

/// `SetPPMFailSafe`: the PPM input as it is now recorded (`&R`) and saved (`&W`), on the local
/// or remote radio. "Done" when the record was taken.
/// `// C#: Radio/Sikradio.cs:2382-2427`
///
/// # Errors
/// The port's: the C# has no `catch` here.
pub fn set_ppm_fail_safe<P: Port>(
    s: &mut Session<P>,
    remote: bool,
    report: &mut dyn Report,
) -> io::Result<()> {
    let (set, save) = if remote {
        ("RT&R", "RT&W")
    } else {
        ("AT&R", "AT&W")
    };
    report.status("Connecting");
    if s.put_into_at_command_mode()? != Mode::AtCommand {
        report.status("Fail");
        report.failed(SAVE_NO_COMMAND_MODE);
        return Ok(());
    }
    report.status("Doing Command");
    s.wire().discard_in_buffer()?;
    let result = s.command(set)?;
    s.wire().discard_in_buffer()?;
    s.command(save)?;
    report.status(if result { "Done" } else { "Fail" });
    Ok(())
}

/// `EncryptionCheckChangedEvtHdlr`'s wire half: the encryption level written to the radio at
/// once, and the key it then has read back for the key box - `None` when the radio refused the
/// level. The radio is left in AT command mode.
/// `// C#: Radio/Sikradio.cs:2543-2591`
///
/// # Errors
/// The port's: the C# has no `catch` here.
pub fn set_encryption_level<P: Port>(
    s: &mut Session<P>,
    remote: bool,
    level: i32,
    report: &mut dyn Report,
) -> io::Result<Option<String>> {
    s.put_into_at_command_mode()?;
    let ati5 = cmd(s, report, if remote { "RTI5" } else { "ATI5" }, true, 0)?;
    let board = s.board;
    let (settings, _) = s.get_settings(remote, board, &ati5, None)?;
    if let Some(setting) = settings.get("ENCRYPTION_LEVEL") {
        // `SetSetting`.
        let command = format!(
            "{}{}={level}",
            if remote { "RT" } else { "AT" },
            setting.designator
        );
        if !cmd(s, report, &command, false, 0)?.contains("OK") {
            return Ok(None);
        }
    }
    let query = if remote { "RT&E?" } else { "AT&E?" };
    let key = remove_multipoint_node_id(cmd(s, report, query, false, 0)?.trim());
    report.status("Done.");
    Ok(Some(key.trim().to_owned()))
}

/// How Upload Firmware ended, for the page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Programmed {
    /// The session ended (`EndSession`): the firmware programmed or not, the modem unknown, or a
    /// failure caught.
    Ended,
    /// The file dialog was cancelled; the session lives on.
    Cancelled,
}

/// "Couldn't communicate with modem.  Try power-cycling modem."
/// `// C#: Radio/Sikradio.cs:2268`
pub const NO_MODEM: &str = "Couldn't communicate with modem.  Try power-cycling modem.";

/// `ProgramFirmware(true)`'s wire half, Upload Firmware (custom): the mode, the modem, the file
/// asked for, the model's programming; "Programming failed.  (Try again?)" for whatever the C#'s
/// `catch` catches.
/// `// C#: Radio/Sikradio.cs:2248-2314`
pub fn program_firmware<P: Port>(s: &mut Session<P>, report: &mut dyn Report) -> Programmed {
    match program_inner(s, report) {
        Ok(ended) => ended,
        Err(_) => {
            report.status("Programming failed.  (Try again?)");
            Programmed::Ended
        }
    }
}

fn program_inner<P: Port>(s: &mut Session<P>, report: &mut dyn Report) -> io::Result<Programmed> {
    report.status("Determining mode...");
    let mode = s.get_mode()?;
    report.status(&format!("Mode is {mode}"));
    let Some(modem) = s.get_modem_object()? else {
        report.status("Unknown modem");
        report.failed(NO_MODEM);
        return Ok(Programmed::Ended);
    };
    report.status("Asking user for firmware file");
    let Some(file) = report.choose_firmware(&modem.filter()) else {
        report.status("Firmware file selection cancelled");
        return Ok(Programmed::Cancelled);
    };
    report.status("Programming firmware into device");
    let mut status = StatusOnly(report);
    if s.program_firmware(modem, &file, &mut status)? {
        report.status("Programmed firmware into device");
    } else {
        report.status("Programming failed.  (Try again?)");
    }
    Ok(Programmed::Ended)
}

/// The report as `UpdateStatusCallback` passes it on: everything, the progress included.
struct StatusOnly<'a>(&'a mut dyn Report);

impl Report for StatusOnly<'_> {
    fn status(&mut self, text: &str) {
        self.0.status(text);
    }
    fn progress(&mut self, completed: f64) {
        self.0.progress(completed);
    }
    fn message(&mut self, text: &str) {
        self.0.message(text);
    }
    fn failed(&mut self, text: &str) {
        self.0.failed(text);
    }
    fn try_again(&mut self) -> bool {
        self.0.try_again()
    }
    fn choose_firmware(&mut self, filter: &str) -> Option<PathBuf> {
        self.0.choose_firmware(filter)
    }
}

/// `Disconnect`, when the page goes: the radio back in transparent mode if the port is open.
/// The port is then closed by dropping it (`FinishedWithComPortForSiKRadio`).
/// `// C#: Radio/Sikradio.cs:278-292`
pub fn disconnect<P: Port>(s: &mut Session<P>) {
    if s.wire().is_open() {
        // Disposing: a radio that does not answer is left as it is.
        let _ = s.put_into_transparent_mode();
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::radio::{Mode as RadioMode, Radio, Unit};
    use crate::session::tests::session;

    /// A report that keeps what it heard.
    #[derive(Debug, Default)]
    pub(crate) struct Heard {
        pub(crate) statuses: Vec<String>,
        pub(crate) progress: Vec<f64>,
        pub(crate) messages: Vec<String>,
        pub(crate) failures: Vec<String>,
        pub(crate) again: Vec<bool>,
        pub(crate) file: Option<PathBuf>,
        pub(crate) asked: Vec<String>,
    }

    impl Report for Heard {
        fn status(&mut self, text: &str) {
            self.statuses.push(text.to_owned());
        }
        fn progress(&mut self, completed: f64) {
            self.progress.push(completed);
        }
        fn message(&mut self, text: &str) {
            self.messages.push(text.to_owned());
        }
        fn failed(&mut self, text: &str) {
            self.failures.push(text.to_owned());
        }
        fn try_again(&mut self) -> bool {
            self.again.pop().unwrap_or(false)
        }
        fn choose_firmware(&mut self, filter: &str) -> Option<PathBuf> {
            self.asked.push(filter.to_owned());
            self.file.clone()
        }
    }

    /// Load Settings over a local and a remote RFD900+: every answer read, and the radio left in
    /// transparent mode.
    #[test]
    fn load_reads_both_radios() {
        let mut s = session(Radio::rfd900p());
        let mut heard = Heard::default();
        let loaded = load(&mut s, &mut heard);
        assert_eq!(loaded.error, None);
        assert!(loaded.entered);
        assert_eq!(loaded.ati.as_deref(), Some("RFD SiK 2.65 on RFD900P"));
        assert_eq!(loaded.multipoint_fix, -1);
        assert_eq!(loaded.freq, Some(Frequency::F915));
        assert_eq!(loaded.board, Some(Board::RFD900P));
        assert_eq!(loaded.country.as_deref(), Some("--"));
        assert_eq!(
            loaded.key,
            Some(KeyBox::Shown("00000000000000000000000000000000".to_owned()))
        );
        assert!(
            loaded
                .rssi
                .as_deref()
                .unwrap()
                .starts_with("L/R RSSI: 210/198")
        );
        let settings = loaded.settings.as_ref().unwrap();
        assert_eq!(settings.get("NETID").unwrap().value(), Some(25));
        assert_eq!(loaded.rti.as_deref(), Some("RFD SiK 2.65 on RFD900P\r\n"));
        let remote = loaded.remote.as_ref().unwrap();
        assert_eq!(remote.rti2.as_deref(), Some("DEVICE_ID_RFD900P"));
        assert!(
            remote
                .settings
                .get("AIR_SPEED")
                .unwrap()
                .options()
                .is_some()
        );
        assert!(heard.messages.is_empty());
        assert_eq!(heard.statuses[0], "Connecting");
        assert!(
            heard
                .statuses
                .contains(&"Doing Command ATI & RTI".to_owned())
        );
        assert!(heard.statuses.contains(&"Doing Command RTI5".to_owned()));
        assert_eq!(s.wire().port().mode(), RadioMode::Transparent, "off hook");
    }

    /// No remote radio: `RTI` unanswered is "", and nothing more is asked of it.
    #[test]
    fn load_without_a_remote_radio() {
        let mut s = session(Radio::with(Unit::rfd900p(), None));
        let loaded = load(&mut s, &mut Heard::default());
        assert_eq!(loaded.rti.as_deref(), Some(""));
        assert_eq!(loaded.remote, None);
        assert!(!s.wire().port().commands().iter().any(|c| c == "RTI5"));
    }

    /// A radio that will not enter command mode: "Fail", on the status line.
    #[test]
    fn load_fails_on_a_deaf_radio() {
        let mut radio = Radio::rfd900p();
        radio.serial_baud = 9600;
        let mut s = session(radio);
        let mut heard = Heard::default();
        let loaded = load(&mut s, &mut heard);
        assert!(!loaded.entered);
        assert_eq!(heard.failures, [LOAD_NO_COMMAND_MODE]);
        assert_eq!(heard.statuses.last().map(String::as_str), Some("Fail"));
    }

    /// A multipoint radio's [n] taken off the band and board; the page does not ask for its key
    /// (`GetSettings` still does, as it does of every radio).
    #[test]
    fn load_reads_past_a_multipoint_node() {
        let key_asked = |s: &mut Session<Radio>| {
            s.wire()
                .port()
                .commands()
                .iter()
                .filter(|c| *c == "AT&E?")
                .count()
        };
        let mut unit = Unit::rfd900p();
        unit.node = Some("[1] ".to_owned());
        let mut s = session(Radio::with(unit, None));
        let loaded = load(&mut s, &mut Heard::default());
        assert_eq!(loaded.error, None, "{loaded:?}");
        assert_eq!(loaded.multipoint_fix, 3);
        assert_eq!(loaded.freq, Some(Frequency::F915));
        assert_eq!(loaded.key, Some(KeyBox::Disabled));
        assert!(loaded.simple_mavlink);
        assert_eq!(key_asked(&mut s), 1, "GetSettings' only");
        let mut point = session(Radio::with(Unit::rfd900p(), None));
        load(&mut point, &mut Heard::default());
        assert_eq!(key_asked(&mut point), 2, "the page's and GetSettings'");
    }

    /// What the C#'s `catch` catches - here a port that is closed - is "Error", and its box on
    /// the status line; the band and board parse as `int.Parse` and `Enum.Parse` do.
    #[test]
    fn load_stops_at_an_exception() {
        let mut s = session(Radio::rfd900p());
        s.wire().close();
        let mut heard = Heard::default();
        let loaded = load(&mut s, &mut heard);
        assert_eq!(loaded.error.as_deref(), Some("The port is closed."));
        assert!(!loaded.entered);
        assert_eq!(heard.statuses.last().map(String::as_str), Some("Error"));
        assert_eq!(heard.failures, ["Error during read The port is closed."]);
        assert_eq!(parse_code("0x91").unwrap(), 0x91);
        assert_eq!(parse_code("145").unwrap(), 0x91);
        assert!(parse_code("SiK").is_err());
        assert!(parse_code("300").is_err(), "not a byte");
    }

    /// Save Settings: the remote changes, the local ones, the key, `&W` and `Z` to each, then
    /// `+++` to the rebooted radio.
    #[test]
    fn save_writes_each_radio_then_reboots_them() {
        let mut s = session(Radio::rfd900p());
        let plan = SavePlan {
            remote: Some(vec![Change {
                name: "NETID".to_owned(),
                designator: "S3".to_owned(),
                value: "40".to_owned(),
                is_one: false,
            }]),
            local: vec![
                Change {
                    name: "FORMAT".to_owned(),
                    designator: "S0".to_owned(),
                    value: "26".to_owned(),
                    is_one: false,
                },
                Change {
                    name: "NETID".to_owned(),
                    designator: "S3".to_owned(),
                    value: "40".to_owned(),
                    is_one: false,
                },
            ],
            remote_key: None,
            local_key: Some(Ok("0123456789ABCDEF0123456789ABCDEF".to_owned())),
        };
        let mut heard = Heard::default();
        assert!(save(&mut s, &plan, &mut heard).unwrap());
        assert!(heard.failures.is_empty(), "{:?}", heard.failures);
        let radio = s.wire().port();
        assert_eq!(radio.local.value("NETID"), Some(40));
        assert_eq!(
            radio.local.value("FORMAT"),
            Some(25),
            "the format is never set"
        );
        assert_eq!(radio.remote.as_ref().unwrap().value("NETID"), Some(40));
        assert_eq!(
            radio.local.key.as_deref(),
            Some("0123456789ABCDEF0123456789ABCDEF")
        );
        assert_eq!(radio.local.eeprom[3], 40, "written");
        assert_eq!(radio.local.reboots, 1);
        // `RTZ` answers nothing, and `doCommand` asks again when the answer is empty; the local
        // radio, rebooted by `ATZ`, does not echo the second one.
        assert_eq!(radio.remote.as_ref().unwrap().reboots, 2);
        let order: Vec<&str> = radio
            .commands()
            .iter()
            .map(String::as_str)
            .filter(|c| c.contains('=') || c.ends_with("&W") || c.ends_with('Z'))
            .collect();
        assert_eq!(
            order,
            [
                "RTS3=40",
                "ATS3=40",
                "AT&E=0123456789ABCDEF0123456789ABCDEF",
                "RT&W",
                "RTZ",
                "RTZ",
                "AT&W",
                "ATZ"
            ]
        );
        assert_eq!(heard.statuses.last().map(String::as_str), Some("Done"));
        assert_eq!(radio.mode(), RadioMode::Command, "+++ after the reboot");
    }

    /// A refused set, and a key too long: "Set Command error" on the status line, the key's box.
    #[test]
    fn save_says_what_failed() {
        let mut s = session(Radio::with(Unit::rfd900p(), None));
        let plan = SavePlan {
            remote: None,
            local: vec![Change {
                name: "NOT_THERE".to_owned(),
                designator: "S99".to_owned(),
                value: "1".to_owned(),
                is_one: false,
            }],
            remote_key: None,
            local_key: Some(Err(32)),
        };
        let mut heard = Heard::default();
        save(&mut s, &plan, &mut heard).unwrap();
        assert_eq!(heard.failures, [SET_COMMAND_ERROR]);
        assert_eq!(heard.messages, [key_invalid_text(32)]);
        assert_eq!(
            key_invalid_text(32),
            "Encryption key not valid hex number <= 32 hex numerals"
        );
    }

    /// An R/C output switched on is followed by `PO=1`.
    #[test]
    fn an_output_switched_on_takes_po() {
        let mut s = session(Radio::with(Unit::rfd900p(), None));
        let plan = SavePlan {
            local: vec![Change {
                name: "GPO1_1R_COUT".to_owned(),
                designator: "S18".to_owned(),
                value: "1".to_owned(),
                is_one: true,
            }],
            ..SavePlan::default()
        };
        save(&mut s, &plan, &mut Heard::default()).unwrap();
        let commands = s.wire().port().commands();
        let at = commands.iter().position(|c| c == "ATS18=1").unwrap();
        assert_eq!(commands[at + 1], "ATPO=1");
    }

    /// Reset to Defaults: each radio's factory settings written and the radios rebooted.
    #[test]
    fn reset_restores_both_radios() {
        let mut radio = Radio::rfd900p();
        radio.local.params[3].value = 99;
        radio.remote.as_mut().unwrap().params[3].value = 98;
        let mut s = session(radio);
        let mut heard = Heard::default();
        reset(&mut s, true, &mut heard).unwrap();
        let radio = s.wire().port();
        assert_eq!(radio.local.value("NETID"), Some(25));
        assert_eq!(radio.remote.as_ref().unwrap().value("NETID"), Some(25));
        assert!(heard.failures.is_empty());
        assert!(heard.statuses.contains(&"Reset".to_owned()));
    }

    /// Set PPM Fail Safe: `&R` and `&W` to the radio asked.
    #[test]
    fn ppm_fail_safe_is_recorded_and_saved() {
        let mut s = session(Radio::rfd900p());
        let mut heard = Heard::default();
        set_ppm_fail_safe(&mut s, true, &mut heard).unwrap();
        assert!(s.wire().port().remote.as_ref().unwrap().ppm_recorded);
        assert!(!s.wire().port().local.ppm_recorded);
        assert_eq!(heard.statuses.last().map(String::as_str), Some("Done"));
    }

    /// The encryption level written at once, and the key read back.
    #[test]
    fn the_encryption_level_is_written_at_once() {
        let mut s = session(Radio::rfd900p());
        let key = set_encryption_level(&mut s, false, 1, &mut Heard::default()).unwrap();
        assert_eq!(key.as_deref(), Some("00000000000000000000000000000000"));
        assert_eq!(s.wire().port().local.value("ENCRYPTION_LEVEL"), Some(1));
        assert_eq!(
            s.wire().port().mode(),
            RadioMode::Command,
            "left in command mode"
        );
    }

    /// Upload Firmware: the mode, the modem, the file asked for with the model's filter, the
    /// programming; cancelled, nothing is done.
    #[test]
    fn upload_asks_for_the_file_and_programs_it() {
        let dir = std::env::temp_dir().join(format!("mp-sikradio-page-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("rfd900p.hex");
        std::fs::write(&file, ":0700100052464439303050E1\n").unwrap();
        let mut s = session(Radio::rfd900p());
        let mut heard = Heard::default();
        assert_eq!(program_firmware(&mut s, &mut heard), Programmed::Cancelled);
        assert_eq!(heard.asked, ["Firmware|*.hex;*.ihx"]);
        assert_eq!(
            heard.statuses.last().map(String::as_str),
            Some("Firmware file selection cancelled")
        );
        assert_eq!(heard.statuses[1], "Mode is AT_COMMAND");

        let mut heard = Heard {
            file: Some(file),
            ..Heard::default()
        };
        assert_eq!(program_firmware(&mut s, &mut heard), Programmed::Ended);
        assert_eq!(
            heard.statuses.last().map(String::as_str),
            Some("Programmed firmware into device")
        );
        assert!(heard.progress.iter().any(|p| *p > 0.0));
        std::fs::remove_dir_all(&dir).ok();
    }

    /// An HM-TRP has no modem object: "Unknown modem", on the status line.
    #[test]
    fn upload_to_an_unknown_modem() {
        let mut s = session(Radio::with(Unit::hm_trp(), None));
        let mut heard = Heard::default();
        assert_eq!(program_firmware(&mut s, &mut heard), Programmed::Ended);
        assert_eq!(heard.failures, [NO_MODEM]);
        assert_eq!(
            heard.statuses.last().map(String::as_str),
            Some("Unknown modem")
        );
    }

    /// Disconnect leaves the radio transparent.
    #[test]
    fn disconnect_leaves_the_radio_transparent() {
        let mut radio = Radio::rfd900p();
        radio.enter_command_mode();
        let mut s = session(radio);
        s.get_mode().unwrap();
        disconnect(&mut s);
        assert_eq!(s.wire().port().mode(), RadioMode::Transparent);
    }
}
